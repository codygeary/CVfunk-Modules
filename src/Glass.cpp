////////////////////////////////////////////////////////////
//
//   Glass
//
//   written by Cody Geary
//   Copyright 2026, MIT License
//
//   Glass armonica physical model synthesizer.
//   37 chromatic bowls (C3-C6). Squeak-oscillator excitation drives
//   passive high-Q resonators. Bowls ring freely after gate release.
//
////////////////////////////////////////////////////////////

#include "plugin.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include "FilterGlass.h"

static constexpr int GLASS_BOWLS    = 37;
static constexpr int GLASS_MAX_POLY = 16;

static float BOWL_VOCT[GLASS_BOWLS];

static void initBowlVoct() {
    for (int i = 0; i < GLASS_BOWLS; ++i) {
        BOWL_VOCT[i] = (i - 12) * (1.f / 12.f);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// GlassBowlState
// ─────────────────────────────────────────────────────────────────────────────
struct GlassBowlState {
    GlassBowl         waveguide;
    GlassPressureEnv  pressureEnv;
    GlassDCBlocker    dcBlocker;

    float baseDelaySamples = 100.f;  // nominal delay for this bowl's pitch
    float delaySamples     = 100.f;  // actual delay after FM, updated each sample
    float envOut           = 0.f;
    float sinePhase        = 0.f;

    void init(float sr, float pitchHz, int bowlIndex = 0) {
        waveguide.init(sr, 130.81f);
        dcBlocker.setSampleRate(sr);
        pressureEnv.reset();
        baseDelaySamples = sr / pitchHz;
        delaySamples     = baseDelaySamples;
        envOut        = 0.f;
        // Stagger starting phases across bowls using the golden ratio.
        sinePhase     = fmodf((float)bowlIndex * 0.6180339f, 1.f);
    }

    void clear() {
        waveguide.clear();
        dcBlocker.reset();
        pressureEnv.reset();
        delaySamples  = baseDelaySamples;
        envOut        = 0.f;
        sinePhase     = 0.f;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Glass module
// ─────────────────────────────────────────────────────────────────────────────
struct Glass : Module {

    enum ParamId {
        SPEED_PARAM,  SPEED_ATT,
        WATER_PARAM,  WATER_ATT,
        DECAY_PARAM,  DECAY_ATT,
        DAMP_PARAM,   DAMP_ATT,
        WOBBLE_PARAM, WOBBLE_ATT,
        DAMP_BUTTON,
        SPREAD_PARAM,
        ROOT_PARAM,
        FM_PARAM, FM_ATT,
        VOLUME_PARAM, VOLUME_ATT,
        AUDIO_IN_GAIN_PARAM, AUDIO_IN_GAIN_ATT,
        PARAMS_LEN
    };

    enum InputId {
        VOCT_INPUT,
        GATE_INPUT,
        PRESSURE_INPUT,
        MUTE_GATE_INPUT,
        SPEED_CV_INPUT,
        WATER_CV_INPUT,
        DECAY_CV_INPUT,
        DAMP_CV_INPUT,
        WOBBLE_CV_INPUT,
        FM_CV_INPUT,
        VOLUME_CV_INPUT,
        DAMP_GATE_INPUT,
        AUDIO_IN_INPUT,
        AUDIO_IN_CV_INPUT,
        INPUTS_LEN
    };

    enum OutputId {
        AUDIO_L_OUTPUT,
        AUDIO_R_OUTPUT,
        ENV_OUTPUT,
        OUTPUTS_LEN
    };

    enum LightId {
        SPEED_LIGHT, WATER_LIGHT, DECAY_LIGHT, DAMP_LIGHT, WOBBLE_LIGHT,
        ENV_LIGHT_0,                          // 5 segments, matches Aulos BREATH_LIGHT_0
        LIGHTS_LEN = ENV_LIGHT_0 + 10
    };

    GlassBowlState   bowls[GLASS_BOWLS];
    float sampleRate = 48000.f;

    int skipCounter = 0;
    static constexpr int SKIP_MAX = 16;

    float attackValue   = 0.15f;
    float releaseValue  = 0.35f;
    float attackCurve   = 0.3f;
    float releaseCurve  = -0.3f;

    // Cached from sub-rate block
    // Sub-rate cached params -- updated every SKIP_MAX samples.
    float cachedSpeedHz       = 2.f;
    float cachedSpeedRaw      = 0.33f;
    float cachedWaterRaw      = 0.4f;
    float cachedWaterCurved   = 0.f;
    float cachedNoiseWeight   = 0.25f;
    float cachedSineWeight    = 0.f;
    float cachedExcitScale    = 0.05f;
    float cachedVolume        = 1.6f;
    float cachedFmRatio       = 1.f;
    float cachedAttackSamples = 1000.f;
    float cachedReleaseSamples= 10000.f;
    float noiseCutoffMax    = 1000.f; // frequency cutoff for dry finger noise excitation
    float tremoloPhase      = 0.f;    // global axis wobble phase, shared across all bowls
    float cachedFeedback    = 0.9997f;
    float cachedLpfCoeff    = 0.02f;     // feedback LPF coeff with current global damp state
    float cachedMutedLpfCoeff = 0.02f;   // feedback LPF coeff as if damp were engaged (per-note mute)
    float cachedNoiseLpfZ   = 0.f;    // one-pole LPF state for excitation noise
    float cachedNoiseLpfA   = 0.5f;   // LPF coefficient, updated in sub-rate block
    float cachedWobbleDepth    = 0.15f;  // axis wobble depth, cached sub-rate
    float cachedRootOffset     = 0.f;    // root transpose, cached sub-rate
    float cachedFmRatioInv     = 1.f;    // 1/cachedFmRatio -- multiply instead of divide per bowl
    float cachedPressureShare  = 1.f;    // hand pressure distributed across active finger count
    int   cachedBowlForChannel[GLASS_MAX_POLY] = {};  // cached nearestBowl per VOCT channel
    float cachedVoctForChannel[GLASS_MAX_POLY] = {};  // last V/oct read per channel
    bool  anyBowlActive = false;
    GlassADAADrive   mixSaturatorL;
    GlassADAADrive   mixSaturatorR;
    GlassEnvFollower envFollower;
    float vuEnv[10] = {};         // VU meter segments for ENV light strip
    float cachedAudioIn = 0.f;    // cached audio input sample

    // Cached connection state -- updated in sub-rate block.
    bool  cachedVoctConnected = false;
    bool  cachedGateConnected = false;
    bool  cachedPressConn     = false;
    bool  cachedMuteConn      = false;
    int   cachedNVoct  = 0;
    int   cachedNGate  = 0;
    int   cachedNPress = 0;
    int   cachedNMute  = 0;

    // Written by DSP, read by draw().
    float bowlEnergy[GLASS_BOWLS] = {};
    float bowlRawAbs[GLASS_BOWLS] = {};  // scratch for SIMD energy pass, module-level to avoid stack zeroing

    // Precomputed per-bowl pan gains -- recomputed when Spread changes.
    // Avoids per-sample division in the bowl loop.
    float panGainL[GLASS_BOWLS] = {};
    float panGainR[GLASS_BOWLS] = {};
    float lastSpread = -1.f;  // sentinel to force recompute on first process()

    void recomputePanGains(float spread) {
        for (int b = 0; b < GLASS_BOWLS; ++b) {
            float panNorm = (float)b / (float)(GLASS_BOWLS - 1);
            float panPos  = 0.5f + (panNorm - 0.5f) * spread;
            panGainL[b]   = 1.f - panPos;
            panGainR[b]   = panPos;
        }
        lastSpread = spread;
    }

    static float voct2freq(float voct) {
        return 261.63f * rack::dsp::exp2_taylor5(voct);
    }

    // Fold voct into the bowl range [-1, +2] (C3..C6) by shifting
    // one octave (1V) at a time. Notes outside the range are mapped
    // to their nearest in-range octave equivalent rather than clamped.
    static float foldVoct(float voct) {
        const float lo = -1.0f;   // C3
        const float hi =  2.0f;   // C6
        while (voct < lo)  voct += 1.f;
        while (voct > hi)  voct -= 1.f;
        return voct;
    }

    static int nearestBowl(float voct) {
        voct = foldVoct(voct);
        int best = 0;
        float bestDist = fabsf(voct - BOWL_VOCT[0]);
        for (int i = 1; i < GLASS_BOWLS; ++i) {
            float dist = fabsf(voct - BOWL_VOCT[i]);
            if (dist < bestDist) { bestDist = dist; best = i; }
        }
        return best;
    }

    float getCV(InputId cvId, ParamId attId, float paramVal, int ch = 0) {
        float cv = inputs[cvId].isConnected()
                 ? inputs[cvId].getPolyVoltage(ch) : 0.f;
        return paramVal + params[attId].getValue() * cv * 0.1f;
    }

    void initBowls(float sr) {
        for (int b = 0; b < GLASS_BOWLS; ++b) {
            bowls[b].init(sr, voct2freq(BOWL_VOCT[b]), b);
            bowls[b].baseDelaySamples = sr / voct2freq(BOWL_VOCT[b]);
            bowls[b].delaySamples     = bowls[b].baseDelaySamples;
        }
    }

    void panic() {
        for (int b = 0; b < GLASS_BOWLS; ++b) {
            bowls[b].clear();
            bowlRawAbs[b] = 0.f;
            bowlEnergy[b] = 0.f;
        }
        anyBowlActive = false;
    }

    Glass() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(SPEED_PARAM,  0.f,  6.f,  3.14f, "Rotation Speed", " Hz");
        configParam(SPEED_ATT,   -2.f,  2.f,  0.f,   "Speed Att.");
        configParam(WATER_PARAM,  0.f,  1.f,  0.85f, "Water");
        configParam(WATER_ATT,   -2.f,  2.f,  0.f,   "Water Att.");
        configParam(DECAY_PARAM,  0.f,  1.f,  0.5f,  "Decay");  
        configParam(DECAY_ATT,   -2.f,  2.f,  0.f,   "Decay Att.");
        configParam(DAMP_PARAM,   0.f,  1.f,  0.75f,  "Damper Strength");
        configParam(DAMP_ATT,    -2.f,  2.f,  0.f,   "Damper Att.");
        configParam(WOBBLE_PARAM, 0.f,  1.0f, 0.15f, "Axis Wobble Depth");
        configParam(WOBBLE_ATT,  -2.f,  2.f,  0.f,   "Wobble Att.");
        configParam(DAMP_BUTTON,  0.f,  1.f,  0.f,   "Damp (all bowls)");
        configParam(SPREAD_PARAM, -1.f, 1.f,  1.0f,  "Stereo Spread");
        configParam(ROOT_PARAM,  -2.f,  2.f,  0.f,   "Root (transpose)", " oct");
        configParam(FM_PARAM,    -1.f,  1.f,  0.f,   "FM");
        configParam(FM_ATT,      -2.f,  2.f,  0.f,   "FM Att.");
        configParam(VOLUME_PARAM, 0.f,  1.f,  0.5f,  "Volume");
        configParam(VOLUME_ATT,  -2.f,  2.f,  0.f,   "Volume Att.");

        configInput(VOCT_INPUT,      "V/Oct (polyphonic)");
        configInput(GATE_INPUT,      "Gate (polyphonic)");
        configInput(PRESSURE_INPUT,  "Pressure (polyphonic)");
        configInput(MUTE_GATE_INPUT, "Mute Bowl Gate (polyphonic)");
        configInput(SPEED_CV_INPUT,  "Speed CV");
        configInput(WATER_CV_INPUT,  "Water CV");
        configInput(DECAY_CV_INPUT,  "Decay CV");
        configInput(DAMP_GATE_INPUT, "Damper Gate (all bowls)");        
        configInput(DAMP_CV_INPUT,   "Damper Amount CV");
        configInput(WOBBLE_CV_INPUT, "Axis Wobble CV");
        configInput(FM_CV_INPUT,     "FM CV");
        configInput(VOLUME_CV_INPUT, "Volume CV");
        configParam(AUDIO_IN_GAIN_PARAM, 0.f, 1.f, 0.5f, "Audio In Gain");
        configParam(AUDIO_IN_GAIN_ATT,  -2.f, 2.f, 0.f, "Audio In Gain Att.");
        configInput(AUDIO_IN_INPUT,    "Audio In");
        configInput(AUDIO_IN_CV_INPUT, "Audio In Gain CV");
        configOutput(AUDIO_L_OUTPUT, "Audio L");
        configOutput(AUDIO_R_OUTPUT, "Audio R");
        configOutput(ENV_OUTPUT,     "Envelope (RMS)");
        initBowls(48000.f);
        envFollower.setCoeff(sqrtf(150.f * 5000.f) / 48000.f, 0.4f, 48000.f);
        initBowlVoct();
    }

    void onSampleRateChange() override {
        sampleRate = APP->engine->getSampleRate();
        initBowls(sampleRate);
        envFollower.setCoeff(sqrtf(150.f * 5000.f) / sampleRate, 0.4f, sampleRate);
    }

    void onReset() override {
        panic();
        mixSaturatorL.reset();
        mixSaturatorR.reset();
        envFollower.reset();
        attackValue  = 0.15f;
        releaseValue = 0.35f;
        attackCurve  = 0.3f;
        releaseCurve = -0.3f;
    }

    json_t* dataToJson() override {
        json_t* root = json_object();
        json_object_set_new(root, "attackValue",   json_real(attackValue));
        json_object_set_new(root, "releaseValue",  json_real(releaseValue));
        json_object_set_new(root, "attackCurve",   json_real(attackCurve));
        json_object_set_new(root, "releaseCurve",  json_real(releaseCurve));
        json_object_set_new(root, "noiseCutoffMax",     json_real(noiseCutoffMax));
        return root;
    }

    void dataFromJson(json_t* root) override {
        auto gr = [&](const char* k, float d) -> float {
            json_t* j = json_object_get(root, k);
            return j ? (float)json_number_value(j) : d;
        };
        attackValue  = gr("attackValue",  0.15f);
        releaseValue = gr("releaseValue", 0.35f);
        attackCurve  = gr("attackCurve",  0.3f);
        releaseCurve = gr("releaseCurve", -0.3f);
        noiseCutoffMax      = gr("noiseCutoffMax",     1000.f);
    }

    void process(const ProcessArgs& args) override {
        const float sr = args.sampleRate;

        // ── Sub-rate block ────────────────────────────────────────────────────
        if (++skipCounter >= SKIP_MAX) {
            skipCounter = 0;

            float decayRaw = clamp(
                getCV(DECAY_CV_INPUT, DECAY_ATT, params[DECAY_PARAM].getValue()),
                0.f, 1.f);
            // Global damp: button or CV gate triggers damp at level set by DAMP slider.
            float buttonHeld  = params[DAMP_BUTTON].getValue() > 0.5f ? 1.f : 0.f;
            float dampGateIn  = (inputs[DAMP_GATE_INPUT].isConnected() &&
                                 inputs[DAMP_GATE_INPUT].getVoltage() > 0.5f) ? 1.f : 0.f;
            float dampSlider  = rack::clamp(
                getCV(DAMP_CV_INPUT, DAMP_ATT, params[DAMP_PARAM].getValue()), 0.f, 1.f);
            // Button and gate input both trigger global damp at the slider level.
            float dampRaw     = rack::clamp((buttonHeld + dampGateIn) > 0.f ? dampSlider : 0.f, 0.f, 1.f);

            // Feedback gain: always < 1, sets passive ring-down time.
            cachedFeedback = 0.985f + powf(decayRaw, 0.33f) * 0.0148f; //adjust .0148 (.9998 decay time)

            // LPF on feedback path: models glass material absorption.
            // At damp=0 almost transparent (~20kHz), glass is very bright.
            // At damp=1 sweeps down to ~300Hz, killing high harmonics quickly.
            // cachedMutedLpfCoeff is the coefficient the global damper would
            // use when engaged -- per-note mute applies it to individual bowls.
            {
                float baseDamp = expf(-2.f * float(M_PI) * 20000.f / sr);
                float maxDamp  = expf(-2.f * float(M_PI) *   300.f / sr);
                cachedLpfCoeff      = baseDamp + dampRaw    * (maxDamp - baseDamp);
                cachedMutedLpfCoeff = baseDamp + dampSlider * (maxDamp - baseDamp);
            }

            // Noise excitation LPF coefficient, updated from waterRaw.
            // Water=0 (dry) -> cutoff ~8000Hz (bright, snappy bursts).
            // Water=1 (wet) -> cutoff ~400Hz  (dark, smooth sliding).
            {
                float waterForNoise = clamp(
                    getCV(WATER_CV_INPUT, WATER_ATT, params[WATER_PARAM].getValue()),
                    0.f, 1.f);
                float noiseCutoff = 200.f + (1.f - waterForNoise) * (noiseCutoffMax - 200.f);
                cachedNoiseLpfA = expf(-2.f * float(M_PI) * noiseCutoff / sr);
            }
            
            // Cache all slow-changing params: speed, water, volume, FM, spread,
            // envelope timing. None need sample-rate precision.
            cachedSpeedHz  = clamp(
                getCV(SPEED_CV_INPUT, SPEED_ATT, params[SPEED_PARAM].getValue()),
                0.f, 6.f);
            cachedSpeedRaw = cachedSpeedHz / 6.f;

            cachedWaterRaw    = clamp(
                getCV(WATER_CV_INPUT, WATER_ATT, params[WATER_PARAM].getValue()),
                0.f, 1.f);
            cachedWaterCurved = cachedWaterRaw * cachedWaterRaw * cachedWaterRaw;
            cachedWaterRaw = sqrt(cachedWaterRaw);

            const float noiseMaxWt = 1.0f;
            const float sineScaleV = 0.08f;
            cachedNoiseWeight = noiseMaxWt * (1.f - cachedWaterCurved);
            cachedSineWeight  = cachedWaterCurved * sineScaleV;
            cachedExcitScale  = cachedSpeedRaw * 0.15f;

            // Hand pressure distributed across active finger count.
            // Total excitation budget is fixed; more notes share it.
            {
                int nActive = std::max(1, cachedNVoct);
                cachedPressureShare = powf((float)nActive, -0.5f);
            }

            cachedVolume = clamp(
                params[VOLUME_PARAM].getValue()
                + params[VOLUME_ATT].getValue()
                * (inputs[VOLUME_CV_INPUT].isConnected()
                   ? inputs[VOLUME_CV_INPUT].getVoltage() : 0.f) * 0.1f,
                0.f, 1.f) * 4.f;  // 0..4x gain, pushes into ADAA saturation at high levels

            // Audio in gain: cached sub-rate, matching Aulos usage.
            float audioInGain = rack::clamp(
                params[AUDIO_IN_GAIN_PARAM].getValue()
                + params[AUDIO_IN_GAIN_ATT].getValue()
                * (inputs[AUDIO_IN_CV_INPUT].isConnected()
                   ? inputs[AUDIO_IN_CV_INPUT].getVoltage() : 0.f) * 0.1f,
                0.f, 1.f);
            cachedAudioIn = inputs[AUDIO_IN_INPUT].isConnected()
                          ? inputs[AUDIO_IN_INPUT].getVoltage() * 0.01f * audioInGain : 0.f;

            cachedFmRatio = rack::dsp::exp2_taylor5(
                clamp(getCV(FM_CV_INPUT, FM_ATT, params[FM_PARAM].getValue()) * 0.167f, -0.5f, 0.5f));
            cachedFmRatioInv = 1.f / cachedFmRatio;

            // Wobble depth and root transpose: slow-changing, read here so the
            // bowl loop never touches params or inputs at audio rate.
            cachedWobbleDepth = clamp(
                getCV(WOBBLE_CV_INPUT, WOBBLE_ATT, params[WOBBLE_PARAM].getValue()), 0.f, 1.0f);
            cachedRootOffset = params[ROOT_PARAM].getValue();

            cachedAttackSamples  = sr * 0.002f * powf(2000.f, attackValue);
            cachedReleaseSamples = sr * 0.002f * powf(2000.f, releaseValue);

            float spread = params[SPREAD_PARAM].getValue();
            if (spread != lastSpread) recomputePanGains(spread);

            // Cache connection state -- changes only when cables are patched.
            cachedVoctConnected  = inputs[VOCT_INPUT].isConnected();
            cachedGateConnected  = inputs[GATE_INPUT].isConnected();
            cachedPressConn      = inputs[PRESSURE_INPUT].isConnected();
            cachedMuteConn       = inputs[MUTE_GATE_INPUT].isConnected();
            cachedNVoct  = cachedVoctConnected  ? inputs[VOCT_INPUT].getChannels()      : 0;
            cachedNGate  = cachedGateConnected  ? inputs[GATE_INPUT].getChannels()      : 0;
            cachedNPress = cachedPressConn      ? inputs[PRESSURE_INPUT].getChannels()  : 0;
            cachedNMute  = cachedMuteConn       ? inputs[MUTE_GATE_INPUT].getChannels() : 0;
        }

        // ── Gate accumulation (sub-rate cached) ───────────────────────────────
        // nearestBowl() is a 37-iteration search. Cache the result per channel
        // and only recompute when the V/oct voltage changes.
        // Gate arrays are stack-allocated and zeroed here cheaply.
        bool  gateHigh[GLASS_BOWLS]     = {};
        float sustainLevel[GLASS_BOWLS] = {};
        bool  dampHigh[GLASS_BOWLS]     = {};
        bool  anyGateEvent = false;

        float rootOffset = cachedRootOffset;

        // Use cached connection state (updated sub-rate).
        int  nVoctCh  = cachedNVoct;
        int  nGateCh  = cachedNGate;
        int  nPressCh = cachedNPress;
        int  nMuteCh  = cachedNMute;
        bool gateConnected     = cachedGateConnected;
        bool pressureConnected = cachedPressConn;

        if (nVoctCh > 0) {
            int nCh = std::min(nVoctCh, GLASS_MAX_POLY);
            for (int ch = 0; ch < nCh; ++ch) {
                float voct = inputs[VOCT_INPUT].getPolyVoltage(ch) + rootOffset;

                // Only recompute nearestBowl when V/oct changes -- saves 37
                // float comparisons per channel per sample when notes are held.
                 if (fabsf(voct - cachedVoctForChannel[ch]) > 1e-5f) {
                     cachedVoctForChannel[ch] = voct;
                     cachedBowlForChannel[ch] = nearestBowl(voct);
                 }

                int bowl = cachedBowlForChannel[ch];

                float level = 1.f;
                if (pressureConnected) {
                    float pressV = (ch < nPressCh)
                                 ? inputs[PRESSURE_INPUT].getPolyVoltage(ch) : 0.f;
                    level = clamp(pressV / 10.f, 0.f, 1.f);
                }
                if (level > sustainLevel[bowl]) sustainLevel[bowl] = level;

                if (gateConnected) {
                    float gateV = (ch < nGateCh)
                                ? inputs[GATE_INPUT].getPolyVoltage(ch) : 0.f;
                    if (gateV > 0.05f) { gateHigh[bowl] = true; anyGateEvent = true; }
                } else {
                    // No gate patched: VOCT acts as a permanent gate.
                    // All active VOCT channels hold their bowls continuously.
                    // Use the damper to stop notes.
                    gateHigh[bowl] = true; anyGateEvent = true;
                }
            }
        }

        // ── Per-note mute ─────────────────────────────────────────────────────
        // Strict 1:1 channel mapping: mute channel ch mutes the bowl currently
        // assigned to V/oct channel ch. Extra mute channels beyond the V/oct
        // count are ignored. Muting works like the global damper but per bowl:
        // it darkens the feedback LPF (cachedMutedLpfCoeff) in the bowl loop.
        // Note: mute does not count as a gate event -- a muted bowl with no
        // ring energy stays dormant, and a ringing one is already processed
        // via anyBowlActive.
        if (cachedMuteConn && nVoctCh > 0) {
            int nCh = std::min(std::min(nMuteCh, nVoctCh), GLASS_MAX_POLY);
            for (int ch = 0; ch < nCh; ++ch) {
                if (inputs[MUTE_GATE_INPUT].getPolyVoltage(ch) > 0.05f)
                    dampHigh[cachedBowlForChannel[ch]] = true;
            }
        }

        // ── Full dormancy: skip almost everything when module is silent ────────
        // If no bowl has energy and no gate events arrived, we have nothing to do.
        // Noise, tremolo, bowl loop, SIMD pass, and output writes all skipped.
        // This costs only the sub-rate block (1/16 samples) and the gate checks above.
        if (!anyGateEvent && !anyBowlActive) {
            outputs[AUDIO_L_OUTPUT].setVoltage(0.f);
            outputs[AUDIO_R_OUTPUT].setVoltage(0.f);
            return;
        }

        // ── Audio computation (only when something is active) ─────────────────
        float filteredNoise = 0.f;
        if (cachedNoiseWeight > 0.001f) {
            float rawNoise = rack::random::normal();
            cachedNoiseLpfZ = (1.f - cachedNoiseLpfA) * rawNoise
                            + cachedNoiseLpfA * cachedNoiseLpfZ;
            filteredNoise = cachedNoiseLpfZ;
        }

        // Dry contact excitation: n*|n| shaping creates a heavy-tailed sparse
        // signal -- occasional sharp stick-slip peaks with near-silence between.
        // This sounds like a dry finger on glass rather than a continuously bowed
        // metal string. Computed once here since all bowls share the same noise.
        // At wet settings cachedNoiseWeight->0 so sparseNoise has no effect there.
        // Tune 0.6f: lower = more impulsive but quieter, higher = louder peaks.
        float sparseNoise = filteredNoise * fabsf(filteredNoise) * 0.6f;

        tremoloPhase += cachedSpeedHz / sr;
        if (tremoloPhase >= 1.f) tremoloPhase -= 1.f;
        float globalTremoloSine = (cachedSpeedHz > 0.01f)
            ? glassDspSin(tremoloPhase * 2.f * float(M_PI)) : 0.f;

        float fmRatioInv = cachedFmRatioInv;

        // ── Bowl DSP loop ─────────────────────────────────────────────────────
        // Ext input feeds only bowls whose gate is currently held (envOut > 0).
        // When gate releases, ext stops feeding that bowl and it decays freely.
        // This prevents latching and allows natural decay.

        const float idleThreshold = 5e-4f;

        float mixL = 0.f, mixR = 0.f;

        for (int b = 0; b < GLASS_BOWLS; ++b) {
            GlassBowlState& state = bowls[b];

            // Dormancy check: skip entirely when no gate and no ringing energy.
            // Mute (dampHigh) intentionally does not wake a bowl -- damping a
            // silent bowl is a no-op, and a ringing one is already processing.
            bool bowlDormant = !gateHigh[b] && (bowlEnergy[b] < idleThreshold);
            if (bowlDormant) {
                bowlRawAbs[b] = 0.f;
                continue;
            }

            state.envOut = state.pressureEnv.process(
                gateHigh[b], sustainLevel[b],
                cachedAttackSamples, cachedReleaseSamples,
                attackCurve, releaseCurve);

            bool bowlAudible = (state.envOut > 0.0001f) || (bowlEnergy[b] > idleThreshold);
            if (!bowlAudible) continue;

            // FM: apply FM ratio to base delay length each sample.
            state.delaySamples = state.baseDelaySamples * fmRatioInv;

            // ── Friction excitation (gate-driven) ────────────────────────────
            float excitation = 0.f;

            if (state.envOut > 0.0001f) {
                state.sinePhase += 1.f / state.delaySamples;
                if (state.sinePhase >= 1.f) state.sinePhase -= 1.f;
                float sineVal = glassDspSin(state.sinePhase * 2.f * float(M_PI));

                // Wobble sine precomputed once -- used for both pressure modulation
                // and water-film thinning below.
                float wobbleSine     = cachedWobbleDepth * globalTremoloSine;
                float pressureModulation = 1.f + wobbleSine;
                float modulatedEnv   = state.envOut * pressureModulation;

                // Wobble periodically thins the water film: at peak off-axis displacement
                // the finger lifts slightly, shifting excitation toward noise.
                // Effect is proportional to cachedSineWeight so it only manifests when
                // water is high -- at full dry cachedSineWeight~=0 and this is silent.
                // Tune 0.05f: fraction of sine weight that shifts to noise at peak wobble.
                // Reduced 0.15->0.05: water attenuation was too audible at moderate wobble.
                float waterFilmShift     = wobbleSine * 0.05f;
                float effectiveSineWeight  = cachedSineWeight  * (1.f - waterFilmShift);
                float effectiveNoiseWeight = cachedNoiseWeight + cachedSineWeight * waterFilmShift;

                excitation = (sparseNoise * effectiveNoiseWeight + sineVal * effectiveSineWeight)
                           * cachedExcitScale * modulatedEnv;

                // ── Tremolo instability ───────────────────────────────────────
                // Uses the same global axis sine -- all bowls wobble together
                // since they share one axis. Depth scales with bowl energy and
                // inverse water (dry = more wobble).
                float tremoloDepth = (1.1f - cachedWaterRaw) * 0.3f
                    * clamp(bowlEnergy[b] * 2.f, 0.f, 1.f);
                excitation *= (1.f + tremoloDepth * globalTremoloSine);

                // Hand pressure shared across active fingers -- solo notes get
                // full pressure, chords distribute it. Applied after all other
                // excitation shaping so it scales the final injected energy.
                excitation *= cachedPressureShare;
            }

            // ── Waveguide ─────────────────────────────────────────────────────
            // Ext input is summed only while gate is held (envOut > 0).
            // When gate releases, ext stops and the bowl decays freely.
            // Per-note mute: identical mechanism to the global damper, applied
            // per bowl. While the bowl's mute gate is high, the feedback LPF
            // sweeps to the damped coefficient (level set by the DAMP slider)
            // and the bowl rings down through the darkening filter. Switching
            // a one-pole coefficient causes no discontinuity, so no smoothing
            // is needed -- same as the sub-rate global damp switch.
            float extContrib = (state.envOut > 0.0001f) ? cachedAudioIn : 0.f;
            float lpfCoeff   = dampHigh[b] ? cachedMutedLpfCoeff : cachedLpfCoeff;

            float bowlRaw = state.waveguide.process(
                excitation + extContrib,
                state.delaySamples,
                lpfCoeff,
                cachedFeedback);

            bowlRaw = state.dcBlocker.process(bowlRaw);
            if (!std::isfinite(bowlRaw)) { bowlRaw = 0.f; state.waveguide.clear(); }

            mixL += bowlRaw * panGainL[b];
            mixR += bowlRaw * panGainR[b];

            bowlRawAbs[b] = fabsf(bowlRaw);
        }

        // ── Energy envelope update + dormancy check ──────────────────────────
        // Single pass over all bowls. bowlRawAbs[b] = 0 for skipped bowls
        // so they always take the slow release path and decay to dormant.
        {
            anyBowlActive = false;
            const float vAttack  = 0.3f;
            const float vRelease = 0.001f;
            for (int b = 0; b < GLASS_BOWLS; ++b) {
                float energy = bowlRawAbs[b];
                float& e     = bowlEnergy[b];
                float coeff  = (energy > e) ? vAttack : vRelease;
                e += coeff * (energy - e);
                if (e > idleThreshold) anyBowlActive = true;
            }
        }

        // ── Outputs ───────────────────────────────────────────────────────────
        // Tune mixScale if output clips at full bowl activity.
        // Signal path matches Node.cpp exactly:
        //   mixL * baseScale  -> fixed scale to normalise bowl mix to audio range
        //   * cachedVolume    -> user gain, pushes into ADAA saturation at high levels
        //   process(inV)      -> clamp +-13.14, /10, ADAA tanh, *6.9
        //   clamp +-10V       -> output protection
        //
        // baseScale doubled 0.2->0.4: solo notes are now 2x louder.
        // Pressure sharing (cachedPressureShare) compensates at high note counts
        // so large chords do not overdrive the saturator.
        // const float baseScale = 0.2f;
        const float baseScale = 0.4f;
        float inL = mixL * baseScale * cachedVolume;
        float inR = mixR * baseScale * cachedVolume;
        float satL = mixSaturatorL.process(inL)*1.9f;
        float satR = mixSaturatorR.process(inR)*1.9f;
        outputs[AUDIO_L_OUTPUT].setVoltage(clamp(satL, -12.f, 12.f));
        outputs[AUDIO_R_OUTPUT].setVoltage(clamp(satR, -12.f, 12.f));

        // ── ENV output and VU lights ──────────────────────────────────────────
        // RMS follower on the mixed output, matching Aulos ENV/RMS pattern.
        float rmsIn = (fabsf(satL) + fabsf(satR)) * 0.5f;
        float envOut = envFollower.process(rmsIn);
        if (outputs[ENV_OUTPUT].isConnected())
            outputs[ENV_OUTPUT].setVoltage(rack::clamp(envOut * 13.f, 0.f, 10.f));

        // VU bar: 10 segments, same bar-graph encoding as Aulos.
        float displayLevel = envOut;
        for (int seg = 0; seg < 10; ++seg)
            vuEnv[seg] = (displayLevel * 13.f > (float)seg) ? 1.f : 0.f;

    }
};

// ─────────────────────────────────────────────────────────────────────────────
// BowlDisplay
// ─────────────────────────────────────────────────────────────────────────────
struct BowlDisplay : Widget {
    Glass* module = nullptr;
    float  animPhase = 0.f;   // widget-side rotation phase, always advancing
    double lastTime  = 0.0;   // for wall-clock delta

    void step() override {
        Widget::step();
        // Advance animPhase each UI frame using glfwGetTime() delta.
        // This runs regardless of DSP dormancy state.
        float speedHz = (module && module->cachedSpeedHz > 0.f)
                      ? module->cachedSpeedHz : 0.f;
        double now = glfwGetTime();
        if (lastTime > 0.0) {
            float dt = (float)(now - lastTime);
            // Clamp dt to avoid a large jump after module load or tab switch.
            dt = clamp(dt, 0.f, 0.1f);
            animPhase += speedHz * dt;
            if (animPhase >= 1.f) animPhase -= 1.f;
        }
        lastTime = now;
    }

    void draw(const DrawArgs& args) override {
        const float W  = box.size.x;
        const float H  = box.size.y;

        // Largest bowl (C3, b=0): radius = H * 0.42 (full height).
        // Smallest bowl (C6, b=36): radius = half the largest.
        // Radius scales linearly across the 37 bowls.
        // Tune rMax and rMin to adjust the size range.
        const float rMax = H * 0.35f;
        const float rMin = rMax * 0.5f;

        // Left padding = rMax + extra margin so C3 circle has breathing room.
        // Right padding = rMax (C6 is small so it fits with less clearance).
        // Tune leftPad margin (currently rMax * 0.2) if left bowl still clips.
        const float leftPad  = rMax + rMax * 0.2f;
        const float usableW  = W - leftPad - rMax;
        const float step     = usableW / (float)(GLASS_BOWLS - 1);
        const float cy       = H * 0.5f;

        nvgBeginPath(args.vg);
        nvgRoundedRect(args.vg, 0, 0, W, H, 3.f);
        nvgFillColor(args.vg, nvgRGB(12, 12, 14));
        nvgFill(args.vg);

        for (int b = 0; b < GLASS_BOWLS; ++b) {
            float cx = leftPad + b * step;

            // Radius decreases linearly from C3 (large) to C6 (small).
            float t  = (float)b / (float)(GLASS_BOWLS - 1);  // 0=C3, 1=C6
            float r  = rMax + t * (rMin - rMax);

            float energy = module
                         // Multiplier recalibrated 3->6: baseScale doubled (0.2->0.4) so
                         // audio is 2x louder but bowlEnergy (pre-baseScale) is unchanged.
                         // 6x restores the display/audio correspondence.
                         ? clamp(module->bowlEnergy[b] * 6.f, 0.f, 1.f)
                         : 0.f;

            // Black keys: C#, D#, F#, G#, A# within each octave.
            // These get a gold outline to distinguish them visually.
            int semitone = b % 12;
            bool isBlack = (semitone == 1 || semitone == 3 ||
                            semitone == 6 || semitone == 8 || semitone == 10);

            // Base fill.
            nvgBeginPath(args.vg);
            nvgCircle(args.vg, cx, cy, r);
            nvgFillColor(args.vg, nvgRGBAf(0.10f, 0.11f, 0.18f, 1.f));
            nvgFill(args.vg);

            // Rotating linear gradient sheen -- simulates light catching the
            // curved glass surface as the bowl spins.
            // The gradient axis rotates at the same rate as the gold dashes,
            // with the same per-bowl golden ratio offset, so the sheen and
            // the dashes move together coherently.
            // Tune sheenAlpha: lower = more subtle, higher = more prominent.
            // Tune the light/dark colors to match your panel aesthetic.
            {
                float bowlOffset  = (float)b * 0.6180339f * 2.f * float(M_PI);
                float sheenAngle  = animPhase * 2.f * float(M_PI) + bowlOffset;
                float dx = cosf(sheenAngle) * r;
                float dy = sinf(sheenAngle) * r;

                // Gradient from light side to dark side across the diameter.
                // Light: slightly brightened base. Dark: slightly darkened base.
                // Keep both close to the base color for a subtle glass sheen.
                const float sheenAlpha = 0.25f;  // Tune: 0.1=very subtle, 0.4=prominent
                NVGpaint grad = nvgLinearGradient(args.vg,
                    cx - dx, cy - dy,   // light end
                    cx + dx, cy + dy,   // dark end
                    nvgRGBAf(0.35f, 0.38f, 0.55f, sheenAlpha),   // light tint
                    nvgRGBAf(0.04f, 0.04f, 0.08f, sheenAlpha));  // dark tint

                nvgBeginPath(args.vg);
                nvgCircle(args.vg, cx, cy, r);
                nvgFillPaint(args.vg, grad);
                nvgFill(args.vg);
            }

            // Gold dashed ring on black keys, rotating at the axis spin rate.
            // Each bowl gets a fixed angular offset based on its index using the
            // golden ratio, so no two bowls are ever in phase with each other.
            // This breaks the moire pattern that occurs when all rings align.
            if (isBlack) {
                const int   nDashes      = 1; //reduced for visual simplicity
                const float dashFraction = 0.95f;
                const float sectorAngle  = 2.f * float(M_PI) / (float)nDashes;
                const float dashAngle    = sectorAngle * dashFraction;

                // Base rotation from widget-side animPhase -- always advances
                // at frame rate regardless of DSP dormancy.
                float baseRot = animPhase * 2.f * float(M_PI);

                // Per-bowl fixed offset using golden ratio -- same technique as
                // the sine phase stagger. Gives maximum spread across bowls.
                float bowlOffset = (float)b * 0.6180339f * 2.f * float(M_PI);
                float rotOffset  = baseRot + bowlOffset;

                nvgStrokeColor(args.vg, nvgRGBAf(0.82f, 0.65f, 0.15f, 0.7f));
                nvgStrokeWidth(args.vg, 1.0f);

                for (int d = 0; d < nDashes; ++d) {
                    float startAngle = rotOffset + (float)d * sectorAngle;
                    float endAngle   = startAngle + dashAngle;
                    nvgBeginPath(args.vg);
                    nvgArc(args.vg, cx, cy, r - 0.5f, startAngle, endAngle, NVG_CW);
                    nvgStroke(args.vg);
                }
            }

            // Active glow: color interpolates hot (active) -> cold (decaying).
            if (energy > 0.005f) {
                // Tune hotR/G/B and coldR/G/B for color palette.
                // Current: deep blue active -> warm orange decaying.
                const float hotR = 0.15f, hotG = 0.55f, hotB = 1.00f;
                const float coldR= 0.90f, coldG= 0.40f, coldB= 0.10f;
                float hot  = energy;
                float cold = 1.f - energy;
                float rv = hot * hotR + cold * coldR;
                float gv = hot * hotG + cold * coldG;
                float bv = hot * hotB + cold * coldB;

                nvgBeginPath(args.vg);
                nvgCircle(args.vg, cx, cy, r);
                nvgFillColor(args.vg, nvgRGBAf(rv, gv, bv, energy * 0.95f));
                nvgFill(args.vg);
            }
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// GlassSlider -- same SVG assets as AulosSlider
// ─────────────────────────────────────────────────────────────────────────────
struct GlassSliderBase : app::SvgSlider {
    GlassSliderBase() {
        setBackgroundSvg(Svg::load(asset::plugin(pluginInstance,
            "res/components/ShortSlider.svg")));
        setHandleSvg(Svg::load(asset::plugin(pluginInstance,
            "res/components/ShortSliderHandle.svg")));
        setHandlePosCentered(math::Vec(10.f, 55.f), math::Vec(10.f, 10.f));
    }
};
template <typename TL = BlueLight>
struct AulosSlider : LightSlider<GlassSliderBase, VCVSliderLight<TL>> {
    AulosSlider() {}
};

// ─────────────────────────────────────────────────────────────────────────────
// GlassWidget -- 20HP
// ─────────────────────────────────────────────────────────────────────────────
struct GlassWidget : ModuleWidget {

    static Vec p(float x, float y) { return mm2px(Vec(x, y)); }

    GlassWidget(Glass* module) {
        setModule(module);
        box.size = Vec(20 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT);

        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Glass.svg"),
            asset::plugin(pluginInstance, "res/Glass-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2*RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2*RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // ── Bowl display strip ────────────────────────────────────────────────
        {
            const float panelW = 20.f * 5.08f;
            auto* disp     = createWidget<BowlDisplay>(mm2px(Vec(7.f, 10.f)));
            disp->box.size = mm2px(Vec(panelW - 14.f, 21.f));
            disp->module   = module;
            addChild(disp);
        }

        // ── Layout constants ──────────────────────────────────────────────────
        // Left column: play inputs + pitch controls (x = 7..37mm)
        // Right: 3 slider groups starting at x=57mm, pitch 12mm
        // Bottom: root/FM/damp/spread/volume/outputs

        const float panelW  = 20.f * 5.08f;  // 101.6mm

        // Slider bank: Speed, Water, Decay, Damp, Wobble
        // 5 sliders on the right half, pitch 10mm, starting at x=50mm.
        // Each column: slider top, trimmer below, CV jack below that.
        const float sxBase  = 40.f;
        const float sPitch  = 12.f;
        const float ySlider = 46.f;
        const float ySlTrim = 57.f;
        const float ySlCV   = 65.f;

        struct SlSpec { Glass::ParamId param, att; Glass::InputId cv; int color; };
        const SlSpec sliders[5] = {
            { Glass::SPEED_PARAM,  Glass::SPEED_ATT,  Glass::SPEED_CV_INPUT,  1 },  // blue
            { Glass::WATER_PARAM,  Glass::WATER_ATT,  Glass::WATER_CV_INPUT,  1 },  // blue
            { Glass::DECAY_PARAM,  Glass::DECAY_ATT,  Glass::DECAY_CV_INPUT,  2 },  // white
            { Glass::DAMP_PARAM,   Glass::DAMP_ATT,   Glass::DAMP_CV_INPUT,   4 },  // red
            { Glass::WOBBLE_PARAM, Glass::WOBBLE_ATT, Glass::WOBBLE_CV_INPUT, 3 },  // yellow
        };
        for (int i = 0; i < 5; ++i) {
            float x = sxBase + i * sPitch;
            if (sliders[i].color == 1)
                addParam(createParamCentered<AulosSlider<BlueLight>>(   p(x, ySlider), module, sliders[i].param));
            else if (sliders[i].color == 2)
                addParam(createParamCentered<AulosSlider<WhiteLight>>(  p(x, ySlider), module, sliders[i].param));
            else if (sliders[i].color == 3)
                addParam(createParamCentered<AulosSlider<YellowLight>>( p(x, ySlider), module, sliders[i].param));
            else
                addParam(createParamCentered<AulosSlider<RedLight>>(    p(x, ySlider), module, sliders[i].param));
            addParam(createParamCentered<Trimpot>(
                p(x, ySlTrim), module, sliders[i].att));
            addInput(createInputCentered<ThemedPJ301MPort>(
                p(x, ySlCV), module, sliders[i].cv));
        }

        // ── Left column: play inputs ──────────────────────────────────────────
        // x positions for 4 jacks in left half
        const float xL1 = 12.f;  // left input column x position

        // Row at ySlider height: V/OCT, GATE, PRESSURE, DAMP gate
        addInput(createInputCentered<ThemedPJ301MPort>(p(xL1, 40.f), module, Glass::VOCT_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(p(xL1, 50.f), module, Glass::GATE_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(p(xL1, 60.f), module, Glass::PRESSURE_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(p(xL1, 75.f), module, Glass::MUTE_GATE_INPUT));

        addInput(createInputCentered<ThemedPJ301MPort>(    p(xL1, 85.f), module, Glass::DAMP_GATE_INPUT));        
        addParam(createParamCentered<TL1105>(              p(xL1+10.f, 85.f), module, Glass::DAMP_BUTTON));

        // ── Bottom rows ───────────────────────────────────────────────────────
        const float yRow2 = 104.f;
        const float yRow3 = 116.f;
        const float yRow4 = 110.f;
        const float cx = panelW * 0.5f;

        // Row 0: Root knob | FM (CV + trim + knob) | Damp button (momentary)
        addParam(createParamCentered<RoundLargeBlackKnob>( p(  44.5f, 85.f), module, Glass::ROOT_PARAM));        
        addInput(createInputCentered<ThemedPJ301MPort>(    p( cx+8.f+5.f, 85.f), module, Glass::FM_CV_INPUT));
        addParam(createParamCentered<Trimpot>(             p(cx+17.f+5.f, 85.f), module, Glass::FM_ATT));
        addParam(createParamCentered<RoundBlackKnob>(      p(cx+26.f+5.f, 85.f), module, Glass::FM_PARAM));

        // Row 1: Spread knob | Volume (CV + trim + knob)
        addParam(createParamCentered<RoundSmallBlackKnob>(      p(44.5f, yRow3), module, Glass::SPREAD_PARAM));

        addInput(createInputCentered<ThemedPJ301MPort>(    p(panelW*0.5f + 8.f, yRow3), module, Glass::VOLUME_CV_INPUT));
        addParam(createParamCentered<Trimpot>(             p(panelW*0.5f +17.f, yRow3), module, Glass::VOLUME_ATT));
        addParam(createParamCentered<RoundBlackKnob>( p(panelW*0.5f +26.f, yRow3), module, Glass::VOLUME_PARAM));


        // Audio In: jack at sxBase+3*sPitch (mirrors Aulos row 2),
        // gain CV/trim/knob aligned with Volume controls (cx+8/17/26).
        addInput(createInputCentered<ThemedPJ301MPort>(    p(  44.5f, yRow2), module, Glass::AUDIO_IN_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(    p( cx+8.f, yRow2), module, Glass::AUDIO_IN_CV_INPUT));
        addParam(createParamCentered<Trimpot>(             p(cx+17.f, yRow2), module, Glass::AUDIO_IN_GAIN_ATT));
        addParam(createParamCentered<RoundSmallBlackKnob>( p(cx+26.f, yRow2), module, Glass::AUDIO_IN_GAIN_PARAM));

        
        // Row 3: ENV lights at BREATH_LIGHT position (cx-36..cx-44), ENV output
        // at 10+1*11.5 = 21.5mm -- exactly matching Aulos ENV_OUTPUT position.
        for (int i = 0; i < 5; ++i) {
            addChild(createLightCentered<SmallLight<BlueLight>>(
                p(cx - 36.f - 2.f * i, yRow4 + 1.f), module, Glass::ENV_LIGHT_0 + i * 2));
            addChild(createLightCentered<SmallLight<BlueLight>>(
                p(cx - 37.f - 2.f * i, yRow4 - 1.f), module, Glass::ENV_LIGHT_0 + i * 2 + 1));
        }
        addOutput(createOutputCentered<ThemedPJ301MPort>(  p(10.f + 1*11.5f, yRow4), module, Glass::ENV_OUTPUT));

        // L/R outputs at Aulos-matching bottom-right
        addOutput(createOutputCentered<ThemedPJ301MPort>(  p(cx+40.f, yRow2), module, Glass::AUDIO_L_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(  p(cx+40.f, yRow3), module, Glass::AUDIO_R_OUTPUT));
    }

    void step() override {
        ModuleWidget::step();
        Glass* m = dynamic_cast<Glass*>(module);
        if (!m) return;
        float dt     = APP->window->getLastFrameDuration();
        float lambda = 30.f;  // smoothing rate, matches Aulos
        for (int seg = 0; seg < 10; ++seg) {
            m->lights[Glass::ENV_LIGHT_0 + seg].setBrightnessSmooth(
                m->vuEnv[seg], dt, lambda);
        }
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
        Glass* m = dynamic_cast<Glass*>(module);
        if (!m) return;

        struct FloatQ : Quantity {
            float* val; float mn, mx, def; std::string lbl;
            FloatQ(float* v, float lo, float hi, float d, std::string l)
                : val(v), mn(lo), mx(hi), def(d), lbl(l) {}
            void  setValue(float v) override { if (val) *val = clamp(v, mn, mx); }
            float getValue() override        { return val ? *val : def; }
            float getDefaultValue() override { return def; }
            float getMinValue() override     { return mn; }
            float getMaxValue() override     { return mx; }
            int   getDisplayPrecision() override { return 3; }
            std::string getLabel() override  { return lbl; }
            std::string getDisplayValueString() override {
                return string::f("%.3f", val ? *val : def);
            }
        };
        auto addFSlider = [&](float* v, float lo, float hi, float def, std::string lbl) {
            auto* sl = new ui::Slider();
            sl->quantity   = new FloatQ(v, lo, hi, def, lbl);
            sl->box.size.x = 200.f;
            menu->addChild(sl);
        };

        menu->addChild(new MenuSeparator());
        menu->addChild(createMenuLabel("Pressure Envelope"));
        addFSlider(&m->attackValue,   0.f, 1.f,  0.15f, "Attack");
        addFSlider(&m->releaseValue,  0.f, 1.f,  0.35f, "Release");
        menu->addChild(createMenuLabel("Envelope Curves"));
        addFSlider(&m->attackCurve,  -1.f, 1.f,  0.3f,  "Attack Curve (log - exp)");
        addFSlider(&m->releaseCurve, -1.f, 1.f, -0.3f,  "Release Curve (log - exp)");

        menu->addChild(new MenuSeparator());
        menu->addChild(createMenuLabel("Water (Grip) Noise Color"));
        addFSlider(&m->noiseCutoffMax, 100.f, 4000.f, 1000.f, "Max Noise Cutoff Hz (dry setting)");

        menu->addChild(new MenuSeparator());
        struct PanicItem : MenuItem {
            Glass* module;
            PanicItem(Glass* m) : module(m) { text = "Panic: Clear All Bowl Energy"; }
            void onAction(const ActionEvent&) override { if (module) module->panic(); }
        };
        menu->addChild(new PanicItem(m));
    }
};

Model* modelGlass = createModel<Glass, GlassWidget>("Glass");