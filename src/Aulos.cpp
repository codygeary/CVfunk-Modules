////////////////////////////////////////////////////////////
//
//   Aulos
//
//   written by Cody Geary
//   Copyright 2026, MIT License
//
//   Polyphonic physical-model wind instrument synthesizer.
//   Dual-waveguide bore resonator with reed/flute excitation morph,
//   ASR breath envelope, Dynamics self-patching, and stereo R voice.
//
////////////////////////////////////////////////////////////

#include "plugin.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include "FilterAulos.h"

static constexpr int AULOS_MAX_POLY = 16;

struct AulosVoice {
    AulosWaveguide   primaryWaveguide;
    AulosWaveguide   secondaryWaveguide;
    AulosJetDelay    jetDelay;
    AulosBreathEnv   breathEnv;
    ADAADrive        loopSaturator;
    AulosDCBlocker   dcBlocker;
    AulosDCBlocker   outputDCBlocker;
    AulosNyquistCap  outputCap;
    AulosEnvFollower dynFollower;
    AulosOnePoleHPF  envHPF;
    AulosOnePoleLPF  envLPF;

    bool  lastGateHigh      = false;

    float breathOut = 0.f;
    float dynEnvOut = 0.f;

    // t_ = target (set in skip block), a_ = audio-rate smoothed value.
    float t_reedMorph = 0.5f, a_reedMorph = 0.5f;
    float t_bore      = 0.3f, a_bore      = 0.3f;
    float t_tone      = 0.5f, a_tone      = 0.5f;
    float t_lip       = 0.8f, a_lip       = 0.8f;
    float t_noise     = 0.1f, a_noise     = 0.1f;
    float t_decay     = 0.7f, a_decay     = 0.7f;
    float t_damp      = 0.1f, a_damp      = 0.1f;
    float t_chiff     = 0.0f, a_chiff     = 0.0f;
    float t_blend     = 0.7f, a_blend     = 0.7f;

    int   chiffCounter        = 0;
    float chiffZ1             = 0.f;
    float cachedChiffDuration = 0.f;
    float cachedChiffAmp      = 0.f;

    float cachedDampCoeff = 0.02f;
    float cachedFeedback  = 0.93f;
    float cachedDecayGain = 0.9999f;

    // Smoothed primary delay - lerped each sample to avoid pitch CV clicks.
    float a_fingerDelay = 0.f;

    // Overblow / stability: safetyRMS tracks loop energy, safetyDecay rises
    // when energy exceeds threshold. Used for display and energy-overblow.
    float safetyRMS   = 0.f;
    float safetyDecay = 0.f;

    // Register: smoothed 0..1..2 value tracking the detected fingering register.
    // 0 = fundamental, 1 = first overblown (octave), 2 = second overblown (2 oct).
    // Smoothed per-sample so register transitions don't cause abrupt timbre jumps.
    float registerSmooth = 0.f;

    // Sleep flag - set when the voice is fully idle (gate low, breath and
    // overblow energy below threshold). A sleeping voice costs one gate
    // compare per sample in the voice loop; it wakes sample-accurately on
    // gate rise.
    bool asleep = true;

    // Startup mute ramp - rises from 0 to 1 over ~15ms on gate rise,
    // masking the waveguide DC transient while the bore settles.
    // Tune startupRampCoeff for a shorter/longer mask window.
    float startupGain = 1.f;

    void init(float sr) {
        primaryWaveguide.init(sr,   0.055f);
        secondaryWaveguide.init(sr, 0.028f);
        // Jet buffer must cover ~half the period of the lowest fundamental
        // (jet delay = 0.47 * period). 0.03s reaches down to ~16Hz.
        jetDelay.init(sr, 0.03f);
        dcBlocker.setSampleRate(sr);
        outputDCBlocker.setSampleRate(sr);
        outputCap.setSampleRate(sr);
        dynFollower.setCoeff(sqrtf(150.f * 5000.f) / sr, 0.25f, sr);
        envHPF.setCutoff(sr, 150.f);
        envLPF.setCutoff(sr, 5000.f);
        breathEnv.reset();
        loopSaturator.reset();
        lastGateHigh      = false;
        breathOut         = 0.f;
        dynEnvOut         = 0.f;
        chiffCounter      = 0x7FFFFFFF;  // start exhausted; resets on gate rise
        chiffZ1           = 0.f;
        safetyRMS         = 0.f;
        safetyDecay       = 0.f;
        registerSmooth    = 0.f;
        a_fingerDelay     = 0.f;
        asleep            = true;
        startupGain       = 1.f;
    }

    void clear() {
        primaryWaveguide.clear();
        secondaryWaveguide.clear();
        jetDelay.clear();
        breathEnv.reset();
        loopSaturator.reset();
        dcBlocker.reset();
        outputDCBlocker.reset();
        outputCap.reset();
        dynFollower.reset();
        envHPF.reset();
        envLPF.reset();
        lastGateHigh      = false;
        breathOut         = 0.f;
        dynEnvOut         = 0.f;
        chiffCounter      = 0x7FFFFFFF;  // start exhausted; resets on gate rise
        chiffZ1           = 0.f;
        safetyRMS         = 0.f;
        safetyDecay       = 0.f;
        registerSmooth    = 0.f;
        a_fingerDelay     = 0.f;
        asleep            = true;
        startupGain       = 1.f;
        a_reedMorph = t_reedMorph = 0.5f;
        a_bore      = t_bore      = 0.3f;
        a_tone      = t_tone      = 0.5f;
        a_lip       = t_lip       = 0.8f;
        a_noise     = t_noise     = 0.1f;
        a_decay     = t_decay     = 0.7f;
        a_damp      = t_damp      = 0.1f;
        a_chiff     = t_chiff     = 0.0f;
        a_blend     = t_blend     = 0.7f;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Aulos module
// ─────────────────────────────────────────────────────────────────────────────
struct Aulos : Module {

    enum ParamId {
        // Slider bank - 8 sliders (each PARAM + ATT)
        EDGE_PARAM,       EDGE_ATT,
        JET_PARAM,        JET_ATT,
        DRIVE_PARAM,      DRIVE_ATT,
        RES_PARAM,        RES_ATT,        // loop feedback (Lip / Holes on panel)
        AIR_PARAM,        AIR_ATT,
        CHIFF_PARAM,      CHIFF_ATT,
        AULOS_PARAM,      AULOS_ATT,   // R channel pipe pitch offset -1..+1 oct
        WARMTH_PARAM,     WARMTH_ATT,
        BREATH_PARAM,     BREATH_ATT,
        VIBRATO_ATT,
        PIPE_TUNE_PARAM,
        FINGER_TUNE_PARAM,
        MANUAL_GATE_BTN,
        DRONE_BTN,
        // Output
        VOLUME_PARAM,     VOLUME_ATT,
        // Audio in gain
        AUDIO_IN_GAIN_PARAM, AUDIO_IN_GAIN_ATT,
        PARAMS_LEN
    };

    enum InputId {
        PIPE_VOCT_INPUT,
        FINGER_VOCT_INPUT,
        GATE_INPUT,
        BREATH_CV_INPUT,
        VIBRATO_INPUT,
        EDGE_CV_INPUT,
        JET_CV_INPUT,
        DRIVE_CV_INPUT,
        RES_CV_INPUT,
        AIR_CV_INPUT,
        AULOS_CV_INPUT,  // CV for R channel pitch offset
        WARMTH_CV_INPUT,
        CHIFF_CV_INPUT,
        AUDIO_IN_INPUT,
        DRONE_CV_INPUT,
        VOLUME_CV_INPUT,
        AUDIO_IN_CV_INPUT,
        INPUTS_LEN
    };

    enum OutputId {
        AUDIO_L_OUTPUT,
        AUDIO_R_OUTPUT,
        ENV_OUTPUT,
        RMS_OUTPUT,
        OUTPUTS_LEN
    };

    enum LightId {
        BREATH_LIGHT_0,
        EXCITE_LIGHT_0    = BREATH_LIGHT_0 + 10,
        MANUAL_GATE_LIGHT = EXCITE_LIGHT_0 + 10,
        DRONE_LIGHT       = MANUAL_GATE_LIGHT + 1,
        LIGHTS_LEN        = DRONE_LIGHT + 1
    };

    AulosVoice voices[AULOS_MAX_POLY];   // left / primary voices
    AulosVoice voicesR[AULOS_MAX_POLY];  // right / drone voices
    int   nVoices    = 1;
    int   prevVoices = 0;
    float sampleRate = 48000.f;
    float moduleTime = 0.f;

    int  skipCounter = SKIP_MAX;  // start at max so the first process() call runs the skip block
    static constexpr int SKIP_MAX = 200;

    // Pitch path decimation - voct2freq and related transcendentals are
    // recomputed every PITCH_DECIM samples (~6kHz update at 48kHz). The
    // resulting delay lengths are already smoothed per-sample by a_fingerDelay.
    static constexpr int PITCH_DECIM = 8;
    int   pitchCounter = 0;
    float cachedPipeFreq[AULOS_MAX_POLY];
    float cachedFingerFreq[AULOS_MAX_POLY];
    float cachedPipeFreqR[AULOS_MAX_POLY];
    float cachedFingerFreqR[AULOS_MAX_POLY];

    // Set in the skip block - attack/release only change via context menu.
    float cachedAttackSamples  = 1000.f;
    float cachedReleaseSamples = 1000.f;

    // R output connection state - R voices are skipped entirely when unpatched,
    // and cleared on reconnect so stale buffer energy doesn't play back.
    bool prevRConnected = false;

    float followTime    = 0.25f;
    float attackCurve   = 0.3f;
    float releaseCurve  = -0.5f;
    float waveguideGain = 1.0f;
    float decayValue    = 0.7f;
    float attackValue   = 0.1f;   // attack time 0..1, set in context menu
    float releaseValue  = 0.3f;   // release time 0..1, set in context menu

    // Internal vibrato LFO. Free-running sine, normalled into the VIBRATO_INPUT:
    // when no cable is patched, this drives both pitch and breath modulation;
    // otherwise the patched CV takes over pitch only. Depth is set by VIBRATO_ATT.
    float vibratoRate        = 7.0f;   // Hz, set in context menu (good flute range 5-9)
    float vibratoBreathDepth = 0.5f;   // breath wobble relative to pitch wobble, 0..1
    float vibratoPhase       = 0.f;    // 0..1 accumulator

    // Legato mode. When enabled and the gate stays high while pitch changes,
    // cachedFingerFreq is glided toward the new target rather than jumping,
    // and loopFeedback briefly dips to create the "give" of a slur.
    bool  legatoEnabled  = true;
    float legatoTime     = 20.f;  // glide time in ms, set in context menu

    // Per-voice glide state: current glided frequency and target.
    float legatoStartL[AULOS_MAX_POLY]    = {};
    float legatoStartR[AULOS_MAX_POLY]    = {};
    float legatoLog2TargetL[AULOS_MAX_POLY] = {};
    float legatoLog2TargetR[AULOS_MAX_POLY] = {};
    float legatoProgressL[AULOS_MAX_POLY] = {};
    float legatoProgressR[AULOS_MAX_POLY] = {};

    // Per-voice feedback-dip ramp, 1.0 at transition start, decays to 0.
    // Applied as loopFeedback multiplier so the resonator briefly releases
    // energy at the slur point - the audible "give" between notes.
    float legatoDipL[AULOS_MAX_POLY] = {};
    float legatoDipR[AULOS_MAX_POLY] = {};

    // Per-sample glide step for the frequency interpolation, computed in the
    // skip block. Sized so PITCH_DECIM steps advance the glide proportionally
    // to legatoTime - the glide completes over the full menu time.
    float cachedLegatoStep = 0.f;

    bool droneActive      = false;
    bool manualGateActive = false;
    rack::dsp::SchmittTrigger droneToggleTrig;
    rack::dsp::SchmittTrigger droneCVTrig;

    float displayActiveFraction  = 1.0f;
    float displayBreath          = 0.0f;
    float displayActiveFractionR = 1.0f;
    float displayBreathR         = 0.0f;
    float displayOverblow        = 0.0f;
    float displayOverblowR       = 0.0f;
    float displayRegister        = 0.0f;
    float displayRegisterR       = 0.0f;
    float displayPipeRatio       = 1.0f;
    float displayPipeFreq        = 261.63f;
    // New display quantities
    float displayRMS             = 0.0f;
    float displayRMSR            = 0.0f;
    float displayAir             = 0.0f;
    float displayChiff           = 0.0f;
    float displayChiffR          = 0.0f;
    float displayEdge            = 0.0f;
    float displaySaturation      = 0.0f;
    float displaySaturationR     = 0.0f;

    int safetyCounter = 0;
    const float idleThreshold = 0.0005f;

    // R channel fingering mode:
    // false = Two Pipes (finger fixed, each pipe has its own register)
    // true  = Tracking  (finger shifts with offset, interval preserved)
    bool aulosTrack = false;

    float vuBreath[10] = {};
    float vuExcite[10] = {};

    uint32_t rngState = 0xA510500Du;
    inline float lcgRandf() {
        rngState = rngState * 1664525u + 1013904223u;
        return (float)(rngState & 0xFFFF) / 32767.5f - 1.f;
    }

    float getCV(InputId cvId, ParamId attId, float paramVal, int vi = 0) {
        float cv = inputs[cvId].isConnected()
                 ? inputs[cvId].getPolyVoltage(vi) : 0.f;
        return paramVal + params[attId].getValue() * cv * 0.1f;
    }

    static float voct2freq(float voct) {
        return 261.63f * rack::dsp::exp2_taylor5(voct);
    }

    // Waveguide delay length in samples.
    static float freqToDelaySamples(float freq, float bore, float sr) {
        return sr / freq;
    }

    void panic() {
        for (int vi = 0; vi < AULOS_MAX_POLY; ++vi) {
            voices[vi].clear();
            voicesR[vi].clear();
        }
    }

    Aulos() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(EDGE_PARAM,          0.f,  1.f,  0.1f, "Edge");
        configParam(EDGE_ATT,           -1.f,  1.f,  0.f,  "Edge Att.");
        configParam(JET_PARAM,           0.f,  1.f,  0.0f, "Jet");
        configParam(JET_ATT,            -1.f,  1.f,  0.f,  "Jet Att.");
        configParam(DRIVE_PARAM,         0.f,  1.f,  0.6f, "Drive");
        configParam(DRIVE_ATT,          -1.f,  1.f,  0.f,  "Drive Att.");
        configParam(RES_PARAM,           0.f,  1.f,  0.6f, "Resonance");
        configParam(RES_ATT,            -1.f,  1.f,  0.f,  "Resonance Att.");
        configParam(AIR_PARAM,           0.f,  1.f,  0.3f, "Air");
        configParam(AIR_ATT,            -1.f,  1.f,  0.f,  "Air Att.");
        configParam(CHIFF_PARAM,         0.f,  1.f,  0.4f, "Chiff");
        configParam(CHIFF_ATT,          -1.f,  1.f,  0.f,  "Chiff Att.");
        configParam(AULOS_PARAM,        -1.f,  1.f,  0.0f, "Aulos");
        configParam(AULOS_ATT,          -1.f,  1.f,  0.f,  "Aulos Att.");
        configParam(WARMTH_PARAM,        0.f,  1.f,  0.1f, "Warmth");
        configParam(WARMTH_ATT,         -1.f,  1.f,  0.f,  "Warmth Att.");
        configParam(BREATH_PARAM,        0.f,  1.f,  0.7f, "Breath");
        configParam(BREATH_ATT,         -1.f,  1.f,  0.f,  "Breath Att.");
        configParam(VIBRATO_ATT,         0.f,  1.f,  0.3f, "Vibrato Depth");
        configParam(PIPE_TUNE_PARAM,    -1.f,  4.f,  0.f,  "Pipe Tune", " oct");
        configParam(FINGER_TUNE_PARAM,  -1.f,  1.f,  0.f,  "Finger Tune", " oct");
        configParam(MANUAL_GATE_BTN,     0.f,  1.f,  0.f,  "Manual Gate");
        configParam(DRONE_BTN,           0.f,  1.f,  0.f,  "Drone Mode");
        configParam(VOLUME_PARAM,        0.f,  1.f,  0.5f, "Volume");
        configParam(VOLUME_ATT,         -1.f,  1.f,  0.f,  "Volume Att.");
        configParam(AUDIO_IN_GAIN_PARAM, 0.f,  1.f,  0.5f, "Audio In Gain");
        configParam(AUDIO_IN_GAIN_ATT,  -1.f,  1.f,  0.f,  "Audio In Gain Att.");

        configInput(PIPE_VOCT_INPUT,   "Pipe V/Oct");
        configInput(FINGER_VOCT_INPUT, "Finger V/Oct");
        configInput(GATE_INPUT,        "Gate");
        configInput(BREATH_CV_INPUT,   "Breath CV");
        configInput(VIBRATO_INPUT,     "Vibrato (FM)");
        configInput(EDGE_CV_INPUT,     "Edge CV");
        configInput(JET_CV_INPUT,      "Jet CV");
        configInput(DRIVE_CV_INPUT,    "Drive CV");
        configInput(RES_CV_INPUT,      "Resonance CV");
        configInput(AIR_CV_INPUT,      "Air CV");
        configInput(AULOS_CV_INPUT,    "Aulos CV");
        configInput(WARMTH_CV_INPUT,   "Warmth CV");
        configInput(CHIFF_CV_INPUT,    "Chiff CV");
        configInput(AUDIO_IN_INPUT,    "Audio In");
        configInput(DRONE_CV_INPUT,    "Drone CV");
        configInput(VOLUME_CV_INPUT,   "Volume CV");
        configInput(AUDIO_IN_CV_INPUT, "Audio In Gain CV");

        configOutput(AUDIO_L_OUTPUT, "Audio L");
        configOutput(AUDIO_R_OUTPUT, "Audio R");
        configOutput(ENV_OUTPUT,     "Breath Envelope");
        configOutput(RMS_OUTPUT,     "Excitation (RMS)");

        for (int vi = 0; vi < AULOS_MAX_POLY; ++vi) {
            cachedPipeFreq[vi]    = 261.63f;
            cachedFingerFreq[vi]  = 261.63f;
            cachedPipeFreqR[vi]   = 261.63f;
            cachedFingerFreqR[vi] = 261.63f;
        }
    }

    void onSampleRateChange() override {
        sampleRate = APP->engine->getSampleRate();
        for (int vi = 0; vi < AULOS_MAX_POLY; ++vi) {
            voices[vi].init(sampleRate);
            voicesR[vi].init(sampleRate);
        }
    }

    void onReset() override {
        for (int vi = 0; vi < AULOS_MAX_POLY; ++vi) {
            voices[vi].clear();
            voicesR[vi].clear();
        }
        followTime    = 0.25f;
        attackCurve   = 0.3f;
        releaseCurve  = -0.5f;
        waveguideGain = 1.0f;
        decayValue    = 0.7f;
        droneActive   = false;
        moduleTime    = 0.f;
        attackValue   = 0.3f;
        releaseValue  = 0.5f;
    }

    json_t* dataToJson() override {
        json_t* root = json_object();
        json_object_set_new(root, "droneActive",  json_boolean(droneActive));
        json_object_set_new(root, "aulosTrack",   json_boolean(aulosTrack));
        json_object_set_new(root, "followTime",   json_real(followTime));
        json_object_set_new(root, "attackCurve",  json_real(attackCurve));
        json_object_set_new(root, "releaseCurve", json_real(releaseCurve));
        json_object_set_new(root, "waveguideGain",json_real(waveguideGain));
        json_object_set_new(root, "decayValue",   json_real(decayValue));
        json_object_set_new(root, "attackValue",  json_real(attackValue));
        json_object_set_new(root, "releaseValue", json_real(releaseValue));
        json_object_set_new(root, "vibratoRate",       json_real(vibratoRate));
        json_object_set_new(root, "vibratoBreathDepth",json_real(vibratoBreathDepth));
        json_object_set_new(root, "legatoEnabled", json_boolean(legatoEnabled));
        json_object_set_new(root, "legatoTime",   json_real(legatoTime));
        return root;
    }

    void dataFromJson(json_t* root) override {
        auto gb = [&](const char* k, bool d) -> bool {
            json_t* j = json_object_get(root, k);
            return j ? json_is_true(j) : d; };
        auto gr = [&](const char* k, float d) -> float {
            json_t* j = json_object_get(root, k);
            return j ? (float)json_number_value(j) : d; };
        droneActive   = gb("droneActive",  false);
        aulosTrack    = gb("aulosTrack",   false);
        followTime    = gr("followTime",   0.25f);
        attackCurve   = gr("attackCurve",  0.3f);
        releaseCurve  = gr("releaseCurve", -0.5f);
        waveguideGain = gr("waveguideGain",1.0f);
        decayValue    = gr("decayValue",   0.7f);
        attackValue   = gr("attackValue",  0.3f);
        releaseValue  = gr("releaseValue", 0.5f);
        vibratoRate        = gr("vibratoRate",        7.0f);
        vibratoBreathDepth = gr("vibratoBreathDepth", 0.5f);
        legatoEnabled = gb("legatoEnabled", false);
        legatoTime    = gr("legatoTime",   60.f);
    }

    // ── DSP helper: process one voice, one sample ─────────────────────────────
    float processVoice(AulosVoice& v,
                       float sr,
                       bool  gateHigh,
                       float breathRaw,
                       float attackSamples,
                       float releaseSamples,
                       float attackCurveV,
                       float releaseCurveV,
                       float pipeFreq,
                       float fingerFreq,
                       float audioIn,
                       float waveguideGainV,
                       float cachedDecayGainV,
                       float legatoDip,
                       bool  legatoActive = false,
                       float refPipeFreq = 0.f) {
    
        // ── Gate rise ─────────────────────────────────────────────────────────
        if (gateHigh && !v.lastGateHigh) {
            v.chiffCounter  = 0;
            v.safetyRMS     = 0.f;
            v.safetyDecay   = 0.f;
            v.a_fingerDelay = 0.f;
            // In legato the waveguide is already running - skip the startup mask
            // so the note change is seamless. Only mute on a cold note-on.
            if (!legatoActive) v.startupGain = 0.f;
            // Do not reset registerSmooth on gate rise - legato register
            // transitions carry over smoothly between notes.
        }
        v.lastGateHigh = gateHigh;

        // ── Breath envelope ───────────────────────────────────────────────────
        v.breathOut = v.breathEnv.process(gateHigh, breathRaw,
            attackSamples, releaseSamples,
            attackCurveV,  releaseCurveV);
        float breathLevel = v.breathOut * 0.1f * breathRaw;

        // ── Sleep detector ────────────────────────────────────────────────────
        float activity = v.breathOut + fabsf(v.safetyDecay);
        if (!gateHigh && activity < idleThreshold) {
            v.asleep         = true;
            v.breathOut      = 0.f;
            v.safetyRMS      = 0.f;
            v.dynEnvOut      = 0.f;
            v.registerSmooth = 0.f;
            return 0.f;
        }

        // ── Register detection ────────────────────────────────────────────────
        // The ratio of finger frequency to pipe fundamental determines which
        // register the player is targeting. Each register octave folds the
        // finger frequency down so the bore reuses the same [0.5, 1.02] range
        // and the jet phase selects which comb mode locks.
        //
        // Thresholds are set half a semitone below each octave boundary so
        // slightly flat CV still resolves to the correct register:
        //   ratio < 0.97:        below fundamental  -> underblown
        //   0.97 .. 1.94:        first octave       -> register 1 (fundamental)
        //   1.94 .. 3.88:        second octave      -> register 2 (overblown 8va)
        //   >= 3.88:             above              -> register 3 (overblown 15ma)
        float refFreq     = (refPipeFreq > 0.f) ? refPipeFreq : pipeFreq;
        float fingerRatio = fingerFreq / refFreq;

        float registerTarget;
        float foldedFingerFreq;
        if (fingerRatio >= 3.88f) {
            registerTarget   = 2.f;
            foldedFingerFreq = fingerFreq * 0.25f; // fold 2 octaves down
        } else if (fingerRatio >= 1.94f) {
            registerTarget   = 1.f;
            foldedFingerFreq = fingerFreq * 0.5f;  // fold 1 octave down
        } else {
            registerTarget   = 0.f;
            foldedFingerFreq = fingerFreq;          // no fold
        }

        // Smooth the register value so timbre morphs over ~200ms rather than
        // snapping. Tune the coefficient for faster/slower register transitions.
        const float registerLerpCoeff = 0.002f;
        v.registerSmooth += registerLerpCoeff * (registerTarget - v.registerSmooth);

        // ── Pitch ─────────────────────────────────────────────────────────────
        // activeFraction uses the folded finger frequency so all registers
        // map into the same bore-length range, reusing the same tube geometry.
        float activeFraction = refFreq / foldedFingerFreq;
        activeFraction = clamp(activeFraction, 0.50f, 1.02f);

        float fullPipeDelaySamples = freqToDelaySamples(pipeFreq, v.a_bore, sr);
        float primaryDelaySamples  = fullPipeDelaySamples * activeFraction;
        primaryDelaySamples = clamp(primaryDelaySamples, 2.f, (float)v.primaryWaveguide.bufSize - 4.f);

        // Energy-overblow: very loud playing pushes safetyDecay above 0.5,
        // halving the tube period so the mode bumps up. This is independent of
        // register and composes with it - blowing hard in register 1 can tip
        // into a brief register-2 excitation.
        float overblowAmt    = clamp(v.safetyDecay, 0.f, 1.f);
        float overblowTarget = primaryDelaySamples * ((overblowAmt > 0.5f) ? 0.5f : 1.0f);

        if (v.a_fingerDelay < 2.f) v.a_fingerDelay = overblowTarget;
        if (legatoActive) {
            // The outer loop is already gliding fingerFreq smoothly, so
            // a_fingerDelay should track the current overblowTarget directly.
            // Any lag here would cause the bore to fall behind the jet,
            // breaking regeneration mid-glide.
            v.a_fingerDelay = overblowTarget;
        } else {
            // Fast one-pole removes zipper noise from the 8-sample pitch decimation.
            v.a_fingerDelay += 0.05f * (overblowTarget - v.a_fingerDelay);
        }
        primaryDelaySamples = v.a_fingerDelay;

        // Secondary waveguide ratio: tuned toward the played mode's sub-harmonic
        // to reinforce mode lock and add the characteristic hollow upper-register
        // tone. Register 1 base tracks bore (0.333..0.5). Register 2 moves to 0.5
        // (reinforces 2nd harmonic). Register 3 moves to 0.25 (reinforces 4th).
        float secondaryRatioBase = 0.333f + v.a_bore * (0.5f - 0.333f);
        float secondaryRatio;
        if (v.registerSmooth >= 1.f) {
            float blend    = clamp(v.registerSmooth - 1.f, 0.f, 1.f);
            secondaryRatio = 0.5f + blend * (0.25f - 0.5f);  // reg2 -> reg3
        } else {
            secondaryRatio = secondaryRatioBase + v.registerSmooth * (0.5f - secondaryRatioBase); // reg1 -> reg2
        }
        float secondaryDelaySamples = primaryDelaySamples * secondaryRatio * 0.994f * (1.f - v.safetyDecay * 0.5f);
        secondaryDelaySamples = clamp(secondaryDelaySamples, 2.f, (float)v.secondaryWaveguide.bufSize - 4.f);

        // ── Excitation ────────────────────────────────────────────────────────
        // Noise increases per register - upper registers are inherently breathy.
        // Tune noiseRegScale to adjust how much breath noise each register adds.
        const float noiseRegScale = 0.4f;
        float noiseAmp = v.a_noise * v.a_noise * 0.03f * v.breathOut
                       * (1.f + v.registerSmooth * noiseRegScale);
        float noiseVal = lcgRandf() * noiseAmp;

        // toneGain drives the reed saturator. Computed fresh each sample so
        // the stability duck never accumulates into a_tone state.
        float toneGain = 1.5f + v.a_tone * 1.7f;

        // Stability tone duck - reduces drive when loop energy is too high,
        // preventing HF runaway without corrupting a_tone.
        // Tune 0.75 for more/less aggressiveness.
        float overdrive = clamp((v.safetyRMS - 1.2f) * 0.5f, 0.f, 1.f);
        toneGain *= (1.f - overdrive * 0.75f);

        float effectiveDrive = toneGain * (0.3f + v.a_reedMorph * 0.3f) * (1.0f + v.a_bore * 0.3f);
        float reedExcite     = breathLevel + noiseVal;
        float tubeEndSample  = v.primaryWaveguide.readEnd(primaryDelaySamples);

        // Jet bias: higher registers need more air velocity to lock the mode.
        // Tune jetRegScale for how eagerly the flute speaks in upper registers.
        const float jetRegScale = 0.25f;
        float jetBias  = breathLevel * (0.8f - v.a_bore * 0.6f) * (1.f + v.registerSmooth * jetRegScale);
        float jetInput = tubeEndSample + jetBias + noiseVal;
        float jetOut   = aulosJetFunction(clamp(jetInput, -3.f, 3.f));

        // Jet travel time ~0.47 of the played period. fingerFreq (unfolded) is
        // correct here - the jet delay models the physical embouchure-to-opening
        // distance, which is independent of which bore mode locks.
        float jetDelaySamples = clamp(sr * 0.00107f * (440.f / fingerFreq), 2.f, sr * 0.028f);

        float fluteExcite = v.jetDelay.process(jetOut, jetDelaySamples) * 1.3f;
        float fluteAmt    = 0.3f + (1.f - v.a_reedMorph) * 0.55f;
        float reedAmt     = 0.15f + v.a_reedMorph * 0.70f;  //these are aligned so that we always have a mix of both modes
        float excite      = reedExcite * reedAmt + fluteExcite * fluteAmt; 
        excite += audioIn;
        excite *= waveguideGainV;

        // ── Chiff transient ───────────────────────────────────────────────────
        if (v.chiffCounter < (int)v.cachedChiffDuration) {
            float env        = 1.f - (float)v.chiffCounter / v.cachedChiffDuration;
            float chiffNoise = lcgRandf() * v.cachedChiffAmp * env * v.breathOut * 0.1f;
            v.chiffZ1        = 0.92f * v.chiffZ1 + 0.08f * chiffNoise;
            excite          += (chiffNoise - v.chiffZ1);
            v.chiffCounter++;
        }

        // ── Waveguides ────────────────────────────────────────────────────────
        float loopSat      = v.loopSaturator.process(excite, effectiveDrive);
        float envelopeGate = clamp(v.breathOut * 0.15f, 0.f, 1.f);
        float loopFeedback = rack::crossfade(cachedDecayGainV * 0.8f, v.cachedFeedback, envelopeGate); // feedback dampens when we don't blow

        // Legato inflection: at a slur transition legatoDip is 1, decaying to 0
        // over ~20ms. + Subtle feedback dip - a slight release of resonator energy so the
        //      new pitch can establish without fighting the old one's stored energy.
        // Tune dipFeedbackDepth for more/less resonator release (0 = none).
        const float dipFeedbackDepth = 0.18f;
        loopFeedback *= (1.f - legatoDip * dipFeedbackDepth);

        // Feedback reduces slightly in higher registers - upper registers are
        // less stable and more sensitive to embouchure. Tune registerFeedbackDrop.
        const float registerFeedbackDrop = 0.03f;
        loopFeedback *= (1.f - v.registerSmooth * registerFeedbackDrop);

        float boreDamp = (0.35f - v.a_bore * 0.28f) * clamp(261.63f / fingerFreq, 0.25f, 1.0f);
        // At C4: full boreDamp. Above C4: scales down - higher pitch, less damping.

        float reedDamp = (1.f - v.a_reedMorph) * 0.5f;

        // Register 3 adds extra damping - thinner, more penetrating tone.
        // Tune registerDampScale to adjust how much extra loss the top register has.
        const float registerDampScale = 0.04f;
        float registerDampExtra = clamp(v.registerSmooth - 1.f, 0.f, 1.f) * registerDampScale;
        float totalDamp = clamp(v.cachedDampCoeff + reedDamp + registerDampExtra, 0.f, 0.95f);

        float primaryOut    = v.primaryWaveguide.process(loopSat, primaryDelaySamples, totalDamp, loopFeedback, boreDamp);
        float secondaryFeed = (0.5f - v.a_bore * 0.35f) * (1.f - v.safetyDecay);
        float secondaryOut  = v.secondaryWaveguide.process(loopSat * secondaryFeed, secondaryDelaySamples, 0.01f, 0.99f);

        // ── Stability / overblow ──────────────────────────────────────────────
        // Single unified RMS tracker. Fast attack, slow release.
        // safetyDecay drives energy-overblow and the display overblow outline.
        // toneGain ducking on excessive energy is already applied above.
        {
            float instEnergy = (primaryOut * primaryOut + secondaryOut * secondaryOut) * 0.5f;
            float rmsTarget  = sqrtf(instEnergy);
            float rmsCoeff   = rmsTarget > v.safetyRMS ? 0.1f : 0.002f;
            v.safetyRMS     += rmsCoeff * (rmsTarget - v.safetyRMS);

            // Threshold scales with activeFraction so shorter bores (higher
            // fingerings) don't trip the energy-overblow as easily.
            float overblowThreshold = 1.6f + (activeFraction - 1.f) * 0.4f;
            if (v.safetyRMS > overblowThreshold) {
                v.safetyDecay = fminf(v.safetyDecay + 0.001f, 1.f);
            } else {
                v.safetyDecay *= 0.9995f;
                if (v.safetyDecay < 0.001f) v.safetyDecay = 0.f;
            }
        }

        // ── Output mix ────────────────────────────────────────────────────────
        float voiceOut = primaryOut * 0.5f + secondaryOut * 0.5f;
        voiceOut = v.dcBlocker.process(voiceOut);
        voiceOut = v.outputCap.process(voiceOut);
        voiceOut = v.outputDCBlocker.process(voiceOut);
        if (!std::isfinite(voiceOut)) voiceOut = 0.f;

        // ── RMS follower ──────────────────────────────────────────────────────
        {
            float midBand = v.envHPF.process(voiceOut);
            midBand       = v.envLPF.process(midBand);
            v.dynEnvOut   = v.dynFollower.process(midBand);
        }
        voiceOut *= clamp(sqrtf(440.f / fingerFreq), 0.25f, 3.0f); // pitch loudness correction

        // Ramp from 0 to 1 over ~15ms on note onset to mask waveguide startup transient.
        // One-pole approach to 1.0 - coeff ~0.004 reaches ~95% in ~15ms at 48kHz.
        // Tune coeff for a shorter/longer mask window: larger = faster ramp.
        v.startupGain += 0.004f * (1.f - v.startupGain);
        voiceOut *= v.startupGain;

        return voiceOut;
    }

    void process(const ProcessArgs& args) override {
        const float sr = args.sampleRate;
        moduleTime += args.sampleTime;

        // ── Buttons ───────────────────────────────────────────────────────────
        manualGateActive = params[MANUAL_GATE_BTN].getValue() > 0.5f;
        if (droneToggleTrig.process(params[DRONE_BTN].getValue())) {
            droneActive = !droneActive;
            for (int vi = 0; vi < AULOS_MAX_POLY; ++vi) voicesR[vi].clear();
        }
        if (inputs[DRONE_CV_INPUT].isConnected()) {
            if (droneCVTrig.process(inputs[DRONE_CV_INPUT].getVoltage())) {
                droneActive = !droneActive;
                for (int vi = 0; vi < AULOS_MAX_POLY; ++vi) voicesR[vi].clear();
            }
        }
        // ── Voice count ───────────────────────────────────────────────────────
        {
            int n = 1;
            if (inputs[GATE_INPUT].isConnected())
                n = std::max(n, inputs[GATE_INPUT].getChannels());
            if (inputs[FINGER_VOCT_INPUT].isConnected())
                n = std::max(n, inputs[FINGER_VOCT_INPUT].getChannels());
            if (inputs[PIPE_VOCT_INPUT].isConnected())
                n = std::max(n, inputs[PIPE_VOCT_INPUT].getChannels());
            nVoices = clamp(n, 1, AULOS_MAX_POLY);
        }
        if (nVoices != prevVoices) {
            for (int vi = nVoices; vi < prevVoices; ++vi) {
                voices[vi].clear();
                voicesR[vi].clear();
            }
            prevVoices = nVoices;
        }

        // ── Sub-rate control block ────────────────────────────────────────────
        const bool doSkip = (++skipCounter >= SKIP_MAX);
        if (doSkip) {
            skipCounter = 0;

            // Read all 8 slider values with their CV inputs.
            float sliderVal[8];
            sliderVal[0] = clamp(getCV(EDGE_CV_INPUT,  EDGE_ATT,  params[EDGE_PARAM].getValue()),  0.f, 1.f);
            sliderVal[1] = clamp(getCV(JET_CV_INPUT,  JET_ATT,  params[JET_PARAM].getValue()),  0.f, 1.f);
            sliderVal[2] = clamp(getCV(DRIVE_CV_INPUT,  DRIVE_ATT,  params[DRIVE_PARAM].getValue()),  0.f, 1.f);
            sliderVal[3] = clamp(getCV(RES_CV_INPUT,   RES_ATT,   params[RES_PARAM].getValue()),   0.f, 1.f);
            sliderVal[4] = clamp(getCV(AIR_CV_INPUT, AIR_ATT, params[AIR_PARAM].getValue()), 0.f, 1.f);
            sliderVal[5] = clamp(getCV(CHIFF_CV_INPUT, CHIFF_ATT, params[CHIFF_PARAM].getValue()), 0.f, 1.f);
            // sliderVal[6] = AULOS_PARAM - read per-sample below
            sliderVal[7] = clamp(getCV(WARMTH_CV_INPUT,  WARMTH_ATT,  params[WARMTH_PARAM].getValue()),  0.f, 1.f);

            // Damp -> loop LPF coefficient, two-phase mapping from Droplet:
            // Phase 1 (0->0.25):   blend in 8kHz LPF.
            // Phase 2 (0.25->1.0): sweep cutoff 8kHz -> 200Hz logarithmically.                
            static constexpr float LOG_8000 = 8.98719723f; // precomputed
            static constexpr float LOG_200  = 5.29831737f;
            static constexpr float LOG_RANGE = LOG_200 - LOG_8000;
            float sharedDampCoeff;
            {
                float d     = sliderVal[7];
                d = d*d;
                float blend = clamp(d / 0.25f, 0.f, 1.f);
                float freqT = clamp((d - 0.25f) / 0.75f, 0.f, 1.f);        
                float dFreq = expf(LOG_8000 + freqT * LOG_RANGE);
            
                sharedDampCoeff = blend * expf(-2.f * float(M_PI) * dFreq / sr);
            }

            // Lip -> loop feedback gain. Tune: range 0.75..0.98.
            float sharedFeedback = 0.75f + sliderVal[3] * 0.18f;  // max 0.93

            // Decay from context menu. Tune: range 0.80..0.97.
            float sharedDecayGain = 0.80f + decayValue * 0.17f;

            float chiffSlider = sliderVal[5];

            // Dynamics follower coefficient - identical for all voices, computed once.
            // Mirrors AulosEnvFollower::setCoeff with bandCenterNorm = sqrt(150*5000)/sr.
            float sharedDynCoeff;
            {
                float bandCenterNorm = sqrtf(150.f * 5000.f) / sr;
                float periodMs = (1.f / (bandCenterNorm * sr)) * 1000.f;
                periodMs = clamp(periodMs, 0.5f, 2000.f);
                float scale = 0.5f + followTime * followTime * 299.5f;
                sharedDynCoeff = 1.f - expf(-1.f / ((periodMs * scale * 0.001f) * sr));
            }

            // Attack/release only change via the context menu - sub-rate is plenty.
            cachedAttackSamples  = sr * 0.002f * powf(2000.f, clamp(attackValue,  0.f, 1.f));
            cachedReleaseSamples = sr * 0.002f * powf(2000.f, clamp(releaseValue, 0.f, 1.f));

            // Legato glide progress increment per doPitch tick.
            // Progress goes 0->1 linearly over legatoTime. Each doPitch tick
            // covers PITCH_DECIM samples, so increment = PITCH_DECIM / legatoSamples.
            {
                float legatoSamples = fmaxf(legatoTime * 0.001f * sr, 1.f);
                cachedLegatoStep = (float)PITCH_DECIM / legatoSamples;
            }

            for (int vi = 0; vi < nVoices; ++vi) {
                for (int side = 0; side < 2; ++side) {
                    AulosVoice& v = (side == 0) ? voices[vi] : voicesR[vi];
                    v.t_reedMorph         = sliderVal[0];
                    v.t_bore              = sliderVal[1];
                    v.t_tone              = sliderVal[2];
                    v.t_lip               = sliderVal[3];
                    v.t_noise             = sliderVal[4];
                    v.t_chiff             = chiffSlider;
                    v.t_damp              = sliderVal[7];
                    v.cachedFeedback      = sharedFeedback;
                    v.cachedDecayGain     = sharedDecayGain;
                    v.cachedDampCoeff     = sharedDampCoeff;
                    v.cachedChiffDuration = chiffSlider * chiffSlider * sr * 0.15f;
                    v.cachedChiffAmp      = chiffSlider * chiffSlider * 0.5f;
                    v.dynFollower.coeff   = sharedDynCoeff;

                    // Sleeping voices don't run the per-sample smoothing -
                    // snap them to target here so they wake with current values.
                    if (v.asleep) {
                        v.a_reedMorph = v.t_reedMorph;
                        v.a_bore      = v.t_bore;
                        v.a_tone      = v.t_tone;
                        v.a_lip       = v.t_lip;
                        v.a_noise     = v.t_noise;
                        v.a_decay     = v.t_decay;
                        v.a_damp      = v.t_damp;
                        v.a_chiff     = v.t_chiff;
                    }
                }
            }
        }
        // Smoothing coefficient for skip-block targets - constant, computed once.
        static const float lerpCoeff = 1.f - expf(-8.f / (float)SKIP_MAX);

        // ── Global audio-rate reads ───────────────────────────────────────────
        float fingerTune = params[FINGER_TUNE_PARAM].getValue();

        // ── Vibrato ───────────────────────────────────────────────────────────
        // The VIBRATO_ATT knob scales depth. CV at VIBRATO_INPUT overrides
        // the internal LFO when patched. The internal LFO is gated per-voice by
        // the breath envelope so vibrato fades in/out with each note. Patched CV
        // is used as raw pitch FM only - no breath coupling.
        float vibratoAtt    = params[VIBRATO_ATT].getValue();
        bool  vibratoPatched = inputs[VIBRATO_INPUT].isConnected();

        // Advance the free-running LFO once per sample. Phase wraps in 0..1.
        vibratoPhase += vibratoRate * args.sampleTime;
        if (vibratoPhase >= 1.f) vibratoPhase -= 1.f;
        float vibratoLFO = aulosDspSin(vibratoPhase * 2.f * (float)M_PI); // -1..1

        // External pitch FM (patched CV) - global, ungated, pitch only.
        float vibratoExternal = vibratoPatched ? (vibratoAtt * inputs[VIBRATO_INPUT].getVoltage() * 0.0167f) : 0.f;
        // Internal LFO depth before envelope gating. ~0.5 semitone peak at full knob.
        float vibratoInternal = vibratoPatched ? 0.f : (vibratoAtt * vibratoLFO * 0.5f * 0.0833f);
                      
        // Attack and release are computed in the skip block (cachedAttackSamples /
        // cachedReleaseSamples) since they only change via the context menu.
        float breathRaw = clamp(  params[BREATH_PARAM].getValue() + params[BREATH_ATT].getValue()
            * (inputs[BREATH_CV_INPUT].isConnected() ? inputs[BREATH_CV_INPUT].getVoltage() : 0.f) * 0.1f,  0.f, 1.f);

        breathRaw = breathRaw * breathRaw; //non-linear breath scaling
        if (breathRaw < 0.05f)
            breathRaw = breathRaw * (0.6f / 0.05f);
        else
            breathRaw = 0.6f + (breathRaw - 0.05f) * (0.4f / 0.95f);

        float attackSamples  = cachedAttackSamples;
        float releaseSamples = cachedReleaseSamples;
        float audioInGain = clamp(  params[AUDIO_IN_GAIN_PARAM].getValue() + params[AUDIO_IN_GAIN_ATT].getValue()
            * (inputs[AUDIO_IN_CV_INPUT].isConnected() ? inputs[AUDIO_IN_CV_INPUT].getVoltage() : 0.f) * 0.1f, 0.f, 1.f);
        float audioIn = inputs[AUDIO_IN_INPUT].isConnected() ? inputs[AUDIO_IN_INPUT].getVoltage() * 0.1f * audioInGain : 0.f;
        float volume = clamp( params[VOLUME_PARAM].getValue() + params[VOLUME_ATT].getValue()
            * (inputs[VOLUME_CV_INPUT].isConnected()  ? inputs[VOLUME_CV_INPUT].getVoltage() : 0.f) * 0.1f, 0.f, 1.f) * 3.f;

        // R voice pipe pitch offset in octaves. 0 = unison with L.
        float aulosOffset = clamp(
            params[AULOS_PARAM].getValue()
            + params[AULOS_ATT].getValue()
            * (inputs[AULOS_CV_INPUT].isConnected()
               ? inputs[AULOS_CV_INPUT].getVoltage() : 0.f), -1.f, 1.f);

        // Shared decay gain (recomputed per-sample since decayValue can change).
        float sharedDecayGain = 0.80f + decayValue * 0.17f;

        // Drone: R voice locked to pipe fundamental, gate always on.
        bool droneEffective = droneActive;
        const int displayVoice = 0;

        // R voices are only processed when the R output is patched. On reconnect,
        // clear them so stale buffer energy doesn't play back.
        const bool processR = outputs[AUDIO_R_OUTPUT].isConnected();
        if (processR && !prevRConnected) {
            for (int vi = 0; vi < AULOS_MAX_POLY; ++vi) voicesR[vi].clear();
        }
        prevRConnected = processR;

        // Pitch path decimation - recompute frequencies every PITCH_DECIM samples.
        const bool doPitch = (pitchCounter == 0);
        if (++pitchCounter >= PITCH_DECIM) pitchCounter = 0;

        // ── Voice loop ────────────────────────────────────────────────────────
        for (int vi = 0; vi < nVoices; ++vi) {
            AulosVoice& v  = voices[vi];
            AulosVoice& vR = voicesR[vi];

            // ── Gate and wake/sleep ───────────────────────────────────────────
            // Gate is read every sample so wake is sample-accurate. A sleeping
            // voice pays only for this compare. Waking forces a pitch recompute
            // so the voice doesn't start on a stale cached frequency.
            float gateV   = inputs[GATE_INPUT].getPolyVoltage(vi);
            bool gateHigh = (gateV > 1.f) || manualGateActive;
            bool gateHighR = droneEffective ? true : gateHigh;

            bool wokeL = v.asleep && gateHigh;
            if (wokeL) v.asleep = false;
            bool wokeR = processR && vR.asleep && gateHighR;
            if (wokeR) vR.asleep = false;

            const bool activeL = !v.asleep;
            const bool activeR = processR && !vR.asleep;

            if (!activeL && !activeR) {
                // Fully idle: outputs stay at zero, skip everything else.
                outputs[AUDIO_L_OUTPUT].setVoltage(0.f, vi);
                outputs[AUDIO_R_OUTPUT].setVoltage(0.f, vi);
                if (outputs[ENV_OUTPUT].isConnected())
                    outputs[ENV_OUTPUT].setVoltage(0.f, vi);
                if (outputs[RMS_OUTPUT].isConnected())
                    outputs[RMS_OUTPUT].setVoltage(0.f, vi);
                continue;
            }

            // Smooth targets for awake sides only. Sleeping sides are snapped
            // to their targets in the skip block.
            if (activeL) {
                v.a_reedMorph += lerpCoeff * (v.t_reedMorph - v.a_reedMorph);
                v.a_bore      += lerpCoeff * (v.t_bore      - v.a_bore);
                v.a_tone      += lerpCoeff * (v.t_tone      - v.a_tone);
                v.a_lip       += lerpCoeff * (v.t_lip       - v.a_lip);
                v.a_noise     += lerpCoeff * (v.t_noise     - v.a_noise);
                v.a_decay     += lerpCoeff * (v.t_decay     - v.a_decay);
                v.a_damp      += lerpCoeff * (v.t_damp      - v.a_damp);
                v.a_chiff     += lerpCoeff * (v.t_chiff     - v.a_chiff);
            }
            if (activeR) {
                vR.a_reedMorph += lerpCoeff * (vR.t_reedMorph - vR.a_reedMorph);
                vR.a_bore      += lerpCoeff * (vR.t_bore      - vR.a_bore);
                vR.a_tone      += lerpCoeff * (vR.t_tone      - vR.a_tone);
                vR.a_lip       += lerpCoeff * (vR.t_lip       - vR.a_lip);
                vR.a_noise     += lerpCoeff * (vR.t_noise     - vR.a_noise);
                vR.a_decay     += lerpCoeff * (vR.t_decay     - vR.a_decay);
                vR.a_damp      += lerpCoeff * (vR.t_damp      - vR.a_damp);
                vR.a_chiff     += lerpCoeff * (vR.t_chiff     - vR.a_chiff);
            }

            // ── Per-voice pitch (decimated) ───────────────────────────────────
            // All voct2freq calls run every PITCH_DECIM samples, and on wake.
            // The resulting delay lengths are smoothed per-sample inside
            // processVoice (a_fingerDelay), so the decimation is inaudible for
            // pitch CV and vibrato-rate FM.
            if (doPitch || wokeL || wokeR) {
                // Envelope-gate the internal vibrato by this voice's breath
                // envelope so it fades in/out with the note. Patched FM CV is
                // global and ungated. One-sample envelope lag is inaudible at
                // control rate.
                float vibGate    = clamp(v.breathOut * 0.1f, 0.f, 1.f);
                float fmDepth    = vibratoExternal + vibratoInternal * vibGate;

                // R voice vibrato gate. Drone runs gate-always-on so its vibrato
                // is continuous; otherwise tracks the R breath envelope.
                float vibGateR   = droneEffective ? 1.f : clamp(vR.breathOut * 0.1f, 0.f, 1.f);
                float fmDepthR   = vibratoExternal + vibratoInternal * vibGateR;

                float pipeVoct = (inputs[PIPE_VOCT_INPUT].isConnected()
                                 ? inputs[PIPE_VOCT_INPUT].getPolyVoltage(vi) : 0.f)
                               + params[PIPE_TUNE_PARAM].getValue();
                cachedPipeFreq[vi] = voct2freq(pipeVoct);

                // fingerVoctBase: pitch CV + tuning, no FM. Used as the stable
                // base for legato endpoint storage so vibrato doesn't drift the
                // glide targets or cause spurious re-arms each vibrato cycle.
                float fingerVoctBase = inputs[FINGER_VOCT_INPUT].isConnected()
                                     ? inputs[FINGER_VOCT_INPUT].getPolyVoltage(vi) + fingerTune
                                     : pipeVoct + fingerTune;
                // Full finger voct with FM - used for non-legato snap and R freq.
                float fingerVoct    = fingerVoctBase + fmDepth;
                float newFingerFreq = voct2freq(fingerVoct);

                // aulosOffset shifts the R pipe voct -> different tube length / timbre.
                float newFingerFreqR;
                cachedPipeFreqR[vi] = voct2freq(pipeVoct + aulosOffset + fmDepthR);
                newFingerFreqR = droneEffective ? cachedPipeFreqR[vi]
                               : (aulosTrack ? voct2freq(fingerVoct + aulosOffset)
                                             : newFingerFreq);

                if (legatoEnabled && !wokeL && gateHigh && legatoProgressL[vi] >= 0.f) {
                    // ── Legato glide L ────────────────────────────────────────
                    // Endpoints are stored in FM-free log2 space (fingerVoctBase)
                    // so the glide arc is stable. fmDepth is added back at output
                    // so vibrato continues uninterrupted through the slur.
                    // The arm threshold compares FM-free values, so vibrato wobble
                    // never causes a spurious re-arm.
                    // log2(261.63 * 2^fingerVoctBase) = log2(261.63) + fingerVoctBase
                    static constexpr float LOG2_C4 = 8.03178968f; // log2(261.63)
                    float newLog2Base = LOG2_C4 + fingerVoctBase;
                    if (fabsf(newLog2Base - legatoLog2TargetL[vi]) > 0.042f) {
                        // New target detected: arm from current glided position.
                        float currentLog2 = legatoStartL[vi] +
                            legatoProgressL[vi] * (legatoLog2TargetL[vi] - legatoStartL[vi]);
                        legatoStartL[vi]      = currentLog2;
                        legatoLog2TargetL[vi] = newLog2Base;
                        legatoProgressL[vi]   = 0.f;
                        legatoDipL[vi]        = 1.f;
                    }
                    legatoProgressL[vi] = fminf(1.f, legatoProgressL[vi] + cachedLegatoStep);
                    float glidedLog2    = legatoStartL[vi] +
                        legatoProgressL[vi] * (legatoLog2TargetL[vi] - legatoStartL[vi]);
                    // Re-apply FM so vibrato modulates pitch during the glide.
                    cachedFingerFreq[vi] = exp2f(glidedLog2 + fmDepth);
                } else {
                    // Fresh note (wokeL) or legato off: snap frequency immediately.
                    // While the gate is low between notes, leave legatoLog2TargetL
                    // alone - if we overwrote it here, the arm would see zero
                    // difference on the next gate rise and never fire.
                    cachedFingerFreq[vi] = newFingerFreq;
                    if (wokeL || !legatoEnabled) {
                        static constexpr float LOG2_C4 = 8.03178968f;
                        legatoStartL[vi]      = LOG2_C4 + fingerVoctBase;
                        legatoLog2TargetL[vi] = legatoStartL[vi];
                        legatoProgressL[vi]   = 1.f;
                    }
                }

                if (legatoEnabled && !wokeR && gateHighR && legatoProgressR[vi] >= 0.f && !droneEffective) {
                    // ── Legato glide R (non-drone only) ──────────────────────
                    float fingerVoctBaseR = aulosTrack ? fingerVoctBase + aulosOffset : fingerVoctBase;
                    static constexpr float LOG2_C4 = 8.03178968f;
                    float newLog2BaseR    = LOG2_C4 + fingerVoctBaseR;
                    if (fabsf(newLog2BaseR - legatoLog2TargetR[vi]) > 0.042f) {
                        float currentLog2R = legatoStartR[vi] +
                            legatoProgressR[vi] * (legatoLog2TargetR[vi] - legatoStartR[vi]);
                        legatoStartR[vi]      = currentLog2R;
                        legatoLog2TargetR[vi] = newLog2BaseR;
                        legatoProgressR[vi]   = 0.f;
                        legatoDipR[vi]        = 1.f;
                    }
                    legatoProgressR[vi] = fminf(1.f, legatoProgressR[vi] + cachedLegatoStep);
                    float glidedLog2R   = legatoStartR[vi] +
                        legatoProgressR[vi] * (legatoLog2TargetR[vi] - legatoStartR[vi]);
                    cachedFingerFreqR[vi] = exp2f(glidedLog2R + fmDepthR);
                } else {
                    cachedFingerFreqR[vi] = newFingerFreqR;
                    if (wokeR || !legatoEnabled) {
                        float fingerVoctBaseR = aulosTrack ? fingerVoctBase + aulosOffset : fingerVoctBase;
                        static constexpr float LOG2_C4 = 8.03178968f;
                        legatoStartR[vi]      = LOG2_C4 + fingerVoctBaseR;
                        legatoLog2TargetR[vi] = legatoStartR[vi];
                        legatoProgressR[vi]   = 1.f;
                    }
                }

                // Decay the dip ramps - linear over ~20ms regardless of legatoTime,
                // since the gap between notes should be brief and consistent.
                // Tune dipDecayStep for a longer/shorter release of the dip.
                const float dipDecayStep = PITCH_DECIM / (0.020f * sr); // 20ms
                legatoDipL[vi] = fmaxf(0.f, legatoDipL[vi] - dipDecayStep);
                legatoDipR[vi] = fmaxf(0.f, legatoDipR[vi] - dipDecayStep);
            }
            float pipeFreq    = cachedPipeFreq[vi];
            float fingerFreq  = cachedFingerFreq[vi];
            float pipeFreqR   = cachedPipeFreqR[vi];
            float fingerFreqR = cachedFingerFreqR[vi];

            // Vibrato breath coupling - internal LFO only (not patched CV).
            // vibratoLFO (-1..1) modulates breathRaw in-phase with pitch.
            // vibGate scales with breathOut so the wobble fades in/out with
            // the note envelope, matching how a player's air pressure varies.
            // Tune the 0.12 scalar for max wobble depth at full knob + full depth.
            float breathL = breathRaw;
            float breathR = breathRaw;
            if (!vibratoPatched) {
                float vibratoBreathAmt = vibratoAtt * vibratoBreathDepth * 0.12f;
                breathL = breathRaw * (1.f + vibratoLFO * clamp(v.breathOut  * 0.1f, 0.f, 1.f) * vibratoBreathAmt);
                float vibGateRForBreath = droneEffective ? 1.f : clamp(vR.breathOut * 0.1f, 0.f, 1.f);
                breathR = breathRaw * (1.f + vibratoLFO * vibGateRForBreath * vibratoBreathAmt);
            }

            // ── Left voice ────────────────────────────────────────────────────
            float voiceOut = 0.f;
            if (activeL) {
                voiceOut = processVoice(v,
                    sr, gateHigh,
                    breathL, attackSamples, releaseSamples,
                    attackCurve, releaseCurve,
                    pipeFreq, fingerFreq,
                    audioIn, waveguideGain, sharedDecayGain,
                    legatoDipL[vi],
                    legatoProgressL[vi] < 1.f);
            }

            // ── Right voice ───────────────────────────────────────────────────
            float voiceOutR = 0.f;
            if (activeR) {
                // Drone mode ducks R slightly so it sits behind the melody.
                float droneLevel = droneEffective ? 0.6f : 1.0f;

                voiceOutR = processVoice(vR,
                    sr, gateHighR,
                    breathR, attackSamples, releaseSamples,
                    attackCurve, releaseCurve,
                    pipeFreqR, fingerFreqR,
                    audioIn, waveguideGain, sharedDecayGain,
                    legatoDipR[vi],
                    legatoProgressR[vi] < 1.f,
                    aulosTrack ? pipeFreqR : pipeFreq);  // tracking: use R pipe as ref; two-pipes: use L pipe

                voiceOutR *= droneLevel;
            }

            // ── Per-voice outputs ─────────────────────────────────────────────
            outputs[AUDIO_L_OUTPUT].setVoltage(clamp(voiceOut  * volume, -10.f, 10.f), vi);
            outputs[AUDIO_R_OUTPUT].setVoltage(clamp(voiceOutR * volume,  -10.f, 10.f), vi);

            if (outputs[ENV_OUTPUT].isConnected())
                outputs[ENV_OUTPUT].setVoltage(v.breathOut, vi);
            if (outputs[RMS_OUTPUT].isConnected())
                outputs[RMS_OUTPUT].setVoltage(
                    clamp(v.dynEnvOut * 10.f, 0.f, 10.f), vi);
        }

        // ── Channel counts ────────────────────────────────────────────────────
        outputs[AUDIO_L_OUTPUT].setChannels(nVoices);
        outputs[AUDIO_R_OUTPUT].setChannels(nVoices);
        if (outputs[ENV_OUTPUT].isConnected())
            outputs[ENV_OUTPUT].setChannels(nVoices);
        if (outputs[RMS_OUTPUT].isConnected())
            outputs[RMS_OUTPUT].setChannels(nVoices);

        // ── Display data ──────────────────────────────────────────────────────
        // Updated at the pitch decimation rate - far above frame rate.
        if (doPitch && nVoices > 0 && displayVoice < nVoices) {
            displayPipeFreq  = cachedPipeFreq[displayVoice];
            displayBreath    = voices[displayVoice].breathOut * 0.1f;

            // Mirror the register fold from processVoice so the display shows
            // the bore length actually in use. Each register folds the finger
            // frequency down an octave, reusing the same physical hole position
            // while the jet locks a higher bore mode. Thresholds match the DSP.
            float fingerF  = cachedFingerFreq[displayVoice];
            float fingerFR = cachedFingerFreqR[displayVoice];
            float refL = displayPipeFreq;
            float refR = aulosTrack ? cachedPipeFreqR[displayVoice] : displayPipeFreq;

            float ratioL  = fingerF / refL;
            float foldedL = (ratioL >= 3.88f) ? fingerF * 0.25f
                          : (ratioL >= 1.94f) ? fingerF * 0.5f
                          : fingerF;
            displayActiveFraction = clamp(refL / foldedL, 0.50f, 1.02f);

            float ratioR  = fingerFR / refR;
            float foldedR = (ratioR >= 3.88f) ? fingerFR * 0.25f
                          : (ratioR >= 1.94f) ? fingerFR * 0.5f
                          : fingerFR;
            displayActiveFractionR = clamp(refR / foldedR, 0.50f, 1.02f);

            // Register comes straight from the voice DSP state, so register
            // transitions sweep the nodal pattern exactly as the timbre morphs.
            displayRegister  = voices[displayVoice].registerSmooth;
            displayRegisterR = processR ? voicesR[displayVoice].registerSmooth : 0.f;

            displayBreathR         = processR ? voicesR[displayVoice].breathOut * 0.1f * (droneEffective ? 0.6f : 1.0f) : 0.f;
            displayOverblow        = voices[displayVoice].safetyDecay;
            displayOverblowR       = processR ? voicesR[displayVoice].safetyDecay : 0.f;

            displayRMS        = voices[displayVoice].dynEnvOut;
            displayRMSR       = processR ? voicesR[displayVoice].dynEnvOut * (droneEffective ? 0.6f : 1.0f) : 0.f;
            displayAir        = voices[displayVoice].a_noise;
            displayEdge       = voices[displayVoice].a_reedMorph;
            displaySaturation = clamp(voices[displayVoice].safetyRMS - 0.6f, 0.f, 1.f);
            displaySaturationR = processR ? clamp(voicesR[displayVoice].safetyRMS - 0.6f, 0.f, 1.f) : 0.f;
            {
                float chiffDur = voices[displayVoice].cachedChiffDuration;
                float chiffCnt = (float)voices[displayVoice].chiffCounter;
                displayChiff   = (chiffDur > 0.f)
                               ? clamp(1.f - chiffCnt / chiffDur, 0.f, 1.f)
                               : 0.f;
            }
            {
                float chiffDurR = processR ? voicesR[displayVoice].cachedChiffDuration : 0.f;
                float chiffCntR = processR ? (float)voicesR[displayVoice].chiffCounter  : 0.f;
                displayChiffR   = (chiffDurR > 0.f)
                                ? clamp(1.f - chiffCntR / chiffDurR, 0.f, 1.f)
                                : 0.f;
            }

            // R tube width: derived directly from the aulosOffset slider + CV so
            // it updates immediately regardless of whether either voice is active.
            // aulosOffset in octaves -> frequency ratio = 2^aulosOffset, so the
            // R pipe is longer/shorter by exactly that factor relative to L.
            displayPipeRatio = clamp(rack::dsp::exp2_taylor5(-aulosOffset), 0.25f, 2.0f);
        }

        for (int seg = 0; seg < 10; ++seg) {
            vuBreath[seg] = (displayBreath * 10.f > (float)seg) ? 1.f : 0.f;
            float exciteLevel = (nVoices > 0 && displayVoice < nVoices)
                              ? voices[displayVoice].dynEnvOut : 0.f;
            vuExcite[seg] = (exciteLevel * 10.f > (float)seg) ? 1.f : 0.f;
        }

        lights[MANUAL_GATE_LIGHT].setBrightness(manualGateActive ? 1.f : 0.f);
        lights[DRONE_LIGHT      ].setBrightness(droneEffective   ? 1.f : 0.f);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// PipeDisplay - two tubes stacked vertically.
// L (top): always half the widget width, left-aligned.
// R (bottom): width scaled by pipeFreqL/pipeFreqR, clamped 25%..100%, left-aligned.
// This makes the relative tube lengths visually apparent.
// ─────────────────────────────────────────────────────────────────────────────
struct PipeDisplay : TransparentWidget {
    Aulos* module = nullptr;

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer == 1) {

            // Self-illuminated display background
            nvgBeginPath(args.vg);
            nvgRoundedRect(args.vg, 0, 0, box.size.x, box.size.y, 4.f);
            nvgFillColor(args.vg, nvgRGBA(12, 12, 20, 220));
            nvgFill(args.vg);

            const float W    = box.size.x;
            const float H    = box.size.y;
            const float gap  = 2.f;
            const float rowH = (H - gap) * 0.5f;

            if (!module) {
                drawTube(args.vg, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
                         3.f, 0.f, W * 0.5f, rowH);
                drawTube(args.vg, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
                         3.f, rowH + gap, W, rowH);
            }
            else {
                const float airReserve = rowH * 0.7f;

                float lWidth = W * 0.5f - airReserve;
                // Reserve right-edge space for air lines — they extend past the
                // bell exit, so cap rWidth so the longest line stays in bounds.
                // airReserve scales with rowH since line length is proportional
                // to bellH which is proportional to rowH.
                float rWidth = clamp(
                    module->displayPipeRatio * 0.5f,
                    0.25f,
                    1.0f
                ) * W - airReserve;

                drawTube(args.vg,
                         module->displayActiveFraction,
                         module->displayBreath,
                         module->displayOverblow,
                         module->displayRegister,
                         module->displayRMS,
                         module->displayAir,
                         module->displayChiff,
                         module->displayEdge,
                         module->displaySaturation,
                         3.f, 0.f,
                         lWidth, rowH);

                drawTube(args.vg,
                         module->displayActiveFractionR,
                         module->displayBreathR,
                         module->displayOverblowR,
                         module->displayRegisterR,
                         module->displayRMSR,
                         module->displayAir,
                         module->displayChiffR,
                         module->displayEdge,
                         module->displaySaturationR,
                         3.f, rowH + gap,
                         rWidth, rowH);
            }
        }

        TransparentWidget::drawLayer(args, layer);
    }

    // bx, by: top-left of the bounding rect for this tube.
    // bw, bh: width and height of the bounding rect.
    // activeFraction: position of the first open finger hole along the tube.
    // breath: breath envelope 0..1, controls tube opacity.
    // overblow: safetyDecay 0..1, triggers overblow outline.
    // registerValue: smoothed register 0..2, drives standing wave mode count.
    // rms: dynEnvOut per-voice, drives standing wave brightness floor.
    // air: a_noise 0..1, drives air lines at bell exit.
    // chiff: chiff envelope 0..1, activates outer air lines.
    // edge: a_reedMorph 0..1, shifts standing wave color toward warm amber.
    // saturation: pre-overblow safetyRMS indicator 0..1, warms tube color early.
    void drawTube(NVGcontext* vg,
          float activeFraction, float breath, float overblow,
          float registerValue,
          float rms, float air, float chiff, float edge, float saturation,
          float bx, float by, float bw, float bh) {

        const float margin  = 3.f;
        const float tubeX   = bx + margin;
        const float tubeW   = bw - margin * 2.f;
        const float centerY = by + bh * 0.5f;

        // Narrower tube — better aspect ratio.
        const float baseH = bh * 0.28f;
        auto tubeHalfHeight = [&](float xNorm) {
            return baseH * (0.88f + 0.12f * xNorm);
        };

        const int CURVE_STEPS = 64;

        // ── Tube fill ─────────────────────────────────────────────────────────
        float tubeAlpha = 0.4f + breath * 0.5f;
        nvgBeginPath(vg);
        for (int i = 0; i <= CURVE_STEPS; ++i) {
            float xNorm = (float)i / (float)CURVE_STEPS;
            float x = tubeX + xNorm * tubeW;
            float y = centerY - tubeHalfHeight(xNorm);
            if (i == 0) nvgMoveTo(vg, x, y); else nvgLineTo(vg, x, y);
        }
        for (int i = CURVE_STEPS; i >= 0; --i) {
            float xNorm = (float)i / (float)CURVE_STEPS;
            float x = tubeX + xNorm * tubeW;
            float y = centerY + tubeHalfHeight(xNorm);
            nvgLineTo(vg, x, y);
        }
        nvgClosePath(vg);
        // Saturation warms the fill color slightly before full overblow.
        nvgFillColor(vg, nvgRGBAf(0.06f + saturation * 0.08f,
                                   0.06f,
                                   0.12f,
                                   tubeAlpha));
        nvgFill(vg);

        // ── Standing wave ─────────────────────────────────────────────────────
        float modeNumber = rack::dsp::exp2_taylor5(registerValue);
        float energyKick = clamp((overblow - 0.35f) / 0.3f, 0.f, 1.f);
        modeNumber      *= (1.f + energyKick);

        float boreEnd = clamp(activeFraction, 0.05f, 1.f);

        // RMS drives the brightness floor — tube glows even at low breath.
        float brightness = 0.15f + 0.85f * fmaxf(rms * 4.f, breath);

        const int N = 64;
        for (int i = 0; i < N; ++i) {
            float xNorm = (float)i / (float)(N - 1);
            float xPix  = tubeX + xNorm * tubeW;
            float segW  = tubeW / (float)N + 1.f;
            float halfH = tubeHalfHeight(xNorm);

            float xDist    = xNorm - boreEnd;
            float leakGain = (xDist <= 0.f) ? 1.f : expf(-xDist * 6.5f);

            float phase    = modeNumber * float(M_PI) * xNorm / boreEnd;
            float pressure = aulosDspSin(phase);
            float amp      = pressure * pressure * leakGain;
            float glow     = amp * brightness;

            // Base warm/cool color scheme.
            float warm = fmaxf(pressure, 0.f);
            float cool = fmaxf(-pressure, 0.f);

            // Edge shifts color toward amber as reedMorph increases.
            // Saturation warms the color before overblow kicks in.
            float r = 0.08f + warm * (0.85f + edge * 0.15f)
                            + saturation * warm * 0.20f;
            float g = 0.05f + warm * (0.30f + edge * 0.20f)
                            + saturation * warm * 0.10f;
            float b = 0.12f + cool * (0.90f - edge * 0.30f);

            float a = (0.35f + glow * 0.65f) * leakGain;
            nvgBeginPath(vg);
            nvgRect(vg, xPix - segW * 0.5f, centerY - halfH, segW, halfH * 2.f);
            nvgFillColor(vg, nvgRGBAf(r, g, b, a));
            nvgFill(vg);
        }

        // ── Embouchure dot ────────────────────────────────────────────────────
        // Glowing dot at the mouthpiece end, pulsing with RMS output level.
        if (breath > 0.01f || rms > 0.01f) {
            float dotX   = tubeX;
            float dotR   = tubeHalfHeight(0.f) * (0.25f + rms * 0.35f);
            float dotGlow = fmaxf(rms * 3.f, breath);
            nvgBeginPath(vg);
            nvgCircle(vg, dotX, centerY, dotR);
            nvgFillColor(vg, nvgRGBAf(0.9f + edge * 0.1f,
                                       0.85f - edge * 0.15f,
                                       0.7f  - edge * 0.3f,
                                       dotGlow * 0.85f));
            nvgFill(vg);
        }

        // ── Air lines ─────────────────────────────────────────────────────────
        // Field-line style bezier curves at bell exit. Middle line is longest,
        // outer lines shorter. Gentle outward curve — not radial. 9 lines total:
        // inner 5 driven by air, outer 4 only active during chiff. Flutter with time.
        {
            float bellX    = tubeX + tubeW;
            float bellH    = tubeHalfHeight(1.f);
            float frameTime = (float)glfwGetTime();
            const int NUM_LINES = 9;

            for (int i = 0; i < NUM_LINES; ++i) {
                // t: 0=center, 1=outermost. Use abs distance from center line.
                float centerIdx = (NUM_LINES - 1) * 0.5f;
                float tAbs = fabsf((float)i - centerIdx) / centerIdx;  // 0 at center, 1 at edge

                // Inner lines driven by air, outer lines need chiff.
                float lineActive;
                if (i >= 1 && i <= (NUM_LINES - 2)) {
                    // All but the outermost pair: air drives them.
                    lineActive = air * breath;
                } else {
                    // Outermost pair: chiff only, with faint air sustain.
                    lineActive = fmaxf(chiff * 0.9f, air * breath * 0.15f);
                }
                if (lineActive < 0.02f) continue;

                // Vertical position at bell — symmetric above/below center.
                float ySign   = ((float)i < centerIdx) ? -1.f : (i > centerIdx ? 1.f : 0.f);
                float yOffset = bellH * tAbs * 0.95f * ySign;

                // Length: middle (tAbs=0) is longest, outer (tAbs=1) shorter.
                float baseLen  = bellH * (1.4f + air * 0.8f + chiff * 1.0f);
                float lineLen  = baseLen * (1.f - tAbs * 0.6f);

                // Flutter: small oscillation on control and end points.
                float flutter  = aulosDspSin(frameTime * (2.5f + i * 1.3f)) * bellH * 0.05f * lineActive;

                // Gentle bezier: mostly rightward with slight outward curve.
                // Control point only modestly offset to keep curve subtle.
                float ctrlX = bellX + lineLen * 0.5f;
                float ctrlY = centerY + yOffset * 1.15f + flutter;
                float endX  = bellX + lineLen;
                float endY  = centerY + yOffset * 1.35f + flutter * 1.5f;

                float alpha = lineActive * (0.9f - tAbs * 0.3f);
                alpha = clamp(alpha, 0.f, 1.f);

                // Color: blue, brightens during chiff.
                float cr = 0.25f + chiff * 0.55f;
                float cg = 0.55f + chiff * 0.35f;
                float cb = 1.0f;
                float sw = 1.5f + (1.f - tAbs) * 0.8f + chiff * 1.0f;  // center thicker

                nvgBeginPath(vg);
                nvgMoveTo(vg, bellX, centerY + yOffset);
                nvgQuadTo(vg, ctrlX, ctrlY, endX, endY);
                nvgStrokeColor(vg, nvgRGBAf(cr, cg, cb, alpha));
                nvgStrokeWidth(vg, sw);
                nvgStroke(vg);
            }
        }

        // ── Fingering marker ──────────────────────────────────────────────────
        float markerNorm = boreEnd;
        if (breath > 0.02f) {
            float markerX = tubeX + markerNorm * tubeW;
            float markerH = tubeHalfHeight(markerNorm);
            float alpha   = 0.5f + breath * 0.5f;
            if (overblow < 0.05f) {
                nvgBeginPath(vg);
                nvgMoveTo(vg, markerX, centerY - markerH - 1.f);
                nvgLineTo(vg, markerX, centerY + markerH + 1.f);
                nvgStrokeColor(vg, nvgRGBAf(1.f, 0.9f, 0.5f, alpha));
                nvgStrokeWidth(vg, 1.5f);
                nvgStroke(vg);
            } else {
                float ellipseW = 2.5f + overblow * 7.f;
                float ellipseH = markerH * (0.35f + overblow * 0.45f);
                nvgBeginPath(vg);
                nvgEllipse(vg, markerX, centerY, ellipseW, ellipseH);
                nvgStrokeColor(vg, nvgRGBAf(1.f, 0.9f, 0.5f, alpha));
                nvgStrokeWidth(vg, 1.5f);
                nvgStroke(vg);
            }
        }

        // ── Overblow outline ──────────────────────────────────────────────────
        if (overblow > 0.02f) {
            nvgBeginPath(vg);
            for (int i = 0; i <= CURVE_STEPS; ++i) {
                float xNorm = (float)i / (float)CURVE_STEPS;
                float x = tubeX + xNorm * tubeW;
                float y = centerY - tubeHalfHeight(xNorm) - 1.5f;
                if (i == 0) nvgMoveTo(vg, x, y); else nvgLineTo(vg, x, y);
            }
            for (int i = CURVE_STEPS; i >= 0; --i) {
                float xNorm = (float)i / (float)CURVE_STEPS;
                float x = tubeX + xNorm * tubeW;
                float y = centerY + tubeHalfHeight(xNorm) + 1.5f;
                nvgLineTo(vg, x, y);
            }
            nvgClosePath(vg);
            nvgStrokeColor(vg, nvgRGBAf(1.f, 0.85f, 0.3f, overblow * 0.9f));
            nvgStrokeWidth(vg, 1.5f);
            nvgStroke(vg);
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// AulosSlider
// ─────────────────────────────────────────────────────────────────────────────
struct AulosSliderBase : app::SvgSlider {
    AulosSliderBase() {
        setBackgroundSvg(Svg::load(asset::plugin(pluginInstance,
            "res/components/ShortSlider.svg")));
        setHandleSvg(Svg::load(asset::plugin(pluginInstance,
            "res/components/ShortSliderHandle.svg")));
        setHandlePosCentered(math::Vec(10.f, 55.f), math::Vec(10.f, 10.f));
    }
};
template <typename TL = YellowLight>
struct AulosSlider : LightSlider<AulosSliderBase, VCVSliderLight<TL>> {
    AulosSlider() {}
};

// ─────────────────────────────────────────────────────────────────────────────
// AulosWidget - current 20HP panel layout
// ─────────────────────────────────────────────────────────────────────────────
struct AulosWidget : ModuleWidget {

    static Vec p(float x, float y) { return mm2px(Vec(x, y)); }

    AulosWidget(Aulos* module) {
        setModule(module);
        box.size = Vec(20 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT);

        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Aulos.svg"),
            asset::plugin(pluginInstance, "res/Aulos-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2*RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2*RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        const float panelW = 20.f * 5.08f;
        const float cx     = panelW * 0.5f;

        // ── Pipe display ──────────────────────────────────────────────────────
        {
            auto* disp     = createWidget<PipeDisplay>(mm2px(Vec(7.f, 10.f)));
            disp->box.size = mm2px(Vec(panelW - 14.f, 21.0f));

            disp->module   = module;
            addChild(disp);
        }

        // ── Slider bank - 8 sliders ───────────────────────────────────────────
        const float sxBase  = 10.f;
        const float sPitch  = 11.5f;
        const float ySlider = 45.5f;
        const float ySlTrim = 57.f;
        const float ySlCV   = 65.f;

        struct SlSpec { Aulos::ParamId param, att; Aulos::InputId cv; int color; };
        const SlSpec specs[8] = {
            {Aulos::EDGE_PARAM,  Aulos::EDGE_ATT,  Aulos::EDGE_CV_INPUT,  1},  // blue
            {Aulos::JET_PARAM,  Aulos::JET_ATT,  Aulos::JET_CV_INPUT,  1},  // blue
            {Aulos::DRIVE_PARAM,  Aulos::DRIVE_ATT,  Aulos::DRIVE_CV_INPUT,  2},  // white
            {Aulos::RES_PARAM,   Aulos::RES_ATT,   Aulos::RES_CV_INPUT,   3},  // yellow
            {Aulos::AIR_PARAM, Aulos::AIR_ATT, Aulos::AIR_CV_INPUT, 3},  // yellow
            {Aulos::CHIFF_PARAM, Aulos::CHIFF_ATT, Aulos::CHIFF_CV_INPUT, 3},  // yellow
            {Aulos::AULOS_PARAM, Aulos::AULOS_ATT, Aulos::AULOS_CV_INPUT, 2},  // white
            {Aulos::WARMTH_PARAM,  Aulos::WARMTH_ATT,  Aulos::WARMTH_CV_INPUT,  4},  // red
        };
        for (int i = 0; i < 8; ++i) {
            const SlSpec& s = specs[i];
            float x = sxBase + i * sPitch;
            if (s.color == 1)
                addParam(createParamCentered<AulosSlider<BlueLight>>(  p(x, ySlider), module, s.param));
            else if (s.color == 2)
                addParam(createParamCentered<AulosSlider<WhiteLight>>( p(x, ySlider), module, s.param));
            else if (s.color == 3)
                addParam(createParamCentered<AulosSlider<YellowLight>>(p(x, ySlider), module, s.param));
            else
                addParam(createParamCentered<AulosSlider<RedLight>>(   p(x, ySlider), module, s.param));
            addParam(createParamCentered<Trimpot>(          p(x, ySlTrim), module, s.att));
            addInput(createInputCentered<ThemedPJ301MPort>( p(x, ySlCV),   module, s.cv));
        }

        // ── Bottom rows ───────────────────────────────────────────────────────
        const float yRow0 = 80.f;
        const float yRow1 = 92.f;
        const float yRow2 = 104.f;
        const float yRow3 = 116.f;

        // Row 0: Pitch
        addInput(createInputCentered<ThemedPJ301MPort>(    p(sxBase, yRow0), module, Aulos::PIPE_VOCT_INPUT));
        addParam(createParamCentered<RoundBlackKnob>(      p(sxBase+sPitch, yRow0), module, Aulos::PIPE_TUNE_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(    p(sxBase+2*sPitch,  yRow0), module, Aulos::FINGER_VOCT_INPUT));
        addParam(createParamCentered<RoundBlackKnob>(      p(sxBase+3*sPitch,  yRow0), module, Aulos::FINGER_TUNE_PARAM));

        // Row 1: Gate + Drone + Audio In jack + Audio In CV
        addParam(createParamCentered<TL1105>(              p(sxBase, yRow1), module, Aulos::MANUAL_GATE_BTN));
        addChild(createLightCentered<MediumLight<GreenLight>>(p(sxBase, yRow1), module, Aulos::MANUAL_GATE_LIGHT));
        addInput(createInputCentered<ThemedPJ301MPort>(    p(sxBase+sPitch, yRow1), module, Aulos::GATE_INPUT));

        addInput(createInputCentered<ThemedPJ301MPort>(    p(sxBase+2*sPitch, yRow1), module, Aulos::VIBRATO_INPUT));
        addParam(createParamCentered<RoundSmallBlackKnob>( p(sxBase+3*sPitch, yRow1), module, Aulos::VIBRATO_ATT));

        addInput(createInputCentered<ThemedPJ301MPort>(    p(cx+8.f, yRow1-6.f), module, Aulos::BREATH_CV_INPUT));
        addParam(createParamCentered<Trimpot>(             p(cx+17.f, yRow1-6.f), module, Aulos::BREATH_ATT));
        addParam(createParamCentered<RoundHugeBlackKnob>( p(cx+34.f, yRow1-6.f), module, Aulos::BREATH_PARAM));

        // Row 2: Breath + Audio In gain (trim + knob)

        addParam(createParamCentered<TL1105>(              p(sxBase,  yRow2), module, Aulos::DRONE_BTN));
        addChild(createLightCentered<MediumLight<YellowLight>>(p(sxBase, yRow2), module, Aulos::DRONE_LIGHT));
        addInput(createInputCentered<ThemedPJ301MPort>(    p(sxBase+sPitch,  yRow2), module, Aulos::DRONE_CV_INPUT));

        addInput(createInputCentered<ThemedPJ301MPort>(    p(sxBase+3*sPitch, yRow2), module, Aulos::AUDIO_IN_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(    p(cx+8.f, yRow2), module, Aulos::AUDIO_IN_CV_INPUT));
        addParam(createParamCentered<Trimpot>(             p(cx+17.f, yRow2), module, Aulos::AUDIO_IN_GAIN_ATT));
        addParam(createParamCentered<RoundSmallBlackKnob>( p(cx+26.f, yRow2), module, Aulos::AUDIO_IN_GAIN_PARAM));

        // Row 3: VU meters, outputs, volume
        for (int i = 0; i < 5; ++i) {
            addChild(createLightCentered<SmallLight<BlueLight>>(
                p(cx - 36.f - 2.f * i, yRow3 + 1.f), module, Aulos::BREATH_LIGHT_0 + i * 2));
            addChild(createLightCentered<SmallLight<BlueLight>>(
                p(cx - 37.f - 2.f * i, yRow3 - 1.f), module, Aulos::BREATH_LIGHT_0 + i * 2 + 1));
        }
        addOutput(createOutputCentered<ThemedPJ301MPort>(  p(10.f+1*11.5f, yRow3), module, Aulos::ENV_OUTPUT));

        for (int i = 0; i < 5; ++i) {
            addChild(createLightCentered<SmallLight<YellowLight>>(
                p(cx - 13.f - 2.f * i, yRow3 + 1.f), module, Aulos::EXCITE_LIGHT_0 + i * 2));
            addChild(createLightCentered<SmallLight<YellowLight>>(
                p(cx - 14.f - 2.f * i, yRow3 - 1.f), module, Aulos::EXCITE_LIGHT_0 + i * 2 + 1));
        }
        addOutput(createOutputCentered<ThemedPJ301MPort>(  p(10.f+3*11.5f,  yRow3), module, Aulos::RMS_OUTPUT));

        addInput(createInputCentered<ThemedPJ301MPort>(    p(cx+8.f,  yRow3), module, Aulos::VOLUME_CV_INPUT));
        addParam(createParamCentered<Trimpot>(             p(cx+17.f, yRow3), module, Aulos::VOLUME_ATT));
        addParam(createParamCentered<RoundSmallBlackKnob>( p(cx+26.f, yRow3), module, Aulos::VOLUME_PARAM));
        addOutput(createOutputCentered<ThemedPJ301MPort>(  p(cx+40.f, yRow2), module, Aulos::AUDIO_L_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(  p(cx+40.f, yRow3), module, Aulos::AUDIO_R_OUTPUT));
    }

    void step() override {
        ModuleWidget::step();
        Aulos* m = dynamic_cast<Aulos*>(module);
        if (!m) return;

        float dt           = APP->window->getLastFrameDuration();
        const float lambda = 18.f;

        for (int seg = 0; seg < 10; ++seg) {
            m->lights[Aulos::BREATH_LIGHT_0 + seg].setBrightnessSmooth(m->vuBreath[seg], dt, lambda);
            m->lights[Aulos::EXCITE_LIGHT_0 + seg].setBrightnessSmooth(m->vuExcite[seg], dt, lambda);
        }
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
        Aulos* m = dynamic_cast<Aulos*>(module);
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
                return string::f("%.3f", val ? *val : def); }
        };
        auto addFSlider = [&](float* v, float lo, float hi, float def, std::string lbl) {
            auto* sl = new ui::Slider();
            sl->quantity   = new FloatQ(v, lo, hi, def, lbl);
            sl->box.size.x = 200.f;
            menu->addChild(sl);
        };

        menu->addChild(new MenuSeparator());
        menu->addChild(createMenuLabel("Aulos R Voice Mode"));
        struct TrackItem : MenuItem {
            Aulos* module;
            TrackItem(Aulos* m) : module(m) { text = "Tracking (interval preserved)"; }
            void onAction(const ActionEvent&) override { if (module) module->aulosTrack = true; }
        };
        struct TwoPipeItem : MenuItem {
            Aulos* module;
            TwoPipeItem(Aulos* m) : module(m) { text = "Two Pipes (fixed registers)"; }
            void onAction(const ActionEvent&) override { if (module) module->aulosTrack = false; }
        };
        auto* trackItem   = new TrackItem(m);
        auto* twoPipeItem = new TwoPipeItem(m);
        trackItem->rightText   = m->aulosTrack  ? CHECKMARK_STRING : "";
        twoPipeItem->rightText = !m->aulosTrack ? CHECKMARK_STRING : "";
        menu->addChild(trackItem);
        menu->addChild(twoPipeItem);

        menu->addChild(new MenuSeparator());
        menu->addChild(createMenuLabel("Resonator"));
        addFSlider(&m->waveguideGain, 0.f, 1.75f,  1.0f, "Waveguide Gain");
        addFSlider(&m->decayValue,    0.f, 1.f,  0.7f, "Decay (ring-off time)");

        menu->addChild(new MenuSeparator());
        menu->addChild(createMenuLabel("Envelope"));
        addFSlider(&m->attackValue,   0.f, 1.f,  0.3f, "Attack");
        addFSlider(&m->releaseValue,  0.f, 1.f,  0.5f, "Release");

        menu->addChild(createMenuLabel("Envelope Curves"));
        addFSlider(&m->attackCurve,  -1.f, 1.f,  0.3f, "Attack Curve (log - exp)");
        addFSlider(&m->releaseCurve, -1.f, 1.f, -0.5f, "Release Curve (log - exp)");

        menu->addChild(new MenuSeparator());
        menu->addChild(createMenuLabel("Vibrato"));
        addFSlider(&m->vibratoRate,        3.f,  12.f, 7.0f, "Rate (Hz)");
        addFSlider(&m->vibratoBreathDepth, 0.f,   1.f, 0.5f, "Breath Coupling");

        menu->addChild(new MenuSeparator());
        menu->addChild(new MenuSeparator());
        menu->addChild(createMenuLabel("Articulation"));
        struct LegatoItem : MenuItem {
            Aulos* module;
            LegatoItem(Aulos* m) : module(m) { text = "Legato"; }
            void onAction(const ActionEvent&) override {
                if (module) module->legatoEnabled = !module->legatoEnabled;
            }
        };
        auto* legatoItem = new LegatoItem(m);
        legatoItem->rightText = m->legatoEnabled ? CHECKMARK_STRING : "";
        menu->addChild(legatoItem);
        addFSlider(&m->legatoTime, 5.f, 80.f, 20.f, "Legato Glide Time (ms)");

        menu->addChild(new MenuSeparator());
        menu->addChild(createMenuLabel("Dynamics Envelope"));
        addFSlider(&m->followTime,    0.f, 1.f,  0.25f, "Follow Time");

        menu->addChild(new MenuSeparator());
        struct PanicItem : MenuItem {
            Aulos* module;
            PanicItem(Aulos* m) : module(m) { text = "Panic: Clear All Energy"; }
            void onAction(const ActionEvent&) override { if (module) module->panic(); }
        };
        menu->addChild(new PanicItem(m));
    }
};
Model* modelAulos = createModel<Aulos, AulosWidget>("Aulos");