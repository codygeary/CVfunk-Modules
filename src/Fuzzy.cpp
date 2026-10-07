////////////////////////////////////////////////////////////
//
//   Fuzzy
//
//   written by Cody Geary
//   Copyright 2026, MIT License
//
//   Eight classic fuzz and overdrive circuits, each simulated from its
//   schematic: five germanium designs from the 1960s, two Japanese silicon
//   octave fuzzes and a British silicon overdriver.
//
////////////////////////////////////////////////////////////



#include "rack.hpp"
#include "plugin.hpp"
#include "FilterFuzzy.h"
using namespace rack;
using simd::float_4;

#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstring>

// ---- Models -------------------------------------------------------------------
static const int kModelCount = 8;
struct ModelInfo {
    const char* label;          // display, at most 10 characters
    const char* name;           // tooltip: what it is after
    const char* detail;         // display, under the name: what is simulated
    double outputHighPassHz;    // the circuit's output capacitor into its level pot
    double outputGain[3];       // level match at FUZZ 0.3, 0.7, 1.0 (held below 0.3)
    double polarity;            // +1 if the circuit's output follows its input, -1 if it inverts
};
//   This pedal's controls: FUZZ (each circuit's own gain or bias pot, mapped
//   over its sweet spot), VOLUME, and GUITAR, the guitar's volume knob in
//   front of the circuit, which is half of how these pedals are played.
//   BATTERY is the battery's charge: turned down, the supply sags and the
//   junctions warm until the circuit sputters and dies. The footswitch
//   fades between the circuit and the clean input.
//
//   The input is levelled in the back end so any source arrives at the
//   level of a hot single-coil pickup; GUITAR then cleans up the same way
//   whatever is driving it. Mono in, mono out, 4x oversampled.


// Output gains are measured for equal loudness (K-weighted, BS.1770 style)
// across a guitar riff, a lead line and a synth saw: every model averages
// the same, and half of each model's own rise with FUZZ is kept, so the
// knob still adds a little level. Hand-tunable.
static const ModelInfo kModels[kModelCount] = {
    { "ROUNDHOUSE", "Roundhouse: after the Arbiter Fuzz Face (UK 1966)",
      "2x Ge PNP, shunt feedback", 32.0, { 2.42,  2.29,  2.24  },  1.0 },
    { "RASP", "Rasp: after the Sola Sound Tone Bender MK1 (UK 1965)",
      "3x Ge PNP, follower + 2 CE", 26.0, { 0.125, 0.122, 0.121 },  1.0 },
    { "SUSTAINER", "Sustainer: after the Sola Sound Tone Bender MK2 (UK 1966)",
      "3x Ge PNP, boost + fb pair", 160.0, { 2.49,  2.46,  2.45  }, -1.0 },
    { "SPLUTTER", "Splutter: after the Vox / JEN Tone Bender (Italy 1966)",
      "2x Ge PNP, Q2 near cutoff", 49.0, { 3.00,  2.64,  2.12  },  1.0 },
    { "BUZZSAW", "Buzzsaw: after the Maestro FZ-1 Fuzz-Tone (US 1962)",
      "3x Ge PNP on a 3 V cell", 95.0, { 0.384, 0.378, 0.380 },  1.0 },
    { "KAIJU", "Kaiju: after the Univox Super-Fuzz, scooped (Japan 1968)",
      "5x Si NPN octave, 2x Ge D", 20.0, { 2.86, 2.67, 2.59 }, 1.0 },
    { "HORNET", "Hornet: after the Ibanez Standard Fuzz, flat (Japan 1970s)",
      "JFET + 3x Si octave, Ge D", 20.0, { 3.95,  3.69,  3.64  },  1.0 },
    { "KALEIDO", "Kaleido: after the Colorsound Overdriver (UK 1970s)",
      "3x Si NPN, fb pair + EQ", 30.0, { 0.318, 0.262, 0.254 }, -1.0 },
};

// ---- Input leveller, hand-tunable -----------------------------------------------
// The input's peak is tracked (fast attack, slow release) and scaled to this
// source EMF, the peak of a hot single-coil pickup.
static const float kTargetPeakVolts = 0.15f;
// The peak follower rises over kLevelAttackSec (short enough that a note
// after a pause is not blasted through at the quiet gain for long, long
// enough to ride over the pick spike) and falls over kLevelReleaseSec. The
// gain comes down within kLevelGainDropSec and back up with the follower.
static const float kLevelAttackSec   = 0.003f;
static const float kLevelGainDropSec = 0.001f;
static const float kLevelReleaseSec = 3.0f;
// Below this input peak (jack volts) the gain stops rising, so silence and
// hum are not lifted to playing level.
static const float kLevelFloorVolts = 0.1f;

// ---- Battery, hand-tunable -----------------------------------------------------------
// Smoothing of the BATTERY knob and CV, seconds. Faster sweeps push a larger
// sub-audio bias swing out of the circuit as it settles.
static const float kBatterySmoothSec = 0.15f;

// ---- Footswitch, hand-tunable -----------------------------------------------------------
// Off, the output is the levelled input, delayed by the resampler's latency
// so it lines up with the circuit, at output volts per input EMF volt that
// sit 3 dB under the fuzz at VOLUME 0.7 (a clean guitar's pick peaks run far
// above a fuzz's). The switch fades over kSwitchFadeSec.
static const int   kDryDelaySamples = 15;
static const float kDryGain = 28.4f;
static const float kSwitchFadeSec = 0.02f;

// ---- Device view, hand-tunable -----------------------------------------------------
// Each simulated junction is read once per base sample; per control block
// its peak and lowest current against its resting current set the glow, and
// the share of samples with the base-collector junction conducting sets the
// saturation tint. The view falls back over kDeviceFallSec. Diodes glow from
// kDiodeGlowFloorAmps (dark) to 1000x that (full).
static const float kDeviceFallSec = 0.15f;
static const float kDiodeGlowFloorAmps = 1e-6f;
// A transistor counts as saturated while its base-collector junction carries
// more than this share of the base-emitter current.
static const float kSaturatedShare = 0.5f;
// Transistor glow: at rest, and the rise per decade of peak current.
static const float kGlowAtRest = 0.3f, kGlowPerDecade = 0.3f;

// ---- Level meter, hand-tunable ------------------------------------------------------
// RMS ballistics (VU-like) and the 0 dB mark, output volts rms.
static const float kMeterSec = 0.3f;
static const float kMeterZeroVolts = 2.f;

// ---- Output DC blocker, hand-tunable --------------------------------------------------
// Second-order high-pass on the output. Sweeping BATTERY moves every bias
// point; a one-pole output capacitor lets that through as a slow thump of
// several volts, a second order settles it in a few cycles.
static const float kDcBlockHz = 20.f;

// ---- Output limiter, hand-tunable ------------------------------------------------------
// Linear up to the knee, then rounding off toward the ceiling (volts).
static const float kLimitKnee = 7.f, kLimitCeiling = 10.f;

// ---- Model switching, hand-tunable ---------------------------------------------------
// A new selection must hold this long before the switch, a switch is at least
// this far from the last, and the old and new circuits crossfade over this.
static const float kModelSettleSec  = 0.03f;
static const float kModelMinHoldSec = 0.1f;
static const float kCrossfadeSec    = 0.015f;

// ---- Noise, hand-tunable --------------------------------------------------------------
// Base noise current of the first gain stage, as a density: shot noise of
// ~10 uA of base current with a germanium flicker corner. Scaled per model.
static const double kNoiseWhiteAmps = 1.8e-12;    // A / sqrt(Hz)
static const double kNoiseFlickerCornerHz = 3000.0;

// Output volts per circuit volt before VOLUME and the model's level match.
static const float kOutScale = 10.f;
// Knob smoothing, seconds, and the control-rate block, base samples. The
// block matches Core::kHeldGlideSamples, so rail changes glide across it.
static const float kSmoothTau = 0.010f;
static const int   kControlBlock = 32;


struct Fuzzy : Module {

    enum ParamIds {
        BATTERY_PARAM, BATTERY_TRIM_PARAM,
        FUZZ_PARAM, FUZZ_TRIM_PARAM,
        GUITAR_PARAM, GUITAR_TRIM_PARAM,
        MODEL_PARAM,
        VOLUME_PARAM,
        SWITCH_PARAM,
        NUM_PARAMS
    };
    enum InputIds { IN_INPUT, BATTERY_CV_INPUT, FUZZ_CV_INPUT, GUITAR_CV_INPUT, MODEL_CV_INPUT, SWITCH_CV_INPUT, VOLUME_CV_INPUT, NUM_INPUTS };
    enum OutputIds { OUT_OUTPUT, NUM_OUTPUTS };
    enum LightIds { SWITCH_LIGHT, NUM_LIGHTS };

    fuzzy::FuzzModel* models[kModelCount] = {};
    bool  modelStarted[kModelCount] = {};
    // One upsampler per model (a model may run a front stage before it),
    // one downsampler for the mix.
    fuzzy::Resampler upsampler[kModelCount];
    fuzzy::Resampler resampler;
    fuzzy::NoiseSource noiseWhite, noisePink;

    // ---- Model switching ---------------------------------------------------------
    int   activeModel = 0;
    int   fadingModel = -1;               // the model being faded out, or -1
    float fadePosition = 1.f;             // 0 -> 1 across the crossfade
    int   requestedModel = 0;
    int   requestHeld = 0;                // samples the request has been stable
    int   sinceSwitch = 1 << 20;          // samples since the last switch

    // ---- Control-rate state -------------------------------------------------------
    int   controlCounter = 0;
    bool  smoothedValid = false;
    float smoothFuzz = 0.7f, smoothGuitar = 1.f, smoothCharge = 1.f;
    float drain = 0.f;
    double noiseAmplitude = 0.0;
    float appliedSampleRate = 0.f;

    // ---- Audio-rate state ---------------------------------------------------------
    float levelEnvelope = 1.f, levelGain = 0.f;
    float levelAttackK = 0.f, levelReleaseK = 0.f, levelDropK = 0.f;
    float dryLine[kDryDelaySamples] = {};
    int   dryIndex = 0;
    // DC blocker: biquad coefficients and state (transposed direct form II).
    float blockB0 = 1.f, blockB1 = 0.f, blockB2 = 0.f, blockA1 = 0.f, blockA2 = 0.f;
    float blockState1 = 0.f, blockState2 = 0.f;
    float highPassState[kModelCount] = {}, highPassInput[kModelCount] = {};
    float highPassCoeff[kModelCount] = {};
    float modelGain[kModelCount] = {};    // level match times polarity, at the current FUZZ

    // ---- Footswitch ----------------------------------------------------------------
    dsp::SchmittTrigger switchTrigger;
    float switchFade = 1.f;               // 0 clean .. 1 circuit
    bool  circuitsIdle = false;           // fully switched off: no circuit runs

    // ---- Display -------------------------------------------------------------------
    // Device view: resting currents per model, this block's extremes, and
    // what the display reads (0..1).
    float deviceRest[kModelCount][fuzzy::kMaxDevices] = {};
    float devicePeak[fuzzy::kMaxDevices] = {}, deviceLow[fuzzy::kMaxDevices] = {};
    int   deviceSaturated[fuzzy::kMaxDevices] = {};
    int   senseSamples = 0;
    float deviceGlow[fuzzy::kMaxDevices] = {}, deviceSaturation[fuzzy::kMaxDevices] = {}, deviceCutoff[fuzzy::kMaxDevices] = {};
    float meterPower = 0.f, meterK = 0.f;
    float meterPeak = 0.f, meterPeakDecay = 0.f;   // peak, volts, falling 20 dB per second

    Fuzzy() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
        std::vector<std::string> names;
        for (int k = 0; k < kModelCount; ++k) names.push_back(kModels[k].name);
        configParam(BATTERY_PARAM, 0.f, 1.f, 1.f, "Battery", " %", 0.f, 100.f);
        configParam(BATTERY_TRIM_PARAM, -1.f, 1.f, 0.f, "Battery CV Trim");
        configParam(FUZZ_PARAM, 0.f, 1.f, 0.7f, "Fuzz", " %", 0.f, 100.f);
        configParam(FUZZ_TRIM_PARAM, -1.f, 1.f, 0.f, "Fuzz CV Trim");
        configParam(GUITAR_PARAM, 0.f, 1.f, 1.f, "Guitar volume", " %", 0.f, 100.f);
        configParam(GUITAR_TRIM_PARAM, -1.f, 1.f, 0.f, "Guitar CV Trim");
        configSwitch(MODEL_PARAM, 0.f, (float)(kModelCount - 1), 0.f, "Model", names);
        configParam(VOLUME_PARAM, 0.f, 1.f, 0.7f, "Volume", " %", 0.f, 100.f);
        configSwitch(SWITCH_PARAM, 0.f, 1.f, 1.f, "Footswitch", { "Off (clean)", "On" });
        configInput(IN_INPUT, "Audio");
        configInput(BATTERY_CV_INPUT, "Battery CV");
        configInput(FUZZ_CV_INPUT, "Fuzz CV");
        configInput(GUITAR_CV_INPUT, "Guitar volume CV");
        configInput(MODEL_CV_INPUT, "Model CV (1 V per model, wraps)");
        configInput(VOLUME_CV_INPUT, "Volume CV (10 V = full range)");
        configInput(SWITCH_CV_INPUT, "Footswitch trigger (toggles)");
        configOutput(OUT_OUTPUT, "Audio");
        configBypass(IN_INPUT, OUT_OUTPUT);

        models[0] = new fuzzy::SingleStageModel<fuzzy::FaceCircuit>();
        models[1] = new fuzzy::SingleStageModel<fuzzy::BenderMk1Circuit>();
        models[2] = new fuzzy::SingleStageModel<fuzzy::BenderMk2Circuit>();
        models[3] = new fuzzy::SingleStageModel<fuzzy::VoxCircuit>();
        models[4] = new fuzzy::SingleStageModel<fuzzy::MaestroCircuit>();
        models[5] = new fuzzy::SuperFuzzModel();
        models[6] = new fuzzy::FetOctaveModel();
        models[7] = new fuzzy::SingleStageModel<fuzzy::OverdriverCircuit>();
        noiseWhite.seed(1);
        noisePink.seed(2);
    }

    ~Fuzzy() {
        for (int k = 0; k < kModelCount; ++k) delete models[k];
    }

    void onReset() override {
        Module::onReset();
        appliedSampleRate = 0.f;    // full re-init on the next sample
    }

    // -------------------------------------------------------------------------
    float readNormalized(int paramId, int trimId, int inputId) {
        float value = params[paramId].getValue();
        if (inputs[inputId].isConnected())
            value += params[trimId].getValue() * inputs[inputId].getVoltage() * 0.1f;
        return clamp(value, 0.f, 1.f);
    }

    // Knob plus CV, wrapping around past either end of the list.
    int readModel() {
        float value = params[MODEL_PARAM].getValue();
        if (inputs[MODEL_CV_INPUT].isConnected()) value += inputs[MODEL_CV_INPUT].getVoltage();
        int index = (int)std::round(value) % kModelCount;
        return (index < 0) ? index + kModelCount : index;
    }

    fuzzy::ModelControls controls() {
        fuzzy::ModelControls c;
        c.fuzz = smoothFuzz;
        // Reverse-audio taper on the guitar pot: these circuits' low input
        // impedance puts all the clean-up in the top tenth of a real pot's
        // travel; this spreads it across the knob.
        c.guitar = 1.f - (1.f - smoothGuitar) * (1.f - smoothGuitar);
        c.drain = drain;
        return c;
    }

    double stepTime(float sampleRate) { return 1.0 / ((double)sampleRate * fuzzy::kOversample); }

    void updateModelGain(int k) {
        const double* g = kModels[k].outputGain;
        double fuzz = smoothFuzz;
        double gain = (fuzz <= 0.3) ? g[0]
                    : (fuzz <= 0.7) ? g[0] + (fuzz - 0.3) / 0.4 * (g[1] - g[0])
                                    : g[1] + (fuzz - 0.7) / 0.3 * (g[2] - g[1]);
        modelGain[k] = (float)(gain * kModels[k].polarity);
    }

    // Brings a model up at the current controls: matrices and DC operating point.
    void startModel(int k, float sampleRate) {
        fuzzy::ModelControls c = controls();
        models[k]->configure(c);
        if (!models[k]->start(stepTime(sampleRate), modelStarted[k]))
            models[k]->start(stepTime(sampleRate), false);
        modelStarted[k] = true;
        highPassState[k] = highPassInput[k] = 0.f;
        upsampler[k].reset();
        updateModelGain(k);
        // Resting currents, for the device view.
        float forward[fuzzy::kMaxDevices] = {}, reverse[fuzzy::kMaxDevices] = {};
        models[k]->sense(forward, reverse);
        for (int d = 0; d < models[k]->deviceCount(); ++d)
            deviceRest[k][d] = (models[k]->deviceKind(d) == fuzzy::DEVICE_DIODE)
                             ? kDiodeGlowFloorAmps : std::max(std::fabs(forward[d]), 1e-7f);
        resetSense();
    }

    void resetSense() {
        for (int d = 0; d < fuzzy::kMaxDevices; ++d) { devicePeak[d] = 0.f; deviceLow[d] = 1e9f; deviceSaturated[d] = 0; }
        senseSamples = 0;
    }

    // Once per base sample: the active circuit's junction currents against rest.
    void senseDevices() {
        float forward[fuzzy::kMaxDevices] = {}, reverse[fuzzy::kMaxDevices] = {};
        fuzzy::FuzzModel* model = models[activeModel];
        model->sense(forward, reverse);
        int count = model->deviceCount();
        for (int d = 0; d < count; ++d) {
            float ratio = std::fabs(forward[d]) / deviceRest[activeModel][d];
            devicePeak[d] = std::max(devicePeak[d], ratio);
            deviceLow[d] = std::min(deviceLow[d], ratio);
            if (reverse[d] > 1e-6f && reverse[d] > kSaturatedShare * std::fabs(forward[d])) deviceSaturated[d]++;
        }
        senseSamples++;
    }

    // Once per control block: this block's extremes into what the display shows.
    void updateDeviceView(float sampleRate) {
        float fall = std::exp(-(float)kControlBlock / (sampleRate * kDeviceFallSec));
        int count = models[activeModel]->deviceCount();
        for (int d = 0; d < fuzzy::kMaxDevices; ++d) {
            float glow = 0.f, saturation = 0.f, cutoff = 0.f;
            if (!circuitsIdle && d < count && senseSamples > 0) {
                bool diode = models[activeModel]->deviceKind(d) == fuzzy::DEVICE_DIODE;
                // Diodes: dark at the floor current, full three decades up.
                glow = diode ? std::log10(devicePeak[d] + 1e-9f) / 3.f
                             : kGlowAtRest + kGlowPerDecade * std::log10(devicePeak[d] + 1e-9f);
                glow = clamp(glow, 0.f, 1.f);
                saturation = (float)deviceSaturated[d] / senseSamples;
                cutoff = (!diode && deviceLow[d] < 0.05f) ? 1.f : 0.f;
            }
            deviceGlow[d] = std::max(glow, deviceGlow[d] * fall);
            deviceSaturation[d] = std::max(saturation, deviceSaturation[d] * fall);
            deviceCutoff[d] = std::max(cutoff, deviceCutoff[d] * fall);
        }
        resetSense();
    }

    void reinit(float sampleRate) {
        readControls(true, 1.f / sampleRate);
        for (int k = 0; k < kModelCount; ++k) modelStarted[k] = false;
        activeModel = requestedModel = readModel();
        fadingModel = -1;
        fadePosition = 1.f;
        startModel(activeModel, sampleRate);
        resampler.reset();
        levelEnvelope = kLevelFloorVolts;
        levelGain = kTargetPeakVolts / kLevelFloorVolts;
        levelAttackK = 1.f - std::exp(-1.f / (sampleRate * kLevelAttackSec));
        levelReleaseK = 1.f - std::exp(-1.f / (sampleRate * kLevelReleaseSec));
        levelDropK = 1.f - std::exp(-1.f / (sampleRate * kLevelGainDropSec));
        for (int k = 0; k < kDryDelaySamples; ++k) dryLine[k] = 0.f;
        meterK = 1.f - std::exp(-1.f / (sampleRate * kMeterSec));
        meterPeakDecay = std::exp(-2.302585f / sampleRate);
        switchFade = params[SWITCH_PARAM].getValue() > 0.5f ? 1.f : 0.f;
        circuitsIdle = (switchFade == 0.f);
        {   // Butterworth high-pass, bilinear.
            float w = std::tan((float)M_PI * kDcBlockHz / sampleRate), q = (float)M_SQRT1_2;
            float norm = 1.f / (1.f + w / q + w * w);
            blockB0 = norm; blockB1 = -2.f * norm; blockB2 = norm;
            blockA1 = 2.f * (w * w - 1.f) * norm;
            blockA2 = (1.f - w / q + w * w) * norm;
            blockState1 = blockState2 = 0.f;
        }
        appliedSampleRate = sampleRate;
        for (int k = 0; k < kModelCount; ++k)
            highPassCoeff[k] = std::exp(-2.f * (float)M_PI * (float)kModels[k].outputHighPassHz * (float)stepTime(sampleRate));
        // Base noise current per sample: white density times sqrt(bandwidth).
        noiseAmplitude = kNoiseWhiteAmps * std::sqrt(0.5 * sampleRate);
    }

    void readControls(bool jump, float sampleTime) {
        float fuzzNorm   = readNormalized(FUZZ_PARAM, FUZZ_TRIM_PARAM, FUZZ_CV_INPUT);
        float guitarNorm = readNormalized(GUITAR_PARAM, GUITAR_TRIM_PARAM, GUITAR_CV_INPUT);
        float chargeNorm = readNormalized(BATTERY_PARAM, BATTERY_TRIM_PARAM, BATTERY_CV_INPUT);
        float blockSec = (float)kControlBlock * sampleTime;
        float k = jump ? 1.f : 1.f - std::exp(-blockSec / kSmoothTau);
        float kCharge = jump ? 1.f : 1.f - std::exp(-blockSec / kBatterySmoothSec);
        if (!smoothedValid) { k = kCharge = 1.f; smoothedValid = true; }
        smoothFuzz   += k * (fuzzNorm - smoothFuzz);
        smoothGuitar += k * (guitarNorm - smoothGuitar);
        smoothCharge += kCharge * (chargeNorm - smoothCharge);
        drain = 1.f - smoothCharge;
    }

    void controlBlock(float sampleRate) {
        readControls(false, 1.f / sampleRate);
        updateDeviceView(sampleRate);

        // Model selection, rate-limited: a request must hold, and switches
        // are spaced, so CV can never switch models at audio rate.
        int wanted = readModel();
        if (wanted != requestedModel) { requestedModel = wanted; requestHeld = 0; }
        else requestHeld += kControlBlock;
        sinceSwitch += kControlBlock;

        // Footswitch off and faded out: nothing runs. Switched back on, the
        // selected circuit restarts from its operating point and fades in.
        bool switchedOn = params[SWITCH_PARAM].getValue() > 0.5f;
        if (circuitsIdle) {
            activeModel = requestedModel;
            if (!switchedOn) return;
            fadingModel = -1;
            fadePosition = 1.f;
            startModel(activeModel, sampleRate);
            resampler.reset();
            circuitsIdle = false;
        } else if (!switchedOn && switchFade <= 0.f) {
            circuitsIdle = true;
            return;
        }

        if (requestedModel != activeModel && fadingModel < 0
            && requestHeld >= (int)(kModelSettleSec * sampleRate)
            && sinceSwitch >= (int)(kModelMinHoldSec * sampleRate)) {
            startModel(requestedModel, sampleRate);
            fadingModel = activeModel;
            activeModel = requestedModel;
            fadePosition = 0.f;
            sinceSwitch = 0;
        }

        fuzzy::ModelControls c = controls();
        const int running[2] = { activeModel, fadingModel };
        for (int r = 0; r < 2; ++r) {
            int k = running[r];
            if (k < 0) continue;
            if (models[k]->configure(c)) models[k]->rebuild(stepTime(sampleRate));
            models[k]->refresh();
            updateModelGain(k);
        }
    }

    // -------------------------------------------------------------------------
    void process(const ProcessArgs& args) override {
        if (appliedSampleRate != args.sampleRate) {
            reinit(args.sampleRate);
            controlCounter = 0;
        }
        if (++controlCounter >= kControlBlock) {
            controlCounter = 0;
            controlBlock(args.sampleRate);
        }

        // ---- Leveller ------------------------------------------------------------
        float in = inputs[IN_INPUT].getVoltageSum();
        float level = std::fabs(in);
        levelEnvelope += (level > levelEnvelope ? levelAttackK : levelReleaseK) * (level - levelEnvelope);
        float targetGain = kTargetPeakVolts / std::max(levelEnvelope, kLevelFloorVolts);
        levelGain += (targetGain < levelGain ? levelDropK : levelAttackK) * (targetGain - levelGain);
        float emf = in * levelGain;
        if (!std::isfinite(emf)) { emf = 0.f; levelEnvelope = kLevelFloorVolts; levelGain = 0.f; }

        // ---- Footswitch --------------------------------------------------------------
        if (switchTrigger.process(inputs[SWITCH_CV_INPUT].getVoltage(), 0.1f, 1.f))
            params[SWITCH_PARAM].setValue(params[SWITCH_PARAM].getValue() > 0.5f ? 0.f : 1.f);
        float switchTarget = params[SWITCH_PARAM].getValue() > 0.5f ? 1.f : 0.f;
        float switchStep = args.sampleTime / kSwitchFadeSec;
        if (!circuitsIdle) switchFade = clamp(switchFade + (switchTarget > switchFade ? switchStep : -switchStep), 0.f, 1.f);
        lights[SWITCH_LIGHT].setBrightnessSmooth(switchTarget, args.sampleTime);

        float dry = dryLine[dryIndex] * kDryGain;
        dryLine[dryIndex] = emf;
        if (++dryIndex >= kDryDelaySamples) dryIndex = 0;

        float wet = 0.f;
        if (!circuitsIdle) wet = runCircuits(emf, args.sampleRate);
        float volume = clamp(params[VOLUME_PARAM].getValue() + 0.1f * inputs[VOLUME_CV_INPUT].getVoltage(), 0.f, 1.f);
        wet *= volume * volume * kOutScale;

        // ---- Mix, DC blocker, limiter ---------------------------------------------------
        float out = wet * switchFade + dry * (1.f - switchFade);
        float blocked = blockB0 * out + blockState1;
        blockState1 = blockB1 * out - blockA1 * blocked + blockState2;
        blockState2 = blockB2 * out - blockA2 * blocked;
        out = blocked;
        float magnitude = std::fabs(out);
        if (magnitude > kLimitKnee) {
            float room = kLimitCeiling - kLimitKnee;
            out = std::copysign(kLimitKnee + room * std::tanh((magnitude - kLimitKnee) / room), out);
        }
        if (!std::isfinite(out)) out = 0.f;
        outputs[OUT_OUTPUT].setVoltage(out);

        meterPower += meterK * (out * out - meterPower);
        meterPeak = std::max(std::fabs(out), meterPeak * meterPeakDecay);
    }

    // The selected circuit (and the one fading out), oversampled; returns the
    // decimated mix before VOLUME.
    float runCircuits(float emf, float sampleRate) {
        double noise = noiseAmplitude * (noiseWhite.gaussian()
                     + std::sqrt(kNoiseFlickerCornerHz / 1000.0) / fuzzy::kPinkGainAt1kHz * noisePink.pink(noisePink.gaussian()));
        const int running[2] = { activeModel, fadingModel };
        float feed[2][fuzzy::kOversample];
        for (int r = 0; r < 2; ++r) {
            int k = running[r];
            if (k < 0) continue;
            models[k]->prepare(noise * models[k]->noiseScale());
            upsampler[k].up((float)models[k]->front(emf), feed[r]);
        }

        float mixed[fuzzy::kOversample];
        const float fadeStep = 1.f / (kCrossfadeSec * sampleRate * fuzzy::kOversample);
        for (int s = 0; s < fuzzy::kOversample; ++s) {
            float sum = 0.f;
            for (int r = 0; r < 2; ++r) {
                int k = running[r];
                if (k < 0) continue;
                // The circuit's output capacitor into its level pot: one-pole high-pass.
                float y = (float)models[k]->step(feed[r][s]);
                highPassState[k] = highPassCoeff[k] * (highPassState[k] + y - highPassInput[k]);
                highPassInput[k] = y;
                float level = highPassState[k] * modelGain[k];
                // Equal-power crossfade: the two circuits are uncorrelated.
                if (fadingModel >= 0)
                    level *= (r == 0) ? std::sin(0.5f * (float)M_PI * fadePosition)
                                      : std::cos(0.5f * (float)M_PI * fadePosition);
                sum += level;
            }
            mixed[s] = sum;
            if (fadingModel >= 0) {
                fadePosition += fadeStep;
                if (fadePosition >= 1.f) { fadePosition = 1.f; fadingModel = -1; }
            }
        }
        // A diverged circuit is restarted at its operating point.
        if (!models[activeModel]->finite()) {
            models[activeModel]->start(stepTime(sampleRate), false);
            highPassState[activeModel] = highPassInput[activeModel] = 0.f;
        }
        senseDevices();
        return resampler.down(mixed);
    }
};


// =============================================================================
struct RoundLargeBlackSnapKnob : RoundLargeBlackKnob {
    RoundLargeBlackSnapKnob() { snap = true; }
};

struct FuzzyWidget : ModuleWidget {

    // Level meter (left), model name and what is simulated (top), battery
    // (top right), and the simulated devices glowing with their currents.
    struct FuzzyDisplay : TransparentWidget {
        Fuzzy* module = nullptr;
        std::shared_ptr<Font> font;

        void draw(const DrawArgs& args) override {
            nvgBeginPath(args.vg);
            nvgRoundedRect(args.vg, 0, 0, box.size.x, box.size.y, 2.f);
            nvgFillColor(args.vg, nvgRGB(8, 8, 12));
            nvgFill(args.vg);
        }

        static NVGcolor withAlpha(NVGcolor color, float alpha) { color.a = alpha; return color; }

        // One device symbol centred at (x, y), radius r.
        void drawDevice(NVGcontext* vg, fuzzy::DeviceKind kind, int index, float x, float y, float r,
                        float glow, float saturation, float cutoff, float dim) {
            // Body: amber glow, pushed toward hot red-white by saturation.
            NVGcolor body = nvgRGBf(0.90f + 0.10f * saturation, 0.58f - 0.25f * saturation, 0.24f + 0.10f * saturation);
            nvgBeginPath(vg);
            nvgCircle(vg, x, y, r);
            nvgFillColor(vg, withAlpha(body, 0.85f * glow * dim));
            nvgFill(vg);
            NVGcolor line = withAlpha(nvgRGB(208, 140, 89), (0.9f - 0.55f * cutoff) * dim);
            nvgStrokeColor(vg, line);
            nvgFillColor(vg, line);
            nvgStrokeWidth(vg, 0.8f);
            nvgBeginPath(vg);
            nvgCircle(vg, x, y, r);
            nvgStroke(vg);

            const float bar = x - 0.25f * r;
            if (kind == fuzzy::DEVICE_PNP || kind == fuzzy::DEVICE_NPN) {
                nvgBeginPath(vg);
                nvgMoveTo(vg, x - 1.25f * r, y); nvgLineTo(vg, bar, y);                       // base
                nvgMoveTo(vg, bar, y - 0.5f * r); nvgLineTo(vg, bar, y + 0.5f * r);           // base bar
                nvgMoveTo(vg, bar, y - 0.22f * r); nvgLineTo(vg, x + 0.45f * r, y - 0.75f * r); // collector
                nvgLineTo(vg, x + 0.45f * r, y - 1.25f * r);
                nvgMoveTo(vg, bar, y + 0.22f * r); nvgLineTo(vg, x + 0.45f * r, y + 0.75f * r); // emitter
                nvgLineTo(vg, x + 0.45f * r, y + 1.25f * r);
                nvgStroke(vg);
                // Emitter arrow: out of the device for NPN, into it for PNP.
                float tipT = (kind == fuzzy::DEVICE_NPN) ? 0.85f : 0.35f, tailT = (kind == fuzzy::DEVICE_NPN) ? 0.45f : 0.75f;
                float ex = x + 0.45f * r - bar, ey = 0.53f * r;
                float tipX = bar + tipT * ex, tipY = y + 0.22f * r + tipT * ey;
                float tailX = bar + tailT * ex, tailY = y + 0.22f * r + tailT * ey;
                float nx = -(tipY - tailY), ny = tipX - tailX;
                nvgBeginPath(vg);
                nvgMoveTo(vg, tipX, tipY);
                nvgLineTo(vg, tailX + 0.5f * nx, tailY + 0.5f * ny);
                nvgLineTo(vg, tailX - 0.5f * nx, tailY - 0.5f * ny);
                nvgClosePath(vg);
                nvgFill(vg);
            } else if (kind == fuzzy::DEVICE_NJFET) {
                const float channel = x + 0.1f * r;
                nvgBeginPath(vg);
                nvgMoveTo(vg, channel, y - 0.55f * r); nvgLineTo(vg, channel, y + 0.55f * r); // channel
                nvgMoveTo(vg, channel, y - 0.4f * r); nvgLineTo(vg, x + 0.5f * r, y - 0.4f * r);
                nvgLineTo(vg, x + 0.5f * r, y - 1.25f * r);                                   // drain
                nvgMoveTo(vg, channel, y + 0.4f * r); nvgLineTo(vg, x + 0.5f * r, y + 0.4f * r);
                nvgLineTo(vg, x + 0.5f * r, y + 1.25f * r);                                   // source
                nvgMoveTo(vg, x - 1.25f * r, y + 0.4f * r); nvgLineTo(vg, channel, y + 0.4f * r); // gate
                nvgStroke(vg);
                nvgBeginPath(vg);                                                              // arrow into the channel
                nvgMoveTo(vg, channel - 0.05f * r, y + 0.4f * r);
                nvgLineTo(vg, channel - 0.45f * r, y + 0.2f * r);
                nvgLineTo(vg, channel - 0.45f * r, y + 0.6f * r);
                nvgClosePath(vg);
                nvgFill(vg);
            } else {
                // Diode, vertical; alternate ones point the other way (the antiparallel pair).
                float direction = (index % 2 == 0) ? 1.f : -1.f;
                float apex = y + 0.35f * r * direction, base = y - 0.35f * r * direction;
                nvgBeginPath(vg);
                nvgMoveTo(vg, x - 0.45f * r, base); nvgLineTo(vg, x + 0.45f * r, base); nvgLineTo(vg, x, apex);
                nvgClosePath(vg);
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgMoveTo(vg, x - 0.45f * r, apex); nvgLineTo(vg, x + 0.45f * r, apex);      // bar
                nvgMoveTo(vg, x, y - 1.25f * r); nvgLineTo(vg, x, y + 1.25f * r);            // leads
                nvgStroke(vg);
            }
        }

        void drawLayer(const DrawArgs& args, int layer) override {
            if (layer != 1) { TransparentWidget::drawLayer(args, layer); return; }
            const float w = box.size.x, h = box.size.y, pad = 3.f;
            int   model  = module ? module->activeModel : 0;
            float charge = module ? 1.f - module->drain : 1.f;
            bool  on     = module ? module->params[Fuzzy::SWITCH_PARAM].getValue() > 0.5f : true;
            float dim    = on ? 1.f : 0.35f;
            const NVGcolor gold = nvgRGB(208, 140, 89);

            // ---- Meter: 12 segments, -24 .. +6 dB around kMeterZeroVolts rms
            const int segments = 12;
            const float meterX = pad, meterW = 4.f, meterTop = pad, meterBottom = h - pad;
            const float segmentH = (meterBottom - meterTop) / segments;
            float levelDb = -60.f;
            bool  limiting = false;
            if (module) {
                levelDb = 10.f * std::log10(module->meterPower / (kMeterZeroVolts * kMeterZeroVolts) + 1e-9f);
                limiting = module->meterPeak > kLimitKnee;
            }
            int lit = (int)std::floor((levelDb + 24.f) / 2.5f) + 1;
            for (int k = 0; k < segments; ++k) {
                float segmentDb = -24.f + 2.5f * k;          // bottom edge of the segment
                NVGcolor color = (segmentDb >= 0.f) ? nvgRGB(230, 60, 40)
                               : (segmentDb >= -5.f) ? nvgRGB(230, 170, 40) : nvgRGB(90, 200, 90);
                bool active = (k < lit) || (k == segments - 1 && limiting);
                color.a = active ? 0.95f : 0.12f;
                float y = meterBottom - (k + 1) * segmentH;
                nvgBeginPath(args.vg);
                nvgRect(args.vg, meterX, y + 0.5f, meterW, segmentH - 1.f);
                nvgFillColor(args.vg, color);
                nvgFill(args.vg);
            }
            const float left = meterX + meterW + 3.f, right = w - pad;

            // ---- Battery, top right
            const float bodyW = 13.f, bodyH = 6.f;
            const float bodyX = right - bodyW - 2.f, bodyY = pad;
            NVGcolor outline = withAlpha(gold, 0.8f);
            nvgBeginPath(args.vg);
            nvgRect(args.vg, bodyX, bodyY, bodyW, bodyH);
            nvgStrokeColor(args.vg, outline);
            nvgStrokeWidth(args.vg, 0.8f);
            nvgStroke(args.vg);
            nvgBeginPath(args.vg);
            nvgRect(args.vg, bodyX + bodyW, bodyY + 1.8f, 1.8f, bodyH - 3.6f);
            nvgFillColor(args.vg, outline);
            nvgFill(args.vg);
            NVGcolor cell = (charge > 0.5f) ? nvgRGB(90, 200, 90) : (charge > 0.2f) ? nvgRGB(230, 170, 40) : nvgRGB(230, 60, 40);
            float fillW = (bodyW - 2.f) * clamp(charge, 0.f, 1.f);
            if (fillW > 0.2f) {
                nvgBeginPath(args.vg);
                nvgRect(args.vg, bodyX + 1.f, bodyY + 1.f, fillW, bodyH - 2.f);
                nvgFillColor(args.vg, cell);
                nvgFill(args.vg);
            }

            // ---- Name and the detail line
            if (!font)
                font = APP->window->loadFont(asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf"));
            if (font) {
                // Name centred on the display, sized to clear the meter and
                // the battery on either side; the detail line as large as fits.
                const char* label = kModels[model].label;
                const char* detail = on ? kModels[model].detail : "OFF";
                const float centre = 0.5f * w;
                float halfRoom = std::min(centre - left, bodyX - 2.f - centre);
                float size = std::min(12.f, 2.f * halfRoom / (0.61f * std::max<size_t>(1, std::strlen(label))));
                nvgFontFaceId(args.vg, font->handle);
                nvgFontSize(args.vg, size);
                nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
                nvgFillColor(args.vg, withAlpha(gold, dim));
                nvgText(args.vg, centre, pad + 6.f, label, NULL);
                float detailSize = std::min(7.5f, 2.f * (centre - left) / (0.61f * std::max<size_t>(1, std::strlen(detail))));
                nvgFontSize(args.vg, detailSize);
                nvgFillColor(args.vg, withAlpha(gold, on ? 0.8f : 1.f));
                nvgText(args.vg, centre, pad + 16.f, detail, NULL);
            }

            // ---- Devices: evenly spaced, Q1.. then D1..
            if (module) {
                fuzzy::FuzzModel* circuit = module->models[model];
                int count = circuit->deviceCount();
                float span = right - left;
                float slot = span / count;
                float r = std::min(5.5f, 0.32f * slot);
                float y = pad + 30.5f;
                int transistor = 0, diode = 0;
                for (int d = 0; d < count; ++d) {
                    fuzzy::DeviceKind kind = circuit->deviceKind(d);
                    float x = left + (d + 0.5f) * slot;
                    drawDevice(args.vg, kind, diode, x, y, r,
                               module->deviceGlow[d], module->deviceSaturation[d], module->deviceCutoff[d], dim);
                    if (font) {
                        char name[8];
                        if (kind == fuzzy::DEVICE_DIODE) snprintf(name, sizeof(name), "D%d", ++diode);
                        else                             snprintf(name, sizeof(name), "Q%d", ++transistor);
                        nvgFontSize(args.vg, 6.5f);
                        nvgFillColor(args.vg, withAlpha(gold, 0.75f * dim));
                        nvgText(args.vg, x, y + r + 5.f, name, NULL);
                    }
                }
            }
            TransparentWidget::drawLayer(args, layer);
        }
    };

    FuzzyWidget(Fuzzy* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Fuzzy.svg"),
            asset::plugin(pluginInstance, "res/Fuzzy-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // 10HP, three columns. Positions in mm.
        const float col[3] = { 10.f, 25.4f, 40.8f };

        auto* display = createWidget<FuzzyDisplay>(mm2px(Vec(4.f, 14.f)));
        display->box.size = mm2px(Vec(42.8f, 18.f));
        display->module = module;
        addChild(display);

        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(col[0], 44.f)), module, Fuzzy::BATTERY_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(col[1], 44.f)), module, Fuzzy::FUZZ_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(col[2], 44.f)), module, Fuzzy::GUITAR_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(col[0], 57.f)), module, Fuzzy::BATTERY_TRIM_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(col[1], 57.f)), module, Fuzzy::FUZZ_TRIM_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(col[2], 57.f)), module, Fuzzy::GUITAR_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(col[0], 66.f)), module, Fuzzy::BATTERY_CV_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(col[1], 66.f)), module, Fuzzy::FUZZ_CV_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(col[2], 66.f)), module, Fuzzy::GUITAR_CV_INPUT));

        // MODEL half a column right of the first column, its CV below and to
        // the left; VOLUME with its CV under it.
        const float modelX = 0.5f * (col[0] + col[1]);
        addParam(createParamCentered<RoundLargeBlackSnapKnob>(mm2px(Vec(modelX, 82.f)), module, Fuzzy::MODEL_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(col[0], 94.f)), module, Fuzzy::MODEL_CV_INPUT));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(col[2], 82.f)), module, Fuzzy::VOLUME_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(col[2], 94.f)), module, Fuzzy::VOLUME_CV_INPUT));

        // Footswitch trigger input, the footswitch and its light under it, on
        // the jack row (in line with Twang's lower row).
        addInput (createInputCentered<ThemedPJ301MPort> (mm2px(Vec(col[1], 105.f)), module, Fuzzy::SWITCH_CV_INPUT));
        addParam(createLightParamCentered<VCVLightBezelLatch<RedLight>>(mm2px(Vec(col[1], 117.f)), module, Fuzzy::SWITCH_PARAM, Fuzzy::SWITCH_LIGHT));
        addInput (createInputCentered<ThemedPJ301MPort> (mm2px(Vec(col[0], 112.f)), module, Fuzzy::IN_INPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(col[2], 112.f)), module, Fuzzy::OUT_OUTPUT));
    }
};

Model* modelFuzzy = createModel<Fuzzy, FuzzyWidget>("Fuzzy");