////////////////////////////////////////////////////////////
//
//   Vega
//
//   written by Cody Geary
//   Copyright 2026, MIT License
//
//   Bass pedals after a classic 1976 analog bass pedal synthesizer.
//   Two sawtooth oscillators into the transistor ladder, contour
//   generators, VCA and the pitch hold, simulated from the Norlin
//   schematic and service bulletin 1178. Presets VEGA, TUBA and BASS use
//   the values read from the schematic's preset switches; VARIABLE plays
//   from the programmed preset sliders.
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
#include "FilterVega.h"
using namespace rack;

#include <atomic>
#include <cmath>
#include <algorithm>

// ---- Presets ----------------------------------------------------------------------
// The schematic's preset switch lines (A Variable, B Vega, C Tuba, D Bass): the
// IC801 mixer dividers, the contour attack and decay resistors into 5.6 uF
// (IC201 volume, IC401 filter, whose divider level is the contour amount) and
// the IC503 emphasis resistor.
struct VegaPreset {
    const char* name;
    float weightA, weightB;
    float ratioB;                    // osc B against osc A (bulletin 4.5)
    float loudLevel, loudAttackOhms, loudDecayOhms;
    float filterLevel, filterAttackOhms, filterDecayOhms;
    float emphasisOhms;
};
enum { PRESET_VARIABLE, PRESET_VEGA, PRESET_TUBA, PRESET_BASS };   // panel order
static const VegaPreset kPresets[3] = {     // indexed by preset - 1
    { "VEGA", 5.1f / 9.8f, 4.7f / 9.8f, 1.f,
      1.f, 22e3f, 300e3f,
      15.f / 16.2f, 1.2e3f * 15e3f / 16.2e3f, 360e3f,
      51e3f },
    { "TUBA", 9.1f / 9.2f, 0.1f / 9.2f, 2.f,
      1.f, 47.f, 15e3f,
      18.f / 38.f, 20e3f * 18e3f / 38e3f, 110e3f,
      10e3f },
    { "BASS", 5.1f / 9.8f, 4.7f / 9.8f, 1.f,
      1.f, 47.f, 43e3f,
      1.f, 47.f, 47.f,
      51e3f },
};
// Osc B's scale error against osc A, cents per octave below high C, where
// it is trimmed to zero beat. Hand-tunable.
static const float kVegaBeatCentsPerOctave = 1.5f;
static const float kBassBeatCentsPerOctave = 0.5f;

// ---- Calibration (bulletin 4.6) ---------------------------------------------------
static const float kFootTopHz = 3000.f;          // top of the foot FILTER, the one assumption
static const float kCutoffMinHz = 42.9f;         // foot FILTER down
static const float kFootOctaves = 6.128f;        // log2(kFootTopHz / kCutoffMinHz)
static const float kEmphasisAt8 = 4.28f;         // loop gain at the oscillation threshold
static const float kEmphasisOhmsAt8 = 5e3f;      // the EMPHASIS slider at 8, as a preset resistor; hand-tunable
static const float kRegenOhms = 1e3f;
static const float kContourOctaves = 4.5f;       // the filter contour's reach; hand-tunable
static const float kKeyboardTracking = 0.4f;     // held pitch into the cutoff (R511); hand-tunable
static const float kFastReleaseOhms = 10e3f;     // release with DECAY off (R213, R414)
static const float kContourSwitchOhms = 300.f;   // the 4016 in every contour arm; hand-tunable

// ---- VARIABLE's sliders -----------------------------------------------------------
// Time sliders run exponentially from these floors to their pots.
static const float kAttackFloorOhms = 47.f;
static const float kDecayFloorOhms = 1e3f;
static const float kLoudAttackOhms = 100e3f, kLoudDecayOhms = 500e3f;      // R201, R209
static const float kFilterAttackOhms = 10e3f, kFilterDecayOhms = 500e3f;   // R402, R410
static const float kOscBLowSemitones = -1.f, kOscBSpanSemitones = 14.f;    // OSC B: -1 .. +13
static const float kVariableCutoffOctaves = 4.f;                           // CUTOFF's reach; hand-tunable

// ---- Global controls ------------------------------------------------------------
static const float kTuneSemitones = 1.5f;        // TUNE, either way
static const float kBeatCents = 34.3f;           // BEAT on osc B, either way (2%)
static const float kGlideMinSec = 0.005f, kGlideMaxSec = 1.f;
static const float kHoldSettleSec = 0.0005f;     // the pitch hold with GLIDE off
static const float kLeakCentsPerSecond = 12.f;   // the hold's droop in the release; hand-tunable, 0 holds

// ---- Character --------------------------------------------------------------------
static const float kOutputVolts = 6.30f;         // output volts per unit of A501's swing
static const float kGritRange = 4.f;             // GRIT: operating level 1/4x .. 4x around noon
// GRIT's makeup gain at each tenth of the knob: takes back three quarters of
// the level change (K-weighted, all presets). Re-measure if the circuit changes.
static const float kGritMakeup[11] = { 6.117f, 4.113f, 2.796f, 1.932f, 1.367f, 1.000f,
                                       0.765f, 0.628f, 0.562f, 0.522f, 0.495f };
// SNARL (see voiceControls). Hand-tunable.
static const float kSnarlFilterOctaves = 4.f;    // osc B's swing of the cutoff, either way
static const float kSnarlFilterReach = 0.5f;     // knob position where that swing is full
static const float kSnarlSubStart = 0.4f;        // knob position where the sub-octave comes in
static const float kSnarlSubDepth = 0.6f;        // how far the cutoff's modulator goes over to it
static const float kSnarlIndex = 0.2f;           // the sub-octave's FM of osc A at full travel

// ---- Engine -----------------------------------------------------------------------
static const float kLimitKnee = 7.f, kLimitCeiling = 10.f;   // output limiter, volts
static const float kSmoothSec = 0.01f;           // panel control smoothing
static const int kControlBlock = 32;
static const int kOversampleFactors[3] = { 1, 2, 4 };
static const int kKeys = 13;
static const float kLowCHz = 32.703f;            // the low C pedal
static const float kFoldMarginVolts = 0.5f / 12.f;   // V/Oct this close to a C folds as that C

// ---- Polyphony --------------------------------------------------------------------
// Voice 0 runs alone (the mono engine); voices 1 .. 15 run four at a time,
// group g holding voices 1 + 4g .. 4 + 4g. Per-voice arrays run one past the
// last voice so the last group's lanes always load as a float_4.
static const int kMaxVoices = 16;
static const int kGroups = (kMaxVoices - 1 + 3) / 4;
static const int kVoiceSlots = 1 + 4 * kGroups;

// ---- Voice variation: each part's largest tolerance either way ------------------
// Voice 0 is the calibrated unit. Hand-tunable.
static const float kSpreadTuneCents = 3.f;             // both oscillators
static const float kSpreadBeatCents = 2.f;             // osc B against osc A
static const float kSpreadScaleCentsPerOctave = 1.f;   // osc B's scale error
static const float kSpreadCutoffOctaves = 0.06f;       // ladder and expo converter
static const float kSpreadEmphasis = 0.04f;            // REGEN share, as a fraction
static const float kSpreadContourCap = 0.1f;           // C201, C402, as a fraction
static const float kSpreadPairOffset = 0.035f;         // ladder pair offsets, over 2 Vt (~1.8 mV)
enum Spread { SPREAD_TUNE, SPREAD_BEAT, SPREAD_SCALE, SPREAD_CUTOFF, SPREAD_EMPHASIS, SPREAD_LOUD_CAP, SPREAD_FILTER_CAP,
              SPREAD_PAIR, NUM_SPREADS = SPREAD_PAIR + vega::kLadderPairs };

// ---- Strips: every control with a CV (10% of travel per volt) ----------------------
// The console first (GLIDE, then VARIABLE's sliders in panel order), then the
// knobs with a trim: VOLUME, FILTER, GRIT, SNARL.
enum Strip {
    STRIP_GLIDE,
    STRIP_MIX, STRIP_OSC_B,
    STRIP_VOLUME_ATTACK, STRIP_VOLUME_SUSTAIN, STRIP_VOLUME_DECAY,
    STRIP_CUTOFF, STRIP_EMPHASIS, STRIP_CONTOUR, STRIP_FILTER_ATTACK, STRIP_FILTER_DECAY,
    STRIP_VOLUME, STRIP_FILTER, STRIP_GRIT, STRIP_SNARL,
    NUM_STRIPS
};
static const int kSliderStrips = STRIP_VOLUME;   // the console's strips, no trim
static const int kTrimmedStrips = NUM_STRIPS - kSliderStrips;


// The knobs' and sliders' readouts: three significant figures, never in
// scientific notation, and anything within rounding of zero reads 0 (the
// OSC B slider's unison sits on a float that misses 0 by ~1e-8).
struct VegaQuantity : ParamQuantity {
    std::string getDisplayValueString() override {
        const float value = getDisplayValue();
        const float magnitude = std::fabs(value);
        if (magnitude < 5e-4f) return "0";
        const int decimals = std::max(0, 2 - (int)std::floor(std::log10(magnitude)));
        return string::f("%.*f", decimals, value);
    }
};

struct Vega : Module {
    enum ParamIds {
        STRIP_PARAM,
        STRIP_TRIM_PARAM = STRIP_PARAM + NUM_STRIPS,    // the knob strips only, from STRIP_VOLUME
        PRESET_PARAM = STRIP_TRIM_PARAM + kTrimmedStrips,   // no panel control: set by the buttons
        VARIABLE_BUTTON, VEGA_BUTTON, TUBA_BUTTON, BASS_BUTTON,
        GLIDE_PARAM, DECAY_PARAM, OCTAVE_PARAM,
        RANGE_PARAM,                                    // programmed preset OCTAVE, LO MED HI
        TUNE_PARAM, BEAT_PARAM,
        OVERSAMPLE_PARAM,                               // no panel control: context menu
        AUX_MODE_PARAM,                                 // the button over AUX IN
        PEDAL_RANGE_PARAM,                              // context menu: fold V/Oct onto the pedals, or full range
        OUT_MODE_PARAM,                                 // context menu: polyphonic OUT, or summed to mono
        VARIATION_PARAM,                                // context menu: per-voice part tolerances
        NUM_PARAMS
    };
    enum InputIds {
        STRIP_CV_INPUT,
        VOCT_INPUT = STRIP_CV_INPUT + NUM_STRIPS,
        GATE_INPUT,
        MODE_CV_INPUT,                                  // 1 V per preset, overrides the buttons
        AUX_INPUT,                                      // audio into the mixer
        TUNE_CV_INPUT, BEAT_CV_INPUT,                   // +-5 V spans each knob's travel
        NUM_INPUTS
    };
    enum OutputIds { OUT_OUTPUT, VOCT_OUTPUT, GATE_OUTPUT, NUM_OUTPUTS };
    enum LightIds {
        VARIABLE_LIGHT, VEGA_LIGHT, TUBA_LIGHT, BASS_LIGHT,
        GLIDE_LIGHT, DECAY_LIGHT, OCTAVE_LIGHT, AUX_MODE_LIGHT,
        STRIP_LIGHT,                                    // slider lights, from STRIP_MIX (GLIDE is a knob)
        NUM_LIGHTS = STRIP_LIGHT + kSliderStrips
    };

    vega::Voice voice0;                             // the mono engine
    vega::VoiceGroup groups[kGroups];               // voices 1 .. 15
    vega::Contour loudContour[kVoiceSlots], filterContour[kVoiceSlots];
    vega::PitchHold hold[kVoiceSlots];
    dsp::SchmittTrigger gateTriggers[kVoiceSlots];
    dsp::SchmittTrigger presetTriggers[4];
    float spread[kVoiceSlots][NUM_SPREADS] = {};

    // The pedal held with the mouse (-1 none), written by the keyboard widget.
    // It plays voice 0.
    std::atomic<int> mouseKey{ -1 };
    // The pedals lit on the keyboard, one bit per pedal, for the widget.
    std::atomic<uint32_t> litKeys{ 0 };

    // ---- Control rate ------------------------------------------------------------------
    int   controlCounter = 0;
    float appliedSampleRate = 0.f;
    int   oversample = 2;
    float sampleTime = 1.f / 48000.f;
    int   voices = 1;
    float droopPerSample = 0.f, smoothK = 1.f;
    bool  auxFilterOnly = false;
    // Per voice, set at control rate.
    int   voicePreset[kVoiceSlots] = {};
    float loudLevel[kVoiceSlots] = {}, loudAttackCoeff[kVoiceSlots] = {}, loudReleaseCoeff[kVoiceSlots] = {};
    float filterLevel[kVoiceSlots] = {}, filterAttackCoeff[kVoiceSlots] = {}, filterReleaseCoeff[kVoiceSlots] = {};
    float followCoeff[kVoiceSlots] = {};
    alignas(16) float ratioB[kVoiceSlots] = {}, beatCentsPerOctave[kVoiceSlots] = {};
    alignas(16) float octaveVolts[kVoiceSlots] = {}, tuneVolts[kVoiceSlots] = {};
    alignas(16) float cutoffSpreadOctaves[kVoiceSlots] = {};
    // Targets of the smoothed controls, per voice.
    alignas(16) float weightATarget[kVoiceSlots] = {}, weightBTarget[kVoiceSlots] = {};
    alignas(16) float emphasisTarget[kVoiceSlots] = {}, cutoffOffsetTarget[kVoiceSlots] = {};
    alignas(16) float footFilterTarget[kVoiceSlots] = {}, footLoudnessTarget[kVoiceSlots] = {};
    alignas(16) float levelTarget[kVoiceSlots] = {}, outputGainTarget[kVoiceSlots] = {}, filterFmTarget[kVoiceSlots] = {};
    alignas(16) float subBlendTarget[kVoiceSlots] = {}, crossFmTarget[kVoiceSlots] = {};
    // Smoothed per sample: voice 0 on its own, the rest four to a float_4.
    struct Smoothed {
        float weightA = 0.f, weightB = 0.f, emphasis = 0.f, cutoffOffset = 0.f;
        float footFilter = 0.f, footLoudness = 0.f, level = 1.f, outputGain = 0.f, filterFm = 0.f, subBlend = 0.f, crossFm = 0.f;
    } voice0Smoothed;
    simd::float_4 weightA[kGroups], weightB[kGroups], emphasis[kGroups], cutoffOffset[kGroups];
    simd::float_4 footFilter[kGroups], footLoudness[kGroups], level[kGroups], outputGain[kGroups], filterFm[kGroups], subBlend[kGroups], crossFm[kGroups];

    // ---- Per sample ----------------------------------------------------------------------
    float targetVolts[kVoiceSlots] = {};
    // Each voice's last V/Oct and the pedal it went to, for folding a C.
    float lastInputVolts[kVoiceSlots] = {}, lastPedalVolts[kVoiceSlots] = {};
    bool  foldHistory[kVoiceSlots] = {};

    Vega() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        // Names follow the panel. The time sliders read in the schematic's RC
        // time constants.
        configParam<VegaQuantity>(STRIP_PARAM + STRIP_GLIDE, 0.f, 1.f, 0.4f, "Glide time", " s", kGlideMaxSec / kGlideMinSec, kGlideMinSec);
        configParam<VegaQuantity>(STRIP_PARAM + STRIP_MIX, 0.f, 1.f, 0.5f, "B-A mix", "% A", 0.f, 100.f);
        configParam<VegaQuantity>(STRIP_PARAM + STRIP_OSC_B, 0.f, 1.f, -kOscBLowSemitones / kOscBSpanSemitones,
                    "Osc B", " semitones", 0.f, kOscBSpanSemitones, kOscBLowSemitones);
        configParam<VegaQuantity>(STRIP_PARAM + STRIP_VOLUME_ATTACK, 0.f, 1.f, 0.f, "Volume attack", " ms",
                    kLoudAttackOhms / kAttackFloorOhms, kAttackFloorOhms * vega::kContourFarads * 1000.f);
        configParam<VegaQuantity>(STRIP_PARAM + STRIP_VOLUME_SUSTAIN, 0.f, 1.f, 1.f, "Volume sustain", "%", 0.f, 100.f);
        configParam<VegaQuantity>(STRIP_PARAM + STRIP_VOLUME_DECAY, 0.f, 1.f, 0.6f, "Volume decay", " s",
                    kLoudDecayOhms / kDecayFloorOhms, kDecayFloorOhms * vega::kContourFarads);
        configParam<VegaQuantity>(STRIP_PARAM + STRIP_CUTOFF, 0.f, 1.f, 0.f, "Cutoff", " oct", 0.f, kVariableCutoffOctaves);
        configParam<VegaQuantity>(STRIP_PARAM + STRIP_EMPHASIS, 0.f, 1.f, 0.5f, "Emphasis", "", 0.f, 10.f);
        configParam<VegaQuantity>(STRIP_PARAM + STRIP_CONTOUR, 0.f, 1.f, 0.6f, "Filter amount", "%", 0.f, 100.f);
        configParam<VegaQuantity>(STRIP_PARAM + STRIP_FILTER_ATTACK, 0.f, 1.f, 0.f, "Filter attack", " ms",
                    kFilterAttackOhms / kAttackFloorOhms, kAttackFloorOhms * vega::kContourFarads * 1000.f);
        configParam<VegaQuantity>(STRIP_PARAM + STRIP_FILTER_DECAY, 0.f, 1.f, 0.6f, "Filter decay", " s",
                    kFilterDecayOhms / kDecayFloorOhms, kDecayFloorOhms * vega::kContourFarads);
        configParam<VegaQuantity>(STRIP_PARAM + STRIP_VOLUME, 0.f, 1.f, 1.f, "Volume", "%", 0.f, 100.f);
        configParam<VegaQuantity>(STRIP_PARAM + STRIP_FILTER, 0.f, 1.f, 0.4f, "Filter", " Hz", kFootTopHz / kCutoffMinHz, kCutoffMinHz);
        configParam<VegaQuantity>(STRIP_PARAM + STRIP_GRIT, 0.f, 1.f, 0.5f, "Grit", "x", kGritRange * kGritRange, 1.f / kGritRange);
        configParam<VegaQuantity>(STRIP_PARAM + STRIP_SNARL, 0.f, 1.f, 0.f, "Snarl", "%", 0.f, 100.f);
        getParamQuantity(STRIP_PARAM + STRIP_GRIT)->description = "Operating level of the circuit; noon is stock";
        getParamQuantity(STRIP_PARAM + STRIP_SNARL)->description = "Osc B sweeps the filter, then a sub-octave growl";
        const char* stripNames[NUM_STRIPS] = {
            "Glide", "B-A mix", "Osc B", "Volume attack", "Volume sustain", "Volume decay",
            "Cutoff", "Emphasis", "Filter amount", "Filter attack", "Filter decay",
            "Volume", "Filter", "Grit", "Snarl" };
        for (int s = 0; s < NUM_STRIPS; ++s) {
            const std::string name = stripNames[s];
            configInput(STRIP_CV_INPUT + s, name + " CV");
            if (s >= kSliderStrips)
                configParam<VegaQuantity>(STRIP_TRIM_PARAM + (s - kSliderStrips), -1.f, 1.f, 0.f, name + " CV trim", "%", 0.f, 100.f);
        }

        configSwitch(PRESET_PARAM, 0.f, 3.f, (float)PRESET_VEGA, "Preset", { "Variable", "Vega", "Tuba", "Bass" });
        configButton(VARIABLE_BUTTON, "Variable");
        configButton(VEGA_BUTTON, "Vega");
        configButton(TUBA_BUTTON, "Tuba");
        configButton(BASS_BUTTON, "Bass");
        configSwitch(GLIDE_PARAM, 0.f, 1.f, 0.f, "Glide", { "Off", "On" });
        configSwitch(DECAY_PARAM, 0.f, 1.f, 1.f, "Decay", { "Off", "On" });
        configSwitch(OCTAVE_PARAM, 0.f, 1.f, 0.f, "Octave", { "Low", "High" });
        configSwitch(RANGE_PARAM, 0.f, 2.f, 0.f, "Variable octave", { "Lo", "Med", "Hi" });
        configParam<VegaQuantity>(TUNE_PARAM, -1.f, 1.f, 0.f, "Tune", " cents", 0.f, kTuneSemitones * 100.f);
        configParam<VegaQuantity>(BEAT_PARAM, -1.f, 1.f, 0.f, "Beat", " cents", 0.f, kBeatCents);
        configSwitch(OVERSAMPLE_PARAM, 0.f, 2.f, 1.f, "Oversampling", { "Off (1x)", "2x", "4x" });
        getParamQuantity(OVERSAMPLE_PARAM)->randomizeEnabled = false;
        configSwitch(AUX_MODE_PARAM, 0.f, 1.f, 0.f, "Aux mode", { "Mix", "Filter only" });
        getParamQuantity(AUX_MODE_PARAM)->description = "Filter only: oscillators off, VOLUME opens the VCA";
        getParamQuantity(AUX_MODE_PARAM)->randomizeEnabled = false;
        configSwitch(PEDAL_RANGE_PARAM, 0.f, 1.f, 0.f, "V/Oct range", { "Folded onto the pedals", "Full range" });
        getParamQuantity(PEDAL_RANGE_PARAM)->randomizeEnabled = false;
        getParamQuantity(PRESET_PARAM)->randomizeEnabled = false;
        configSwitch(OUT_MODE_PARAM, 0.f, 1.f, 0.f, "OUT", { "Polyphonic", "Summed to mono" });
        getParamQuantity(OUT_MODE_PARAM)->randomizeEnabled = false;
        configSwitch(VARIATION_PARAM, 0.f, 1.f, 1.f, "Voice variation", { "Off", "On" });
        getParamQuantity(VARIATION_PARAM)->randomizeEnabled = false;

        configInput(VOCT_INPUT, "Pitch (V/Oct)");
        configInput(GATE_INPUT, "Gate");
        configInput(AUX_INPUT, "Aux");
        configInput(TUNE_CV_INPUT, "Tune CV");
        configInput(BEAT_CV_INPUT, "Beat CV");
        configInput(MODE_CV_INPUT, "Mode CV")->description = "1 V per preset: 0 Variable, 1 Vega, 2 Tuba, 3 Bass";
        configOutput(OUT_OUTPUT, "Audio");
        configOutput(VOCT_OUTPUT, "Pitch (V/Oct)");
        configOutput(GATE_OUTPUT, "Gate");

        // Each voice's part tolerances, fixed: a small hash of the voice and
        // the part, uniform in -1 .. 1. Voice 0 is the calibrated unit.
        for (int v = 0; v < kMaxVoices; ++v)
            for (int k = 0; k < NUM_SPREADS; ++k) {
                uint32_t h = (uint32_t)(v * 7919 + k * 104729 + 12345);
                h ^= h << 13; h ^= h >> 17; h ^= h << 5; h ^= h << 13; h ^= h >> 17; h ^= h << 5;
                spread[v][k] = (v == 0) ? 0.f : (float)(h & 0xFFFFu) / 32767.5f - 1.f;
            }
        for (int g = 0; g < kGroups; ++g) groups[g].seedNoise(1 + 4 * g);
    }

    void onReset() override {
        Module::onReset();
        appliedSampleRate = 0.f;
    }

    // A voice's preset: the buttons' choice, or the preset CV while it is
    // patched, 1 V per preset, wrapping every 4 V in either direction. A
    // polyphonic preset CV gives each voice its own.
    int preset(int v) {
        if (inputs[MODE_CV_INPUT].isConnected()) {
            const int step = (int)std::round(inputs[MODE_CV_INPUT].getPolyVoltage(v));
            return ((step % 4) + 4) % 4;
        }
        return clamp((int)std::round(params[PRESET_PARAM].getValue()), 0, 3);
    }

    void reinit(float sampleRate) {
        appliedSampleRate = sampleRate;
        sampleTime = 1.f / sampleRate;
        oversample = kOversampleFactors[clamp((int)std::round(params[OVERSAMPLE_PARAM].getValue()), 0, 2)];
        voice0.setRate(sampleRate, oversample);
        for (int g = 0; g < kGroups; ++g) groups[g].setRate(sampleRate, oversample);
        smoothK = 1.f - std::exp(-sampleTime / kSmoothSec);
        controlBlock(true);
        controlCounter = 0;
    }

    // A strip's position for voice v: its control plus 10% of the travel per
    // volt, scaled by the trim on the knob strips. A mono CV reaches every
    // voice; a polyphonic one goes channel to channel.
    float stripValue(int strip, int v) {
        float value = params[STRIP_PARAM + strip].getValue();
        if (inputs[STRIP_CV_INPUT + strip].isConnected()) {
            const float depth = (strip >= kSliderStrips) ? params[STRIP_TRIM_PARAM + (strip - kSliderStrips)].getValue() : 1.f;
            value += depth * inputs[STRIP_CV_INPUT + strip].getPolyVoltage(v) * 0.1f;
        }
        return clamp(value, 0.f, 1.f);
    }

    // One voice's coefficients from its preset, the panel and its CVs.
    void voiceControls(int v, bool decayOn, bool glideOn, bool octaveOn, bool variationOn) {
        const int which = preset(v);
        voicePreset[v] = which;
        const float* tolerance = spread[v];
        const float vary = variationOn ? 1.f : 0.f;

        float loudAttack, loudDecay, filterAttack, filterDecay, emphasisGain, rangeVolts = 0.f;
        if (which == PRESET_VARIABLE) {
            // B-MIX-A: the bottom of the slider is all osc B, the top all osc A.
            const float mix = stripValue(STRIP_MIX, v);
            weightATarget[v] = mix;
            weightBTarget[v] = 1.f - mix;
            ratioB[v] = std::exp2((kOscBLowSemitones + kOscBSpanSemitones * stripValue(STRIP_OSC_B, v)) / 12.f);
            beatCentsPerOctave[v] = 0.f;
            loudLevel[v] = stripValue(STRIP_VOLUME_SUSTAIN, v);
            filterLevel[v] = stripValue(STRIP_CONTOUR, v);
            loudAttack = kAttackFloorOhms * std::pow(kLoudAttackOhms / kAttackFloorOhms, stripValue(STRIP_VOLUME_ATTACK, v));
            loudDecay = kDecayFloorOhms * std::pow(kLoudDecayOhms / kDecayFloorOhms, stripValue(STRIP_VOLUME_DECAY, v));
            filterAttack = kAttackFloorOhms * std::pow(kFilterAttackOhms / kAttackFloorOhms, stripValue(STRIP_FILTER_ATTACK, v));
            filterDecay = kDecayFloorOhms * std::pow(kFilterDecayOhms / kDecayFloorOhms, stripValue(STRIP_FILTER_DECAY, v));
            // The slider's 0..10 scale, threshold at 8 (bulletin 4.6).
            emphasisGain = kEmphasisAt8 * 10.f * stripValue(STRIP_EMPHASIS, v) / 8.f;
            cutoffOffsetTarget[v] = kVariableCutoffOctaves * stripValue(STRIP_CUTOFF, v);
            rangeVolts = (float)clamp((int)std::round(params[RANGE_PARAM].getValue()), 0, 2);
        } else {
            const VegaPreset& p = kPresets[which - 1];
            weightATarget[v] = p.weightA;
            weightBTarget[v] = p.weightB;
            ratioB[v] = p.ratioB;
            beatCentsPerOctave[v] = (which == PRESET_VEGA) ? kVegaBeatCentsPerOctave
                                  : (which == PRESET_BASS) ? kBassBeatCentsPerOctave : 0.f;
            loudLevel[v] = p.loudLevel;
            filterLevel[v] = p.filterLevel;
            loudAttack = p.loudAttackOhms;
            loudDecay = p.loudDecayOhms;
            filterAttack = p.filterAttackOhms;
            filterDecay = p.filterDecayOhms;
            emphasisGain = kEmphasisAt8 * (kEmphasisOhmsAt8 + kRegenOhms) / (p.emphasisOhms + kRegenOhms);
            cutoffOffsetTarget[v] = 0.f;
        }
        emphasisTarget[v] = emphasisGain * (1.f + vary * kSpreadEmphasis * tolerance[SPREAD_EMPHASIS]);
        cutoffSpreadOctaves[v] = vary * kSpreadCutoffOctaves * tolerance[SPREAD_CUTOFF];
        beatCentsPerOctave[v] += vary * kSpreadScaleCentsPerOctave * tolerance[SPREAD_SCALE];

        // BEAT trims osc B on every preset; TUNE moves both oscillators.
        // Their CVs add a fifth of the knob's span per volt.
        const float beatCv = inputs[BEAT_CV_INPUT].isConnected() ? inputs[BEAT_CV_INPUT].getPolyVoltage(v) : 0.f;
        const float tuneCv = inputs[TUNE_CV_INPUT].isConnected() ? inputs[TUNE_CV_INPUT].getPolyVoltage(v) : 0.f;
        const float beat = clamp(params[BEAT_PARAM].getValue() + beatCv * 0.2f, -1.f, 1.f);
        const float tune = clamp(params[TUNE_PARAM].getValue() + tuneCv * 0.2f, -1.f, 1.f);
        ratioB[v] *= std::exp2((kBeatCents * beat + vary * kSpreadBeatCents * tolerance[SPREAD_BEAT]) / 1200.f);
        tuneVolts[v] = kTuneSemitones * tune / 12.f + vary * kSpreadTuneCents * tolerance[SPREAD_TUNE] / 1200.f;

        // Contours: the arm, then the 4016 switch (IC201, IC401) into the
        // voice's own 5.6 uF. The switch's ~300 ohms keeps the fastest contour
        // near 2 ms, so note-ons don't click.
        loudAttack += kContourSwitchOhms;
        loudDecay += kContourSwitchOhms;
        filterAttack += kContourSwitchOhms;
        filterDecay += kContourSwitchOhms;
        if (!decayOn) {
            loudDecay = std::min(loudDecay, kFastReleaseOhms);
            filterDecay = std::min(filterDecay, kFastReleaseOhms);
        }
        // The ladder's pair offsets: voice 0's typical set, or with variation
        // each further voice its own.
        float pairOffsets[vega::kLadderPairs];
        for (int k = 0; k < vega::kLadderPairs; ++k)
            pairOffsets[k] = (v > 0 && variationOn) ? kSpreadPairOffset * tolerance[SPREAD_PAIR + k] : vega::kPairOffsets[k];
        if (v == 0) voice0.ladder.setOffsets(pairOffsets);
        else groups[(v - 1) / 4].ladder.setOffsets((v - 1) & 3, pairOffsets);

        const float loudFarads = vega::kContourFarads * (1.f + vary * kSpreadContourCap * tolerance[SPREAD_LOUD_CAP]);
        const float filterFarads = vega::kContourFarads * (1.f + vary * kSpreadContourCap * tolerance[SPREAD_FILTER_CAP]);
        loudAttackCoeff[v] = vega::rcCoeff(loudAttack, loudFarads, sampleTime);
        loudReleaseCoeff[v] = vega::rcCoeff(loudDecay, loudFarads, sampleTime);
        filterAttackCoeff[v] = vega::rcCoeff(filterAttack, filterFarads, sampleTime);
        filterReleaseCoeff[v] = vega::rcCoeff(filterDecay, filterFarads, sampleTime);

        // Glide into the voice's pitch hold.
        const float glideSec = kGlideMinSec * std::pow(kGlideMaxSec / kGlideMinSec, stripValue(STRIP_GLIDE, v));
        followCoeff[v] = 1.f - std::exp(-sampleTime / (glideOn ? glideSec : kHoldSettleSec));
        // The foot OCTAVE and the programmed preset's range both move the
        // keyboard voltage, so both reach the cutoff through R511.
        octaveVolts[v] = (octaveOn ? 1.f : 0.f) + rangeVolts;

        footFilterTarget[v] = stripValue(STRIP_FILTER, v);
        footLoudnessTarget[v] = stripValue(STRIP_VOLUME, v);
        const float grit = stripValue(STRIP_GRIT, v);
        levelTarget[v] = std::pow(kGritRange, 2.f * grit - 1.f);
        const float gritScaled = grit * 10.f;
        const int gritIndex = std::min(9, (int)gritScaled);
        const float makeup = kGritMakeup[gritIndex] + (gritScaled - gritIndex) * (kGritMakeup[gritIndex + 1] - kGritMakeup[gritIndex]);
        // SNARL in two halves. The bottom: osc B's saw swings the cutoff,
        // reaching its full swing at kSnarlFilterReach. From kSnarlSubStart up:
        // the swing crossfades to the note's sub-octave, which also bends osc
        // A's frequency, so alternate cycles differ and the note growls an
        // octave down.
        const float snarlKnob = stripValue(STRIP_SNARL, v);
        filterFmTarget[v] = kSnarlFilterOctaves * std::min(1.f, snarlKnob / kSnarlFilterReach);
        const float subAmount = clamp((snarlKnob - kSnarlSubStart) / (1.f - kSnarlSubStart), 0.f, 1.f);
        subBlendTarget[v] = kSnarlSubDepth * subAmount;
        crossFmTarget[v] = kSnarlIndex * subAmount * subAmount;
        outputGainTarget[v] = kOutputVolts * makeup;

        // AUX IN, filter only: the oscillators drop out of the mixer.
        if (auxFilterOnly) { weightATarget[v] = 0.f; weightBTarget[v] = 0.f; }
    }

    // Panel, presets and CVs into every voice's coefficients.
    void controlBlock(bool jump) {
        for (int k = 0; k < 4; ++k)
            if (presetTriggers[k].process(params[VARIABLE_BUTTON + k].getValue()))
                params[PRESET_PARAM].setValue((float)k);
        const bool decayOn = params[DECAY_PARAM].getValue() > 0.5f;
        const bool glideOn = params[GLIDE_PARAM].getValue() > 0.5f;
        const bool octaveOn = params[OCTAVE_PARAM].getValue() > 0.5f;
        const bool variationOn = params[VARIATION_PARAM].getValue() > 0.5f;
        droopPerSample = kLeakCentsPerSecond / 1200.f * sampleTime;
        // AUX IN, filter only: with the jack patched the VCA follows VOLUME
        // alone; the filter contour still plays from the pedals or the gate.
        auxFilterOnly = inputs[AUX_INPUT].isConnected() && params[AUX_MODE_PARAM].getValue() > 0.5f;

        for (int v = 0; v < voices; ++v) voiceControls(v, decayOn, glideOn, octaveOn, variationOn);
        if (jump) {
            voice0Smoothed.weightA = weightATarget[0];
            voice0Smoothed.weightB = weightBTarget[0];
            voice0Smoothed.emphasis = emphasisTarget[0];
            voice0Smoothed.cutoffOffset = cutoffOffsetTarget[0];
            voice0Smoothed.footFilter = footFilterTarget[0];
            voice0Smoothed.footLoudness = footLoudnessTarget[0];
            voice0Smoothed.level = levelTarget[0];
            voice0Smoothed.outputGain = outputGainTarget[0];
            voice0Smoothed.filterFm = filterFmTarget[0];
            voice0Smoothed.subBlend = subBlendTarget[0];
            voice0Smoothed.crossFm = crossFmTarget[0];
            for (int g = 0; g < kGroups; ++g) snapGroup(g);
        }

        // A new oversampling setting re-initializes on the next sample.
        if (kOversampleFactors[clamp((int)std::round(params[OVERSAMPLE_PARAM].getValue()), 0, 2)] != oversample)
            appliedSampleRate = 0.f;

        // Lights: every preset a voice is playing, the three switches, and the
        // programmed preset's sliders while any voice is on VARIABLE.
        int presetsInUse = 0;
        for (int v = 0; v < voices; ++v) presetsInUse |= 1 << voicePreset[v];
        const float blockSeconds = kControlBlock * sampleTime;
        for (int k = 0; k < 4; ++k)
            lights[VARIABLE_LIGHT + k].setBrightnessSmooth((presetsInUse >> k) & 1 ? 1.f : 0.f, blockSeconds);
        lights[GLIDE_LIGHT].setBrightnessSmooth(glideOn ? 1.f : 0.f, blockSeconds);
        lights[DECAY_LIGHT].setBrightnessSmooth(decayOn ? 1.f : 0.f, blockSeconds);
        lights[OCTAVE_LIGHT].setBrightnessSmooth(octaveOn ? 1.f : 0.f, blockSeconds);
        lights[AUX_MODE_LIGHT].setBrightnessSmooth(params[AUX_MODE_PARAM].getValue() > 0.5f ? 1.f : 0.f, blockSeconds);
        const float variableLight = (presetsInUse & (1 << PRESET_VARIABLE)) ? 1.f : 0.f;
        for (int s = STRIP_MIX; s < kSliderStrips; ++s)
            lights[STRIP_LIGHT + s].setBrightnessSmooth(variableLight, blockSeconds);
    }

    // The smoothed controls of a group's four voices jump to their targets.
    void snapGroup(int g) {
        const int first = 1 + 4 * g;
        weightA[g] = simd::float_4::load(&weightATarget[first]);
        weightB[g] = simd::float_4::load(&weightBTarget[first]);
        emphasis[g] = simd::float_4::load(&emphasisTarget[first]);
        cutoffOffset[g] = simd::float_4::load(&cutoffOffsetTarget[first]);
        footFilter[g] = simd::float_4::load(&footFilterTarget[first]);
        footLoudness[g] = simd::float_4::load(&footLoudnessTarget[first]);
        level[g] = simd::float_4::load(&levelTarget[first]);
        outputGain[g] = simd::float_4::load(&outputGainTarget[first]);
        filterFm[g] = simd::float_4::load(&filterFmTarget[first]);
        subBlend[g] = simd::float_4::load(&subBlendTarget[first]);
        crossFm[g] = simd::float_4::load(&crossFmTarget[first]);
    }

    // Voices first-new .. voices-1 come into use: their contours, hold and
    // circuit start from rest, their controls from their targets.
    void startVoices(int firstNew) {
        const bool decayOn = params[DECAY_PARAM].getValue() > 0.5f;
        const bool glideOn = params[GLIDE_PARAM].getValue() > 0.5f;
        const bool octaveOn = params[OCTAVE_PARAM].getValue() > 0.5f;
        const bool variationOn = params[VARIATION_PARAM].getValue() > 0.5f;
        for (int v = firstNew; v < voices; ++v) {
            voiceControls(v, decayOn, glideOn, octaveOn, variationOn);
            loudContour[v].value = 0.f;
            filterContour[v].value = 0.f;
            hold[v] = vega::PitchHold();
            gateTriggers[v].reset();
            // Voice 0 is always in use, so a new voice is always in a group.
            const int g = (v - 1) / 4, lane = (v - 1) & 3;
            if (lane == 0) groups[g].reset();
            else groups[g].resetLane(lane);
        }
        for (int g = (firstNew - 1) / 4; 1 + 4 * g < voices; ++g) {
            // Keep the voices already sounding in a shared group where they are.
            const int first = 1 + 4 * g;
            float lanes[4];
            simd::float_4* smoothed[11] = { &weightA[g], &weightB[g], &emphasis[g], &cutoffOffset[g], &footFilter[g],
                                            &footLoudness[g], &level[g], &outputGain[g], &filterFm[g], &subBlend[g], &crossFm[g] };
            const float* targets[11] = { weightATarget, weightBTarget, emphasisTarget, cutoffOffsetTarget, footFilterTarget,
                                         footLoudnessTarget, levelTarget, outputGainTarget, filterFmTarget, subBlendTarget, crossFmTarget };
            for (int j = 0; j < 11; ++j) {
                smoothed[j]->store(lanes);
                for (int lane = 0; lane < 4; ++lane)
                    if (first + lane >= firstNew) lanes[lane] = targets[j][first + lane];
                *smoothed[j] = simd::float_4::load(lanes);
            }
        }
    }

    // A voice's V/Oct onto the pedals (0 = low C, 1 = high C), unless the full
    // range is chosen. Every note but C has one pedal in the octave. A C has
    // two, and takes the one that continues the line: the folded note moves
    // by the same interval the input did, so D -> C lands on the low C, B -> C
    // on the high C, a held C stays put, and playing within the pedals' own
    // octave comes through exactly.
    float pedalVolts(int v, float voct) {
        const float volts = clamp(voct + 3.f, -20.f, 20.f);   // C1 = -3 V
        if (params[PEDAL_RANGE_PARAM].getValue() > 0.5f) return volts;
        const float fromC = volts - std::round(volts);         // -0.5 .. 0.5 around the nearest C
        float pedal;
        if (std::fabs(fromC) <= kFoldMarginVolts) {
            const float wanted = foldHistory[v] ? lastPedalVolts[v] + (volts - lastInputVolts[v]) : volts;
            const float low = fromC, high = 1.f + fromC;
            pedal = (std::fabs(wanted - high) < std::fabs(wanted - low)) ? high : low;
        } else {
            pedal = volts - std::floor(volts);
        }
        lastInputVolts[v] = volts;
        lastPedalVolts[v] = pedal;
        foldHistory[v] = true;
        return pedal;
    }

    void process(const ProcessArgs& args) override {
        if (appliedSampleRate != args.sampleRate) reinit(args.sampleRate);

        // ---- Voices: one per V/Oct or GATE channel ---------------------------------------
        const bool voctPatched = inputs[VOCT_INPUT].isConnected(), gatePatched = inputs[GATE_INPUT].isConnected();
        const int channels = std::max(1, std::max(inputs[VOCT_INPUT].getChannels(), inputs[GATE_INPUT].getChannels()));
        if (channels != voices) {
            const int previous = voices;
            voices = std::min(channels, kMaxVoices);
            if (voices > previous) startVoices(previous);
        }
        if (++controlCounter >= kControlBlock) {
            controlCounter = 0;
            controlBlock(false);
        }

        // ---- Pedals, gates, pitch hold and contours, per voice -----------------------------
        // The mouse plays voice 0; a patched GATE plays whatever V/Oct holds.
        const int key = mouseKey.load(std::memory_order_relaxed);
        alignas(16) float pitch[kVoiceSlots] = {}, loud[kVoiceSlots] = {}, filterEnv[kVoiceSlots] = {};
        bool gates[kVoiceSlots] = {};
        uint32_t lit = 0;
        for (int v = 0; v < voices; ++v) {
            bool gate = false;
            if (gatePatched) {
                gateTriggers[v].process(inputs[GATE_INPUT].getPolyVoltage(v), 0.1f, 1.f);
                gate = gateTriggers[v].isHigh();
            }
            if (v == 0 && key >= 0) {
                gate = true;
                targetVolts[0] = key / 12.f;
            } else if (voctPatched) {
                targetVolts[v] = pedalVolts(v, inputs[VOCT_INPUT].getPolyVoltage(v));
            }
            gates[v] = gate;
            pitch[v] = hold[v].process(gate, targetVolts[v], followCoeff[v], droopPerSample) + octaveVolts[v];
            loud[v] = loudContour[v].process(gate, loudLevel[v], loudAttackCoeff[v], loudReleaseCoeff[v]);
            filterEnv[v] = filterContour[v].process(gate, filterLevel[v], filterAttackCoeff[v], filterReleaseCoeff[v]);
            if (gate) {
                const int nearest = (int)std::round(targetVolts[v] * 12.f);
                if (nearest >= 0 && nearest < kKeys) lit |= 1u << nearest;
            }
        }
        litKeys.store(lit, std::memory_order_relaxed);

        // ---- Voice 0, the mono engine ------------------------------------------------------
        // Its part tolerances are the calibrated unit's, so it carries no
        // cutoff spread.
        const bool auxPatched = inputs[AUX_INPUT].isConnected();
        const bool summed = params[OUT_MODE_PARAM].getValue() > 0.5f;
        alignas(16) float out[kVoiceSlots] = {};
        {
            Smoothed& m = voice0Smoothed;
            m.weightA += smoothK * (weightATarget[0] - m.weightA);
            m.weightB += smoothK * (weightBTarget[0] - m.weightB);
            m.emphasis += smoothK * (emphasisTarget[0] - m.emphasis);
            m.cutoffOffset += smoothK * (cutoffOffsetTarget[0] - m.cutoffOffset);
            m.footFilter += smoothK * (footFilterTarget[0] - m.footFilter);
            m.footLoudness += smoothK * (footLoudnessTarget[0] - m.footLoudness);
            m.level += smoothK * (levelTarget[0] - m.level);
            m.outputGain += smoothK * (outputGainTarget[0] - m.outputGain);
            m.filterFm += smoothK * (filterFmTarget[0] - m.filterFm);
            m.subBlend += smoothK * (subBlendTarget[0] - m.subBlend);
            m.crossFm += smoothK * (crossFmTarget[0] - m.crossFm);

            vega::VoiceControls c;
            c.frequencyA = kLowCHz * dsp::exp2_taylor5(clamp(pitch[0] + tuneVolts[0], -4.f, 8.f));
            c.frequencyB = c.frequencyA * ratioB[0] * dsp::exp2_taylor5(beatCentsPerOctave[0] * (1.f - pitch[0]) / 1200.f);
            c.weightA = m.weightA;
            c.weightB = m.weightB;
            // Foot FILTER sets the base; the programmed CUT-OFF, the contour
            // and the held pitch (R511, relative to the high C pedal) add.
            c.cutoffHz = kCutoffMinHz * dsp::exp2_taylor5(m.footFilter * kFootOctaves + m.cutoffOffset
                                                          + kContourOctaves * filterEnv[0] + kKeyboardTracking * (pitch[0] - 1.f));
            c.emphasis = m.emphasis;
            c.loudness = (auxFilterOnly ? 1.f : loud[0]) * m.footLoudness;
            c.aux = inputs[AUX_INPUT].getVoltage();
            c.level = m.level;
            c.filterFm = m.filterFm;
            c.subBlend = m.subBlend;
            c.crossFm = m.crossFm;
            out[0] = voice0.process(c) * m.outputGain;
            if (!std::isfinite(out[0]) || !voice0.ladder.finite()) {
                voice0.reset();
                out[0] = 0.f;
            }
        }

        // ---- Voices 1 .. 15, four at a time -----------------------------------------------
        for (int g = 0; 1 + 4 * g < voices; ++g) {
            const int first = 1 + 4 * g;
            weightA[g] += smoothK * (simd::float_4::load(&weightATarget[first]) - weightA[g]);
            weightB[g] += smoothK * (simd::float_4::load(&weightBTarget[first]) - weightB[g]);
            emphasis[g] += smoothK * (simd::float_4::load(&emphasisTarget[first]) - emphasis[g]);
            cutoffOffset[g] += smoothK * (simd::float_4::load(&cutoffOffsetTarget[first]) - cutoffOffset[g]);
            footFilter[g] += smoothK * (simd::float_4::load(&footFilterTarget[first]) - footFilter[g]);
            footLoudness[g] += smoothK * (simd::float_4::load(&footLoudnessTarget[first]) - footLoudness[g]);
            level[g] += smoothK * (simd::float_4::load(&levelTarget[first]) - level[g]);
            outputGain[g] += smoothK * (simd::float_4::load(&outputGainTarget[first]) - outputGain[g]);
            filterFm[g] += smoothK * (simd::float_4::load(&filterFmTarget[first]) - filterFm[g]);
            subBlend[g] += smoothK * (simd::float_4::load(&subBlendTarget[first]) - subBlend[g]);
            crossFm[g] += smoothK * (simd::float_4::load(&crossFmTarget[first]) - crossFm[g]);

            const simd::float_4 pitchVolts = simd::float_4::load(&pitch[first]);
            simd::float_4 frequencyA = kLowCHz * dsp::exp2_taylor5(simd::clamp(pitchVolts + simd::float_4::load(&tuneVolts[first]), -4.f, 8.f));
            simd::float_4 frequencyB = frequencyA * simd::float_4::load(&ratioB[first])
                * dsp::exp2_taylor5(simd::float_4::load(&beatCentsPerOctave[first]) * (1.f - pitchVolts) / 1200.f);
            const simd::float_4 cutoffHz = kCutoffMinHz * dsp::exp2_taylor5(footFilter[g] * kFootOctaves + cutoffOffset[g]
                + kContourOctaves * simd::float_4::load(&filterEnv[first]) + kKeyboardTracking * (pitchVolts - 1.f)
                + simd::float_4::load(&cutoffSpreadOctaves[first]));
            const simd::float_4 contour = auxFilterOnly ? simd::float_4(1.f) : simd::float_4::load(&loud[first]);

            // Lanes past the last voice stay silent and still.
            const int lanesInUse = std::min(4, voices - first);
            float active[4], aux[4];
            for (int lane = 0; lane < 4; ++lane) {
                active[lane] = (lane < lanesInUse) ? 1.f : 0.f;
                aux[lane] = (auxPatched && lane < lanesInUse) ? inputs[AUX_INPUT].getPolyVoltage(first + lane) : 0.f;
            }
            const simd::float_4 activeLanes = simd::float_4::load(active);

            vega::GroupControls c;
            float laneA[4], laneB[4];
            frequencyA.store(laneA);
            frequencyB.store(laneB);
            for (int lane = 0; lane < 4; ++lane) { c.frequencyA[lane] = laneA[lane]; c.frequencyB[lane] = laneB[lane]; }
            c.weightA = weightA[g] * activeLanes;
            c.weightB = weightB[g] * activeLanes;
            c.cutoffHz = cutoffHz;
            c.emphasis = emphasis[g] * activeLanes;
            c.loudness = contour * footLoudness[g] * activeLanes;
            c.aux = simd::float_4::load(aux);
            c.level = level[g];
            c.filterFm = filterFm[g];
            c.subBlend = subBlend[g];
            crossFm[g].store(c.crossFm);
            simd::float_4 groupOut = groups[g].process(c, lanesInUse) * outputGain[g];
            groupOut.store(&out[first]);

            bool finite = groups[g].ladder.finite();
            for (int lane = 0; lane < 4; ++lane) finite = finite && std::isfinite(out[first + lane]);
            if (!finite) {
                groups[g].reset();
                for (int lane = 0; lane < 4; ++lane) out[first + lane] = 0.f;
            }
        }

        // ---- Outputs ---------------------------------------------------------------------------
        // Limiter: linear to the knee, rounding off toward the ceiling.
        auto limit = [](float x) {
            const float magnitude = std::fabs(x);
            if (magnitude <= kLimitKnee) return x;
            const float room = kLimitCeiling - kLimitKnee;
            const float over = (magnitude - kLimitKnee) / room;
            return std::copysign(kLimitKnee + room * over / std::sqrt(1.f + over * over), x);
        };
        if (summed) {
            // 1 / sqrt(n) for n voices.
            float sum = 0.f;
            for (int v = 0; v < voices; ++v) sum += out[v];
            outputs[OUT_OUTPUT].setChannels(1);
            outputs[OUT_OUTPUT].setVoltage(limit(sum / std::sqrt((float)voices)));
        } else {
            outputs[OUT_OUTPUT].setChannels(voices);
            for (int v = 0; v < voices; ++v) outputs[OUT_OUTPUT].setVoltage(limit(out[v]), v);
        }
        outputs[VOCT_OUTPUT].setChannels(voices);
        outputs[GATE_OUTPUT].setChannels(voices);
        for (int v = 0; v < voices; ++v) {
            outputs[VOCT_OUTPUT].setVoltage(pitch[v] - 3.f, v);
            outputs[GATE_OUTPUT].setVoltage(gates[v] ? 10.f : 0.f, v);
        }
    }
};


// =============================================================================
struct VegaWidget : ModuleWidget {

    // The set's slider, lit while it is live.
    struct VegaSliderBase : app::SvgSlider {
        VegaSliderBase() {
            setBackgroundSvg(Svg::load(asset::plugin(pluginInstance, "res/components/ShortSlider.svg")));
            setHandleSvg(Svg::load(asset::plugin(pluginInstance, "res/components/ShortSliderHandle.svg")));
            setHandlePosCentered(math::Vec(10.f, 55.f), math::Vec(10.f, 10.f));
        }
    };
    template <typename TL>
    struct VegaSlider : LightSlider<VegaSliderBase, VCVSliderLight<TL>> { VegaSlider() {} };

    // One pedal. Pressing plays it; dragging across the keys slides from
    // pedal to pedal, legato, as a foot would.
    struct PedalKey : OpaqueWidget {
        Vega* module = nullptr;
        int note = 0;
        bool black = false;

        void drawLayer(const DrawArgs& args, int layer) override {
            if (layer != 1) return;
            bool lit = module && ((module->litKeys.load(std::memory_order_relaxed) >> note) & 1u);
            nvgBeginPath(args.vg);
            nvgRoundedRect(args.vg, 0.5f, 0.f, box.size.x - 1.f, box.size.y - 0.5f, 1.5f);
            if (lit) nvgFillColor(args.vg, nvgRGB(208, 140, 89));
            else nvgFillColor(args.vg, black ? nvgRGB(24, 24, 24) : nvgRGB(160, 160, 160));
            nvgFill(args.vg);
            nvgStrokeWidth(args.vg, 1.f);
            nvgStrokeColor(args.vg, nvgRGB(12, 12, 12));
            nvgStroke(args.vg);
        }
        void onDragStart(const event::DragStart& e) override {
            if (e.button == GLFW_MOUSE_BUTTON_LEFT && module) module->mouseKey.store(note);
            OpaqueWidget::onDragStart(e);
        }
        void onDragEnter(const event::DragEnter& e) override {
            if (module && dynamic_cast<PedalKey*>(e.origin)) module->mouseKey.store(note);
            OpaqueWidget::onDragEnter(e);
        }
        void onDragEnd(const event::DragEnd& e) override {
            if (e.button == GLFW_MOUSE_BUTTON_LEFT && module) module->mouseKey.store(-1);
            OpaqueWidget::onDragEnd(e);
        }
    };

    VegaWidget(Vega* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Vega.svg"),
            asset::plugin(pluginInstance, "res/Vega-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // 28HP, positions in mm. All text is panel artwork.
        const float panelWidth = 28.f * 5.08f;

        // ---- Top row under the name, mirrored about the centre ----
        // Left: V/OCT and GATE in, then GRIT with its trim and CV stepping
        // down toward the centre. Right, mirrored: SNARL's CV, trim and
        // knob, then the held GATE and V/OCT. The two halves make a shallow V
        // under the name.
        const float yTop = 18.f, jackPitch = 10.16f;
        const float yTopTrim = yTop + 2.5f, yTopCv = yTop + 5.f;
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(8.f, yTop-4)), module, Vega::VOCT_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(10.f + jackPitch, yTop-4)), module, Vega::GATE_INPUT));
        const float gritX = 33.f, topTrimOffset = 10.f, topCvOffset = 18.5f;
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(gritX+3, yTop-4)), module, Vega::STRIP_PARAM + STRIP_GRIT));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(gritX + topTrimOffset+3, yTopTrim-4)), module, Vega::STRIP_TRIM_PARAM + (STRIP_GRIT - kSliderStrips)));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(gritX + topCvOffset+9, yTopCv)-4), module, Vega::STRIP_CV_INPUT + STRIP_GRIT));

        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(panelWidth - 8.f, yTop-4)), module, Vega::VOCT_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(panelWidth - 10.f - jackPitch, yTop-4)), module, Vega::GATE_OUTPUT));
        const float snarlX = panelWidth - gritX;
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(snarlX-3, yTop-4)), module, Vega::STRIP_PARAM + STRIP_SNARL));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(snarlX - topTrimOffset-3, yTopTrim-4)), module, Vega::STRIP_TRIM_PARAM + (STRIP_SNARL - kSliderStrips)));
        // Mirrors GRIT's CV (whose Vec(...) - 4 moves it 4 mm left and up).
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(snarlX - topCvOffset-9, yTopCv)+Vec(4, -4)), module, Vega::STRIP_CV_INPUT + STRIP_SNARL));

        // ---- Programmed preset console, after the hardware ----
        // TUNE, BEAT and GLIDE stacked as knobs, each with its CV on its
        // left; the OCTAVE range switch; then ten sliders, each over its CV,
        // spaced so the console is centred on the panel (outer jack edges
        // 4 mm in from each side). The slider's drawn track runs from 13 mm
        // above its centre to 7.8 mm below, so the switch sits on the
        // track's middle and the CV clears the track's end.
        const float xKnobCv = 8.f, xKnobs = 20.5f, xRange = 33.f, xFirstSlider = 41.f;
        const float sliderPitch = (panelWidth - 8.f - xFirstSlider) / 9.f;
        const float ySlider = 47.f, yTrackMiddle = ySlider - 2.6f, ySliderCv = ySlider + 13.5f;
        const float yKnobs[3] = { 37.f, 52.5f, 68.f };
        const int knobParams[3] = { Vega::TUNE_PARAM, Vega::BEAT_PARAM, Vega::STRIP_PARAM + STRIP_GLIDE };
        const int knobInputs[3] = { Vega::TUNE_CV_INPUT, Vega::BEAT_CV_INPUT, Vega::STRIP_CV_INPUT + STRIP_GLIDE };
        for (int k = 0; k < 3; ++k) {
            addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(xKnobs, yKnobs[k])), module, knobParams[k]));
            addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(xKnobCv, yKnobs[k])), module, knobInputs[k]));
        }

        addParam(createParamCentered<CKSSThree>(mm2px(Vec(xRange, yTrackMiddle)), module, Vega::RANGE_PARAM));

        for (int k = 0; k < 10; ++k) {
            const int strip = STRIP_MIX + k;
            const float x = xFirstSlider + k * sliderPitch;
            // The volume sliders carry red caps, as on the hardware.
            if (strip >= STRIP_VOLUME_ATTACK && strip <= STRIP_VOLUME_DECAY)
                addParam(createLightParamCentered<VegaSlider<RedLight>>(mm2px(Vec(x, ySlider)), module,
                                                                       Vega::STRIP_PARAM + strip, Vega::STRIP_LIGHT + strip));
            else
                addParam(createLightParamCentered<VegaSlider<WhiteLight>>(mm2px(Vec(x, ySlider)), module,
                                                                         Vega::STRIP_PARAM + strip, Vega::STRIP_LIGHT + strip));
            addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(x, ySliderCv)), module, Vega::STRIP_CV_INPUT + strip));
        }

        // ---- Mode buttons, directly over the pedals, centred on the panel ----
        // The preset CV, VARIABLE VEGA TUBA BASS, a gap, GLIDE DECAY OCTAVE.
        const float yButtons = 80.f, buttonPitch = 9.5f, buttonGroupGap = 14.f, presetCvGap = 10.f;
        const float buttonsLeft = 0.5f * (panelWidth - (presetCvGap + 5.f * buttonPitch + buttonGroupGap));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(buttonsLeft, yButtons)), module, Vega::MODE_CV_INPUT));
        const float presetsLeft = buttonsLeft + presetCvGap;
        for (int k = 0; k < 4; ++k)
            addParam(createLightParamCentered<VCVLightBezel<YellowLight>>(mm2px(Vec(presetsLeft + k * buttonPitch, yButtons)), module,
                                                                      Vega::VARIABLE_BUTTON + k, Vega::VARIABLE_LIGHT + k));
        for (int k = 0; k < 3; ++k)
            addParam(createLightParamCentered<VCVLightBezelLatch<RedLight>>(
                mm2px(Vec(presetsLeft + 3.f * buttonPitch + buttonGroupGap + k * buttonPitch, yButtons)), module,
                Vega::GLIDE_PARAM + k, Vega::GLIDE_LIGHT + k));

        // ---- The foot controls flank the pedals: VOLUME left, FILTER right ----
        // Each knob sits out at the panel's edge (its rim in line with the
        // outer jacks' rims); its trim and CV step down and inward toward the
        // pedals, the lower arms of the V, the trim just outside the CV. The
        // CVs stay level with AUX IN and OUT.
        const float hugeKnobRadius = 9.12f;
        const float footKnobInset = 4.f + hugeKnobRadius, footCvInset = 21.5f+6;
        const float yFootKnob = 90.5f, yFootTrim = 104.f, yFootCv = 114.5f;
        const int footStrips[2] = { STRIP_VOLUME, STRIP_FILTER };
        for (int k = 0; k < 2; ++k) {
            // k = 0 measures from the left edge, k = 1 from the right.
            const float side = (k == 0) ? 1.f : -1.f, edge = (k == 0) ? 0.f : panelWidth;
            addParam(createParamCentered<RoundHugeBlackKnob>(mm2px(Vec(edge + side * footKnobInset, yFootKnob)), module, Vega::STRIP_PARAM + footStrips[k]));
            addParam(createParamCentered<Trimpot>(mm2px(Vec(edge + side * (footCvInset-2), yFootTrim)), module, Vega::STRIP_TRIM_PARAM + (footStrips[k] - kSliderStrips)));
            addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(edge + side * footCvInset, yFootCv)), module, Vega::STRIP_CV_INPUT + footStrips[k]));
        }

        // ---- AUX IN bottom left, OUT bottom right, level with the foot CVs ----
        // AUX IN's mode button sits over it, level with the foot trims.
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(8.f, yFootCv)), module, Vega::AUX_INPUT));
        addParam(createLightParamCentered<VCVLightBezelLatch<RedLight>>(mm2px(Vec(8.f, yFootTrim)), module,
                                                                        Vega::AUX_MODE_PARAM, Vega::AUX_MODE_LIGHT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(panelWidth - 8.f, yFootCv)), module, Vega::OUT_OUTPUT));

        // ---- The 13 pedals, C to C: eight white, five black, black on top ----
        const float whiteWidth = 9.f, whiteHeight = 21.5f, blackWidth = 5.6f, blackHeight = 13.f;
        const float keyboardLeft = 0.5f * (panelWidth - 8.f * whiteWidth), keyboardTop = 93.5f;
        const int whiteNotes[8] = { 0, 2, 4, 5, 7, 9, 11, 12 };
        for (int k = 0; k < 8; ++k) {
            PedalKey* pedal = createWidget<PedalKey>(mm2px(Vec(keyboardLeft + k * whiteWidth, keyboardTop)));
            pedal->box.size = mm2px(Vec(whiteWidth, whiteHeight));
            pedal->module = module;
            pedal->note = whiteNotes[k];
            addChild(pedal);
        }
        // Black keys sit over the line after white key index k.
        const int blackNotes[5] = { 1, 3, 6, 8, 10 };
        const int blackAfter[5] = { 0, 1, 3, 4, 5 };
        for (int k = 0; k < 5; ++k) {
            const float centre = keyboardLeft + (blackAfter[k] + 1) * whiteWidth;
            PedalKey* pedal = createWidget<PedalKey>(mm2px(Vec(centre - 0.5f * blackWidth, keyboardTop)));
            pedal->box.size = mm2px(Vec(blackWidth, blackHeight));
            pedal->module = module;
            pedal->note = blackNotes[k];
            pedal->black = true;
            addChild(pedal);
        }
    }

    void appendContextMenu(Menu* menu) override {
        Vega* vegaModule = getModule<Vega>();
        if (!vegaModule) return;
        menu->addChild(new MenuSeparator);
        menu->addChild(createIndexSubmenuItem("Oversampling", { "Off (1x)", "2x (default)", "4x" },
            [=]() { return (size_t)clamp((int)std::round(vegaModule->params[Vega::OVERSAMPLE_PARAM].getValue()), 0, 2); },
            [=](size_t index) { vegaModule->params[Vega::OVERSAMPLE_PARAM].setValue((float)index); }));
        menu->addChild(createIndexSubmenuItem("OUT", { "Polyphonic, a channel per voice (default)", "Summed to mono" },
            [=]() { return (size_t)clamp((int)std::round(vegaModule->params[Vega::OUT_MODE_PARAM].getValue()), 0, 1); },
            [=](size_t index) { vegaModule->params[Vega::OUT_MODE_PARAM].setValue((float)index); }));
        menu->addChild(createBoolMenuItem("Voice variation (each voice its own part tolerances)", "",
            [=]() { return vegaModule->params[Vega::VARIATION_PARAM].getValue() > 0.5f; },
            [=](bool on) { vegaModule->params[Vega::VARIATION_PARAM].setValue(on ? 1.f : 0.f); }));
        menu->addChild(createIndexSubmenuItem("V/Oct range", { "Folded onto the pedals, C1 to C2 (default)", "Full range" },
            [=]() { return (size_t)clamp((int)std::round(vegaModule->params[Vega::PEDAL_RANGE_PARAM].getValue()), 0, 1); },
            [=](size_t index) { vegaModule->params[Vega::PEDAL_RANGE_PARAM].setValue((float)index); }));
    }
};

Model* modelVega = createModel<Vega, VegaWidget>("Vega");