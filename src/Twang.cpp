////////////////////////////////////////////////////////////
//
//   Twang
//
//   written by Cody Geary
//   Copyright 2026
//   Released under the MIT License
//
//   Polyphonic single-string physical model with both bowed and plucked excitations.
//
////////////////////////////////////////////////////////////

#include "plugin.hpp"
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <vector>
#include <functional>
#include "FilterTwang.h"

static constexpr int TWANG_MAX_POLY = 16;
static constexpr int TWANG_QUADS    = TWANG_MAX_POLY / 4;

// Stereo: the body is MONO. The image comes from a mid-side width built
// out of two short TwangSpaceDelay lines fed by the same core
// coherent L/R around one central object.

static constexpr float TWANG_BOW_DEAD_ZONE = 0.005f;

// How long a pluck keeps the module awake no matter what the output reads.
// Must cover the worst-case pluck-point-to-bridge travel plus the body's response.
static constexpr float TWANG_PLUCK_WAKE_SEC = 0.25f;

// Gain staging and Voicing constants
static constexpr float TWANG_TONE_COMPENSATION = 2.0f;   // gain lift as TONE darkens
static constexpr float TWANG_PLUCK_BRIGHTEN    = 0.35f;  // velocity -> HARDNESS
static constexpr float TWANG_BOW_POSITION_MIN  = 0.08f;  // contact range, off the
static constexpr float TWANG_BOW_POSITION_MAX  = 0.92f;  // unstable ends of the string
static constexpr float TWANG_BRIDGE_COUPLING   = 0.35f;  // body load in the reflection

// Highest lowest-mode the BRIDGE ADMITTANCE bank is allowed to track, in Hz.
//
// The admittance is subtracted from the string's reflection, and its five
// modes run to ratio 5.60. The stability sweep behind setCoupling()'s 0.7
// ceiling was run over a body range topping out near here, where those modes
// still sit inside the bridge low-pass's passband and the direct term
// dominates the reflection. Push the body far above it and the top admittance
// modes land where TONE has already rolled the direct term away, leaving the
// subtraction to set |r| on its own -- which is how the string starts feeding
// back. The body the listener hears may go higher; the string's termination
// stops following it here.
static constexpr float TWANG_BRIDGE_BODY_MAX_HZ = 700.f;

static constexpr float TWANG_INPUT_GAIN    = 0.4f;
static constexpr float TWANG_OUTPUT_MAKEUP = 8.5f;

// Baseline string damping. Expressed as a decay TIME 
static constexpr float TWANG_STRING_STIFFNESS = 0.06f;

// Phase delay the dispersion allpasses add to the loop, in samples.
//
// A first-order allpass (a + z^-1)/(1 + a z^-1) has phase delay (1-a)/(1+a)
// at DC, so two of them at a = 0.06 contribute 1.774 samples. This is a
// constant, unlike the bridge filter's contribution, which tracks TONE.
static constexpr float TWANG_DISPERSION_DELAY =
    TwangStringSIMD::DISPERSION_STAGES * (1.f - TWANG_STRING_STIFFNESS)
                                       / (1.f + TWANG_STRING_STIFFNESS);

// --------------------------------------------------------------------------
// Twang module
// --------------------------------------------------------------------------
struct Twang : Module {

    enum ParamIds {
        FREQ_PARAM,
        // Left rail A: knob + trim + CV trios
        BOW_SPEED_PARAM, BOW_SPEED_TRIM_PARAM,
        MUTE_PARAM,      MUTE_TRIM_PARAM,
        TWANG_PARAM,     TWANG_TRIM_PARAM,
        // Left rail B
        VOLUME_PARAM,
        MUTE_BUTTON_PARAM,
        PLUCK_BUTTON_PARAM,
        FM_TRIM_PARAM,
        // Bow row sliders
        BOW_POSITION_PARAM,    BOW_POSITION_TRIM_PARAM,
        BOW_PRESSURE_PARAM,    BOW_PRESSURE_TRIM_PARAM,
        ATTACK_PARAM,            ATTACK_TRIM_PARAM,
        SLIP_PARAM,              SLIP_TRIM_PARAM,
        TONE_PARAM,            TONE_TRIM_PARAM,
        // Pluck row sliders
        PLUCK_POSITION_PARAM,     PLUCK_POSITION_TRIM_PARAM,
        PLUCK_HARDNESS_PARAM,     PLUCK_HARDNESS_TRIM_PARAM,
        BOW_LEVEL_PARAM,          BOW_LEVEL_TRIM_PARAM,
        DECAY_PARAM,              DECAY_TRIM_PARAM,
        RESONATOR_SIZE_PARAM,     RESONATOR_SIZE_TRIM_PARAM,
        RESONATOR_MATERIAL_PARAM, RESONATOR_MATERIAL_TRIM_PARAM,
        DRIVE_PARAM,              DRIVE_TRIM_PARAM,
        // Appended rather than slotted in with the other rail B entries, so
        // every existing param index keeps its number and saved patches load.
        PLUCK_VELOCITY_PARAM,
        NUM_PARAMS
    };

    enum InputIds {
        VOCT_INPUT,
        PLUCK_TRIG_INPUT,
        FM_CV_INPUT,
        BOW_SPEED_CV_INPUT,
        MUTE_CV_INPUT,
        TWANG_CV_INPUT,
        VOLUME_CV_INPUT,
        BOW_POSITION_CV_INPUT,
        BOW_PRESSURE_CV_INPUT,
        ATTACK_CV_INPUT,
        SLIP_CV_INPUT,
        TONE_CV_INPUT,
        PLUCK_POSITION_CV_INPUT,
        PLUCK_HARDNESS_CV_INPUT,
        BOW_LEVEL_CV_INPUT,
        DECAY_CV_INPUT,
        RESONATOR_SIZE_CV_INPUT,
        RESONATOR_MATERIAL_CV_INPUT,
        DRIVE_CV_INPUT,
        NUM_INPUTS
    };

    enum OutputIds {
        OUT_L,
        OUT_R,
        NUM_OUTPUTS
    };

    enum LightIds {
        MUTE_LIGHT,          // green: mute DEPTH from the knob / CV
        MUTE_BUTTON_LIGHT,   // red:   the latch is physically engaged
        PLUCK_LIGHT,
        NUM_LIGHTS
    };

    // 16 voices as 4 quads of 4 symmetric SIMD lanes. Voice v lives in
    // quad v/4, lane v%4. Every lane runs identical physics -- only pitch
    // and pluck state differ 
    TwangStringSIMD voiceQuad[TWANG_QUADS];
    dsp::SchmittTrigger pluckTrigger[TWANG_MAX_POLY];
    float a_segmentSamples[TWANG_MAX_POLY];

    // Output chain: a single MONO core -- voice sum -> DC block -> drive ->
    // body resonator -- then a subtle mid-side width. The stereo image is
    // built from two short feedback delay lines fed by that same core: their
    // difference is mixed as the SIDE channel only, so L/R stay coherent
    // (one instrument in a room, mono sum = 2x core) instead of being two
    // independent resonators.
    TwangDCBlocker   dcBlocker;
    TwangADAADrive   outputDrive;
    // Modal body. 
    TwangBodyModal   body;
    TwangSpaceDelay  spaceL, spaceR;
    // Per-voice output chain for Poly Out, one quad at a time. The bodies
    // share their coefficients across lanes 
    TwangDCBlockerSIMD  polyDC[TWANG_QUADS];
    TwangADAADriveSIMD  polyDrive[TWANG_QUADS];
    TwangBodyModalSIMD  polyBody[TWANG_QUADS];
    float_4             polyVoice[TWANG_QUADS];

    float sampleRate = 48000.f;
    int   nVoices    = 1;
    int   prevVoices = 1;
    // Smoothed 1/sqrt(sounding voices); see the poly normalisation in process().
    float a_polyNormalize = 1.f;
    // Samples for which a pluck forces the voice awake regardless of output
    // level. See the sleep re-arm in process() for why this is needed.
    int   wakeHold = 0;
    int   nQuads     = 1;

    static constexpr int CTRL_SKIP = 32;
    int ctrlDivCounter = 0;

    // Cached control-rate values. 
    float_4 cachedToneCoeff[TWANG_QUADS];
    float_4 cachedLossPerSample[TWANG_QUADS];
    // Extra phase delay the loop's filters add on top of 2 * segmentSamples,
    // per lane. Subtracted from the segment target so the pitch is right.
    float_4 cachedLoopExtra[TWANG_QUADS];
    float   muteDepthV[TWANG_MAX_POLY] = {};
    // Open-string T60 from the DECAY slider, pitch-independent. MUTE scales
    // it down further.
    float stringDecaySeconds  = 5.f;
    float cachedDriveGain     = 1.f;
    float cachedVolume        = 0.8f;
    float cachedLowestMode    = 131.f;   // lowest body mode, Hz (RESONATOR SIZE)
    
    // Bridge admittance coupling: how hard the body loads the string termination. 
    float bowLevel            = 0.7f;
    float pluckTilt           = 0.6f;
    // Pluck level. 
    float pluckDrive          = 12.0f;   // now driven by the PLUCK LEVEL slider
    // Bow coupling. Moved off the panel to the context menu -- it is a voicing
    // constant more than a performance control. Stored as the final value, not
    // a 0..1 slider position; the old panel default (0.31) mapped to 0.641.
    float stringImpedance     = 0.641f;
    // How much MATERIAL tilts body mode amplitude toward the upper modes.
    // 1.0 is the original behaviour; lower values hand brightness back to TONE.
    float bodyTilt            = 0.7f;
    // How hard the body loads the string's bridge termination. 
    float bridgeCoupling      = TWANG_BRIDGE_COUPLING;
    // Per-voice bow speed. BOW SPEED CV is polyphonic so a MIDI-driven patch
    // can bow each note separately
    float t_bowSpeedV[TWANG_MAX_POLY] = {};
    float a_bowSpeedV[TWANG_MAX_POLY] = {};
    float cachedTwangDepth    = 0.f;
    float cachedPluckHardness = 0.5f;
    // Per-voice pluck controls. Only filled, and only read, when the matching
    // CV input is polyphonic; otherwise the scalars above are used and these
    // stay untouched.
    bool  pluckPosPoly        = false;
    bool  pluckHardPoly       = false;
    float pluckPositionV[TWANG_MAX_POLY] = {};
    float pluckHardnessV[TWANG_MAX_POLY] = {};
    float cachedBaseVOct      = 0.f;
    float cachedToneCompensationGain = 1.f;
    // Scalar copies of the two contact positions, for the display only.
    float cachedBowPosition   = 0.75f;
    float cachedPluckPosition = 0.25f;
    float t_bowSpeed = 0.f;
    float a_bowSpeed = 0.f;
    bool  asleep     = true;
    bool  pluckButtonWasDown = false;

    // Context-menu tunables
    float fmDepthSemitones       = 1.f;
    float twangMaxBendSemitones  = 3.0f;  // pitch rise at full TWANG and full attack transient
    // Tension bounce. Past 12 semitones of bend the string is being stretched
    // far enough that a real one would not settle straight back, so the
    // rebound is scheduled to grow on its own from there -- Max Bend past 12
    // buys weirdness as well as range.
    float twangBounceAmount      = 0.5f;  // 0 = the old monotonic bend
    float twangBounceHz          = 7.0f;  // rebound rate at Max Bend <= 12
    float cachedBounceDepth      = 0.f;
    float stereoWidth            = 0.35f; // mid-side width of the stereo output
    // Poly output: OUT L carries one channel per voice instead of the stereo
    // mix, for patches that want to pan or process each string separately.
    //
    // Each voice gets the full chain including its own body, via
    // TwangBodyModalSIMD -- four lanes sharing one set of coefficients, so a
    // quad of bodies costs about what one scalar body did. Only allocated
    // work when this is on; the mono path is untouched.
    bool  polyOutput             = false;

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
        json_object_set_new(rootJ, "fmDepthSemitones",       json_real(fmDepthSemitones));
        json_object_set_new(rootJ, "pluckTilt",              json_real(pluckTilt));
        json_object_set_new(rootJ, "pluckDrive",             json_real(pluckDrive));
        json_object_set_new(rootJ, "stringImpedance",        json_real(stringImpedance));
        json_object_set_new(rootJ, "bodyTilt",               json_real(bodyTilt));
        json_object_set_new(rootJ, "bridgeCoupling",         json_real(bridgeCoupling));
        json_object_set_new(rootJ, "twangMaxBendSemitones",  json_real(twangMaxBendSemitones));
        json_object_set_new(rootJ, "twangBounceAmount",      json_real(twangBounceAmount));
        json_object_set_new(rootJ, "twangBounceHz",          json_real(twangBounceHz));
        json_object_set_new(rootJ, "stereoWidth",            json_real(stereoWidth));
        json_object_set_new(rootJ, "polyOutput",             json_boolean(polyOutput));
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        json_t* j;
        j = json_object_get(rootJ, "fmDepthSemitones");
        if (j) fmDepthSemitones = clamp((float)json_real_value(j), 0.f, 24.f);
        j = json_object_get(rootJ, "pluckTilt");
        if (j) pluckTilt = clamp((float)json_real_value(j), 0.f, 1.f);
        j = json_object_get(rootJ, "pluckDrive");
        if (j) pluckDrive = clamp((float)json_real_value(j), 1.f, 30.f);
        j = json_object_get(rootJ, "stringImpedance");
        if (j) stringImpedance = clamp((float)json_real_value(j), 0.30f, 1.40f);
        j = json_object_get(rootJ, "bodyTilt");
        if (j) bodyTilt = clamp((float)json_real_value(j), 0.f, 1.f);
        j = json_object_get(rootJ, "bridgeCoupling");
        if (j) bridgeCoupling = clamp((float)json_real_value(j), 0.f, 0.7f);
        j = json_object_get(rootJ, "twangMaxBendSemitones");
        if (j) twangMaxBendSemitones = clamp((float)json_real_value(j), 0.f, 24.f);
        j = json_object_get(rootJ, "twangBounceAmount");
        if (j) twangBounceAmount = clamp((float)json_real_value(j), 0.f, 2.f);
        j = json_object_get(rootJ, "twangBounceHz");
        if (j) twangBounceHz = clamp((float)json_real_value(j), 0.5f, 30.f);
        j = json_object_get(rootJ, "stereoWidth");
        if (j) stereoWidth = clamp((float)json_real_value(j), 0.f, 1.f);
        j = json_object_get(rootJ, "polyOutput");
        if (j) polyOutput = json_is_true(j);
    }

    Twang() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        configParam(FREQ_PARAM, -2.f, 2.f, 0.f, "Frequency", " V");

        configParam(BOW_SPEED_PARAM,      -1.f, 1.f, 0.f,  "Bow Speed");
        configParam(BOW_SPEED_TRIM_PARAM, -1.f, 1.f, 0.f,  "Bow Speed CV Trim");
        configParam(MUTE_PARAM,            0.f, 1.f, 0.f,  "Mute (palm mute depth)");
        configParam(MUTE_TRIM_PARAM,      -1.f, 1.f, 0.f,  "Mute CV Trim");
        configParam(TWANG_PARAM,           0.f, 1.f, 0.f,  "Twang (tension bend)");
        configParam(TWANG_TRIM_PARAM,     -1.f, 1.f, 0.f,  "Twang CV Trim");

        configParam(VOLUME_PARAM,       0.f, 1.f, 0.80f, "Volume");
        configSwitch(MUTE_BUTTON_PARAM,  0.f, 1.f, 0.f,  "Mute",  {"Open", "Muted"});
        configParam(PLUCK_BUTTON_PARAM,  0.f, 1.f, 0.f,  "Pluck");
        // Full-scale pluck velocity, in volts of trigger. The PLUCK TRIG jack
        // still reads its own voltage as velocity; this sets what a 10 V hit is
        // worth, so a patch driven by plain gates (which are all 10 V and would
        // otherwise every one of them be a maximum-force pluck) gets a usable
        // dynamic level, and the panel button gets a level at all. Default 10 V
        // leaves CV-driven velocity exactly as it was.
        configParam(PLUCK_VELOCITY_PARAM, 0.f, 10.f, 10.f, "Max Pluck Velocity", " V");
        configParam(FM_TRIM_PARAM,      0.f, 1.f, 0.f,   "FM CV Trim");

        configParam(BOW_POSITION_PARAM,         0.f, 1.f, 0.75f, "Bow Position");
        configParam(BOW_POSITION_TRIM_PARAM,   -1.f, 1.f, 0.f,   "Bow Position CV Trim");
        configParam(BOW_PRESSURE_PARAM,         0.f, 1.f, 0.40f, "Bow Pressure");
        configParam(BOW_PRESSURE_TRIM_PARAM,   -1.f, 1.f, 0.f,   "Bow Pressure CV Trim");
        // Pluck Level drives the pluck exciter (was the "Pluck Drive" menu
        // float). The enum ids are still ATTACK_*; only the user-facing name
        // changed, so patches keep loading.
        // 0.379 maps to the old default of 12.
        configParam(ATTACK_PARAM,               0.f, 1.f, 0.379f, "Pluck Level");
        configParam(ATTACK_TRIM_PARAM,         -1.f, 1.f, 0.f,    "Pluck Level CV Trim");
        configParam(SLIP_PARAM,                 0.f, 1.f, 0.27f, "Slip (bow bite)");
        configParam(SLIP_TRIM_PARAM,           -1.f, 1.f, 0.f,   "Slip CV Trim");
        configParam(TONE_PARAM,                 0.f, 1.f, 0.70f, "Tone");
        configParam(TONE_TRIM_PARAM,           -1.f, 1.f, 0.f,   "Tone CV Trim");

        configParam(PLUCK_POSITION_PARAM,          0.f, 1.f, 0.25f, "Pluck Position");
        configParam(PLUCK_POSITION_TRIM_PARAM,    -1.f, 1.f, 0.f,   "Pluck Position CV Trim");
        configParam(PLUCK_HARDNESS_PARAM,          0.f, 1.f, 0.55f, "Pluck Hardness (finger - pick)");
        configParam(PLUCK_HARDNESS_TRIM_PARAM,    -1.f, 1.f, 0.f,   "Pluck Hardness CV Trim");
        configParam(BOW_LEVEL_PARAM,               0.f, 1.f, 0.43f, "Bow Level");
        configParam(BOW_LEVEL_TRIM_PARAM,         -1.f, 1.f, 0.f,   "Bow Level CV Trim");
        configParam(DECAY_PARAM,                   0.f, 1.f, 0.62f, "Pluck Decay");
        configParam(DECAY_TRIM_PARAM,             -1.f, 1.f, 0.f,   "Pluck Decay CV Trim");
        configParam(RESONATOR_SIZE_PARAM,          0.f, 1.f, 0.60f, "Resonator Size");
        configParam(RESONATOR_SIZE_TRIM_PARAM,    -1.f, 1.f, 0.f,   "Resonator Size CV Trim");
        configParam(RESONATOR_MATERIAL_PARAM,      0.f, 1.f, 0.20f, "Resonator Material (wood - metal)");
        configParam(RESONATOR_MATERIAL_TRIM_PARAM,-1.f, 1.f, 0.f,   "Resonator Material CV Trim");
        configParam(DRIVE_PARAM,                   0.f, 1.f, 0.f,   "Drive");
        configParam(DRIVE_TRIM_PARAM,             -1.f, 1.f, 0.f,   "Drive CV Trim");

        configInput(VOCT_INPUT,       "V/Oct (polyphonic, sets voice count)");
        configInput(PLUCK_TRIG_INPUT, "Pluck Trigger (polyphonic, voltage sets velocity)");
        configInput(FM_CV_INPUT,      "FM CV");
        configInput(BOW_SPEED_CV_INPUT,         "Bow Speed CV");
        configInput(MUTE_CV_INPUT,              "Mute CV");
        configInput(TWANG_CV_INPUT,             "Twang CV");
        configInput(VOLUME_CV_INPUT,            "Volume CV");
        configInput(BOW_POSITION_CV_INPUT,      "Bow Position CV");
        configInput(BOW_PRESSURE_CV_INPUT,      "Bow Pressure CV");
        configInput(ATTACK_CV_INPUT,            "Pluck Level CV");
        configInput(SLIP_CV_INPUT,              "Slip CV");
        configInput(TONE_CV_INPUT,              "Tone CV");
        configInput(PLUCK_POSITION_CV_INPUT,    "Pluck Position CV");
        configInput(PLUCK_HARDNESS_CV_INPUT,    "Pluck Hardness CV");
        configInput(BOW_LEVEL_CV_INPUT,         "Bow Level CV");
        configInput(DECAY_CV_INPUT,             "Pluck Decay CV");
        configInput(RESONATOR_SIZE_CV_INPUT,    "Resonator Size CV");
        configInput(RESONATOR_MATERIAL_CV_INPUT,"Resonator Material CV");
        configInput(DRIVE_CV_INPUT,             "Drive CV");

        configOutput(OUT_L, "Left");
        configOutput(OUT_R, "Right");

        initVoice();
    }

    void initVoice() {
        for (int q = 0; q < TWANG_QUADS; ++q) voiceQuad[q].init(sampleRate);
        for (int v = 0; v < TWANG_MAX_POLY; ++v) a_segmentSamples[v] = 200.f;
        body.init(sampleRate);
        dcBlocker.setSampleRate(sampleRate);
        dcBlocker.reset();
        outputDrive.reset();
        for (int q = 0; q < TWANG_QUADS; ++q) {
            polyDC[q].setSampleRate(sampleRate);
            polyDC[q].reset();
            polyDrive[q].reset();
            polyBody[q].init(sampleRate);
            // Give it real coefficients up front. The control block only
            // refreshes these while Poly Out is on, so without this the first
            // UI tick after switching it on would run an all-zero filter bank.
            polyBody[q].setBody(cachedLowestMode, 0.3f + 0.5f * 5.0f, 0.5f, sampleRate);
            polyVoice[q] = float_4(0.f);
        }
        spaceL.init(sampleRate, 0.006f);
        spaceR.init(sampleRate, 0.006f);
        // Fixed width lengths: two short, differently-delayed lines decorrelate
        // the side feed without audible smearing.
        spaceL.setDelay((int)roundf(0.0025f * sampleRate));
        spaceR.setDelay((int)roundf(0.004f  * sampleRate));
        a_bowSpeed = 0.f;
        t_bowSpeed = 0.f;
        wakeHold        = 0;
        a_polyNormalize = 1.f;
        for (int v = 0; v < TWANG_MAX_POLY; ++v) { a_bowSpeedV[v] = 0.f; t_bowSpeedV[v] = 0.f; }
        asleep     = true;
    }

    // initVoice() reallocates every delay line. Doing that only from process()
    // (as the sample-rate check below the audio block does) puts a heap
    // allocation on the audio thread. Rack fires this on the engine thread, so
    // the rate change is absorbed here; the in-process check stays as a net,
    // matching Strands.
    void onSampleRateChange() override {
        sampleRate = APP->engine->getSampleRate();
        initVoice();
    }

    void panic() {
        for (int q = 0; q < TWANG_QUADS; ++q) voiceQuad[q].panic();
        body.clear();
        dcBlocker.reset();
        outputDrive.reset();
        for (int q = 0; q < TWANG_QUADS; ++q) {
            polyDC[q].reset(); polyDrive[q].reset(); polyBody[q].clear();
            polyVoice[q] = float_4(0.f);
        }
        spaceL.clear();
        spaceR.clear();
        // Keep the button edge detector honest across a panic: a held button
        // must not re-strum the moment audio resumes.
        pluckButtonWasDown = params[PLUCK_BUTTON_PARAM].getValue() > 0.5f;
        a_bowSpeed      = 0.f;
        wakeHold        = 0;
        a_polyNormalize = 1.f;
        for (int v = 0; v < TWANG_MAX_POLY; ++v) { a_bowSpeedV[v] = 0.f; t_bowSpeedV[v] = 0.f; }
        asleep          = true;
    }

    void onReset() override {
        Module::onReset();
        fmDepthSemitones       = 1.f;
        twangMaxBendSemitones  = 3.0f;
        twangBounceAmount      = 0.5f;
        twangBounceHz          = 7.0f;
        stereoWidth            = 0.35f;
        polyOutput             = false;
        pluckDrive             = 12.0f;
        stringImpedance        = 0.641f;
        bodyTilt               = 0.7f;
        pluckTilt              = 0.6f;
        bridgeCoupling         = TWANG_BRIDGE_COUPLING;
        panic();
    }

    inline float vOctToSegmentSamples(float vOct) {
        constexpr float C4Hz = 261.63f;
        float freq = C4Hz * exp2f(vOct);
        return sampleRate / (2.f * freq);
    }

    // Slider + trim + CV, all normalized to 0..1.
    // Per-voice variant. Only called for the two pluck controls, and only
    // when their CV cable actually has more than one channel -- see
    // pluckPosPoly / pluckHardPoly below. With a mono or unpatched cable the
    // module does exactly what it did before: one scalar read, one broadcast
    // float_4, no per-lane loop.
    //
    // Same channel rule as BOW SPEED and MUTE: the bottom channel telegraphs
    // downwards, so a cable with fewer channels than voices holds its last
    // channel for the rest.
    inline float readSliderCVPoly(int paramId, int trimId, int inputId, int v) {
        float base = params[paramId].getValue();
        Input& in  = inputs[inputId];
        if (in.isConnected()) {
            int   nch = in.getChannels();
            float cv  = in.getVoltage(std::min(v, nch - 1)) * 0.1f;
            base = clamp(base + params[trimId].getValue() * cv, 0.f, 1.f);
        }
        return base;
    }

    inline float readSliderCV(int paramId, int trimId, int inputId) {
        float base = params[paramId].getValue();
        if (inputs[inputId].isConnected()) {
            float cv  = inputs[inputId].getVoltage(0) * 0.1f;
            float att = params[trimId].getValue();
            base = clamp(base + att * cv, 0.f, 1.f);
        }
        return base;
    }

    void process(const ProcessArgs& args) override {

        if (sampleRate != args.sampleRate) {
            sampleRate = args.sampleRate;
            initVoice();
        }

        // -- Control rate --------------------------------------------------
        ctrlDivCounter++;
        if (ctrlDivCounter >= CTRL_SKIP) {
            ctrlDivCounter = 0;

            int maxCh = 1;
            if (inputs[VOCT_INPUT].isConnected())
                maxCh = std::max(maxCh, inputs[VOCT_INPUT].getChannels());
            if (inputs[PLUCK_TRIG_INPUT].isConnected())
                maxCh = std::max(maxCh, inputs[PLUCK_TRIG_INPUT].getChannels());
            // A poly bow cable alone is enough to make voices: bowing several
            // notes with no pluck trigger patched is a normal way to play.
            if (inputs[BOW_SPEED_CV_INPUT].isConnected())
                maxCh = std::max(maxCh, inputs[BOW_SPEED_CV_INPUT].getChannels());
            nVoices = clamp(maxCh, 1, TWANG_MAX_POLY);
            // Clear strings that just dropped out of range. Without this a
            // dropped voice keeps its ring in the rails and it comes straight
            // back when the channel count grows again.
            if (nVoices < prevVoices) {
                for (int v = nVoices; v < prevVoices; ++v)
                    voiceQuad[v / 4].panicLane(v % 4);
            }
            prevVoices = nVoices;
            // Only the quads holding active voices get processed. A partial
            // final quad has its spare lanes masked (see the audio block).
            nQuads  = (nVoices + 3) / 4;

            // TONE: logarithmic 300 Hz -> 18 kHz.
            float toneSlider = readSliderCV(TONE_PARAM, TONE_TRIM_PARAM, TONE_CV_INPUT);
            float toneCutoff = 300.f * powf(60.f, toneSlider * 0.7f + 0.3f);
            float toneCoeff  = expf(-2.f * float(M_PI) * toneCutoff / sampleRate);
            cachedToneCompensationGain = 1.f + (1.f - toneSlider) * TWANG_TONE_COMPENSATION;

            // BOW LEVEL / DECAY: panel sliders. Decay is exponential so the
            // bottom of the travel is usable -- linear would put everything
            // short in the first tenth of the slider.
            bowLevel = 0.1f + 1.4f * readSliderCV(BOW_LEVEL_PARAM, BOW_LEVEL_TRIM_PARAM,
                                                  BOW_LEVEL_CV_INPUT);
            // 0.5s .. 60s. The top was 20s; extended so the slider reaches a
            // ring-on-forever sustain and the whole upper half stays expressive.
            stringDecaySeconds = 0.5f * powf(120.f, readSliderCV(DECAY_PARAM, DECAY_TRIM_PARAM,
                                                                DECAY_CV_INPUT));

            // MUTE: palm mute. Continuous depth, not a switch -- palm pressure
            // isn't on/off on a real instrument. Button and CV both engage it;
            // the button jumps straight to full depth.
            //
            // Per string. A monophonic cable palm-mutes the whole instrument,
            // a polyphonic one mutes each string on its own -- and, as with
            // BOW SPEED CV, the bottom channel telegraphs downwards so a cable
            // with fewer channels than voices holds its last channel for the
            // rest instead of leaving those strings unmuted.
            float muteBase   = params[MUTE_PARAM].getValue();
            float muteAtt    = params[MUTE_TRIM_PARAM].getValue();
            bool  muteCVConn = inputs[MUTE_CV_INPUT].isConnected();
            int   muteCVCh   = muteCVConn ? inputs[MUTE_CV_INPUT].getChannels() : 0;
            bool  muteButton = params[MUTE_BUTTON_PARAM].getValue() > 0.5f;
            // Measured BEFORE the latch is applied, so the green lamp reports
            // the knob and CV only. The latch gets its own red lamp below.
            float loudestMute = 0.f;
            for (int v = 0; v < nVoices; ++v) {
                float d = muteBase;
                if (muteCVConn)
                    d = clamp(muteBase + muteAtt * 0.1f
                              * inputs[MUTE_CV_INPUT].getVoltage(std::min(v, muteCVCh - 1)),
                              0.f, 1.f);
                loudestMute = fmaxf(loudestMute, d);
                if (muteButton) d = 1.f;
                muteDepthV[v] = d;
            }
            // Two lamps in the same bezel, because one lamp could not say
            // which of the two ways of muting was doing it. They are exclusive,
            // not additive: the latch forces every string to full mute, so
            // while it is down the knob and CV have no say at all and a green
            // reading of them would be reporting a state the module is not in.
            //   latch down -> red, and only red
            //   latch up   -> green, following the most muted string (with a
            //                 mono cable that is every string, the common case)
            lights[MUTE_LIGHT].setBrightness(muteButton ? 0.f : loudestMute);
            lights[MUTE_BUTTON_LIGHT].setBrightness(muteButton ? 1.f : 0.f);

            // A real palm mute kills sustain as well as brightness, so it acts
            // on both the bridge filter cutoff and the loop gain.
            //
            // The loss is per SAMPLE of string length, not per round trip; see
            // TwangStringSIMD::lossPerSample. A fixed per-round-trip gain made
            // decay run as 1/f (16.8 s at E2 against 2.1 s at E5) and made the
            // playing threshold pitch-dependent. With this, the minimum bow
            // speed to sustain measures 0.221 / 0.221 / 0.220 / 0.221 / 0.223
            // from E2 to A4 -- flat.
            float mutedCoeff = expf(-2.f * float(M_PI) * 400.f / sampleRate);
            for (int q = 0; q < TWANG_QUADS; ++q) {
                float tc[4], lp[4], ex[4];
                for (int lane = 0; lane < 4; ++lane) {
                    int v = q * 4 + lane;
                    float d = (v < nVoices) ? muteDepthV[v] : muteDepthV[0];
                    tc[lane] = toneCoeff + d * (mutedCoeff - toneCoeff);
                    float sec = stringDecaySeconds * (1.f - d * 0.85f);
                    lp[lane] = 13.816f / fmaxf(sec * sampleRate, 1.f);
                    // The bridge filter is a one-pole y = a*y + (1-a)*x, whose
                    // phase delay at DC is a/(1-a) samples. That is nothing at
                    // a bright TONE and a lot at a dark one: 0.54 samples at
                    // 8 kHz, 1.72 at 3.5 kHz, 7.05 at 1 kHz, and 18.6 with the
                    // palm mute fully in. Add the allpasses and this is the
                    // whole error.
                    float a = clamp(tc[lane], 0.f, 0.9995f);
                    ex[lane] = a / (1.f - a) + TWANG_DISPERSION_DELAY;
                }
                cachedToneCoeff[q]     = float_4::load(tc);
                cachedLossPerSample[q] = float_4::load(lp);
                cachedLoopExtra[q]     = float_4::load(ex);
            }

            // STRING IMPEDANCE: how hard the string resists the bow. Low
            // (light/gut-like) couples bow energy easily -- loud, easy to
            // excite, a bit of edge; high (heavy/steel-like) fights the bow,
            // wants pressure and speed, but rings back harder. 
            // PLUCK LEVEL: pluck exciter drive, 1..30 (was the Pluck Drive menu float).
            pluckDrive = 1.f + 29.f * readSliderCV(ATTACK_PARAM, ATTACK_TRIM_PARAM,
                                                   ATTACK_CV_INPUT);

            // SLIP: how fast the bow friction collapses once the string
            // breaks free of the sticking region. Low = the bow keeps
            // dragging after it slips (lazy, buzzy, sustaining scrape);
            // high = the slip dies fast (crisp, articulated, more stick slip bite). 
            float slipSlider = readSliderCV(SLIP_PARAM, SLIP_TRIM_PARAM,
                                            SLIP_CV_INPUT);
            float slipFalloff = 4.f + slipSlider * 56.f;

            // PRESSURE: collapses the sticking plateau and the capture width
            // into one gesture. Light = airy and easy to lose; heavy = grippy,
            // edgy, and will crunch at the top of the range. The default
            // (0.40) is the violin anchor: sticking 3.0, capture 0.35.
            float pressureSlider   = readSliderCV(BOW_PRESSURE_PARAM, BOW_PRESSURE_TRIM_PARAM,
                                                  BOW_PRESSURE_CV_INPUT);
            // Remapped. The old range started below Schelleng's minimum bow force
            float stickingFriction = 3.0f  + pressureSlider * 3.5f;
            float captureVelocity  = 0.22f + pressureSlider * 0.45f;

            float bowPositionSlider = readSliderCV(BOW_POSITION_PARAM, BOW_POSITION_TRIM_PARAM,
                                                   BOW_POSITION_CV_INPUT);
            cachedBowPosition = TWANG_BOW_POSITION_MIN
                              + (TWANG_BOW_POSITION_MAX - TWANG_BOW_POSITION_MIN)
                                * bowPositionSlider;

            float pluckPositionSlider = readSliderCV(PLUCK_POSITION_PARAM, PLUCK_POSITION_TRIM_PARAM,
                                                     PLUCK_POSITION_CV_INPUT);
            // Kept off the exact ends: a pluck at the nut or bridge has no
            // displacement to give.
            const float pluckPosLo   = 0.05f;
            const float pluckPosSpan = 0.85f;
            cachedPluckPosition = pluckPosLo + pluckPosSpan * pluckPositionSlider;
            pluckPosPoly = inputs[PLUCK_POSITION_CV_INPUT].getChannels() > 1;
            if (pluckPosPoly)
                for (int v = 0; v < nVoices; ++v)
                    pluckPositionV[v] = pluckPosLo + pluckPosSpan
                        * readSliderCVPoly(PLUCK_POSITION_PARAM, PLUCK_POSITION_TRIM_PARAM,
                                           PLUCK_POSITION_CV_INPUT, v);

            cachedPluckHardness = readSliderCV(PLUCK_HARDNESS_PARAM, PLUCK_HARDNESS_TRIM_PARAM,
                                               PLUCK_HARDNESS_CV_INPUT);
            pluckHardPoly = inputs[PLUCK_HARDNESS_CV_INPUT].getChannels() > 1;
            if (pluckHardPoly)
                for (int v = 0; v < nVoices; ++v)
                    pluckHardnessV[v] = readSliderCVPoly(PLUCK_HARDNESS_PARAM,
                                                         PLUCK_HARDNESS_TRIM_PARAM,
                                                         PLUCK_HARDNESS_CV_INPUT, v);

            // Every lane gets the same physics -- no per-lane multipliers.
            for (int q = 0; q < TWANG_QUADS; ++q) {
                voiceQuad[q].bowJunction.stickingFriction = float_4(stickingFriction);
                voiceQuad[q].bowJunction.captureVelocity  = float_4(captureVelocity);
                voiceQuad[q].bowJunction.slipFalloff      = float_4(slipFalloff);
                voiceQuad[q].bowJunction.bowLevel = float_4(bowLevel);
                voiceQuad[q].pluckDrive = float_4(pluckDrive);
                for (int lane = 0; lane < 4; ++lane)
                    voiceQuad[q].pluck[lane].tilt = pluckTilt;
                voiceQuad[q].bowJunction.stringImpedance  = float_4(stringImpedance);
                voiceQuad[q].bowPosition                  = float_4(cachedBowPosition);
                if (pluckPosPoly) {
                    float pp[4];
                    for (int lane = 0; lane < 4; ++lane) {
                        int v = q * 4 + lane;
                        pp[lane] = (v < nVoices) ? pluckPositionV[v] : cachedPluckPosition;
                    }
                    voiceQuad[q].pluckPosition            = float_4::load(pp);
                } else {
                    voiceQuad[q].pluckPosition            = float_4(cachedPluckPosition);
                }
                voiceQuad[q].lossPerSample                = cachedLossPerSample[q];
            }

            // DRIVE: 1.0 (clean) to ~8.0 (heavily saturated).
            float driveSlider = readSliderCV(DRIVE_PARAM, DRIVE_TRIM_PARAM, DRIVE_CV_INPUT);
            cachedDriveGain   = 1.f + driveSlider * 7.f;

            // RESONATOR SIZE: scales every body line and stage delay
            // together, which moves the whole box resonance -- short is a
            // small bright body, long is a large boomy one.
            float sizeSlider = readSliderCV(RESONATOR_SIZE_PARAM, RESONATOR_SIZE_TRIM_PARAM,
                                            RESONATOR_SIZE_CV_INPUT);
            // SIZE sets the LOWEST body mode; every other mode follows by its
            // measured ratio. Scaling the whole modal set in frequency is very
            // nearly what distinguishes a cello from a guitar from a violin,
            // so one knob covers the family:
            // The slider now runs the way its name reads: UP IS A BIGGER BODY.
            // It was inverted -- a bigger number meant a higher lowest mode,
            // which is a SMALLER box -- for as long as this control has existed.
            // Nobody had caught it because the old 55-330 Hz range was narrow
            // enough to hear as timbre rather than as size.
            //   0.0 -> 1200 Hz (smaller than the string: a metallic, formant-
            //                   like whistle sitting above the note)
            //   0.6 ->  131 Hz (guitar, the default)
            //   1.0 ->   30 Hz (a box so large its lowest mode is below pitch,
            //                   and reads as pure thud and room)
            //
            // The top is 1200 Hz, not the 1800 Hz it briefly was. 1800 put the
            // admittance's top mode at 10 kHz, well outside the range the
            // coupling's stability sweep covers, and the string could feed back
            // there; see TWANG_BRIDGE_BODY_MAX_HZ. 1200 keeps the whole 7-mode
            // bank under 14 kHz, so nothing hits the Nyquist clamp in
            // twangResonatorCoeffs at 44.1 kHz either.
            cachedLowestMode = 30.f * powf(40.f, 1.f - sizeSlider);

            // RESONATOR MATERIAL: wood -> metal -> glass. Wood is warm with
            // moderate decay, metal is bright and ringy, glass is thin and
            // long-ringing with very little absorption.
            float bodyMaterial = readSliderCV(RESONATOR_MATERIAL_PARAM, RESONATOR_MATERIAL_TRIM_PARAM,
                                              RESONATOR_MATERIAL_CV_INPUT);
            // MATERIAL now scales mode Q (wood is lossy and short, glass is
            // near-lossless and long) and tilts energy toward the upper modes.
            // Q was 22-88 on the lowest mode: all of that is "ringing", none
            // of it is "wooden thud", so MATERIAL only changed decay length.
            // 0.3-5.3 puts Q between about 7 and 115 -- broad enough at the
            // bottom to read as a formant, narrow enough at the top to sing.
            // MATERIAL owns body RING TIME (mode Q, ~7..115): wood is lossy and
            // thuds, glass is near-lossless and sings. 
            float bodyQScale = 0.3f + bodyMaterial * 5.0f;
            float bodyTiltAmt = bodyMaterial * bodyTilt;
            body.setBody(cachedLowestMode, bodyQScale, bodyTiltAmt, sampleRate);
            if (polyOutput)
                for (int q = 0; q < TWANG_QUADS; ++q)
                    polyBody[q].setBody(cachedLowestMode, bodyQScale, bodyTiltAmt, sampleRate);
            // The string's termination must track the body the user is
            // hearing, or the two stop belonging to the same instrument -- but
            // only up to the point where the coupled reflection is still known
            // to be stable. Past that the body keeps rising and the termination
            // holds, which costs a little realism at one end of one slider and
            // buys not feeding back.
            const float bridgeBodyHz = fminf(cachedLowestMode, TWANG_BRIDGE_BODY_MAX_HZ);
            for (int q = 0; q < TWANG_QUADS; ++q) {
                voiceQuad[q].bridgeAdmittance.setBody(bridgeBodyHz, bodyQScale, sampleRate);
                voiceQuad[q].bridgeAdmittance.setCoupling(bridgeCoupling);
            }
            // FDN per-line loop gain, in the musical ring-down range. With
            // per-line feedback the decay is once per line period, so wood
            // rings ~0.1-0.2 s and glass ~0.5-3 s (more at large body
            // sizes). Always < 1, so the network is stable at any size.
            // Per-line peak gain reaches ~1/(1-g) near the comb modes --
            // that IS a real body resonance; VOLUME is the safety valve.
            // Body -> string re-radiation is low-first: a wooden box pushes
            // its fundamental sympathetics more than any clang, so the
            // re-injection is lowpassed hard.

            cachedTwangDepth = readSliderCV(TWANG_PARAM, TWANG_TRIM_PARAM, TWANG_CV_INPUT);

            // "weird" ramps in over the 12 -> 30 semitone stretch of Max Bend.
            // Below 12 the rebound is slow and heavily damped (one soft
            // overshoot); above it the rate climbs, the damping falls away and
            // the pitch rings through several visible aftershocks.
            float weird = clamp((twangMaxBendSemitones - 12.f) / 12.f, 0.f, 1.f);
            cachedBounceDepth = twangBounceAmount * (0.15f + weird * 0.35f);
            for (int q = 0; q < TWANG_QUADS; ++q)
                voiceQuad[q].setTwangBounce(twangBounceHz * (1.f + weird * 4.f),
                                            1.2f + weird * 9.f, sampleRate);

            float bowKnob = params[BOW_SPEED_PARAM].getValue();
            float bowTrim = params[BOW_SPEED_TRIM_PARAM].getValue();
            bool  bowCVConnected = inputs[BOW_SPEED_CV_INPUT].isConnected();
            int   bowCVChannels  = bowCVConnected
                                 ? inputs[BOW_SPEED_CV_INPUT].getChannels() : 0;
            for (int v = 0; v < nVoices; ++v) {
                // The bottom channel telegraphs downwards: a monophonic cable
                // bows every voice, and a cable with fewer channels than voices
                // holds its last channel for the rest rather than leaving those
                // strings silent.
                float bowCV = 0.f;
                if (bowCVConnected)
                    bowCV = inputs[BOW_SPEED_CV_INPUT].getVoltage(
                                std::min(v, bowCVChannels - 1));
                // Scale 0.2 maps +-5V CV to a +-1.0 contribution, matching the
                // knob's own range so the two sum naturally.
                t_bowSpeedV[v] = clamp(bowKnob + bowTrim * bowCV * 0.2f, -2.f, 2.f);
            }
            t_bowSpeed = t_bowSpeedV[0];   // display / panel mirror

            cachedVolume = params[VOLUME_PARAM].getValue();
            if (inputs[VOLUME_CV_INPUT].isConnected())
                cachedVolume += inputs[VOLUME_CV_INPUT].getVoltage() * 0.1f;
            cachedVolume = clamp(cachedVolume, 0.f, 1.f);

            cachedBaseVOct = params[FREQ_PARAM].getValue();
        }

        // -- Audio rate ----------------------------------------------------
        // Bow speed per voice: slow attack, faster release -- a bow arm takes
        // a moment to get moving and coasts to a stop.
        float loudestBow = 0.f;
        for (int v = 0; v < nVoices; ++v) {
            float t = t_bowSpeedV[v];
            if (fabsf(t) > fabsf(a_bowSpeedV[v]))
                a_bowSpeedV[v] += 0.002f * (t - a_bowSpeedV[v]);
            else
                a_bowSpeedV[v] += 0.02f  * (t - a_bowSpeedV[v]);
            loudestBow = fmaxf(loudestBow, fabsf(a_bowSpeedV[v]));
        }
        // Panel display and the sleep gate follow the busiest string.
        a_bowSpeed = (nVoices > 0) ? a_bowSpeedV[0] : 0.f;

        // Pluck button: strums every active voice on the rising edge. It
        // shares the exact excitation path of the CV trigger -- impulse at
        // the pluck point with the bow lifted -- so the two can't stack.
        // Velocity comes from the MAX PLUCK VELOCITY trimmer, which is the
        // only velocity source the button has -- it was a hardcoded 0.8.
        const float pluckVelMax = clamp(params[PLUCK_VELOCITY_PARAM].getValue(), 0.f, 10.f) * 0.1f;
        bool pluckButtonDown = params[PLUCK_BUTTON_PARAM].getValue() > 0.5f;
        lights[PLUCK_LIGHT].setBrightness(pluckButtonDown ? 1.f : 0.f);
        if (pluckButtonDown && !pluckButtonWasDown) {
            for (int v = 0; v < nVoices; ++v) {
                int q    = v / 4;
                int lane = v % 4;
                voiceQuad[q].pluck[lane].trigger(
                    pluckHardPoly ? pluckHardnessV[v] : cachedPluckHardness,
                    pluckVelMax, sampleRate);
                voiceQuad[q].triggerPluck(lane, pluckVelMax);
            }
            asleep   = false;
            wakeHold = (int)(TWANG_PLUCK_WAKE_SEC * sampleRate);
        }
        pluckButtonWasDown = pluckButtonDown;

        // Pluck triggers, per voice. Trigger voltage doubles as velocity, and
        // MAX PLUCK VELOCITY scales what full scale means. At the default 10 V
        // this is a multiply by 1 and the mapping is unchanged; below that it
        // compresses the whole velocity range toward the bottom, which is what
        // makes a gate sequencer usable -- every gate is 10 V, so without it
        // every hit is a maximum-force pluck.
        bool anyPluckActive = false;
        if (inputs[PLUCK_TRIG_INPUT].isConnected()) {
            for (int v = 0; v < nVoices; ++v) {
                int q    = v / 4;
                int lane = v % 4;
                float trigVolts = inputs[PLUCK_TRIG_INPUT].getPolyVoltage(v);
                if (pluckTrigger[v].process(trigVolts, 0.1f, 1.f)) {
                    float velocity = clamp(trigVolts * 0.1f, 0.05f, 1.f) * pluckVelMax;
                    // A harder pluck engages more string stiffness and so comes
                    // out brighter, not just louder -- velocity nudges the
                    // effective hardness upward rather than needing its own knob.
                    float baseHardness = pluckHardPoly ? pluckHardnessV[v] : cachedPluckHardness;
                    float effectiveHardness = clamp(
                        baseHardness + velocity * TWANG_PLUCK_BRIGHTEN, 0.f, 1.f);
                    voiceQuad[q].pluck[lane].trigger(effectiveHardness, velocity, sampleRate);
                    // Bow lifts, junction moves to the pluck point, twang
                    // kicks (see TwangStringSIMD).
                    voiceQuad[q].triggerPluck(lane, velocity);
                    asleep   = false;
                    wakeHold = (int)(TWANG_PLUCK_WAKE_SEC * sampleRate);
                }
                if (voiceQuad[q].pluck[lane].active) anyPluckActive = true;
            }
        } else {
            // No trig jack connected: the button is the only excitation
            // source, so still track active plucks to keep the sleep gate
            // honest.
            for (int v = 0; v < nVoices; ++v)
                if (voiceQuad[v / 4].pluck[v % 4].active) { anyPluckActive = true; break; }
        }

        if (loudestBow >= TWANG_BOW_DEAD_ZONE * 0.5f) asleep = false;

        // A pluck holds the module awake for a fixed window regardless of what
        // the output is doing. Nothing below can strand a non-zero wakeHold,
        // because a non-zero one forces asleep false and so skips the early
        // return.
        if (wakeHold > 0) {
            --wakeHold;
            asleep = false;
        }

        // Sleep only when nothing is bowing, nothing is plucking, and the
        // strings have rung out.
        if (asleep && !anyPluckActive) {
            // Keep the declared width. Collapsing to 1 here and back to nVoices
            // on the next note made the port flicker between mono and poly at
            // every note-off, which downstream poly modules have to re-adapt to.
            int sleepCh = polyOutput ? nVoices : 1;
            outputs[OUT_L].setChannels(sleepCh);
            outputs[OUT_R].setChannels(sleepCh);
            for (int c = 0; c < sleepCh; ++c) {
                outputs[OUT_L].setVoltage(0.f, c);
                outputs[OUT_R].setVoltage(0.f, c);
            }
            return;
        }

        float fmSemitonesBase = params[FM_TRIM_PARAM].getValue() * fmDepthSemitones;
        bool  voctConnected   = inputs[VOCT_INPUT].isConnected();
        bool  fmConnected     = inputs[FM_CV_INPUT].isConnected();

        float_4 sumQuad = float_4(0.f);

        for (int q = 0; q < nQuads; ++q) {
            // Per-lane values are assembled in plain arrays and loaded once,
            // rather than written into a float_4 lane by lane.
            float segArr[4];
            float bowArr[4];
            float maskArr[4];

            for (int lane = 0; lane < 4; ++lane) {
                int v = q * 4 + lane;

                if (v >= nVoices) {
                    // Spare lane in a partial final quad. Bow speed 0 keeps it
                    // from being excited by the broadcast bow gesture, and the
                    // mask keeps it out of the sum.
                    segArr[lane]  = a_segmentSamples[v];
                    bowArr[lane]  = 0.f;
                    maskArr[lane] = 0.f;
                    continue;
                }

                float vOct = cachedBaseVOct;
                if (voctConnected) vOct += inputs[VOCT_INPUT].getPolyVoltage(v);
                if (fmConnected) {
                    // Scale 0.2 maps a typical +-5V CV to a +-1.0 contribution
                    // before depth/trim, matching the BOW SPEED convention.
                    vOct += inputs[FM_CV_INPUT].getPolyVoltage(v) * 0.2f * fmSemitonesBase / 12.f;
                }

                // Fold back down an octave at a time if played too high -- the
                // bow junction can go unstable up there. Also floor the low end
                // so the segment length stays inside the allocated rails.
                //
                // Bound the input before folding. Nothing stops an upstream
                // module from putting Inf or a huge value on V/Oct or FM, and
                // `while (x > 2) x -= 1` never terminates on Inf and runs ~1e30
                // times on 1e30 -- an infinite loop on the audio thread, which
                // freezes the whole host. +-20V is far outside any real V/Oct,
                // so every musical input folds exactly as before; only the
                // pathological ones are caught.
                if (!std::isfinite(vOct)) vOct = 0.f;
                vOct = clamp(vOct, -20.f, 20.f);
                while (vOct > 2.0f) vOct -= 1.f;
                vOct = fmaxf(vOct, -3.5f);

                // TWANG: real string tension rises with displacement, so a
                // pluck goes briefly sharp and the string settles back to its
                // resting pitch as it rings. 
                if (cachedTwangDepth > 0.f) {
                    float levelTransient = (voiceQuad[q].energySmooth[lane]
                                          - voiceQuad[q].levelRef[lane]) * 0.25f;
                    float twangAmt = clamp(levelTransient + voiceQuad[q].pluckKick[lane],
                                           0.f, 1.f);
                    // The bounce is SIGNED -- the dips below the resting pitch
                    // are the point. The clamp is asymmetric because the
                    // physics is: tension can rise a long way above resting,
                    // but it cannot fall far below it without the string going
                    // slack, so a deep downward rebound is not a thing a real
                    // string does.
                    float bend = clamp(twangAmt + cachedBounceDepth * voiceQuad[q].twangRing[lane],
                                       -0.35f, 1.5f);
                    vOct += cachedTwangDepth * twangMaxBendSemitones * bend / 12.f;

                    // The bend is added AFTER the octave fold above, so at very
                    // large Max Bend it could push vOct several octaves past
                    // anything the waveguide can represent: 
                    vOct = clamp(vOct, -3.5f, 2.0f);
                }

                // 16 rather than 4: at the bow/pluck position extremes one of
                // the two spans is under a tenth of the total, and a span below
                // about 2 samples is shorter than the Lagrange interpolator and
                // the dispersion allpasses need.
                // The waveguide's period is the whole round trip, which is
                // 2 * segmentSamples PLUS whatever the bridge filter and the
                // dispersion allpasses contribute. Ignoring that made every
                // note flat, and progressively worse upward because the error
                // is a fixed number of samples against a shrinking period:
                // measured -6 cents at C2 but -83 cents at C6, and it moved
                // with the TONE slider (-33 cents at C4 bright, -80 dark).
                // Halved because the correction applies to the one-way span.
                float target = clamp(vOctToSegmentSamples(vOct)
                                     - cachedLoopExtra[q][lane] * 0.5f,
                                     16.f, (float)voiceQuad[q].nutToP.bufSize - 4.f);
                a_segmentSamples[v] += 0.05f * (target - a_segmentSamples[v]);

                segArr[lane]  = a_segmentSamples[v];
                bowArr[lane]  = a_bowSpeedV[v];
                maskArr[lane] = 1.f;
            }

            // The body output from the PREVIOUS sample is re-injected at each
            // lane's bridge: one sample of travel time, no instantaneous
            // feedback loop.
            float_4 out = voiceQuad[q].process(float_4::load(bowArr), float_4::load(segArr),
                                               cachedToneCoeff[q],
                                               float_4(TWANG_STRING_STIFFNESS));
            float_4 masked = out * float_4::load(maskArr);
            sumQuad += masked;
            if (polyOutput) polyVoice[q] = masked;
        }

        float sumArr[4];
        sumQuad.store(sumArr);
        float voiceSum = sumArr[0] + sumArr[1] + sumArr[2] + sumArr[3];

        // Keep the summed level roughly constant as voices are added, so a
        // 6-note chord isn't six times louder than a single note.
        //
        // Counted from the strings that are actually SOUNDING, not from nVoices
        // (the cable's channel count). nVoices comes from whatever width the
        // upstream MIDI-CV module is set to, so normalising by it meant a single
        // note through a 16-channel cable came out 1/sqrt(16) = 12 dB quieter
        // than the identical note through a mono cable -- the level depended on
        // a setting in a different module rather than on what was being played.
        int soundingVoices = 0;
        for (int v = 0; v < nVoices; ++v) {
            const int q = v / 4, lane = v % 4;
            if (fabsf(a_bowSpeedV[v]) >= TWANG_BOW_DEAD_ZONE * 0.5f
                || voiceQuad[q].pluck[lane].active
                || voiceQuad[q].energySmooth[lane] > 1e-4f)
                ++soundingVoices;
        }
        if (soundingVoices < 1) soundingVoices = 1;
        // Smoothed (~45ms) so the gain slides as notes come and go instead of
        // stepping on every note-on.
        a_polyNormalize += 0.0005f * (1.f / sqrtf((float)soundingVoices) - a_polyNormalize);
        float mixed = voiceSum * a_polyNormalize * cachedToneCompensationGain
                    * TWANG_INPUT_GAIN;

        // -- Mono core -----------------------------------------------------
        // The instrument is one object, so its body is mono. The stereo
        // image is built below from a mid-side width on this same core, so
        // L/R stay locked together (mono sum = 2x core) instead of being two
        // independent resonators.
        float core = dcBlocker.process(mixed);

        // DRIVE lives here, outside any string's feedback loop -- applied
        // once after the voice sum, before the body resonator.
        core = outputDrive.process(core, cachedDriveGain);

        // -- Body resonator ------------------------------------------------
        // FDN: four feedback delay lines with PRIMES-incommensurate delays.
        // Per-line feedback: each line's own tail is material-lowpassed and
        // fed back into that line (a shared-sum input would make all lines
        // identical copies of one signal and the low modes would decay once
        // per SAMPLE instead of once per line period, killing the ring in
        // ~10 ms). The box output is the direct string feed plus 1/4 of the
        // summed ring; the allpass chain after adds the short "air" blur.
        float bodyOut = body.process(core) * TWANG_OUTPUT_MAKEUP;

        // Body -> string: lowpass the box's output to the low end and re-
        // inject it at the next sample's bridge pass. The box now sings

        // -- Stereo width ---------------------------------------------------
        // Two short feedback delay lines, same input, slightly different
        // lengths. Their difference is decorrelated detail that lives only
        // in the SIDE channel; L/R read as one thing in a room, and a mono
        // sum cancels the side entirely.
        float spaceOutL = spaceL.process(bodyOut);
        float spaceOutR = spaceR.process(bodyOut);
        float side = 0.5f * (spaceOutL - spaceOutR) * stereoWidth;
        float voiceOutL = bodyOut + side;
        float voiceOutR = bodyOut - side;
        if (!std::isfinite(voiceOutL) || !std::isfinite(voiceOutR)) {
            voiceOutL = 0.f;
            voiceOutR = 0.f;
            panic();
        }

        // The output level alone is NOT a valid "has it rung out?" test for a
        // plucked string. The pluck is injected at the pluck point and the
        // signal is tapped at the bridge, so there is a propagation delay of up
        // to one rail length -- ~218 samples at A2 -- before a pluck reaches the
        // output at all. The excitation lobe is far shorter than that (93
        // samples at the default hardness, 12 at a hard pick), so the instant it
        // ends, anyPluckActive goes false while the output is still zero simply
        // because the wave has not arrived yet.
        //
        // Without wakeHold that put the module to sleep one sample later, the
        // early return above froze the rails mid-flight, and the wave never
        // arrived. Each further pluck added a little more to the frozen rails
        // until the total happened to clear the threshold inside a pluck window
        // -- which is why the first few plucks after a patch load or a Reset
        // were silent and everything was fine from then on.
        if (loudestBow < TWANG_BOW_DEAD_ZONE * 0.5f && !anyPluckActive
            && wakeHold == 0
            && fabsf(voiceOutL) < 1e-5f && fabsf(voiceOutR) < 1e-5f)
            asleep = true;

        if (polyOutput) {
            // The full stereo chain, per voice, including the body -- the only
            // thing missing is the stereo widener, which has nothing to do on a
            // per-voice output. No 0.7 fudge factor any more: this really is
            // the same signal path, so switching modes does not change the tone.
            outputs[OUT_L].setChannels(nVoices);
            outputs[OUT_R].setChannels(nVoices);
            float scale = cachedToneCompensationGain * TWANG_INPUT_GAIN;
            for (int q = 0; q < nQuads; ++q) {
                float_4 x = polyDrive[q].process(polyDC[q].process(polyVoice[q] * float_4(scale)),
                                                 cachedDriveGain);
                x = rack::simd::clamp(polyBody[q].process(x) * float_4(TWANG_OUTPUT_MAKEUP * cachedVolume),
                                      float_4(-10.f), float_4(10.f));
                float tmp[4];
                x.store(tmp);
                for (int lane = 0; lane < 4; ++lane) {
                    int v = q * 4 + lane;
                    if (v >= nVoices) break;
                    float y = std::isfinite(tmp[lane]) ? tmp[lane] : 0.f;
                    outputs[OUT_L].setVoltage(y, v);
                    outputs[OUT_R].setVoltage(y, v);
                }
            }
        } else {
            outputs[OUT_L].setChannels(1);
            outputs[OUT_R].setChannels(1);
            outputs[OUT_L].setVoltage(clamp(voiceOutL * cachedVolume, -10.f, 10.f));
            outputs[OUT_R].setVoltage(clamp(voiceOutR * cachedVolume, -10.f, 10.f));
        }
    }
};

struct TwangSliderBase : app::SvgSlider {
    TwangSliderBase() {
        setBackgroundSvg(Svg::load(asset::plugin(pluginInstance,
            "res/components/ShortSlider.svg")));
        setHandleSvg(Svg::load(asset::plugin(pluginInstance,
            "res/components/ShortSliderHandle.svg")));
        setHandlePosCentered(math::Vec(10.f, 55.f), math::Vec(10.f, 10.f));
    }
};
template <typename TL = BlueLight>
struct AulosSlider : LightSlider<TwangSliderBase, VCVSliderLight<TL>> {
    AulosSlider() {}
};

// ==========================================================================
// Widget
// ==========================================================================
struct TwangWidget : ModuleWidget {
 
    struct WaveDisplay : TransparentWidget {
        Twang* module = nullptr;
 
        void drawLayer(const DrawArgs& args, int layer) override {
            if (layer != 1) { TransparentWidget::drawLayer(args, layer); return; }
 
            const float w   = box.size.x;
            const float h   = box.size.y;
            const float pad = 3.f;
            const int   pts = 64;
 
            nvgBeginPath(args.vg);
            nvgRoundedRect(args.vg, 0, 0, w, h, 3.f);
            nvgFillColor(args.vg, nvgRGB(12, 12, 14));
            nvgFill(args.vg);
 
            if (!module) {
                nvgBeginPath(args.vg);
                nvgMoveTo(args.vg, pad, h * 0.5f);
                nvgLineTo(args.vg, w - pad, h * 0.5f);
                nvgStrokeColor(args.vg, nvgRGBAf(0.55f, 0.65f, 0.75f, 0.5f));
                nvgStrokeWidth(args.vg, 1.2f);
                nvgStroke(args.vg);
                TransparentWidget::drawLayer(args, layer);
                return;
            }
 
            // Voice 0 (quad 0, lane 0), stitched at the junction's split point.
            float displayBuf[pts];
            for (int i = 0; i < pts; ++i) {
                float positionFraction = (float)i / (float)(pts - 1);
                displayBuf[i] = module->voiceQuad[0].sampleStringAt(0, positionFraction);
            }
 
            float peak = 0.f;
            for (int i = 0; i < pts; ++i)
                peak = fmaxf(peak, fabsf(displayBuf[i]));
            float scale = (peak > 0.01f) ? (0.5f / peak) : 0.f;
 
            float driveSlider = module->params[Twang::DRIVE_PARAM].getValue();
            float bowAmt  = clamp(fabsf(module->a_bowSpeed) * 2.f, 0.f, 1.f);
            float heatAmt = clamp(driveSlider * 0.6f + bowAmt * 0.4f, 0.f, 1.f);
            float lineR = 0.55f + heatAmt * (1.00f - 0.55f);
            float lineG = 0.65f + heatAmt * (0.75f - 0.65f);
            float lineB = 0.75f + heatAmt * (0.20f - 0.75f);
 
            nvgBeginPath(args.vg);
            for (int i = 0; i < pts; ++i) {
                float xPos = pad + (float)i / (float)(pts - 1) * (w - 2.f * pad);
                float yPos = h * 0.5f - displayBuf[i] * scale * (h * 0.5f - pad);
                if (i == 0) nvgMoveTo(args.vg, xPos, yPos);
                else        nvgLineTo(args.vg, xPos, yPos);
            }
            nvgStrokeColor(args.vg, nvgRGBAf(lineR, lineG, lineB, 0.85f));
            nvgStrokeWidth(args.vg, 1.5f);
            nvgStroke(args.vg);
 
            // Contact markers.
            //
            // The old pair was a warm sand line and a pale green line, both at
            // 35-40% alpha: nearly the same colour as each other, and the sand
            // one sat right on top of the string trace once DRIVE pushed it
            // toward (1.00, 0.75, 0.20), so under the exact conditions where you
            // most want to see the bow, the bow marker vanished.
            //
            // Each marker takes its own row's colour -- bow blue, pluck white,
            // the same code as the sliders -- and both are solid lines. What
            // tells them apart is a small glyph at one end: a violin bow lying
            // across the top of the bow marker, a plectrum standing on the
            // bottom of the pluck marker. Each line STOPS where its glyph
            // starts, so the two never overlap.
            //
            // The display is 40 x 17 mm, so at ~121 x 51 px there is room for a
            // ~9 px icon that stays readable without a font dependency. The
            // glyphs are wider than the line, and with EXTENDED CONTACT RANGE a
            // marker can sit at 0.01, so the whole block is scissored to the
            // display rather than trusting the glyph to stay inside.
            const float markTop = pad;
            const float markBot = h - pad;

            nvgSave(args.vg);
            nvgScissor(args.vg, 0.f, 0.f, w, h);

            // Bow: glyph across the top, line hanging below it.
            //
            // A violin bow is a long stick with a SLIGHT camber and hair
            // stretched nearly straight across it -- 13 px long against 2.5 px
            // of arc. An arc any deeper than this stops reading as a violin bow
            // and starts reading as an archer's bow. The small block at the
            // left end is the frog, which is what fixes the direction.
            {
                float markX = pad + module->cachedBowPosition * (w - 2.f * pad);
                NVGcolor c  = nvgRGBAf(0.16f, 0.70f, 0.94f, 0.90f);   // SCHEME_BLUE

                const float halfW = 6.5f;
                const float hairY = markTop + 5.f;

                nvgBeginPath(args.vg);
                // hair: straight, the edge that meets the string
                nvgMoveTo(args.vg, markX - halfW, hairY);
                nvgLineTo(args.vg, markX + halfW, hairY);
                // stick: quadratic control 5 px up puts the apex 2.5 px up
                nvgMoveTo(args.vg, markX - halfW, hairY);
                nvgQuadTo(args.vg, markX, hairY - 5.f, markX + halfW, hairY);
                nvgStrokeColor(args.vg, c);
                nvgStrokeWidth(args.vg, 1.f);
                nvgStroke(args.vg);

                // frog
                nvgBeginPath(args.vg);
                nvgRect(args.vg, markX - halfW - 1.6f, hairY - 3.2f, 2.2f, 3.6f);
                nvgFillColor(args.vg, c);
                nvgFill(args.vg);

                nvgBeginPath(args.vg);
                nvgMoveTo(args.vg, markX, hairY + 1.5f);
                nvgLineTo(args.vg, markX, markBot);
                nvgStrokeColor(args.vg, c);
                nvgStrokeWidth(args.vg, 1.4f);
                nvgStroke(args.vg);
            }

            // Pluck: line from the top, plectrum standing on the bottom with
            // its tip pointing up at the string. A filled rounded triangle is
            // the one silhouette nobody mistakes for anything else.
            {
                float markX = pad + module->cachedPluckPosition * (w - 2.f * pad);
                NVGcolor c  = nvgRGBAf(0.94f, 0.94f, 0.94f, 0.90f);   // SCHEME_WHITE

                const float tipY = markBot - 8.5f;

                nvgBeginPath(args.vg);
                nvgMoveTo(args.vg, markX, markTop);
                nvgLineTo(args.vg, markX, tipY - 1.5f);
                nvgStrokeColor(args.vg, c);
                nvgStrokeWidth(args.vg, 1.4f);
                nvgStroke(args.vg);

                nvgBeginPath(args.vg);
                nvgMoveTo(args.vg, markX,        tipY);                  // tip
                nvgQuadTo(args.vg, markX + 3.9f, markBot - 4.0f,
                                   markX + 3.2f, markBot - 1.8f);        // right edge
                nvgQuadTo(args.vg, markX,        markBot,
                                   markX - 3.2f, markBot - 1.8f);        // rounded base
                nvgQuadTo(args.vg, markX - 3.9f, markBot - 4.0f,
                                   markX,        tipY);                  // left edge
                nvgClosePath(args.vg);
                nvgFillColor(args.vg, c);
                nvgFill(args.vg);
            }

            nvgRestore(args.vg);

            TransparentWidget::drawLayer(args, layer);
        }
    };
 
    TwangWidget(Twang* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Twang.svg"),
            asset::plugin(pluginInstance, "res/Twang-dark.svg")
        ));
 
        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
 
        // 20HP = 101.6mm. Two left rails of controls, then six slider
        // columns spanning the rest of the panel.
        const float railA = 9.5f;    // knob + trim + CV trios
        const float railB = 22.0f;   // lighter, mixed-height controls
 
        // -- Rail A: three knob/trim/jack trios ----------------------------
        // BOW SPEED
        addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(railA, 18.f)), module, Twang::BOW_SPEED_PARAM));
        addParam(createParamCentered<Trimpot>            (mm2px(Vec(railA, 28.f)), module, Twang::BOW_SPEED_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>   (mm2px(Vec(railA, 37.f)), module, Twang::BOW_SPEED_CV_INPUT));
 
        // MUTE
        addParam(createLightParamCentered<VCVLightBezelLatch<GreenLight>>(
            mm2px(Vec(railA, 52.f)), module, Twang::MUTE_BUTTON_PARAM, Twang::MUTE_LIGHT));
        // Red latch lamp, overlaid on the same bezel. VCVBezelLight is exactly
        // the light the bezel builds for itself (same 17.545 px box, transparent
        // border and background), so it lands in register and adds no backing
        // of its own. It is a LightWidget, hence a TransparentWidget, whose
        // onButton does nothing and consumes nothing -- the click falls through
        // to the latch underneath.
        addChild(createLightCentered<VCVBezelLight<RedLight>>(
            mm2px(Vec(railA, 52.f)), module, Twang::MUTE_BUTTON_LIGHT));
        addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(railA, 62.f)), module, Twang::MUTE_PARAM));
        addParam(createParamCentered<Trimpot>            (mm2px(Vec(railA, 72.f)), module, Twang::MUTE_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>   (mm2px(Vec(railA, 81.f)), module, Twang::MUTE_CV_INPUT));
 
        // TWANG
        addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(railA, 96.f)), module, Twang::TWANG_PARAM));
        addParam(createParamCentered<Trimpot>            (mm2px(Vec(railA,106.f)), module, Twang::TWANG_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>   (mm2px(Vec(railA,115.f)), module, Twang::TWANG_CV_INPUT));
 
        // -- Rail B: lighter controls --------------------------------------
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(railB, 19.f)), module, Twang::FREQ_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>   (mm2px(Vec(railB, 32.f)), module, Twang::VOCT_INPUT));
 
        // VOLUME and FM move up, which frees 15mm above the pluck button for the
        // velocity trimmer. FM and the pluck group sit on rail A's own rows --
        // 72/81 against the MUTE trim and jack, 96/106/115 against the TWANG
        // trio -- so those read as one grid across both columns rather than two
        // independent stacks. The rails are 12.5mm apart, so a shared row is
        // alignment, not a collision.
        //
        // VOLUME is the one group deliberately off that grid, sitting 5mm above
        // rail A's MUTE rows: its jack needs a label underneath, and on rail A's
        // rows the FM trimmer came up too close to leave room. At 47/57 it gets
        // the same 15mm of clear space below it that FM has below its own jack.
        addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(railB, 47.f)), module, Twang::VOLUME_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>   (mm2px(Vec(railB, 57.f)), module, Twang::VOLUME_CV_INPUT));
 
        addParam(createParamCentered<Trimpot>         (mm2px(Vec(railB, 72.f)), module, Twang::FM_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(railB, 81.f)), module, Twang::FM_CV_INPUT));
 
        addParam(createParamCentered<Trimpot>              (mm2px(Vec(railB,  96.f)), module, Twang::PLUCK_VELOCITY_PARAM));
        addParam(createParamCentered<LEDButton>            (mm2px(Vec(railB, 106.f)), module, Twang::PLUCK_BUTTON_PARAM));
        addChild(createLightCentered<SmallLight<BlueLight>>(mm2px(Vec(railB, 106.f)), module, Twang::PLUCK_LIGHT));
        addInput(createInputCentered<ThemedPJ301MPort>     (mm2px(Vec(railB, 115.f)), module, Twang::PLUCK_TRIG_INPUT));
 
 
        // -- Display -------------------------------------------------------
        // Narrower than the panel, sitting over the slider region only, so
        // the left rails can run the full panel height.
        auto* wd = createWidget<WaveDisplay>(mm2px(Vec(33.f, 15.f)));
        wd->box.size = mm2px(Vec(40.f, 17.f));
        wd->module   = module;
        addChild(wd);
 
        // -- Slider columns ------------------------------------------------
        // Six columns at 12mm pitch, matching the GLASS/AULOS slider bank.
        const float col[6] = { 36.f, 48.f, 60.f, 72.f, 84.f, 96.f };
 
        // Slider handle colour, same scheme as GLASS and AULOS:
        //   1 blue   contact / excitation controls
        //   2 white  amount and string-property controls
        //   3 yellow resonator and character controls
        //   4 red    the terminal control of the row
        // The two rows are colour-aligned column by column, so LEVEL/DECAY
        // read as a pair and TONE/DRIVE close each row the way WOOD closes
        // AULOS and DAMP closes GLASS.
        struct SlSpec { int param, att, cv, color; };
 
        auto addSliderColumn = [&](const SlSpec& s, float x, float ys, float yt, float yj) {
            if (s.color == 1)
                addParam(createParamCentered<AulosSlider<BlueLight>>  (mm2px(Vec(x, ys)), module, s.param));
            else if (s.color == 2)
                addParam(createParamCentered<AulosSlider<WhiteLight>> (mm2px(Vec(x, ys)), module, s.param));
            else if (s.color == 3)
                addParam(createParamCentered<AulosSlider<YellowLight>>(mm2px(Vec(x, ys)), module, s.param));
            else
                addParam(createParamCentered<AulosSlider<RedLight>>   (mm2px(Vec(x, ys)), module, s.param));
            addParam(createParamCentered<Trimpot>         (mm2px(Vec(x, yt)), module, s.att));
            addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(x, yj)), module, s.cv));
        };
 
        // Bow row (top): level, position, pressure, slip, material, tone.
        // Column 3 was String Impedance, now a context-menu voicing constant;
        // RESONATOR SIZE moved up into it so PLUCK LEVEL could take bottom-left,
        // then traded places with SLIP. SLIP is a property of the bow's
        // stick-slip friction and belongs with the other three bow controls, so
        // the row is now four blue excitation controls, one yellow resonator
        // control, and the red terminal -- the exact shape of the pluck row
        // below it, category by category, column by column.
        const float bowSliderY = 52.f;
        const float bowTrimY   = bowSliderY + 11.f;
        const float bowJackY   = bowSliderY + 19.f;
 
        const SlSpec bowRow[6] = {
            { Twang::BOW_LEVEL_PARAM,        Twang::BOW_LEVEL_TRIM_PARAM,        Twang::BOW_LEVEL_CV_INPUT,        1 },  
            { Twang::BOW_POSITION_PARAM,     Twang::BOW_POSITION_TRIM_PARAM,     Twang::BOW_POSITION_CV_INPUT,     1 },  
            { Twang::BOW_PRESSURE_PARAM,     Twang::BOW_PRESSURE_TRIM_PARAM,     Twang::BOW_PRESSURE_CV_INPUT,     1 },  
            { Twang::SLIP_PARAM,             Twang::SLIP_TRIM_PARAM,             Twang::SLIP_CV_INPUT,             1 },  
            { Twang::RESONATOR_MATERIAL_PARAM, Twang::RESONATOR_MATERIAL_TRIM_PARAM, Twang::RESONATOR_MATERIAL_CV_INPUT, 3 },  
            { Twang::TONE_PARAM,             Twang::TONE_TRIM_PARAM,             Twang::TONE_CV_INPUT,             4 },  
        };
        for (int i = 0; i < 6; ++i)
            addSliderColumn(bowRow[i], col[i], bowSliderY, bowTrimY, bowJackY);
 
        // Pluck row (bottom): pluck level, position, decay, hardness, size, drive.
        // PLUCK LEVEL leads the row as the pluck's input stage. POSITION sits in
        // column 1 so it lands directly under BOW POSITION -- the two contact
        // points are the same physical quantity and now read as a column, which
        // is also what the two display markers show. DRIVE/TONE stay
        // column-aligned as the terminal control of each row.
        const float pluckSliderY = 96.f;
        const float pluckTrimY   = pluckSliderY + 11.f;
        const float pluckJackY   = pluckSliderY + 19.f;
 
        const SlSpec pluckRow[6] = {
            { Twang::ATTACK_PARAM,              Twang::ATTACK_TRIM_PARAM,              Twang::ATTACK_CV_INPUT,              2 },  
            { Twang::PLUCK_POSITION_PARAM,      Twang::PLUCK_POSITION_TRIM_PARAM,      Twang::PLUCK_POSITION_CV_INPUT,      2 },  
            { Twang::DECAY_PARAM,               Twang::DECAY_TRIM_PARAM,               Twang::DECAY_CV_INPUT,               2 },  
            { Twang::PLUCK_HARDNESS_PARAM,      Twang::PLUCK_HARDNESS_TRIM_PARAM,      Twang::PLUCK_HARDNESS_CV_INPUT,      2 },  
            { Twang::RESONATOR_SIZE_PARAM,      Twang::RESONATOR_SIZE_TRIM_PARAM,      Twang::RESONATOR_SIZE_CV_INPUT,      3 },  
            { Twang::DRIVE_PARAM,               Twang::DRIVE_TRIM_PARAM,               Twang::DRIVE_CV_INPUT,               4 },  
        };
        for (int i = 0; i < 6; ++i)
            addSliderColumn(pluckRow[i], col[i], pluckSliderY, pluckTrimY, pluckJackY);
 
 
        // -- Outputs -------------------------------------------------------
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(col[4], 23.5f)), module, Twang::OUT_L));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(col[5], 23.5f)), module, Twang::OUT_R));
    }
 
    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
        Twang* m = dynamic_cast<Twang*>(module);
        if (!m) return;
 
        menu->addChild(new MenuSeparator());
 
        struct FloatQuantity : Quantity {
            float* target; float lo, hi, def; std::string label;
            FloatQuantity(float* t, float lo, float hi, float def, std::string lbl)
                : target(t), lo(lo), hi(hi), def(def), label(lbl) {}
            void   setValue(float v) override { *target = clamp(v, lo, hi); }
            float  getValue() override        { return *target; }
            float  getDefaultValue() override { return def; }
            float  getMinValue() override     { return lo; }
            float  getMaxValue() override     { return hi; }
            int    getDisplayPrecision() override { return 3; }
            std::string getLabel() override   { return label; }
        };
        // ui::Slider does not delete `quantity` in its destructor; this does.
        struct OwnedSlider : ui::Slider {
            ~OwnedSlider() { delete quantity; quantity = nullptr; }
        };
        auto addFSlider = [](Menu* menu, float* v, float lo, float hi, float def, std::string lbl) {
            auto* sl = new OwnedSlider();
            sl->quantity   = new FloatQuantity(v, lo, hi, def, lbl);
            sl->box.size.x = 200.f;
            menu->addChild(sl);
        };
 
        struct BoolItem : MenuItem {
            bool* target = nullptr;
            void onAction(const event::Action& e) override { *target = !*target; }
            void step() override { rightText = (*target) ? "on" : "off"; MenuItem::step(); }
        };
        auto addToggle = [](Menu* menu, bool* v, std::string lbl) {
            auto* it = new BoolItem;
            it->text   = lbl;
            it->target = v;
            menu->addChild(it);
        };
 
        // Flat, with section labels instead of submenus. The set here is
        // deliberately small: everything else that used to be exposed
        // (velocity brightening, bow contact range, tone compensation, output
        // make-up, body string drive, bridge coupling) is a voicing decision
        // rather than a performance control, and now just keeps its tuned
        // default. The members and their patch keys are still there, so a
        // saved patch that moved one of them still loads and sounds the same.
 
        menu->addChild(createMenuLabel("Pluck"));
        addFSlider(menu, &m->stringImpedance, 0.30f, 1.40f, 0.641f, "String Impedance (bow coupling)");
        addFSlider(menu, &m->bodyTilt,        0.f,   1.f,   0.7f,   "Material Brightness Tilt");
        addFSlider(menu, &m->pluckTilt,  0.f, 1.f,  0.6f, "Pluck Tilt");

        menu->addChild(new MenuSeparator());
        menu->addChild(createMenuLabel("Body"));
        // Was the fixed constant TWANG_BRIDGE_COUPLING. 0.35 is the tuned
        // instrument; 0 is a dead bridge and a pure synthetic string, and the
        // top of the range is as far as the reflection stays under unity.
        addFSlider(menu, &m->bridgeCoupling, 0.f, 0.7f, TWANG_BRIDGE_COUPLING,
                   "Bridge Coupling");
 
        menu->addChild(new MenuSeparator());
        menu->addChild(createMenuLabel("Twang"));
        // Labels stay under the width of "Tone Compensation Gain", the longest
        // string that fits the menu on one line. Anything longer wraps and
        // looks broken; explanations belong in comments.
        addFSlider(menu, &m->twangMaxBendSemitones, 0.f, 24.f, 3.f,  "Max Bend (semitones)");
        addFSlider(menu, &m->twangBounceAmount,     0.f, 2.f,  0.5f, "Twang Bounce");
        addFSlider(menu, &m->twangBounceHz,         0.5f, 30.f, 7.f, "Bounce Rate (Hz)");
 
        menu->addChild(new MenuSeparator());
        menu->addChild(createMenuLabel("Pitch"));
        addFSlider(menu, &m->fmDepthSemitones, 0.f, 24.f, 1.f, "FM Depth (semitones)");
 
        menu->addChild(new MenuSeparator());
        menu->addChild(createMenuLabel("Output"));
        addToggle(menu, &m->polyOutput, "Poly Out");
        addFSlider(menu, &m->stereoWidth, 0.f, 1.f, 0.35f, "Stereo Width");
 
        menu->addChild(new MenuSeparator());
        struct PanicItem : MenuItem {
            Twang* module;
            PanicItem(Twang* m) : module(m) { text = "Panic: Clear All Energy"; }
            void onAction(const ActionEvent&) override { if (module) module->panic(); }
        };
        menu->addChild(new PanicItem(m));
    }
};
 
Model* modelTwang = createModel<Twang, TwangWidget>("Twang");