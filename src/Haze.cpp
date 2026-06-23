////////////////////////////////////////////////////////////
//
//   Haze
//
//   written by Cody Geary
//   Copyright 2026, MIT License
//
//   Three-voice stereo chorus.
//   6 Lagrange-interpolated delay lines (3 per channel).
//   LFOs at 0/120/240 deg (L) and 60/180/300 deg (R).
//   Triangle LED display shows LFO polarity per voice.
//
////////////////////////////////////////////////////////////

#include "plugin.hpp"
#include <cmath>
#include <algorithm>
#include "FilterGlass.h"

static constexpr int   HAZE_BUF_SIZE      = 8192;   // power-of-2; ~170ms at 48kHz, ~85ms at 96kHz
static constexpr int   HAZE_BUF_MASK      = HAZE_BUF_SIZE - 1;
static constexpr float HAZE_BASE_DELAY_MS = 12.f;   // center delay (ms)
static constexpr float HAZE_DEPTH_MAX_MS  =  8.f;   // max LFO modulation swing (ms)
static constexpr int   HAZE_VOICES        =  3;

// ─────────────────────────────────────────────────────────────────────────────
// HazeDelayLine
// Ring buffer with Lagrange 4-point fractional read, matching GlassBowl.
// ─────────────────────────────────────────────────────────────────────────────
struct HazeDelayLine {
    float buf[HAZE_BUF_SIZE] = {};
    int   writeIndex = 0;

    void write(float in) {
        buf[writeIndex] = in;
        writeIndex = (writeIndex + 1) & HAZE_BUF_MASK;
    }

    float read(float delaySamples) const {
        delaySamples = clamp(delaySamples, 1.f, (float)HAZE_BUF_SIZE - 4.f);
        float rp   = (float)writeIndex - delaySamples;
        int   base = ((int)floorf(rp)) & HAZE_BUF_MASK;
        float frac = rp - floorf(rp);
        float y0   = buf[(base - 1) & HAZE_BUF_MASK];
        float y1   = buf[base];
        float y2   = buf[(base + 1) & HAZE_BUF_MASK];
        float y3   = buf[(base + 2) & HAZE_BUF_MASK];
        return glassLagrange(y0, y1, y2, y3, frac);
    }

    void clear() {
        std::fill(buf, buf + HAZE_BUF_SIZE, 0.f);
        writeIndex = 0;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// HazeAllpassChain
// Four cascaded Schroeder allpass sections applied to the voice tap OUTPUT only.
// The output never re-enters the feedback path, so there is no nested resonance.
// |H(z)| = 1 per stage: no amplitude coloring, only phase dispersion.
// Four stages accumulate ~15ms of group delay spread -- clearly audible as smear.
//
// Each voice uses a distinct set of prime delay lengths so the three chains
// have different diffusion characters.
//
//   stage: out    = delayed - g * input
//          buf[n] = input  + g * delayed
// ─────────────────────────────────────────────────────────────────────────────
static constexpr int HAZE_AP_STAGES = 4;

struct HazeAllpassStage {
    static constexpr int AP_SIZE = 512;   // power of 2, fits all delay lengths below
    float buf[AP_SIZE] = {};
    int   idx = 0;

    float process(float input, int delayLen, float g) {
        int   readIdx = (idx - delayLen) & (AP_SIZE - 1);
        float delayed = buf[readIdx];
        float out     = delayed - g * input;
        buf[idx]      = (1.f - g * g) * input + g * delayed;  // true Schroeder: buf = x + g*y

        idx           = (idx + 1) & (AP_SIZE - 1);
        return out;
    }

    void clear() { std::fill(buf, buf + AP_SIZE, 0.f); idx = 0; }
};

struct HazeAllpassChain {
    HazeAllpassStage stage[HAZE_AP_STAGES];

    float process(float x, const int* delays, float g) {
        for (int i = 0; i < HAZE_AP_STAGES; ++i)
            x = stage[i].process(x, delays[i], g);
        return x;
    }

    void clear() { for (auto& s : stage) s.clear(); }
};

// Per-voice delay tables -- prime lengths, all < AP_SIZE=512.
// Accumulated group delay at 48kHz:
//   voice 0: 347+211+113+67 = 738 samples ~ 15.4ms
//   voice 1: 251+167+ 89+53 = 560 samples ~ 11.7ms
//   voice 2: 283+149+103+71 = 606 samples ~ 12.6ms
static constexpr int   HAZE_AP_DELAYS[HAZE_VOICES][HAZE_AP_STAGES] = {
    { 347, 211, 113,  67 },
    { 251, 167,  89,  53 },
    { 283, 149, 103,  71 },
};
// Default allpass coefficient. Exposed in the context menu as hazeApCoeff.
// Higher = more diffusion, also slightly more energy per stage.
// The true Schroeder formula keeps |H|=1 regardless of this value.
static constexpr float HAZE_AP_COEFF_DEFAULT = 0.7f;

// ─────────────────────────────────────────────────────────────────────────────
// Forward declaration -- HazeButtonQuantity is defined before Haze but its
// getLabel() implementation needs Haze to be complete, so it is defined
// out-of-line after the Haze struct.
// ─────────────────────────────────────────────────────────────────────────────
struct Haze;

struct HazeButtonQuantity : ParamQuantity {
    int voiceIdx = 0;
    std::string getLabel() override;          // implemented after Haze
    std::string getString() override { return getLabel(); }
};

// ─────────────────────────────────────────────────────────────────────────────
// Haze module
// ─────────────────────────────────────────────────────────────────────────────
struct Haze : Module {

    enum ParamId {
        RATE_PARAM,    RATE_ATT,
        DEPTH_PARAM,   DEPTH_ATT,
        HAZE_PARAM, HAZE_ATT,
        MIX_PARAM,     MIX_ATT,
        DENSITY_PARAM, DENSITY_ATT,
        GAIN_PARAM,
        BUTTON_PARAM_0, BUTTON_PARAM_1, BUTTON_PARAM_2,
        PARAMS_LEN
    };

    enum InputId {
        IN_L_INPUT,
        IN_R_INPUT,
        RATE_CV_INPUT,
        DEPTH_CV_INPUT,
        HAZE_CV_INPUT,
        MIX_CV_INPUT,
        DENSITY_CV_INPUT,
        INPUTS_LEN
    };

    enum OutputId {
        OUT_L_OUTPUT,
        OUT_R_OUTPUT,
        OUTPUTS_LEN
    };

    // Two lights per vertex: red (positive half-cycle) and blue (negative).
    // Stacked at the same panel position to show LFO polarity.
    enum LightId {
        LED0_R, LED0_G, LED0_B,
        LED1_R, LED1_G, LED1_B,
        LED2_R, LED2_G, LED2_B,
        LIGHTS_LEN
    };

    // 3 independent delay lines per channel -- each voice has its own feedback loop.
    HazeDelayLine delayL[HAZE_VOICES];
    HazeDelayLine delayR[HAZE_VOICES];

    // DC blockers on the delay line read output -- one per voice per channel.
    // Prevents hot-signal DC offset from accumulating inside each feedback loop.
    GlassDCBlocker dcBlockL[HAZE_VOICES];
    GlassDCBlocker dcBlockR[HAZE_VOICES];

    // Per-voice one-pole LPF state for tone darkening on the feedback path.
    float lpfZL[HAZE_VOICES] = {};
    float lpfZR[HAZE_VOICES] = {};

    // ADAA saturator on the wet output sum -- prevents feedback accumulation from
    // overloading downstream modules.
    GlassADAADrive saturatorL, saturatorR;

    float lfoPhase = 0.f;

    // LPF cutoff at full Haze, set via context menu. At 20kHz the filter is
    // bypassed entirely (transparent). Saved in patch JSON.
    float hazeMaxCutoff = 2000.f;  // Hz; range 200..20000

    // Allpass coefficient, set via context menu. Higher = more diffusion.
    // The true Schroeder formula keeps |H|=1 at any value in (0,1).
    float hazeApCoeff = HAZE_AP_COEFF_DEFAULT;

    bool allpassMode[3] = {false,false,false};

    // Button edge detectors -- toggle allpassMode on press.
    dsp::BooleanTrigger buttonTrigger[HAZE_VOICES];

    // 4-stage allpass diffusion chains, one per voice per channel.
    // Applied to the tap OUTPUT only -- never feeds back into the delay loop.
    HazeAllpassChain apL[HAZE_VOICES];
    HazeAllpassChain apR[HAZE_VOICES];

    // Per-voice crossfade gain for allpass mode (0=direct, 1=full allpass).
    // Linear ramp over ~3ms avoids the click from a hard mode switch.
    // The chain always runs so it stays warm and the transition is seamless.
    float apGain[HAZE_VOICES] = {};

    // Per-voice LFO sin values -- written by process(), read by step() for LEDs.
    float lfoSinL[HAZE_VOICES] = {};

    // ── Parameter cache ───────────────────────────────────────────────────────
    // Values that depend only on sample rate: computed once in onSampleRateChange.
    float srLpfBright    = 0.073f;  // exp(-2pi*20000/sr) -- upper LPF bound
    float srBaseDelay    = 576.f;   // HAZE_BASE_DELAY_MS * sr * 0.001
    float srApStep       = 1.f / (0.003f * 48000.f);  // 3ms ramp step

    // Values that change at knob/CV rate: decimated every PARAM_DIV samples.
    // Updating at sr/4 (~11kHz) is imperceptible for chorus parameters.
    static constexpr int PARAM_DIV = 4;
    int   paramDiv           = 0;
    float cachedRateHz       = 0.1f;
    float cachedFeedback     = 0.905f;
    float cachedLpfCoeff     = 0.073f;
    float cachedDepthSamples = 0.f;

    Haze() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(RATE_PARAM,    0.f, 1.f, 0.3f,  "Rate");
        configParam(RATE_ATT,     -1.f, 1.f, 0.f,   "Rate Att.");
        configParam(DEPTH_PARAM,   0.f, 1.f, 0.5f,  "Depth");
        configParam(DEPTH_ATT,    -1.f, 1.f, 0.f,   "Depth Att.");
        configParam(HAZE_PARAM,    0.f, 1.f, 0.0f,  "Haze");
        configParam(HAZE_ATT,     -1.f, 1.f, 0.f,   "Haze Att.");
        configParam(MIX_PARAM,     0.f, 1.f, 0.5f,  "Mix (Dry/Wet)");
        configParam(MIX_ATT,      -1.f, 1.f, 0.f,   "Mix Att.");
        configParam(DENSITY_PARAM, 0.f, 1.f, 0.5f,  "Density");
        configParam(DENSITY_ATT,  -1.f, 1.f, 0.f,   "Density Att.");
        configParam(GAIN_PARAM,    0.f, 2.f, 1.0f,  "Gain");
        configParam<HazeButtonQuantity>(BUTTON_PARAM_0, 0.f, 1.f, 0.f, "Node I")->voiceIdx   = 0;
        configParam<HazeButtonQuantity>(BUTTON_PARAM_1, 0.f, 1.f, 0.f, "Node II")->voiceIdx  = 1;
        configParam<HazeButtonQuantity>(BUTTON_PARAM_2, 0.f, 1.f, 0.f, "Node III")->voiceIdx = 2;

        configInput(IN_L_INPUT,       "Audio L");
        configInput(IN_R_INPUT,       "Audio R");
        configInput(RATE_CV_INPUT,    "Rate CV");
        configInput(DEPTH_CV_INPUT,   "Depth CV");
        configInput(HAZE_CV_INPUT,    "Haze CV");
        configInput(DENSITY_CV_INPUT, "Density CV");
        configInput(MIX_CV_INPUT,     "Mix CV");


        configOutput(OUT_L_OUTPUT, "Audio L");
        configOutput(OUT_R_OUTPUT, "Audio R");        
    }

    void processBypass(const ProcessArgs& args) override {
        // Note: Does not support polyphony
        float left = getInput(IN_L_INPUT).getVoltage();
        // Route left input to right output if right input is disconnected
        float right = getInput(IN_R_INPUT).isConnected() ? getInput(IN_R_INPUT).getVoltage() : left;
        getOutput(OUT_L_OUTPUT).setVoltage(left);
        getOutput(OUT_R_OUTPUT).setVoltage(right);
    }

    void onReset() override {
        for (int v = 0; v < HAZE_VOICES; ++v) {
            delayL[v].clear();
            delayR[v].clear();
            dcBlockL[v].reset();
            dcBlockR[v].reset();
            lpfZL[v] = 0.f;
            lpfZR[v] = 0.f;
            apL[v].clear();
            apR[v].clear();
            allpassMode[v] = false;
            apGain[v]      = 0.f;
        }
        saturatorL.reset();
        saturatorR.reset();
        lfoPhase = 0.f;
    }

    void onSampleRateChange(const SampleRateChangeEvent& e) override {
        for (int v = 0; v < HAZE_VOICES; ++v) {
            dcBlockL[v].setSampleRate(e.sampleRate);
            dcBlockR[v].setSampleRate(e.sampleRate);
        }
        srLpfBright = expf(-2.f * float(M_PI) * 20000.f / e.sampleRate);
        srBaseDelay = HAZE_BASE_DELAY_MS * e.sampleRate * 0.001f;
        srApStep    = 1.f / (0.003f * e.sampleRate);
        // Force a full parameter refresh on next process() call.
        paramDiv = PARAM_DIV;
    }

    float getCV(InputId cvId, ParamId attId, float paramVal) {
        float cv = inputs[cvId].isConnected() ? inputs[cvId].getVoltage() : 0.f;
        return paramVal + params[attId].getValue() * cv * 0.1f;
    }

    json_t* dataToJson() override {
        json_t* root = json_object();
        json_object_set_new(root, "hazeMaxCutoff", json_real(hazeMaxCutoff));
        json_object_set_new(root, "hazeApCoeff",   json_real(hazeApCoeff));
        json_object_set_new(root, "allpassMode0",  json_boolean(allpassMode[0]));
        json_object_set_new(root, "allpassMode1",  json_boolean(allpassMode[1]));
        json_object_set_new(root, "allpassMode2",  json_boolean(allpassMode[2]));
        return root;
    }

    void dataFromJson(json_t* root) override {
        json_t* j = json_object_get(root, "hazeMaxCutoff");
        if (j) hazeMaxCutoff = (float)json_number_value(j);
        json_t* jc = json_object_get(root, "hazeApCoeff");
        if (jc) hazeApCoeff = clamp((float)json_number_value(jc), 0.1f, 0.95f);
        json_t* am0 = json_object_get(root, "allpassMode0");
        if (am0) allpassMode[0] = json_boolean_value(am0);
        json_t* am1 = json_object_get(root, "allpassMode1");
        if (am1) allpassMode[1] = json_boolean_value(am1);
        json_t* am2 = json_object_get(root, "allpassMode2");
        if (am2) allpassMode[2] = json_boolean_value(am2);
        // Snap apGain to match restored state so no fade-in on patch load.
        for (int v = 0; v < HAZE_VOICES; ++v)
            apGain[v] = allpassMode[v] ? 1.f : 0.f;
    }

    void process(const ProcessArgs& args) override {

        // ── Cheap params: read every sample ────────────────────────────────────
        float sustainNorm = clamp(getCV(HAZE_CV_INPUT, HAZE_ATT, params[HAZE_PARAM].getValue()), 0.f, 1.f);
        float mixNorm     = clamp(getCV(MIX_CV_INPUT,  MIX_ATT,  params[MIX_PARAM].getValue()),  0.f, 1.f);
        float gain        = params[GAIN_PARAM].getValue();
        float density     = clamp(getCV(DENSITY_CV_INPUT, DENSITY_ATT, params[DENSITY_PARAM].getValue()*1.8f + 0.2f), 0.2f, 2.0f);

        // ── Expensive params: recomputed every PARAM_DIV samples ───────────────
        // powf/expf run only at sr/PARAM_DIV (~11kHz), which is well above any
        // useful modulation rate for a chorus.
        if (++paramDiv >= PARAM_DIV) {
            paramDiv = 0;
            const float sr    = args.sampleRate;
            float rateNorm    = clamp(getCV(RATE_CV_INPUT,  RATE_ATT,  params[RATE_PARAM].getValue()),  0.f, 1.f);
            float depthNorm   = clamp(getCV(DEPTH_CV_INPUT, DEPTH_ATT, params[DEPTH_PARAM].getValue()), 0.f, 1.f);
            cachedRateHz       = 0.01f * powf(80.f, rateNorm);
            float decaySec     = 0.12f + 11.88f * sustainNorm * sustainNorm * sustainNorm;
            cachedFeedback     = expf(-HAZE_BASE_DELAY_MS * 0.001f / decaySec);
            float lpfDark      = (hazeMaxCutoff >= 19900.f) ? 0.f
                               : expf(-2.f * float(M_PI) * hazeMaxCutoff / sr);
            cachedLpfCoeff     = srLpfBright + sustainNorm * sustainNorm * (lpfDark - srLpfBright);
            cachedDepthSamples = depthNorm * depthNorm * HAZE_DEPTH_MAX_MS * sr * 0.001f;
        }

        // ── Inputs ─────────────────────────────────────────────────────────────
        float inL = inputs[IN_L_INPUT].getVoltage();
        float inR = inputs[IN_R_INPUT].isConnected() ? inputs[IN_R_INPUT].getVoltage() : inL;
        inL *= density * gain;
        inR *= density * gain;

        // ── LFO advance ────────────────────────────────────────────────────────
        lfoPhase += cachedRateHz / args.sampleRate;
        if (lfoPhase >= 1.f) lfoPhase -= 1.f;

        // ── Button toggles ─────────────────────────────────────────────────────
        for (int v = 0; v < HAZE_VOICES; ++v) {
            if (buttonTrigger[v].process(params[BUTTON_PARAM_0 + v].getValue() > 0.5f))
                allpassMode[v] = !allpassMode[v];
        }

        // ── LFO sines: 2 trig calls, all 6 values derived by identity ──────────
        // All voices share the same lfoPhase with fixed offsets, so only one
        // sin+cos pair is needed. On ARM the compiler fuses sinf/cosf into sincos.
        // L: 0, 120, 240 deg.  R = -(L rotated by one step) -- pure negation, free.
        //   L0 =  s
        //   L1 = (√3·c − s) / 2
        //   L2 = −(s + √3·c) / 2
        //   R0 = −L2,  R1 = −L0,  R2 = −L1
        const float theta = lfoPhase * float(2.0 * M_PI);
        const float s     = sinf(theta);
        const float c     = cosf(theta);
        const float sc3   = c * 1.7320508f;  // √3·cos(θ), reused in all three L values
        const float sinLv[3] = { s,  (sc3 - s) * 0.5f,  -(s + sc3) * 0.5f };
        const float sinRv[3] = { -sinLv[2], -sinLv[0], -sinLv[1] };

        // ── Three independent chorus voices per channel ─────────────────────────
        float wetL = 0.f, wetR = 0.f;

        for (int v = 0; v < HAZE_VOICES; ++v) {
            float sinL = sinLv[v];
            float sinR = sinRv[v];

            float delayTimeL = srBaseDelay + sinL * cachedDepthSamples;
            float delayTimeR = srBaseDelay + sinR * cachedDepthSamples;

            float outL = delayL[v].read(delayTimeL);
            float outR = delayR[v].read(delayTimeR);

            // DC block the delay output before it re-enters the feedback loop.
            outL = dcBlockL[v].process(outL);
            outR = dcBlockR[v].process(outR);

            // One-pole LPF for tone darkening -- lpfCoeff=0 is transparent.
            lpfZL[v] = (1.f - cachedLpfCoeff) * outL + cachedLpfCoeff * lpfZL[v];
            lpfZR[v] = (1.f - cachedLpfCoeff) * outR + cachedLpfCoeff * lpfZR[v];
            outL = lpfZL[v];
            outR = lpfZR[v];

            // Write: feedback uses the pre-allpass signal.
            delayL[v].write(clamp(inL + cachedFeedback * outL, -12.f, 12.f));
            delayR[v].write(clamp(inR + cachedFeedback * outR, -12.f, 12.f));

            // Allpass diffusion chains run unconditionally so state stays warm.
            float apOutL = apL[v].process(outL, HAZE_AP_DELAYS[v], hazeApCoeff);
            float apOutR = apR[v].process(outR, HAZE_AP_DELAYS[v], hazeApCoeff);

            // Linear 3ms crossfade toward the target gain.
            float apTarget = allpassMode[v] ? 1.f : 0.f;
            if (apGain[v] < apTarget) apGain[v] = std::min(apGain[v] + srApStep, apTarget);
            else                      apGain[v] = std::max(apGain[v] - srApStep, apTarget);

            outL = outL + apGain[v] * (apOutL - outL);
            outR = outR + apGain[v] * (apOutR - outR);

            wetL += outL;
            wetR += outR;

            // Store for step() -- LEDs are updated at frame rate, not audio rate.
            lfoSinL[v] = sinL;
        }

        wetL *= (1.f / HAZE_VOICES);
        wetR *= (1.f / HAZE_VOICES);

        // ADAA saturator on the wet sum.
        // GlassADAADrive has 0.69x gain at small signal (out = inV * 6.9/10),
        // so compensate by 10/6.9 to restore unity gain in the clean range.
        const float satCompensation = 10.f / 6.9f;
        float satWetL = saturatorL.process(wetL) * satCompensation;
        float satWetR = saturatorR.process(wetR) * satCompensation;

        // ── Dry/wet blend ──────────────────────────────────────────────────────
        //density 0.2 .. 2.0
        float correction = 1.f/density;
        float correctionClamped = clamp(correction, 0.5f, 1.5f);
        
        outputs[OUT_L_OUTPUT].setVoltage((inL * (1.f - mixNorm)* correction + satWetL * mixNorm * correctionClamped));
        outputs[OUT_R_OUTPUT].setVoltage((inR * (1.f - mixNorm)* correction + satWetR * mixNorm * correctionClamped));
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// HazeButtonQuantity::getLabel() -- defined here so Haze is complete.
// Tooltip reads "Node I: Chorus" or "Node I: Diffuse" depending on live state.
// ─────────────────────────────────────────────────────────────────────────────
std::string HazeButtonQuantity::getLabel() {
    Haze* m = dynamic_cast<Haze*>(this->module);
    if (!m) return name;
    static const char* labels[] = { "Node I", "Node II", "Node III" };
    return std::string(labels[voiceIdx])
         + (m->allpassMode[voiceIdx] ? ": Diffuse" : ": Chorus");
}

// ─────────────────────────────────────────────────────────────────────────────
// Context menu: Allpass diffusion coefficient
// ─────────────────────────────────────────────────────────────────────────────
struct HazeApCoeffQuantity : Quantity {
    Haze* module;
    HazeApCoeffQuantity(Haze* m) : module(m) {}
    void  setValue(float v) override { module->hazeApCoeff = clamp(v, 0.1f, 0.95f); }
    float getValue()          override { return module->hazeApCoeff; }
    float getDefaultValue()   override { return HAZE_AP_COEFF_DEFAULT; }
    float getMinValue()       override { return 0.1f; }
    float getMaxValue()       override { return 0.95f; }
    std::string getLabel()    override { return "Diffuse coefficient"; }
    std::string getUnit()     override { return ""; }
    int getDisplayPrecision() override { return 2; }
};

struct HazeApCoeffSlider : ui::Slider {
    HazeApCoeffQuantity* qty;
    HazeApCoeffSlider(Haze* m) {
        qty      = new HazeApCoeffQuantity(m);
        quantity = qty;
        box.size.x = 200.f;
    }
    ~HazeApCoeffSlider() { delete qty; quantity = nullptr; }
};

// ─────────────────────────────────────────────────────────────────────────────
// Slider -- reuses ShortSlider SVG assets shared across the collection
// ─────────────────────────────────────────────────────────────────────────────
struct HazeSliderBase : app::SvgSlider {
    HazeSliderBase() {
        setBackgroundSvg(Svg::load(asset::plugin(pluginInstance,
            "res/components/ShortSlider.svg")));
        setHandleSvg(Svg::load(asset::plugin(pluginInstance,
            "res/components/ShortSliderHandle.svg")));
        setHandlePosCentered(math::Vec(10.f, 55.f), math::Vec(10.f, 10.f));
    }
};
template <typename TL = BlueLight>
struct HazeSlider : LightSlider<HazeSliderBase, VCVSliderLight<TL>> {};

// ─────────────────────────────────────────────────────────────────────────────
// Context menu: Haze dark cutoff frequency
// HazeCutoffSlider owns its Quantity and deletes it on destruction,
// so there is no memory leak when the menu closes.
// ─────────────────────────────────────────────────────────────────────────────
struct HazeCutoffQuantity : Quantity {
    Haze* module;
    HazeCutoffQuantity(Haze* m) : module(m) {}
    void  setValue(float v) override { module->hazeMaxCutoff = clamp(v, 200.f, 20000.f); }
    float getValue()          override { return module->hazeMaxCutoff; }
    float getDefaultValue()   override { return 8000.f; }
    float getMinValue()       override { return  200.f; }
    float getMaxValue()       override { return 20000.f; }
    std::string getLabel()    override { return "Haze Filter"; }
    std::string getUnit()     override { return " Hz"; }
    int getDisplayPrecision() override { return 5; }
};

struct HazeCutoffSlider : ui::Slider {
    HazeCutoffQuantity* qty;
    HazeCutoffSlider(Haze* m) {
        qty      = new HazeCutoffQuantity(m);
        quantity = qty;
        box.size.x = 200.f;
    }
    ~HazeCutoffSlider() {
        delete qty;
        quantity = nullptr;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// HazeWidget -- 8HP
// ─────────────────────────────────────────────────────────────────────────────
struct HazeWidget : ModuleWidget {

    static Vec p(float x, float y) { return mm2px(Vec(x, y)); }

    HazeWidget(Haze* module) {
        setModule(module);
        box.size = Vec(8 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT);

        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Haze.svg"),
            asset::plugin(pluginInstance, "res/Haze-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // ── Triangle LED display ──────────────────────────────────────────────
        // Equilateral triangle: circumradius 7mm, center (20.32, 18) mm.
        //   Vertex 0 (top):          (20.32, 11.0)
        //   Vertex 1 (bottom-right): (26.38, 21.5)
        //   Vertex 2 (bottom-left):  (14.26, 21.5)
        // Yellow and blue LargeLights stacked at each vertex for polarity display.
        const Vec triV[3] = {
            p(20.32f, 11.0f+5.f),
            p(26.38f, 21.5f+5.f),
            p(14.26f, 21.5f+5.f),
        };
        for (int v = 0; v < 3; ++v) {
            addParam(createParamCentered<TL1105>(  triV[v] , module, Haze::BUTTON_PARAM_0 + v));
            addChild(createLightCentered<LargeLight<RedLight>> (triV[v], module, Haze::LED0_R + v * 3));
            addChild(createLightCentered<LargeLight<GreenLight>>   (triV[v], module, Haze::LED0_G + v * 3));
            addChild(createLightCentered<LargeLight<BlueLight>>  (triV[v], module, Haze::LED0_B + v * 3));
        }

        // ── Sliders: Rate, Depth, Sustain -- Y-values matching collection ─────
        const float sx[3]   = { 9.f, 20.32f, 31.64f };
        const float ySlider = 46.f;
        const float yTrim   = 57.f;
        const float yCv     = 65.f;

        addParam(createParamCentered<HazeSlider<BlueLight>>(   p(sx[0], ySlider), module, Haze::RATE_PARAM));
        addParam(createParamCentered<Trimpot>(                  p(sx[0], yTrim),   module, Haze::RATE_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(         p(sx[0], yCv),     module, Haze::RATE_CV_INPUT));

        addParam(createParamCentered<HazeSlider<YellowLight>>( p(sx[1], ySlider), module, Haze::DEPTH_PARAM));
        addParam(createParamCentered<Trimpot>(                  p(sx[1], yTrim),   module, Haze::DEPTH_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(         p(sx[1], yCv),     module, Haze::DEPTH_CV_INPUT));

        addParam(createParamCentered<HazeSlider<GreenLight>>(  p(sx[2], ySlider), module, Haze::HAZE_PARAM));
        addParam(createParamCentered<Trimpot>(                  p(sx[2], yTrim),   module, Haze::HAZE_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(         p(sx[2], yCv),     module, Haze::HAZE_CV_INPUT));

        // ── Mix row: CV -- Trim -- Knob ───────────────────────────────────────
        const float yMix = 78.f;
        addInput(createInputCentered<ThemedPJ301MPort>(    p(sx[0], yMix), module, Haze::DENSITY_CV_INPUT));
        addParam(createParamCentered<Trimpot>(             p(sx[1], yMix), module, Haze::DENSITY_ATT));
        addParam(createParamCentered<RoundSmallBlackKnob>( p(sx[2], yMix), module, Haze::DENSITY_PARAM));

        addInput(createInputCentered<ThemedPJ301MPort>(    p(sx[0], yMix + 14.f), module, Haze::MIX_CV_INPUT));
        addParam(createParamCentered<Trimpot>(             p(sx[1], yMix + 14.f), module, Haze::MIX_ATT));
        addParam(createParamCentered<RoundSmallBlackKnob>( p(sx[2], yMix + 14.f), module, Haze::MIX_PARAM));

        // ── Audio I/O -- inputs left column, outputs right column ─────────────
        addInput(createInputCentered<ThemedPJ301MPort>( p( 9.f,    92.f+13.f), module, Haze::IN_L_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>( p( 9.f,   105.f+10.f), module, Haze::IN_R_INPUT));

        addParam(createParamCentered<RoundSmallBlackKnob>( p((9.f+31.64f)/2.f, (92.f+13.f+105.f+10.f)/2.f), module, Haze::GAIN_PARAM));        
      
        addOutput(createOutputCentered<ThemedPJ301MPort>(p(31.64f,  92.f+13.f), module, Haze::OUT_L_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(p(31.64f, 105.f+10.f), module, Haze::OUT_R_OUTPUT));
    }

    void step() override {
        Haze* module = dynamic_cast<Haze*>(this->module);
        if (!module) return;

        for (int v = 0; v < HAZE_VOICES; ++v) {
            float pos = clamp( module->lfoSinL[v], 0.f, 1.f);
            float neg = clamp(-module->lfoSinL[v], 0.f, 1.f);
            if (!module->allpassMode[v]) {
                // Chorus: Blue <-> Yellow
                module->lights[Haze::LED0_R + v * 3].setBrightness(neg);
                module->lights[Haze::LED0_G + v * 3].setBrightness(neg);
                module->lights[Haze::LED0_B + v * 3].setBrightness(pos);
            } else {
                // Diffuse: Purple <-> Green
                module->lights[Haze::LED0_R + v * 3].setBrightness(pos);
                module->lights[Haze::LED0_G + v * 3].setBrightness(neg);
                module->lights[Haze::LED0_B + v * 3].setBrightness(pos);
            }
        }

        ModuleWidget::step();
    }

    void appendContextMenu(Menu* menu) override {
        Haze* module = dynamic_cast<Haze*>(this->module);
        if (!module) return;
        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Haze dark cutoff (20kHz = off)"));
        menu->addChild(new HazeCutoffSlider(module));
        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Diffuse mode coefficient"));
        menu->addChild(new HazeApCoeffSlider(module));
    }
};

Model* modelHaze = createModel<Haze, HazeWidget>("Haze");