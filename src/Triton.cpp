////////////////////////////////////////////////////////////
//
//   Triton
//
//   written by Cody Geary
//   Copyright 2026, MIT License
//
//   Stereo three-band spectral splitter with envelope followers.
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
using namespace rack;
#include <cmath>
#include <functional>
#include "FilterTriton.h"

// ─────────────────────────────────────────────────────────────────────────────
struct OnePole {
    float z = 0.f;
    float process(float in, float coeff) { z += coeff*(in-z); return z; }
    void reset() { z = 0.f; }
};

// ─────────────────────────────────────────────────────────────────────────────
// ADAA tanh — exact Node.cpp chain
// ─────────────────────────────────────────────────────────────────────────────
struct ADAADrive {
    float lastInput = 0.f;
    static float polyTanh(float x) {
        float x2=x*x;
        return x - x*x2*(1.f/3.f - x2*(2.f/15.f - 17.f/315.f*x2));
    }
    static float antiderivative(float x) {
        float x2=x*x;
        return 0.5f*x2-(1.f/12.f)*x2*x2+(1.f/45.f)*x2*x2*x2-(17.f/2520.f)*x2*x2*x2*x2;
    }
    static float applyADAA(float input, float last) {
        float d=input-last;
        return fabsf(d)>1e-6f ? (antiderivative(input)-antiderivative(last))/d : polyTanh(input);
    }
    float process(float inV, float driveGain) {
        float sig  = clamp(inV*driveGain, -13.14f, 13.14f);
        float norm = sig/10.f;
        float out  = applyADAA(norm, lastInput);
        lastInput  = norm;
        return clamp(out*6.9f, -10.f, 10.f);
    }
    void reset() { lastInput=0.f; }
};

// ─────────────────────────────────────────────────────────────────────────────
struct EnvFollower {
    float rms=0.f, coeff=0.001f;
    void setCoeff(float bandCenterNorm, float followTime, float sampleRate) {
        float periodMs=(bandCenterNorm>1e-4f)?(1.f/(bandCenterNorm*sampleRate))*1000.f:2000.f;
        periodMs=rack::clamp(periodMs,0.5f,2000.f);
        float scale=0.5f+followTime*followTime*299.5f;
        coeff=1.f-expf(-1.f/((periodMs*scale*0.001f)*sampleRate));
    }
    float process(float in) {
        rms+=coeff*(in*in-rms);
        return sqrtf(rms<0.f?0.f:rms);
    }
    void reset() { rms=0.f; }
};

// ─────────────────────────────────────────────────────────────────────────────
struct CenterHzQuantity : ParamQuantity {
    float getDisplayValue() override { return dsp::FREQ_C4*dsp::exp2_taylor5(getValue()); }
    void setDisplayValue(float hz) override { setValue(log2f((hz>0.f?hz:1.f)/dsp::FREQ_C4)); }
    std::string getDisplayValueString() override {
        float hz=getDisplayValue();
        return hz>=1000.f?string::f("%.2f k",hz/1000.f):string::f("%.1f",hz);
    }
    std::string getUnit() override { return " Hz"; }
};

// =============================================================================
struct Triton : Module {

    // Width mapping — bitmask so multiple params can be mapped simultaneously
    enum WidthBit { W_CENTER=1, W_SPREAD=2, W_GAP=4, W_SHARP=8, W_RES=16, W_DRIVE=32 };

    enum ParamId {
        // Filter row 1
        CENTER_PARAM,     CENTER_TRIM_PARAM,
        SPREAD_PARAM,     SPREAD_TRIM_PARAM,
        GAP_PARAM,        GAP_TRIM_PARAM,
        // Filter row 2
        SHARP_PARAM,      SHARP_TRIM_PARAM,
        RES_PARAM,        RES_TRIM_PARAM,
        DRIVE_PARAM,      DRIVE_TRIM_PARAM,
        // Band levels
        LOW_LEVEL_PARAM,  LOW_LEVEL_TRIM_PARAM,
        MID_LEVEL_PARAM,  MID_LEVEL_TRIM_PARAM,
        HIGH_LEVEL_PARAM, HIGH_LEVEL_TRIM_PARAM,
        MIX_LEVEL_PARAM, MIX_LEVEL_TRIM_PARAM,
        // Width
        WIDTH_PARAM,      WIDTH_TRIM_PARAM,
        // Per-slider width map buttons — one per mappable filter param
        MAP_CENTER_PARAM, MAP_SPREAD_PARAM, MAP_GAP_PARAM,
        MAP_SHARP_PARAM,  MAP_RES_PARAM,    MAP_DRIVE_PARAM,
        PARAMS_LEN
    };

    enum InputId {
        AUDIO_L_INPUT, AUDIO_R_INPUT,
        VOCT_INPUT,
        CENTER_CV_INPUT,
        SPREAD_CV_INPUT,
        GAP_CV_INPUT,
        SHARP_CV_INPUT,
        RES_CV_INPUT,
        DRIVE_CV_INPUT,
        LOW_LEVEL_CV_INPUT,
        MID_LEVEL_CV_INPUT,
        HIGH_LEVEL_CV_INPUT,
        MIX_LEVEL_CV_INPUT,
        WIDTH_CV_INPUT,
        INPUTS_LEN
    };

    enum OutputId {
        LOW_L_OUTPUT,  LOW_R_OUTPUT,
        MID_L_OUTPUT,  MID_R_OUTPUT,
        HIGH_L_OUTPUT, HIGH_R_OUTPUT,
        LOW_ENV_OUTPUT,  LOW_INV_OUTPUT,
        MID_ENV_OUTPUT,  MID_INV_OUTPUT,
        HIGH_ENV_OUTPUT, HIGH_INV_OUTPUT,
        SUM_L_OUTPUT,  SUM_R_OUTPUT,
        MIX_ENV_OUTPUT,
        OUTPUTS_LEN
    };

    enum LightId {
        // LEDs above each filter slider showing width mapping target
        LED_CENTER, LED_SPREAD, LED_GAP, LED_SHARP, LED_RES, LED_DRIVE,
        // Envelope level indicators
        LED_ENV_LOW, LED_ENV_MID, LED_ENV_HIGH, LED_ENV_MIX,
        LIGHTS_LEN
    };

    // ── DSP — Left channel ────────────────────────────────────────────────────
    FilterTriton lpLowL, hpLowL, lpHighL, hpHighL;
    DCBlocker    dcBlockL;
    NyquistCap   nyqCapL;
    ADAADrive    driveL;

    // ── DSP — Right channel ───────────────────────────────────────────────────
    FilterTriton lpLowR, hpLowR, lpHighR, hpHighR;
    DCBlocker    dcBlockR;
    NyquistCap   nyqCapR;
    ADAADrive    driveR;

    // Envelope followers (tap L channel)
    EnvFollower  envLow, envMid, envHigh;

    // Smoothers
    OnePole smCenter, smSpread, smGap, smSharp, smRes, smDrive, smWidth;
    OnePole smLowLvl, smMidLvl, smHighLvl, smMixLvl;

    // Width mapping — bitmask, each bit enables one parameter
    int  widthTarget = 0;
    // One trigger per map button
    dsp::SchmittTrigger mapTriggers[6];  // CENTER SPREAD GAP SHARP RES DRIVE

    // Clock divider — param reads and filter coefficient updates run
    // every PARAM_STRIDE samples instead of every sample.
    // At 44.1kHz and stride=32: param rate ≈ 1.4kHz, inaudible given smoothers.
    static const int PARAM_STRIDE = 32;
    dsp::ClockDivider paramDivider;

    // Cached smoothed values — written by param tier, read by audio tier
    float cachedDriveGainL = 1.f, cachedDriveGainR = 1.f;
    float cachedLowLvl = 1.f, cachedMidLvl = 1.f, cachedHighLvl = 1.f, cachedMixLvl = 1.f;

    // Dirty flag — set when filter coeffs change, cleared after display copy
    bool coeffsDirty = true;

    // Follow time (context menu, 0..1)
    float followTime = 0.12f;

    // Display state
    float displayFcLow  = 0.05f, displayFcHigh = 0.25f;
    float displayEnvLow = 0.f,   displayEnvMid = 0.f, displayEnvHigh = 0.f;
    BiquadCoeffs dispLpLow[TRITON_STAGES], dispHpLow[TRITON_STAGES];
    BiquadCoeffs dispLpHigh[TRITON_STAGES], dispHpHigh[TRITON_STAGES];
    BiquadCoeffs dispLpLowR[TRITON_STAGES], dispHpLowR[TRITON_STAGES];
    BiquadCoeffs dispLpHighR[TRITON_STAGES], dispHpHighR[TRITON_STAGES];
    float dispSharp = 1.f, dispSharpR = 1.f;
    float displayFcLowR = 0.05f, displayFcHighR = 0.25f;
    float sampleRate = 44100.f;

    // Feedback system
    bool feedbackEnabled = false;    
    float mixL = 0.f; //used for feedback
    float mixR = 0.f;
    float feedbackL = 0.f, feedbackR = 0.f;
    float fbGainL = 0.f, fbGainR = 0.f;        // actual feedback gains, audio-rate smoothed
    float fbEnvL  = 0.f, fbEnvR  = 0.f;        // peak envelope trackers for AGC

    // ── JSON ──────────────────────────────────────────────────────────────────
    json_t* dataToJson() override {
        json_t* r = json_object();
        json_object_set_new(r, "widthTarget", json_integer(widthTarget));
        json_object_set_new(r, "followTime",  json_real(followTime));
        json_object_set_new(r, "feedbackEnabled", json_boolean(feedbackEnabled));
        return r;
    }
    void dataFromJson(json_t* r) override {
        json_t* j;
        j = json_object_get(r,"widthTarget"); if (j) widthTarget=json_integer_value(j);
        j = json_object_get(r,"followTime");  if (j) followTime =json_real_value(j);
        j = json_object_get(r,"feedbackEnabled"); if (j) feedbackEnabled=json_boolean_value(j);
    }

    // ── Constructor ───────────────────────────────────────────────────────────
    Triton() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam<CenterHzQuantity>(CENTER_PARAM, -2.f, 4.f, 0.f, "Center");
        configParam(CENTER_TRIM_PARAM,    -2.f, 2.f, 0.f, "Center CV Trim");
        configParam(SPREAD_PARAM,          0.f, 1.f, 0.4f,"Spread");
        configParam(SPREAD_TRIM_PARAM,    -2.f, 2.f, 0.f, "Spread CV Trim");
        configParam(GAP_PARAM,            -1.f, 1.f, 0.f, "Gap");
        configParam(GAP_TRIM_PARAM,       -2.f, 2.f, 0.f, "Gap CV Trim");
        configParam(SHARP_PARAM,           0.f, 1.f, 1.f, "Sharpness");
        configParam(SHARP_TRIM_PARAM,     -2.f, 2.f, 0.f, "Sharpness CV Trim");
        configParam(RES_PARAM,             0.f, 1.f, 0.f, "Resonance");
        configParam(RES_TRIM_PARAM,       -2.f, 2.f, 0.f, "Resonance CV Trim");
        configParam(DRIVE_PARAM,           0.f, 1.f, 0.f, "Drive");
        configParam(DRIVE_TRIM_PARAM,     -2.f, 2.f, 0.f, "Drive CV Trim");
        configParam(LOW_LEVEL_PARAM,       0.f, 1.f, 1.f, "Low Level");
        configParam(LOW_LEVEL_TRIM_PARAM, -2.f, 2.f, 0.f, "Low Level CV Trim");
        configParam(MID_LEVEL_PARAM,       0.f, 1.f, 1.f, "Mid Level");
        configParam(MID_LEVEL_TRIM_PARAM, -2.f, 2.f, 0.f, "Mid Level CV Trim");
        configParam(HIGH_LEVEL_PARAM,      0.f, 1.f, 1.f, "High Level");
        configParam(HIGH_LEVEL_TRIM_PARAM,-2.f, 2.f, 0.f, "High Level CV Trim");
        configParam(MIX_LEVEL_PARAM,       0.f, 1.f, 1.f,  "Mix Level");
        configParam(MIX_LEVEL_TRIM_PARAM, -2.f, 2.f, 0.f,  "Mix Level CV Trim");
        configParam(WIDTH_PARAM,          -1.f, 1.f, 0.f, "Width");
        configParam(WIDTH_TRIM_PARAM,     -2.f, 2.f, 0.f, "Width CV Trim");
        configParam(MAP_CENTER_PARAM,      0.f, 1.f, 0.f, "Map Width to Center");
        configParam(MAP_SPREAD_PARAM,      0.f, 1.f, 0.f, "Map Width to Spread");
        configParam(MAP_GAP_PARAM,         0.f, 1.f, 0.f, "Map Width to Gap");
        configParam(MAP_SHARP_PARAM,       0.f, 1.f, 0.f, "Map Width to Sharpness");
        configParam(MAP_RES_PARAM,         0.f, 1.f, 0.f, "Map Width to Resonance");
        configParam(MAP_DRIVE_PARAM,       0.f, 1.f, 0.f, "Map Width to Drive");

        configInput(AUDIO_L_INPUT,      "Audio L");
        configInput(AUDIO_R_INPUT,      "Audio R");
        configInput(VOCT_INPUT,         "V/Oct");
        configInput(CENTER_CV_INPUT,    "Center CV");
        configInput(SPREAD_CV_INPUT,    "Spread CV");
        configInput(GAP_CV_INPUT,       "Gap CV");
        configInput(SHARP_CV_INPUT,     "Sharpness CV");
        configInput(RES_CV_INPUT,       "Resonance CV");
        configInput(DRIVE_CV_INPUT,     "Drive CV");
        configInput(LOW_LEVEL_CV_INPUT, "Low Level CV");
        configInput(MID_LEVEL_CV_INPUT, "Mid Level CV");
        configInput(HIGH_LEVEL_CV_INPUT,"High Level CV");
        configInput(MIX_LEVEL_CV_INPUT, "Mix Level CV");
        configInput(WIDTH_CV_INPUT,     "Width CV");

        configOutput(LOW_L_OUTPUT,   "Low L");
        configOutput(LOW_R_OUTPUT,   "Low R");
        configOutput(MID_L_OUTPUT,   "Mid L");
        configOutput(MID_R_OUTPUT,   "Mid R");
        configOutput(HIGH_L_OUTPUT,  "High L");
        configOutput(HIGH_R_OUTPUT,  "High R");
        configOutput(LOW_ENV_OUTPUT, "Low Envelope");
        configOutput(MID_ENV_OUTPUT, "Mid Envelope");
        configOutput(HIGH_ENV_OUTPUT,"High Envelope");
        configOutput(SUM_L_OUTPUT,   "Sum L");
        configOutput(SUM_R_OUTPUT,   "Sum R");
        configOutput(MIX_ENV_OUTPUT, "Mix Envelope");

        lpLowL.setMode(FilterTriton::LOWPASS);
        hpLowL.setMode(FilterTriton::HIGHPASS);
        lpHighL.setMode(FilterTriton::LOWPASS);
        hpHighL.setMode(FilterTriton::HIGHPASS);
        lpLowR.setMode(FilterTriton::LOWPASS);
        hpLowR.setMode(FilterTriton::HIGHPASS);
        lpHighR.setMode(FilterTriton::LOWPASS);
        hpHighR.setMode(FilterTriton::HIGHPASS);

        for (int i=0;i<TRITON_STAGES;i++) {
            dispLpLow[i]=dispHpLow[i]=dispLpHigh[i]=dispHpHigh[i]=BiquadCoeffs{};
            dispLpLowR[i]=dispHpLowR[i]=dispLpHighR[i]=dispHpHighR[i]=BiquadCoeffs{};
        }

        paramDivider.setDivision(PARAM_STRIDE);

        // Initialise with safe defaults so filters are valid before first param update
        dcBlockL.setSampleRate(44100.f); dcBlockR.setSampleRate(44100.f);
        nyqCapL.setSampleRate(44100.f);  nyqCapR.setSampleRate(44100.f);
        float defaultFc = 0.1f;
        lpLowL.setParameters(defaultFc,1.f,0.f); hpLowL.setParameters(defaultFc,1.f,0.f);
        lpHighL.setParameters(defaultFc,1.f,0.f); hpHighL.setParameters(defaultFc,1.f,0.f);
        lpLowR.setParameters(defaultFc,1.f,0.f); hpLowR.setParameters(defaultFc,1.f,0.f);
        lpHighR.setParameters(defaultFc,1.f,0.f); hpHighR.setParameters(defaultFc,1.f,0.f);
    }

    void onSampleRateChange(const SampleRateChangeEvent& e) override {
        sampleRate=e.sampleRate;
        dcBlockL.setSampleRate(sampleRate); dcBlockR.setSampleRate(sampleRate);
        nyqCapL.setSampleRate(sampleRate);  nyqCapR.setSampleRate(sampleRate);
    }

    // ── Helper: compute cutoff frequencies from base params + optional offset ─
    struct CutoffSet { float lpLow, hpLow, lpHigh, hpHigh, fA, fB; };

    CutoffSet computeCutoffs(float centerV, float spread, float gap, float sr) {
        float centerHz = dsp::FREQ_C4 * dsp::exp2_taylor5(centerV);
        float spreadOct = 0.5f + spread * 2.5f;
        float gapOct    = gap * 2.0f;              // ±2 oct total range

        float fA = centerHz * powf(2.f, -spreadOct);
        float fB = centerHz * powf(2.f,  spreadOct);

        float fAlow  = fA * powf(2.f, -gapOct*0.5f);
        float fAhigh = fA * powf(2.f,  gapOct*0.5f);
        float fBlow  = fB * powf(2.f, -gapOct*0.5f);
        float fBhigh = fB * powf(2.f,  gapOct*0.5f);

        float minFc = 20.f/sr;
        CutoffSet c;
        c.lpLow  = clamp(fAlow  /sr, minFc, 0.47f);
        c.hpLow  = clamp(fAhigh /sr, minFc, 0.47f);
        c.lpHigh = clamp(fBlow  /sr, minFc, 0.47f);
        c.hpHigh = clamp(fBhigh /sr, minFc, 0.47f);
        c.fA     = fA/sr;
        c.fB     = fB/sr;
        return c;
    }

    // ── process ───────────────────────────────────────────────────────────────
    void process(const ProcessArgs& args) override {
        sampleRate = args.sampleRate;

        // ── PARAM TIER — runs every PARAM_STRIDE samples ──────────────────────
        // All expensive non-audio work: param reads, smoothing, filter coeff
        // updates, envelope follower coeff updates. Safe because all CV inputs
        // are smoothed — a 32-sample delay is inaudible (~0.7ms at 44.1kHz).
        if (paramDivider.process()) {

            // Per-slider map buttons — each toggles its bit in the widthTarget bitmask
            struct { ParamId p; int bit; } mapBtns[6] = {
                {MAP_CENTER_PARAM, W_CENTER}, {MAP_SPREAD_PARAM, W_SPREAD},
                {MAP_GAP_PARAM,    W_GAP   }, {MAP_SHARP_PARAM,  W_SHARP },
                {MAP_RES_PARAM,    W_RES   }, {MAP_DRIVE_PARAM,  W_DRIVE }
            };
            for (int i = 0; i < 6; i++) {
                if (mapTriggers[i].process(params[mapBtns[i].p].getValue()))
                    widthTarget ^= mapBtns[i].bit;  // toggle bit
            }

            auto readCV = [&](InputId i) -> float {
                return inputs[i].isConnected() ? inputs[i].getVoltage() : 0.f;
            };

            float centerV  = params[CENTER_PARAM].getValue()
                           + readCV(VOCT_INPUT)
                           + readCV(CENTER_CV_INPUT)*params[CENTER_TRIM_PARAM].getValue();
            float spread   = clamp(params[SPREAD_PARAM].getValue()
                           + readCV(SPREAD_CV_INPUT)*0.1f*params[SPREAD_TRIM_PARAM].getValue(), 0.f,1.f);
            float gap      = clamp(2.f*params[GAP_PARAM].getValue()
                           + readCV(GAP_CV_INPUT)*0.2f*params[GAP_TRIM_PARAM].getValue(), -2.f,2.f);
            float sharp    = clamp(params[SHARP_PARAM].getValue()
                           + readCV(SHARP_CV_INPUT)*0.1f*params[SHARP_TRIM_PARAM].getValue(), 0.f,1.f);
            float res      = clamp(params[RES_PARAM].getValue()
                           + readCV(RES_CV_INPUT)*0.1f*params[RES_TRIM_PARAM].getValue(), 0.f,1.f);
            float driveKnob= clamp(params[DRIVE_PARAM].getValue()
                           + readCV(DRIVE_CV_INPUT)*0.1f*params[DRIVE_TRIM_PARAM].getValue(), 0.f,1.f);
            float width    = clamp(params[WIDTH_PARAM].getValue()
                           + readCV(WIDTH_CV_INPUT)*0.1f*params[WIDTH_TRIM_PARAM].getValue(), -1.f,1.f);

            float centerSc  = smCenter.process(centerV,  0.04f);   // coeff scaled for sub-rate
            float spreadSc = smSpread.process(spread,   0.04f);
            float gapSc  = smGap.process   (gap,      0.04f);
            float sharpSc = smSharp.process (sharp,    0.06f);
            float resSc  = smRes.process   (res,      0.04f);
            float driveSc  = smDrive.process (driveKnob,0.16f);
            float widthSc  = smWidth.process (width,    0.16f);

            // Non-linear response curves — applied after smoothing, before width offset
            // Sharpness: most action in top quarter — 1-(1-x)^3 stretches that range out
            sharpSc = clamp(1.f - (1.f-sharpSc)*(1.f-sharpSc)*(1.f-sharpSc), 0.f, 1.f);
            // Resonance: most action in bottom fifth — x^3 keeps it subtle until pushed
            resSc  = clamp(resSc*resSc*resSc, 0.f, 1.f)*0.5f;

            // Width pushes L and R symmetrically in opposite directions.
            // Each active bit contributes its offset — multiple bits stack.
            float centerL=centerSc,  spreadL=spreadSc,  gapL=gapSc,  sharpL=sharpSc,  resL=resSc,  driveL=driveSc;
            float centerR=centerSc,  spreadR=spreadSc,  gapR=gapSc,  sharpR=sharpSc,  resR=resSc,  driveR=driveSc;
            if (widthTarget & W_CENTER) {
                centerL = centerSc  - widthSc*2.f;
                centerR = centerSc  + widthSc*2.f;
            }
            if (widthTarget & W_SPREAD) {
                spreadL = clamp(spreadSc - widthSc, 0.f,1.f);
                spreadR = clamp(spreadSc + widthSc, 0.f,1.f);
            }
            if (widthTarget & W_GAP) {
                gapL = clamp(gapSc - widthSc, -2.f,2.f);
                gapR = clamp(gapSc + widthSc, -2.f,2.f);
            }
            if (widthTarget & W_SHARP) {
                sharpL = clamp(sharpSc - widthSc, 0.f,1.f);
                sharpR = clamp(sharpSc + widthSc, 0.f,1.f);
            }
            if (widthTarget & W_RES) {
                resL = clamp(resSc - widthSc*0.5f, 0.f,0.5f);
                resR = clamp(resSc + widthSc*0.5f, 0.f,0.5f);
            }
            if (widthTarget & W_DRIVE) {
                driveL = clamp(driveSc - widthSc, 0.f,1.f);
                driveR = clamp(driveSc + widthSc, 0.f,1.f);
            }

            //Feedback
            feedbackL = 0.5f*resL; //save to global var for use in audio feedback
            feedbackR = 0.5f*resR;

            // Levels
            auto readLvl = [&](ParamId p, ParamId t, InputId i) -> float {
                float v=params[p].getValue();
                if (inputs[i].isConnected()) v+=inputs[i].getVoltage()*0.1f*params[t].getValue();
                return clamp(v,0.f,1.f);
            };
            cachedLowLvl  = smLowLvl.process (readLvl(LOW_LEVEL_PARAM, LOW_LEVEL_TRIM_PARAM, LOW_LEVEL_CV_INPUT), 0.16f);
            cachedMidLvl  = smMidLvl.process (readLvl(MID_LEVEL_PARAM, MID_LEVEL_TRIM_PARAM, MID_LEVEL_CV_INPUT), 0.16f);
            cachedHighLvl = smHighLvl.process(readLvl(HIGH_LEVEL_PARAM,HIGH_LEVEL_TRIM_PARAM,HIGH_LEVEL_CV_INPUT),0.16f);
            cachedMixLvl = smMixLvl.process(readLvl(MIX_LEVEL_PARAM,MIX_LEVEL_TRIM_PARAM,MIX_LEVEL_CV_INPUT),0.16f);

            // Crossover frequencies — both L and R may differ when Width is mapped
            CutoffSet csL = computeCutoffs(centerL, spreadL, gapL, sampleRate);
            CutoffSet csR = computeCutoffs(centerR, spreadR, gapR, sampleRate);

            // Update filter coefficients
            lpLowL.setParameters (csL.lpLow,  sharpL, resL);
            hpLowL.setParameters (csL.hpLow,  sharpL, resL);
            lpHighL.setParameters(csL.lpHigh, sharpL, resL);
            hpHighL.setParameters(csL.hpHigh, sharpL, resL);
            lpLowR.setParameters (csR.lpLow,  sharpR, resR);
            hpLowR.setParameters (csR.hpLow,  sharpR, resR);
            lpHighR.setParameters(csR.lpHigh, sharpR, resR);
            hpHighR.setParameters(csR.hpHigh, sharpR, resR);

            // Envelope follower coefficients
            float fcLowCenter  = csL.lpLow  * 0.6f;
            float fcMidCenter  = sqrtf(csL.lpLow * csL.lpHigh) * 0.5f;
            float fcHighCenter = clamp(csL.hpHigh * 4.0f, 0.001f, 0.499f);

            envLow.setCoeff (fcLowCenter,  followTime, sampleRate);
            envMid.setCoeff (fcMidCenter,  followTime, sampleRate);
            envHigh.setCoeff(fcHighCenter, followTime, sampleRate);

            // Display coefficients — copy once per param update, not per sample
            for (int i=0;i<TRITON_STAGES;i++) {
                dispLpLow[i]  = lpLowL.coeffs[i];
                dispHpLow[i]  = hpLowL.coeffs[i];
                dispLpHigh[i] = lpHighL.coeffs[i];
                dispHpHigh[i] = hpHighL.coeffs[i];
            }
            dispSharp    = sharpL;
            displayFcLow = csL.fA;
            displayFcHigh= csL.fB;
            for (int i=0;i<TRITON_STAGES;i++) {
                dispLpLowR[i]  = lpLowR.coeffs[i];
                dispHpLowR[i]  = hpLowR.coeffs[i];
                dispLpHighR[i] = lpHighR.coeffs[i];
                dispHpHighR[i] = hpHighR.coeffs[i];
            }
            dispSharpR     = sharpR;
            displayFcLowR  = csR.fA;
            displayFcHighR = csR.fB;

            cachedDriveGainL = 1.f + driveL * 9.f;
            cachedDriveGainR = 1.f + driveR * 9.f;
        } // end paramDivider

        // ── AUDIO TIER — runs every sample ────────────────────────────────────
        float inL = inputs[AUDIO_L_INPUT].getVoltage();
        float inR = inputs[AUDIO_R_INPUT].isConnected()
                  ? inputs[AUDIO_R_INPUT].getVoltage() : inL;

        // ── Feedback AGC ──────────────────────────────────────────────────────
        const float fbAttack  = 0.9f;
        const float fbRelease = 0.0005f;
        const float fbTarget  = 5.0f;

        float mixLabs = fabsf(mixL);
        float mixRabs = fabsf(mixR);
        fbEnvL = mixLabs > fbEnvL ? mixLabs*fbAttack + fbEnvL*(1.f-fbAttack)
                                  : fbEnvL*(1.f-fbRelease);
        fbEnvR = mixRabs > fbEnvR ? mixRabs*fbAttack + fbEnvR*(1.f-fbAttack)
                                  : fbEnvR*(1.f-fbRelease);

        float agcL = (fbEnvL > fbTarget) ? fbTarget / fbEnvL : 1.f;
        float agcR = (fbEnvR > fbTarget) ? fbTarget / fbEnvR : 1.f;
        fbGainL += 0.01f * (feedbackL * agcL - fbGainL);
        fbGainR += 0.01f * (feedbackR * agcR - fbGainR);

        if (feedbackEnabled) {
            inL += mixL * fbGainL + 0.003f * feedbackL;
            inR += mixR * fbGainR + 0.003f * feedbackR;
        }        

        float drivenL = driveL.process(inL, cachedDriveGainL);
        float drivenR = driveR.process(inR, cachedDriveGainR);

        // ── Band splitting — L ────────────────────────────────────────────────
        float lowL  = dcBlockL.process(lpLowL.process(drivenL));
        float midL  = lpHighL.process(hpLowL.process(drivenL));
        float highL = nyqCapL.process(hpHighL.process(drivenL));
                        
        lowL  = clamp(lowL,  -12.f,12.f);
        midL  = clamp(midL,  -12.f,12.f);
        highL = clamp(highL, -12.f,12.f);

        // ── Band splitting — R ────────────────────────────────────────────────
        float lowR  = dcBlockR.process(lpLowR.process(drivenR));
        float midR  = lpHighR.process(hpLowR.process(drivenR));
        float highR = nyqCapR.process(hpHighR.process(drivenR));
        lowR  = clamp(lowR,  -12.f,12.f);
        midR  = clamp(midR,  -12.f,12.f);
        highR = clamp(highR, -12.f,12.f);

        // ── Envelope followers (L channel) ────────────────────────────────────
        
        float fcMin = 20.f / sampleRate;     // ~20Hz normalised
        float fcMax = 20000.f / sampleRate;  // ~20kHz normalised
        
        float fcL = clamp(displayFcLow  * 0.35f, fcMin, fcMax);
        float fcM = clamp(sqrtf(displayFcLow * displayFcHigh), fcMin, fcMax);
        float fcH = clamp(displayFcHigh * 2.5f, fcMin, fcMax);
        
        float logRange = log2f(fcMax / fcMin);  // total log range
        float scaleL = powf(2.f, 2.f * log2f(fcL / fcMin) / logRange);
        float scaleM = powf(2.f, 2.f * log2f(fcM / fcMin) / logRange);
        float scaleH = powf(2.f, 2.f * log2f(fcH / fcMin) / logRange);
               
        const float envScale = 8.0f;
        float rawL = envLow.process (lowL);
        float rawM = envMid.process (midL);
        float rawH = envHigh.process(highL);        
        
        // Cap to avoid extreme values at unusual Center/Spread settings
        scaleL = clamp(scaleL, 0.25f, 8.0f);
        scaleH = clamp(scaleH, 0.25f, 8.0f);
        
        float envL_ = clamp(rawL * envScale * scaleL, 0.f,10.f);
        float envM_ = clamp(rawM * envScale * scaleM, 0.f,10.f);
        float envH_ = clamp(rawH * envScale * scaleH, 0.f,10.f);
        
        float envMix = clamp((envL_+envM_+envH_)/3.f, 0.f,10.f);

        // ── Outputs ───────────────────────────────────────────────────────────
        float lowLvl=cachedLowLvl, midLvl=cachedMidLvl, highLvl=cachedHighLvl, mixLvl=cachedMixLvl;
        outputs[LOW_L_OUTPUT ].setVoltage(clamp(lowL  *lowLvl,  -10.f,10.f));
        outputs[LOW_R_OUTPUT ].setVoltage(clamp(lowR  *lowLvl,  -10.f,10.f));
        outputs[MID_L_OUTPUT ].setVoltage(clamp(midL  *midLvl,  -10.f,10.f));
        outputs[MID_R_OUTPUT ].setVoltage(clamp(midR  *midLvl,  -10.f,10.f));
        outputs[HIGH_L_OUTPUT].setVoltage(clamp(highL *highLvl, -10.f,10.f));
        outputs[HIGH_R_OUTPUT].setVoltage(clamp(highR *highLvl, -10.f,10.f));

        mixL = lowL*lowLvl+midL*midLvl+highL*highLvl;
        mixR = lowR*lowLvl+midR*midLvl+highR*highLvl;

        outputs[SUM_L_OUTPUT  ].setVoltage(clamp(mixL*mixLvl,-10.f,10.f));
        outputs[SUM_R_OUTPUT  ].setVoltage(clamp(mixR*mixLvl,-10.f,10.f));        

        outputs[LOW_ENV_OUTPUT ].setVoltage(envL_);
        outputs[MID_ENV_OUTPUT ].setVoltage(envM_);
        outputs[HIGH_ENV_OUTPUT].setVoltage(envH_);

        outputs[MIX_ENV_OUTPUT].setVoltage(envMix);

        // Display
        displayEnvLow  = envL_/10.f;
        displayEnvMid  = envM_/10.f;
        displayEnvHigh = envH_/10.f;
    }
};

// =============================================================================
// Widget
// =============================================================================
struct TritonWidget : ModuleWidget {

    struct TritonSliderBase : app::SvgSlider {
        TritonSliderBase() {
            setBackgroundSvg(Svg::load(asset::plugin(pluginInstance,"res/components/ShortSlider.svg")));
            setHandleSvg    (Svg::load(asset::plugin(pluginInstance,"res/components/ShortSliderHandle.svg")));
            setHandlePosCentered(math::Vec(10.f,55.f),math::Vec(10.f,10.f));
        }
    };
    template <typename TL=YellowLight>
    struct TritonSlider : LightSlider<TritonSliderBase,VCVSliderLight<TL>> { TritonSlider(){} };

    // ── Filter display ────────────────────────────────────────────────────────
    struct FilterDisplay : TransparentWidget {
        Triton* module=nullptr;
        void drawLayer(const DrawArgs& args, int layer) override {
            if (layer!=1){TransparentWidget::drawLayer(args,layer);return;}
            const float w=box.size.x,h=box.size.y,pad=3.f;
            const int N=512;
            float fcLow  =module?module->displayFcLow :0.05f;
            float fcHigh =module?module->displayFcHigh:0.25f;
            float envL   =module?module->displayEnvLow :0.f;
            float envM   =module?module->displayEnvMid :0.f;
            float envH   =module?module->displayEnvHigh:0.f;
            float sharp  =module?module->dispSharp      :1.f;
            float sr     =module?module->sampleRate     :44100.f;

            float fnLo=clamp(20.f/sr,0.0001f,0.49f);
            float fnHi=clamp(20000.f/sr,fnLo+0.001f,0.499f);
            auto fnToX=[&](float fn)->float{
                fn=clamp(fn,fnLo,fnHi);
                return pad+logf(fn/fnLo)/logf(fnHi/fnLo)*(w-2.f*pad);
            };
            const float dBRange=10.f;
            const float midY=(pad+h-pad)*0.5f;
            auto magToY=[&](float mag)->float{
                float dB=20.f*log10f(mag<1e-6f?1e-6f:mag);
                float norm=clamp(dB/dBRange,-1.f,1.f);
                return midY-norm*(h-2.f*pad)*0.5f; 
            };
            auto evalBand=[&](std::function<float(float)> magFn){
                std::vector<std::pair<float,float>> pts(N+1);
                for(int k=0;k<=N;k++){
                    float t=(float)k/N;
                    float fn=fnLo*powf(fnHi/fnLo,t);
                    pts[k]={fnToX(fn),magToY(magFn(fn))};
                }
                return pts;
            };
            auto drawBand=[&](const std::vector<std::pair<float,float>>& pts,
                              NVGcolor dimCol,NVGcolor brightCol,float envFill){
                const float baseY=h-pad;

            nvgBeginPath(args.vg);
            bool started = false;
            for(int k=0;k<=N;k++){
 
                float px = pts[k].first;
                float py = pts[k].second;
                bool visible = (py < baseY - 1.f);  // above the threshold

                if(!started){
                    if(k == 0){
                        nvgMoveTo(args.vg, px, py);  // start at curve height, no drop
                    } else {
                        nvgMoveTo(args.vg, px, baseY);
                        nvgLineTo(args.vg, px, py);
                    }
                    started = true;
                }
            
                if(visible){
                    if(!started){
                        // First visible point — drop a vertical line from baseline first
                        nvgMoveTo(args.vg, px, baseY);
                        nvgLineTo(args.vg, px, py);
                        started = true;
                    } else {
                        nvgLineTo(args.vg, px, py);
                    }
                } else if(started){
                    // Just went invisible — close down to baseline at this x
                    nvgLineTo(args.vg, px, baseY);
                    started = false;
                }
            }

                nvgStrokeColor(args.vg,dimCol);
                nvgStrokeWidth(args.vg,1.2f);
                nvgStroke(args.vg);
                float fill=clamp(envFill,0.f,1.f);
                if(fill>0.005f){
                    nvgBeginPath(args.vg);
                    nvgMoveTo(args.vg,pts[0].first,baseY);
                    for(int k=0;k<=N;k++){
                        float py=baseY+(pts[k].second-baseY)*fill;
                        nvgLineTo(args.vg,pts[k].first,py);
                    }
                    nvgLineTo(args.vg,pts[N].first,baseY);
                    nvgClosePath(args.vg);
                    nvgFillColor(args.vg,brightCol);
                    nvgFill(args.vg);
                }
            };
            // 0dB line
            nvgBeginPath(args.vg);
            nvgMoveTo(args.vg,pad,midY);nvgLineTo(args.vg,w-pad,midY);
            nvgStrokeColor(args.vg,nvgRGBAf(1.f,1.f,1.f,0.25f));
            nvgStrokeWidth(args.vg,0.5f);nvgStroke(args.vg);
            if(module){
                float sharpR = module->dispSharpR;
                // L channel — full brightness
                auto lowPts=evalBand([&](float fn){return cascadeMagSharp(module->dispLpLow, fn,sharp);});
                auto highPts=evalBand([&](float fn){return cascadeMagSharp(module->dispHpHigh,fn,sharp);});
                auto midPts=evalBand([&](float fn){
                    return cascadeMagSharp(module->dispHpLow, fn,sharp)
                          *cascadeMagSharp(module->dispLpHigh,fn,sharp);
                });
                drawBand(lowPts, nvgRGBAf(0.75f,0.42f,0.08f,0.50f),nvgRGBAf(1.00f,0.58f,0.05f,0.85f),envL);
                drawBand(midPts, nvgRGBAf(0.15f,0.40f,0.88f,0.50f),nvgRGBAf(0.22f,0.54f,1.00f,0.85f),envM);
                drawBand(highPts,nvgRGBAf(0.08f,0.78f,0.72f,0.50f),nvgRGBAf(0.10f,1.00f,0.88f,0.85f),envH);
                // R channel — dimmer dashed-style overlay (drawn without fill, outline only)
                auto lowPtsR=evalBand([&](float fn){return cascadeMagSharp(module->dispLpLowR, fn,sharpR);});
                auto highPtsR=evalBand([&](float fn){return cascadeMagSharp(module->dispHpHighR,fn,sharpR);});
                auto midPtsR=evalBand([&](float fn){
                    return cascadeMagSharp(module->dispHpLowR, fn,sharpR)
                          *cascadeMagSharp(module->dispLpHighR,fn,sharpR);
                });
                drawBand(lowPtsR, nvgRGBAf(0.75f,0.42f,0.08f,0.25f),nvgRGBAf(1.00f,0.58f,0.05f,0.30f),0.f);
                drawBand(midPtsR, nvgRGBAf(0.15f,0.40f,0.88f,0.25f),nvgRGBAf(0.22f,0.54f,1.00f,0.30f),0.f);
                drawBand(highPtsR,nvgRGBAf(0.08f,0.78f,0.72f,0.25f),nvgRGBAf(0.10f,1.00f,0.88f,0.30f),0.f);
            }
            // Crossover markers
            nvgBeginPath(args.vg);
            nvgMoveTo(args.vg,fnToX(fcLow), pad);nvgLineTo(args.vg,fnToX(fcLow), h-pad);
            nvgMoveTo(args.vg,fnToX(fcHigh),pad);nvgLineTo(args.vg,fnToX(fcHigh),h-pad);
            nvgStrokeColor(args.vg,nvgRGBAf(1.f,1.f,1.f,0.25f));
            nvgStrokeWidth(args.vg,0.5f);nvgStroke(args.vg);
            TransparentWidget::drawLayer(args,layer);
        }
    };

    // ── Context menu — Follow slider ──────────────────────────────────────────
    struct FollowQuantity : Quantity {
        Triton* module;
        FollowQuantity(Triton* m):module(m){}
        void setValue(float v) override { module->followTime=clamp(v,0.f,1.f); }
        float getValue() override { return module->followTime; }
        float getDefaultValue() override { return 0.12f; }
        float getMinValue() override { return 0.f; }
        float getMaxValue() override { return 1.f; }
        int getDisplayPrecision() override { return 2; }
        std::string getLabel() override { return "Follow Time"; }
        std::string getDisplayValueString() override {
            return string::f("%.2f", getValue());
        }
    };


    TritonWidget(Triton* module){
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance,"res/Triton.svg"),
            asset::plugin(pluginInstance,"res/Triton-dark.svg")));

        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH,0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x-2*RACK_GRID_WIDTH,0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH,RACK_GRID_HEIGHT-RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x-2*RACK_GRID_WIDTH,RACK_GRID_HEIGHT-RACK_GRID_WIDTH)));

        const float panelW = 20.f*5.08f;

        // Display
        auto* disp=createWidget<FilterDisplay>(mm2px(Vec(6.f,10.f)));
        disp->box.size=mm2px(Vec(panelW-12.f,14.5));
        disp->module=module; addChild(disp);

        // Slider row: 7 filter sliders + Width (extra gap)
        const float sxBase=10.f, sPitch=11.5f, btnOff=0.f;
        const float sx0=sxBase+1*sPitch, sx1=sxBase+2*sPitch, sx2=sxBase+3*sPitch;
        const float sx3=sxBase+4*sPitch, sx4=sxBase+5*sPitch, sx5=sxBase+6*sPitch;
        const float sx6=sxBase+7*sPitch;
        const float yLed=30.f, ySlider=45.5f, ySlTrim=57.f, ySlCV=65.f;

        // Left input column
        const float xIn=7.f;
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(sxBase,ySlCV-33)),module,Triton::AUDIO_L_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(sxBase,ySlCV-18)),module,Triton::AUDIO_R_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(sxBase,ySlCV)),module,Triton::VOCT_INPUT));


        // Center
        addParam(createParamCentered<TL1105>(mm2px(Vec(sx0+btnOff,yLed)),module,Triton::MAP_CENTER_PARAM));
        addChild(createLightCentered<MediumLight<BlueLight>>(mm2px(Vec(sx0+btnOff,yLed)),module,Triton::LED_CENTER));
        addParam(createParamCentered<TritonSlider<BlueLight>>(mm2px(Vec(sx0,ySlider)),module,Triton::CENTER_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(sx0,ySlTrim)),module,Triton::CENTER_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(sx0,ySlCV)),module,Triton::CENTER_CV_INPUT));

        // Spread
        addParam(createParamCentered<TL1105>(mm2px(Vec(sx1+btnOff,yLed)),module,Triton::MAP_SPREAD_PARAM));
        addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(sx1+btnOff,yLed)),module,Triton::LED_SPREAD));
        addParam(createParamCentered<TritonSlider<YellowLight>>(mm2px(Vec(sx1,ySlider)),module,Triton::SPREAD_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(sx1,ySlTrim)),module,Triton::SPREAD_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(sx1,ySlCV)),module,Triton::SPREAD_CV_INPUT));

        // Gap
        addParam(createParamCentered<TL1105>(mm2px(Vec(sx2+btnOff,yLed)),module,Triton::MAP_GAP_PARAM));
        addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(sx2+btnOff,yLed)),module,Triton::LED_GAP));
        addParam(createParamCentered<TritonSlider<GreenLight>>(mm2px(Vec(sx2,ySlider)),module,Triton::GAP_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(sx2,ySlTrim)),module,Triton::GAP_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(sx2,ySlCV)),module,Triton::GAP_CV_INPUT));

        // Sharpness
        addParam(createParamCentered<TL1105>(mm2px(Vec(sx3+btnOff,yLed)),module,Triton::MAP_SHARP_PARAM));
        addChild(createLightCentered<MediumLight<WhiteLight>>(mm2px(Vec(sx3+btnOff,yLed)),module,Triton::LED_SHARP));
        addParam(createParamCentered<TritonSlider<WhiteLight>>(mm2px(Vec(sx3,ySlider)),module,Triton::SHARP_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(sx3,ySlTrim)),module,Triton::SHARP_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(sx3,ySlCV)),module,Triton::SHARP_CV_INPUT));

        // Resonance
        addParam(createParamCentered<TL1105>(mm2px(Vec(sx4+btnOff,yLed)),module,Triton::MAP_RES_PARAM));
        addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(sx4+btnOff,yLed)),module,Triton::LED_RES));
        addParam(createParamCentered<TritonSlider<RedLight>>(mm2px(Vec(sx4,ySlider)),module,Triton::RES_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(sx4,ySlTrim)),module,Triton::RES_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(sx4,ySlCV)),module,Triton::RES_CV_INPUT));

        // Drive
        addParam(createParamCentered<TL1105>(mm2px(Vec(sx5+btnOff,yLed)),module,Triton::MAP_DRIVE_PARAM));
        addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(sx5+btnOff,yLed)),module,Triton::LED_DRIVE));
        addParam(createParamCentered<TritonSlider<YellowLight>>(mm2px(Vec(sx5,ySlider)),module,Triton::DRIVE_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(sx5,ySlTrim)),module,Triton::DRIVE_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(sx5,ySlCV)),module,Triton::DRIVE_CV_INPUT));

        // Width
        addParam(createParamCentered<TritonSlider<WhiteLight>>(mm2px(Vec(sx6,ySlider)),module,Triton::WIDTH_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(sx6,ySlTrim)),module,Triton::WIDTH_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(sx6,ySlCV)),module,Triton::WIDTH_CV_INPUT));


        // Output rows: CV | trim | knob | LED | ENV | L | R
        const float ox0=sxBase + 0.5*sPitch,ox1=sxBase+1.5*sPitch,ox2=sxBase + 2.5*sPitch,ox3=sx6-35-0.5*sPitch,ox4=sx6-25-0.5*sPitch,ox5=sx6-10-0.5*sPitch,ox6=sx6-0.5*sPitch;
        const float yRow0=80.f, yRow1=91.f, yRow2=102.f, yRow3=113.f;

        // Low
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(ox0,yRow0)),module,Triton::LOW_LEVEL_CV_INPUT));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(ox1,yRow0)),module,Triton::LOW_LEVEL_TRIM_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(ox2,yRow0)),module,Triton::LOW_LEVEL_PARAM));
        addChild(createLightCentered<LargeLight<YellowLight>>(mm2px(Vec(ox3,yRow0)),module,Triton::LED_ENV_LOW));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(ox4,yRow0)),module,Triton::LOW_ENV_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(ox5,yRow0)),module,Triton::LOW_L_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(ox6,yRow0)),module,Triton::LOW_R_OUTPUT));

        // Mid
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(ox0,yRow1)),module,Triton::MID_LEVEL_CV_INPUT));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(ox1,yRow1)),module,Triton::MID_LEVEL_TRIM_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(ox2,yRow1)),module,Triton::MID_LEVEL_PARAM));
        addChild(createLightCentered<LargeLight<BlueLight>>(mm2px(Vec(ox3,yRow1)),module,Triton::LED_ENV_MID));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(ox4,yRow1)),module,Triton::MID_ENV_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(ox5,yRow1)),module,Triton::MID_L_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(ox6,yRow1)),module,Triton::MID_R_OUTPUT));

        // High
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(ox0,yRow2)),module,Triton::HIGH_LEVEL_CV_INPUT));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(ox1,yRow2)),module,Triton::HIGH_LEVEL_TRIM_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(ox2,yRow2)),module,Triton::HIGH_LEVEL_PARAM));
        addChild(createLightCentered<LargeLight<GreenLight>>(mm2px(Vec(ox3,yRow2)),module,Triton::LED_ENV_HIGH));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(ox4,yRow2)),module,Triton::HIGH_ENV_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(ox5,yRow2)),module,Triton::HIGH_L_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(ox6,yRow2)),module,Triton::HIGH_R_OUTPUT));

        // Mix
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(ox0,yRow3)),module,Triton::MIX_LEVEL_CV_INPUT));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(ox1,yRow3)),module,Triton::MIX_LEVEL_TRIM_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(ox2,yRow3)),module,Triton::MIX_LEVEL_PARAM));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(ox3,yRow3)),module,Triton::LED_ENV_MIX));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(ox4,yRow3)),module,Triton::MIX_ENV_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(ox5,yRow3)),module,Triton::SUM_L_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(ox6,yRow3)),module,Triton::SUM_R_OUTPUT));
    }

    // ── Context menu ──────────────────────────────────────────────────────────
    void step() override {
        Triton* m = dynamic_cast<Triton*>(module);
        if (m) {
            m->lights[Triton::LED_CENTER].setBrightness((m->widthTarget & Triton::W_CENTER)?1.f:0.f);
            m->lights[Triton::LED_SPREAD].setBrightness((m->widthTarget & Triton::W_SPREAD)?1.f:0.f);
            m->lights[Triton::LED_GAP   ].setBrightness((m->widthTarget & Triton::W_GAP   )?1.f:0.f);
            m->lights[Triton::LED_SHARP ].setBrightness((m->widthTarget & Triton::W_SHARP )?1.f:0.f);
            m->lights[Triton::LED_RES   ].setBrightness((m->widthTarget & Triton::W_RES   )?1.f:0.f);
            m->lights[Triton::LED_DRIVE ].setBrightness((m->widthTarget & Triton::W_DRIVE )?1.f:0.f);

            m->lights[Triton::LED_ENV_LOW].setBrightness(m->displayEnvLow);
            m->lights[Triton::LED_ENV_MID].setBrightness(m->displayEnvMid);
            m->lights[Triton::LED_ENV_HIGH].setBrightness(m->displayEnvHigh);
            m->lights[Triton::LED_ENV_MIX].setBrightness((m->displayEnvLow+m->displayEnvMid+m->displayEnvHigh)/3.f);

        }
        ModuleWidget::step();
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
        Triton* m=dynamic_cast<Triton*>(module);
        if(!m) return;

        menu->addChild(new MenuSeparator());

        auto* followSlider=new ui::Slider();
        followSlider->quantity=new FollowQuantity(m);
        followSlider->box.size.x=200.f;
        menu->addChild(followSlider);
        
        menu->addChild(new MenuSeparator());
        auto* fbItem = createMenuItem("Internal Feedback", CHECKMARK(m->feedbackEnabled),
            [m]() { m->feedbackEnabled = !m->feedbackEnabled; });
        menu->addChild(fbItem);
    }
};

Model* modelTriton = createModel<Triton, TritonWidget>("Triton");