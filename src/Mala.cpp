////////////////////////////////////////////////////////////
//
//   Mala
//
//   written by Cody Geary
//   Copyright 2026, MIT License
//
//   Complex oscillator on the Clpy saturation transfer.
//   Osc A is a shaped sine, Osc B an 8 partial additive.
//   HEAT grafts B into A's crests; zero crossings stay clean.
//   MALA is one LFO routed by the map buttons, antiphase L/R.
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
using namespace rack;

#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstdint>
#include <atomic>


static const int   kMaxPoly     = 16;
static const int   kVoicePairs  = kMaxPoly / 2;
static const int   kMaxOs       = 8;
static const int   kCaptureN    = 256;
static const int   kPartials    = 8;
static const int   kLutSize     = 8192;
static const int   kLutLevels   = 7;

// B cycles per pass through the noise table.  Harmonics start at kNoiseLow so
// the gcd stays 1 and the texture repeats only once per table.
static const int   kNoiseRepeat = 32;
static const int   kNoiseLow    = 24;

static const float kPi          = 3.14159265358979f;
static const float kTwoPi       = 6.28318530717959f;

// Clpy's shaper threshold.  Below this the crossfade weight is identically
// zero, which is the dead zone every reset in this module is aligned to.
static const float kThresh      = 0.926605548037825f;
// The window ramp's span past the threshold, as a reciprocal for the audio path.
static const float kWindowScale = 1.f / (kPi - kThresh);

// Decimation cutoff, fraction of the base rate.  Below Nyquist for more
// rejection where fold-back lands.  Lower is cleaner but darker.
static const float kDecimCutoff = 0.45f;

// Nyquist limit for partial generation, as a fraction of the base rate.
static const float kPartialNyq  = 0.45f;

// B brightness knee in Hz: two pole rolloff in absolute frequency on the
// partial weights.  The normalizer is taken before it, so it really attenuates.
// Context menu slider range and default.
static const float kBrightMin   = 800.f;
static const float kBrightMax   = 20000.f;
static const float kBrightDef   = 3500.f;

// Final output trim, ahead of the VOLUME knob.  Hand-tunable.
static const float kOutGain     = 1.0f;

// Peak estimator blend for the additive sum.  sqrt(sum of squares) is right
// for sparse spectra, sum of magnitudes for dense ones.  Hand-tunable.
static const float kPeakBlendRms = 0.60f;

// SHAPE endpoints, both linear in the knob.
//   kShapeThin: rational thinning, must stay above -1.
//   kSatMax: gain into the ADAA shaper at full SHAPE (4 -> rounded square).
static const float kShapeThin   = -0.95f;
static const float kSatMax      =  4.00f;

// Total detune spread at full travel, in octaves (0.025 = 30 cents).
// kDetuneWide multiplies it from the context menu.
static const float kDetuneOct   = 0.025f;
static const float kDetuneWide  = 8.00f;

// Linear detune slew, amount units per second (3 -> full scale in 1/3 s).
static const float kDetuneRate  = 3.0f;

// Top of HEAT's drive range; the bottom is pinned to kThresh.
static const float kDriveMax    = 5.00f;

// Re-unison glide: a constant bend, as a fraction of the right phasor's
// increment, until the gap closes exactly.  0.006 is about 10 cents.
static const double kRealignBend = 0.006;

// B phase offset range per side, in B cycles.  Modulation carries on past
// either end of the slider instead of wrapping, so SPREAD's inharmonic
// partials never jump.
static const float kPhaseRange  = 1.00f;

// Full-scale modulation ranges, before the per-destination trim.
static const float kModShape    = 1.00f;   // shape units
static const float kModRatio    = 2.00f;   // log2, two octaves
static const float kModMorph    = 7.50f;   // morph stops, half the bank
static const float kModSpread   = 1.00f;   // spread units
static const float kModDetune   = 1.00f;   // detune amount
static const float kModPhase    = 1.00f;   // phase amount
static const float kModHeat     = 5.00f;   // Heat units

// Map button gate ramp time, seconds.
static const float kGateTau     = 0.020f;

// Partial weight rebuild period, in samples.  Weights, normalizer, noise mix and
// noise mip level are rebuilt this often and ramped linearly in between, so
// their cost does not depend on how much the controls or the pitch move (through-
// zero FM is the exception: it rebuilds every sample).  Lower follows audio-rate
// MORPH/SPREAD more closely at more CPU.
static const int   kWeightDiv   = 16;

// Front panel buttons are polled every this many samples (16 is 0.33 ms at 48 kHz).
static const int   kButtonDiv   = 16;

// =============================================================================
// ADAA tanh saturator (GlassADAADrive / FilterAulos curve).  The clamp to
// |x| <= 1 is the saturation.
// =============================================================================
struct MalaADAADrive {
    float lastInput = 0.f;

    static float polyTanh(float x) {
        float x2 = x * x;
        return x - x * x2 * (1.f/3.f - x2 * (2.f/15.f - 17.f/315.f * x2));
    }
    // Static transfer, used by the normalizer and by the display.
    static float curve(float x) { return polyTanh(clamp(x, -1.f, 1.f)); }

    // lastInput is the undriven signal, so both points of the difference
    // quotient see the same drive and the state stays valid while bypassed.
    // ADAA averages the linear part over half a sample; linearRestore adds
    // that back (1 = none of the averaging, matching the unshaped signal).
    float process(float in, float drive, float linearRestore) {
        // Clamp before the difference quotient so both the current and the
        // previous value sit where the series and its antiderivative agree.
        float x  = clamp(drive * in, -1.f, 1.f);
        float x0 = clamp(drive * lastInput, -1.f, 1.f);
        lastInput = in;
        // (F(x) - F(x0)) / (x - x0) for F = antiderivative of polyTanh,
        // F = x^2/2 - x^4/12 + x^6/45 - 17x^8/2520, with the division done
        // term by term: no cancellation, no small-step fallback.
        float sum  = x + x0;
        float prod = x * x0;
        float sq   = x * x + x0 * x0;
        float out  = sum * (0.5f - sq * (1.f/12.f)
                          + (sq * sq - prod * prod) * (1.f/45.f)
                          - sq * (sq * sq - 2.f * prod * prod) * (17.f/2520.f));
        return out + linearRestore * 0.5f * (x - x0);
    }
    void reset() { lastInput = 0.f; }
};


// Saturator normaliser once the drive reaches the clamp.
static const float kSatNormFull = 1.f / MalaADAADrive::curve(1.f);


// =============================================================================
// Sine and cosine polynomials, for angles folded into [-pi, pi].
//
// Sine: x (pi^2 - x^2) (c0 + c1 x^2 + c2 x^4 + c3 x^6), degree 9.  The factor
// puts exact zeros at +-pi, so the wave is continuous where the fold wraps;
// the coefficients are a minimax fit with the curvature also continuous
// there.  Max error 1.5e-5; every harmonic sits below -99 dB, and H10 up below
// -129 dB, so nothing audible is left to alias.  (A plain Taylor series of the
// same degree leaves a 0.014 step at the wrap and 1/n harmonics.)
//
// Cosine: c0 + c1 x^2 + ... + c4 x^8, minimax with zero slope at +-pi so it
// too is smooth across the wrap.  Max error 6.5e-5.  It only pairs with the
// sine in the B PHASE rotation.
// =============================================================================
static const float kPiSquared = kPi * kPi;
static const float kSinC0 =  1.013162234e-01f;
static const float kSinC1 = -6.612881286e-03f;
static const float kSinC2 =  1.702231847e-04f;
static const float kSinC3 = -2.044066883e-06f;
static const float kCosC0 =  9.999346761e-01f;
static const float kCosC1 = -4.996958253e-01f;
static const float kCosC2 =  4.143966119e-02f;
static const float kCosC3 = -1.329117405e-03f;
static const float kCosC4 =  1.823248472e-05f;

// =============================================================================
// Scalar helpers.
// =============================================================================

// Sine for x in [0, 2*pi].
static inline float malaSin2pi(float x) {
    if (x > kPi) x -= kTwoPi;
    float x2 = x * x;
    return x * (kPiSquared - x2) * (kSinC0 + x2 * (kSinC1 + x2 * (kSinC2 + x2 * kSinC3)));
}

// x - floor(x).  32-bit ARM has no rounding instruction, so floorf is a
// library call there; truncating through int32 and stepping down for negative
// non-integers is exact for |x| < 2^31.
static inline float malaWrap01(float x) {
#if defined(__arm__)
    float whole = (float)(int32_t)x;
    return x - ((x < whole) ? whole - 1.f : whole);
#else
    return x - floorf(x);
#endif
}

// floor() for the B phasors, on the same terms: simd::floor() is four floorf
// calls on 32-bit ARM.  Phases and increments here stay far below 2^31.
static inline simd::float_4 malaFloor4(simd::float_4 x) {
#if defined(__arm__)
    simd::float_4 whole = simd::float_4(simd::int32_4(x));
    return whole - simd::ifelse(x < whole, simd::float_4(1.f), simd::float_4(0.f));
#else
    return simd::floor(x);
#endif
}

// =============================================================================
// Graft window, C4 smoothstep: w = t^5 (126 - 420t + 540t^2 - 315t^3 + 70t^4)
// Softer: C2 t*t*t*(t*(t*6-15)+10), C3 t^4*(35 - 84t + 70t^2 - 20t^3).
// =============================================================================
static inline float malaWindow(float t) {
    float t2 = t * t;
    float t4 = t2 * t2;
    return t4 * t * (126.f + t * (-420.f + t * (540.f + t * (-315.f + 70.f * t))));
}


// =============================================================================
// SIMD helpers.  Phases are in [0,1), so angles are in [0, 2*pi].
// =============================================================================
static inline simd::float_4 malaSin2pi4(simd::float_4 x) {
    x = simd::ifelse(x > simd::float_4(kPi), x - simd::float_4(kTwoPi), x);
    simd::float_4 x2 = x * x;
    return x * (simd::float_4(kPiSquared) - x2)
             * (simd::float_4(kSinC0) + x2 * (simd::float_4(kSinC1)
             + x2 * (simd::float_4(kSinC2) + x2 * simd::float_4(kSinC3))));
}

static inline simd::float_4 malaCos2pi4(simd::float_4 x) {
    x = simd::ifelse(x > simd::float_4(kPi), x - simd::float_4(kTwoPi), x);
    simd::float_4 x2 = x * x;
    return simd::float_4(kCosC0) + x2 * (simd::float_4(kCosC1)
             + x2 * (simd::float_4(kCosC2) + x2 * (simd::float_4(kCosC3)
             + x2 * simd::float_4(kCosC4))));
}




// =============================================================================
// Morph path: 20 stops, interpolated.  Columns 0..7 are partial weights (may be
// negative), column 8 is the noise mix.  Rows are hand-tunable.
// =============================================================================
static const int kMorphStops = 20;
static const float kMorphTable[kMorphStops][kPartials + 1] = {
    //  w1      w2      w3      w4      w5      w6      w7      w8    noise
    { 1.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.00f }, //  0 Sine
    { 1.000f, 0.600f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.00f }, //  1 Octave
    { 1.000f, 0.000f,-0.111f, 0.000f, 0.040f, 0.000f,-0.020f, 0.000f, 0.00f }, //  2 Triangle
    { 1.000f, 0.000f, 0.500f, 0.000f, 0.250f, 0.000f, 0.120f, 0.000f, 0.00f }, //  3 Hollow
    { 1.000f, 0.000f, 0.333f, 0.000f, 0.200f, 0.000f, 0.143f, 0.000f, 0.00f }, //  4 Square
    { 1.000f, 0.300f, 0.600f, 0.200f, 0.350f, 0.150f, 0.200f, 0.100f, 0.00f }, //  5 Reed
    { 1.000f, 0.500f, 0.333f, 0.250f, 0.200f, 0.167f, 0.143f, 0.125f, 0.00f }, //  6 Saw
    { 1.000f, 0.707f, 0.333f, 0.000f, 0.200f, 0.236f, 0.143f, 0.000f, 0.00f }, //  7 Pulse 1/4
    { 1.000f, 0.924f, 0.805f, 0.653f, 0.483f, 0.308f, 0.143f, 0.000f, 0.00f }, //  8 Pulse 1/8
    { 0.350f, 0.450f, 0.550f, 0.700f, 0.850f, 1.000f, 0.900f, 0.750f, 0.00f }, //  9 Bright
    { 0.250f, 0.300f, 1.000f, 0.850f, 0.350f, 0.200f, 0.150f, 0.100f, 0.00f }, // 10 Nasal
    { 1.000f, 0.550f, 0.120f, 0.050f, 0.030f, 0.020f, 0.020f, 0.010f, 0.00f }, // 11 Vowel oo
    { 1.000f, 0.700f, 0.350f, 0.550f, 0.300f, 0.120f, 0.080f, 0.050f, 0.00f }, // 12 Vowel ah
    { 1.000f, 0.250f, 0.100f, 0.150f, 0.450f, 0.800f, 0.550f, 0.200f, 0.00f }, // 13 Vowel ee
    { 1.000f, 0.150f, 0.550f, 0.200f, 0.700f, 0.180f, 0.450f, 0.150f, 0.00f }, // 14 Bell
    { 0.600f,-0.400f, 0.800f,-0.300f, 0.550f,-0.250f, 0.400f,-0.200f, 0.00f }, // 15 Glass
    { 0.900f,-0.900f, 0.800f,-0.800f, 0.700f,-0.700f, 0.600f,-0.600f, 0.04f }, // 16 Comb
    { 0.550f, 0.700f, 0.600f, 0.850f, 0.700f, 0.950f, 0.800f, 1.000f, 0.08f }, // 17 Clang
    { 0.450f, 0.500f, 0.550f, 0.600f, 0.650f, 0.700f, 0.750f, 0.800f, 0.45f }, // 18 Haze
    { 0.250f, 0.250f, 0.250f, 0.250f, 0.250f, 0.250f, 0.250f, 0.250f, 1.00f }, // 19 Noise
};

// Inharmonic stretch offsets, about n^1.35 - n.  Partial 1 never moves.
// Scale the table down (~0.6) to tame the top of SPREAD.
static const float kSpreadOffset[kPartials] =
    { 0.f, 0.549f, 1.407f, 2.498f, 3.782f, 5.232f, 6.830f, 8.560f };

// Harmonic counts for each noise LUT mip level, brightest first.
static const int kLutHarm[kLutLevels] = { 2048, 1024, 512, 256, 128, 64, 32 };

// Shared sine table (C++11 magic static).
struct MalaSinTab {
    float t[kLutSize];
    MalaSinTab() {
        for (int i = 0; i < kLutSize; i++)
            t[i] = sinf(kTwoPi * (float)i / (float)kLutSize);
    }
};
static const MalaSinTab& malaSinTab() { static MalaSinTab tab; return tab; }

// Selectable texture seeds.  A seed changes only the phases: the waveform,
// not the spectrum or level.
static const int kNoiseSeeds = 32;

// Ratio quantize targets as log2, sorted ascending (the search stops once the
// error grows).  Non-integers place formants between harmonics under SYNC.
static const int kQuantCount = 32;
static const float kQuantLog2[kQuantCount] = {
    -3.000000f,  // 1/8
    -2.584963f,  // 1/6
    -2.321928f,  // 1/5
    -2.000000f,  // 1/4
    -1.584963f,  // 1/3
    -1.321928f,  // 2/5
    -1.000000f,  // 1/2
    -0.584963f,  // 2/3
    -0.415037f,  // 3/4
     0.000000f,  // 1
     0.321928f,  // 5/4
     0.415037f,  // 4/3
     0.584963f,  // 3/2
     0.736966f,  // 5/3
     0.807355f,  // 7/4
     1.000000f,  // 2
     1.321928f,  // 5/2
     1.415037f,  // 8/3
     1.584963f,  // 3
     1.807355f,  // 7/2
     2.000000f,  // 4
     2.169925f,  // 9/2
     2.321928f,  // 5
     2.459432f,  // 11/2
     2.584963f,  // 6
     2.807355f,  // 7
     3.000000f,  // 8
     3.169925f,  // 9
     3.321928f,  // 10
     3.584963f,  // 12
     3.807355f,  // 14
     4.000000f   // 16
};


// =============================================================================
// Decimator: Filter6pButter's cascade on float_4, two voices' L and R in the
// four lanes.  Keep the Q schedule in step with Filter6pButter.h.
// Coefficients are dsp::TBiquadFilter's LOWPASS; the structure is transposed
// direct form II, and since a lowpass numerator is b0 * (1, 2, 1) each stage
// needs one multiply on its input and no state shuffling.
// =============================================================================
struct MalaDecim2 {
    simd::float_4 gain[3], feedback1[3], feedback2[3];   // b0, a1, a2 per stage
    simd::float_4 state1[3], state2[3];

    MalaDecim2() { reset(); }

    void setCutoffFreq(float normalizedCutoff) {
        normalizedCutoff = clamp(normalizedCutoff, 1e-5f, 0.4999f);
        float t  = normalizedCutoff / 0.49f;
        float q[3] = { 0.51763809f,
                       0.70710678f + (1.f - t) * 0.30f,
                       1.9318517f  - (1.f - t) * 0.80f };
        float K = std::tan(M_PI * normalizedCutoff);
        for (int s = 0; s < 3; s++) {
            float norm   = 1.f / (1.f + K / q[s] + K * K);
            gain[s]      = simd::float_4(K * K * norm);
            feedback1[s] = simd::float_4(2.f * (K * K - 1.f) * norm);
            feedback2[s] = simd::float_4((1.f - K / q[s] + K * K) * norm);
        }
    }

    // Filters count inputs spaced stride floats apart and returns the last
    // output.  The state stays in registers for the whole block.
    simd::float_4 processBlock(const float* in, int stride, int count) {
        simd::float_4 stage0Z1 = state1[0], stage0Z2 = state2[0];
        simd::float_4 stage1Z1 = state1[1], stage1Z2 = state2[1];
        simd::float_4 stage2Z1 = state1[2], stage2Z2 = state2[2];
        simd::float_4 out(0.f);
        for (int i = 0; i < count; i++) {
            simd::float_4 x = simd::float_4::load(in + i * stride);
            simd::float_4 scaledIn = gain[0] * x;
            out      = scaledIn + stage0Z1;
            stage0Z1 = scaledIn + scaledIn - feedback1[0] * out + stage0Z2;
            stage0Z2 = scaledIn - feedback2[0] * out;
            scaledIn = gain[1] * out;
            out      = scaledIn + stage1Z1;
            stage1Z1 = scaledIn + scaledIn - feedback1[1] * out + stage1Z2;
            stage1Z2 = scaledIn - feedback2[1] * out;
            scaledIn = gain[2] * out;
            out      = scaledIn + stage2Z1;
            stage2Z1 = scaledIn + scaledIn - feedback1[2] * out + stage2Z2;
            stage2Z2 = scaledIn - feedback2[2] * out;
        }
        state1[0] = stage0Z1; state2[0] = stage0Z2;
        state1[1] = stage1Z1; state2[1] = stage1Z2;
        state1[2] = stage2Z1; state2[2] = stage2Z2;
        return out;
    }

    void reset() {
        for (int s = 0; s < 3; s++) state1[s] = state2[s] = simd::float_4(0.f);
    }
};


// =============================================================================
// Per side SHAPE setup, once per voice.
//   shape < 0 : s * (1 + k) / (1 + k * |s|), k < 0  -- thins the crest
//   shape > 0 : ADAA polyTanh, normalized at the crest -- saturates
// =============================================================================
struct MalaShapeSetup {
    bool  saturate = false;
    bool  bypass   = false;
    float thinK    = 0.f;
    float satDrive = 1e-4f;
    float satNorm  = 1.f;
    float linearRestore = 1.f;

    void set(float shape) {
        // Identity at centre; skip the divide.
        bypass   = (fabsf(shape) < 1e-6f);
        saturate = (shape > 0.f);
        if (saturate) {
            // Normalized by the curve at the same gain, so the crest stays at unity.
            satDrive = fmaxf(shape * kSatMax, 1e-4f);
            // The curve clamps at 1, so from there on its peak is a constant.
            satNorm  = (satDrive >= 1.f) ? kSatNormFull : 1.f / MalaADAADrive::curve(satDrive);
            // Fades out the ADAA half-sample average near zero, so the
            // saturator meets the bypass with no step.
            linearRestore = 1.f - shape;
            thinK    = 0.f;
        }
        else {
            thinK    = fabsf(shape) * kShapeThin;
            satDrive = 1e-4f;
            satNorm  = 1.f;
        }
    }
};


// =============================================================================
struct Mala : Module {

    // Mod bus destinations, as a bitmask.  Stereo mapping SHAPE/HEAT costs a second
    // window, RATIO/SPREAD a second partial set, MORPH/SPREAD a second weight set.
    enum MalaBit {
        M_SHAPE  = 1,
        M_RATIO  = 2,
        M_MORPH  = 4,
        M_SPREAD = 8,
        M_DETUNE = 16,
        M_PHASE  = 32,
        M_HEAT   = 64
    };

    enum ParamIds {
        SHAPE_PARAM,   SHAPE_TRIM_PARAM,
        RATIO_PARAM,   RATIO_TRIM_PARAM,
        MORPH_PARAM,   MORPH_TRIM_PARAM,
        SPREAD_PARAM,  SPREAD_TRIM_PARAM,
        DETUNE_PARAM,  DETUNE_TRIM_PARAM,
        PHASE_PARAM,   PHASE_TRIM_PARAM,
        MODRATE_PARAM,      MODRATE_TRIM_PARAM,
        HEAT_PARAM,      HEAT_TRIM_PARAM,
        MODDEPTH_PARAM,    MODDEPTH_TRIM_PARAM,
        FREQ_PARAM,
        FM_AMT_PARAM,
        VOLUME_PARAM,
        MAP_SHAPE_PARAM, MAP_RATIO_PARAM, MAP_MORPH_PARAM,
        MAP_SPREAD_PARAM, MAP_DETUNE_PARAM, MAP_PHASE_PARAM, MAP_HEAT_PARAM,
        SYNC_PARAM,
        QUANT_PARAM,
        SYM_PARAM,
        NUM_PARAMS
    };

    enum InputIds {
        VOCT_INPUT,
        FM_INPUT,
        SHAPE_CV_INPUT,
        RATIO_CV_INPUT,
        MORPH_CV_INPUT,
        SPREAD_CV_INPUT,
        DETUNE_CV_INPUT,
        PHASE_CV_INPUT,
        MODRATE_CV_INPUT,
        HEAT_CV_INPUT,
        MODDEPTH_CV_INPUT,
        VOLUME_CV_INPUT,
        NUM_INPUTS
    };

    enum OutputIds {
        OUT_L_OUTPUT,
        OUT_R_OUTPUT,
        A_OUTPUT,
        B_OUTPUT,
        MOD_OUTPUT,
        NUM_OUTPUTS
    };

    enum LightIds {
        LED_SHAPE, LED_RATIO, LED_MORPH, LED_SPREAD, LED_DETUNE, LED_PHASE, LED_HEAT,
        LED_SYNC, LED_QUANT, LED_SYM,
        NUM_LIGHTS
    };

    // ---- Oscillator state ----------------------------------------------------
    double phaseA[kMaxPoly]  = {};
    // Right channel fundamental, used only while DETUNE is engaged.
    double phaseAR[kMaxPoly] = {};
    // B partial and noise phases.  The right copies mirror the left unless the
    // sides differ.
    float  partialPhase [kMaxPoly][kPartials] = {};
    float  partialPhaseR[kMaxPoly][kPartials] = {};
    float  lutPhase [kMaxPoly] = {};
    float  lutPhaseR[kMaxPoly] = {};

    // One saturator per voice per side; the ADAA needs the previous input.
    MalaADAADrive satL[kMaxPoly], satR[kMaxPoly];

    // ---- Decimation and DC ---------------------------------------------------
    // Two voices share each filter: lanes are voice 2p L, voice 2p R, voice
    // 2p+1 L, voice 2p+1 R.  The DC state uses the same layout, voice * 2 + side.
    MalaDecim2 decim[kVoicePairs];
    float dcIn [kMaxPoly * 2] = {};
    float dcOut[kMaxPoly * 2] = {};

    // ---- Noise -------------------------------------------------------------
    // Triple buffered: rebuilt on the UI thread while audio reads, and the third
    // buffer keeps the published table from being the next one overwritten.
    float lut[3][kLutLevels][kLutSize];
    std::atomic<int> lutActive{0};
    int   lutSeed = 0;
    float brightHz = kBrightDef;
    float lutAcc[kLutSize];   // build scratch for buildLut

    // ---- MALA bus ------------------------------------------------------------
    int   modTarget = 0;
    dsp::SchmittTrigger mapTriggers[7];
    // Per destination gate, ramped so map buttons fade the bus in and out.
    float modGate[7] = {};
    int   gatesSettledAt = -1;   // routing the gates have fully landed on, -1 while ramping
    // B phase rotation coefficients.  Every 32 samples a new target angle is
    // set and the rotation glides to it by a fixed per-sample step.  Seeded to
    // the identity so an unrefreshed voice is unrotated, not silent.
    float phaseCosD[kMaxPoly][kPartials];
    float phaseSinD[kMaxPoly][kPartials] = {};
    float phaseStepCos[kMaxPoly][kPartials];
    float phaseStepSin[kMaxPoly][kPartials] = {};
    float phaseAngle[kMaxPoly][kPartials] = {};   // where the current glide ends
    bool  phaseWasOn[kMaxPoly] = {};
    int   phaseHold[kMaxPoly] = {};   // samples left before an idle rotation disengages
    int   phaseDivCounter = 0;
    dsp::SchmittTrigger quantTrigger, syncTrigger;
    int   buttonDivCounter = 0;
    float lfoPhase = 0.f;
    float lfoVoiceOffset[kMaxPoly] = {};   // vi / nVoices, for the spread LFO

    // ---- Oversampling --------------------------------------------------------
    // Fixed 4x by default: Auto moves the decimator and so the output's
    // headroom whenever it switches.
    int  osActive   = 4;
    int  osSetting  = 4;      // 1, 2, 4 (default), 8, or 0 for Auto
    int  osPending  = 4;
    int  osStable   = 0;
    int  osDivCounter = 0;
    // Auto oversampling skirt width, in units of the fundamental, fitted to
    // measured fold-back.  Raise for margin at more CPU.
    float autoOsK = 8.0f;

    // ---- Options -------------------------------------------------------------
    bool quantOn      = false;
    bool syncOn       = true;
    bool polyOutputs  = true;
    bool detuneWide   = false;  // context menu, multiplies the detune range
    int  lfoPhaseMode = 0;      // 0 spread, 1 unison
    int  lfoShape     = 0;      // 0 sine, 1 triangle, 2 slow random
    bool extOverride  = false;  // a patched Ma CV replaces the Mala wave

    // Slow random: four most recent targets per voice, Catmull-Rom between the
    // middle two, one new target per Mala cycle.
    float rndPoints[kMaxPoly][4] = {};
    float rndLastPhase[kMaxPoly] = {};

    // ---- Dead zone latching --------------------------------------------------
    // w is exactly zero around A's zero crossing, so B-only changes made there are
    // inaudible.  Used for SYM and the split -> shared B collapse, per voice.
    bool symOn = false;     // graft follows A's sign, versus both crests +B
    bool symAct[kMaxPoly] = {};
    bool splitAct[kMaxPoly] = {};
    float lastWL[kMaxPoly] = {}, lastWR[kMaxPoly] = {};
    // The right saturator is only fed while stereoCore holds, so it needs
    // catching up whenever that resumes.
    bool  corePrev[kMaxPoly] = {};
    int   latchWait[kMaxPoly] = {};
    dsp::SchmittTrigger symTrigger;

    // Slewed detune amount per voice, so releasing the slider glides.
    float detuneSlew[kMaxPoly] = {};

    // ---- Control rate state --------------------------------------------------
    float ccRatioIn [kMaxPoly][2] = {};   // quantizer input,  L and R
    float ccRatioOut[kMaxPoly][2] = {};   // quantizer output
    float ccMul     [kMaxPoly][2][kPartials] = {};
    // Cleared by an invalidation (reset, sample rate, brightness); the next
    // rebuild then snaps the weights to their targets instead of ramping.
    bool  ccValid   [kMaxPoly]    = {};

    // Weights, normalizer, noise mix and noise mip level, ramped linearly to a
    // target rebuilt every kWeightDiv samples.  Side 0 is L, side 1 is R.
    float ccWeight      [kMaxPoly][2][kPartials] = {};
    float ccWeightTarget[kMaxPoly][2][kPartials] = {};
    float ccWeightStep  [kMaxPoly][2][kPartials] = {};
    float ccNorm        [kMaxPoly][2] = {};
    float ccNormTarget  [kMaxPoly][2] = {};
    float ccNormStep    [kMaxPoly][2] = {};
    float ccNoise       [kMaxPoly][2] = {};
    float ccNoiseTarget [kMaxPoly][2] = {};
    float ccNoiseStep   [kMaxPoly][2] = {};
    float ccLutLevel      [kMaxPoly] = {};
    float ccLutLevelTarget[kMaxPoly] = {};
    float ccLutLevelStep  [kMaxPoly] = {};
    int   weightRampLeft  [kMaxPoly] = {};   // samples until the ramp lands and the next rebuild

    // Exact-input caches: recomputed whenever the input differs at all, so the
    // result is identical to computing it every sample.  They pay off while a
    // control sits still; under modulation or pot noise they simply miss.
    float ccShapeIn   [kMaxPoly][2] = {};
    MalaShapeSetup ccShape[kMaxPoly][2];
    float lfoRateIn = -1.f, lfoRateHz = 0.f;

    int prevVoices = 0;

    // ---- Display -------------------------------------------------------------
    float displayShape    = 0.f;
    float displayHeat     = 0.f;
    float displayNoiseMix = 0.f;
    float displayRatio    = 1.f;   // linear, for the readout
    float displayWeight[kPartials]   = {};
    float displayRatioMul[kPartials] = {};

    // Double buffered.  The audio thread fills capWrite; the UI only ever
    // draws capRead, which always holds a completed cycle.
    float capL[2][kCaptureN] = {}, capR[2][kCaptureN] = {}, capW[2][kCaptureN] = {};
    int   capCount[2] = { 2, 2 };
    int   capWrite = 0, capRead = 1;
    int   capIndex = 0, capStride = 1, capStrideCount = 1;
    int   capMinPoints = 2;   // a wrap before this many points is FM jitter, not a cycle
    int   capIdle = 0;        // samples waited for a wrap since the last capture filled
    bool  capWriting = false;

    // =========================================================================
    Mala() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        configParam(SHAPE_PARAM,        -1.f,  1.f,  0.f,  "Shape");
        configParam(SHAPE_TRIM_PARAM,   -1.f,  1.f,  0.f,  "Shape Mod Depth");
        configParam(RATIO_PARAM,        -3.f,  4.f,  0.f,  "Ratio", " x", 2.f);
        configParam(RATIO_TRIM_PARAM,   -1.f,  1.f,  0.f,  "Ratio Mod Depth");
        configParam(MORPH_PARAM,         0.f, (float)(kMorphStops - 1), 6.f, "Morph");
        configParam(MORPH_TRIM_PARAM,   -1.f,  1.f,  0.f,  "Morph Mod Depth");
        configParam(SPREAD_PARAM,        0.f,  1.f,  0.f,  "Spread");
        configParam(SPREAD_TRIM_PARAM,  -1.f,  1.f,  0.f,  "Spread Mod Depth");
        configParam(DETUNE_PARAM,        0.f,  1.f,  0.f,  "Detune");
        configParam(DETUNE_TRIM_PARAM,  -1.f,  1.f,  0.f,  "Detune Mod Depth");
        configParam(PHASE_PARAM,         0.f,  1.f,  0.f,  "B Phase");
        configParam(PHASE_TRIM_PARAM,   -1.f,  1.f,  0.f,  "B Phase Mod Depth");
        configParam(MODRATE_PARAM,            0.f,  1.f,  0.25f, "Ma (rate)");
        configParam(MODRATE_TRIM_PARAM,      -1.f,  1.f,  0.f,  "Ma CV Trim");
        configParam(HEAT_PARAM,            0.f, 10.f,  0.f,  "Heat");
        configParam(HEAT_TRIM_PARAM,      -1.f,  1.f,  0.f,  "Heat Mod Depth");
        configParam(MODDEPTH_PARAM,          0.f,  1.f,  0.f,  "La (mod index)");
        configParam(MODDEPTH_TRIM_PARAM,    -1.f,  1.f,  0.f,  "La CV Trim");
        configParam(FREQ_PARAM,         -5.f,  5.f,  0.f,  "Frequency", " V");
        configParam(FM_AMT_PARAM,       -1.f,  1.f,  0.f,  "Through-zero FM index");
        configParam(VOLUME_PARAM,        0.f,  1.f,  1.f,  "Volume");

        configParam(MAP_SHAPE_PARAM,   0.f, 1.f, 0.f, "Map Mala to Shape");
        configParam(MAP_RATIO_PARAM,   0.f, 1.f, 0.f, "Map Mala to Ratio");
        configParam(MAP_MORPH_PARAM,   0.f, 1.f, 0.f, "Map Mala to Morph");
        configParam(MAP_SPREAD_PARAM,  0.f, 1.f, 0.f, "Map Mala to Spread");
        configParam(MAP_DETUNE_PARAM,  0.f, 1.f, 0.f, "Map Mala to Detune");
        configParam(MAP_PHASE_PARAM,   0.f, 1.f, 0.f, "Map Mala to B Phase");
        configParam(MAP_HEAT_PARAM,    0.f, 1.f, 0.f, "Map Mala to Heat");
        configParam(SYNC_PARAM,        0.f, 1.f, 0.f, "B Sync");
        configParam(QUANT_PARAM,       0.f, 1.f, 0.f, "Ratio Quantize");
        configParam(SYM_PARAM,         0.f, 1.f, 0.f, "Symmetry: graft follows A's sign");

        configInput(VOCT_INPUT,       "V/Oct (poly)");
        configInput(FM_INPUT,         "Through-zero linear FM (poly)");
        configInput(SHAPE_CV_INPUT,   "Shape CV");
        configInput(RATIO_CV_INPUT,   "Ratio CV");
        configInput(MORPH_CV_INPUT,   "Morph CV");
        configInput(SPREAD_CV_INPUT,  "Spread CV");
        configInput(DETUNE_CV_INPUT,  "Detune CV");
        configInput(PHASE_CV_INPUT,   "B Phase CV");
        configInput(MODRATE_CV_INPUT, "Ma rate CV");
        configInput(HEAT_CV_INPUT,    "Heat CV");
        configInput(MODDEPTH_CV_INPUT,"La index CV");
        configInput(VOLUME_CV_INPUT,  "Volume CV (VCA, 0-10 V, knob sets max)");

        configOutput(OUT_L_OUTPUT, "Left");
        configOutput(OUT_R_OUTPUT, "Right");
        configOutput(A_OUTPUT,     "Osc A (shaped sine)");
        configOutput(B_OUTPUT,     "Osc B (additive)");
        configOutput(MOD_OUTPUT,  "Mala LFO");

        for (int i = 0; i < kMaxPoly; i++)
            for (int n = 0; n < kPartials; n++) phaseCosD[i][n] = phaseStepCos[i][n] = 1.f;

        buildLutAllBuffers(0);
        retuneDecimators();
    }

    // All three buffers hold the same seed outside a seed change, so build one
    // and copy it; the build is the slow part of instantiating the module.
    void buildLutAllBuffers(int seedIdx) {
        buildLut(0, seedIdx);
        std::copy(&lut[0][0][0], &lut[0][0][0] + kLutLevels * kLutSize, &lut[1][0][0]);
        std::copy(&lut[0][0][0], &lut[0][0][0] + kLutLevels * kLutSize, &lut[2][0][0]);
    }

    // -------------------------------------------------------------------------
    // Band-limited noise table from random phase partials, one mip level per
    // halving of the harmonic count.
    void buildLut(int buf, int seedIdx) {
        // Harmonics accumulate in ascending order and are snapshotted at each level's
        // limit, so the whole mip stack is one pass.
        const float* sinTab = malaSinTab().t;

        uint32_t seed = 0x5EED1234u + (uint32_t)seedIdx * 0x9E3779B9u;
        // Per instance scratch, so two modules can rebuild at once.
        float* acc = lutAcc;
        for (int i = 0; i < kLutSize; i++) acc[i] = 0.f;

        int level = kLutLevels - 1;                 // coarsest first
        const int topHarm = kLutHarm[0];
        for (int n = kNoiseLow; n <= topHarm; n++) {
            seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
            int   ph  = (int)(seed & (uint32_t)(kLutSize - 1));
            float amp = 1.f / sqrtf((float)n / (float)kNoiseLow);
            for (int i = 0; i < kLutSize; i++)
                acc[i] += amp * sinTab[(n * i + ph) & (kLutSize - 1)];

            while (level >= 0 && n == kLutHarm[level]) {
                for (int i = 0; i < kLutSize; i++) lut[buf][level][i] = acc[i];
                level--;
            }
        }
        for (; level >= 0; level--)
            for (int i = 0; i < kLutSize; i++) lut[buf][level][i] = acc[i];

        // RMS normalized so every seed sits at the same level.
        for (int lv = 0; lv < kLutLevels; lv++) {
            float* tab = lut[buf][lv];
            double sumSq = 0.0;
            for (int i = 0; i < kLutSize; i++) sumSq += (double)tab[i] * (double)tab[i];
            float rms = sqrtf((float)(sumSq / (double)kLutSize));
            // Target RMS is hand-tunable; 0.35 leaves room for the soft limit.
            float scale = (rms > 1e-9f) ? (0.35f / rms) : 0.f;
            for (int i = 0; i < kLutSize; i++) {
                float v = tab[i] * scale;
                // Soft limit.
                tab[i] = v / sqrtf(1.f + v * v * 0.5f);
            }
        }
    }

    // UI thread.  The weight cache does not key on the knee, so invalidate it.
    void setBright(float hz) {
        hz = clamp(hz, kBrightMin, kBrightMax);
        if (hz == brightHz) return;
        brightHz = hz;
        for (int i = 0; i < kMaxPoly; i++) ccValid[i] = false;
    }

    // Called from the UI thread when the seed changes.
    void setNoiseSeed(int seedIdx) {
        seedIdx = clamp(seedIdx, 0, kNoiseSeeds - 1);
        if (seedIdx == lutSeed) return;
        int idle = lutActive.load(std::memory_order_relaxed) + 1;
        if (idle > 2) idle = 0;
        buildLut(idle, seedIdx);
        lutSeed = seedIdx;
        lutActive.store(idle, std::memory_order_release);
    }

    // Override turns the Ma CV jack into the Mala source while it is patched.
    void setExtOverride(bool on) {
        extOverride = on;
        inputInfos[MODRATE_CV_INPUT]->name = on
            ? "Mala override (+-5 V, depth by La; Mala wave when unpatched)" : "Ma rate CV";
    }

    void retuneDecimators() {
        float cutoff = kDecimCutoff / (float)osActive;
        for (int p = 0; p < kVoicePairs; p++) decim[p].setCutoffFreq(cutoff);
    }

    // -------------------------------------------------------------------------
    json_t* dataToJson() override {
        json_t* rootJ = json_object();
        json_object_set_new(rootJ, "modTarget",   json_integer(modTarget));
        json_object_set_new(rootJ, "oversampling", json_integer(osSetting));
        json_object_set_new(rootJ, "quantOn",      json_boolean(quantOn));
        json_object_set_new(rootJ, "syncOn",       json_boolean(syncOn));
        json_object_set_new(rootJ, "polyOutputs",  json_boolean(polyOutputs));
        json_object_set_new(rootJ, "detuneWide",   json_boolean(detuneWide));
        json_object_set_new(rootJ, "lutSeed",      json_integer(lutSeed));
        json_object_set_new(rootJ, "brightHz",     json_real(brightHz));
        json_object_set_new(rootJ, "symOn",        json_boolean(symOn));
        json_object_set_new(rootJ, "lfoPhaseMode", json_integer(lfoPhaseMode));
        json_object_set_new(rootJ, "lfoShape",     json_integer(lfoShape));
        json_object_set_new(rootJ, "extOverride",  json_boolean(extOverride));
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        json_t* j;
        j = json_object_get(rootJ, "modTarget");   if (j) modTarget   = (int)json_integer_value(j);
        // "osSetting" is the old key, saved when Auto (0) was the default.  It
        // loads as 4x, so older patches get the steady headroom too; the new
        // key keeps an Auto chosen from now on.
        j = json_object_get(rootJ, "oversampling");
        if (j) osSetting = (int)json_integer_value(j);
        else {
            j = json_object_get(rootJ, "osSetting");
            if (j) { osSetting = (int)json_integer_value(j); if (osSetting == 0) osSetting = 4; }
        }
        if (osSetting != 0 && osSetting != 1 && osSetting != 2 && osSetting != 4 && osSetting != 8)
            osSetting = 4;
        j = json_object_get(rootJ, "quantOn");      if (j) quantOn      = json_is_true(j);
        j = json_object_get(rootJ, "syncOn");       if (j) syncOn       = json_is_true(j);
        j = json_object_get(rootJ, "polyOutputs");  if (j) polyOutputs  = json_is_true(j);
        j = json_object_get(rootJ, "detuneWide");   if (j) detuneWide   = json_is_true(j);
        j = json_object_get(rootJ, "brightHz");     if (j)
            brightHz = clamp((float)json_real_value(j), kBrightMin, kBrightMax);
        j = json_object_get(rootJ, "lutSeed");      if (j) {
            int sd = clamp((int)json_integer_value(j), 0, kNoiseSeeds - 1);
            if (sd != lutSeed) { buildLutAllBuffers(sd); lutSeed = sd; }
        }
        j = json_object_get(rootJ, "symOn");        if (j) symOn        = json_is_true(j);
        // Clamped: older patches may carry a 2 from the removed Random mode.
        j = json_object_get(rootJ, "lfoPhaseMode");
        if (j) lfoPhaseMode = clamp((int)json_integer_value(j), 0, 1);
        j = json_object_get(rootJ, "lfoShape");
        lfoShape = j ? clamp((int)json_integer_value(j), 0, 2) : 0;
        j = json_object_get(rootJ, "extOverride");
        setExtOverride(j && json_is_true(j));

        // A patch load adopts SYM immediately.
        for (int i = 0; i < kMaxPoly; i++) symAct[i] = symOn;

        osActive  = (osSetting == 0) ? 4 : osSetting;
        osPending = osActive;
        retuneDecimators();
    }

    void onReset() override {
        Module::onReset();
        for (int i = 0; i < kMaxPoly; i++) {
            phaseA[i]    = 0.0;
            phaseAR[i]   = 0.0;
            satL[i].reset(); satR[i].reset();
            lutPhase[i] = lutPhaseR[i] = 0.f;
            for (int n = 0; n < kPartials; n++)
                partialPhase[i][n] = partialPhaseR[i][n] = 0.f;
            dcIn[i * 2] = dcIn[i * 2 + 1] = dcOut[i * 2] = dcOut[i * 2 + 1] = 0.f;
            detuneSlew[i] = 0.f;
            lastWL[i] = lastWR[i] = 0.f;
            latchWait[i] = 0;
            symAct[i] = false;
            splitAct[i] = false;
            corePrev[i] = false;
            ccValid[i] = false;
        }
        for (int p = 0; p < kVoicePairs; p++) decim[p].reset();
        for (int i = 0; i < 7; i++) modGate[i] = 0.f;
        gatesSettledAt = -1;
        lfoPhase     = 0.f;
        modTarget   = 0;
        osSetting    = 4;
        osActive     = 4;
        osPending    = 4;
        quantOn      = false;
        syncOn       = true;
        polyOutputs  = true;
        lfoShape     = 0;
        setExtOverride(false);
        detuneWide   = false;
        if (lutSeed != 0) { buildLutAllBuffers(0); lutSeed = 0; }
        brightHz     = kBrightDef;
        symOn        = false;
        capWriting   = false;
        capIdle      = 0;
        retuneDecimators();
    }

    void onSampleRateChange() override {
        Module::onSampleRateChange();
        // The weight cache keys on a Nyquist fade that moves with the rate.
        for (int i = 0; i < kMaxPoly; i++) ccValid[i] = false;
        retuneDecimators();
    }

    // =========================================================================
    void process(const ProcessArgs& args) override {

        int nVoices = inputs[VOCT_INPUT].isConnected()
                    ? std::max(1, inputs[VOCT_INPUT].getChannels()) : 1;

        if (nVoices < prevVoices) {
            for (int vi = nVoices; vi < prevVoices; vi++) {
                phaseA[vi]   = 0.0;
                phaseAR[vi]  = 0.0;
                satL[vi].reset(); satR[vi].reset();
                lutPhase[vi] = lutPhaseR[vi] = 0.f;
                for (int n = 0; n < kPartials; n++)
                    partialPhase[vi][n] = partialPhaseR[vi][n] = 0.f;
                detuneSlew[vi] = 0.f;
                lastWL[vi] = lastWR[vi] = 0.f;
                splitAct[vi] = false;
                corePrev[vi] = false;
                dcIn[vi * 2] = dcIn[vi * 2 + 1] = dcOut[vi * 2] = dcOut[vi * 2 + 1] = 0.f;
            }
        }
        // Spread LFO phase offsets, one division per voice only when the count changes.
        if (nVoices != prevVoices)
            for (int vi = 0; vi < nVoices; vi++) lfoVoiceOffset[vi] = (float)vi / (float)nVoices;
        prevVoices = nVoices;

        // ---- Latching buttons ------------------------------------------------
        struct { int param; int bit; } mapBtns[7] = {
            { MAP_SHAPE_PARAM,  M_SHAPE  }, { MAP_RATIO_PARAM,  M_RATIO  },
            { MAP_MORPH_PARAM,  M_MORPH  }, { MAP_SPREAD_PARAM, M_SPREAD },
            { MAP_DETUNE_PARAM, M_DETUNE }, { MAP_PHASE_PARAM,  M_PHASE  },
            { MAP_HEAT_PARAM,   M_HEAT   }
        };
        // Polled at a control rate; a press lasts far longer than kButtonDiv samples.
        const bool pollButtons = (buttonDivCounter == 0);
        if (++buttonDivCounter >= kButtonDiv) buttonDivCounter = 0;
        if (pollButtons) {
            for (int i = 0; i < 7; i++)
                if (mapTriggers[i].process(params[mapBtns[i].param].getValue()))
                    modTarget ^= mapBtns[i].bit;
            if (quantTrigger.process(params[QUANT_PARAM].getValue())) quantOn = !quantOn;
            if (syncTrigger.process (params[SYNC_PARAM ].getValue())) syncOn  = !syncOn;
            if (symTrigger.process  (params[SYM_PARAM  ].getValue())) symOn   = !symOn;
        }

        // Routing gates ramp toward their button state.  Once every gate has
        // landed for the current routing the ramp step is exactly zero, so the
        // loop is skipped until the routing changes.
        if (modTarget != gatesSettledAt) {
            float gateCoef = clamp(args.sampleTime / kGateTau, 0.f, 1.f);
            bool settled = true;
            for (int i = 0; i < 7; i++) {
                float target = (modTarget & mapBtns[i].bit) ? 1.f : 0.f;
                if (modGate[i] == target) continue;
                modGate[i] += (target - modGate[i]) * gateCoef;
                if (fabsf(target - modGate[i]) < 1e-4f) modGate[i] = target;
                else settled = false;
            }
            if (settled) gatesSettledAt = modTarget;
        }

        // Max wait for a dead zone before a latched change is forced, in samples.
        const int kLatchTimeout = (int)(args.sampleRate * 0.25f);

        // ---- Global params ---------------------------------------------------
        float shapeBase  = params[SHAPE_PARAM].getValue();
        float shapeTrim  = params[SHAPE_TRIM_PARAM].getValue();
        float ratioBase  = params[RATIO_PARAM].getValue();
        float ratioTrim  = params[RATIO_TRIM_PARAM].getValue();
        float morphBase  = params[MORPH_PARAM].getValue();
        float morphTrim  = params[MORPH_TRIM_PARAM].getValue();
        float spreadBase = params[SPREAD_PARAM].getValue();
        float spreadTrim = params[SPREAD_TRIM_PARAM].getValue();
        float detuneBase = params[DETUNE_PARAM].getValue();
        float detuneTrim = params[DETUNE_TRIM_PARAM].getValue();
        float phaseBase  = params[PHASE_PARAM].getValue();
        float phaseTrim  = params[PHASE_TRIM_PARAM].getValue();
        float heatBase     = params[HEAT_PARAM].getValue();
        float heatTrim     = params[HEAT_TRIM_PARAM].getValue();
        float modRateKnob     = params[MODRATE_PARAM].getValue();
        float modRateTrim     = params[MODRATE_TRIM_PARAM].getValue() * 0.1f;
        float depthKnob   = params[MODDEPTH_PARAM].getValue();
        float depthTrimAmt  = params[MODDEPTH_TRIM_PARAM].getValue() * 0.1f;
        float freqKnob   = params[FREQ_PARAM].getValue();
        float fmAmount   = params[FM_AMT_PARAM].getValue();

        bool fmOn = inputs[FM_INPUT].isConnected() && fmAmount != 0.f;

        // ---- Ma: the rate of the Mala LFO ------------------------------------
        // Exponential, 0.02 Hz to 50 Hz: log2(2500) = 11.2877.  With the override
        // on and the jack patched, the Ma CV is the source and stops steering the rate.
        bool extSource = extOverride && inputs[MODRATE_CV_INPUT].isConnected();
        bool rateCvOn  = inputs[MODRATE_CV_INPUT].isConnected() && !extSource;
        float modRateNorm = clamp(modRateKnob + (rateCvOn
                     ? inputs[MODRATE_CV_INPUT].getVoltage() * modRateTrim : 0.f), 0.f, 1.f);
        if (modRateNorm != lfoRateIn) {
            lfoRateIn = modRateNorm;
            lfoRateHz = 0.02f * dsp::exp2_taylor5(modRateNorm * 11.2877f);
        }
        // The LFO phase sums below all stay in [0, 2), so one conditional subtract
        // gives exactly x - floor(x) without the floorf call.
        lfoPhase += lfoRateHz * args.sampleTime;
        if (lfoPhase >= 1.f) lfoPhase -= 1.f;

        // ---- Mala LFO value per voice ----------------------------------------
        float modLfo[kMaxPoly];
        for (int vi = 0; vi < nVoices; vi++) {
            if (extSource) {
                // +-5 V spans the same swing as the internal LFO.
                modLfo[vi] = clamp(inputs[MODRATE_CV_INPUT].getPolyVoltage(vi) * 0.2f, -1.f, 1.f);
                continue;
            }
            float p = lfoPhase;
            if (lfoPhaseMode != 1) {
                p += lfoVoiceOffset[vi];
                if (p >= 1.f) p -= 1.f;
            }
            if (lfoShape == 1) {
                // Triangle, phase aligned with the sine.
                float quarterAhead = p + 0.25f;
                if (quarterAhead >= 1.f) quarterAhead -= 1.f;
                modLfo[vi] = 1.f - 4.f * fabsf(quarterAhead - 0.5f);
            }
            else if (lfoShape == 2) {
                // Unison voices share voice 0's sequence.
                if (lfoPhaseMode == 1 && vi > 0) { modLfo[vi] = modLfo[0]; continue; }
                float* pts = rndPoints[vi];
                if (p < rndLastPhase[vi]) {
                    pts[0] = pts[1]; pts[1] = pts[2]; pts[2] = pts[3];
                    pts[3] = 2.f * random::uniform() - 1.f;
                }
                rndLastPhase[vi] = p;
                // Catmull-Rom from pts[1] to pts[2]; clamped for its small overshoot.
                float c = 0.5f * ((-pts[0] + 3.f * pts[1] - 3.f * pts[2] + pts[3]) * p * p * p
                                + (2.f * pts[0] - 5.f * pts[1] + 4.f * pts[2] - pts[3]) * p * p
                                + (pts[2] - pts[0]) * p
                                + 2.f * pts[1]);
                modLfo[vi] = clamp(c, -1.f, 1.f);
            }
            else {
                modLfo[vi] = malaSin2pi(kTwoPi * p);
            }
        }

        // Both dividers count modulo 32 so they never overflow in a patch left
        // running for days; only their low five bits were ever used.
        phaseDivCounter = (phaseDivCounter + 1) & 31;

        // ---- Oversampling decision -------------------------------------------
        const bool osTick = (osDivCounter == 0);
        osDivCounter = (osDivCounter + 1) & 31;
        if (osTick) {
            if (osSetting != 0) {
                osPending = osSetting;
                osStable  = 2;
            }
            else {
                float worstF0   = 0.f;
                float worstTop  = 0.f;
                // Budget for the higher channel when detune is spreading them.
                float ratioBaseHz = exp2f(clamp(ratioBase, -3.f, 4.f))
                                  * exp2f(clamp(detuneBase, 0.f, 1.f) * kDetuneOct
                                          * (detuneWide ? kDetuneWide : 1.f) * 0.5f);
                float topMul = (float)kPartials
                             + clamp(spreadBase, 0.f, 1.f) * kSpreadOffset[kPartials - 1];
                for (int vi = 0; vi < nVoices; vi++) {
                    float voct = inputs[VOCT_INPUT].isConnected()
                               ? inputs[VOCT_INPUT].getPolyVoltage(vi) : 0.f;
                    float f0 = dsp::FREQ_C4 * dsp::exp2_taylor5(voct + freqKnob);
                    worstF0  = fmaxf(worstF0, f0);
                    worstTop = fmaxf(worstTop, f0 * ratioBaseHz * topMul);
                }

                // Partials above the Nyquist fade are already gone, so cap the estimate.
                worstTop = fminf(worstTop, kPartialNyq * args.sampleRate);

                // Must mirror the HEAT mapping used in the voice loop.
                float drive = kThresh + clamp(heatBase, 0.f, 10.f) * 0.1f
                            * (kDriveMax - kThresh);
                int want = 1;
                if (drive > kThresh) {
                    // Phase width of the window's transition.
                    float thHi = asinf(fminf(kPi / drive, 1.f));
                    float thLo = asinf(fminf(kThresh / drive, 1.f));
                    float dTheta = fmaxf(thHi - thLo, 1e-3f);

                    // Window depth at the crest.  The sqrt is empirical.
                    float tPk = clamp((drive - kThresh) / (kPi - kThresh), 0.f, 1.f);
                    float wPk = malaWindow(tPk);

                    // SHAPE's saturator adds its own harmonics.  Compared against half the base
                    // rate, since the graft skirt sits on top of B's partials.
                    float shapeTop = worstF0 * (1.f + 10.f * fmaxf(shapeBase, 0.f));
                    float bwNeeded = fmaxf(worstTop, shapeTop)
                                   + worstF0 * autoOsK * sqrtf(wPk) / dTheta;

                    float need = bwNeeded / (0.5f * args.sampleRate);
                    want = (need > 4.f) ? 8 : (need > 2.f) ? 4 : (need > 1.f) ? 2 : 1;
                }

                if (want == osPending) { if (osStable < 2) osStable++; }
                else { osPending = want; osStable = 0; }
            }
        }

        // ---- Per voice -------------------------------------------------------
        float mixL = 0.f, mixR = 0.f;
        float nyq      = kPartialNyq * args.sampleRate;
        float fadeSpan = 1.f / (0.25f * nyq);
        float volume   = clamp(params[VOLUME_PARAM].getValue(), 0.f, 1.f);
        bool  bPatched = outputs[B_OUTPUT].isConnected();
        // Acquire pairs with the release store in setNoiseSeed.
        const float (*lutTab)[kLutSize] = lut[lutActive.load(std::memory_order_acquire)];

        // Jack and zero-trim tests hoisted out of the voice loop.
        bool cvOnShape  = inputs[SHAPE_CV_INPUT ].isConnected() && shapeTrim  != 0.f;
        bool cvOnRatio  = inputs[RATIO_CV_INPUT ].isConnected() && ratioTrim  != 0.f;
        bool cvOnMorph  = inputs[MORPH_CV_INPUT ].isConnected() && morphTrim  != 0.f;
        bool cvOnSpread = inputs[SPREAD_CV_INPUT].isConnected() && spreadTrim != 0.f;
        bool cvOnDetune = inputs[DETUNE_CV_INPUT].isConnected() && detuneTrim != 0.f;
        bool cvOnPhase  = inputs[PHASE_CV_INPUT ].isConnected() && phaseTrim  != 0.f;
        bool cvOnHeat   = inputs[HEAT_CV_INPUT  ].isConnected() && heatTrim   != 0.f;
        bool cvOnDepth  = inputs[MODDEPTH_CV_INPUT].isConnected() && depthTrimAmt != 0.f;
        bool voctOn     = inputs[VOCT_INPUT].isConnected();
        bool vcaOn      = inputs[VOLUME_CV_INPUT].isConnected();

        // Voice invariant scalars, likewise.
        const float morphTop   = (float)(kMorphStops - 1);
        const float detuneSpan = kDetuneOct * (detuneWide ? kDetuneWide : 1.f);
        const float dStep      = kDetuneRate * args.sampleTime;
        const float fMax       = args.sampleRate * 0.45f;
        const float heatToDriv = 0.1f * (kDriveMax - kThresh);
        const int   os         = osActive;
        const float osInv      = 1.f / (float)os;
        const float incScale   = args.sampleTime * osInv;

        // Oversampled graft per voice, filled by the voice loop and decimated
        // two voices at a time after it.  Index voice * 2 + side.
        float decimIn[kMaxOs][kMaxPoly * 2];
        // Per voice DC blocker pole, and what voice 0 hands to the scope.
        float dcPole[kMaxPoly];
        bool  scopeWrapped = false;
        float scopeFreq = 0.f, scopeWindow = 0.f;

        for (int vi = 0; vi < nVoices; vi++) {

            // Exact-input caches recompute on the first sample after an invalidation.
            const bool cacheFresh = !ccValid[vi];

            // --- Pitch, converted further down with the other exponentials ---
            float voct = voctOn ? inputs[VOCT_INPUT].getPolyVoltage(vi) : 0.f;

            // --- Modulation ---
            // CV + trim + slider set the centre point, common to both channels.  The bus
            // reaches a destination through its ramped gate, in antiphase L/R.
            float modDepth = clamp(depthKnob + (cvOnDepth
                            ? inputs[MODDEPTH_CV_INPUT].getPolyVoltage(vi) * depthTrimAmt : 0.f), 0.f, 1.f);
            float bus = modLfo[vi] * modDepth;

            float cvShape  = cvOnShape  ? inputs[SHAPE_CV_INPUT ].getPolyVoltage(vi) * 0.2f * shapeTrim  * kModShape  : 0.f;
            float cvRatio  = cvOnRatio  ? inputs[RATIO_CV_INPUT ].getPolyVoltage(vi) * 0.2f * ratioTrim  * kModRatio  : 0.f;
            float cvMorph  = cvOnMorph  ? inputs[MORPH_CV_INPUT ].getPolyVoltage(vi) * 0.2f * morphTrim  * kModMorph  : 0.f;
            float cvSpread = cvOnSpread ? inputs[SPREAD_CV_INPUT].getPolyVoltage(vi) * 0.2f * spreadTrim * kModSpread : 0.f;
            float cvDetune = cvOnDetune ? inputs[DETUNE_CV_INPUT].getPolyVoltage(vi) * 0.2f * detuneTrim * kModDetune : 0.f;
            float cvPhase  = cvOnPhase  ? inputs[PHASE_CV_INPUT ].getPolyVoltage(vi) * 0.2f * phaseTrim  * kModPhase  : 0.f;
            float cvHeat   = cvOnHeat   ? inputs[HEAT_CV_INPUT  ].getPolyVoltage(vi) * 0.2f * heatTrim   * kModHeat   : 0.f;

            float busShape  = modGate[0] * bus * kModShape;
            float busRatio  = modGate[1] * bus * kModRatio;
            float busMorph  = modGate[2] * bus * kModMorph;
            float busSpread = modGate[3] * bus * kModSpread;
            float busDetune = modGate[4] * bus * kModDetune;
            float busPhase  = modGate[5] * bus * kModPhase;
            float busHeat   = modGate[6] * bus * kModHeat;

            float shapeL  = clamp(shapeBase  + cvShape  + busShape,  -1.f, 1.f);
            float shapeR  = clamp(shapeBase  + cvShape  - busShape,  -1.f, 1.f);
            float heatL   = clamp(heatBase   + cvHeat   + busHeat,    0.f, 10.f);
            float heatR   = clamp(heatBase   + cvHeat   - busHeat,    0.f, 10.f);
            float morphL  = clamp(morphBase  + cvMorph  + busMorph,   0.f, morphTop);
            float morphR  = clamp(morphBase  + cvMorph  - busMorph,   0.f, morphTop);
            float spreadL = clamp(spreadBase + cvSpread + busSpread,  0.f, 1.f);
            float spreadR = clamp(spreadBase + cvSpread - busSpread,  0.f, 1.f);

            // Detune and B phase are already stereo, so the bus scales their amount.
            // Detune is bipolar internally so the bus swings it both ways.
            // Only slider and CV glide; the Mala bus is already smooth and goes straight through.
            float detuneAmt = clamp(detuneBase + cvDetune, -1.f, 1.f);
            // Neither wrapped nor clamped -- see kPhaseRange.
            float phaseAmt  = phaseBase + cvPhase + busPhase;

            float ratioVL = clamp(ratioBase + cvRatio + busRatio, -3.f, 4.f);
            float ratioVR = clamp(ratioBase + cvRatio - busRatio, -3.f, 4.f);

            // --- Quantize before detune, so the channels keep their interval ---
            if (quantOn) {
                // Sorted table: stop once the error grows.  Cached on its input.
                if (!ccValid[vi] || ratioVL != ccRatioIn[vi][0]) {
                    float best = kQuantLog2[0], err = fabsf(ratioVL - kQuantLog2[0]);
                    for (int q = 1; q < kQuantCount; q++) {
                        float e = fabsf(ratioVL - kQuantLog2[q]);
                        if (e > err) break;
                        err = e; best = kQuantLog2[q];
                    }
                    ccRatioIn[vi][0] = ratioVL; ccRatioOut[vi][0] = best;
                }
                if (!ccValid[vi] || ratioVR != ccRatioIn[vi][1]) {
                    float best = kQuantLog2[0], err = fabsf(ratioVR - kQuantLog2[0]);
                    for (int q = 1; q < kQuantCount; q++) {
                        float e = fabsf(ratioVR - kQuantLog2[q]);
                        if (e > err) break;
                        err = e; best = kQuantLog2[q];
                    }
                    ccRatioIn[vi][1] = ratioVR; ccRatioOut[vi][1] = best;
                }
                ratioVL = ccRatioOut[vi][0];
                ratioVR = ccRatioOut[vi][1];
            }

            // --- Detune splits the fundamental ---
            // Each side gets its own A (and so B) frequency around f0, so SYNC stays
            // coherent per side.  The amount slews linearly.
            float dGap  = detuneAmt - detuneSlew[vi];
            if (dGap >  dStep) detuneSlew[vi] += dStep;
            else if (dGap < -dStep) detuneSlew[vi] -= dStep;
            else detuneSlew[vi] = detuneAmt;

            float halfDetune = clamp(detuneSlew[vi] + busDetune, -1.f, 1.f) * detuneSpan * 0.5f;
            bool  stereoPitch = (halfDetune != 0.f);
            // All of the voice's exponentials in one vector: pitch, detune up and
            // down, left ratio.  Lane for lane the same as the scalar call.
            simd::float_4 expIn(voct + freqKnob, halfDetune, -halfDetune, ratioVL);
            simd::float_4 expOut = dsp::exp2_taylor5(expIn);
            float ratioLinL = expOut[3];
            float ratioLinR = (ratioVR == ratioVL) ? ratioLinL : dsp::exp2_taylor5(ratioVR);

            float f0 = clamp(dsp::FREQ_C4 * expOut[0], 0.001f, fMax);
            // Through-zero linear FM: 1 V at full index deviates by f0, and a negative
            // frequency runs every phasor backwards.  Clamped to +-Nyquist because the
            // phasors wrap with one add or subtract.
            float fmVolts = fmOn ? inputs[FM_INPUT].getPolyVoltage(vi) * fmAmount : 0.f;
            float f0Fm = clamp(f0 + fmVolts * f0, -fMax, fMax);

            float f0L = stereoPitch ? (f0Fm * expOut[1]) : f0Fm;
            float f0R = stereoPitch ? (f0Fm * expOut[2]) : f0Fm;

            float fBL = f0L * ratioLinL;
            float fBR = f0R * ratioLinR;

            // --- Return to unison: constant bend back to the left phasor ---
            double phaseGap = phaseA[vi] - phaseAR[vi];
            if (phaseGap >  0.5) phaseGap -= 1.0;
            if (phaseGap < -0.5) phaseGap += 1.0;
            bool realigning = !stereoPitch && phaseGap != 0.0;

            // --- Dead zone latching ---
            // Splitting is always safe; collapsing B's right channel and SYM changes wait
            // for the window to be shut.
            bool wantSplit = stereoPitch || realigning || fabsf(busRatio) > 1e-6f;
            if (wantSplit) splitAct[vi] = true;

            bool pendingSym   = (symAct[vi] != symOn);
            bool pendingSplit = (splitAct[vi] && !wantSplit);
            if (pendingSym || pendingSplit) {
                bool inDeadZone = (lastWL[vi] <= 0.f && lastWR[vi] <= 0.f);
                if (inDeadZone || ++latchWait[vi] >= kLatchTimeout) {
                    symAct[vi] = symOn;
                    if (pendingSplit) splitAct[vi] = false;
                    latchWait[vi] = 0;
                }
            }
            else latchWait[vi] = 0;

            bool splitPitch = splitAct[vi];
            bool stereoCore = splitPitch
                           || fabsf(busShape) > 1e-6f || fabsf(busHeat) > 1e-6f;
            bool stereoOsc  = splitPitch
                           || fabsf(busRatio) > 1e-6f || fabsf(busSpread) > 1e-6f;
            bool stereoWgt  = fabsf(busMorph) > 1e-6f || fabsf(busSpread) > 1e-6f;
            // Held on for two refresh periods after the offset reaches zero,
            // so the rotation glides home before it is dropped.
            if (fabsf(phaseAmt) > 1e-5f) phaseHold[vi] = 64;
            else if (phaseHold[vi] > 0) phaseHold[vi]--;
            bool usePhase   = (phaseHold[vi] > 0);
            const bool symMode = symAct[vi];

            // The right channel is computed separately only when something differs.
            // Only the saturator needs catching up when that resumes.
            bool anyStereo = stereoCore || stereoOsc || stereoWgt || usePhase;
            if (stereoCore && !corePrev[vi]) satR[vi] = satL[vi];
            corePrev[vi] = stereoCore;


            // --- Partial multipliers, every sample: they set B's partial pitches ---
            float* ratioMulL = ccMul[vi][0];
            float* ratioMulR = ccMul[vi][1];
            const simd::float_4 harmonic0(1.f, 2.f, 3.f, 4.f), harmonic1(5.f, 6.f, 7.f, 8.f);
            const simd::float_4 stretch0 = simd::float_4::load(&kSpreadOffset[0]);
            const simd::float_4 stretch1 = simd::float_4::load(&kSpreadOffset[4]);
            simd::float_4 mul0  = harmonic0 + simd::float_4(spreadL) * stretch0;
            simd::float_4 mul1  = harmonic1 + simd::float_4(spreadL) * stretch1;
            simd::float_4 mul0R = stereoOsc ? harmonic0 + simd::float_4(spreadR) * stretch0 : mul0;
            simd::float_4 mul1R = stereoOsc ? harmonic1 + simd::float_4(spreadR) * stretch1 : mul1;
            mul0.store(&ratioMulL[0]);  mul1.store(&ratioMulL[4]);
            mul0R.store(&ratioMulR[0]); mul1R.store(&ratioMulR[4]);

            // --- Partial weights, rebuilt at a fixed control rate and ramped ---
            // The Nyquist fade is sized by the higher channel.
            float fFade = fmaxf(fabsf(fBL), fabsf(fBR));
            float* weightL   = ccWeight[vi][0];
            float* weightR   = ccWeight[vi][1];

            const float brightInv = 1.f / clamp(brightHz, kBrightMin, kBrightMax);
            auto buildWeights = [&](float morphPos, const float* mul,
                                    float* wOut, float& normOut, float& nzOut) {
                int   lo = (int)morphPos;
                if (lo > kMorphStops - 2) lo = kMorphStops - 2;
                float m  = morphPos - (float)lo;
                // Four partials at a time.  Under FM this runs every sample.
                simd::float_4 sumAbs4(0.f), sumSq4(0.f);
                for (int half = 0; half < kPartials; half += 4) {
                    simd::float_4 rowLo = simd::float_4::load(&kMorphTable[lo][half]);
                    simd::float_4 rowHi = simd::float_4::load(&kMorphTable[lo + 1][half]);
                    simd::float_4 w = rowLo + simd::float_4(m) * (rowHi - rowLo);
                    // Fade over the top quarter octave so pitch sweeps don't click.
                    simd::float_4 f = simd::float_4::load(&mul[half]) * simd::float_4(fFade);
                    w *= simd::clamp((simd::float_4(nyq) - f) * simd::float_4(fadeSpan), 0.f, 1.f);

                    // Normalizer taken before the brightness rolloff.
                    sumAbs4 += simd::fabs(w);
                    sumSq4  += w * w;

                    // Two pole rolloff in absolute frequency.  See kBrightDef.
                    simd::float_4 x = f * simd::float_4(brightInv);
                    (w / (simd::float_4(1.f) + x * x)).store(&wOut[half]);
                }
                float sumAbs = (sumAbs4[0] + sumAbs4[1]) + (sumAbs4[2] + sumAbs4[3]);
                float sumSq  = (sumSq4[0]  + sumSq4[1])  + (sumSq4[2]  + sumSq4[3]);
                float peakEst = kPeakBlendRms * sqrtf(sumSq) + (1.f - kPeakBlendRms) * sumAbs;
                normOut = (peakEst > 1e-6f) ? (1.f / peakEst) : 0.f;
                nzOut   = kMorphTable[lo][kPartials]
                        + m * (kMorphTable[lo + 1][kPartials] - kMorphTable[lo][kPartials]);
            };

            // Rebuilt every kWeightDiv samples whatever the controls are doing, so
            // the cost is the same with a still knob, pot noise or modulation.
            // In between, each value ramps linearly and lands exactly on its
            // target, so still controls give exactly the values a rebuild would.
            // A fresh voice snaps to its first targets; later rebuilds are
            // staggered across voices.  Through-zero FM moves the Nyquist fade
            // and brightness at audio rate, so while it is engaged the weights
            // are rebuilt every sample and follow it exactly.
            if (!ccValid[vi] || weightRampLeft[vi] <= 0 || fmOn) {
                float* targetL = ccWeightTarget[vi][0];
                float* targetR = ccWeightTarget[vi][1];
                buildWeights(morphL, ratioMulL, targetL, ccNormTarget[vi][0], ccNoiseTarget[vi][0]);
                if (stereoOsc || stereoWgt)
                    buildWeights(morphR, ratioMulR, targetR, ccNormTarget[vi][1], ccNoiseTarget[vi][1]);
                else {
                    for (int n = 0; n < kPartials; n++) targetR[n] = targetL[n];
                    ccNormTarget[vi][1]  = ccNormTarget[vi][0];
                    ccNoiseTarget[vi][1] = ccNoiseTarget[vi][0];
                }

                // Noise mip level for the same pitch, ramped with the rest.  Under
                // FM it is only needed while noise is in the mix.
                if (!fmOn || ccNoiseTarget[vi][0] > 1e-4f || ccNoiseTarget[vi][1] > 1e-4f) {
                    float budget = nyq * (float)kNoiseRepeat / fmaxf(fFade, 0.001f);
                    float level  = log2f(fmaxf((float)kLutHarm[0] / fmaxf(budget, 1e-3f), 1.f));
                    ccLutLevelTarget[vi] = clamp(level, 0.f, (float)(kLutLevels - 1));
                }

                if (!ccValid[vi] || fmOn) {
                    // Snap: a fresh voice, or FM.  Steps are cleared so a ramp
                    // that follows starts from rest.
                    for (int side = 0; side < 2; side++) {
                        for (int n = 0; n < kPartials; n++) {
                            ccWeight[vi][side][n]     = ccWeightTarget[vi][side][n];
                            ccWeightStep[vi][side][n] = 0.f;
                        }
                        ccNorm[vi][side]  = ccNormTarget[vi][side];  ccNormStep[vi][side]  = 0.f;
                        ccNoise[vi][side] = ccNoiseTarget[vi][side]; ccNoiseStep[vi][side] = 0.f;
                    }
                    ccLutLevel[vi] = ccLutLevelTarget[vi];
                    ccLutLevelStep[vi] = 0.f;
                    // Fresh voices are staggered so they do not all rebuild on the
                    // same sample; under FM the next sample rebuilds anyway.
                    weightRampLeft[vi] = fmOn ? 0 : 1 + (vi % kWeightDiv);
                    ccValid[vi] = true;
                }
                else {
                    const float rampInv = 1.f / (float)kWeightDiv;
                    for (int side = 0; side < 2; side++) {
                        for (int n = 0; n < kPartials; n++)
                            ccWeightStep[vi][side][n] =
                                (ccWeightTarget[vi][side][n] - ccWeight[vi][side][n]) * rampInv;
                        ccNormStep[vi][side]  = (ccNormTarget[vi][side]  - ccNorm[vi][side])  * rampInv;
                        ccNoiseStep[vi][side] = (ccNoiseTarget[vi][side] - ccNoise[vi][side]) * rampInv;
                    }
                    ccLutLevelStep[vi] = (ccLutLevelTarget[vi] - ccLutLevel[vi]) * rampInv;
                    weightRampLeft[vi] = kWeightDiv;
                }
            }

            // One ramp step per sample; the last step lands exactly on the target.
            if (weightRampLeft[vi] > 0) {
                if (--weightRampLeft[vi] > 0) {
                    for (int side = 0; side < 2; side++) {
                        for (int n = 0; n < kPartials; n++)
                            ccWeight[vi][side][n] += ccWeightStep[vi][side][n];
                        ccNorm[vi][side]  += ccNormStep[vi][side];
                        ccNoise[vi][side] += ccNoiseStep[vi][side];
                    }
                    ccLutLevel[vi] += ccLutLevelStep[vi];
                }
                else {
                    for (int side = 0; side < 2; side++) {
                        for (int n = 0; n < kPartials; n++)
                            ccWeight[vi][side][n] = ccWeightTarget[vi][side][n];
                        ccNorm[vi][side]  = ccNormTarget[vi][side];
                        ccNoise[vi][side] = ccNoiseTarget[vi][side];
                    }
                    ccLutLevel[vi] = ccLutLevelTarget[vi];
                }
            }

            float bNormL = ccNorm[vi][0],  bNormR = ccNorm[vi][1];
            float noiseMixL = ccNoise[vi][0], noiseMixR = ccNoise[vi][1];

            // --- Shape setups, cached on the shape value (the saturator setup divides) ---
            if (cacheFresh || shapeL != ccShapeIn[vi][0]) {
                ccShapeIn[vi][0] = shapeL;
                ccShape[vi][0].set(shapeL);
            }
            if (stereoCore && (cacheFresh || shapeR != ccShapeIn[vi][1])) {
                ccShapeIn[vi][1] = shapeR;
                ccShape[vi][1].set(shapeR);
            }
            const MalaShapeSetup& setupL = ccShape[vi][0];
            const MalaShapeSetup& setupR = stereoCore ? ccShape[vi][1] : ccShape[vi][0];

            // HEAT maps linearly from the shaper threshold to kDriveMax.
            float driveL = kThresh + heatL * heatToDriv;
            float driveR = stereoCore ? (kThresh + heatR * heatToDriv) : driveL;

            // Skip B when neither window can open and B is not patched out.
            bool needB = (driveL > kThresh) || (driveR > kThresh)
                       || bPatched;

            // --- LUT mip level, ramped with the weights; read only when noise is in the mix ---
            bool  needNoise = (noiseMixL > 1e-4f || noiseMixR > 1e-4f);
            float lutLevelF = needNoise ? ccLutLevel[vi] : 0.f;
            int   lutLo = (int)lutLevelF;
            if (lutLo > kLutLevels - 2) lutLo = kLutLevels - 2;
            float lutM  = lutLevelF - (float)lutLo;

            // --- Increments ---
            double incAL = (double)f0L * incScale;
            double incAR = (double)f0R * incScale;
            const float noiseRate = 1.f / (float)kNoiseRepeat;
            float lutIncL = fBL * noiseRate * incScale;
            float lutIncR = fBR * noiseRate * incScale;
            simd::float_4 inc0  = mul0 * simd::float_4(fBL) * simd::float_4(incScale);
            simd::float_4 inc1  = mul1 * simd::float_4(fBL) * simd::float_4(incScale);
            simd::float_4 inc0R = stereoOsc ? mul0R * simd::float_4(fBR) * simd::float_4(incScale) : inc0;
            simd::float_4 inc1R = stereoOsc ? mul1R * simd::float_4(fBR) * simd::float_4(incScale) : inc1;

            // Rotation coefficients: every 32 samples, staggered by voice, and at once
            // when B PHASE first engages.
            const bool wasOn = phaseWasOn[vi];
            bool phaseRefresh = usePhase
                              && (!wasOn || ((phaseDivCounter + vi) & 31) == 0);
            phaseWasOn[vi] = usePhase;
            if (phaseRefresh) {
                float offset = phaseAmt * kPhaseRange;
                for (int n = 0; n < kPartials; n++) {
                    // libm on purpose: the pairs must stay true rotations (s^2 + c^2 = 1).
                    float target = kTwoPi * ratioMulL[n] * offset;
                    if (!wasOn) phaseAngle[vi][n] = target;
                    // Restart exactly from where the last glide ended, then step to the target.
                    phaseCosD[vi][n] = cosf(phaseAngle[vi][n]);
                    phaseSinD[vi][n] = sinf(phaseAngle[vi][n]);
                    float step = (target - phaseAngle[vi][n]) * (1.f / 32.f);
                    phaseStepCos[vi][n] = cosf(step);
                    phaseStepSin[vi][n] = sinf(step);
                    phaseAngle[vi][n] = target;
                }
            }

            simd::float_4 cosD0 = simd::float_4::load(&phaseCosD[vi][0]);
            simd::float_4 cosD1 = simd::float_4::load(&phaseCosD[vi][4]);
            simd::float_4 sinD0 = simd::float_4::load(&phaseSinD[vi][0]);
            simd::float_4 sinD1 = simd::float_4::load(&phaseSinD[vi][4]);

            simd::float_4 wgt0  = simd::float_4::load(&weightL[0]);
            simd::float_4 wgt1  = simd::float_4::load(&weightL[4]);
            simd::float_4 wgt0R = simd::float_4::load(&weightR[0]);
            simd::float_4 wgt1R = simd::float_4::load(&weightR[4]);

            simd::float_4 ph0  = simd::float_4::load(&partialPhase[vi][0]);
            simd::float_4 ph1  = simd::float_4::load(&partialPhase[vi][4]);
            simd::float_4 ph0R = simd::float_4::load(&partialPhaseR[vi][0]);
            simd::float_4 ph1R = simd::float_4::load(&partialPhaseR[vi][4]);

            float aOut = 0.f, bOut = 0.f;
            float wLast = 0.f, wLastR = 0.f;
            bool  voiceWrapped = false;

            // Per voice state lives in locals through the block and is written
            // back after it, so the compiler can keep it in registers.
            double phaseLeft  = phaseA[vi];
            double phaseRight = phaseAR[vi];
            float  lutPhaseLeft  = lutPhase[vi];
            float  lutPhaseRight = lutPhaseR[vi];
            MalaADAADrive satLeft  = satL[vi];
            MalaADAADrive satRight = satR[vi];

            // A window can only open once its drive passes the threshold.
            const bool windowL = driveL > kThresh;
            const bool windowR = driveR > kThresh;

            // ================= Oversampled block =================
            for (int sub = 0; sub < os; sub++) {

                // --- Advance A, one phasor per side when detuned ---
                // Wraps either way, since TZFM can run the phasors backwards.
                phaseLeft += incAL;
                bool wrappedL = false;
                if (phaseLeft >= 1.0)     { phaseLeft -= 1.0; wrappedL = true; voiceWrapped = true; }
                else if (phaseLeft < 0.0) { phaseLeft += 1.0; wrappedL = true; voiceWrapped = true; }

                bool wrappedR = wrappedL;
                double stepR = incAL;
                if (splitPitch) {
                    stepR = incAR;
                    if (realigning) {
                        // Constant bend; the last step takes exactly the remaining gap, recomputed
                        // here because the per-sample gap is a block stale.
                        double g = phaseLeft - phaseRight;
                        if (g >  0.5) g -= 1.0;
                        if (g < -0.5) g += 1.0;
                        double e = g - incAL;
                        double move = fabs(incAL) * kRealignBend;
                        if (e >  move) e =  move;
                        if (e < -move) e = -move;
                        stepR = incAL + e;
                    }
                    phaseRight += stepR;
                    wrappedR = false;
                    if (phaseRight >= 1.0) { phaseRight -= 1.0; wrappedR = true; }
                    else if (phaseRight < 0.0) { phaseRight += 1.0; wrappedR = true; }
                }
                else {
                    phaseRight = phaseLeft;
                }

                // --- Advance B ---
                // The noise phasors move under a quarter cycle per substep (fB / 32
                // stays below a quarter of the base rate), so one conditional wrap
                // gives exactly x - floor(x) without the floorf call.
                ph0 += inc0; ph0 -= malaFloor4(ph0);
                ph1 += inc1; ph1 -= malaFloor4(ph1);
                lutPhaseLeft += lutIncL;
                if (lutPhaseLeft >= 1.f) lutPhaseLeft -= 1.f;
                else if (lutPhaseLeft < 0.f) lutPhaseLeft += 1.f;
                if (stereoOsc) {
                    ph0R += inc0R; ph0R -= malaFloor4(ph0R);
                    ph1R += inc1R; ph1R -= malaFloor4(ph1R);
                    lutPhaseRight += lutIncR;
                    if (lutPhaseRight >= 1.f) lutPhaseRight -= 1.f;
                    else if (lutPhaseRight < 0.f) lutPhaseRight += 1.f;
                }
                else {
                    ph0R = ph0; ph1R = ph1;
                    lutPhaseRight = lutPhaseLeft;
                }

                // --- Sync: each side resets on its own A wrap, inside its dead zone ---
                if (syncOn) {
                    // frac: substeps since the crossing, counted from 0 going forward or
                    // from 1 going backward.
                    // Each partial restarts at its increment times frac, wrapped.
                    if (wrappedL) {
                        simd::float_4 frac((float)((incAL > 0.0 ? phaseLeft : phaseLeft - 1.0) / incAL));
                        ph0 = inc0 * frac; ph0 -= malaFloor4(ph0);
                        ph1 = inc1 * frac; ph1 -= malaFloor4(ph1);
                        if (!stereoOsc) { ph0R = ph0; ph1R = ph1; }
                    }
                    if (stereoOsc && wrappedR) {
                        simd::float_4 fracR((float)((stepR > 0.0 ? phaseRight : phaseRight - 1.0) / stepR));
                        ph0R = inc0R * fracR; ph0R -= malaFloor4(ph0R);
                        ph1R = inc1R * fracR; ph1R -= malaFloor4(ph1R);
                    }
                }

                // --- Osc A ---
                float s  = malaSin2pi(kTwoPi * (float)phaseLeft);
                float sR = splitPitch ? malaSin2pi(kTwoPi * (float)phaseRight) : s;

                // The saturator tracks its input even when unused, so it
                // picks up mid-cycle without a step.
                float shapedL;
                if (setupL.saturate)
                    shapedL = satLeft.process(s, setupL.satDrive, setupL.linearRestore) * setupL.satNorm;
                else {
                    satLeft.lastInput = s;
                    shapedL = setupL.bypass ? s
                            : (s * (1.f + setupL.thinK) / (1.f + setupL.thinK * fabsf(s)));
                }
                float shapedR = shapedL;
                if (stereoCore) {
                    if (setupR.saturate)
                        shapedR = satRight.process(sR, setupR.satDrive, setupR.linearRestore) * setupR.satNorm;
                    else {
                        satRight.lastInput = sR;
                        shapedR = setupR.bypass ? sR
                                : (sR * (1.f + setupR.thinK) / (1.f + setupR.thinK * fabsf(sR)));
                    }
                }

                // --- Window, zero around A's crossings ---
                float wL = 0.f;
                if (windowL) {
                    float xL = driveL * shapedL;
                    float tL = clamp((fabsf(xL) - kThresh) * kWindowScale, 0.f, 1.f);
                    wL = malaWindow(tL);
                }

                float wR = wL;
                if (stereoCore) {
                    wR = 0.f;
                    if (windowR) {
                        float xR = driveR * shapedR;
                        float tR = clamp((fabsf(xR) - kThresh) * kWindowScale, 0.f, 1.f);
                        wR = malaWindow(tR);
                    }
                }

                // --- Osc B, skipped per substep while both windows are shut ---
                float bL = 0.f, bR = 0.f;
                if (needB && (wL > 0.f || wR > 0.f || bPatched)) {
                    simd::float_4 sinA = malaSin2pi4(ph0 * simd::float_4(kTwoPi));
                    simd::float_4 sinB = malaSin2pi4(ph1 * simd::float_4(kTwoPi));
                    simd::float_4 sinAR = sinA, sinBR = sinB;
                    if (stereoOsc) {
                        sinAR = malaSin2pi4(ph0R * simd::float_4(kTwoPi));
                        sinBR = malaSin2pi4(ph1R * simd::float_4(kTwoPi));
                    }

                    // Phase offset as a rotation of the partial sum, not a second evaluation.
                    if (usePhase) {
                        simd::float_4 cosA = malaCos2pi4(ph0 * simd::float_4(kTwoPi));
                        simd::float_4 cosB = malaCos2pi4(ph1 * simd::float_4(kTwoPi));
                        simd::float_4 cosAR = cosA, cosBR = cosB;
                        if (stereoOsc) {
                            cosAR = malaCos2pi4(ph0R * simd::float_4(kTwoPi));
                            cosBR = malaCos2pi4(ph1R * simd::float_4(kTwoPi));
                        }
                        simd::float_4 rotAL = sinA  * cosD0 + cosA  * sinD0;
                        simd::float_4 rotBL = sinB  * cosD1 + cosB  * sinD1;
                        simd::float_4 rotAR = sinAR * cosD0 - cosAR * sinD0;
                        simd::float_4 rotBR = sinBR * cosD1 - cosBR * sinD1;
                        sinA = rotAL; sinB = rotBL; sinAR = rotAR; sinBR = rotBR;
                    }

                    simd::float_4 accL = sinA * wgt0 + sinB * wgt1;
                    bL = (accL[0] + accL[1] + accL[2] + accL[3]) * bNormL;
                    bR = bL;
                    if (anyStereo) {
                        simd::float_4 accR = sinAR * wgt0R + sinBR * wgt1R;
                        bR = (accR[0] + accR[1] + accR[2] + accR[3]) * bNormR;
                    }

                    // --- Noise ---
                    if (needNoise) {
                        float off = usePhase ? (phaseAmt * kPhaseRange) : 0.f;
                        float nL = lutRead(lutTab, lutLo, lutM, malaWrap01(lutPhaseLeft + off));
                        // Own phasor, so detune and ratio widen the texture too.
                        float nR = (stereoOsc || usePhase)
                                 ? lutRead(lutTab, lutLo, lutM, malaWrap01(lutPhaseRight - off))
                                 : nL;
                        bL = bL * (1.f - noiseMixL) + nL * noiseMixL;
                        bR = bR * (1.f - noiseMixR) + nR * noiseMixR;
                    }
                }  // needB

                // --- Graft ---
                // SYM off: both crests get +B (even harmonics, brighter).
                // SYM on: the tail follows A's sign (odd symmetry, hollower).
                float tailL = bL, tailR = bR;
                if (symMode) {
                    if (shapedL < 0.f) tailL = -tailL;
                    if (shapedR < 0.f) tailR = -tailR;
                }

                float mL = shapedL * (1.f - wL) + tailL * wL;
                float mR = shapedR * (1.f - wR) + tailR * wR;

                // --- Handed to the paired decimator after the voice loop ---
                decimIn[sub][vi * 2]     = mL;
                decimIn[sub][vi * 2 + 1] = mR;

                aOut  = shapedL;
                bOut  = bL;
                wLast = wL;
                wLastR = wR;
            }
            // ================= end oversampled block =================

            phaseA[vi]    = phaseLeft;
            phaseAR[vi]   = phaseRight;
            lutPhase[vi]  = lutPhaseLeft;
            lutPhaseR[vi] = lutPhaseRight;
            satL[vi] = satLeft;
            satR[vi] = satRight;

            ph0.store(&partialPhase[vi][0]);
            ph1.store(&partialPhase[vi][4]);
            ph0R.store(&partialPhaseR[vi][0]);
            ph1R.store(&partialPhaseR[vi][4]);

            // B phase glide: advance the rotation one step toward its target.
            if (usePhase) {
                simd::float_4 stepCos0 = simd::float_4::load(&phaseStepCos[vi][0]);
                simd::float_4 stepCos1 = simd::float_4::load(&phaseStepCos[vi][4]);
                simd::float_4 stepSin0 = simd::float_4::load(&phaseStepSin[vi][0]);
                simd::float_4 stepSin1 = simd::float_4::load(&phaseStepSin[vi][4]);
                (cosD0 * stepCos0 - sinD0 * stepSin0).store(&phaseCosD[vi][0]);
                (cosD1 * stepCos1 - sinD1 * stepSin1).store(&phaseCosD[vi][4]);
                (sinD0 * stepCos0 + cosD0 * stepSin0).store(&phaseSinD[vi][0]);
                (sinD1 * stepCos1 + cosD1 * stepSin1).store(&phaseSinD[vi][4]);
            }

            // Kept for the dead zone latch.
            lastWL[vi] = wLast;
            lastWR[vi] = wLastR;

            // DC blocker corner tracked to pitch; the filter itself runs per pair below.
            float dcCut = clamp(f0 * 0.1f, 0.5f, 20.f);
            dcPole[vi] = 1.f - kTwoPi * dcCut * args.sampleTime;

            outputs[A_OUTPUT].setVoltage(clamp(aOut * 5.f, -10.f, 10.f), vi);
            outputs[B_OUTPUT].setVoltage(clamp(bOut * 5.f, -10.f, 10.f), vi);
            outputs[MOD_OUTPUT].setVoltage(modLfo[vi] * 5.f, vi);

            // --- Voice 0 drives the displays and the capture scope ---
            if (vi == 0) {
                // The panel only redraws at the UI frame rate, so these are
                // refreshed on the button poll rather than every sample.
                if (pollButtons) {
                    displayShape    = shapeL;
                    displayRatio    = ratioLinL;
                    displayHeat     = heatL;
                    displayNoiseMix = noiseMixL;
                    for (int n = 0; n < kPartials; n++) {
                        displayWeight[n]   = weightL[n] * bNormL;
                        displayRatioMul[n] = ratioMulL[n];
                    }
                }
                scopeWrapped = voiceWrapped;
                scopeFreq    = f0;    // carrier, before FM
                scopeWindow  = wLast;
            }
        }

        // ---- Decimate and DC block, two voices per float_4 --------------------
        // An odd voice count leaves the last pair's upper lanes idle; they are
        // fed silence so their state stays finite.
        const int nPairs = (nVoices + 1) / 2;
        if (nVoices & 1) {
            for (int sub = 0; sub < os; sub++)
                decimIn[sub][nVoices * 2] = decimIn[sub][nVoices * 2 + 1] = 0.f;
            dcPole[nVoices] = dcPole[nVoices - 1];
        }
        float voiceOut[kMaxPoly * 2];
        for (int p = 0; p < nPairs; p++) {
            simd::float_4 dec = decim[p].processBlock(&decimIn[0][p * 4], kMaxPoly * 2, os);

            simd::float_4 pole(dcPole[p * 2], dcPole[p * 2], dcPole[p * 2 + 1], dcPole[p * 2 + 1]);
            simd::float_4 y = dec - simd::float_4::load(&dcIn[p * 4])
                            + pole * simd::float_4::load(&dcOut[p * 4]);
            dec.store(&dcIn[p * 4]);
            y.store(&dcOut[p * 4]);
            y.store(&voiceOut[p * 4]);
        }
        // An idle upper lane is not a voice: keep its DC state at rest, as a
        // dropped voice's would be.
        if (nVoices & 1)
            dcIn[nVoices * 2] = dcIn[nVoices * 2 + 1] = dcOut[nVoices * 2] = dcOut[nVoices * 2 + 1] = 0.f;

        for (int vi = 0; vi < nVoices; vi++) {
            float outLv = voiceOut[vi * 2];
            float outRv = voiceOut[vi * 2 + 1];

            // VOLUME only here, so the scope shows the signal before it.  A patched
            // CV is a 0-10 V VCA under the knob.
            float gain = vcaOn
                       ? volume * clamp(inputs[VOLUME_CV_INPUT].getPolyVoltage(vi) * 0.1f, 0.f, 1.f)
                       : volume;
            float vL = clamp(outLv * gain * 5.f * kOutGain, -10.f, 10.f);
            float vR = clamp(outRv * gain * 5.f * kOutGain, -10.f, 10.f);

            if (polyOutputs) {
                outputs[OUT_L_OUTPUT].setVoltage(vL, vi);
                outputs[OUT_R_OUTPUT].setVoltage(vR, vi);
            }
            else {
                mixL += vL;
                mixR += vR;
            }
        }

        // ---- Capture scope, voice 0 ------------------------------------------
        // Captures run wrap to wrap, so without FM the back buffer holds exactly
        // one cycle.  The span is sized by the carrier (V/Oct + FREQ, before
        // FM): through-zero FM can slow, stop or reverse the phasor, so sizing
        // by the instantaneous frequency could set a huge stride, and wraps that
        // jitter back and forth around zero would restart the capture before it
        // published anything.  Instead, a wrap less than half a carrier cycle
        // into a capture is ignored, a full buffer is published at once, and if
        // no wrap follows within a carrier period the next capture starts anyway.
        {
            float outLv = voiceOut[0], outRv = voiceOut[1];
            int per = (int)(args.sampleRate / scopeFreq);
            bool startCapture = false;
            if (capWriting) {
                if (scopeWrapped && capIndex >= capMinPoints) {
                    capCount[capWrite] = capIndex;
                    capRead  = capWrite;
                    capWrite ^= 1;
                    startCapture = true;
                }
            }
            else if (scopeWrapped || ++capIdle > per) {
                startCapture = true;
            }
            if (startCapture) {
                // Ceiling division, so one whole cycle always fits.
                capStride = 1 + (per > 1 ? (per - 1) / kCaptureN : 0);
                capMinPoints = std::max(2, (per / capStride) / 2);
                capIndex  = 0;
                capStrideCount = 1;
                capWriting = true;
            }
            if (capWriting && --capStrideCount <= 0) {
                capStrideCount = capStride;
                capL[capWrite][capIndex] = outLv;
                capR[capWrite][capIndex] = outRv;
                capW[capWrite][capIndex] = scopeWindow;
                if (++capIndex >= kCaptureN) {
                    // Full: publish now and wait for the next wrap (or the timeout).
                    capCount[capWrite] = capIndex;
                    capRead  = capWrite;
                    capWrite ^= 1;
                    capWriting = false;
                    capIdle = 0;
                }
            }

            // Oversampling changes apply at a phase wrap, after two matching requests.
            if (scopeWrapped && osStable >= 2 && osPending != osActive) {
                osActive = osPending;
                retuneDecimators();
            }
        }

        if (!polyOutputs) {
            // 1/sqrt(n) for n uncorrelated voices.
            float norm = 1.f / sqrtf((float)nVoices);
            outputs[OUT_L_OUTPUT].setChannels(1);
            outputs[OUT_R_OUTPUT].setChannels(1);
            outputs[OUT_L_OUTPUT].setVoltage(clamp(mixL * norm, -10.f, 10.f));
            outputs[OUT_R_OUTPUT].setVoltage(clamp(mixR * norm, -10.f, 10.f));
        }
        else {
            outputs[OUT_L_OUTPUT].setChannels(nVoices);
            outputs[OUT_R_OUTPUT].setChannels(nVoices);
        }
        outputs[A_OUTPUT].setChannels(nVoices);
        outputs[B_OUTPUT].setChannels(nVoices);
        outputs[MOD_OUTPUT].setChannels(nVoices);
    }

    // -------------------------------------------------------------------------
    // Linearly interpolated read, crossfaded between two mip levels.
    inline float lutRead(const float (*tab)[kLutSize], int levelLo, float levelM, float phase) {
        float pos = phase * (float)kLutSize;
        int   i0  = (int)pos;
        if (i0 >= kLutSize) i0 = kLutSize - 1;
        int   i1  = (i0 + 1) & (kLutSize - 1);
        float f   = pos - (float)i0;

        float a = tab[levelLo][i0]     + f * (tab[levelLo][i1]     - tab[levelLo][i0]);
        float b = tab[levelLo + 1][i0] + f * (tab[levelLo + 1][i1] - tab[levelLo + 1][i0]);
        return a + levelM * (b - a);
    }

    // Analytic evaluation of B for the display.
    float displayB(float phase) {
        float sum = 0.f;
        for (int n = 0; n < kPartials; n++)
            sum += displayWeight[n] * malaSin2pi(malaWrap01(displayRatioMul[n] * phase) * kTwoPi);
        return sum;
    }

    // Same curve the audio path uses, for the A display.
    static float shapeCurve(float s, float shape) {
        if (shape > 0.f) {
            float drive = fmaxf(shape * kSatMax, 1e-4f);
            return MalaADAADrive::curve(drive * s) / MalaADAADrive::curve(drive);
        }
        float k = fabsf(shape) * kShapeThin;
        return s * (1.f + k) / (1.f + k * fabsf(s));
    }
};


// =============================================================================
struct MalaWidget : ModuleWidget {

    // Matches the slider used across the rest of the set.
    struct MalaSliderBase : app::SvgSlider {
        MalaSliderBase() {
            setBackgroundSvg(Svg::load(asset::plugin(pluginInstance, "res/components/ShortSlider.svg")));
            setHandleSvg    (Svg::load(asset::plugin(pluginInstance, "res/components/ShortSliderHandle.svg")));
            setHandlePosCentered(math::Vec(10.f, 55.f), math::Vec(10.f, 10.f));
        }
    };
    template <typename TL = YellowLight>
    struct MalaSlider : LightSlider<MalaSliderBase, VCVSliderLight<TL>> { MalaSlider() {} };

    // -------------------------------------------------------------------------
    // Osc A, with the graft window shaded behind the trace.
    struct ShapeDisplay : TransparentWidget {
        Mala* module = nullptr;

        void draw(const DrawArgs& args) override {
            nvgBeginPath(args.vg);
            nvgRoundedRect(args.vg, 0, 0, box.size.x, box.size.y, 2.f);
            nvgFillColor(args.vg, nvgRGB(8, 8, 12));
            nvgFill(args.vg);
        }

        void drawLayer(const DrawArgs& args, int layer) override {
            if (layer != 1 || !module) { TransparentWidget::drawLayer(args, layer); return; }

            float shape = module->displayShape;
            // Mirrors the audio path's HEAT mapping.
            float drive = kThresh + clamp(module->displayHeat, 0.f, 10.f) * 0.1f
                        * (kDriveMax - kThresh);

            const int N = 128;
            const float pad = 2.f, w = box.size.x, h = box.size.y;
            const float midY = h * 0.5f;
            const float ampY = (h - 2.f * pad) * 0.5f;

            // The shaped curve, evaluated once for both the shading and the trace.
            float curveAt[N + 1];
            for (int k = 0; k <= N; k++)
                curveAt[k] = Mala::shapeCurve(malaSin2pi(kTwoPi * (float)k / N), shape);

            // Shaded window, alpha following w.  Slices are grouped into a few
            // shade levels so the window takes one fill per level instead of one
            // per slice.  24 levels is a 0.015 alpha step, below what shows.
            const int kShadeLevels = 24;
            int shadeLevel[N];
            for (int k = 0; k < N; k++) {
                float t  = clamp((fabsf(drive * curveAt[k]) - kThresh) / (kPi - kThresh), 0.f, 1.f);
                float wv = malaWindow(t);
                shadeLevel[k] = (wv < 0.01f) ? 0 : 1 + std::min((int)(wv * kShadeLevels), kShadeLevels - 1);
            }
            for (int level = 1; level <= kShadeLevels; level++) {
                bool anySlice = false;
                for (int k = 0; k < N; k++) {
                    if (shadeLevel[k] != level) continue;
                    if (!anySlice) { nvgBeginPath(args.vg); anySlice = true; }
                    float plotX = pad + (float)k / N * (w - 2.f * pad);
                    nvgRect(args.vg, plotX, pad, (w - 2.f * pad) / N + 0.7f, h - 2.f * pad);
                }
                if (!anySlice) continue;
                float wv = ((float)level - 0.5f) / kShadeLevels;
                nvgFillColor(args.vg, nvgRGBAf(1.00f, 0.32f, 0.10f, 0.35f * wv));
                nvgFill(args.vg);
            }

            nvgBeginPath(args.vg);
            for (int k = 0; k <= N; k++) {
                float phase = (float)k / N;
                float v = curveAt[k];
                float plotX = pad + phase * (w - 2.f * pad);
                float plotY = midY - clamp(v, -1.2f, 1.2f) * ampY;
                k == 0 ? nvgMoveTo(args.vg, plotX, plotY) : nvgLineTo(args.vg, plotX, plotY);
            }
            nvgStrokeColor(args.vg, nvgRGBAf(1.00f, 0.72f, 0.22f, 0.95f));
            nvgStrokeWidth(args.vg, 1.4f);
            nvgStroke(args.vg);

            TransparentWidget::drawLayer(args, layer);
        }
    };

    // -------------------------------------------------------------------------
    struct BDisplay : TransparentWidget {
        Mala* module = nullptr;

        void draw(const DrawArgs& args) override {
            nvgBeginPath(args.vg);
            nvgRoundedRect(args.vg, 0, 0, box.size.x, box.size.y, 2.f);
            nvgFillColor(args.vg, nvgRGB(8, 8, 12));
            nvgFill(args.vg);
        }

        void drawLayer(const DrawArgs& args, int layer) override {
            if (layer != 1 || !module) { TransparentWidget::drawLayer(args, layer); return; }

            const int N = 160;
            const float pad = 2.f, w = box.size.x, h = box.size.y;
            const float midY = h * 0.5f;
            const float ampY = (h - 2.f * pad) * 0.5f;

            nvgBeginPath(args.vg);
            for (int k = 0; k <= N; k++) {
                float phase = (float)k / N;
                float v = module->displayB(phase) * (1.f - module->displayNoiseMix);
                float plotX = pad + phase * (w - 2.f * pad);
                float plotY = midY - clamp(v, -1.2f, 1.2f) * ampY;
                k == 0 ? nvgMoveTo(args.vg, plotX, plotY) : nvgLineTo(args.vg, plotX, plotY);
            }
            nvgStrokeColor(args.vg, nvgRGBAf(1.00f, 0.34f, 0.24f, 0.95f));
            nvgStrokeWidth(args.vg, 1.4f);
            nvgStroke(args.vg);

            // Ratio readout.
            std::shared_ptr<Font> font =
                APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
            if (font) {
                char label[16];
                snprintf(label, sizeof(label), "%.2fx", module->displayRatio);
                nvgFontFaceId(args.vg, font->handle);
                nvgFontSize(args.vg, 8.5f);
                nvgTextAlign(args.vg, NVG_ALIGN_RIGHT | NVG_ALIGN_TOP);
                nvgFillColor(args.vg, nvgRGBAf(1.00f, 0.72f, 0.30f, 0.80f));
                nvgText(args.vg, w - pad - 1.f, pad + 0.5f, label, NULL);
            }

            // Noise content shown as a band, since a trace cannot represent it.
            float nz = module->displayNoiseMix;
            if (nz > 0.01f) {
                nvgBeginPath(args.vg);
                nvgRect(args.vg, pad, midY - ampY * nz, w - 2.f * pad, 2.f * ampY * nz);
                nvgFillColor(args.vg, nvgRGBAf(1.00f, 0.34f, 0.24f, 0.13f));
                nvgFill(args.vg);
            }

            TransparentWidget::drawLayer(args, layer);
        }
    };

    // -------------------------------------------------------------------------
    // Captured scope.  L and R overlaid so the stereo width is visible, with
    // the grafted segments in the warm hue.
    struct OutDisplay : TransparentWidget {
        Mala* module = nullptr;

        void draw(const DrawArgs& args) override {
            nvgBeginPath(args.vg);
            nvgRoundedRect(args.vg, 0, 0, box.size.x, box.size.y, 2.f);
            nvgFillColor(args.vg, nvgRGB(8, 8, 12));
            nvgFill(args.vg);
        }

        void drawLayer(const DrawArgs& args, int layer) override {
            if (layer != 1 || !module) { TransparentWidget::drawLayer(args, layer); return; }

            // Latch the index once so a buffer flip mid-draw cannot split the
            // trace across two cycles.
            int buf = module->capRead;
            int n   = module->capCount[buf];
            if (n < 2) { TransparentWidget::drawLayer(args, layer); return; }

            const float* bufL = module->capL[buf];
            const float* bufR = module->capR[buf];
            const float* bufW = module->capW[buf];

            const float pad = 2.f, w = box.size.x, h = box.size.y;
            const float midY = h * 0.5f;
            // Just under half height, leaving headroom for overshoot.
            const float ampY = (h - 2.f * pad) * 0.42f;

            drawTrace(args, bufR, n, w, pad, midY, ampY,
                      nvgRGBAf(0.30f, 0.62f, 1.00f, 0.55f));
            drawTrace(args, bufL, n, w, pad, midY, ampY,
                      nvgRGBAf(1.00f, 0.72f, 0.22f, 0.90f));

            // Overlay the grafted portions of L in the heat hue.
            nvgStrokeColor(args.vg, nvgRGBAf(1.00f, 0.30f, 0.12f, 0.95f));
            nvgStrokeWidth(args.vg, 1.6f);
            bool open = false;
            for (int k = 0; k < n; k++) {
                bool grafted = bufW[k] > 0.5f;
                float plotX = pad + (float)k / (float)(n - 1) * (w - 2.f * pad);
                float plotY = midY - clamp(bufL[k], -1.2f, 1.2f) * ampY;
                if (grafted && !open) { nvgBeginPath(args.vg); nvgMoveTo(args.vg, plotX, plotY); open = true; }
                else if (grafted)     { nvgLineTo(args.vg, plotX, plotY); }
                else if (open)        { nvgStroke(args.vg); open = false; }
            }
            if (open) nvgStroke(args.vg);

            TransparentWidget::drawLayer(args, layer);
        }

        static void drawTrace(const DrawArgs& args, const float* buf, int n,
                              float w, float pad, float midY, float ampY,
                              NVGcolor col) {
            nvgBeginPath(args.vg);
            for (int k = 0; k < n; k++) {
                float plotX = pad + (float)k / (float)(n - 1) * (w - 2.f * pad);
                float plotY = midY - clamp(buf[k], -1.2f, 1.2f) * ampY;
                k == 0 ? nvgMoveTo(args.vg, plotX, plotY) : nvgLineTo(args.vg, plotX, plotY);
            }
            nvgStrokeColor(args.vg, col);
            nvgStrokeWidth(args.vg, 1.4f);
            nvgStroke(args.vg);
        }
    };

    // -------------------------------------------------------------------------
    MalaWidget(Mala* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Mala.svg"),
            asset::plugin(pluginInstance, "res/Mala-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // ---- Displays --------------------------------------------------------
        // A over B on the left, output scope on the right, centred as a block.
        const float dTop = 11.f, dGap = 3.f;
        const float dNarrow = 25.3f, dWide = 48.6f;
        const float dLeft = (21.f * 5.08f - (dNarrow + dGap + dWide)) * 0.5f;

        auto* shapeDisp = createWidget<ShapeDisplay>(mm2px(Vec(dLeft, dTop)));
        shapeDisp->box.size = mm2px(Vec(dNarrow, 7.0f));
        shapeDisp->module = module;
        addChild(shapeDisp);

        auto* bDisp = createWidget<BDisplay>(mm2px(Vec(dLeft, dTop + 8.f)));
        bDisp->box.size = mm2px(Vec(dNarrow, 7.0f));
        bDisp->module = module;
        addChild(bDisp);

        auto* outDisp = createWidget<OutDisplay>(mm2px(Vec(dLeft + dNarrow + dGap, dTop)));
        outDisp->box.size = mm2px(Vec(dWide, 15.0f));
        outDisp->module = module;
        addChild(outDisp);

        // ---- Mod bus stacks: button + LED, control, trim, CV ------------------
        // HEAT rail and last slider column sit equally far from their edges; all
        // lower-section x positions derive from these columns.
        const float panelWidth = 21.f * 5.08f;
        const float xHeat      = 13.5f;                     // left rail
        const float xBank0     = 30.f;                      // first slider column
        const float bankPitch  = (panelWidth - xHeat - xBank0) / 5.f;   // ~12.64 mm
        const float colX[6] = { xBank0,                 xBank0 +     bankPitch,
                                xBank0 + 2 * bankPitch, xBank0 + 3 * bankPitch,
                                xBank0 + 4 * bankPitch, xBank0 + 5 * bankPitch };
        const float yLed = 33.f, ySlider = 48.f, ySlTrim = 59.5f, ySlCV = 67.5f;

        // Heat
        addParam(createParamCentered<TL1105>(mm2px(Vec(xHeat, yLed)), module, Mala::MAP_HEAT_PARAM));
        addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(xHeat, yLed)), module, Mala::LED_HEAT));
        addParam(createParamCentered<RoundBigBlackKnob>(mm2px(Vec(xHeat, 46.f)), module, Mala::HEAT_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(xHeat, ySlTrim)), module, Mala::HEAT_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(xHeat, ySlCV)), module, Mala::HEAT_CV_INPUT));

        // Shape
        addParam(createParamCentered<TL1105>(mm2px(Vec(colX[0], yLed)), module, Mala::MAP_SHAPE_PARAM));
        addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(colX[0], yLed)), module, Mala::LED_SHAPE));
        addParam(createParamCentered<MalaSlider<YellowLight>>(mm2px(Vec(colX[0], ySlider)), module, Mala::SHAPE_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(colX[0], ySlTrim)), module, Mala::SHAPE_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(colX[0], ySlCV)), module, Mala::SHAPE_CV_INPUT));

        // Ratio
        addParam(createParamCentered<TL1105>(mm2px(Vec(colX[1], yLed)), module, Mala::MAP_RATIO_PARAM));
        addChild(createLightCentered<MediumLight<BlueLight>>(mm2px(Vec(colX[1], yLed)), module, Mala::LED_RATIO));
        addParam(createParamCentered<MalaSlider<BlueLight>>(mm2px(Vec(colX[1], ySlider)), module, Mala::RATIO_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(colX[1], ySlTrim)), module, Mala::RATIO_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(colX[1], ySlCV)), module, Mala::RATIO_CV_INPUT));

        // Morph
        addParam(createParamCentered<TL1105>(mm2px(Vec(colX[2], yLed)), module, Mala::MAP_MORPH_PARAM));
        addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(colX[2], yLed)), module, Mala::LED_MORPH));
        addParam(createParamCentered<MalaSlider<RedLight>>(mm2px(Vec(colX[2], ySlider)), module, Mala::MORPH_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(colX[2], ySlTrim)), module, Mala::MORPH_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(colX[2], ySlCV)), module, Mala::MORPH_CV_INPUT));

        // Spread
        addParam(createParamCentered<TL1105>(mm2px(Vec(colX[3], yLed)), module, Mala::MAP_SPREAD_PARAM));
        addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(colX[3], yLed)), module, Mala::LED_SPREAD));
        addParam(createParamCentered<MalaSlider<GreenLight>>(mm2px(Vec(colX[3], ySlider)), module, Mala::SPREAD_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(colX[3], ySlTrim)), module, Mala::SPREAD_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(colX[3], ySlCV)), module, Mala::SPREAD_CV_INPUT));

        // Detune
        addParam(createParamCentered<TL1105>(mm2px(Vec(colX[4], yLed)), module, Mala::MAP_DETUNE_PARAM));
        addChild(createLightCentered<MediumLight<WhiteLight>>(mm2px(Vec(colX[4], yLed)), module, Mala::LED_DETUNE));
        addParam(createParamCentered<MalaSlider<WhiteLight>>(mm2px(Vec(colX[4], ySlider)), module, Mala::DETUNE_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(colX[4], ySlTrim)), module, Mala::DETUNE_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(colX[4], ySlCV)), module, Mala::DETUNE_CV_INPUT));

        // B Phase
        addParam(createParamCentered<TL1105>(mm2px(Vec(colX[5], yLed)), module, Mala::MAP_PHASE_PARAM));
        addChild(createLightCentered<MediumLight<BlueLight>>(mm2px(Vec(colX[5], yLed)), module, Mala::LED_PHASE));
        addParam(createParamCentered<MalaSlider<BlueLight>>(mm2px(Vec(colX[5], ySlider)), module, Mala::PHASE_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(colX[5], ySlTrim)), module, Mala::PHASE_TRIM_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(colX[5], ySlCV)), module, Mala::PHASE_CV_INPUT));

        // ---- Lower section ---------------------------------------------------
        // Each port or group keeps ~3.2 mm below it for a 7pt label.
        const float yOut = 115.f;

        // Left rail: FM index over its jack, then FREQ over V/Oct.
        addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(xHeat, 79.0f)),  module, Mala::FM_AMT_PARAM));
        addInput (createInputCentered<ThemedPJ301MPort>  (mm2px(Vec(xHeat, 87.5f)),  module, Mala::FM_INPUT));
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(xHeat, 102.5f)), module, Mala::FREQ_PARAM));

        // Mode buttons at slider positions 1, 2.5 and 4, lights on the buttons.
        const float yBtn   = 80.f;
        const float xQuant = xBank0 + 1.0f * bankPitch;   // under RATIO
        const float xSync  = xBank0 + 2.5f * bankPitch;   // centre of the bank
        const float xSym   = xBank0 + 4.0f * bankPitch;   // under DETUNE

        addParam(createParamCentered<TL1105>(mm2px(Vec(xQuant, yBtn)), module, Mala::QUANT_PARAM));
        addChild(createLightCentered<MediumLight<BlueLight>>(mm2px(Vec(xQuant, yBtn)), module, Mala::LED_QUANT));

        addParam(createParamCentered<TL1105>(mm2px(Vec(xSync, yBtn)), module, Mala::SYNC_PARAM));
        addChild(createLightCentered<LargeLight<GreenLight>>(mm2px(Vec(xSync, yBtn)), module, Mala::LED_SYNC));

        addParam(createParamCentered<TL1105>(mm2px(Vec(xSym, yBtn)), module, Mala::SYM_PARAM));
        addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(xSym, yBtn)), module, Mala::LED_SYM));

        // MALA bus as a shallow V centred on SYNC: CV, trim and knob on one
        // diagonal per side, with equal clear space between their edges.
        const float busGap   = 3.3f;                   // edge-to-edge spacing along each arm
        const float yBusCV   = 88.f, yBusKnob = 95.f;
        const float xMaKnob  = xSync - 0.75f * bankPitch;
        const float xLaKnob  = xSync + 0.75f * bankPitch;
        const float armLen   = (4.3f + 2.6f) + (2.6f + 7.5f) + 2.f * busGap;   // CV centre to knob centre
        const float armDrop  = yBusKnob - yBusCV;
        const float armDx    = sqrtf(armLen * armLen - armDrop * armDrop);
        const float trimFrac = (2.6f + 7.5f + busGap) / armLen;               // knob -> trim along the arm
        const float yBusTrim = yBusKnob - trimFrac * armDrop;
        const float xMaCV    = xMaKnob - armDx,            xLaCV   = xLaKnob + armDx;
        const float xMaTrim  = xMaKnob - trimFrac * armDx, xLaTrim = xLaKnob + trimFrac * armDx;

        addInput (createInputCentered<ThemedPJ301MPort>(mm2px(Vec(xMaCV,   yBusCV)),   module, Mala::MODRATE_CV_INPUT));
        addParam(createParamCentered<Trimpot>          (mm2px(Vec(xMaTrim, yBusTrim)), module, Mala::MODRATE_TRIM_PARAM));
        addParam(createParamCentered<RoundBigBlackKnob>(mm2px(Vec(xMaKnob, yBusKnob)), module, Mala::MODRATE_PARAM));

        addParam(createParamCentered<RoundBigBlackKnob>(mm2px(Vec(xLaKnob, yBusKnob)), module, Mala::MODDEPTH_PARAM));
        addParam(createParamCentered<Trimpot>          (mm2px(Vec(xLaTrim, yBusTrim)), module, Mala::MODDEPTH_TRIM_PARAM));
        addInput (createInputCentered<ThemedPJ301MPort>(mm2px(Vec(xLaCV,   yBusCV)),   module, Mala::MODDEPTH_CV_INPUT));

        // ---- Bottom row ------------------------------------------------------
        // V/Oct under FREQ; MALA/A/B packed from the first slider column; VCA CV
        // then VOLUME; L stacked over R on the last slider column.
        const float outPitch = 10.5f;               // MALA/A/B spacing
        const float xVolCV   = 67.f;
        const float xVolume  = xVolCV + 11.5f;      // jack edge to knob edge ~2.2 mm
        const float yOutR    = 117.f;
        const float yOutL    = yOutR - 9.5f;        // stacked jack pitch

        addInput (createInputCentered<ThemedPJ301MPort> (mm2px(Vec(xHeat,   yOut)), module, Mala::VOCT_INPUT));

        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(colX[0],                 yOut)), module, Mala::MOD_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(colX[0] +     outPitch, yOut)), module, Mala::A_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(colX[0] + 2 * outPitch, yOut)), module, Mala::B_OUTPUT));

        addInput (createInputCentered<ThemedPJ301MPort> (mm2px(Vec(xVolCV,  yOut)), module, Mala::VOLUME_CV_INPUT));
        addParam (createParamCentered<RoundBlackKnob>   (mm2px(Vec(xVolume, yOut)), module, Mala::VOLUME_PARAM));

        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(colX[5], yOutL)), module, Mala::OUT_L_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(colX[5], yOutR)), module, Mala::OUT_R_OUTPUT));
    }

    // =========================================================================
    void step() override {
        Mala* m = dynamic_cast<Mala*>(module);
        if (m) {
            m->lights[Mala::LED_SHAPE ].setBrightness((m->modTarget & Mala::M_SHAPE ) ? 1.f : 0.f);
            m->lights[Mala::LED_RATIO ].setBrightness((m->modTarget & Mala::M_RATIO ) ? 1.f : 0.f);
            m->lights[Mala::LED_MORPH ].setBrightness((m->modTarget & Mala::M_MORPH ) ? 1.f : 0.f);
            m->lights[Mala::LED_SPREAD].setBrightness((m->modTarget & Mala::M_SPREAD) ? 1.f : 0.f);
            m->lights[Mala::LED_DETUNE].setBrightness((m->modTarget & Mala::M_DETUNE) ? 1.f : 0.f);
            m->lights[Mala::LED_PHASE ].setBrightness((m->modTarget & Mala::M_PHASE ) ? 1.f : 0.f);
            m->lights[Mala::LED_HEAT  ].setBrightness((m->modTarget & Mala::M_HEAT  ) ? 1.f : 0.f);
            m->lights[Mala::LED_SYNC  ].setBrightness(m->syncOn ? 1.f : 0.f);
            m->lights[Mala::LED_QUANT ].setBrightness(m->quantOn ? 1.f : 0.f);
            // Dim while a SYM change waits for a dead zone.
            m->lights[Mala::LED_SYM   ].setBrightness(
                m->symOn ? (m->symAct[0] == m->symOn ? 1.f : 0.35f) : 0.f);
        }
        ModuleWidget::step();
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
        Mala* m = dynamic_cast<Mala*>(module);
        if (!m) return;

        menu->addChild(new MenuSeparator());

        menu->addChild(createSubmenuItem("Supersampling", "", [m](Menu* sub) {
            struct OsItem : MenuItem {
                Mala* module; int factor;
                void onAction(const event::Action&) override {
                    module->osSetting = factor;
                    if (factor != 0) { module->osActive = factor; module->retuneDecimators(); }
                }
                void step() override {
                    rightText = (module->osSetting == factor) ? CHECKMARK_STRING : "";
                    MenuItem::step();
                }
            };
            const int   factors[4] = { 2, 4, 8, 0 };
            const char* labels[4]  = { "2x", "4x (default)", "8x", "Auto" };
            for (int i = 0; i < 4; i++) {
                auto* it = new OsItem();
                it->text = labels[i]; it->module = m; it->factor = factors[i];
                sub->addChild(it);
            }
        }));

        menu->addChild(new MenuSeparator());

        menu->addChild(createMenuItem("Polyphonic outputs (default)", CHECKMARK(m->polyOutputs),
            [m]() { m->polyOutputs = !m->polyOutputs; }));

        menu->addChild(createMenuItem("Wide detune (240 cents)", CHECKMARK(m->detuneWide),
            [m]() { m->detuneWide = !m->detuneWide; }));

        menu->addChild(new MenuSeparator());

        // B brightness knee.
        struct BrightQuantity : Quantity {
            Mala* module;
            BrightQuantity(Mala* m) : module(m) {}
            void setValue(float v) override {
                if (module) module->setBright(std::pow(2.f, clamp(v, getMinValue(), getMaxValue())));
            }
            float getValue() override {
                return module ? std::log2(module->brightHz) : std::log2(kBrightDef);
            }
            float getDefaultValue() override { return std::log2(kBrightDef); }
            float getMinValue() override { return std::log2(kBrightMin); }
            float getMaxValue() override { return std::log2(kBrightMax); }
            std::string getLabel() override { return "B brightness"; }
            std::string getUnit() override { return " Hz"; }
            std::string getDisplayValueString() override {
                return string::f("%.0f", module ? module->brightHz : kBrightDef);
            }
        };
        struct OwnedSliderB : ui::Slider {
            ~OwnedSliderB() { delete quantity; quantity = nullptr; }
        };
        auto* brightSlider = new OwnedSliderB();
        brightSlider->quantity = new BrightQuantity(m);
        brightSlider->box.size.x = 200.f;
        menu->addChild(brightSlider);

        // Texture seed; rebuilds into an idle buffer on the UI thread.
        struct SeedQuantity : Quantity {
            Mala* module;
            SeedQuantity(Mala* m) : module(m) {}
            void setValue(float v) override {
                if (module) module->setNoiseSeed((int)std::round(clamp(v, 0.f, (float)(kNoiseSeeds - 1))));
            }
            float getValue() override { return module ? (float)module->lutSeed : 0.f; }
            float getDefaultValue() override { return 0.f; }
            float getMinValue() override { return 0.f; }
            float getMaxValue() override { return (float)(kNoiseSeeds - 1); }
            int getDisplayPrecision() override { return 0; }
            std::string getLabel() override { return "Texture seed"; }
            std::string getDisplayValueString() override {
                return string::f("%d", module ? module->lutSeed : 0);
            }
        };
        // ui::Slider does not delete its quantity; this subclass does.
        struct OwnedSlider : ui::Slider {
            ~OwnedSlider() { delete quantity; quantity = nullptr; }
        };
        auto* seedSlider = new OwnedSlider();
        seedSlider->quantity = new SeedQuantity(m);
        seedSlider->box.size.x = 200.f;
        menu->addChild(seedSlider);

        menu->addChild(new MenuSeparator());

        menu->addChild(createSubmenuItem("Mala wave", "", [m](Menu* sub) {
            struct LfoShapeItem : MenuItem {
                Mala* module; int shape;
                void onAction(const event::Action&) override { module->lfoShape = shape; }
                void step() override {
                    rightText = (module->lfoShape == shape) ? CHECKMARK_STRING : "";
                    MenuItem::step();
                }
            };
            const char* labels[3] = { "Sine (default)", "Triangle", "Slow random" };
            for (int i = 0; i < 3; i++) {
                auto* it = new LfoShapeItem();
                it->text = labels[i]; it->module = m; it->shape = i;
                sub->addChild(it);
            }
        }));

        menu->addChild(createMenuItem("Ma CV overrides Mala (depth by La)", CHECKMARK(m->extOverride),
            [m]() { m->setExtOverride(!m->extOverride); }));

        menu->addChild(createSubmenuItem("Mala voice phase", "", [m](Menu* sub) {
            struct LfoModeItem : MenuItem {
                Mala* module; int mode;
                void onAction(const event::Action&) override { module->lfoPhaseMode = mode; }
                void step() override {
                    rightText = (module->lfoPhaseMode == mode) ? CHECKMARK_STRING : "";
                    MenuItem::step();
                }
            };
            const char* labels[2] = { "Spread (default)", "Unison" };
            for (int i = 0; i < 2; i++) {
                auto* it = new LfoModeItem();
                it->text = labels[i]; it->module = m; it->mode = i;
                sub->addChild(it);
            }
        }));
    }
};

Model* modelMala = createModel<Mala, MalaWidget>("Mala");