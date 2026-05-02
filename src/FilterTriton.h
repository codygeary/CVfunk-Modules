#pragma once
#include <assert.h>
#include <cmath>
#include "rack.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// BiquadCoeffs
// ─────────────────────────────────────────────────────────────────────────────
struct BiquadCoeffs {
    float b0 = 1.f, b1 = 0.f, b2 = 0.f;
    float a1 = 0.f, a2 = 0.f;

    float magnitude(float fn) const {
        float w   = 2.f * float(M_PI) * fn;
        float cw  = cosf(w), sw = sinf(w);
        float numR = b0 + b1*cw  + b2*cosf(2.f*w);
        float numI =    - b1*sw  - b2*sinf(2.f*w);
        float denR = 1.f + a1*cw  + a2*cosf(2.f*w);
        float denI =     - a1*sw  - a2*sinf(2.f*w);
        float d2   = denR*denR + denI*denI;
        if (d2 < 1e-30f) return 0.f;
        return sqrtf((numR*numR + numI*numI) / d2);
    }
};

static inline BiquadCoeffs makeLowpass(float fn, float Q) {
    float w0 = 2.f*float(M_PI)*fn, alpha = sinf(w0)/(2.f*Q), c0 = cosf(w0);
    float ai = 1.f/(1.f+alpha);
    BiquadCoeffs c;
    c.b0=(1.f-c0)*0.5f*ai; c.b1=(1.f-c0)*ai; c.b2=c.b0;
    c.a1=-2.f*c0*ai; c.a2=(1.f-alpha)*ai;
    return c;
}
static inline BiquadCoeffs makeHighpass(float fn, float Q) {
    float w0 = 2.f*float(M_PI)*fn, alpha = sinf(w0)/(2.f*Q), c0 = cosf(w0);
    float ai = 1.f/(1.f+alpha);
    BiquadCoeffs c;
    c.b0=(1.f+c0)*0.5f*ai; c.b1=-(1.f+c0)*ai; c.b2=c.b0;
    c.a1=-2.f*c0*ai; c.a2=(1.f-alpha)*ai;
    return c;
}

// Number of biquad stages — 8 gives −96dB/oct at maximum sharpness
static const int TRITON_STAGES = 8;

// Magnitude with sharpness blend across all 8 stages.
// sharpness 0..1 sweeps from 1 stage (−12dB/oct) to 8 stages (−96dB/oct).
static inline float cascadeMagSharp(const BiquadCoeffs s[TRITON_STAGES], float fn, float sharpness) {
    float m[TRITON_STAGES];
    m[0] = s[0].magnitude(fn);
    for (int k = 1; k < TRITON_STAGES; k++)
        m[k] = m[k-1] * s[k].magnitude(fn);

    float idx  = sharpness * (TRITON_STAGES - 1);
    int   lo   = (int)idx;
    float frac = idx - lo;
    if (lo >= TRITON_STAGES - 1) return m[TRITON_STAGES - 1];
    return m[lo] + frac * (m[lo+1] - m[lo]);
}

// ─────────────────────────────────────────────────────────────────────────────
// DCBlocker / NyquistCap
// ─────────────────────────────────────────────────────────────────────────────
struct DCBlocker {
    float x1=0.f, y1=0.f, R=0.9995f;
    void setSampleRate(float sr) {
        R = rack::clamp(1.f - 2.f*float(M_PI)*20.f/sr, 0.990f, 0.9999f);
    }
    float process(float x) { y1=x-x1+R*y1; x1=x; return y1; }
    void reset() { x1=0.f; y1=0.f; }
};

struct NyquistCap {
    float z=0.f, coeff=0.8f;
    void setSampleRate(float) {
        coeff = rack::clamp(1.f-expf(-2.f*float(M_PI)*0.225f), 0.5f, 0.99f);
    }
    float process(float x) { z+=coeff*(x-z); return z; }
    void reset() { z=0.f; }
};

// ─────────────────────────────────────────────────────────────────────────────
// FilterTriton — 8-stage cascaded biquad, LP or HP
//
// sharpness 0..1 : blends output from stage 0 (−12dB/oct) to stage 7 (−96dB/oct)
//                  All 8 stages always run. Q values are Butterworth throughout.
//
// resonance 0..1 : Q of the final stage only (stage 7), decoupled from sharpness.
//                  0 = Butterworth Q, 1 = Q=80 (self-oscillation territory)
//                  All earlier stages remain Butterworth for stability.
// ─────────────────────────────────────────────────────────────────────────────
class FilterTriton {
public:
    enum Mode { LOWPASS, HIGHPASS };
    void setMode(Mode m) { mode = m; }

    void setParameters(float normalizedCutoff, float sharpness, float resonance) {
        assert(normalizedCutoff > 0.f && normalizedCutoff < 0.5f);
        sharpness = rack::clamp(sharpness, 0.f, 1.f);
        resonance = rack::clamp(resonance, 0.f, 1.f);
        sharp = sharpness;

        // Butterworth Q values for a 16-pole (8 biquad) cascade.
        // Formula: Q_k = 1 / (2 * cos((2k-1) * pi / (2*N)))  for k=1..N/2
        // where N=16 (total poles), k=1..8
        // Computed: cos((2k-1)*pi/32) for k=1..8
        const float qButter[TRITON_STAGES] = {
            0.50979558f,  // k=1: cos(pi/32)
            0.53104260f,  // k=2: cos(3pi/32)
            0.56672739f,  // k=3: cos(5pi/32)
            0.62689136f,  // k=4: cos(7pi/32)
            0.72537555f,  // k=5: cos(9pi/32)
            0.90100653f,  // k=6: cos(11pi/32)
            1.24722195f,  // k=7: cos(13pi/32)
            2.56291556f,  // k=8: cos(15pi/32)  — highest Q stage
        };

        auto type = (mode == LOWPASS)
                  ? rack::dsp::TBiquadFilter<float>::LOWPASS
                  : rack::dsp::TBiquadFilter<float>::HIGHPASS;

        auto makeFn = (mode == LOWPASS) ? makeLowpass : makeHighpass;

        for (int k = 0; k < TRITON_STAGES; k++) {
            // Last stage gets resonance Q override
            float q = (k == TRITON_STAGES - 1)
                    ? qButter[k] + resonance * (200.f - qButter[k])
                    : qButter[k];
            f[k].setParameters(type, normalizedCutoff, q, 1.f);
            coeffs[k] = makeFn(normalizedCutoff, q);
        }
    }

    float process(float x) {
        // All stages always run so the resonant final stage can oscillate
        float s[TRITON_STAGES];
        s[0] = f[0].process(x);
        for (int k = 1; k < TRITON_STAGES; k++)
            s[k] = f[k].process(s[k-1]);

        // Blend between adjacent stage outputs based on sharpness
        float idx  = sharp * (TRITON_STAGES - 1);
        int   lo   = (int)idx;
        float frac = idx - lo;
        if (lo >= TRITON_STAGES - 1) return s[TRITON_STAGES - 1];
        return s[lo] + frac * (s[lo+1] - s[lo]);
    }

    float magnitude(float fn) const {
        return cascadeMagSharp(coeffs, fn, sharp);
    }

    void reset() {
        for (int k = 0; k < TRITON_STAGES; k++) f[k].reset();
    }

    BiquadCoeffs coeffs[TRITON_STAGES];
    float sharp = 1.f;

private:
    Mode mode = LOWPASS;
    rack::dsp::TBiquadFilter<float> f[TRITON_STAGES];
};