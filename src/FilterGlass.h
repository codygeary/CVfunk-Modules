////////////////////////////////////////////////////////////
//
//   FilterGlass.h
//
//   written by Cody Geary
//   Copyright 2026, MIT License
//
//   DSP primitives for the Glass glass armonica synthesizer.
//
////////////////////////////////////////////////////////////

#pragma once
#include <cmath>
#include <vector>
#include <algorithm>
#include "rack.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Utility
// ─────────────────────────────────────────────────────────────────────────────

inline float glassLagrange(float y0, float y1, float y2, float y3, float t) {
    return ((-t * (t-1.f) * (t-2.f)) / 6.f) * y0
         + (((t+1.f) * (t-1.f) * (t-2.f)) / 2.f) * y1
         + ((-(t+1.f) * t * (t-2.f)) / 2.f) * y2
         + (((t+1.f) * t * (t-1.f)) / 6.f) * y3;
}

inline float glassFastExp(float x) {
    x = 1.0f + x / 256.0f;
    x *= x; x *= x; x *= x; x *= x;
    x *= x; x *= x; x *= x; x *= x;
    return x;
}

inline float glassMorphShape(float t, float m, float A = 6.f) {
    float a = m * A;
    if (fabsf(a) < 1e-3f) return t;
    return (glassFastExp(a * t) - 1.f) / (glassFastExp(a) - 1.f);
}

// glassDspSin / glassDspCos
// Taylor-series sin/cos accurate to <0.0002 across full range.
// Ported from SandsDSP::dspSin (MIT, Cody Geary).
// Wraps input to (-pi, pi] before evaluating the polynomial.
inline float glassDspWrapToPi(float x) {
    const float twoPi = 2.0f * float(M_PI);
    x = fmodf(x + float(M_PI), twoPi);
    if (x < 0.f) x += twoPi;
    return x - float(M_PI);
}

inline float glassDspSin(float x) {
    x = glassDspWrapToPi(x);
    float x2 = x * x;
    return x * (1.f - x2 * (1.f/6.f - x2 * (1.f/120.f
               - x2 * (1.f/5040.f - x2 / 362880.f))));
}

// ─────────────────────────────────────────────────────────────────────────────
// GlassDCBlocker
// ─────────────────────────────────────────────────────────────────────────────
struct GlassDCBlocker {
    float x1 = 0.f, y1 = 0.f, R = 0.9995f;
    void setSampleRate(float sr) {
        R = rack::clamp(1.f - 2.f * float(M_PI) * 20.f / sr, 0.990f, 0.9999f);
    }
    float process(float x) {
        y1 = x - x1 + R * y1;
        x1 = x;
        return y1;
    }
    void reset() { x1 = 0.f; y1 = 0.f; }
};

// ─────────────────────────────────────────────────────────────────────────────
// GlassEnvFollower — RMS follower for the ENV output
// ─────────────────────────────────────────────────────────────────────────────
struct GlassEnvFollower {
    float rms = 0.f, coeff = 0.001f;
    void setCoeff(float bandCenterNorm, float followTime, float sampleRate) {
        float periodMs = (bandCenterNorm > 1e-4f)
                       ? (1.f / (bandCenterNorm * sampleRate)) * 1000.f : 2000.f;
        periodMs = rack::clamp(periodMs, 0.5f, 2000.f);
        float scale = 0.5f + followTime * followTime * 299.5f;
        coeff = 1.f - expf(-1.f / ((periodMs * scale * 0.001f) * sampleRate));
    }
    float process(float in) {
        rms += coeff * (in * in - rms);
        return sqrtf(rms < 0.f ? 0.f : rms);
    }
    void reset() { rms = 0.f; }
};

// ─────────────────────────────────────────────────────────────────────────────
// GlassADAADrive
// Anti-derivative anti-aliased tanh saturator. Ported from ADAADrive in
// FilterAulos.h (MIT). Applied to bowl output for harmonic richness.
//
// Input expected in Rack voltage scale (~[-10..10]).
// driveGain: ~0.3 (clean) to ~2.0 (saturated).
// Output in [-10..10].
// ─────────────────────────────────────────────────────────────────────────────
struct GlassADAADrive {
    float lastInput = 0.f;

    // Polynomial tanh approximation -- valid for |x| <= 1.
    // Matched to the antiderivative below (same Taylor series).
    static float polyTanh(float x) {
        float x2 = x * x;
        return x - x * x2 * (1.f/3.f - x2 * (2.f/15.f - 17.f/315.f * x2));
    }
    // Antiderivative of polyTanh -- valid for |x| <= 1.
    static float antiderivative(float x) {
        float x2 = x * x;
        return 0.5f*x2 - (1.f/12.f)*x2*x2 + (1.f/45.f)*x2*x2*x2
               - (17.f/2520.f)*x2*x2*x2*x2;
    }
    static float applyADAA(float input, float last) {
        float d = input - last;
        return fabsf(d) > 1e-6f
             ? (antiderivative(input) - antiderivative(last)) / d
             : polyTanh(input);
    }
    // Clamp norm to [-1,1] before the polynomial so it stays in its valid range.
    // tanh(x) for |x|>1 is already deep into saturation so clamping is correct.
    float process(float inV) {
        float norm = rack::clamp(inV, -13.14f, 13.14f) / 10.f;
        norm = rack::clamp(norm, -1.f, 1.f);
        float out  = applyADAA(norm, lastInput);
        lastInput  = norm;
        return rack::clamp(out * 6.9f, -10.f, 10.f);
    }
    void reset() { lastInput = 0.f; }
};


// ─────────────────────────────────────────────────────────────────────────────
// GlassBowl
//
// High-Q passive resonator. Delay line with one-pole LPF on the feedback path.
// feedbackGain always < 1. Rings up from excitation, decays freely on release.
// ─────────────────────────────────────────────────────────────────────────────
struct GlassBowl {
    std::vector<float> buf;
    int   bufSize    = 0;
    int   bufMask    = 0;
    int   writeIndex = 0;
    float lpfZ       = 0.f;
    float lastOut    = 0.f;

    void init(float sr, float lowestPitchHz = 130.81f) {
        float maxDelaySec = 1.f / lowestPitchHz * 1.5f;
        int needed = (int)ceilf(maxDelaySec * sr) + 8;
        bufSize = 1;
        while (bufSize < needed) bufSize <<= 1;
        bufMask  = bufSize - 1;
        buf.assign(bufSize, 0.f);
        writeIndex = 0;
        lpfZ    = 0.f;
        lastOut = 0.f;
    }

    inline float lagrangeRead(float delaySamples) const {
        delaySamples = rack::clamp(delaySamples, 1.f, (float)bufSize - 4.f);
        float rp   = (float)writeIndex - delaySamples;
        int   base = ((int)floorf(rp)) & bufMask;
        float frac = rp - floorf(rp);
        float y0   = buf[(base - 1) & bufMask];
        float y1   = buf[base       & bufMask];
        float y2   = buf[(base + 1) & bufMask];
        float y3   = buf[(base + 2) & bufMask];
        return glassLagrange(y0, y1, y2, y3, frac);
    }

    // excitation:   signal injected this sample.
    // delaySamples: bowl resonant frequency period in samples.
    // lpfCoeff:     one-pole LPF coeff on feedback (0=bright, ~0.3=dull).
    // feedbackGain: loop recirculation. < 1 = decay, > 1 = overblow.
    float process(float excitation, float delaySamples,
                  float lpfCoeff, float feedbackGain) {

        float delayed = lagrangeRead(delaySamples);

        // One-pole LPF on feedback path.
        lpfZ = (1.f - lpfCoeff) * delayed + lpfCoeff * lpfZ;
        if (!std::isfinite(lpfZ)) lpfZ = 0.f;

        float writeVal = excitation + feedbackGain * lpfZ;
        if (!std::isfinite(writeVal)) writeVal = 0.f;

        buf[writeIndex] = writeVal;
        writeIndex = (writeIndex + 1) & bufMask;

        lastOut = delayed;
        return delayed;
    }

    void clear() {
        std::fill(buf.begin(), buf.end(), 0.f);
        writeIndex = 0;
        lpfZ    = 0.f;
        lastOut = 0.f;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// GlassPressureEnv
// ASR envelope. Gate amplitude = peak pressure. Smooth retrigger.
// ─────────────────────────────────────────────────────────────────────────────
enum class GlassEnvPhase { IDLE, GROWTH, SUSTAIN, DECAY };

struct GlassPressureEnv {
    GlassEnvPhase phase = GlassEnvPhase::IDLE;
    float counter = 0.f, out = 0.f, baseline = 0.f, decayStart = 0.f;

    void startAttack()  { baseline = out; counter = 0.f; phase = GlassEnvPhase::GROWTH; }
    void startRelease() { decayStart = out; counter = 0.f; phase = GlassEnvPhase::DECAY; }

    float process(bool gateHigh, float sustainLevel,
                  float attackSamples, float releaseSamples,
                  float attackCurve,   float releaseCurve) {

        float peak = rack::clamp(sustainLevel, 0.f, 1.f);

        switch (phase) {
        case GlassEnvPhase::IDLE:
            out = 0.f;
            if (gateHigh) startAttack();
            break;
        case GlassEnvPhase::GROWTH:
            counter += 1.f;
            if (attackSamples <= 0.f || counter >= attackSamples) {
                out = peak; phase = GlassEnvPhase::SUSTAIN; counter = 0.f;
            } else {
                float t = counter / attackSamples;
                out = rack::clamp(baseline + (peak - baseline)
                                  * glassMorphShape(t, attackCurve), 0.f, 1.f);
            }
            if (!gateHigh) startRelease();
            break;
        case GlassEnvPhase::SUSTAIN:
            out = peak;
            if (!gateHigh) startRelease();
            break;
        case GlassEnvPhase::DECAY:
            counter += 1.f;
            {
                float scaledR = releaseSamples * rack::clamp(decayStart, 0.f, 1.f);
                if (scaledR <= 1.f || out <= 0.0001f || counter >= scaledR) {
                    out = 0.f; phase = GlassEnvPhase::IDLE;
                } else {
                    float t = counter / scaledR;
                    out = rack::clamp(decayStart
                          * (1.f - glassMorphShape(t, releaseCurve)), 0.f, 1.f);
                }
            }
            if (gateHigh) startAttack();
            break;
        }
        return out;
    }

    void reset() {
        phase = GlassEnvPhase::IDLE; counter = 0.f;
        out = 0.f; baseline = 0.f; decayStart = 0.f;
    }
};