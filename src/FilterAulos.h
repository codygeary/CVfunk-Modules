////////////////////////////////////////////////////////////
//
//   FilterAulos.h
//
//   written by Cody Geary
//   Copyright 2026, MIT License
//
//   DSP primitives for the Aulos wind instrument synthesizer.
//   All structs are self-contained and rack-state-free.
//
//   Primitives drawn from MIT-licensed sources:
//     - AulosDCBlocker, AulosNyquistCap   <- FilterTriton.h (MIT)
//     - AulosEnvFollower                  <- Triton.cpp (MIT)
//     - aulosLagrange()                   <- Droplet.cpp (MIT, original math)
//     - ADAADrive                         <- Triton.cpp (MIT)
//   New structures (AulosWaveguide, AulosJetDelay, AulosBreathEnv,
//   aulosJetFunction) are original to this file.
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

// Four-point Lagrange interpolation — identical to droplet_lagrange().
// Evaluates a cubic interpolant through four equally spaced samples y0..y3
// at fractional position t in [0,1] between y1 and y2.
inline float aulosLagrange(float y0, float y1, float y2, float y3, float t) {
    return ((-t * (t-1.f) * (t-2.f)) / 6.f) * y0
         + (((t+1.f) * (t-1.f) * (t-2.f)) / 2.f) * y1
         + ((-(t+1.f) * t * (t-2.f)) / 2.f) * y2
         + (((t+1.f) * t * (t-1.f)) / 6.f) * y3;
}

// Taylor-series sin/cos accurate to <0.0002 across full range.
// Wraps input to (-pi, pi] before evaluating the polynomial.
inline float aulosDspWrapToPi(float x) {
    const float twoPi = 2.0f * float(M_PI);
    x = fmodf(x + float(M_PI), twoPi);
    if (x < 0.f) x += twoPi;
    return x - float(M_PI);
}

inline float aulosDspSin(float x) {
    x = aulosDspWrapToPi(x);
    float x2 = x * x;
    return x * (1.f - x2 * (1.f/6.f - x2 * (1.f/120.f
               - x2 * (1.f/5040.f - x2 / 362880.f))));
}



// Cubic jet nonlinearity — models the vortex-to-edge coupling in a flute
// embouchure. Soft symmetric saturation: output approaches +-2/3 as x -> +-inf.
// Keeps flute mode cleaner than the asymmetric reed saturator.
inline float aulosJetFunction(float x) {
    return x - x * x * x * (1.f / 3.f);
}

// Fast exponential approximation via repeated squaring — same as Dunes/Droplet.
// Accurate to ~0.1% for |x| < 8, which covers all envelope curve use cases.
inline float aulosFastExp(float x) {
    x = 1.0f + x / 256.0f;
    x *= x; x *= x; x *= x; x *= x;
    x *= x; x *= x; x *= x; x *= x;
    return x;
}

// Exponential curve shaping for envelope segments — same as Dunes morphShape().
// t in [0,1], m in [-1,1]: m=0 linear, m>0 convex (fast rise),
// m<0 concave (slow rise). A=6 gives a perceptually wide range.
inline float aulosMorphShape(float t, float m, float A = 6.f) {
    float a = m * A;
    if (fabsf(a) < 1e-3f) return t;
    return (aulosFastExp(a * t) - 1.f) / (aulosFastExp(a) - 1.f);
}

// ─────────────────────────────────────────────────────────────────────────────
// AulosDCBlocker
// First-order high-pass at ~20Hz. Removes DC from waveguide output.
// Adapted from FilterTriton.h (MIT).
// ─────────────────────────────────────────────────────────────────────────────
struct AulosDCBlocker {
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
// AulosOnePoleLPF
// One-pole lowpass filter. coeff = exp(-2*pi*fc/sr).
// coeff near 0 -> transparent (high cutoff). coeff near 1 -> heavy damping.
// ─────────────────────────────────────────────────────────────────────────────
struct AulosOnePoleLPF {
    float z = 0.f, coeff = 0.f;

    void setCutoff(float sr, float fc) {
        coeff = expf(-2.f * float(M_PI) * fc / sr);
    }
    float process(float x) {
        z = coeff * z + (1.f - coeff) * x;
        return z;
    }
    void reset() { z = 0.f; }
};

// ─────────────────────────────────────────────────────────────────────────────
// AulosOnePoleHPF
// One-pole highpass filter. HPF = input minus its LP component.
// ─────────────────────────────────────────────────────────────────────────────
struct AulosOnePoleHPF {
    float z = 0.f, coeff = 0.f;

    void setCutoff(float sr, float fc) {
        coeff = expf(-2.f * float(M_PI) * fc / sr);
    }
    float process(float x) {
        z = coeff * z + (1.f - coeff) * x;
        return x - z;
    }
    void reset() { z = 0.f; }
};

// ─────────────────────────────────────────────────────────────────────────────
// AulosNyquistCap
// One-pole anti-alias cap fixed near Nyquist (~0.225 * sr, ~10.8kHz at 48kHz).
// Applied to voiceOut after the DC blocker. Prevents alias energy from waveguide
// harmonics and overblow transients reaching the output. Ported from NyquistCap
// in FilterTriton.h.
// ─────────────────────────────────────────────────────────────────────────────
struct AulosNyquistCap {
    float z = 0.f, coeff = 0.8f;
    void setSampleRate(float sr) {
        // Fixed cutoff at ~0.225 * sr — well above audible range, catches alias energy.
        coeff = rack::clamp(1.f - expf(-2.f * float(M_PI) * 0.225f), 0.5f, 0.99f);
        (void)sr;  // cutoff is normalized, sr not needed
    }
    float process(float x) { z += coeff * (x - z); return z; }
    void reset() { z = 0.f; }
};

// ─────────────────────────────────────────────────────────────────────────────
// AulosEnvFollower
// RMS envelope follower. Adapted from Triton.cpp (MIT).
// bandCenterNorm: normalized center frequency of the signal being tracked.
// followTime 0..1: 0=fast response, 1=very slow.
// ─────────────────────────────────────────────────────────────────────────────
struct AulosEnvFollower {
    float rms = 0.f, coeff = 0.001f;

    void setCoeff(float bandCenterNorm, float followTime, float sampleRate) {
        float periodMs = (bandCenterNorm > 1e-4f)
                       ? (1.f / (bandCenterNorm * sampleRate)) * 1000.f
                       : 2000.f;
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
// ADAADrive
// Anti-derivative anti-aliased tanh saturator. Adapted from Triton.cpp (MIT).
// Used as the reed nonlinearity in the waveguide loop.
//
// driveGain range: 1.0 (transparent / linear waveguide) to ~8.0 (hard reed).
// Input expected roughly in [-1,1] at driveGain=1.
// ─────────────────────────────────────────────────────────────────────────────
struct ADAADrive {
    float lastInput = 0.f;

    static float polyTanh(float x) {
        float x2 = x * x;
        return x - x * x2 * (1.f/3.f - x2 * (2.f/15.f - 17.f/315.f * x2));
    }
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

    float process(float inV, float driveGain) {
        float sig  = rack::clamp(inV * driveGain, -13.14f, 13.14f);
        float norm = sig / 10.f;
        float out  = applyADAA(norm, lastInput);
        lastInput  = norm;
        return rack::clamp(out * 6.9f, -10.f, 10.f);
    }
    void reset() { lastInput = 0.f; }
};

// ─────────────────────────────────────────────────────────────────────────────
// AulosWaveguide
// Bidirectional delay line — the bore resonator.
//
// The write path applies a one-pole LPF (dampCoeff) that models material
// absorption. Amplitude-dependent loop gain compression (same mechanism as
// AlloyNode in Droplet) prevents nonlinear runaway at high resonance.
//
// process() injects `input` and returns the delayed read at `delaySamples`.
// readEnd() reads at a specified delay without advancing the write pointer —
// used by the flute jet feedback path before process() is called.
//
// Buffer sized at init for the lowest expected pitch (~18Hz at 55ms).
// At 48kHz: 4096 samples = ~8KB per waveguide.
// ─────────────────────────────────────────────────────────────────────────────
struct AulosWaveguide {
    std::vector<float> buf;
    int   bufSize    = 0;
    int   bufMask    = 0;
    int   writeIndex = 0;
    float dampZ1     = 0.f;   // one-pole LPF state on write path
    float lastInput  = 0.f;   // retained for ADAA continuity (not currently used
                               // inside this struct, but useful for future ADAA here)

    // Per-sample drain scalar. Setting this below 1.0 causes all reads and
    // writes to be attenuated, equivalent to multiplying every buffer sample
    // each sample — but at O(1) cost instead of O(bufSize).
    // Set to emergencyDrain each sample to replicate the original drain behavior
    // without iterating the buffer.
    float drainGain  = 1.f;

    // Higher compressionAmount = more gain reduction at high amplitudes.
    // Aulos runs a single tight feedback loop with continuous excitation, so it
    // needs more compression than Droplet's parallel nodes to stay stable at
    // high Holes + Tone settings. Tune upward if runaway persists.
    static constexpr float compressionAmount = 0.30f;

    // Intrinsic material damping — models air column loss and bore wall absorption
    // present in any real instrument regardless of the user Damp slider setting.
    // Equivalent to a one-pole LPF at ~14kHz at 48kHz. Tune if the open timbre
    // is too bright or too dull at damp=0.
    static constexpr float baseDampCoeff = 0.16f;

    void init(float sr, float maxDelaySec = 0.055f) {
        int needed = (int)ceilf(maxDelaySec * sr) + 8;
        bufSize = 1;
        while (bufSize < needed) bufSize <<= 1;
        bufMask  = bufSize - 1;
        buf.assign(bufSize, 0.f);
        writeIndex = 0;
        dampZ1     = 0.f;
        lastInput  = 0.f;
        drainGain  = 1.f;
    }

    // Lagrange fractional delay read. Does NOT advance the write pointer.
    inline float lagrangeRead(float delaySamples) const {
        delaySamples = rack::clamp(delaySamples, 1.f, (float)bufSize - 4.f);
        float rp   = (float)writeIndex - delaySamples;
        int   base = ((int)floorf(rp)) & bufMask;
        float frac = rp - floorf(rp);
        float y0   = buf[(base - 1) & bufMask];
        float y1   = buf[base & bufMask];
        float y2   = buf[(base + 1) & bufMask];
        float y3   = buf[(base + 2) & bufMask];
        return drainGain * aulosLagrange(y0, y1, y2, y3, frac);
    }

    // Read the tube's far (bell) end — used by the flute jet feedback path.
    // Call this before process() so the read precedes the write on the same sample.
    inline float readEnd(float delaySamples) const {
        return lagrangeRead(delaySamples);
    }

    // Inject input and return the delayed output.
    // dampCoeff: one-pole coefficient (0=no loss, ~0.97=heavy damping).
    // feedback:  loop recirculation gain (0=no resonance, ~0.98=high resonance).
    float process(float input, float delaySamples, float dampCoeff, float feedback, float boreDamp = baseDampCoeff) {
    
        float delayed = lagrangeRead(delaySamples);

        // Amplitude-dependent compression on the feedback path — prevents runaway.
        float loopComp = 1.f / (1.f + fabsf(delayed) * compressionAmount);
        float loopIn   = input + feedback * loopComp * delayed;

        // One-pole LPF on write path — material absorption / damping.
        float effectiveDamp = fmaxf(dampCoeff, boreDamp);
        dampZ1 = (1.f - effectiveDamp) * loopIn * drainGain + effectiveDamp * dampZ1;

        if (!std::isfinite(dampZ1)) dampZ1 = 0.f;

        buf[writeIndex] = dampZ1;
        writeIndex      = (writeIndex + 1) & bufMask;
        lastInput       = loopIn;

        return delayed;
    }

    void clear() {
        std::fill(buf.begin(), buf.end(), 0.f);
        writeIndex = 0;
        dampZ1     = 0.f;
        lastInput  = 0.f;
        drainGain  = 1.f;
    }

    void idleTick() {}

    // Scale all buffer contents by gain — soft energy drain without a hard clear.
    // Kept for non-realtime use (e.g. panic). In the audio loop, set drainGain
    // instead for O(1) cost.
    void drain(float gain) {
        for (auto& s : buf) s *= gain;
        dampZ1    *= gain;
        lastInput *= gain;
        drainGain  = 1.f;  // buffer is already scaled, reset the live scalar
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// AulosJetDelay
// Short pure delay line for the flute jet travel path.
// No feedback loop — just write-then-read with Lagrange interpolation.
//
// The jet delay models the travel time of a vortex from the embouchure hole
// to the resonator opening, ~0.47 of the played period. It is pitch-scaled
// by the caller:
//   delaySamples = clamp(sr * 0.00107 * (440 / fingerFreq), 2, sr * 0.028)
// The buffer must cover half the period of the lowest playable fundamental.
// ─────────────────────────────────────────────────────────────────────────────
struct AulosJetDelay {
    std::vector<float> buf;
    int bufSize    = 0;
    int bufMask    = 0;
    int writeIndex = 0;

    void init(float sr, float maxDelaySec = 0.03f) {
        int needed = (int)ceilf(maxDelaySec * sr) + 8;
        bufSize = 1;
        while (bufSize < needed) bufSize <<= 1;
        bufMask = bufSize - 1;
        buf.assign(bufSize, 0.f);
        writeIndex = 0;
    }

    // Write input to the delay line and return the output at delaySamples ago.
    float process(float input, float delaySamples) {
        buf[writeIndex] = input;
        writeIndex      = (writeIndex + 1) & bufMask;

        delaySamples = rack::clamp(delaySamples, 1.f, (float)bufSize - 4.f);
        float rp   = (float)writeIndex - delaySamples;
        int   base = ((int)floorf(rp)) & bufMask;
        float frac = rp - floorf(rp);
        float y0   = buf[(base - 1) & bufMask];
        float y1   = buf[base & bufMask];
        float y2   = buf[(base + 1) & bufMask];
        float y3   = buf[(base + 2) & bufMask];
        return aulosLagrange(y0, y1, y2, y3, frac);
    }

    void clear() {
        std::fill(buf.begin(), buf.end(), 0.f);
        writeIndex = 0;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// AulosBreathEnv
// ASR envelope adapted from Dunes (original math by Cody Geary, MIT).
//
// Phases: IDLE -> GROWTH (attack) -> SUSTAIN (gate held) -> DECAY (release).
//
// sustainLevel (0-1) scales the peak — unlike Dunes where peak is fixed at 10V.
// This means SUSTAIN knob directly controls breath pressure during a held note.
//
// Smooth retrigger: attack starts from the current output level rather than 0,
// so legato playing doesn't cause a level dip between notes.
//
// attackCurve / releaseCurve: aulosMorphShape m parameter (-1 to +1).
// Passed in from module-level context-menu settings each sample.
// ─────────────────────────────────────────────────────────────────────────────
enum class AulosEnvPhase { IDLE, GROWTH, SUSTAIN, DECAY };

struct AulosBreathEnv {
    AulosEnvPhase phase   = AulosEnvPhase::IDLE;
    float counter         = 0.f;   // sample counter within current phase
    float out             = 0.f;   // current output, 0..10V
    float baseline        = 0.f;   // output level at last attack start (smooth retrig)
    float decayStart      = 0.f;   // output level when decay phase began

    void startAttack() {
        baseline = out;
        counter  = 0.f;
        phase    = AulosEnvPhase::GROWTH;
    }

    void startRelease() {
        decayStart = out;
        counter    = 0.f;
        phase      = AulosEnvPhase::DECAY;
    }

    // Process one sample. Returns envelope output in volts (0..10V).
    // gateHigh:       true while gate voltage is above threshold
    // sustainLevel:   target peak level, 0..1  (SUSTAIN knob + CV, normalized)
    // attackSamples:  attack phase duration in samples
    // releaseSamples: release phase duration in samples (at full level; scales shorter for softer notes)
    // attackCurve:    morphShape m param for attack shape
    // releaseCurve:   morphShape m param for release shape
    float process(bool gateHigh,
                  float sustainLevel,
                  float attackSamples, float releaseSamples,
                  float attackCurve,   float releaseCurve) {

        float peakVoltage = rack::clamp(sustainLevel, 0.f, 1.f) * 10.f;

        switch (phase) {

        case AulosEnvPhase::IDLE:
            out = 0.f;
            if (gateHigh) startAttack();
            break;

        case AulosEnvPhase::GROWTH:
            counter += 1.f;
            if (attackSamples <= 0.f || counter >= attackSamples) {
                out   = peakVoltage;
                phase = AulosEnvPhase::SUSTAIN;
                counter = 0.f;
            } else {
                float t     = counter / attackSamples;
                float shaped = aulosMorphShape(t, attackCurve);
                out = baseline + (peakVoltage - baseline) * shaped;
                out = rack::clamp(out, 0.f, 10.f);
            }
            // Gate released before attack completed -> skip directly to release.
            if (!gateHigh) startRelease();
            break;

        case AulosEnvPhase::SUSTAIN:
            // Track peakVoltage in real time so SUSTAIN knob acts as live breath pressure.
            out = peakVoltage;
            if (!gateHigh) startRelease();
            break;

        case AulosEnvPhase::DECAY:
            counter += 1.f;
            {
                // Scale release time proportionally to level at gate-fall —
                // a soft note releases proportionally faster than a loud one.
                float scale   = rack::clamp(decayStart / 10.f, 0.f, 1.f);
                float scaledR = releaseSamples * scale;

                if (scaledR <= 1.f || out <= 0.001f || counter >= scaledR) {
                    out   = 0.f;
                    phase = AulosEnvPhase::IDLE;
                } else {
                    float t = counter / scaledR;
                    out = decayStart * (1.f - aulosMorphShape(t, releaseCurve));
                    out = rack::clamp(out, 0.f, 10.f);
                }
            }
            // Gate retriggered during release -> smooth restart from current level.
            if (gateHigh) startAttack();
            break;
        }

        return out;
    }

    void reset() {
        phase      = AulosEnvPhase::IDLE;
        counter    = 0.f;
        out        = 0.f;
        baseline   = 0.f;
        decayStart = 0.f;
    }
};