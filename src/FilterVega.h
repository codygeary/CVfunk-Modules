////////////////////////////////////////////////////////////
//
//   FilterVega.h
//
//   written by Cody Geary
//   Copyright 2026, MIT License
//
//   Voice engine for Vega, after a 1976 analog bass pedal synthesizer
//   (Norlin schematic 993-040749 and service bulletin 1178).
//
//   Signal path, as on the schematic:
//     two sawtooth oscillators (3080 integrators reset by an E112 FET)
//     -> IC801 4016 mixer switch (passes audio inside about +-7.5 V)
//     -> R513 68K / C501 0.1 uF / R520 1K into the ladder's bottom pair
//     -> transistor ladder Q502-Q511, four 0.1 uF rungs (C504-C507)
//     -> A501 CA3080 reading the top rung into R526 470K || C508 500 pF,
//        whose output also feeds the emphasis back through C509 10 uF
//     -> R304 33K / R306 2.2K into A301, the CA3080 VCA
//     -> C306 0.22 uF into the AUD level pot.
//
//   Every transistor pair follows tanh of its differential voltage over
//   2 Vt, as a bounded rational (see pairLaw).
//
////////////////////////////////////////////////////////////

#pragma once
#include "rack.hpp"
#include "FilterFuzzy.h"
#include <cmath>
#include <algorithm>

namespace vega {

// ---- Schematic values ------------------------------------------------------------
static const float kTwoVt = 2.f * 0.025693f;                           // 25 C
static const float kInputDivider = 1e3f / 69e3f;                       // R520 / (R513 + R520)
static const float kInputHighPassHz = 1.f / (2.f * 3.14159265f * 0.1e-6f * 69e3f);   // C501, ~23 Hz
static const float kLoadHz = 1.f / (2.f * 3.14159265f * 470e3f * 500e-12f);           // A501 load, ~677 Hz
static const float kFeedbackHighPassHz = 1.f;                           // C509 into the emphasis network
static const float kVcaDivider = 2.2e3f / 35.2e3f;                     // R306 / (R304 + R306)
static const float kOutputHighPassHz = 1.f / (2.f * 3.14159265f * 0.22e-6f * 100e3f);  // C306 into R317, ~7 Hz
static const float kContourFarads = 5.6e-6f;                           // C201, C402
static const float kSwitchRailVolts = 7.5f;                            // 4016 control levels, bulletin note 6

// ---- Operating levels the schematic does not fix (hand-tunable) ------------------
static const float kSawVolts = 5.f;              // saw peak at the mixer; GRIT scales it
static const float kSwitchKneeVolts = 6.f;       // the 4016 ceiling's knee
static const float kA501Volts = 1.f;             // A501's full output swing
static const float kLadderNoise = 2e-4f;         // bottom-pair noise; lets the ladder self-start
static const float kMaxCutoffG = 6.3137515f;     // tan(pi 0.45): the cutoff's ceiling under SNARL
static const float kSnarlSubSquare = 0.5f;       // SNARL's sub-octave: 0 ramp (smooth) .. 1 square (hard)

// ==========================================================================
// Pair law
// ==========================================================================
// tanh as Mala's bounded rational x (27 + x^2) / (27 + 9 x^2): within 0.024
// of tanh up to |x| = 3, where it reaches 1 with zero slope and holds.
static const float kPairLimit = 3.f;
inline float pairLaw(float v) {
    const float x = std::max(-kPairLimit, std::min(kPairLimit, v));
    const float x2 = x * x;
    return x * (27.f + x2) / (27.f + 9.f * x2);
}
// The pair law and its slope at once (one divide).
inline void pairLawSlope(float v, float& value, float& slope) {
    const float x = std::max(-kPairLimit, std::min(kPairLimit, v));
    const float x2 = x * x;
    const float inverse = 1.f / (27.f + 9.f * x2);
    const float notch = 9.f - x2;
    value = x * (27.f + x2) * inverse;
    slope = 9.f * notch * notch * inverse * inverse;
}

// ==========================================================================
// Sawtooth with a two-sample polynomial band-limited step at the wrap.
// Measured at 2x: non-harmonic energy -62 (C5) to -74 dB (C1).
// ==========================================================================
struct Saw {
    double phase = 0.0;

    // increment: cycles per sample at the rate this runs at (0 .. 0.5).
    // stepInverse: 1 / increment, computed once per base sample.
    inline float process(double increment, float stepInverse) {
        phase += increment;
        if (phase >= 1.0) phase -= 1.0;
        const float t = (float)phase, step = (float)increment;
        float out = 2.f * t - 1.f;
        // The wrap drops the ramp by 2; the drop is spread over the sample
        // before and the sample after it with a quadratic.
        if (t < step) {
            const float x = t * stepInverse;
            out -= x + x - x * x - 1.f;
        } else if (t > 1.f - step) {
            const float x = (t - 1.f) * stepInverse;
            out -= x * x + x + x + 1.f;
        }
        return out;
    }

    // The same saw with its frequency modulated through zero: the increment
    // may go negative and the phase then runs backward. The step's smoothing
    // is the same function of the phase either way, over the step's size.
    inline float processThroughZero(double increment) {
        phase += increment;
        if (phase >= 1.0) phase -= 1.0;
        else if (phase < 0.0) phase += 1.0;
        const float t = (float)phase, step = (float)std::fabs(increment);
        float out = 2.f * t - 1.f;
        if (step > 1e-9f) {
            const float stepInverse = 1.f / step;
            if (t < step) {
                const float x = t * stepInverse;
                out -= x + x - x * x - 1.f;
            } else if (t > 1.f - step) {
                const float x = (t - 1.f) * stepInverse;
                out -= x * x + x + x + 1.f;
            }
        }
        return out;
    }
};

// ==========================================================================
// Ladder
//
// Normalized units: each pair's differential voltage over 2 Vt.
// Unknowns per substep (trapezoidal, TPT integrators):
//   rung[k]  the four capacitor voltages, C507 (bottom) .. C504 (top)
//   load     A501's output node (470K || 500 pF), in units of its full swing
// Equations:
//   rung1 = state1 + g (S(input - emphasis (load - loadLow)) - S(rung1))
//   rungk = statek + g (S(rung(k-1)) - S(rungk))
//   load  = stateL + gLoad (S(rung4) - load)
// loadLow is a slow low-pass of the load: the emphasis returns through
// C509, so the feedback carries no DC.
// The Jacobian is a chain closed by the feedback: each Newton iteration
// solves it by walking the chain once.
// ==========================================================================

// Newton stop: largest change of any unknown, normalized units. Hand-tunable.
// Newton converges quadratically from the straight-line guess, so a step
// under 1e-3 leaves an error near 1e-6: measured -109 to -120 dB against a
// 1e-9 double solve, at 1.1-1.4 iterations per substep.
static const float kLadderTolerance = 1e-3f;
static const int kLadderMaxIterations = 12;

// Each pair's input offset (Vbe mismatch, over 2 Vt; ~1.5 mV): the bottom
// pair, then the pairs reading C507 .. C504. It bends the ladder slightly out
// of symmetry for even harmonics; its DC is taken back out so the VCA cannot
// thump. Voice 0's set; hand-tunable.
static const int kLadderPairs = 5;
static const float kPairOffsets[kLadderPairs] = { 0.031f, -0.018f, 0.024f, -0.027f, 0.012f };

struct Ladder {
    float state[4] = {};
    alignas(16) float rung[4] = {};
    float loadState = 0.f, load = 0.f, loadLow = 0.f;
    // Last substep's solution, for a straight-line first guess.
    float rungPrevious[4] = {}, loadPrevious = 0.f;
    // Pair offsets (bottom, then the four rung pairs) and the DC each would
    // carry, which is taken back out.
    float bottomOffset = 0.f, bottomOffsetLevel = 0.f;
    alignas(16) float rungOffset[4] = {}, rungOffsetLevel[4] = {};

    void setOffsets(const float* offsets) {
        bottomOffset = offsets[0];
        bottomOffsetLevel = pairLaw(offsets[0]);
        for (int k = 0; k < 4; ++k) {
            rungOffset[k] = offsets[k + 1];
            rungOffsetLevel[k] = pairLaw(offsets[k + 1]);
        }
    }

    // input: drive at Q510's base, normalized. g, gLoad: tan(pi fc / rate).
    // emphasis: loop gain. lowCoeff: the C509 corner as a one-pole.
    inline float process(float input, float g, float gLoad, float emphasis, float lowCoeff) {
        for (int k = 0; k < 4; ++k) {
            const float guess = rung[k] + rung[k] - rungPrevious[k];
            rungPrevious[k] = rung[k];
            rung[k] = guess;
        }
        {
            const float guess = load + load - loadPrevious;
            loadPrevious = load;
            load = guess;
        }
        for (int iteration = 0; iteration < kLadderMaxIterations; ++iteration) {
            // All four rung pairs at once (one vector divide).
            float split[4], slope[4];
            {
                const rack::simd::float_4 x = rack::simd::clamp(rack::simd::float_4::load(rung) + rack::simd::float_4::load(rungOffset),
                                                               -kPairLimit, kPairLimit);
                const rack::simd::float_4 x2 = x * x;
                const rack::simd::float_4 inverse = 1.f / (27.f + 9.f * x2);
                const rack::simd::float_4 notch = 9.f - x2;
                (x * (27.f + x2) * inverse - rack::simd::float_4::load(rungOffsetLevel)).store(split);
                (9.f * notch * notch * inverse * inverse).store(slope);
            }
            const float drive = input - emphasis * (load - loadLow);
            float bottom, bottomSlope;
            pairLawSlope(drive + bottomOffset, bottom, bottomSlope);
            bottom -= bottomOffsetLevel;
            // d bottom / d load
            const float bottomPerLoad = -emphasis * bottomSlope;

            float residual[4];
            residual[0] = rung[0] - state[0] - g * (bottom - split[0]);
            for (int k = 1; k < 4; ++k)
                residual[k] = rung[k] - state[k] - g * (split[k - 1] - split[k]);
            const float residualLoad = load - loadState - gLoad * (split[3] - load);

            // Walk the chain: each rung's change as offset + perLoad * loadChange.
            float rungOffset[4], rungPerLoad[4];
            float diagonal = 1.f + g * slope[0];
            rungOffset[0] = -residual[0] / diagonal;
            rungPerLoad[0] = g * bottomPerLoad / diagonal;
            for (int k = 1; k < 4; ++k) {
                diagonal = 1.f + g * slope[k];
                const float coupling = g * slope[k - 1];
                rungOffset[k] = (-residual[k] + coupling * rungOffset[k - 1]) / diagonal;
                rungPerLoad[k] = coupling * rungPerLoad[k - 1] / diagonal;
            }
            const float coupling = gLoad * slope[3];
            const float loadChange = (-residualLoad + coupling * rungOffset[3])
                                   / (1.f + gLoad - coupling * rungPerLoad[3]);

            float largest = std::fabs(loadChange);
            load += loadChange;
            for (int k = 0; k < 4; ++k) {
                const float change = rungOffset[k] + rungPerLoad[k] * loadChange;
                rung[k] += change;
                largest = std::max(largest, std::fabs(change));
            }
            if (largest < kLadderTolerance) break;
        }
        for (int k = 0; k < 4; ++k) state[k] = rung[k] + rung[k] - state[k];
        loadState = load + load - loadState;
        loadLow += lowCoeff * (load - loadLow);
        return load;
    }

    bool finite() const {
        bool ok = std::isfinite(load) && std::isfinite(loadState) && std::isfinite(loadLow);
        for (int k = 0; k < 4; ++k) ok = ok && std::isfinite(rung[k]) && std::isfinite(state[k]);
        return ok;
    }

    void reset() {
        for (int k = 0; k < 4; ++k) state[k] = rung[k] = rungPrevious[k] = 0.f;
        loadState = load = loadLow = loadPrevious = 0.f;
    }
};

// ==========================================================================
// Contour generator (Q201 loudness, Q401 filter). The gate drives a
// Darlington follower; its emitter charges the 5.6 uF contour cap through
// the preset's attack divider and diode, toward that divider's level, and
// holds there while the pedal is down. Lifting the pedal turns on the
// discharge path: the preset's decay resistor with DECAY on, the 10K
// (R213, R414) with it off.
// ==========================================================================
struct Contour {
    float value = 0.f;

    inline float process(bool gate, float level, float attackCoeff, float releaseCoeff) {
        if (gate) value += (level - value) * attackCoeff;
        else      value -= value * releaseCoeff;
        return value;
    }
};

// One-pole coefficient for an RC time constant at sampleTime.
inline float rcCoeff(float ohms, float farads, float sampleTime) {
    return 1.f - std::exp(-sampleTime / std::max(ohms * farads, 1e-7f));
}

// ==========================================================================
// Pitch hold (C112, buffered by the Q101 FET, switched by IC106 4016).
// While the pedal is down the cap follows the keyboard voltage, through the
// glide resistance when GLIDE is on. Lifted, the switch opens and the cap
// holds the last note through the release tail, drooping a little as an aged
// board and FET leak (the bulletin's servicing guide 2).
// Volts are 1 V/oct, 0 = the low C pedal.
// ==========================================================================
struct PitchHold {
    float volts = 0.f;

    // followCoeff: per sample, glide or the switch's own quick settle.
    // droopPerSample: volts lost per sample while holding.
    inline float process(bool gate, float target, float followCoeff, float droopPerSample) {
        if (gate) volts += (target - volts) * followCoeff;
        else volts -= droopPerSample;
        return volts;
    }
};

// ==========================================================================
// Voice: oscillators, mixer switch, ladder, A501, A301 VCA, output coupling,
// oversampled by `oversample` and decimated by Fuzzy's resampler.
// ==========================================================================
struct VoiceControls {
    double frequencyA = 32.703, frequencyB = 32.703;   // Hz
    float weightA = 0.5f, weightB = 0.5f;              // mixer divider
    float cutoffHz = 100.f;
    float emphasis = 0.f;                              // loop gain
    float loudness = 0.f;                              // A301's bias, 0 .. 1
    float level = 1.f;                                 // GRIT: operating level multiplier
    float filterFm = 0.f;                              // SNARL: octaves of cutoff swing per unit of the modulator
    float subBlend = 0.f;                              // SNARL: the cutoff's modulator, 0 = osc B's saw .. 1 = its sub-octave
    float crossFm = 0.f;                               // SNARL: osc A's frequency swing per unit of the sub-octave (through-zero FM index)
    float aux = 0.f;                                   // AUX IN at the mixer, volts (this base sample)
};

struct Voice {
    Saw sawA, sawB;
    Ladder ladder;
    fuzzy::Resampler down;
    fuzzy::NoiseSource noise;
    int   oversample = 2;
    float rate = 96000.f;
    float inputHighPassCoeff = 1.f, inputHighPassState = 0.f, inputHighPassLast = 0.f;
    float gLoad = 0.f, lowCoeff = 0.f;
    float outputHighPassCoeff = 1.f, outputHighPassState = 0.f, outputHighPassLast = 0.f;
    // Cutoff and loudness move linearly across each base sample's substeps.
    float lastCutoffG = 0.f, lastLoudness = 0.f;
    // AUX IN at the previous base sample, interpolated across the substeps.
    float lastAux = 0.f;
    // SNARL's sub-octave divider: osc A's unmodulated phase at half rate.
    double subPhase = 0.0;

    void setRate(float baseRate, int factor) {
        oversample = factor;
        rate = baseRate * factor;
        down.setFactor(factor);
        inputHighPassCoeff = std::exp(-2.f * 3.14159265f * kInputHighPassHz / rate);
        gLoad = std::tan(3.14159265f * kLoadHz / rate);
        lowCoeff = 1.f - std::exp(-2.f * 3.14159265f * kFeedbackHighPassHz / rate);
        outputHighPassCoeff = std::exp(-2.f * 3.14159265f * kOutputHighPassHz / baseRate);
        reset();
    }

    void reset() {
        ladder.reset();
        down.reset();
        inputHighPassState = inputHighPassLast = 0.f;
        outputHighPassState = outputHighPassLast = 0.f;
        lastCutoffG = 0.f;
        lastLoudness = 0.f;
        lastAux = 0.f;
        subPhase = 0.5 * sawA.phase;
    }

    // One base sample in volts at the AUD pot (before the module's output gain).
    float process(const VoiceControls& c) {
        // Keep the cutoff below the substep rate's Nyquist.
        const float cutoff = std::min(c.cutoffHz, 0.45f * rate);
        const float cutoffG = std::tan(3.14159265f * cutoff / rate);
        const float startG = (lastCutoffG > 0.f) ? lastCutoffG : cutoffG;
        const float stepG = (cutoffG - startG) / oversample;
        const float stepLoudness = (c.loudness - lastLoudness) / oversample;
        lastCutoffG = cutoffG;

        const double incrementA = c.frequencyA / rate, incrementB = c.frequencyB / rate;
        const float inverseA = (float)(1.0 / incrementA), inverseB = (float)(1.0 / incrementB);
        const float mixScale = kSawVolts * c.level;
        const float inputScale = kInputDivider / kTwoVt;
        // A301's input pair sees A501's swing through R304 / R306.
        const float vcaDrive = kA501Volts * kVcaDivider / kTwoVt * c.level;
        const float vcaNormalize = kTwoVt / (kA501Volts * kVcaDivider);
        const float noiseLevel = kLadderNoise * noise.gaussian();

        const float stepAux = (c.aux - lastAux) / oversample;
        float g = startG, loudness = lastLoudness, aux = lastAux;
        float substeps[fuzzy::kMaxOversample];
        for (int s = 0; s < oversample; ++s) {
            g += stepG;
            loudness += stepLoudness;
            aux += stepAux;
            // AUX IN joins the saws at the mixer, at the GRIT level.
            const float sawValueB = sawB.process(incrementB, inverseB);
            // SNARL's divider: a ramp at half osc A's unmodulated rate, its
            // square edges on osc A's wraps.
            subPhase += 0.5 * incrementA;
            if (subPhase >= 1.0) subPhase -= 1.0;
            const float subRamp = 2.f * (float)subPhase - 1.f;
            const float subOctave = subRamp + kSnarlSubSquare * ((subPhase < 0.5 ? -1.f : 1.f) - subRamp);
            // SNARL, oscillator side: the sub-octave swings osc A's frequency
            // linearly, through zero, so osc A's average pitch stays put.
            const float sawValueA = (c.crossFm > 0.f) ? sawA.processThroughZero(incrementA * (1.0 + c.crossFm * subOctave))
                                                      : sawA.process(incrementA, inverseA);
            float mix = (c.weightA * sawValueA + c.weightB * sawValueB) * mixScale
                      + aux * c.level;
            // IC801's ceiling: linear to the knee, a cubic into the rail.
            const float magnitude = std::fabs(mix);
            if (magnitude > kSwitchKneeVolts) {
                const float width = 1.5f * (kSwitchRailVolts - kSwitchKneeVolts);
                const float over = std::min(magnitude - kSwitchKneeVolts, width);
                mix = std::copysign(kSwitchKneeVolts + over - over * over * over / (3.f * width * width), mix);
            }
            // C501 into the bottom pair's base.
            inputHighPassState = inputHighPassCoeff * (inputHighPassState + mix - inputHighPassLast);
            inputHighPassLast = mix;
            // SNARL, filter side: osc B's saw, crossfading to the sub-octave,
            // swings the cutoff exponentially, the same number of octaves at
            // every pitch and cutoff, whatever the mixer takes from osc B.
            // Not scaled by GRIT. Inverted, so the cutoff peaks as the saws
            // drop: their edges come through bright and each cycle snaps
            // shut. With the sub-octave, alternate cycles open differently:
            // the note growls an octave down.
            const float filterModulator = sawValueB + c.subBlend * (subOctave - sawValueB);
            const float modulatedG = (c.filterFm > 0.f) ? std::min(g * rack::dsp::exp2_taylor5(-c.filterFm * filterModulator), kMaxCutoffG) : g;
            const float load = ladder.process(inputHighPassState * inputScale + noiseLevel, modulatedG, gLoad, c.emphasis, lowCoeff);
            // A301: its bias current (the volume contour) times its input
            // pair's split. Divided by the drive so a small signal passes at
            // the same level whatever GRIT is. VCA BAL (R307) is on its trim,
            // so the bias current does not leak into the output.
            substeps[s] = loudness * (pairLaw(load * vcaDrive) * vcaNormalize);
        }
        lastLoudness = c.loudness;
        lastAux = c.aux;
        float out = down.down(substeps);
        // C306 into the AUD pot.
        outputHighPassState = outputHighPassCoeff * (outputHighPassState + out - outputHighPassLast);
        outputHighPassLast = out;
        return outputHighPassState;
    }
};

// ==========================================================================
// Four voices at once
//
// The same circuit as Voice, with every quantity a float_4: lane k is one
// voice. The ladder's Newton solve walks the same chain in all four lanes
// and stops when every lane has converged (a lane that is already there
// takes the extra step, which only refines it further).
// ==========================================================================
using rack::simd::float_4;

// The pair law across four lanes (see pairLaw).
inline float_4 pairLaw4(float_4 v) {
    const float_4 x = rack::simd::clamp(v, -kPairLimit, kPairLimit);
    const float_4 x2 = x * x;
    return x * (27.f + x2) / (27.f + 9.f * x2);
}

struct LadderGroup {
    float_4 state[4], rung[4], rungPrevious[4];
    float_4 loadState = 0.f, load = 0.f, loadLow = 0.f, loadPrevious = 0.f;
    // Each lane's pair offsets and the DC they would carry (see Ladder).
    float_4 bottomOffset = 0.f, bottomOffsetLevel = 0.f;
    float_4 rungOffset[4], rungOffsetLevel[4];

    LadderGroup() {
        reset();
        for (int k = 0; k < 4; ++k) rungOffset[k] = rungOffsetLevel[k] = 0.f;
    }

    // One lane's pair offsets, bottom first.
    void setOffsets(int lane, const float* offsets) {
        float lanes[4];
        bottomOffset.store(lanes); lanes[lane] = offsets[0]; bottomOffset = float_4::load(lanes);
        bottomOffsetLevel.store(lanes); lanes[lane] = pairLaw(offsets[0]); bottomOffsetLevel = float_4::load(lanes);
        for (int k = 0; k < 4; ++k) {
            rungOffset[k].store(lanes); lanes[lane] = offsets[k + 1]; rungOffset[k] = float_4::load(lanes);
            rungOffsetLevel[k].store(lanes); lanes[lane] = pairLaw(offsets[k + 1]); rungOffsetLevel[k] = float_4::load(lanes);
        }
    }

    inline float_4 process(float_4 input, float_4 g, float gLoad, float_4 emphasis, float lowCoeff) {
        for (int k = 0; k < 4; ++k) {
            const float_4 guess = rung[k] + rung[k] - rungPrevious[k];
            rungPrevious[k] = rung[k];
            rung[k] = guess;
        }
        {
            const float_4 guess = load + load - loadPrevious;
            loadPrevious = load;
            load = guess;
        }
        for (int iteration = 0; iteration < kLadderMaxIterations; ++iteration) {
            float_4 split[4], slope[4];
            for (int k = 0; k < 4; ++k) {
                const float_4 x = rack::simd::clamp(rung[k] + rungOffset[k], -kPairLimit, kPairLimit);
                const float_4 x2 = x * x;
                const float_4 inverse = 1.f / (27.f + 9.f * x2);
                const float_4 notch = 9.f - x2;
                split[k] = x * (27.f + x2) * inverse - rungOffsetLevel[k];
                slope[k] = 9.f * notch * notch * inverse * inverse;
            }
            const float_4 drive = input - emphasis * (load - loadLow);
            const float_4 bottomX = rack::simd::clamp(drive + bottomOffset, -kPairLimit, kPairLimit);
            const float_4 bottomX2 = bottomX * bottomX;
            const float_4 bottomInverse = 1.f / (27.f + 9.f * bottomX2);
            const float_4 bottomNotch = 9.f - bottomX2;
            const float_4 bottom = bottomX * (27.f + bottomX2) * bottomInverse - bottomOffsetLevel;
            const float_4 bottomPerLoad = -emphasis * 9.f * bottomNotch * bottomNotch * bottomInverse * bottomInverse;

            float_4 residual[4];
            residual[0] = rung[0] - state[0] - g * (bottom - split[0]);
            for (int k = 1; k < 4; ++k)
                residual[k] = rung[k] - state[k] - g * (split[k - 1] - split[k]);
            const float_4 residualLoad = load - loadState - gLoad * (split[3] - load);

            float_4 rungOffset[4], rungPerLoad[4];
            float_4 diagonal = 1.f + g * slope[0];
            rungOffset[0] = -residual[0] / diagonal;
            rungPerLoad[0] = g * bottomPerLoad / diagonal;
            for (int k = 1; k < 4; ++k) {
                diagonal = 1.f + g * slope[k];
                const float_4 coupling = g * slope[k - 1];
                rungOffset[k] = (-residual[k] + coupling * rungOffset[k - 1]) / diagonal;
                rungPerLoad[k] = coupling * rungPerLoad[k - 1] / diagonal;
            }
            const float_4 coupling = gLoad * slope[3];
            const float_4 loadChange = (-residualLoad + coupling * rungOffset[3])
                                     / (1.f + gLoad - coupling * rungPerLoad[3]);

            float_4 largest = rack::simd::fabs(loadChange);
            load += loadChange;
            for (int k = 0; k < 4; ++k) {
                const float_4 change = rungOffset[k] + rungPerLoad[k] * loadChange;
                rung[k] += change;
                largest = rack::simd::fmax(largest, rack::simd::fabs(change));
            }
            if (rack::simd::movemask(largest >= float_4(kLadderTolerance)) == 0) break;
        }
        for (int k = 0; k < 4; ++k) state[k] = rung[k] + rung[k] - state[k];
        loadState = load + load - loadState;
        loadLow += lowCoeff * (load - loadLow);
        return load;
    }

    // A sum of the whole state is non-finite whenever any part of it is.
    bool finite() {
        float_4 sum = load + loadState + loadLow;
        for (int k = 0; k < 4; ++k) sum += rung[k] + state[k];
        float lanes[4];
        sum.store(lanes);
        return std::isfinite(lanes[0]) && std::isfinite(lanes[1]) && std::isfinite(lanes[2]) && std::isfinite(lanes[3]);
    }

    void reset() {
        for (int k = 0; k < 4; ++k) state[k] = rung[k] = rungPrevious[k] = 0.f;
        loadState = load = loadLow = loadPrevious = 0.f;
    }
};

// Zero one lane of a float_4, for clearing a single voice.
inline void clearLane(float_4& value, int lane) {
    float lanes[4];
    value.store(lanes);
    lanes[lane] = 0.f;
    value = float_4::load(lanes);
}

// Fuzzy's decimator for four voices: each lane its own ring, laid out and
// summed exactly as fuzzy::Resampler::down does it, so a lane is filtered as
// a mono voice would be. Unused lanes are skipped.
struct GroupDecimator {
    int factor = 1, taps = 0;
    float_4 kernel[fuzzy::kMaxResampleTaps / 4];
    float ring[4][2 * fuzzy::kMaxResampleTaps];
    int index = 0;

    GroupDecimator() { reset(); }

    void setFactor(int newFactor) {
        factor = newFactor;
        taps = factor * fuzzy::kResampleQuality;
        if (factor > 1) {
            // The mono resampler designs the kernel; copy it.
            fuzzy::Resampler design;
            design.setFactor(factor);
            for (int i = 0; i < taps / 4; ++i) kernel[i] = design.downKernel[i];
        }
        reset();
    }
    void reset() {
        std::memset(ring, 0, sizeof(ring));
        index = 0;
    }
    void clearLane(int lane) { std::memset(ring[lane], 0, sizeof(ring[lane])); }

    // `factor` substeps in (oldest first), one base sample out per lane.
    inline float_4 down(const float_4* in, int lanes) {
        if (factor == 1) return in[0];
        float samples[fuzzy::kMaxOversample][4];
        for (int s = 0; s < factor; ++s) { float_4 value = in[s]; value.store(samples[s]); }
        for (int s = 0; s < factor; ++s) {
            if (--index < 0) index += taps;
            for (int lane = 0; lane < lanes; ++lane) {
                ring[lane][index] = samples[s][lane];
                ring[lane][index + taps] = samples[s][lane];
            }
        }
        float result[4] = {};
        for (int lane = 0; lane < lanes; ++lane) {
            const float* window = &ring[lane][index];
            float_4 acc = float_4(0.f);
            for (int i = 0; i < taps / 4; ++i) acc += float_4::load(window + 4 * i) * kernel[i];
            result[lane] = acc[0] + acc[1] + acc[2] + acc[3];
        }
        return float_4::load(result);
    }
};

struct GroupControls {
    double frequencyA[4] = { 32.703, 32.703, 32.703, 32.703 }, frequencyB[4] = { 32.703, 32.703, 32.703, 32.703 };
    float_4 weightA = 0.f, weightB = 0.f;
    float_4 cutoffHz = 100.f;
    float_4 emphasis = 0.f;
    float_4 loudness = 0.f;
    float_4 level = 1.f;
    float_4 filterFm = 0.f;
    float_4 subBlend = 0.f;
    float crossFm[4] = {};
    float_4 aux = 0.f;
};

struct VoiceGroup {
    Saw sawA[4], sawB[4];
    LadderGroup ladder;
    GroupDecimator down;
    fuzzy::NoiseSource noise[4];
    int   oversample = 2;
    float rate = 96000.f;
    float inputHighPassCoeff = 1.f, gLoad = 0.f, lowCoeff = 0.f, outputHighPassCoeff = 1.f;
    float_4 inputHighPassState = 0.f, inputHighPassLast = 0.f;
    float_4 outputHighPassState = 0.f, outputHighPassLast = 0.f;
    float_4 lastCutoffG = 0.f, lastLoudness = 0.f, lastAux = 0.f;
    double subPhase[4] = {};

    // Each lane's noise from its own seed; voice 0 keeps the mono voice's.
    void seedNoise(int firstVoice) {
        for (int lane = 0; lane < 4; ++lane)
            if (firstVoice + lane > 0) noise[lane].seed(0x51u + (uint32_t)(firstVoice + lane));
    }

    void setRate(float baseRate, int factor) {
        oversample = factor;
        rate = baseRate * factor;
        down.setFactor(factor);
        inputHighPassCoeff = std::exp(-2.f * 3.14159265f * kInputHighPassHz / rate);
        gLoad = std::tan(3.14159265f * kLoadHz / rate);
        lowCoeff = 1.f - std::exp(-2.f * 3.14159265f * kFeedbackHighPassHz / rate);
        outputHighPassCoeff = std::exp(-2.f * 3.14159265f * kOutputHighPassHz / baseRate);
        reset();
    }

    void reset() {
        ladder.reset();
        down.reset();
        inputHighPassState = inputHighPassLast = 0.f;
        outputHighPassState = outputHighPassLast = 0.f;
        lastCutoffG = lastLoudness = lastAux = 0.f;
        for (int lane = 0; lane < 4; ++lane) subPhase[lane] = 0.5 * sawA[lane].phase;
    }

    // Clears one voice, for a voice that has just come into use.
    void resetLane(int lane) {
        for (int k = 0; k < 4; ++k) {
            clearLane(ladder.state[k], lane); clearLane(ladder.rung[k], lane); clearLane(ladder.rungPrevious[k], lane);
        }
        clearLane(ladder.loadState, lane); clearLane(ladder.load, lane); clearLane(ladder.loadLow, lane); clearLane(ladder.loadPrevious, lane);
        down.clearLane(lane);
        clearLane(inputHighPassState, lane); clearLane(inputHighPassLast, lane);
        clearLane(outputHighPassState, lane); clearLane(outputHighPassLast, lane);
        clearLane(lastCutoffG, lane); clearLane(lastLoudness, lane); clearLane(lastAux, lane);
        subPhase[lane] = 0.5 * sawA[lane].phase;
    }

    // One base sample for four voices, volts at the AUD pot (before the
    // module's output gain). Lanes from `lanes` on are unused: their cutoff
    // warp, saws and noise are skipped, and they stay silent.
    float_4 process(const GroupControls& c, int lanes = 4) {
        float_4 cutoff = rack::simd::fmin(c.cutoffHz, float_4(0.45f * rate));
        float cutoffLanes[4];
        cutoff.store(cutoffLanes);
        for (int lane = 0; lane < 4; ++lane) cutoffLanes[lane] = (lane < lanes) ? std::tan(3.14159265f * cutoffLanes[lane] / rate) : 0.01f;
        const float_4 cutoffG = float_4::load(cutoffLanes);
        const float_4 startG = rack::simd::ifelse(lastCutoffG > 0.f, lastCutoffG, cutoffG);
        const float_4 stepG = (cutoffG - startG) / (float)oversample;
        const float_4 stepLoudness = (c.loudness - lastLoudness) / (float)oversample;
        const float_4 stepAux = (c.aux - lastAux) / (float)oversample;
        lastCutoffG = cutoffG;

        double incrementA[4], incrementB[4];
        float inverseA[4], inverseB[4], noiseLanes[4] = {};
        for (int lane = 0; lane < lanes; ++lane) {
            incrementA[lane] = c.frequencyA[lane] / rate;
            incrementB[lane] = c.frequencyB[lane] / rate;
            inverseA[lane] = (float)(1.0 / incrementA[lane]);
            inverseB[lane] = (float)(1.0 / incrementB[lane]);
            noiseLanes[lane] = kLadderNoise * noise[lane].gaussian();
        }
        const float_4 noiseLevel = float_4::load(noiseLanes);
        const float_4 mixScale = kSawVolts * c.level;
        const float inputScale = kInputDivider / kTwoVt;
        const float_4 vcaDrive = kA501Volts * kVcaDivider / kTwoVt * c.level;
        const float vcaNormalize = kTwoVt / (kA501Volts * kVcaDivider);
        const float width = 1.5f * (kSwitchRailVolts - kSwitchKneeVolts);

        float_4 g = startG, loudness = lastLoudness, aux = lastAux;
        float_4 substeps[fuzzy::kMaxOversample];
        for (int s = 0; s < oversample; ++s) {
            g += stepG;
            loudness += stepLoudness;
            aux += stepAux;
            float sawLanesA[4] = {}, sawLanesB[4] = {}, subLanes[4] = {};
            for (int lane = 0; lane < lanes; ++lane) {
                sawLanesB[lane] = sawB[lane].process(incrementB[lane], inverseB[lane]);
                // SNARL's divider and sub-octave (see Voice).
                subPhase[lane] += 0.5 * incrementA[lane];
                if (subPhase[lane] >= 1.0) subPhase[lane] -= 1.0;
                const float subRamp = 2.f * (float)subPhase[lane] - 1.f;
                subLanes[lane] = subRamp + kSnarlSubSquare * ((subPhase[lane] < 0.5 ? -1.f : 1.f) - subRamp);
                sawLanesA[lane] = (c.crossFm[lane] > 0.f)
                    ? sawA[lane].processThroughZero(incrementA[lane] * (1.0 + c.crossFm[lane] * subLanes[lane]))
                    : sawA[lane].process(incrementA[lane], inverseA[lane]);
            }
            // AUX IN joins the saws at the mixer, at the GRIT level.
            const float_4 sawValuesB = float_4::load(sawLanesB);
            float_4 mix = (c.weightA * float_4::load(sawLanesA) + c.weightB * sawValuesB) * mixScale
                        + aux * c.level;
            // IC801's ceiling: linear to the knee, a cubic into the rail.
            const float_4 magnitude = rack::simd::fabs(mix);
            const float_4 over = rack::simd::fmin(magnitude - kSwitchKneeVolts, float_4(width));
            const float_4 shaped = kSwitchKneeVolts + over - over * over * over / (3.f * width * width);
            mix = rack::simd::ifelse(magnitude > kSwitchKneeVolts, rack::simd::ifelse(mix < 0.f, -shaped, shaped), mix);
            // C501 into the bottom pair's base.
            inputHighPassState = inputHighPassCoeff * (inputHighPassState + mix - inputHighPassLast);
            inputHighPassLast = mix;
            // SNARL, filter side: osc B's saw crossfading to the sub-octave
            // swings the cutoff exponentially, inverted (see Voice).
            const float_4 filterModulator = sawValuesB + c.subBlend * (float_4::load(subLanes) - sawValuesB);
            const float_4 modulatedG = rack::simd::fmin(g * rack::dsp::exp2_taylor5(-c.filterFm * filterModulator), float_4(kMaxCutoffG));
            const float_4 load = ladder.process(inputHighPassState * inputScale + noiseLevel, modulatedG, gLoad, c.emphasis, lowCoeff);
            // A301: the volume contour times its input pair's split.
            substeps[s] = loudness * (pairLaw4(load * vcaDrive) * vcaNormalize);
        }
        lastLoudness = c.loudness;
        lastAux = c.aux;
        const float_4 out = down.down(substeps, lanes);
        // C306 into the AUD pot.
        outputHighPassState = outputHighPassCoeff * (outputHighPassState + out - outputHighPassLast);
        outputHighPassLast = out;
        return outputHighPassState;
    }
};

} // namespace vega