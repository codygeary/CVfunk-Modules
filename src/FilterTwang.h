////////////////////////////////////////////////////////////
//
//   FilterTwang.h
//
//   written by Cody Geary
//   Copyright 2026
//   Released under the MIT License
//
//   DSP primitives for the Twang polyphonic string synthesizer.
//
//   Self-contained on purpose: this header duplicates the pieces it
//   needs rather than including FilterStrands.h, so the MIT module has
//   no dependency on the all-rights-reserved Sands Collection sources.
//
//   v2: the string voice is 4-lane SIMD (TwangStringSIMD). All lanes are
//   symmetric -- same coefficients, same physics, only pitch and pluck
//   state differ per lane. MIDI hands out poly channels in arbitrary
//   order, so no lane may carry a baked-in identity.
//
//   v3: the string is a TWO-span waveguide split at one moving junction.
//   While bowing, the junction sits at the bow contact point (friction
//   excitation, Strands-style) and the pluck point does not exist in the
//   loop at all. A pluck trigger moves the junction exactly to the pluck
//   point, lifts the bow (junction goes transparent, no grip), and injects
//   the pluck impulse there, so bowing and plucking are two different
//   physical configurations of the same loop -- crossfaded by moving the
//   split point -- instead of a permanent three-span split. The hollow
//   body (FDN resonator) now rings in the musical decay range and re-
//   drives the strings through the bridge, closing the string<->body loop.
//
////////////////////////////////////////////////////////////

#pragma once
#include <cmath>
#include <cstdint>
#include <vector>
#include <algorithm>
#include "rack.hpp"

using float_4 = rack::simd::float_4;

// --------------------------------------------------------------------------
// Small math helpers
// --------------------------------------------------------------------------
inline float twangWrapToPi(float x) {
    const float twoPi = 2.0f * (float)M_PI;
    x = fmodf(x + (float)M_PI, twoPi);
    if (x < 0.f) x += twoPi;
    return x - (float)M_PI;
}

inline float twangCos(float x) {
    x = twangWrapToPi(x);
    float x2 = x * x;
    return 1.f - x2 * (0.5f - x2 * (1.f/24.f - x2 * (1.f/720.f - x2 / 40320.f)));
}

// Four-point Lagrange interpolation, for fractional delay reads.
inline float twangLagrange(float y0, float y1, float y2, float y3, float t) {
    return ((-t * (t-1.f) * (t-2.f)) / 6.f) * y0
         + (((t+1.f) * (t-1.f) * (t-2.f)) / 2.f) * y1
         + ((-(t+1.f) * t * (t-2.f)) / 2.f) * y2
         + (((t+1.f) * t * (t-1.f)) / 6.f) * y3;
}

// SIMD twin of twangLagrange -- identical polynomial, 4 lanes at once.
inline float_4 twangLagrangeSIMD(float_4 y0, float_4 y1, float_4 y2, float_4 y3, float_4 t) {
    return ((-t * (t-1.f) * (t-2.f)) / 6.f) * y0
         + (((t+1.f) * (t-1.f) * (t-2.f)) / 2.f) * y1
         + ((-(t+1.f) * t * (t-2.f)) / 2.f) * y2
         + (((t+1.f) * t * (t-1.f)) / 6.f) * y3;
}

// --------------------------------------------------------------------------
// TwangDCBlocker
// --------------------------------------------------------------------------
struct TwangDCBlocker {
    float x1 = 0.f, y1 = 0.f, R = 0.9995f;
    void setSampleRate(float sr) {
        R = rack::clamp(1.f - 2.f * float(M_PI) * 20.f / sr, 0.990f, 0.9999f);
    }
    inline float process(float x) {
        y1 = x - x1 + R * y1;
        x1 = x;
        return y1;
    }
    void reset() { x1 = 0.f; y1 = 0.f; }
};

// --------------------------------------------------------------------------
// TwangOnePoleLPFSIMD
// 4-lane one-pole. Used as the bridge tone filter inside each string quad
// (TONE, already darkened by MUTE).
// --------------------------------------------------------------------------
struct TwangOnePoleLPFSIMD {
    float_4 z = float_4(0.f), coeff = float_4(0.f);
    inline void setCoeff(float_4 c) {
        coeff = rack::simd::clamp(c, float_4(0.f), float_4(0.9999f));
    }
    inline float_4 process(float_4 x) {
        z = coeff * z + (float_4(1.f) - coeff) * x;
        return z;
    }
    void reset() { z = float_4(0.f); }
};

// --------------------------------------------------------------------------
// TwangAllpass1SIMD
// First-order allpass, 4 lanes. Cascaded in the string loop to model
// stiffness dispersion: high partials travel faster than low ones, which is
// what makes a thick/stiff string sound inharmonic and "clangy" rather than
// perfectly harmonic. coeff 0 = no dispersion (ideal flexible string).
// --------------------------------------------------------------------------
struct TwangAllpass1SIMD {
    float_4 x1 = float_4(0.f), y1 = float_4(0.f);
    float_4 coeff = float_4(0.f);

    inline float_4 process(float_4 x) {
        float_4 y = coeff * (x - y1) + x1;
        x1 = x;
        y1 = y;
        return y;
    }
    void reset() { x1 = float_4(0.f); y1 = float_4(0.f); }
};

// --------------------------------------------------------------------------
// TwangADAADrive
// Anti-derivative anti-aliased tanh saturator. Scalar: this sits after the
// voice sum, once per output channel, not inside any string loop.
// --------------------------------------------------------------------------
// 4-lane versions of the two scalar stages, for the polyphonic output path.
// Shared coefficients, per-lane state -- the same pattern as the mode banks.
struct TwangDCBlockerSIMD {
    float_4 x1 = float_4(0.f), y1 = float_4(0.f);
    float   R  = 0.9995f;
    void setSampleRate(float sr) {
        R = rack::clamp(1.f - 2.f * float(M_PI) * 20.f / sr, 0.990f, 0.9999f);
    }
    inline float_4 process(float_4 x) {
        y1 = x - x1 + float_4(R) * y1;
        x1 = x;
        return y1;
    }
    void reset() { x1 = y1 = float_4(0.f); }
};

struct TwangADAADriveSIMD {
    float_4 lastInput = float_4(0.f);

    static float_4 polyTanh(float_4 x) {
        float_4 x2 = x * x;
        return x - x * x2 * (float_4(1.f/3.f) - x2 * (float_4(2.f/15.f)
                                                     - float_4(17.f/315.f) * x2));
    }
    static float_4 antiderivative(float_4 x) {
        float_4 x2 = x * x, x4 = x2 * x2;
        return float_4(0.5f)*x2 - float_4(1.f/12.f)*x4 + float_4(1.f/45.f)*x4*x2
             - float_4(17.f/2520.f)*x4*x4;
    }
    float_4 process(float_4 inV, float driveGain) {
        float_4 sig  = rack::simd::clamp(inV * float_4(driveGain),
                                         float_4(-13.14f), float_4(13.14f));
        float_4 norm = sig * float_4(0.1f);
        float_4 d    = norm - lastInput;
        // Branchless equivalent of the scalar version's small-delta guard.
        float_4 safe = rack::simd::ifelse(rack::simd::fabs(d) > float_4(1e-6f),
                                          d, float_4(1.f));
        float_4 quot = (antiderivative(norm) - antiderivative(lastInput)) / safe;
        float_4 out  = rack::simd::ifelse(rack::simd::fabs(d) > float_4(1e-6f),
                                          quot, polyTanh(norm));
        lastInput = norm;
        return rack::simd::clamp(out * float_4(6.9f), float_4(-10.f), float_4(10.f));
    }
    void reset() { lastInput = float_4(0.f); }
};

struct TwangADAADrive {
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

    // The polynomial approximations are only accurate for small inputs,
    // so normalize into a safe range before applying them.
    float process(float inV, float driveGain) {
        float sig  = rack::clamp(inV * driveGain, -13.14f, 13.14f);
        float norm = sig / 10.f;
        float out  = applyADAA(norm, lastInput);
        lastInput  = norm;
        return rack::clamp(out * 6.9f, -10.f, 10.f);
    }

    void reset() { lastInput = 0.f; }
};

// --------------------------------------------------------------------------
// TwangRailSIMD
// 4-lane one-directional delay segment. All 4 lanes are always read and
// written together once per sample, so a single shared writeIndex/bufSize/
// bufMask covers every lane -- only the sample buffers are per-lane.
//
// The buffer reads themselves are inherently scalar (each lane has its own
// fractional read position and there is no SIMD gather here), so read() and
// write() do 4 scalar buffer accesses each. What vectorizes is the
// surrounding math: the Lagrange weights, clamp, and floor that would
// otherwise be repeated four separate times.
// --------------------------------------------------------------------------
struct TwangRailSIMD {
    static constexpr int LANES = 4;

    std::vector<float> buf[LANES];
    int bufSize    = 0;
    int bufMask    = 0;
    int writeIndex = 0;

    void init(float sr, float maxDelaySec = 0.065f) {
        int needed = (int)ceilf(maxDelaySec * sr) + 8;
        bufSize = 1;
        while (bufSize < needed) bufSize <<= 1;
        bufMask = bufSize - 1;
        for (int lane = 0; lane < LANES; ++lane)
            buf[lane].assign(bufSize, 0.f);
        writeIndex = 0;
    }

    inline float_4 read(float_4 delaySamples) const {
        float_4 ds      = rack::simd::clamp(delaySamples, float_4(1.f), float_4((float)bufSize - 4.f));
        float_4 rp      = float_4((float)writeIndex) - ds;
        float_4 rpFloor = rack::simd::floor(rp);
        float_4 frac    = rp - rpFloor;

        float y0[LANES], y1[LANES], y2[LANES], y3[LANES];
        for (int lane = 0; lane < LANES; ++lane) {
            int base = ((int)rpFloor[lane]) & bufMask;
            y0[lane] = buf[lane][(base - 1) & bufMask];
            y1[lane] = buf[lane][base       & bufMask];
            y2[lane] = buf[lane][(base + 1) & bufMask];
            y3[lane] = buf[lane][(base + 2) & bufMask];
        }
        return twangLagrangeSIMD(float_4::load(y0), float_4::load(y1),
                                 float_4::load(y2), float_4::load(y3), frac);
    }

    // Single-lane scalar read, for the display only. Not used in the audio path.
    inline float readLane(int lane, float delaySamples) const {
        delaySamples = rack::clamp(delaySamples, 1.f, (float)bufSize - 4.f);
        float rp   = (float)writeIndex - delaySamples;
        int   base = ((int)floorf(rp)) & bufMask;
        float frac = rp - floorf(rp);
        float y0   = buf[lane][(base - 1) & bufMask];
        float y1   = buf[lane][base       & bufMask];
        float y2   = buf[lane][(base + 1) & bufMask];
        float y3   = buf[lane][(base + 2) & bufMask];
        return twangLagrange(y0, y1, y2, y3, frac);
    }

    inline void write(float_4 value) {
        float arr[LANES];
        value.store(arr);
        for (int lane = 0; lane < LANES; ++lane)
            buf[lane][writeIndex] = arr[lane];
        writeIndex = (writeIndex + 1) & bufMask;
    }

    void clear() {
        for (int lane = 0; lane < LANES; ++lane)
            std::fill(buf[lane].begin(), buf[lane].end(), 0.f);
        writeIndex = 0;
    }
};

// --------------------------------------------------------------------------
// TwangBowJunctionSIMD
// Scattering-junction friction model, 4 lanes. The friction curve is shaped
// as a REFLECTION COEFFICIENT, not a rising force curve: flat near zero
// relative velocity (string stuck to the bow hair), falling off once the
// string breaks away and slips.
//
// The scalar "if (magnitude <= captureVelocity)" becomes a branchless
// select: both the sticking and slipping values are computed for all lanes,
// then picked per-lane with ifelse. A divide-by-near-zero in a lane that
// gets masked out is harmless, since ifelse's bitwise select zeroes those
// bits whether they held Inf, NaN, or a normal float.
//
// stickingFriction and captureVelocity are driven by PRESSURE;
// stringImpedance and slipFalloff by the STRING IMPEDANCE and SLIP sliders.
// All four are the same across every lane.
// --------------------------------------------------------------------------
struct TwangBowJunctionSIMD {
    float_4 stringImpedance  = float_4(1.f);
    float_4 captureVelocity  = float_4(0.2f);
    float_4 stickingFriction = float_4(6.0f);
    float_4 slipFalloff      = float_4(20.0f);
    // How hard the bow drives the string. The friction correction is the
    // energy the bow puts in every sample, and a bowed note builds it up over
    // many round trips, so this sets the sustained level -- a plucked note
    // gets one injection and decays from there, and needs none of it.
    float_4 bowLevel = float_4(1.f);

    inline float_4 frictionTable(float_4 differentialVelocity) const {
        float_4 magnitude = rack::simd::fabs(differentialVelocity);
        float_4 excess    = magnitude - captureVelocity;
        float_4 slipping  = stickingFriction / (float_4(1.f) + excess * slipFalloff);
        float_4 stuck     = magnitude <= captureVelocity;
        return rack::simd::ifelse(stuck, stickingFriction, slipping);
    }

    // The same correction goes into both outgoing waves -- injecting it
    // asymmetrically produces a lopsided, unstable junction.
    //
    // "Left" is toward the nut, "right" toward the bridge, relative to this
    // junction's position on the string. A wave arriving from the left
    // continues out to the right and vice versa.
    //
    // engagement scales the friction: 1 = bow on the hair, 0 = bow lifted
    // (e.g. while a pluck is sounding on that lane). At 0 the junction is a
    // transparent pass-through, so the string reads as one continuous span
    // -- which is exactly what a lifted bow should do.
    inline void process(float_4 incomingFromLeft, float_4 incomingFromRight,
                        float_4 bowSpeed, float_4 engagement,
                        float_4& outgoingToRight, float_4& outgoingToLeft) const {

        float_4 stringVelocity       = incomingFromLeft + incomingFromRight;
        float_4 differentialVelocity = bowSpeed - stringVelocity;
        float_4 frictionCoeff        = frictionTable(differentialVelocity);

        // Friedlander-Keller normalization (impedance ratio -> reflection).
        float_4 impedanceRatio = 0.25f * frictionCoeff / stringImpedance;
        float_4 reflection     = impedanceRatio / (float_4(1.f) + impedanceRatio);
        float_4 correction     = reflection * differentialVelocity * engagement * bowLevel;

        outgoingToRight = incomingFromLeft + correction;
        outgoingToLeft  = incomingFromRight + correction;
    }

    void reset() {}
};

// --------------------------------------------------------------------------
// TwangPluckExciter
// One-shot excitation, entirely separate from the bow junction: a pluck is
// an injected impulse, not a continuous friction coupling, so nothing here
// touches the junction at all.
//
// Shape is a single raised-cosine (Hann) lobe whose width IS the hardness
// control -- a hard pick has near-zero contact time (short lobe, broadband)
// while a soft fingertip has a longer contact time (wide lobe, naturally
// lowpassed before it ever reaches the string). One axis covers the whole
// fingertip-to-plectrum range, so no separate attack-time parameter is
// needed.
//
// Deliberately kept scalar, one instance per lane, rather than vectorized:
// each lane triggers at its own moment with its own width and amplitude, so
// a SIMD version would need per-lane writes into a float_4 on every trigger.
// The envelope itself is a handful of operations and only runs during the
// few milliseconds a pluck is active, so there is very little to win here.
// --------------------------------------------------------------------------
struct TwangPluckExciter {
    bool  active       = false;
    float pulsePhase   = 0.f;    // 0..1 across the pulse duration
    float widthSamples = 32.f;
    float amplitude    = 0.f;

    // Spectral tilt. The Hann lobe alone is a FORCE pulse, which is nearly
    // flat in the band that matters for a hard pick -- measured spectral
    // centroid 2607 Hz at hardness 1.0, i.e. a click with a string behind
    // it. A real string is released from a triangular displacement, whose
    // spectrum falls at -6 dB/octave. Leaky-integrating the lobe supplies
    // that tilt (centroid drops to 1199 Hz at tilt 0.6), while soft plucks,
    // already dark because of their long contact time, barely move.
    // tilt 0 = the original force pulse, 1 = full displacement tilt.
    float tilt         = 0.6f;
    float integrator   = 0.f;
    float leakCoeff    = 0.9958f;   // ~5 ms, set from sample rate in trigger()
    float integGain    = 1.f;       // 2/widthSamples: keeps the peak at `velocity`

    void reset() {
        active     = false;
        pulsePhase = 0.f;
        amplitude  = 0.f;
        integrator = 0.f;
    }

    // hardness: 0 = soft fingertip (long contact), 1 = hard pick (short contact).
    // velocity: 0..1, scales amplitude. Retriggering mid-pulse simply restarts
    // the lobe -- a new pluck on a still-ringing string superimposes on whatever
    // is already traveling in the delay lines, which is the physically correct
    // behaviour and needs no special case.
    inline void trigger(float hardness, float velocity, float sampleRate) {
        // Contact time: 4.0ms (pad of a finger) down to 0.25ms (rigid pick).
        // Tune: widen the range for more extreme pluck characters.
        float contactMs = 4.0f - 3.75f * rack::clamp(hardness, 0.f, 1.f);
        widthSamples = fmaxf(contactMs * 0.001f * sampleRate, 2.f);
        amplitude    = velocity;
        pulsePhase   = 0.f;
        active       = true;
        leakCoeff    = expf(-1.f / fmaxf(0.005f * sampleRate, 1.f));
        // The integral of a unit Hann lobe is widthSamples/2, so this keeps
        // the tilted pulse at the same peak height as the raw one and the
        // hardness control stays a pure timbre axis.
        integGain    = 2.f / widthSamples;
    }

    inline float process() {
        // The integrator keeps running for a few ms after the lobe ends --
        // that decaying tail IS the displacement step, and the string's
        // inverting nut reflection cancels it over one round trip.
        if (!active && fabsf(integrator) < 1e-7f) {
            integrator = 0.f;
            return 0.f;
        }

        float envelope = 0.f;
        if (active) {
            envelope = 0.5f * (1.f - twangCos(2.f * float(M_PI) * pulsePhase));
            pulsePhase += 1.f / widthSamples;
            if (pulsePhase >= 1.f) {
                active     = false;
                pulsePhase = 0.f;
            }
        }

        float raw   = envelope * amplitude;
        integrator  = leakCoeff * integrator + raw;
        float tilted = integrator * integGain;
        return raw + tilt * (tilted - raw);
    }
};

// --------------------------------------------------------------------------
// Mode table: (frequency ratio to the lowest mode, relative amplitude, Q).
//
// Ratios are irregular ON PURPOSE -- that irregularity is the whole
// difference between a body and a comb. Do not "tidy" them toward integers.
// --------------------------------------------------------------------------
static constexpr int TWANG_MODE_COUNT = 7;
struct TwangModeSpec { float ratio, amp, Q; };
static const TwangModeSpec TWANG_MODES[TWANG_MODE_COUNT] = {
    { 1.00f, 1.00f, 22.f },   // A0 / Helmholtz air mode -- the "boom"
    { 1.95f, 0.80f, 26.f },   // T(1,1)2 top plate
    { 2.75f, 0.55f, 30.f },   // back plate
    { 4.05f, 0.45f, 34.f },   // T(2,1)
    { 5.60f, 0.34f, 38.f },
    { 7.90f, 0.26f, 42.f },
    { 11.5f, 0.20f, 40.f },
};

// The bridge admittance filter inside the string loop only needs the modes
// that actually overlap the low partials; the rest cost SIMD biquads for no
// audible return.
static constexpr int TWANG_ADMIT_MODES = 5;

// --------------------------------------------------------------------------
// twangResonatorCoeffs
// Steiglitz constant-peak-gain two-pole resonator:
//
//     H(z) = b0 * (1 - z^-2) / (1 + a1 z^-1 + a2 z^-2)
//
// with b0 = (1-R^2)/2, a1 = -2R cos(theta), a2 = R^2, and theta chosen so
// the peak is EXACTLY 1.0 at f0 for any Q. That exactness matters here: the
// bank is summed and also subtracted from the string's reflection, so a
// resonator whose peak drifts with Q would make both the body level and the
// coupling's stability margin depend on the MATERIAL knob.
//
// Verified numerically: peak |H| = 1.0000 for every mode in the table across
// the full size range.
// --------------------------------------------------------------------------
inline void twangResonatorCoeffs(float f0, float Q, float sr,
                                 float& b0, float& a1, float& a2) {
    // 1 Hz floor rather than 20: the body never uses anything that low, but
    // the twang bounce is a pitch envelope and lives down at 5-35 Hz.
    f0 = rack::clamp(f0, 1.f, sr * 0.45f);
    Q  = rack::clamp(Q, 0.5f, 400.f);

    float w0 = 2.f * float(M_PI) * f0 / sr;
    float BW = f0 / Q;                                  // -3 dB bandwidth, Hz
    float R  = expf(-float(M_PI) * BW / sr);            // pole radius

    // cos(theta) can exceed 1 for very low f0 at high Q; clamping puts the
    // pole on the real axis, which is stable and inaudible down there.
    float c     = rack::clamp((1.f + R * R) / (2.f * R) * cosf(w0), -1.f, 1.f);
    float theta = acosf(c);

    b0 = (1.f - R * R) * 0.5f;
    a1 = -2.f * R * cosf(theta);
    a2 = R * R;
}

// --------------------------------------------------------------------------
// TwangModeBiquad -- scalar, for the mono body bank.
// --------------------------------------------------------------------------
struct TwangModeBiquad {
    float b0 = 0.f, a1 = 0.f, a2 = 0.f;
    float x1 = 0.f, x2 = 0.f, y1 = 0.f, y2 = 0.f;

    void set(float f0, float Q, float sr) { twangResonatorCoeffs(f0, Q, sr, b0, a1, a2); }

    // Transposed direct form II would need b1/b2 storage; the numerator here
    // is b0*(1 - z^-2), so direct form I with two x taps is cheaper.
    inline float process(float x) {
        float y = b0 * (x - x2) - a1 * y1 - a2 * y2;
        x2 = x1; x1 = x;
        y2 = y1; y1 = y;
        return y;
    }
    void reset() { x1 = x2 = y1 = y2 = 0.f; }
};

// --------------------------------------------------------------------------
// TwangModeBiquadSIMD -- 4 lanes, ONE set of coefficients.
//
// Every lane sees the same instrument body, so the coefficients are shared
// and only the filter state is per-lane. This is what makes the per-string
// bridge admittance affordable: 4 quads x 5 modes = 20 SIMD biquads/sample.
// --------------------------------------------------------------------------
struct TwangModeBiquadSIMD {
    float_4 b0 = float_4(0.f), a1 = float_4(0.f), a2 = float_4(0.f);
    float_4 x1 = float_4(0.f), x2 = float_4(0.f);
    float_4 y1 = float_4(0.f), y2 = float_4(0.f);

    void set(float f0, float Q, float sr) {
        float sb0, sa1, sa2;
        twangResonatorCoeffs(f0, Q, sr, sb0, sa1, sa2);
        b0 = float_4(sb0); a1 = float_4(sa1); a2 = float_4(sa2);
    }

    inline float_4 process(float_4 x) {
        float_4 y = b0 * (x - x2) - a1 * y1 - a2 * y2;
        x2 = x1; x1 = x;
        y2 = y1; y1 = y;
        return y;
    }
    void reset() {
        x1 = x2 = y1 = y2 = float_4(0.f);
    }
};

// --------------------------------------------------------------------------
// TwangBridgeAdmittanceSIMD
//
// The realism change. A real string does not terminate in a fixed lowpass:
// it terminates in the body's driving-point admittance. Partials that
// coincide with a body mode dump their energy into the box and die quickly;
// partials between modes ring on. That uneven decay is most of what
// separates a physical model from a synthesizer, and a single one-pole
// bridge filter cannot produce it at all.
//
// Applied as a LOSS on the reflected wave, not as a feedback injection:
//
//     reflected = -(toned * loopGain - k * admittance(atBridge))
//
// so there is no string->body->string loop to go unstable. The bank is a
// bandpass (zero magnitude at DC and Nyquist), so it can only remove energy
// where the body actually resonates.
//
// Stability, swept over every tone setting, body size and coupling value:
// the coupling never raises |r| above the uncoupled maximum of 0.995 for
// k <= 0.7. At k = 1.0 it reaches 1.006 (the bank's phase is +-90 degrees
// off resonance, so the subtraction can add in quadrature). Hence the 0.7
// clamp in setCoupling().
// --------------------------------------------------------------------------
struct TwangBridgeAdmittanceSIMD {
    TwangModeBiquadSIMD mode[TWANG_ADMIT_MODES];
    float_4 amp[TWANG_ADMIT_MODES];
    float_4 coupling = float_4(0.f);

    TwangBridgeAdmittanceSIMD() {
        for (int i = 0; i < TWANG_ADMIT_MODES; ++i) amp[i] = float_4(TWANG_MODES[i].amp);
    }

    // lowestModeHz and qScale come from RESONATOR SIZE / MATERIAL, so the
    // string's termination tracks the body the user is actually hearing.
    void setBody(float lowestModeHz, float qScale, float sr) {
        for (int i = 0; i < TWANG_ADMIT_MODES; ++i) {
            mode[i].set(lowestModeHz * TWANG_MODES[i].ratio,
                        TWANG_MODES[i].Q * qScale, sr);
            amp[i] = float_4(TWANG_MODES[i].amp);
        }
    }

    void setCoupling(float k) { coupling = float_4(rack::clamp(k, 0.f, 0.7f)); }

    inline float_4 process(float_4 atBridge) {
        float_4 sum = float_4(0.f);
        for (int i = 0; i < TWANG_ADMIT_MODES; ++i)
            sum += amp[i] * mode[i].process(atBridge);
        return coupling * sum;
    }

    void reset() { for (int i = 0; i < TWANG_ADMIT_MODES; ++i) mode[i].reset(); }
};

// --------------------------------------------------------------------------
// TwangStringSIMD
// Four complete string voices in lockstep. Each string is a bidirectional
// waveguide split at ONE moving junction point P between the nut (0) and
// the bridge (1), giving two spans. A bridge termination chain (tone LPF
// and a small fixed stiffness dispersion) and a rigid nut close the loop.
//
// The same junction is the whole excitation story, and it MOVES between two
// physical configurations:
//   BOWING:   P sits at the bow contact point; the junction is a friction
//             scattering junction (Strands-style) driven by the bow speed.
//             The pluck point is NOT in the loop, so it cannot bias which
//             harmonics the bow can excite.
//   PLUCKING: a pluck trigger jumps P exactly to the pluck point, lifts
//             the bow (bowLift -> 1, junction becomes a transparent
//             pass-through with no grip), and injects the pluck impulse
//             there. The characteristic pluck spectrum (harmonics with a
//             node at the pluck point cancelled) emerges from the
//             reflections. As bowLift decays back to 0 the junction glides
//             back to the bow position while the bow re-grips, so bowing
//             resumes on a still-ringing string.
//
// Moving P only re-stitches the two delay spans at a different point of the
// SAME continuous string -- no wave is disturbed -- which is what makes
// the crossfade between the two models glitch-free.
//
// Every lane is architecturally identical. All coefficients are broadcast
// across the quad; only the segment lengths (pitch), pluck state, bow lift,
// junction position, twang transients, and safety-limiter history vary per
// lane.
// --------------------------------------------------------------------------
struct TwangStringSIMD {
    static constexpr int LANES = 4;
    static constexpr int DISPERSION_STAGES = 2;

    // How long a plucked lane's bow stays off the string before it re-grips
    // (and how long the junction stays parked at the pluck point).
    static constexpr float BOW_LIFT_TIME = 0.15f;   // seconds

    // Two one-way delay spans: the string is split at the single moving
    // junction point P (nut = 0, bridge = 1):
    //   segA: nut <-> P,   segC: P <-> bridge
    // Each span is carried by a pair of rails (one per direction).
    TwangRailSIMD nutToP, pToNut;        // segA (nut <-> P)
    TwangRailSIMD pToBridge, bridgeToP;  // segC (P <-> bridge)

    TwangBowJunctionSIMD bowJunction;
    TwangOnePoleLPFSIMD  bridgeTone;     // TONE + MUTE darkening
    // The body's driving-point admittance, seen from the bridge. A real
    // string does not terminate in a fixed lowpass; partials that land on a
    // body mode drain into the box and die fast while partials between modes
    // ring on. That uneven decay is most of what separates a physical model
    // from a synthesizer, and no single-pole bridge filter can produce it.
    // Applied as a LOSS on the reflected wave, so there is no
    // string->body->string loop and nothing to destabilize.
    TwangBridgeAdmittanceSIMD bridgeAdmittance;
    // Small FIXED stiffness: real strings are slightly inharmonic. Kept
    // constant (not a parameter) so it adds "stringy" character without
    // being another brightness knob.
    TwangAllpass1SIMD    dispersion[DISPERSION_STAGES];
    TwangPluckExciter    pluck[LANES];   // scalar per lane, see note above

    // Positions, broadcast across the quad (0 = nut, 1 = bridge).
    float_4 bowPosition   = float_4(0.8f);
    float_4 pluckPosition = float_4(0.3f);   // independent of bowPosition
    // The junction's CURRENT split point, per lane: at the bow position
    // while bowing, exactly at the pluck point the instant a pluck
    // triggers, then gliding back to the bow position as the lift decays.
    float_4 junctionPos   = float_4(0.8f);

    // Span lengths (one-way, in samples), per lane.
    float_4 segALen = float_4(100.f);
    float_4 segCLen = float_4(100.f);
    float_4 totalSegmentSamples = float_4(200.f);

    // Per-lane bow lift: 1 = bow fully lifted (junction transparent),
    // 0 = bow engaged. Set to 1 on a pluck trigger, decays back to 0
    // (re-engagement) over BOW_LIFT_TIME. This is the model's way of
    // enforcing "you can't bow and pluck at once".
    float_4 bowLift = float_4(0.f);
    float   bowLiftDecay = 0.999f;   // per-sample, set from sample rate in init
    // Per-sample glide rate for the junction returning to the bow position
    // after a pluck (~30 ms). The out-bound move is instant -- the trigger
    // jumps the junction exactly to the pluck point -- so only the return
    // glide uses this.
    float   junctionReturnCoeff = 0.01f;

    // Grip knee: the bow speed at which the string is half gripped.
    float   gripKnee     = 0.06f;

    // Per-round-trip loss, expressed as loss per SAMPLE of string length.
    //
    // A fixed per-round-trip loop gain is what made the playing threshold
    // pitch-dependent. The bow injects energy every sample, so a long (low)
    // string collects injection over many more samples per round trip than a
    // short one -- but with a constant loop gain both lose the same fraction
    // per round trip. Low notes therefore start easily and high notes need
    // far more bow, and decay time runs as 1/f (measured 16.8 s at E2 against
    // 2.1 s at E5 for loopGain 0.995).
    //
    // Making the loss proportional to length cancels both: loss per round
    // trip is now proportional to `total`, injection per round trip is
    // proportional to `total`, and their ratio -- which is what sets the
    // minimum playable bow speed -- no longer depends on pitch. Decay time
    // becomes pitch-independent too.
    float_4 lossPerSample = float_4(0.f);
    float_4 loopGain      = float_4(1.f);   // derived per lane in process()

    // Pluck/bow balance. A bow builds a standing wave over many round trips
    // and settles near injection/(1-loopGain); a pluck gets ONE injection and
    // decays from there. With both at unit amplitude the bow ends up tens of
    // times louder, which is not what either instrument does. This scales the
    // injected impulse only -- it changes the pluck's energy, not the string
    // physics.
    float_4 pluckDrive = float_4(1.f);

    // Safety limiter state, independent history per lane.
    float_4 drainGain   = float_4(1.f);
    float_4 safetyRMS   = float_4(0.f);
    float_4 safetyDecay = float_4(0.f);

    // TWANG: the amplitude-dependent pitch bend is a pure transient.
    // `energySmooth` is a cycle-mean smoothed detector on the bridge loop
    // energy (slow enough to average out the per-fundamental-cycle energy
    // pulses of a bowed note, so a sustained note has a FLAT reading and
    // bends nothing). `levelRef` tracks energySmooth with a slower attack
    // and a faster release; (energySmooth - levelRef) is a small positive
    // transient on a bow attack. `pluckKick` is set to the pluck velocity
    // on each pluck trigger and decays over ~120 ms: the sharp
    // "rubber-band" bend on the impact. Both go back to zero while the
    // string rings, so the resting pitch is never shifted.
    float_4 energySmooth = float_4(0.f);
    float_4 levelRef     = float_4(0.f);
    float_4 pluckKick    = float_4(0.f);

    // ---- tension bounce -------------------------------------------------
    // A monotonic decaying bend is what a stiff steel string does: tension
    // rises with the pluck and settles straight back. A slack, stretchy
    // string does not settle straight back. Its tension has real inertia, so
    // the restoring force overshoots and the string re-tensions itself
    // several times on the way down -- the pitch bends up, drops past its
    // resting point, comes back, and rings out. That is the "boing", and it
    // is a second-order response, not a first-order one.
    //
    // Two resonators at an irrational frequency ratio, so the ripples never
    // line up into a periodic LFO. Their output is signed: the negative
    // half-cycles are the pitch dipping BELOW the resting note, which is what
    // makes it read as a physical rebound rather than a decaying vibrato.
    TwangModeBiquadSIMD twangBounce1, twangBounce2;
    float_4 twangRing    = float_4(0.f);
    float_4 prevKick     = float_4(0.f);
    float   bounceNorm   = 0.f;      // impulse response peak -> ~1

    // Frequencies stay well below audio: this is a pitch envelope, not an
    // oscillator. Above about 40 Hz it stops being a wobble and turns into
    // sideband noise on the note.
    void setTwangBounce(float hz, float q, float sr) {
        hz = rack::clamp(hz, 0.5f, 60.f);
        q  = rack::clamp(q, 0.5f, 40.f);
        twangBounce1.set(hz,         q, sr);
        twangBounce2.set(hz * 2.71f, q * 0.7f, sr);
        // A constant-peak-gain resonator has unity gain for a SINE at f0, but
        // this one is driven by an IMPULSE, and with the (1 - z^-2) numerator
        // its impulse response peaks at about 2*b0 -- 4.7e-4 at Q 7, so
        // without normalizing, the bounce would silently vanish exactly as the
        // ripples got interesting. Measured against a direct impulse test:
        // peak * bounceNorm comes out at 1.00.
        float b0 = twangBounce1.b0[0];
        bounceNorm = (b0 > 1e-9f) ? rack::clamp(0.5f / b0, 0.f, 20000.f) : 0.f;
    }

    void init(float sr, float maxDelaySec = 0.065f) {
        nutToP.init(sr, maxDelaySec);
        pToNut.init(sr, maxDelaySec);
        pToBridge.init(sr, maxDelaySec);
        bridgeToP.init(sr, maxDelaySec);
        bridgeTone.reset();
        bridgeAdmittance.reset();
        for (int i = 0; i < DISPERSION_STAGES; ++i) dispersion[i].reset();
        for (int lane = 0; lane < LANES; ++lane) pluck[lane].reset();
        bowJunction.reset();
        drainGain           = float_4(1.f);
        safetyRMS           = float_4(0.f);
        safetyDecay         = float_4(0.f);
        energySmooth        = float_4(0.f);
        levelRef            = float_4(0.f);
        pluckKick           = float_4(0.f);
        prevKick            = float_4(0.f);
        twangRing           = float_4(0.f);
        twangBounce1.reset(); twangBounce2.reset();
        bowLift             = float_4(0.f);
        // The lifted bow re-engages over BOW_LIFT_TIME after a pluck, and
        // the junction glides back to the bow position at a similar pace.
        bowLiftDecay        = expf(-1.f / fmaxf(BOW_LIFT_TIME * sr, 1.f));
        junctionReturnCoeff = expf(-1.f / fmaxf(0.03f * sr, 1.f));
    }

    void panic() {
        nutToP.clear();
        pToNut.clear();
        pToBridge.clear();
        bridgeToP.clear();
        bridgeTone.reset();
        bridgeAdmittance.reset();
        for (int i = 0; i < DISPERSION_STAGES; ++i) dispersion[i].reset();
        for (int lane = 0; lane < LANES; ++lane) pluck[lane].reset();
        drainGain          = float_4(1.f);
        safetyRMS          = float_4(0.f);
        safetyDecay        = float_4(0.f);
        energySmooth       = float_4(0.f);
        levelRef           = float_4(0.f);
        pluckKick          = float_4(0.f);
        prevKick           = float_4(0.f);
        twangRing          = float_4(0.f);
        twangBounce1.reset(); twangBounce2.reset();
        bowLift            = float_4(0.f);
        junctionPos        = bowPosition;
    }

    // Pluck this lane: lift the bow, move the junction EXACTLY to the pluck
    // point (so the impulse is injected there, not at the bow), and start
    // the twang kick at the pluck's velocity. The junction glides back to
    // the bow position on its own as the lift decays (see process).
    // Positions are broadcast across the quad, so lane 0's value is the
    // lane's value.
    inline void triggerPluck(int lane, float velocity) {
        float liftArr[LANES];
        bowLift.store(liftArr);
        liftArr[lane] = 1.f;
        bowLift       = float_4::load(liftArr);

        float posArr[LANES];
        junctionPos.store(posArr);
        posArr[lane] = pluckPosition[0];
        junctionPos  = float_4::load(posArr);

        float kickArr[LANES];
        pluckKick.store(kickArr);
        kickArr[lane] = velocity;
        pluckKick     = float_4::load(kickArr);
    }

    // total:           per-lane (smoothed) one-way transit length nut->bridge
    //                  in samples. Pitch is independent of where the
    //                  junction sits -- moving it only changes the timbre
    //                  (which harmonics are cancelled), exactly like a real
    //                  string.
    // toneCoeff:       bridge LPF coefficient (TONE, already darkened by MUTE)
    // dispersionCoeff: allpass coefficient (fixed small string stiffness)
    // bowSpeed:        per-lane bow gesture. Lanes beyond the active voice
    //                  count are passed 0 by the caller so they stay silent
    //                  (a zero-speed bow has no grip, see below).
    //
    // Returns each lane's bridge output.
    inline float_4 process(float_4 bowSpeed, float_4 total,
                           float_4 toneCoeff, float_4 dispersionCoeff) {

        bridgeTone.setCoeff(toneCoeff);
        for (int i = 0; i < DISPERSION_STAGES; ++i)
            dispersion[i].coeff = dispersionCoeff;

        // Bow lift decays toward re-engagement (0). Grip also needs a MOVING
        // bow: a resting bow (speed ~ 0) exerts no friction, so a pluck on a
        // lane that isn't being bowed rings freely without any lift at all.
        // Full grip above |speed| 0.25; typical bowing speeds 0.5-1.5 are
        // unaffected.
        bowLift = bowLift * float_4(bowLiftDecay);
        // Grip vs bow speed. A hard clamp at |speed|*4 put a knee at 0.25:
        // below it the grip fell away linearly and then stopped dead, so
        // there was a speed under which nothing sounded at all and no useful
        // control above it. The smooth saturating form has no threshold --
        // quiet bowing gives a quiet note instead of silence, which is what
        // makes soft playing possible.
        float_4 absSpeed  = rack::simd::fabs(bowSpeed);
        float_4 bowMotion = absSpeed / (absSpeed + float_4(gripKnee));
        float_4 engagement = (float_4(1.f) - bowLift) * bowMotion;

        // The junction's split point: parked at the pluck point while the bow
        // is lifted, gliding back to the bow position as the lift decays (a
        // trigger set it exactly to the pluck point, so the pulse is always
        // injected there). While bowing it tracks the bow position with ~30 ms
        // of lag -- a bow arm taking its time, and inaudible.
        float_4 targetPos = bowPosition + (pluckPosition - bowPosition) * bowLift;
        junctionPos += float_4(junctionReturnCoeff) * (targetPos - junctionPos);

        // 1. Split the string at the junction point, per lane. The one-way
        //    transit nut->bridge is always `total`, so pitch is independent
        //    of where the junction sits.
        total                 = rack::simd::fmax(total, float_4(4.f));
        totalSegmentSamples   = total;
        float_4 P             = rack::simd::clamp(junctionPos, float_4(0.01f), float_4(0.99f));
        segALen = total * P;
        segCLen = total * (float_4(1.f) - P);

        // 2. Read all four spans before writing anything -- no rail may be
        //    touched by a write until all four reads are done.
        float_4 atP_fromLeft  = nutToP.read(segALen)    * drainGain;   // arriving at P from the nut side
        float_4 atNut         = pToNut.read(segALen)    * drainGain;   // arriving at the nut
        float_4 atBridge      = pToBridge.read(segCLen) * drainGain;   // arriving at the bridge
        float_4 atP_fromRight = bridgeToP.read(segCLen) * drainGain;   // arriving at P from the bridge side

        // Per-lane pluck impulse (0 on lanes not plucking this sample). A
        // pluck launches equal waves in both directions from the pluck point,
        // so the characteristic pluck spectrum (harmonics with a node at the
        // pluck point cancelled) emerges naturally from the reflections.
        float pluckRawArr[LANES];
        for (int lane = 0; lane < LANES; ++lane)
            pluckRawArr[lane] = pluck[lane].process();
        float_4 pluckRaw = float_4::load(pluckRawArr);

        // 3. The single junction at P. One branchless formula covers both
        //    configurations: the friction correction is scaled by engagement
        //    (0 while the bow is lifted or resting, so the junction becomes a
        //    transparent pass-through), and the per-lane pluck impulse is
        //    added to both outgoing waves. A pluck therefore launches equal
        //    waves in both directions from the pluck point, and the
        //    characteristic pluck spectrum emerges from the reflections.
        float_4 outToRight, outToLeft;
        bowJunction.process(atP_fromLeft, atP_fromRight, bowSpeed, engagement,
                            outToRight, outToLeft);
        pluckRaw   *= pluckDrive;
        outToRight += pluckRaw;
        outToLeft  += pluckRaw;

        // 4. End reflections, computed from the already-read values. The nut
        //    is a rigid inverting reflection. The bridge carries the whole
        //    loss/colour chain (tone LPF + stiffness dispersion + the MUTE
        //    loop-gain reduction, which also carries the baseline string
        //    damping), plus the body's re-radiation: the hollow box pushes
        //    the string at the bridge, re-exciting its partials (sympathetic
        //    ring) and giving plucks their long sustain.
        float_4 reflectedAtNut = -atNut;

        // Length-compensated loop gain, exp(-total * lossPerSample) via its
        // series expansion. total*lossPerSample is ~0.01 in normal use, where
        // the quadratic term is accurate to about 1 part in 10^7.
        float_4 x  = rack::simd::clamp(total * lossPerSample, float_4(0.f), float_4(1.f));
        loopGain   = float_4(1.f) - x + float_4(0.5f) * x * x;

        float_4 toned = bridgeTone.process(atBridge);
        for (int i = 0; i < DISPERSION_STAGES; ++i)
            toned = dispersion[i].process(toned);

        // Bridge admittance: subtract the body's resonant load from the
        // reflected wave. The bank is a bandpass (zero at DC and Nyquist),
        // so this can only REMOVE energy, and only at frequencies where the
        // body actually resonates. Swept over every tone/size/coupling
        // setting, |r| never exceeds its uncoupled maximum of 0.995 for
        // coupling <= 0.7 (the value is clamped there in setCoupling).
        float_4 admittanceLoad = bridgeAdmittance.process(atBridge);

        float_4 reflectedAtBridge = -(toned * loopGain - admittanceLoad);

        // 5. All writes last.
        nutToP.write(reflectedAtNut);       // nut -> P
        pToNut.write(outToLeft);            // P -> nut
        pToBridge.write(outToRight);        // P -> bridge
        bridgeToP.write(reflectedAtBridge); // bridge -> P

        // 6. TWANG transients. Reuse the same loop-energy quantity the safety
        //    limiter tracks, so this costs a few multiply-adds per lane
        //    rather than a whole new detector.
        float_4 loopEnergy = atBridge * atBridge;
        // Cycle-mean smoothed detector (tau ~75 ms): averages out the
        // per-fundamental-cycle energy pulses of a bowed note, so a sustained
        // note reads flat and bends nothing -- only level CHANGES and pluck
        // impacts produce a transient.
        energySmooth += float_4(0.00028f) * (loopEnergy - energySmooth);
        // Level reference: attacks slower than energySmooth (tau ~260 ms) so
        // an attack leaves a positive transient, releases faster (tau ~100 ms)
        // so it catches up when the level falls and the transient decays.
        float_4 levelRising = energySmooth > levelRef;
        float_4 refCoeff    = rack::simd::ifelse(levelRising, float_4(0.00008f), float_4(0.0002f));
        levelRef += refCoeff * (energySmooth - levelRef);
        // Pluck kick: set to the pluck velocity on each trigger, decays over
        // ~120 ms (the rubber band).
        pluckKick *= float_4(0.99983f);

        // Driven by the STEP in the kick, so the resonators see an impulse at
        // the pluck and nothing at all while the string simply rings.
        float_4 kickStep = pluckKick - prevKick;
        prevKick  = pluckKick;
        twangRing = (twangBounce1.process(kickStep)
                   + float_4(0.6f) * twangBounce2.process(kickStep)) * float_4(bounceNorm);

        // 7. Safety limiter -- branchless per-lane translation of the scalar
        //    version's two nested ifs.
        safetyRMS = rack::simd::fmax(loopEnergy, safetyRMS * 0.9999f);

        // Ceiling in STRING units, not volts, and it only exists to catch a
        // genuine runaway (which grows without bound). A driven pluck is not
        // that: at pluckDrive 6 the injected impulse is amplitude 6, which
        // tripped the old ceilings of 3 and 5 and made turning the pluck up
        // make it quieter. 100 (amplitude 10) still catches divergence long
        // before anything reaches the output clamp, and a bowed note runs an
        // order of magnitude below it.
        float_4 tripped = safetyRMS > float_4(900.f);
        safetyDecay = rack::simd::ifelse(tripped, float_4(0.9998f), safetyDecay);
        safetyRMS   = rack::simd::ifelse(tripped, float_4(0.f), safetyRMS);

        float_4 decaying        = safetyDecay > float_4(0.f);
        float_4 drainGainPre    = safetyDecay;   // drainGain takes the pre-decay value
        float_4 safetyDecayNext = safetyDecay * 0.9998f;
        float_4 expired         = safetyDecayNext < float_4(0.01f);

        float_4 safetyDecayDecaying = rack::simd::ifelse(expired, float_4(0.f), safetyDecayNext);
        float_4 drainGainDecaying   = rack::simd::ifelse(expired, float_4(1.f), drainGainPre);

        safetyDecay = rack::simd::ifelse(decaying, safetyDecayDecaying, safetyDecay);
        drainGain   = rack::simd::ifelse(decaying, drainGainDecaying, float_4(1.f));

        return reflectedAtBridge;
    }

    // Sample one lane's standing wave at a normalized position along the whole
    // string (0 = nut, 1 = bridge), for the display only. Stitches the two
    // spans together at the junction's current split point.
    float sampleStringAt(int lane, float positionFraction) const {
        positionFraction = rack::clamp(positionFraction, 0.f, 1.f);
        float P = junctionPos[lane];
        if (positionFraction <= P) {
            // segA: nut -> P
            float x = (P > 1e-6f) ? (positionFraction / P) : 0.f;
            return nutToP.readLane(lane, x * segALen[lane])
                 + pToNut.readLane(lane, (1.f - x) * segALen[lane]);
        } else {
            // segC: P -> bridge
            float span = 1.f - P;
            float z = (span > 1e-6f) ? ((positionFraction - P) / span) : 0.f;
            return pToBridge.readLane(lane, z * segCLen[lane])
                 + bridgeToP.readLane(lane, (1.f - z) * segCLen[lane]);
        }
    }
};

// --------------------------------------------------------------------------
// TwangDiffuserStage
// Schroeder allpass, same form as HazeAllpassStage: |H| = 1, phase only.
// Used for the dense high region of the body, where individual modes are
// neither computable nor audible as separate things.
// --------------------------------------------------------------------------
struct TwangDiffuserStage {
    std::vector<float> buf;
    int bufSize = 0, bufMask = 0, idx = 0;
    int delayLen = 149;

    void init(int maxDelay) {
        int needed = maxDelay + 4;
        bufSize = 1;
        while (bufSize < needed) bufSize <<= 1;
        bufMask = bufSize - 1;
        buf.assign(bufSize, 0.f);
        idx = 0;
    }
    inline void setDelay(int d) { delayLen = rack::clamp(d, 2, bufSize - 4); }

    inline float process(float input, float g) {
        int   readIdx = (idx - delayLen) & bufMask;
        float delayed = buf[readIdx];
        float out     = delayed - g * input;
        buf[idx]      = (1.f - g * g) * input + g * delayed;
        idx           = (idx + 1) & bufMask;
        return out;
    }
    void clear() { std::fill(buf.begin(), buf.end(), 0.f); idx = 0; }
};

static constexpr int TWANG_DIFFUSE_STAGES = 4;
static constexpr int TWANG_DIFFUSE_PRIME[TWANG_DIFFUSE_STAGES] = { 347, 211, 113, 67 };

// --------------------------------------------------------------------------
// TwangDiffuserStageSIMD / TwangBodyModalSIMD
//
// Four-lane copies of the body, for the polyphonic output path where each
// voice needs its own resonator state. The coefficients are identical across
// lanes -- it is one instrument body, not four -- so only the filter state and
// the delay buffers are per-lane, which is what makes this affordable: a quad
// of bodies costs barely more than one scalar body, not four.
//
// Only instantiated and run when Poly Out is on; the mono path is untouched.
// --------------------------------------------------------------------------
struct TwangDiffuserStageSIMD {
    std::vector<float_4> buf;
    int bufSize = 0, bufMask = 0, idx = 0;
    int delayLen = 149;

    void init(int maxDelay) {
        int needed = maxDelay + 4;
        bufSize = 1;
        while (bufSize < needed) bufSize <<= 1;
        bufMask = bufSize - 1;
        buf.assign(bufSize, float_4(0.f));
        idx = 0;
    }
    inline void setDelay(int d) { delayLen = rack::clamp(d, 2, bufSize - 4); }

    inline float_4 process(float_4 input, float g) {
        int     readIdx = (idx - delayLen) & bufMask;
        float_4 delayed = buf[readIdx];
        float_4 out     = delayed - float_4(g) * input;
        buf[idx]        = float_4(1.f - g * g) * input + float_4(g) * delayed;
        idx             = (idx + 1) & bufMask;
        return out;
    }
    void clear() { std::fill(buf.begin(), buf.end(), float_4(0.f)); idx = 0; }
};

struct TwangBodyModalSIMD {
    TwangModeBiquadSIMD mode[TWANG_MODE_COUNT];
    float               amp[TWANG_MODE_COUNT] = {};
    TwangModeBiquadSIMD hill;
    float               hillGain = 0.45f;

    TwangDiffuserStageSIMD diffuse[TWANG_DIFFUSE_STAGES];
    float   diffuseFb = 0.45f, diffuseG = 0.55f, diffuseGain = 0.30f;
    float_4 diffuseState = float_4(0.f);
    float_4 hpState = float_4(0.f), dampState = float_4(0.f);
    float   hpCoeff = 0.9f, dampCoeff = 0.5f;
    float   directGain = 0.18f, outputTrim = 0.8f;

    void init(float sr) {
        for (int i = 0; i < TWANG_DIFFUSE_STAGES; ++i) {
            diffuse[i].init((int)ceilf(TWANG_DIFFUSE_PRIME[i] * sr / 48000.f) + 8);
            diffuse[i].setDelay((int)roundf(TWANG_DIFFUSE_PRIME[i] * sr / 48000.f));
        }
        for (int i = 0; i < TWANG_MODE_COUNT; ++i) amp[i] = TWANG_MODES[i].amp;
        clear();
    }

    // Deliberately mirrors TwangBodyModal::setBody exactly. If one changes the
    // other has to, or Poly Out stops matching the stereo mix.
    void setBody(float lowestModeHz, float qScale, float material, float sr) {
        for (int i = 0; i < TWANG_MODE_COUNT; ++i) {
            mode[i].set(lowestModeHz * TWANG_MODES[i].ratio,
                        TWANG_MODES[i].Q * qScale, sr);
            amp[i] = TWANG_MODES[i].amp * (1.f + material * 2.2f * (float)i / TWANG_MODE_COUNT);
        }
        float hillHz = rack::clamp(800.f + 3500.f * rack::clamp(lowestModeHz / 500.f, 0.f, 1.f),
                                   600.f, 5000.f);
        hill.set(hillHz, 1.6f + material * 4.0f, sr);
        hillGain    = 0.15f + material * 0.65f;
        diffuseFb   = 0.25f + material * 0.55f;
        diffuseG    = 0.45f + material * 0.25f;
        diffuseGain = 0.15f + material * 0.45f;
        dampCoeff   = expf(-2.f * float(M_PI) * (1500.f + material * 12000.f) / sr);
        hpCoeff     = expf(-2.f * float(M_PI) * 700.f / sr);
    }

    inline float_4 process(float_4 core) {
        float_4 modal = float_4(0.f);
        for (int i = 0; i < TWANG_MODE_COUNT; ++i)
            modal += float_4(amp[i]) * mode[i].process(core);

        float_4 hillOut = float_4(hillGain) * hill.process(core);

        hpState += float_4(1.f - hpCoeff) * (core - hpState);
        float_4 hp = core - hpState;

        float_4 d = hp + float_4(diffuseFb) * diffuseState;
        for (int i = 0; i < TWANG_DIFFUSE_STAGES; ++i)
            d = diffuse[i].process(d, diffuseG);
        dampState   += float_4(1.f - dampCoeff) * (d - dampState);
        diffuseState = dampState;

        return float_4(outputTrim) * (float_4(directGain) * core + modal
                                    + hillOut + float_4(diffuseGain) * d);
    }

    void clear() {
        for (int i = 0; i < TWANG_MODE_COUNT; ++i) mode[i].reset();
        hill.reset();
        for (int i = 0; i < TWANG_DIFFUSE_STAGES; ++i) diffuse[i].clear();
        diffuseState = hpState = dampState = float_4(0.f);
    }
};

// --------------------------------------------------------------------------
// TwangBodyModal
//
// The whole body, in parallel branches off the bridge force:
//
//     bodyOut =   direct * core
//               + modal bank (named low modes)
//               + bridge hill (broad 2-3 kHz peak)
//               + diffuse tail (allpass loop, highpassed)
//
// PARALLEL is the point. The old chain ran four Schroeder allpasses in
// SERIES on the full signal including the direct path, which phase-smears
// the pluck transient -- that is why plucks read as soft and washy rather
// than as a thump with wood behind it. Here the direct path reaches the
// output untouched and the diffusion is a separate voice underneath it.
// --------------------------------------------------------------------------

struct TwangBodyModal {
    TwangModeBiquad   mode[TWANG_MODE_COUNT];
    float             amp[TWANG_MODE_COUNT] = {};

    TwangModeBiquad   hill;          // bridge hill: broad, low-Q
    float             hillGain = 0.45f;

    TwangDiffuserStage diffuse[TWANG_DIFFUSE_STAGES];
    float              diffuseFb   = 0.45f;
    float              diffuseG    = 0.55f;
    float              diffuseGain = 0.30f;
    float              diffuseState = 0.f;   // one sample of loop delay
    float              hpState = 0.f, hpCoeff = 0.9f;   // pre-diffuser highpass
    float              dampState = 0.f, dampCoeff = 0.5f;

    // Direct (un-resonated) path. Lowered from 0.30: with a third of the
    // signal bypassing the body, a broadband source came out looking almost
    // the same at every SIZE and MATERIAL -- measured spectral centroid moved
    // less than 4% across the whole 2D range. The body has to be most of what
    // you hear before its controls can mean anything.
    float directGain = 0.18f;
    float outputTrim = 0.8f;

    float sampleRate = 48000.f;

    void init(float sr) {
        sampleRate = sr;
        for (int i = 0; i < TWANG_DIFFUSE_STAGES; ++i) {
            diffuse[i].init((int)ceilf(TWANG_DIFFUSE_PRIME[i] * sr / 48000.f) + 8);
            diffuse[i].setDelay((int)roundf(TWANG_DIFFUSE_PRIME[i] * sr / 48000.f));
        }
        for (int i = 0; i < TWANG_MODE_COUNT; ++i) amp[i] = TWANG_MODES[i].amp;
        clear();
    }

    // lowestModeHz: RESONATOR SIZE.   qScale/brightness: RESONATOR MATERIAL.
    void setBody(float lowestModeHz, float qScale, float material, float sr) {
        sampleRate = sr;
        for (int i = 0; i < TWANG_MODE_COUNT; ++i) {
            mode[i].set(lowestModeHz * TWANG_MODES[i].ratio,
                        TWANG_MODES[i].Q * qScale, sr);
            // Metal/glass keep far more energy in the upper modes than wood.
            // The old 0.6 tilt was too small to hear against the direct path.
            amp[i] = TWANG_MODES[i].amp * (1.f + material * 2.2f * (float)i / TWANG_MODE_COUNT);
        }

        // The bridge hill belongs to the BRIDGE, not the box, so it tracks
        // size only loosely -- but it was previously pinned to 2.0-3.2 kHz,
        // which made it the loudest feature at every setting and flattened the
        // whole control range. Now it sweeps 800 Hz to 4.3 kHz, and MATERIAL
        // takes it from a barely-there bump to the dominant formant, which is
        // roughly the difference between a guitar and a violin.
        float hillHz = rack::clamp(800.f + 3500.f * rack::clamp(lowestModeHz / 500.f, 0.f, 1.f),
                                   600.f, 5000.f);
        hill.set(hillHz, 1.6f + material * 4.0f, sr);
        hillGain = 0.15f + material * 0.65f;

        diffuseFb   = 0.25f + material * 0.55f;
        diffuseG    = 0.45f + material * 0.25f;
        diffuseGain = 0.15f + material * 0.45f;

        float dampHz = 1500.f + material * 12000.f;
        dampCoeff    = expf(-2.f * float(M_PI) * dampHz / sr);
        hpCoeff      = expf(-2.f * float(M_PI) * 700.f / sr);   // keep lows out of the diffuser
    }

    inline float process(float core) {
        float modal = 0.f;
        for (int i = 0; i < TWANG_MODE_COUNT; ++i)
            modal += amp[i] * mode[i].process(core);

        float hillOut = hillGain * hill.process(core);

        // Diffuse tail: highpass, then an allpass loop with damping. Only the
        // dense region above ~700 Hz goes in, so the low modes stay clean.
        hpState += (1.f - hpCoeff) * (core - hpState);
        float hp = core - hpState;

        float d = hp + diffuseFb * diffuseState;
        for (int i = 0; i < TWANG_DIFFUSE_STAGES; ++i)
            d = diffuse[i].process(d, diffuseG);
        dampState    += (1.f - dampCoeff) * (d - dampState);
        diffuseState  = dampState;

        return outputTrim * (directGain * core + modal + hillOut + diffuseGain * d);
    }

    void clear() {
        for (int i = 0; i < TWANG_MODE_COUNT; ++i) mode[i].reset();
        hill.reset();
        for (int i = 0; i < TWANG_DIFFUSE_STAGES; ++i) diffuse[i].clear();
        diffuseState = hpState = dampState = 0.f;
    }
};

// --------------------------------------------------------------------------
// TwangSpaceDelay
// Very short feedback delay for the stereo width stage. Two instances
// (one per output channel) are fed the SAME mono core signal with slightly
// different delays; their difference is decorrelated detail that gets mixed
// only into the SIDE channel of a mid-side widen. That keeps L/R coherent --
// one instrument in a room, and a mono sum (L+R) cancels the side entirely --
// instead of two independent resonators each with their own character.
//
// Scalar: runs once per sample per channel on the summed signal, never per
// voice.
// --------------------------------------------------------------------------
struct TwangSpaceDelay {
    std::vector<float> buf;
    int   bufSize    = 0;
    int   bufMask    = 0;
    int   writeIndex = 0;
    int   delaySamples = 120;
    float coeff      = 0.3f;   // echo feedback; kept modest so the width stays subtle

    // Allocate once for the longest supported delay; setDelay() only moves
    // the read position, so the length can change at run time without
    // reallocation.
    void init(float sr, float maxDelaySec) {
        int needed = (int)ceilf(maxDelaySec * sr) + 8;
        bufSize = 1;
        while (bufSize < needed) bufSize <<= 1;
        bufMask  = bufSize - 1;
        buf.assign(bufSize, 0.f);
        writeIndex   = 0;
        delaySamples = std::max((int)roundf(0.0025f * sr), 2);
    }

    inline void setDelay(int samples) {
        delaySamples = rack::clamp(samples, 2, bufSize - 4);
    }

    // Returns the delayed (echo-coloured) sample; the fresh input is mixed
    // into the line via coeff.
    inline float process(float input) {
        int   readIndex = (writeIndex - delaySamples) & bufMask;
        float delayed   = buf[readIndex];
        float toWrite   = input + coeff * delayed;
        buf[writeIndex] = toWrite;
        writeIndex      = (writeIndex + 1) & bufMask;
        return delayed;
    }

    void clear() {
        std::fill(buf.begin(), buf.end(), 0.f);
        writeIndex = 0;
    }
};
