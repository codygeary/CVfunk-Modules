////////////////////////////////////////////////////////////
//
//   HotPot
//
//   written by Cody Geary
//   Copyright 2026, MIT License
//
//   Three-rail polyphonic spatial mixer. Sources are placed
//   on a ring, a stage and a depth line around a virtual
//   listener, cued with level, time and head filtering,
//   sent into a shared diffuse room (derived from Haze),
//   and summed through an ADAA saturating stereo bus.
//
////////////////////////////////////////////////////////////

#include "plugin.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include <atomic>
#include "FilterGlass.h"

using simd::float_4;

// =============================================================================
// Layout of sources
// =============================================================================
static const int NUM_RAILS     = 3;                       // 0 = ring, 1 = stage, 2 = depth
static const int RAIL_SOURCES  = 32;                      // 16 channels on A + 16 on B
static const int RAIL_GROUPS   = RAIL_SOURCES / 4;        // float_4 groups per rail
static const int TOTAL_SOURCES = NUM_RAILS * RAIL_SOURCES;

static const int CONTROL_DIV   = 32;    // samples between geometry / coefficient updates
static const int DISPLAY_DIV   = 512;   // samples between display snapshots

// =============================================================================
// Tunable constants -- head model and cues
// =============================================================================
static constexpr float HEAD_RADIUS_M        = 0.0875f;  // ITD range and shadow corner
static constexpr float SPEED_OF_SOUND       = 343.f;    // m/s
static constexpr float HAAS_MAX_SEC         = 0.060f;   // far-ear delay ceiling at the widest Haas setting
// Context menu Haas settings, 0 = natural ITD .. 1 = HAAS_MAX_SEC.
static const float HAAS_AMOUNTS[4] = { 0.f, 0.33f, 0.66f, 1.f };
static constexpr float PROPAGATION_MAX_SEC  = 0.250f;   // Doppler delay cap
// Doppler is the propagation delay changing, and its pitch shift is the
// source's speed over the speed of sound. Sources in a room move at walking
// pace, so the delay is followed smoothly (DOPPLER_SMOOTH_SEC) and its rate of
// change is capped at DOPPLER_MAX_SPEED: 0.8 m/s, an unhurried performer, is at
// most about 4 cents. The ear-to-ear delays have their own cap (ITD_MAX_CENTS
// of pitch at either ear), except that a full sweep of the widest Haas
// setting may take no less than ITD_MIN_SWEEP_SEC. Together they stay within
// about 6 cents.
// Fast motion, stepped glides and slider jumps all stay within that.
static constexpr float DOPPLER_MAX_SPEED    = 0.8f;     // m/s
static constexpr float ITD_MAX_CENTS        = 2.f;
static constexpr float ITD_MIN_SWEEP_SEC    = 0.5f;
static constexpr float DOPPLER_SMOOTH_SEC   = 0.25f;
static constexpr float MIN_TAP_DELAY        = 2.f;      // samples; keeps the 4-point read behind the write head
static constexpr float ILD_FLOOR            = 0.25f;    // far-ear gain floor (about -12 dB) up to CUES = 100%
static constexpr float SHADOW_ALPHA_MIN     = 0.1f;     // Brown-Duda shadow depth
static constexpr float SHADOW_THETA_SCALE   = 180.f / 150.f;  // 180 deg / theta_min (150 deg)
static constexpr float SHADOW_ALPHA_LOW     = 0.01f;    // clamp for exaggerated shadow
static constexpr float SHADOW_ALPHA_HIGH    = 4.f;
static constexpr float FRONT_PEAK_HZ        = 3000.f;   // front presence bump
static constexpr float FRONT_PEAK_Q         = 0.7f;
static constexpr float FRONT_PEAK_DB        = 3.f;      // per 100% CUES
static constexpr float REAR_SHELF_HZ        = 4000.f;   // rear shelf corner up to CUES = 100%
static constexpr float REAR_SHELF_HZ_LOW    = 1500.f;   // rear shelf corner at CUES = 300%
static constexpr float REAR_SHELF_DB        = -8.f;     // per 100% CUES
static constexpr float NEAR_GAIN_CAP        = 4.f;      // +12 dB ceiling on distance gain
static constexpr float AIR_DISTANCE_M       = 20.f;     // air absorption rate
static constexpr float AIR_MIN_HZ           = 1000.f;
static constexpr float IN_HEAD_RADIUS_M     = 0.3f;     // cues fade to center inside this (x scene scale)
static constexpr float REAR_SEND_BOOST      = 0.3f;     // extra room send behind, per 100% CUES

// Spice above 100% ("extra" = CUES - 1, up to 2) exaggerates the cues that read
// as space rather than as loudness, so the mix gets more vivid, not just quieter:
//   distance loudness grows only SPICE_DISTANCE_EXTRA per 100% past natural,
//   far sources get wetter and near ones drier (SPICE_WET_EXPONENT),
//   and the ear-to-ear delay widens by SPICE_ITD_WIDEN per 100%.
static constexpr float SPICE_DISTANCE_EXTRA = 0.25f;
static constexpr float SPICE_WET_EXPONENT   = 0.6f;
static constexpr float SPICE_ITD_WIDEN      = 1.0f;

// =============================================================================
// Tunable constants -- geometry and motion
// =============================================================================
static constexpr float RING_RADIUS_M        = 4.f;      // at 1x; also the unity-gain distance
static constexpr float STAGE_DISTANCE_M     = 8.f;      // stage line distance in front, at 1x
static constexpr float RAIL_HALF_LENGTH_M   = 12.f;     // stage half-width, at 1x
// Fan: two tracks, one each side, that diverge from narrow behind the
// listener to wide in front. They never meet, and pass about 6.9 m from the
// listener at 1x, so nothing crosses the head.
static constexpr float FAN_BACK_Y_M         = -10.f;    // rear ends
static constexpr float FAN_BACK_HALF_M      = 2.5f;
static constexpr float FAN_FRONT_Y_M        = 4.f;      // front ends, inside the stage line
static constexpr float FAN_FRONT_HALF_M     = 10.f;
static constexpr float SCENE_SCALE_MIN      = 0.25f;    // SIZE = 0
static constexpr float SCENE_SCALE_RANGE    = 40.f;     // SIZE = 1 -> 0.25 * 40 = 10x
static constexpr float MOVE_OFF             = 0.01f;    // MOVE at or below this is off
static constexpr float MOVE_RATE_MIN_HZ     = 0.01f;
static constexpr float MOVE_RATE_RANGE      = 700.f;    // exponential over the whole slider: 0.01 Hz .. 7 Hz (Leslie fast is ~6-7 Hz)
static constexpr float MOTION_STEPS         = 8.f;      // steps per cycle in stepped shape
static constexpr float STEP_GLIDE_FRACTION  = 0.2f;     // share of each step spent gliding to the next
static constexpr float MOTION_FADE_SEC      = 0.5f;     // motion fade in/out when MOVE turns on or off
static constexpr float MODE_FADE_SEC        = 0.3f;     // parallel <-> mirrored crossfade
static constexpr float SKIM_SEC             = 0.15f;    // how fast Skim settles everything home
static constexpr float MUTE_FADE_SEC        = 0.010f;   // mute fade, so muting never clicks
static constexpr float DELAY_SLEW_SEC       = 0.020f;   // ear-tap delay smoothing
static constexpr float LEVEL_RELEASE_SEC    = 0.050f;   // display level follower release
static constexpr float GAIN_SLEW_SEC        = 0.050f;   // source-count and main gain smoothing

// =============================================================================
// Tunable constants -- room (derived from Haze diffuse mode)
// =============================================================================
static constexpr int   ROOM_LOOPS           = 6;        // 0..2 left side, 3..5 right side
static constexpr int   ROOM_AP_STAGES       = 4;
static constexpr float ROOM_LOOP_MIN_MS     = 8.f;
static constexpr float ROOM_LOOP_RANGE      = 15.f;     // 8 ms .. 120 ms
static constexpr float ROOM_PREDELAY_MIN_MS = 5.f;
static constexpr float ROOM_PREDELAY_RANGE  = 30.f;     // 5 ms .. 150 ms
static constexpr float ROOM_AP_REF_LOOP_MS  = 12.f;     // Haze loop length the allpass tables were tuned for
static constexpr float ROOM_AP_REF_SR       = 48000.f;
static constexpr float ROOM_MOD_MAX_MS      = 1.5f;     // HAZE = 1
static constexpr float ROOM_MOD_RATE_HZ     = 0.23f;
static constexpr float ROOM_CROSS_ANGLE     = 0.35f;    // rad; how fast tails spread across sides
static constexpr float ROOM_DIFFUSER_COEFF  = 0.6f;
static constexpr float ROOM_DAMP_MIN_HZ     = 1000.f;
static constexpr float ROOM_DAMP_RANGE      = 20.f;     // 1 kHz .. 20 kHz
static constexpr float ROOM_DECAY_DARKEN    = 0.2f;     // loop cutoff multiplier at DECAY = 1 (Haze 2k/10k)
static constexpr float ROOM_LOOP_SAT_KNEE   = 7.f;      // HazeLoopSat constants
static constexpr float ROOM_LOOP_SAT_WIDTH  = 10.5f;
static constexpr float ROOM_WET_GAIN        = 3.5f;     // tank output level; sets the wet/dry balance range of ROOM
static constexpr float ROOM_LENGTH_SLEW_SEC = 0.050f;
// The tank runs at half the engine rate, which halves its cost. The band it
// carries is limited to ROOM_BAND of the half rate (9.6 kHz at 48 kHz), in the
// manner of classic hardware reverbs; its own damping sits below that anyway.
static constexpr float ROOM_BAND            = 0.4f;     // "Full-bandwidth room" in the menu runs it at the engine rate

// Loop lengths relative to the SIZE base length -- mutually incommensurate.
static const float ROOM_LOOP_RATIO[ROOM_LOOPS] = { 1.000f, 1.187f, 1.389f, 1.071f, 1.279f, 1.453f };

// In-loop allpass lengths in samples at 48 kHz for a 12 ms loop. The first
// three sets are Haze's; the right side gets its own primes.
static const int ROOM_LOOP_AP_DELAYS[ROOM_LOOPS][ROOM_AP_STAGES] = {
    { 347, 211, 113,  67 },
    { 251, 167,  89,  53 },
    { 283, 149, 103,  71 },
    { 331, 199, 127,  61 },
    { 239, 173,  83,  59 },
    { 293, 157, 109,  79 },
};

// Input diffuser lengths in samples at 48 kHz for a 12 ms loop, per side.
static const int ROOM_DIFFUSER_DELAYS[2][ROOM_AP_STAGES] = {
    { 113,  79, 173, 131 },
    { 109,  83, 167, 127 },
};

// Modulation phase offsets for the six loops (60 deg apart), so one sin/cos
// pair per sample yields all six LFO values.
static const float ROOM_MOD_COS[ROOM_LOOPS] = { 1.f,  0.5f, -0.5f, -1.f, -0.5f,  0.5f };
static const float ROOM_MOD_SIN[ROOM_LOOPS] = { 0.f,  0.8660254f,  0.8660254f, 0.f, -0.8660254f, -0.8660254f };

// =============================================================================
// Tunable constants -- mix bus
// =============================================================================
// Output safety limiter after the tube stage: linear below the knee, soft
// into a 10.5 V ceiling. Only reached by extreme levels.
static constexpr float BUS_SAT_KNEE         = 8.f;
static constexpr float BUS_SAT_WIDTH        = 3.75f;    // ceiling = knee + 2 * width / 3

// Simmer tube stage. The lower half of the slider is plain volume; the upper
// half raises the drive into a biased algebraic sigmoid, v / sqrt(1 + v^2),
// run with exact first-order antiderivative anti-aliasing.
//   TUBE_HEADROOM_V: level that maps to v = 1 at unity drive. 5 V peaks sit at
//     v = 0.25, where the curve is within 0.3 dB of linear.
//   TUBE_DRIVE_MAX_DB: drive added across the upper half.
//   TUBE_BIAS_MAX: grid bias at full drive; the asymmetry is what gives the
//     even harmonics. Zero at unity, so the unity sound stays symmetric and clean.
//   TUBE_MAKEUP_REF_V: level whose loudness the makeup gain holds roughly
//     steady as drive rises (half of the compression is made up).
//   TUBE_WARM_HZ: post-drive rolloff at full drive, sliding up to open at unity.
static constexpr float TUBE_HEADROOM_V      = 20.f;
static constexpr float TUBE_DRIVE_MAX_DB    = 20.f;
static constexpr float TUBE_BIAS_MAX        = 0.35f;
static constexpr float TUBE_MAKEUP_REF_V    = 5.f;
static constexpr float TUBE_WARM_HZ         = 7000.f;

// Headroom. Three stages, all automatic:
//   1. Anticipation: the source sum is scaled by 1 / sqrt(1 + HEADROOM_PER_SOURCE * (N - 1)),
//      about -3 dB per doubling once N is large, nothing for a few sources.
//   2. The room's input rides its own loop level, so a crowded pot does not
//      pile energy into the tank (ROOM_TARGET_V and its follower).
//   3. A soft-knee peak compressor on the mix, ahead of Simmer, holds the sum
//      near 5 V. Its gain is ramped linearly across each control block, so the
//      control rate never shows up as ripple. Simmer's drive then works on a
//      predictable level.
static constexpr float HEADROOM_PER_SOURCE  = 0.1f;
static constexpr float MIX_THRESHOLD_V      = 5.f;      // compressor threshold
static constexpr float MIX_RATIO            = 6.f;
static constexpr float MIX_KNEE_DB          = 6.f;
static constexpr float MIX_ATTACK_SEC       = 0.005f;
static constexpr float MIX_RELEASE_SEC      = 0.300f;
static constexpr float ROOM_TARGET_V        = 3.5f;     // loop peak the room input is ridden to hold (loop knee is 7 V)
static constexpr float ROOM_ATTACK_SEC      = 0.020f;
static constexpr float ROOM_RELEASE_SEC     = 1.0f;
static constexpr float LEVEL_UNITY_POS      = 0.75f;    // stage level slider position for unity; gain = (x / pos)^2

// HAZE is one control for diffusion and loop modulation: diffusion rises
// first and is full at 1 / HAZE_DIFFUSE_SCALE, modulation enters above
// HAZE_MOD_START.
static constexpr float HAZE_DIFFUSE_SCALE   = 1.6f;
static constexpr float HAZE_MOD_START       = 0.2f;

// -----------------------------------------------------------------------------
// HotPotSat -- HazeLoopSat's curve with adjustable knee and width.
// Exactly linear below the knee, cubic into a zero-slope ceiling, first-order
// ADAA throughout. No corner anywhere, so it never hard-clips.
// -----------------------------------------------------------------------------
struct HotPotSat {
    float knee    = BUS_SAT_KNEE;
    float width   = BUS_SAT_WIDTH;
    float ceiling = BUS_SAT_KNEE + 2.f * BUS_SAT_WIDTH / 3.f;
    float last    = 0.f;

    void setShape(float kneeVolts, float widthVolts) {
        knee    = kneeVolts;
        width   = widthVolts;
        ceiling = knee + 2.f * width / 3.f;
    }

    float shape(float x) const {
        float m = fabsf(x);
        if (m <= knee) return x;
        if (m >= knee + width) return (x < 0.f) ? -ceiling : ceiling;
        float u = m - knee;
        float g = knee + u - u * u * u / (3.f * width * width);
        return (x < 0.f) ? -g : g;
    }

    // Antiderivative of shape() (even function).
    float antiderivative(float x) const {
        float m = fabsf(x);
        if (m <= knee) return 0.5f * m * m;
        if (m >= knee + width) {
            float offset = -0.5f * knee * knee - (2.f / 3.f) * knee * width - 0.25f * width * width;
            return ceiling * m + offset;
        }
        float u = m - knee;
        return knee * m + 0.5f * u * u - u * u * u * u / (12.f * width * width) - 0.5f * knee * knee;
    }

    float process(float x) {
        // Linear fast path: exact identity, no divide.
        if (fabsf(x) <= knee && fabsf(last) <= knee) { last = x; return x; }
        float step = x - last;
        float out  = (fabsf(step) > 1e-3f) ? (antiderivative(x) - antiderivative(last)) / step
                                           : shape(0.5f * (x + last));
        last = x;
        return out;
    }

    void reset() { last = 0.f; }
};

// -----------------------------------------------------------------------------
// HotPotDelay -- scalar ring buffer sized from the sample rate, 4-point
// Lagrange fractional read (same polynomial as glassLagrange).
// -----------------------------------------------------------------------------
struct HotPotDelay {
    std::vector<float> buf;
    int mask       = 0;
    int writeIndex = 0;

    void init(int minSamples) {
        int size = 16;
        while (size < minSamples) size <<= 1;
        buf.assign(size, 0.f);
        mask       = size - 1;
        writeIndex = 0;
    }

    void write(float value) {
        buf[writeIndex] = value;
        writeIndex = (writeIndex + 1) & mask;
    }

    float read(float delaySamples) const {
        // Minimum of 2 keeps the fourth Lagrange point on written samples
        // rather than the slot about to be overwritten.
        delaySamples = clamp(delaySamples, MIN_TAP_DELAY, (float)mask - 4.f);
        float readPos   = (float)writeIndex - delaySamples;
        float readFloor = floorf(readPos);
        int   base      = ((int)readFloor) & mask;
        float frac      = readPos - readFloor;
        return glassLagrange(buf[(base - 1) & mask], buf[base],
                             buf[(base + 1) & mask], buf[(base + 2) & mask], frac);
    }

    void clear() {
        std::fill(buf.begin(), buf.end(), 0.f);
        writeIndex = 0;
    }
};

// -----------------------------------------------------------------------------
// HotPotAllpass -- Haze's Schroeder allpass stage.
//   H(z) = (z^-D - g) / (1 - g z^-D),  |H| = 1 for any |g| < 1.
// -----------------------------------------------------------------------------
struct HotPotAllpass {
    std::vector<float> buf;
    int mask = 0, index = 0;

    void init(int sizePow2) {
        buf.assign(sizePow2, 0.f);
        mask  = sizePow2 - 1;
        index = 0;
    }

    // delayLength must be 1..mask (set that way by HotPotRoom::setParams and
    // init's buffer sizing); feedGain is 1 - g * g, computed once per sample.
    float process(float input, int delayLength, float g, float feedGain) {
        float delayed = buf[(index - delayLength) & mask];
        float out     = delayed - g * input;
        buf[index]    = feedGain * input + g * delayed;
        index         = (index + 1) & mask;
        return out;
    }

    void clear() { std::fill(buf.begin(), buf.end(), 0.f); index = 0; }
};

// -----------------------------------------------------------------------------
// HotPotVoiceDelay -- four independent delay lines sharing one write head,
// read as float_4 with 4-point Lagrange interpolation.
//
// Each lane's line carries a mirror of its first three samples past the end,
// so the four points of any read are contiguous: one unaligned load per lane,
// then a 4x4 transpose turns the four lane rows into the y0..y3 columns the
// interpolation wants. Both ears are read in one pass.
// -----------------------------------------------------------------------------
struct HotPotVoiceDelay {
    std::vector<float> buf;   // lane k occupies [k * stride, k * stride + size + 3)
    int mask       = 0;
    int size       = 0;
    int stride     = 0;
    int writeIndex = 0;

    void init(int minSamples) {
        size = 16;
        while (size < minSamples) size <<= 1;
        mask   = size - 1;
        stride = size + 4;
        buf.assign(4 * stride, 0.f);
        writeIndex = 0;
    }

    void write(float_4 value) {
        float lanes[4];
        value.store(lanes);
        float* base = buf.data();
        for (int lane = 0; lane < 4; ++lane) base[lane * stride + writeIndex] = lanes[lane];
        if (writeIndex < 3)
            for (int lane = 0; lane < 4; ++lane) base[lane * stride + size + writeIndex] = lanes[lane];
        writeIndex = (writeIndex + 1) & mask;
    }

    // One tap: four contiguous points per lane, transposed into columns.
    inline float_4 tap(float_4 delaySamples) const {
        const float_4 clamped   = simd::clamp(delaySamples, float_4(MIN_TAP_DELAY), float_4((float)mask - 4.f));
        const float_4 readPos   = float_4((float)writeIndex) - clamped;
        float_4 readFloor       = simd::floor(readPos);
        const float_4 t         = readPos - readFloor;

        float floorLanes[4];
        readFloor.store(floorLanes);
        const float* base = buf.data();
        const __m128 row0 = _mm_loadu_ps(base + 0 * stride + (((int)floorLanes[0] - 1) & mask));
        const __m128 row1 = _mm_loadu_ps(base + 1 * stride + (((int)floorLanes[1] - 1) & mask));
        const __m128 row2 = _mm_loadu_ps(base + 2 * stride + (((int)floorLanes[2] - 1) & mask));
        const __m128 row3 = _mm_loadu_ps(base + 3 * stride + (((int)floorLanes[3] - 1) & mask));
        const __m128 low01  = _mm_unpacklo_ps(row0, row1);
        const __m128 low23  = _mm_unpacklo_ps(row2, row3);
        const __m128 high01 = _mm_unpackhi_ps(row0, row1);
        const __m128 high23 = _mm_unpackhi_ps(row2, row3);
        const float_4 y0 = float_4(_mm_movelh_ps(low01, low23));
        const float_4 y1 = float_4(_mm_movehl_ps(low23, low01));
        const float_4 y2 = float_4(_mm_movelh_ps(high01, high23));
        const float_4 y3 = float_4(_mm_movehl_ps(high23, high01));

        const float_4 tPlus  = t + 1.f;
        const float_4 tMinus = t - 1.f;
        const float_4 tMinus2 = t - 2.f;
        return (-t * tMinus * tMinus2 * (1.f / 6.f))     * y0
             + (tPlus * tMinus * tMinus2 * 0.5f)         * y1
             + (-tPlus * t * tMinus2 * 0.5f)             * y2
             + (tPlus * t * tMinus * (1.f / 6.f))        * y3;
    }

    void read(float_4 delayLeft, float_4 delayRight, float_4& outLeft, float_4& outRight) const {
        outLeft  = tap(delayLeft);
        outRight = tap(delayRight);
    }

    void clear() {
        std::fill(buf.begin(), buf.end(), 0.f);
        writeIndex = 0;
    }
};

// -----------------------------------------------------------------------------
// HotPotVoiceGroup -- all per-sample state for four sources.
// Gains ramp linearly across each control block; delays slew per sample.
// -----------------------------------------------------------------------------
struct HotPotVoiceGroup {
    HotPotVoiceDelay delay;
    bool active = false;

    float_4 airState  = float_4(0.f), airCoeff = float_4(0.f);

    // Front/back tone biquad, transposed direct form II.
    float_4 toneB0 = float_4(1.f), toneB1 = float_4(0.f), toneB2 = float_4(0.f);
    float_4 toneA1 = float_4(0.f), toneA2 = float_4(0.f);
    float_4 toneZ1 = float_4(0.f), toneZ2 = float_4(0.f);

    float_4 delayL = float_4(MIN_TAP_DELAY), delayR = float_4(MIN_TAP_DELAY);
    float_4 delayTargetL = float_4(MIN_TAP_DELAY), delayTargetR = float_4(MIN_TAP_DELAY);
    float_4 propagation = float_4(0.f), propagationTarget = float_4(0.f);   // Doppler delay, samples
    bool    snapPropagation = true;                                         // jump rather than glide on next load

    float_4 gainL = float_4(0.f), gainR = float_4(0.f);
    float_4 gainStepL = float_4(0.f), gainStepR = float_4(0.f);
    float_4 sendL = float_4(0.f), sendR = float_4(0.f);
    float_4 sendStepL = float_4(0.f), sendStepR = float_4(0.f);

    // Head shadow one-pole/one-zero per ear: y = b0 x + b1 x1 - a1 y1.
    float_4 shadowB0L = float_4(1.f), shadowB1L = float_4(0.f), shadowA1L = float_4(0.f);
    float_4 shadowB0R = float_4(1.f), shadowB1R = float_4(0.f), shadowA1R = float_4(0.f);
    float_4 shadowInL = float_4(0.f), shadowOutL = float_4(0.f);
    float_4 shadowInR = float_4(0.f), shadowOutR = float_4(0.f);

    float_4 levelGain = float_4(0.f);   // distance gain, for the display follower
    float_4 level     = float_4(0.f);

    void resetState() {
        delay.clear();
        airState = float_4(0.f);
        toneB0 = float_4(1.f); toneB1 = toneB2 = toneA1 = toneA2 = float_4(0.f);
        toneZ1 = toneZ2 = float_4(0.f);
        delayL = delayR = delayTargetL = delayTargetR = float_4(MIN_TAP_DELAY);
        propagation = propagationTarget = float_4(0.f);
        snapPropagation = true;
        gainL = gainR = gainStepL = gainStepR = float_4(0.f);
        sendL = sendR = sendStepL = sendStepR = float_4(0.f);
        shadowB0L = shadowB0R = float_4(1.f);
        shadowB1L = shadowB1R = shadowA1L = shadowA1R = float_4(0.f);
        shadowInL = shadowOutL = shadowInR = shadowOutR = float_4(0.f);
        levelGain = level = float_4(0.f);
    }
};

// -----------------------------------------------------------------------------
// HotPotRoom -- Haze's diffuse mode rebuilt as a stereo tank.
// Pre-delay -> input diffuser -> six allpass-loaded loops (three per side),
// each with DC blocker, damping LPF and soft limiter. An orthogonal rotation
// cross-couples left and right loops so tails bloom across the field.
// -----------------------------------------------------------------------------
struct HotPotRoom {
    HotPotDelay   preDelayL, preDelayR;
    HotPotAllpass diffuserL[ROOM_AP_STAGES], diffuserR[ROOM_AP_STAGES];
    int            diffuserLength[2][ROOM_AP_STAGES] = {};

    HotPotDelay   loopDelay[ROOM_LOOPS];
    HotPotAllpass loopAllpass[ROOM_LOOPS][ROOM_AP_STAGES];
    int            loopAllpassLength[ROOM_LOOPS][ROOM_AP_STAGES] = {};
    GlassDCBlocker loopDcBlock[ROOM_LOOPS];
    HotPotSat     loopSat[ROOM_LOOPS];
    float          loopDampState[ROOM_LOOPS]    = {};
    float          loopLength[ROOM_LOOPS]       = {};
    float          loopLengthTarget[ROOM_LOOPS] = {};
    float          loopFeedback[ROOM_LOOPS]     = {};

    float preDelayLength = 0.f, preDelayTarget = 0.f;
    float dampCoeff       = 0.f;
    float allpassCoeff    = 0.5f;
    float modDepthSamples = 0.f;
    float modCos = 1.f, modSin = 0.f;        // loop modulation phasor, rotated each sample
    float modRotCos = 1.f, modRotSin = 0.f;
    float lengthSlewCoeff = 0.001f;
    float crossCos        = cosf(ROOM_CROSS_ANGLE);
    float crossSin        = sinf(ROOM_CROSS_ANGLE);
    float sampleRate      = 48000.f;

    // Input riding: the loop peak is followed, and the input is turned down
    // whenever it rises past ROOM_TARGET_V.
    float loopEnv         = 0.f;
    float inputGain       = 1.f;
    float envAttackCoeff  = 0.001f, envReleaseCoeff = 0.0001f;

    // Buffers are sized for the engine rate, so the tank can switch between
    // full and half rate (setRate) without reallocating.
    void init(float sr) {
        const float maxScale = ROOM_LOOP_RANGE * ROOM_LOOP_MIN_MS / ROOM_AP_REF_LOOP_MS * sr / ROOM_AP_REF_SR;

        // Loop buffers hold the longest loop plus modulation swing.
        int loopSamples = (int)ceilf((ROOM_LOOP_MIN_MS * ROOM_LOOP_RANGE * 1.5f + ROOM_MOD_MAX_MS) * 0.001f * sr) + 8;
        for (int v = 0; v < ROOM_LOOPS; ++v) {
            loopDelay[v].init(loopSamples);
            loopSat[v].setShape(ROOM_LOOP_SAT_KNEE, ROOM_LOOP_SAT_WIDTH);
        }

        int longestAllpass = 1;
        for (int v = 0; v < ROOM_LOOPS; ++v)
            for (int s = 0; s < ROOM_AP_STAGES; ++s)
                longestAllpass = std::max(longestAllpass, (int)ceilf(ROOM_LOOP_AP_DELAYS[v][s] * maxScale));
        int allpassSize = 512;
        while (allpassSize <= longestAllpass) allpassSize <<= 1;
        for (int v = 0; v < ROOM_LOOPS; ++v)
            for (int s = 0; s < ROOM_AP_STAGES; ++s)
                loopAllpass[v][s].init(allpassSize);

        int longestDiffuser = 1;
        for (int side = 0; side < 2; ++side)
            for (int s = 0; s < ROOM_AP_STAGES; ++s)
                longestDiffuser = std::max(longestDiffuser, (int)ceilf(ROOM_DIFFUSER_DELAYS[side][s] * maxScale));
        int diffuserSize = 512;
        while (diffuserSize <= longestDiffuser) diffuserSize <<= 1;
        for (int s = 0; s < ROOM_AP_STAGES; ++s) {
            diffuserL[s].init(diffuserSize);
            diffuserR[s].init(diffuserSize);
        }

        int preSamples = (int)ceilf(ROOM_PREDELAY_MIN_MS * ROOM_PREDELAY_RANGE * 0.001f * sr) + 8;
        preDelayL.init(preSamples);
        preDelayR.init(preSamples);
        setRate(sr);
    }

    // The rate the tank actually runs at. Clears the tank; call setParams
    // with snap afterwards.
    void setRate(float sr) {
        sampleRate = sr;
        for (int v = 0; v < ROOM_LOOPS; ++v) loopDcBlock[v].setSampleRate(sr);
        lengthSlewCoeff = 1.f - expf(-1.f / (ROOM_LENGTH_SLEW_SEC * sr));
        modRotCos       = cosf(2.f * float(M_PI) * ROOM_MOD_RATE_HZ / sr);
        modRotSin       = sinf(2.f * float(M_PI) * ROOM_MOD_RATE_HZ / sr);
        envAttackCoeff  = 1.f - expf(-1.f / (ROOM_ATTACK_SEC * sr));
        envReleaseCoeff = 1.f - expf(-1.f / (ROOM_RELEASE_SEC * sr));
        clear();
    }

    // Control rate. All inputs 0..1.
    void setParams(float sizeKnob, float decayKnob, float dampKnob, float diffuseKnob, float hazeKnob, bool snap) {
        const float sr         = sampleRate;
        const float loopBaseMs = ROOM_LOOP_MIN_MS * powf(ROOM_LOOP_RANGE, sizeKnob);
        const float apScale    = loopBaseMs / ROOM_AP_REF_LOOP_MS * sr / ROOM_AP_REF_SR;
        const float decaySec   = 0.12f + 11.88f * decayKnob * decayKnob * decayKnob;

        for (int v = 0; v < ROOM_LOOPS; ++v) {
            loopLengthTarget[v] = loopBaseMs * ROOM_LOOP_RATIO[v] * 0.001f * sr;
            float effectiveLength = loopLengthTarget[v];
            for (int s = 0; s < ROOM_AP_STAGES; ++s) {
                int length = std::max(1, (int)lroundf(ROOM_LOOP_AP_DELAYS[v][s] * apScale));
                loopAllpassLength[v][s] = length;
                effectiveLength += (float)length;
            }
            // Per-loop feedback from its own total length, so all loops decay together.
            loopFeedback[v] = expf(-effectiveLength / (decaySec * sr));
            if (snap) loopLength[v] = loopLengthTarget[v];
        }
        for (int side = 0; side < 2; ++side)
            for (int s = 0; s < ROOM_AP_STAGES; ++s)
                diffuserLength[side][s] = std::max(1, (int)lroundf(ROOM_DIFFUSER_DELAYS[side][s] * apScale));

        // Haze's coupled darkening: longer decay pulls the cutoff down from the DAMP ceiling.
        float dampHz = ROOM_DAMP_MIN_HZ * powf(ROOM_DAMP_RANGE, dampKnob);
        float cutoff = dampHz * (1.f + (ROOM_DECAY_DARKEN - 1.f) * decayKnob * decayKnob);
        dampCoeff    = (cutoff >= 19900.f) ? 0.f : expf(-2.f * float(M_PI) * cutoff / sr);

        allpassCoeff    = 0.1f + 0.75f * diffuseKnob;
        modDepthSamples = hazeKnob * hazeKnob * ROOM_MOD_MAX_MS * 0.001f * sr;
        preDelayTarget  = ROOM_PREDELAY_MIN_MS * powf(ROOM_PREDELAY_RANGE, sizeKnob) * 0.001f * sr;
        if (snap) preDelayLength = preDelayTarget;
    }

    void process(float inL, float inR, float& outL, float& outR) {
        preDelayLength += (preDelayTarget - preDelayLength) * lengthSlewCoeff;
        preDelayL.write(inL * inputGain);
        preDelayR.write(inR * inputGain);
        float diffusedL = preDelayL.read(preDelayLength);
        float diffusedR = preDelayR.read(preDelayLength);
        const float diffuserFeed = 1.f - ROOM_DIFFUSER_COEFF * ROOM_DIFFUSER_COEFF;
        for (int s = 0; s < ROOM_AP_STAGES; ++s) {
            diffusedL = diffuserL[s].process(diffusedL, diffuserLength[0][s], ROOM_DIFFUSER_COEFF, diffuserFeed);
            diffusedR = diffuserR[s].process(diffusedR, diffuserLength[1][s], ROOM_DIFFUSER_COEFF, diffuserFeed);
        }

        // Rotate the modulation phasor; the magnitude is pulled back to 1 with
        // one Newton step so rounding never lets it drift.
        const float rotatedCos = modCos * modRotCos - modSin * modRotSin;
        const float rotatedSin = modSin * modRotCos + modCos * modRotSin;
        const float renorm     = 1.5f - 0.5f * (rotatedCos * rotatedCos + rotatedSin * rotatedSin);
        modCos = rotatedCos * renorm;
        modSin = rotatedSin * renorm;
        const float loopFeed = 1.f - allpassCoeff * allpassCoeff;

        float loopOut[ROOM_LOOPS];
        for (int v = 0; v < ROOM_LOOPS; ++v) {
            loopLength[v] += (loopLengthTarget[v] - loopLength[v]) * lengthSlewCoeff;
            float modulation = modSin * ROOM_MOD_COS[v] + modCos * ROOM_MOD_SIN[v];   // sin(theta + v * 60 deg)
            float out = loopDelay[v].read(loopLength[v] + modDepthSamples * modulation);
            out = loopDcBlock[v].process(out);
            loopDampState[v] = (1.f - dampCoeff) * out + dampCoeff * loopDampState[v];
            out = loopDampState[v];
            for (int s = 0; s < ROOM_AP_STAGES; ++s)
                out = loopAllpass[v][s].process(out, loopAllpassLength[v][s], allpassCoeff, loopFeed);
            loopOut[v] = out;
        }

        float loopPeak = 0.f;
        for (int v = 0; v < ROOM_LOOPS; ++v) loopPeak = std::max(loopPeak, fabsf(loopOut[v]));
        loopEnv  += (loopPeak - loopEnv) * ((loopPeak > loopEnv) ? envAttackCoeff : envReleaseCoeff);
        inputGain = (loopEnv > ROOM_TARGET_V) ? ROOM_TARGET_V / loopEnv : 1.f;

        outL = (loopOut[0] + loopOut[1] + loopOut[2]) * (ROOM_WET_GAIN / 3.f);
        outR = (loopOut[3] + loopOut[4] + loopOut[5]) * (ROOM_WET_GAIN / 3.f);

        // Lossless rotation between left loop v and right loop (v + 1) mod 3.
        float rotated[ROOM_LOOPS];
        for (int v = 0; v < 3; ++v) {
            int   right = 3 + (v + 1) % 3;
            float left  = loopOut[v];
            float other = loopOut[right];
            rotated[v]     = crossCos * left - crossSin * other;
            rotated[right] = crossSin * left + crossCos * other;
        }

        for (int v = 0; v < ROOM_LOOPS; ++v) {
            float sideInput = (v < 3) ? diffusedL : diffusedR;
            loopDelay[v].write(loopSat[v].process(sideInput + loopFeedback[v] * rotated[v]));
        }
    }

    void clear() {
        preDelayL.clear();
        preDelayR.clear();
        for (int s = 0; s < ROOM_AP_STAGES; ++s) { diffuserL[s].clear(); diffuserR[s].clear(); }
        for (int v = 0; v < ROOM_LOOPS; ++v) {
            loopDelay[v].clear();
            loopDcBlock[v].reset();
            loopSat[v].reset();
            loopDampState[v] = 0.f;
            for (int s = 0; s < ROOM_AP_STAGES; ++s) loopAllpass[v][s].clear();
        }
        loopEnv   = 0.f;
        inputGain = 1.f;
    }
};

// -----------------------------------------------------------------------------
// HotPotTube -- the Simmer drive stage, run at 2x with L in lane 0 and R in
// lane 1 of one float_4.
//
// Shaper g(v) = v / sqrt(1 + v^2), biased by b. Its antiderivative is
// sqrt(1 + v^2), and the ADAA difference quotient of that simplifies exactly
// to (v1 + v0) / (sqrt(1 + v1^2) + sqrt(1 + v0^2)): no divide by a small
// step, so no fallback branch. ADAA and the 2x rate together keep the fold-
// back of a hard-driven 5 kHz tone about 85 dB down.
//
// Resampling is a 6-pole Butterworth (Filter6pButter's Q schedule) at 0.45
// of the base rate, once each way.
// -----------------------------------------------------------------------------
struct HotPotTube {
    dsp::TBiquadFilter<float_4> upFilter[3], downFilter[3];
    float_4 lastIn   = float_4(0.f);
    float_4 lastRoot = float_4(1.f);   // sqrt(1 + lastIn^2)

    void setup() {
        const float cutoff = 0.45f * 0.5f;   // 0.45 of the base rate, as a fraction of the 2x rate
        const float stageQ[3] = { 0.51763809f, 0.70710678f, 1.93185165f };
        for (int k = 0; k < 3; ++k) {
            upFilter[k].setParameters(dsp::TBiquadFilter<float_4>::LOWPASS, cutoff, stageQ[k], 1.f);
            downFilter[k].setParameters(dsp::TBiquadFilter<float_4>::LOWPASS, cutoff, stageQ[k], 1.f);
        }
    }

    void reset() {
        for (int k = 0; k < 3; ++k) { upFilter[k].reset(); downFilter[k].reset(); }
        lastIn   = float_4(0.f);
        lastRoot = float_4(1.f);
    }

    // One shaper step at the 2x rate. inScale maps volts to v, bias shifts
    // the curve, biasOut = g(bias) is removed, outScale maps back to volts.
    inline float_4 shape(float_4 x, float inScale, float bias, float biasOut, float outScale) {
        const float_4 v    = x * inScale + bias;
        const float_4 root = simd::sqrt(1.f + v * v);
        const float_4 y    = (v + lastIn) / (root + lastRoot);
        lastIn   = v;
        lastRoot = root;
        return (y - biasOut) * outScale;
    }

    float_4 process(float_4 x, float inScale, float bias, float biasOut, float outScale) {
        // Zero-stuffed upsample (gain 2 restores the level), shape both
        // samples, keep the second after the decimation filter.
        float_4 first = x * 2.f, second = float_4(0.f);
        for (int k = 0; k < 3; ++k) first  = upFilter[k].process(first);
        for (int k = 0; k < 3; ++k) second = upFilter[k].process(second);
        first  = shape(first,  inScale, bias, biasOut, outScale);
        second = shape(second, inScale, bias, biasOut, outScale);
        for (int k = 0; k < 3; ++k) first  = downFilter[k].process(first);
        for (int k = 0; k < 3; ++k) second = downFilter[k].process(second);
        return second;
    }
};

// -----------------------------------------------------------------------------
// HotPotHalfRate -- 6-pole Butterworth pair (Filter6pButter's Q schedule) for
// moving the room between the engine rate and half of it. L in lane 0, R in
// lane 1. The same filter shape serves both directions.
// -----------------------------------------------------------------------------
struct HotPotHalfRate {
    dsp::TBiquadFilter<float_4> filter[3];

    void setup() {
        const float cutoff = ROOM_BAND * 0.5f;   // fraction of the half rate, as a fraction of the full rate
        const float stageQ[3] = { 0.51763809f, 0.70710678f, 1.93185165f };
        for (int k = 0; k < 3; ++k)
            filter[k].setParameters(dsp::TBiquadFilter<float_4>::LOWPASS, cutoff, stageQ[k], 1.f);
    }
    void reset() { for (int k = 0; k < 3; ++k) filter[k].reset(); }
    inline float_4 process(float_4 x) {
        for (int k = 0; k < 3; ++k) x = filter[k].process(x);
        return x;
    }
};

// -----------------------------------------------------------------------------
// Smooth staircase of a phase in cycles: MOTION_STEPS steps per cycle, each
// holding and then gliding into the next over its last STEP_GLIDE_FRACTION.
// Stateless and continuous, so stepped motion never jumps.
// -----------------------------------------------------------------------------
static inline float hotPotSteppedPhase(float phase) {
    const float scaled    = phase * MOTION_STEPS;
    const float stepIndex = floorf(scaled);
    float glide = clamp((scaled - stepIndex - (1.f - STEP_GLIDE_FRACTION)) / STEP_GLIDE_FRACTION, 0.f, 1.f);
    glide = glide * glide * (3.f - 2.f * glide);
    return (stepIndex + glide) / MOTION_STEPS;
}

// -----------------------------------------------------------------------------
// Display snapshot, double-buffered: the audio thread fills the unpublished
// frame, then publishes its index; the widget reads the published frame.
// -----------------------------------------------------------------------------
struct HotPotDisplaySource {
    float x = 0.f, y = 0.f;      // meters at 1x scene scale (front = +y)
    float level  = 0.f;
    int   rail   = 0;
    bool  isB    = false;
    bool  active = false;
};

struct HotPotDisplayFrame {
    HotPotDisplaySource sources[TOTAL_SOURCES];
    float sceneScale = 1.f;
    float wetLevel   = 0.f;
};

// Simmer readout: volume in dB on the lower half, drive in dB on the upper.
struct SimmerQuantity : ParamQuantity {
    std::string getDisplayValueString() override {
        const float value = getValue();
        if (value <= 0.f) return "-inf dB";
        if (value <= 0.5f) return string::f("%.1f dB", 40.f * log10f(2.f * value));
        return string::f("Drive +%.1f dB", (value - 0.5f) * 2.f * TUBE_DRIVE_MAX_DB);
    }
    std::string getUnit() override { return ""; }
};

// MOVE readout: "Off" at the bottom, otherwise the motion rate in Hz.
struct MoveQuantity : ParamQuantity {
    std::string getDisplayValueString() override {
        const float value = getValue();
        if (value <= MOVE_OFF) return "Off";
        return string::f("%.3f", MOVE_RATE_MIN_HZ * powf(MOVE_RATE_RANGE, (value - MOVE_OFF) / (1.f - MOVE_OFF)));
    }
    std::string getUnit() override { return (getValue() <= MOVE_OFF) ? "" : " Hz"; }
};

// =============================================================================
// HotPot module
// =============================================================================
struct HotPot : Module {

    enum ParamIds {
        POS_PARAM,
        POS_ATT_PARAM    = POS_PARAM        + NUM_RAILS,
        SPREAD_PARAM     = POS_ATT_PARAM    + NUM_RAILS,
        SPREAD_ATT_PARAM = SPREAD_PARAM     + NUM_RAILS,
        MOVE_PARAM       = SPREAD_ATT_PARAM + NUM_RAILS,
        MOVE_ATT_PARAM   = MOVE_PARAM       + NUM_RAILS,
        LEVEL_PARAM      = MOVE_ATT_PARAM   + NUM_RAILS,
        LEVEL_ATT_PARAM  = LEVEL_PARAM      + NUM_RAILS,
        ROOM_PARAM       = LEVEL_ATT_PARAM  + NUM_RAILS,
        ROOM_ATT_PARAM,
        DECAY_PARAM,
        DECAY_ATT_PARAM,
        HAZE_PARAM,
        HAZE_ATT_PARAM,
        DAMP_PARAM,
        DAMP_ATT_PARAM,
        SIZE_PARAM,
        SIZE_ATT_PARAM,
        CUES_PARAM,
        CUES_ATT_PARAM,
        MAIN_PARAM,
        MAIN_ATT_PARAM,
        RESET_PARAM,
        MODE_PARAM,                                   // per rail: 0 = parallel, 1 = mirrored
        MUTE_PARAM       = MODE_PARAM + NUM_RAILS,    // per rail
        MAIN_MUTE_PARAM  = MUTE_PARAM + NUM_RAILS,
        NUM_PARAMS
    };

    enum InputIds {
        IN_A_INPUT,
        IN_B_INPUT       = IN_A_INPUT      + NUM_RAILS,
        POS_CV_INPUT     = IN_B_INPUT      + NUM_RAILS,
        SPREAD_CV_INPUT  = POS_CV_INPUT    + NUM_RAILS,
        MOVE_CV_INPUT    = SPREAD_CV_INPUT + NUM_RAILS,
        LEVEL_CV_INPUT   = MOVE_CV_INPUT   + NUM_RAILS,
        ROOM_CV_INPUT    = LEVEL_CV_INPUT  + NUM_RAILS,
        DECAY_CV_INPUT,
        HAZE_CV_INPUT,
        DAMP_CV_INPUT,
        SIZE_CV_INPUT,
        CUES_CV_INPUT,
        MAIN_CV_INPUT,
        NUM_INPUTS
    };

    enum OutputIds {
        OUT_L_OUTPUT,
        OUT_R_OUTPUT,
        NUM_OUTPUTS
    };

    enum LightIds {
        RESET_LIGHT,
        MODE_LIGHT,
        MUTE_LIGHT       = MODE_LIGHT + NUM_RAILS,
        MAIN_MUTE_LIGHT  = MUTE_LIGHT + NUM_RAILS,
        NUM_LIGHTS
    };

    enum MotionShape { SHAPE_SINE = 0, SHAPE_TRIANGLE, SHAPE_STEPPED };

    // -- Saved settings (context menu) ----------------------------------------
    int  motionShape[NUM_RAILS] = { SHAPE_SINE, SHAPE_SINE, SHAPE_SINE };
    bool doppler  = true;
    int  haasMode = 0;                              // index into HAAS_AMOUNTS

    // -- Voices ---------------------------------------------------------------
    HotPotVoiceGroup groups[NUM_RAILS][RAIL_GROUPS];
    float railInput[NUM_RAILS][RAIL_SOURCES + 4] = {};   // gathered A then B channels
    int   countA[NUM_RAILS]      = {};
    int   sourceCount[NUM_RAILS] = {};

    // Source positions at 1x scene scale, for the display.
    float sourceX[NUM_RAILS][RAIL_SOURCES] = {};
    float sourceY[NUM_RAILS][RAIL_SOURCES] = {};

    // -- Motion ---------------------------------------------------------------
    float motionPhase[NUM_RAILS]  = {};
    float motionAmount[NUM_RAILS] = {};            // rails: fades motion in and out as MOVE turns on or off
    float modeBlend[NUM_RAILS] = { 1.f, 1.f, 1.f };  // 0 = parallel, 1 = mirrored, crossfaded
    float muteGain[NUM_RAILS]  = { 1.f, 1.f, 1.f };  // faded rail mutes
    float mainMuteGain = 1.f, mainMuteCoeff = 0.002f;
    bool  skimming[NUM_RAILS]  = {};               // Skim in progress: settling home
    float ringTurn = 0.f;                          // Stir rotation in cycles, kept in [-0.5, 0.5)
    dsp::SchmittTrigger resetButtonTrigger;

    // -- Room and bus ---------------------------------------------------------
    HotPotRoom room;
    HotPotHalfRate roomDecimator, roomInterpolator;         // into and out of the half-rate tank
    float_4 roomHeld  = float_4(0.f);                       // last tank output, L and R lanes
    bool    roomPhase = false;                              // tank runs when this is false
    bool    roomFullRate = false;                           // context menu: tank at the engine rate
    bool    roomFullRateActive = false;                     // what the tank is currently set up for
    HotPotSat  busSatL, busSatR;
    float countGainCurrent = 1.f, countGainTarget = 1.f;   // headroom anticipation from the source count
    float mixEnv = 0.f;                                    // mix peak follower
    float potGain = 1.f, potGainStep = 0.f;                // mix compressor gain, ramped per block
    float mixAttackCoeff = 0.01f, mixReleaseCoeff = 0.0001f;

    // Simmer: targets set at control rate, smoothed per sample.
    float volumeTarget = 1.f, volumeCurrent = 1.f;
    float driveTarget  = 1.f, driveCurrent  = 1.f;
    float biasTarget   = 0.f, biasCurrent   = 0.f;
    float makeupTarget = 1.f, makeupCurrent = 1.f;
    float warmCoeff    = 0.f;                               // post-drive one-pole, 0 = open
    HotPotTube tube;
    float_4 warmState = float_4(0.f);                       // post-drive rolloff, L and R lanes
    float_4 dcInLast  = float_4(0.f), dcOutLast = float_4(0.f);
    float   dcCoeff   = 0.9974f;                            // 20 Hz DC blocker after the tube
    bool  roomSnapPending = true;
    float lastRoomArgs[5] = { -1.f, -1.f, -1.f, -1.f, -1.f };   // room.setParams inputs last applied
    float sceneScaleCurrent = 1.f;                 // SIZE after CV, for the display frame

    // -- Rates and cached coefficients -----------------------------------------
    float sampleRate        = 48000.f;
    float delaySlewCoeff    = 0.001f;
    float dopplerCoeff      = 0.0001f;  // propagation follower
    float dopplerMaxStep    = 0.0035f;  // samples of delay change per sample at DOPPLER_MAX_SPEED
    float itdMaxStep        = 0.0035f;  // same limit for the ear-to-ear delays, set at control rate
    bool  dopplerWasOn      = true;
    float levelReleaseCoeff = 0.999f;
    float smoothCoeff       = 0.001f;   // source-count and main gain smoothing
    int   controlCounter    = CONTROL_DIV;
    int   displayCounter    = 0;
    float wetFollower       = 0.f;

    // -- Display handoff ------------------------------------------------------
    HotPotDisplayFrame displayFrames[2];
    std::atomic<int>    displayPublished{0};

    // Far-ear ITD at 90 deg for the Woodworth model: (a / c) * (pi/2 + 1).
    const float itdMaxSec = HEAD_RADIUS_M / SPEED_OF_SOUND * (0.5f * float(M_PI) + 1.f);

    HotPot() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        const char* railNames[NUM_RAILS] = { "Stir", "Shabu", "Fan" };

        configParam(POS_PARAM + 0, -1.f, 1.f, 0.f,  "Stir angle", " deg", 0.f, 180.f);
        configParam(POS_PARAM + 1, -1.f, 1.f, 0.f,  "Shabu position (left to right)", " m at 1x", 0.f, RAIL_HALF_LENGTH_M);
        configParam(POS_PARAM + 2, -1.f, 1.f, 0.f,  "Fan position (back to front)", "%", 0.f, 50.f, 50.f);

        for (int r = 0; r < NUM_RAILS; ++r) {
            std::string name = railNames[r];
            configParam(POS_ATT_PARAM    + r, -1.f, 1.f, 0.f,  name + " position CV attenuverter", "%", 0.f, 100.f);
            configParam(SPREAD_PARAM     + r,  0.f, 1.f, 0.3f, name + " spread", "%", 0.f, 100.f);
            configParam(SPREAD_ATT_PARAM + r, -1.f, 1.f, 0.f,  name + " spread CV attenuverter", "%", 0.f, 100.f);
            configParam<MoveQuantity>(MOVE_PARAM + r, 0.f, 1.f, 0.f, name + " motion rate");
            configParam(MOVE_ATT_PARAM   + r, -1.f, 1.f, 0.f,  name + " move CV attenuverter", "%", 0.f, 100.f);
            // dB readout: 40 log10(x) - 40 log10(LEVEL_UNITY_POS).
            configParam(LEVEL_PARAM      + r,  0.f, 1.f, LEVEL_UNITY_POS, name + " level", " dB", -10.f, 40.f,
                        -40.f * log10f(LEVEL_UNITY_POS));
            configParam(LEVEL_ATT_PARAM  + r,  0.f, 1.f, 1.f,  name + " level CV depth", "%", 0.f, 100.f);

            configInput(IN_A_INPUT      + r, name + " A");
            configInput(IN_B_INPUT      + r, name + " B");
            configInput(POS_CV_INPUT    + r, name + " position CV (poly: per source)");
            configInput(SPREAD_CV_INPUT + r, name + " spread CV");
            configInput(MOVE_CV_INPUT   + r, name + " move CV");
            configInput(LEVEL_CV_INPUT  + r, name + " level CV, 0-10 V VCA (poly: per source)");
        }

        configParam(ROOM_PARAM,      0.f, 1.f, 0.25f,  "Broth (reverb send)", "%", 0.f, 100.f);
        configParam(ROOM_ATT_PARAM, -1.f, 1.f, 0.f,    "Broth CV attenuverter", "%", 0.f, 100.f);
        configParam(DECAY_PARAM,     0.f, 1.f, 0.4f,   "MSG (decay)", "%", 0.f, 100.f);
        configParam(DECAY_ATT_PARAM,-1.f, 1.f, 0.f,    "MSG CV attenuverter", "%", 0.f, 100.f);
        configParam(HAZE_PARAM,      0.f, 1.f, 0.375f, "Steam (diffusion, then modulation)", "%", 0.f, 100.f);
        configParam(HAZE_ATT_PARAM, -1.f, 1.f, 0.f,    "Steam CV attenuverter", "%", 0.f, 100.f);
        configParam(DAMP_PARAM,      0.f, 1.f, 0.7f,   "Lid (damping ceiling)", " Hz", ROOM_DAMP_RANGE, ROOM_DAMP_MIN_HZ);
        configParam(DAMP_ATT_PARAM, -1.f, 1.f, 0.f,    "Lid CV attenuverter", "%", 0.f, 100.f);
        configParam(SIZE_PARAM,      0.f, 1.f, 0.376f, "Pot (size)", "x", SCENE_SCALE_RANGE, SCENE_SCALE_MIN);
        configParam(SIZE_ATT_PARAM, -1.f, 1.f, 0.f,    "Pot CV attenuverter", "%", 0.f, 100.f);
        configParam(CUES_PARAM,      0.f, 3.f, 1.f,    "Spice (cue strength; above 100% exaggerates)", "%", 0.f, 100.f);
        configParam(CUES_ATT_PARAM, -1.f, 1.f, 0.f,    "Spice CV attenuverter", "%", 0.f, 100.f);
        configParam<SimmerQuantity>(MAIN_PARAM, 0.f, 1.f, 0.5f, "Simmer (volume / tube drive)");
        configParam(MAIN_ATT_PARAM, -1.f, 1.f, 0.f,    "Simmer CV attenuverter", "%", 0.f, 100.f);
        configButton(RESET_PARAM, "Skim (motion reset)");
        configSwitch(MODE_PARAM + 0, 0.f, 1.f, 1.f, "Stir motion",  { "Parallel: all turn together", "Mirrored: halves turn opposite ways" });
        configSwitch(MODE_PARAM + 1, 0.f, 1.f, 1.f, "Shabu motion", { "Parallel: the group swishes together", "Mirrored: halves open and close" });
        configSwitch(MODE_PARAM + 2, 0.f, 1.f, 1.f, "Fan motion",   { "Parallel: the two tracks rock against each other", "Mirrored: both tracks move together" });
        configSwitch(MUTE_PARAM + 0, 0.f, 1.f, 0.f, "Stir mute",  { "On", "Muted" });
        configSwitch(MUTE_PARAM + 1, 0.f, 1.f, 0.f, "Shabu mute", { "On", "Muted" });
        configSwitch(MUTE_PARAM + 2, 0.f, 1.f, 0.f, "Fan mute",   { "On", "Muted" });
        configSwitch(MAIN_MUTE_PARAM, 0.f, 1.f, 0.f, "Main mute", { "On", "Muted" });

        configInput(ROOM_CV_INPUT,  "Broth CV");
        configInput(DECAY_CV_INPUT, "MSG CV");
        configInput(HAZE_CV_INPUT,  "Steam CV");
        configInput(DAMP_CV_INPUT,  "Lid CV");
        configInput(SIZE_CV_INPUT,  "Pot CV");
        configInput(CUES_CV_INPUT,  "Spice CV (10 V spans the full 0-300%)");
        configInput(MAIN_CV_INPUT,  "Simmer CV");

        configOutput(OUT_L_OUTPUT, "Left");
        configOutput(OUT_R_OUTPUT, "Right");

        busSatL.setShape(BUS_SAT_KNEE, BUS_SAT_WIDTH);
        busSatR.setShape(BUS_SAT_KNEE, BUS_SAT_WIDTH);

        initDsp(APP->engine->getSampleRate());
    }

    void initDsp(float sr) {
        sampleRate = sr;
        // HAAS + propagation + margin. Always allocated at full size so toggling
        // Doppler never reallocates while audio runs.
        int voiceDelaySamples = (int)ceilf((HAAS_MAX_SEC + PROPAGATION_MAX_SEC) * sr) + 16;
        for (int r = 0; r < NUM_RAILS; ++r)
            for (int g = 0; g < RAIL_GROUPS; ++g) {
                groups[r][g].delay.init(voiceDelaySamples);
                groups[r][g].resetState();
                groups[r][g].active = false;
            }
        room.init(sr);
        room.setRate(roomFullRate ? sr : 0.5f * sr);
        roomFullRateActive = roomFullRate;
        roomDecimator.setup();
        roomInterpolator.setup();
        roomDecimator.reset();
        roomInterpolator.reset();
        roomHeld  = float_4(0.f);
        roomPhase = false;
        roomSnapPending = true;

        delaySlewCoeff    = 1.f - expf(-1.f / (DELAY_SLEW_SEC * sr));
        dopplerCoeff      = 1.f - expf(-1.f / (DOPPLER_SMOOTH_SEC * sr));
        dopplerMaxStep    = DOPPLER_MAX_SPEED / SPEED_OF_SOUND;
        levelReleaseCoeff = expf(-1.f / (LEVEL_RELEASE_SEC * sr));
        smoothCoeff       = 1.f - expf(-1.f / (GAIN_SLEW_SEC * sr));
        mixAttackCoeff    = 1.f - expf(-1.f / (MIX_ATTACK_SEC * sr));
        mixReleaseCoeff   = 1.f - expf(-1.f / (MIX_RELEASE_SEC * sr));
        tube.setup();
        tube.reset();
        dcCoeff = 1.f - 2.f * float(M_PI) * 20.f / sr;
        mainMuteCoeff     = 1.f - expf(-1.f / (MUTE_FADE_SEC * sr));
        controlCounter    = CONTROL_DIV;   // force a control update on the next sample
    }

    void onSampleRateChange(const SampleRateChangeEvent& e) override {
        initDsp(e.sampleRate);
    }

    void onReset() override {
        for (int r = 0; r < NUM_RAILS; ++r) {
            motionShape[r]    = SHAPE_SINE;
            motionPhase[r]    = 0.f;
            motionAmount[r]   = 0.f;
            modeBlend[r]      = 1.f;
            skimming[r]       = false;
            for (int g = 0; g < RAIL_GROUPS; ++g) { groups[r][g].resetState(); groups[r][g].active = false; }
        }
        doppler  = true;
        roomFullRate = false;
        ringTurn = 0.f;
        haasMode = 0;
        mixEnv   = 0.f;
        potGain  = 1.f;
        potGainStep = 0.f;
        room.clear();
        roomDecimator.reset();
        roomInterpolator.reset();
        roomHeld = float_4(0.f);
        busSatL.reset();
        busSatR.reset();
        tube.reset();
        warmState = dcInLast = dcOutLast = float_4(0.f);
        roomSnapPending = true;
    }

    json_t* dataToJson() override {
        json_t* root = json_object();
        for (int r = 0; r < NUM_RAILS; ++r) {
            std::string key = "motionShape" + std::to_string(r);
            json_object_set_new(root, key.c_str(), json_integer(motionShape[r]));
        }
        json_object_set_new(root, "doppler",  json_boolean(doppler));
        json_object_set_new(root, "haasMode", json_integer(haasMode));
        json_object_set_new(root, "roomFullRate", json_boolean(roomFullRate));
        return root;
    }

    void dataFromJson(json_t* root) override {
        for (int r = 0; r < NUM_RAILS; ++r) {
            std::string key = "motionShape" + std::to_string(r);
            json_t* shapeJ = json_object_get(root, key.c_str());
            if (shapeJ) motionShape[r] = clamp((int)json_integer_value(shapeJ), 0, 2);
        }
        json_t* dopplerJ = json_object_get(root, "doppler");
        if (dopplerJ) doppler = json_boolean_value(dopplerJ);
        json_t* fullRateJ = json_object_get(root, "roomFullRate");
        if (fullRateJ) roomFullRate = json_boolean_value(fullRateJ);
        json_t* haasJ = json_object_get(root, "haasMode");
        if (haasJ) haasMode = clamp((int)json_integer_value(haasJ), 0, 3);
    }

    // Bypass: A inputs to left, B inputs to right (A normalled to both when B
    // is unpatched), all channels summed.
    void processBypass(const ProcessArgs& args) override {
        float left = 0.f, right = 0.f;
        for (int r = 0; r < NUM_RAILS; ++r) {
            float a = inputs[IN_A_INPUT + r].getVoltageSum();
            left += a;
            right += inputs[IN_B_INPUT + r].isConnected() ? inputs[IN_B_INPUT + r].getVoltageSum() : a;
        }
        outputs[OUT_L_OUTPUT].setVoltage(left);
        outputs[OUT_R_OUTPUT].setVoltage(right);
    }

    // Motion waveform in [-1, 1] for a phase in cycles.
    float motionShapeValue(int shape, float phase) const {
        phase -= floorf(phase);
        if (shape == SHAPE_TRIANGLE) {
            float shifted = phase + 0.25f;
            shifted -= floorf(shifted);
            return 1.f - 4.f * fabsf(shifted - 0.5f);
        }
        if (shape == SHAPE_STEPPED) phase = hotPotSteppedPhase(phase);
        return sinf(2.f * float(M_PI) * phase);
    }

    // =========================================================================
    // Control-rate update: motion, geometry, cues, room parameters.
    // =========================================================================
    void controlUpdate() {
        const float sr        = sampleRate;
        const float tickSec   = (float)CONTROL_DIV / sr;
        const float sizeKnob  = clamp(params[SIZE_PARAM].getValue()
                                    + params[SIZE_ATT_PARAM].getValue() * inputs[SIZE_CV_INPUT].getVoltage() * 0.1f, 0.f, 1.f);
        const float sceneScale = SCENE_SCALE_MIN * powf(SCENE_SCALE_RANGE, sizeKnob);
        sceneScaleCurrent = sceneScale;
        const float cues      = clamp(params[CUES_PARAM].getValue()
                                    + params[CUES_ATT_PARAM].getValue() * inputs[CUES_CV_INPUT].getVoltage() * 0.3f, 0.f, 3.f);
        const float roomAmount = clamp(params[ROOM_PARAM].getValue()
                                     + params[ROOM_ATT_PARAM].getValue() * inputs[ROOM_CV_INPUT].getVoltage() * 0.1f, 0.f, 1.f);

        // Haas (context menu) stretches the far-ear maximum from natural ITD to HAAS_MAX_SEC.
        const float haasMaxSamples = itdMaxSec * powf(HAAS_MAX_SEC / itdMaxSec, HAAS_AMOUNTS[clamp(haasMode, 0, 3)]) * sr;

        // -- Motion reset button ----------------------------------------------
        // Skim glides everything home rather than jumping; the light stays on
        // until it has settled, then motion resumes from the start.
        if (resetButtonTrigger.process(params[RESET_PARAM].getValue()))
            for (int r = 0; r < NUM_RAILS; ++r) skimming[r] = true;
        lights[RESET_LIGHT].setBrightness((skimming[0] || skimming[1] || skimming[2]) ? 1.f : 0.f);

        // -- Values shared by every source this tick ---------------------------
        const float ildFloor = (cues <= 1.f) ? ILD_FLOOR : std::max(0.f, ILD_FLOOR * (2.f - cues));

        // Head shadow bilinear constant: t = 2 sr / (2 omega0'), omega0' = (c / a) / sqrt(max(CUES, 1)).
        const float shadowOmega = SPEED_OF_SOUND / HEAD_RADIUS_M / sqrtf(std::max(cues, 1.f));
        const float shadowT     = sr / shadowOmega;

        const float frontW   = 2.f * float(M_PI) * FRONT_PEAK_HZ / sr;
        const float frontCos = cosf(frontW);
        const float frontAlpha = sinf(frontW) / (2.f * FRONT_PEAK_Q);

        const float shelfSlide = clamp((cues - 1.f) * 0.5f, 0.f, 1.f);
        const float shelfHz    = REAR_SHELF_HZ + (REAR_SHELF_HZ_LOW - REAR_SHELF_HZ) * shelfSlide;
        const float shelfW     = 2.f * float(M_PI) * shelfHz / sr;
        const float shelfCos   = cosf(shelfW);
        const float shelfAlpha = sinf(shelfW) * 0.5f * float(M_SQRT2);   // shelf slope S = 1

        const float unityDistance = RING_RADIUS_M * sceneScale;
        const float inHeadRadius  = IN_HEAD_RADIUS_M * sceneScale;
        const float fadeCoeff      = 1.f - expf(-tickSec / MOTION_FADE_SEC);
        const float skimCoeff      = 1.f - expf(-tickSec / SKIM_SEC);
        const float modeCoeff      = 1.f - expf(-tickSec / MODE_FADE_SEC);
        const float muteCoeff      = 1.f - expf(-tickSec / MUTE_FADE_SEC);

        int totalSources = 0;

        for (int r = 0; r < NUM_RAILS; ++r) {
            const int nA = inputs[IN_A_INPUT + r].getChannels();
            const int nB = inputs[IN_B_INPUT + r].getChannels();
            const int n  = nA + nB;
            countA[r]      = nA;
            sourceCount[r] = n;
            totalSources  += n;

            // -- Motion --------------------------------------------------------
            // MOVE is the rate: off at the bottom, then exponential across the
            // whole slider. Rails fade their motion in and out; Stir keeps a
            // running turn that winds back home the short way when stopped.
            const float moveValue = clamp(params[MOVE_PARAM + r].getValue()
                                        + params[MOVE_ATT_PARAM + r].getValue() * inputs[MOVE_CV_INPUT + r].getVoltage() * 0.1f,
                                          0.f, 1.f);
            const bool moving = (moveValue > MOVE_OFF) && !skimming[r];
            if (moving) {
                const float hz = MOVE_RATE_MIN_HZ * powf(MOVE_RATE_RANGE, (moveValue - MOVE_OFF) / (1.f - MOVE_OFF));
                motionPhase[r] += hz * tickSec;
                motionPhase[r] -= floorf(motionPhase[r]);
                if (r == 0) {
                    ringTurn += hz * tickSec;
                    ringTurn -= floorf(ringTurn + 0.5f);   // a whole turn is invisible
                }
            }
            const float settleCoeff = skimming[r] ? skimCoeff : fadeCoeff;
            motionAmount[r] += ((moving ? 1.f : 0.f) - motionAmount[r]) * settleCoeff;
            if (r == 0 && !moving) ringTurn -= ringTurn * settleCoeff;
            // Once settled home, restart from phase zero so the next start is clean.
            if (!moving && motionAmount[r] < 1e-3f && (r != 0 || fabsf(ringTurn) < 1e-3f)) {
                motionAmount[r] = 0.f;
                motionPhase[r]  = 0.f;
                if (r == 0) ringTurn = 0.f;
                skimming[r] = false;
            }
            const float amount = motionAmount[r];
            const int   shape  = motionShape[r];

            const float modeTarget = (params[MODE_PARAM + r].getValue() > 0.5f) ? 1.f : 0.f;
            modeBlend[r] += (modeTarget - modeBlend[r]) * modeCoeff;
            const float blend = modeBlend[r];
            lights[MODE_LIGHT + r].setBrightness(modeTarget);

            // Rail mute fades the source level, so dry and send stop together
            // and the room tail rings out naturally.
            const bool muted = params[MUTE_PARAM + r].getValue() > 0.5f;
            muteGain[r] += ((muted ? 0.f : 1.f) - muteGain[r]) * muteCoeff;
            lights[MUTE_LIGHT + r].setBrightness(muted ? 1.f : 0.f);

            // -- Placement inputs ----------------------------------------------
            const float spreadNorm = clamp(params[SPREAD_PARAM + r].getValue()
                                         + params[SPREAD_ATT_PARAM + r].getValue() * inputs[SPREAD_CV_INPUT + r].getVoltage() * 0.1f,
                                           0.f, 1.f);

            // Level: slider gain times a 0-10 V VCA. The CV is normalled to
            // 10 V, and sources beyond a poly CV's channel count are unaffected.
            const float levelSlider   = params[LEVEL_PARAM + r].getValue() / LEVEL_UNITY_POS;
            const float levelSliderGain = levelSlider * levelSlider * muteGain[r];
            const float levelDepth    = params[LEVEL_ATT_PARAM + r].getValue();
            Input& levelCv            = inputs[LEVEL_CV_INPUT + r];
            const int   levelCvChannels = levelCv.getChannels();
            const float posKnob    = params[POS_PARAM + r].getValue();
            const float posAtt     = params[POS_ATT_PARAM + r].getValue();
            Input& posCv           = inputs[POS_CV_INPUT + r];
            const int   posCvChannels = posCv.getChannels();

            // Stir's turn, stepped if asked. The staircase maps -0.5 and +0.5
            // to exactly one turn apart, so the wrap stays invisible.
            const float ringCycles = (shape == SHAPE_STEPPED) ? hotPotSteppedPhase(ringTurn) : ringTurn;

            // Scratch per source, loaded into float_4 groups below.
            float airCoeff[RAIL_SOURCES + 4]  = {};
            float toneB0[RAIL_SOURCES + 4], toneB1[RAIL_SOURCES + 4], toneB2[RAIL_SOURCES + 4];
            float toneA1[RAIL_SOURCES + 4], toneA2[RAIL_SOURCES + 4];
            float delayTargetL[RAIL_SOURCES + 4], delayTargetR[RAIL_SOURCES + 4];
            float propagationTarget[RAIL_SOURCES + 4] = {};
            float gainL[RAIL_SOURCES + 4] = {}, gainR[RAIL_SOURCES + 4] = {};
            float sendL[RAIL_SOURCES + 4] = {}, sendR[RAIL_SOURCES + 4] = {};
            float shadowB0L[RAIL_SOURCES + 4], shadowB1L[RAIL_SOURCES + 4], shadowA1L[RAIL_SOURCES + 4];
            float shadowB0R[RAIL_SOURCES + 4], shadowB1R[RAIL_SOURCES + 4], shadowA1R[RAIL_SOURCES + 4];
            float levelGain[RAIL_SOURCES + 4] = {};
            for (int i = 0; i < RAIL_SOURCES + 4; ++i) {
                toneB0[i] = 1.f; toneB1[i] = toneB2[i] = toneA1[i] = toneA2[i] = 0.f;
                delayTargetL[i] = delayTargetR[i] = MIN_TAP_DELAY;
                shadowB0L[i] = shadowB0R[i] = 1.f;
                shadowB1L[i] = shadowB1R[i] = shadowA1L[i] = shadowA1R[i] = 0.f;
            }

            for (int i = 0; i < n; ++i) {
                // -- Home placement along the span ------------------------------
                float cvVolts = 0.f;
                if (posCvChannels == 1)     cvVolts = posCv.getVoltage(0);
                else if (i < posCvChannels) cvVolts = posCv.getVoltage(i);
                const float posHome = posKnob + posAtt * cvVolts * 0.2f;   // +-5 V covers the whole rail

                float levelVolts = 10.f;
                if (levelCvChannels == 1)     levelVolts = levelCv.getVoltage(0);
                else if (i < levelCvChannels) levelVolts = levelCv.getVoltage(i);
                const float sourceLevel = levelSliderGain
                                        * (1.f - levelDepth + levelDepth * clamp(levelVolts * 0.1f, 0.f, 1.f));

                // Fan splits its sources between the left and right tracks: A
                // left and B right, or alternating when only A is patched. Each
                // side spreads and staggers on its own, so L/R pairs mirror.
                int   sideIndex = i, sideCount = n;
                float fanSide = 0.f;
                if (r == 2) {
                    const int nA = countA[r], nB = n - nA;
                    if (nB > 0) {
                        const bool onLeft = i < nA;
                        fanSide   = onLeft ? -1.f : 1.f;
                        sideIndex = onLeft ? i : i - nA;
                        sideCount = onLeft ? nA : nB;
                    } else {
                        const bool onLeft = (i % 2) == 0;
                        fanSide   = onLeft ? -1.f : 1.f;
                        sideIndex = i / 2;
                        sideCount = onLeft ? (n + 1) / 2 : n / 2;
                    }
                }

                // Evenly spaced, centered offsets: -0.5 .. +0.5 of the span.
                const float centered = (sideCount > 1) ? (float)sideIndex / (float)(sideCount - 1) - 0.5f : 0.f;

                float x = 0.f, y = 0.f;
                if (r == 0) {
                    // Stir: spacing = span / (n - 1), crossfading to span / n at full circle.
                    const float spanDeg   = spreadNorm * 360.f;
                    const float wrapBlend = clamp((spreadNorm - 0.9f) * 10.f, 0.f, 1.f);
                    const float divisor   = std::max((float)(n - 1) + wrapBlend, 1.f);
                    const float offsetDeg = ((float)i - 0.5f * (float)(n - 1)) * spanDeg / divisor;

                    // Parallel: every source turns together. Mirrored: the first
                    // half turns one way, the second half the other, and a middle
                    // source holds. A lone source always turns.
                    const int   twiceIndex = 2 * i + 1;
                    const float mirrorSign = (n < 2) ? 1.f : ((twiceIndex < n) ? 1.f : ((twiceIndex > n) ? -1.f : 0.f));
                    const float turnSign   = 1.f + (mirrorSign - 1.f) * blend;

                    const float angle  = (posHome * 180.f + offsetDeg + 360.f * ringCycles * turnSign) * float(M_PI) / 180.f;
                    const float radius = RING_RADIUS_M * sceneScale;
                    x = radius * sinf(angle);
                    y = radius * cosf(angle);
                } else {
                    // Rails move a rigid formation: the sources keep their order
                    // and spacing and never pass through one another. The
                    // formation's half-width is limited by the room left between
                    // its center and the nearer rail end, so it closes up as it
                    // nears an end instead of folding back.
                    const float home      = clamp(posHome, -1.f, 1.f);
                    const float reach     = 1.f - fabsf(home);            // travel from home to the nearer end
                    const float formation = centered * 2.f;               // -1 .. +1 across this side's sources
                    const float swing     = motionShapeValue(shape, motionPhase[r]);

                    const float restAlong = home + formation * std::min(spreadNorm, reach);

                    // A sweep of the whole formation about home, out to the ends.
                    // Fan's parallel mode sweeps the two tracks in opposite directions.
                    const float sweepCenter = home + swing * reach * ((r == 2) ? fanSide : 1.f);
                    const float alongSweep  = sweepCenter + formation * std::min(spreadNorm, 1.f - fabsf(sweepCenter));
                    const float mirrorCenter = home + swing * reach;
                    const float alongMirrorSweep = mirrorCenter + formation * std::min(spreadNorm, 1.f - fabsf(mirrorCenter));

                    float alongParallel, alongMirrored;
                    if (r == 1) {
                        // Shabu. Parallel: the formation swishes side to side.
                        // Mirrored: the two halves open and close about home,
                        // from together out to the rail ends. A lone source swishes.
                        alongParallel = alongSweep;
                        alongMirrored = (sideCount > 1) ? home + formation * reach * (0.5f + 0.5f * swing)
                                                        : alongSweep;
                    } else {
                        // Fan. Mirrored: both tracks move front and back together.
                        // Parallel: the two tracks rock against each other.
                        alongParallel = alongSweep;
                        alongMirrored = alongMirrorSweep;
                    }
                    const float alongMoving = alongParallel + (alongMirrored - alongParallel) * blend;
                    const float along = restAlong + (alongMoving - restAlong) * amount;

                    if (r == 1) {
                        x = along * RAIL_HALF_LENGTH_M * sceneScale;
                        y = STAGE_DISTANCE_M * sceneScale;
                    } else {
                        const float toFront = 0.5f * (along + 1.f);   // 0 = rear end, 1 = front end
                        x = fanSide * (FAN_BACK_HALF_M + toFront * (FAN_FRONT_HALF_M - FAN_BACK_HALF_M)) * sceneScale;
                        y = (FAN_BACK_Y_M + toFront * (FAN_FRONT_Y_M - FAN_BACK_Y_M)) * sceneScale;
                    }
                }
                sourceX[r][i] = x / sceneScale;
                sourceY[r][i] = y / sceneScale;

                // -- Cues -----------------------------------------------------
                const float distance = sqrtf(x * x + y * y);
                const float safeDist = std::max(distance, 1e-6f);
                const float sinTheta = x / safeDist;    // +1 = right
                const float cosTheta = y / safeDist;    // +1 = front

                // In-head fade: directional cues go neutral near the center.
                float headRatio = clamp(distance / inHeadRadius, 0.f, 1.f);
                const float headFade = headRatio * headRatio * (3.f - 2.f * headRatio);

                const float lateral   = sinTheta * headFade;
                const float frontness = cosTheta * headFade;

                // Distance gain, level-normalized to the ring radius.
                const float spiceExtra = std::max(cues - 1.f, 0.f);
                const float distanceRatio = std::max(distance, unityDistance * 0.25f) / unityDistance;
                float distanceGain = powf(distanceRatio, -(std::min(cues, 1.f) + SPICE_DISTANCE_EXTRA * spiceExtra));
                distanceGain = std::min(distanceGain, NEAR_GAIN_CAP);
                levelGain[i] = distanceGain * sourceLevel;

                // Air absorption, absolute meters.
                float airHz = clamp(20000.f / (1.f + cues * distance / AIR_DISTANCE_M), AIR_MIN_HZ, 20000.f);
                airCoeff[i] = (airHz >= 19999.f) ? 0.f : expf(-2.f * float(M_PI) * airHz / sr);

                // Front/back tone: presence peak in front, high shelf behind.
                if (frontness > 1e-4f) {
                    float gainDb = FRONT_PEAK_DB * cues * frontness;
                    float amp    = powf(10.f, gainDb / 40.f);
                    float a0     = 1.f + frontAlpha / amp;
                    toneB0[i] = (1.f + frontAlpha * amp) / a0;
                    toneB1[i] = -2.f * frontCos / a0;
                    toneB2[i] = (1.f - frontAlpha * amp) / a0;
                    toneA1[i] = -2.f * frontCos / a0;
                    toneA2[i] = (1.f - frontAlpha / amp) / a0;
                } else if (frontness < -1e-4f) {
                    float gainDb = REAR_SHELF_DB * cues * (-frontness);
                    float amp    = powf(10.f, gainDb / 40.f);
                    float twoSqrtAmpAlpha = 2.f * sqrtf(amp) * shelfAlpha;
                    float a0 = (amp + 1.f) - (amp - 1.f) * shelfCos + twoSqrtAmpAlpha;
                    toneB0[i] =  amp * ((amp + 1.f) + (amp - 1.f) * shelfCos + twoSqrtAmpAlpha) / a0;
                    toneB1[i] = -2.f * amp * ((amp - 1.f) + (amp + 1.f) * shelfCos) / a0;
                    toneB2[i] =  amp * ((amp + 1.f) + (amp - 1.f) * shelfCos - twoSqrtAmpAlpha) / a0;
                    toneA1[i] =  2.f * ((amp - 1.f) - (amp + 1.f) * shelfCos) / a0;
                    toneA2[i] = ((amp + 1.f) - (amp - 1.f) * shelfCos - twoSqrtAmpAlpha) / a0;
                }

                // ITD (Woodworth, lateral angle) stretched by HAAS; far ear only.
                const float lateralMagnitude = fabsf(lateral);
                const float farDelay = std::min((asinf(std::min(lateralMagnitude, 1.f)) + lateralMagnitude)
                                                / (0.5f * float(M_PI) + 1.f) * haasMaxSamples
                                                * (1.f + SPICE_ITD_WIDEN * spiceExtra),
                                                HAAS_MAX_SEC * sr);
                // Propagation is physical distance only; Spice does not exaggerate it.
                propagationTarget[i] = doppler ? std::min(distance / SPEED_OF_SOUND, PROPAGATION_MAX_SEC) * sr : 0.f;
                delayTargetL[i] = MIN_TAP_DELAY + ((lateral > 0.f) ? farDelay : 0.f);
                delayTargetR[i] = MIN_TAP_DELAY + ((lateral < 0.f) ? farDelay : 0.f);

                // Constant-power pan with a partial law so the far ear keeps a floor.
                const float panAngle = 0.25f * float(M_PI) * (lateral + 1.f);
                const float panL = cosf(panAngle);
                const float panR = sinf(panAngle);
                const float floorSq = ildFloor * ildFloor;
                gainL[i] = sourceLevel * distanceGain * sqrtf(floorSq + (1.f - floorSq) * panL * panL);
                gainR[i] = sourceLevel * distanceGain * sqrtf(floorSq + (1.f - floorSq) * panR * panR);

                // Room send, post level: plain constant-power pan, constant with distance.
                // Past 100% Spice, distance also sets the wet/dry balance.
                const float rearBoost = (1.f + REAR_SEND_BOOST * cues * std::max(0.f, -frontness))
                                      * clamp(powf(distanceRatio, SPICE_WET_EXPONENT * spiceExtra), 0.25f, 4.f);
                sendL[i] = sourceLevel * roomAmount * panL * rearBoost;
                sendR[i] = sourceLevel * roomAmount * panR * rearBoost;

                // Head shadow (Brown-Duda), angle from each ear's axis.
                const float thetaEarR = acosf(clamp(sinTheta, -1.f, 1.f));
                const float thetaEarL = acosf(clamp(-sinTheta, -1.f, 1.f));
                const float alphaBase = 1.f + 0.5f * SHADOW_ALPHA_MIN;
                const float alphaSwing = 1.f - 0.5f * SHADOW_ALPHA_MIN;
                float alphaR = alphaBase + alphaSwing * cosf(thetaEarR * SHADOW_THETA_SCALE);
                float alphaL = alphaBase + alphaSwing * cosf(thetaEarL * SHADOW_THETA_SCALE);
                alphaR = clamp(1.f + (alphaR - 1.f) * cues * headFade, SHADOW_ALPHA_LOW, SHADOW_ALPHA_HIGH);
                alphaL = clamp(1.f + (alphaL - 1.f) * cues * headFade, SHADOW_ALPHA_LOW, SHADOW_ALPHA_HIGH);
                const float shadowNorm = 1.f / (1.f + shadowT);
                shadowB0L[i] = (1.f + alphaL * shadowT) * shadowNorm;
                shadowB1L[i] = (1.f - alphaL * shadowT) * shadowNorm;
                shadowA1L[i] = (1.f - shadowT) * shadowNorm;
                shadowB0R[i] = (1.f + alphaR * shadowT) * shadowNorm;
                shadowB1R[i] = (1.f - alphaR * shadowT) * shadowNorm;
                shadowA1R[i] = (1.f - shadowT) * shadowNorm;
            }

            // -- Load targets into float_4 groups ------------------------------
            const float stepScale = 1.f / (float)CONTROL_DIV;
            for (int g = 0; g < RAIL_GROUPS; ++g) {
                HotPotVoiceGroup& group = groups[r][g];
                const bool needed = g * 4 < n;
                if (!needed) { group.active = false; continue; }
                if (!group.active) {
                    // Newly active: start silent and ramp in from a clean state.
                    group.resetState();
                    group.active = true;
                }
                const int base = g * 4;
                group.airCoeff = float_4::load(&airCoeff[base]);
                group.toneB0   = float_4::load(&toneB0[base]);
                group.toneB1   = float_4::load(&toneB1[base]);
                group.toneB2   = float_4::load(&toneB2[base]);
                group.toneA1   = float_4::load(&toneA1[base]);
                group.toneA2   = float_4::load(&toneA2[base]);
                group.delayTargetL = float_4::load(&delayTargetL[base]);
                group.delayTargetR = float_4::load(&delayTargetR[base]);
                group.propagationTarget = float_4::load(&propagationTarget[base]);
                // A new group, or Doppler just switched, starts at its distance
                // instead of gliding there from zero.
                if (group.snapPropagation) {
                    // A new group also starts with its ear delays in place.
                    group.delayL = group.delayTargetL;
                    group.delayR = group.delayTargetR;
                }
                if (group.snapPropagation || doppler != dopplerWasOn) {
                    group.propagation     = group.propagationTarget;
                    group.snapPropagation = false;
                }
                group.gainStepL = (float_4::load(&gainL[base]) - group.gainL) * stepScale;
                group.gainStepR = (float_4::load(&gainR[base]) - group.gainR) * stepScale;
                group.sendStepL = (float_4::load(&sendL[base]) - group.sendL) * stepScale;
                group.sendStepR = (float_4::load(&sendR[base]) - group.sendR) * stepScale;
                group.shadowB0L = float_4::load(&shadowB0L[base]);
                group.shadowB1L = float_4::load(&shadowB1L[base]);
                group.shadowA1L = float_4::load(&shadowA1L[base]);
                group.shadowB0R = float_4::load(&shadowB0R[base]);
                group.shadowB1R = float_4::load(&shadowB1R[base]);
                group.shadowA1R = float_4::load(&shadowA1R[base]);
                group.levelGain = float_4::load(&levelGain[base]);
            }
        }

        // -- Globals ----------------------------------------------------------
        dopplerWasOn = doppler;
        {
            // The largest far-ear delay this tick (Haas and Spice widening
            // included) may take no less than ITD_MIN_SWEEP_SEC to cross.
            const float widestFarDelay = std::min(haasMaxSamples * (1.f + SPICE_ITD_WIDEN * std::max(cues - 1.f, 0.f)),
                                                  HAAS_MAX_SEC * sr);
            // A delay changing by k samples per sample shifts pitch by a ratio of 1 - k.
            const float centsStep = 1.f - exp2f(-ITD_MAX_CENTS / 1200.f);
            itdMaxStep = std::max(centsStep, widestFarDelay / (ITD_MIN_SWEEP_SEC * sr));
        }
        // Headroom anticipation from the number of sources in the pot.
        countGainTarget = 1.f / sqrtf(1.f + HEADROOM_PER_SOURCE * (float)std::max(totalSources - 1, 0));

        // Mix compressor gain from the peak follower: soft knee in dB, then
        // (1 - 1 / ratio) of every dB over the threshold is taken back.
        {
            const float overDb = 20.f * log10f(std::max(mixEnv, 1e-6f) / MIX_THRESHOLD_V);
            const float slope  = 1.f - 1.f / MIX_RATIO;
            float reductionDb  = 0.f;
            if (overDb >= 0.5f * MIX_KNEE_DB) {
                reductionDb = slope * overDb;
            } else if (overDb > -0.5f * MIX_KNEE_DB) {
                const float intoKnee = overDb + 0.5f * MIX_KNEE_DB;
                reductionDb = slope * intoKnee * intoKnee / (2.f * MIX_KNEE_DB);
            }
            // Linear ramp to the new gain across the next block.
            potGainStep = (powf(10.f, -reductionDb / 20.f) - potGain) / (float)CONTROL_DIV;
        }
        const float mainKnob = clamp(params[MAIN_PARAM].getValue()
                                   + params[MAIN_ATT_PARAM].getValue() * inputs[MAIN_CV_INPUT].getVoltage() * 0.1f, 0.f, 1.f);
        // Lower half: volume, square law to unity. Upper half: tube drive in dB,
        // with the bias and the warmth rolloff following it.
        const float driveNorm = std::max(0.f, (mainKnob - 0.5f) * 2.f);
        volumeTarget = std::min(4.f * mainKnob * mainKnob, 1.f);
        driveTarget  = powf(10.f, driveNorm * TUBE_DRIVE_MAX_DB / 20.f);
        biasTarget   = TUBE_BIAS_MAX * driveNorm;
        {
            // Makeup: undo half (in dB) of the compression the reference level
            // sees at this drive, so drive changes the character more than the level.
            const float refIn = TUBE_MAKEUP_REF_V * driveTarget / TUBE_HEADROOM_V;
            const float compression = 1.f / sqrtf(1.f + refIn * refIn);   // g(v) / v
            makeupTarget = 1.f / sqrtf(compression);
        }
        warmCoeff = (driveNorm < 1e-3f) ? 0.f
                  : expf(-2.f * float(M_PI) * 20000.f * powf(TUBE_WARM_HZ / 20000.f, driveNorm) / sr);

        const float decayKnob = clamp(params[DECAY_PARAM].getValue()
                                    + params[DECAY_ATT_PARAM].getValue() * inputs[DECAY_CV_INPUT].getVoltage() * 0.1f, 0.f, 1.f);
        const float hazeKnob  = clamp(params[HAZE_PARAM].getValue()
                                    + params[HAZE_ATT_PARAM].getValue()  * inputs[HAZE_CV_INPUT].getVoltage()  * 0.1f, 0.f, 1.f);
        const float diffuseAmount = std::min(hazeKnob * HAZE_DIFFUSE_SCALE, 1.f);
        const float modAmount     = std::max(0.f, (hazeKnob - HAZE_MOD_START) / (1.f - HAZE_MOD_START));
        const float dampKnob  = clamp(params[DAMP_PARAM].getValue()
                                    + params[DAMP_ATT_PARAM].getValue()  * inputs[DAMP_CV_INPUT].getVoltage()  * 0.1f, 0.f, 1.f);
        // The room's coefficients only need rebuilding when one of its inputs moves.
        const float roomArgs[5] = { sizeKnob, decayKnob, dampKnob, diffuseAmount, modAmount };
        bool roomChanged = roomSnapPending;
        for (int k = 0; k < 5; ++k) roomChanged = roomChanged || (roomArgs[k] != lastRoomArgs[k]);
        if (roomChanged) {
            room.setParams(sizeKnob, decayKnob, dampKnob, diffuseAmount, modAmount, roomSnapPending);
            for (int k = 0; k < 5; ++k) lastRoomArgs[k] = roomArgs[k];
        }
        roomSnapPending = false;
    }

    // =========================================================================
    // Display snapshot
    // =========================================================================
    void publishDisplay() {
        const int writeIndex = 1 - displayPublished.load(std::memory_order_relaxed);
        HotPotDisplayFrame& frame = displayFrames[writeIndex];
        for (int r = 0; r < NUM_RAILS; ++r) {
            const int n = sourceCount[r];
            for (int i = 0; i < RAIL_SOURCES; ++i) {
                HotPotDisplaySource& source = frame.sources[r * RAIL_SOURCES + i];
                source.active = i < n;
                if (!source.active) continue;
                float lanes[4];
                groups[r][i / 4].level.store(lanes);
                source.x     = sourceX[r][i];
                source.y     = sourceY[r][i];
                source.level = lanes[i % 4];
                source.rail  = r;
                source.isB   = i >= countA[r];
            }
        }
        frame.sceneScale = sceneScaleCurrent;
        frame.wetLevel   = wetFollower;
        displayPublished.store(writeIndex, std::memory_order_release);
    }

    // =========================================================================
    // Audio
    // =========================================================================
    void process(const ProcessArgs& args) override {
        if (++controlCounter >= CONTROL_DIV) {
            controlCounter = 0;
            controlUpdate();
        }

        float_4 busL  = float_4(0.f), busR  = float_4(0.f);
        float_4 sendBusL = float_4(0.f), sendBusR = float_4(0.f);

        for (int r = 0; r < NUM_RAILS; ++r) {
            const int n = sourceCount[r];
            if (n == 0) continue;

            // Gather A then B channels contiguously; zero the tail of the last group.
            float* gathered = railInput[r];
            inputs[IN_A_INPUT + r].readVoltages(gathered);
            inputs[IN_B_INPUT + r].readVoltages(gathered + countA[r]);
            const int paddedEnd = ((n + 3) / 4) * 4;
            for (int i = n; i < paddedEnd; ++i) gathered[i] = 0.f;

            for (int g = 0; g * 4 < n; ++g) {
                HotPotVoiceGroup& group = groups[r][g];
                float_4 in = float_4::load(&gathered[g * 4]);

                // Air absorption one-pole.
                group.airState = group.airCoeff * (group.airState - in) + in;

                // Front/back tone, TDF2.
                float_4 tone = group.toneB0 * group.airState + group.toneZ1;
                group.toneZ1 = group.toneB1 * group.airState - group.toneA1 * tone + group.toneZ2;
                group.toneZ2 = group.toneB2 * group.airState - group.toneA2 * tone;

                // Delay line and slewed ear taps.
                group.delay.write(tone);
                group.delayL += simd::clamp((group.delayTargetL - group.delayL) * delaySlewCoeff,
                                            float_4(-itdMaxStep), float_4(itdMaxStep));
                group.delayR += simd::clamp((group.delayTargetR - group.delayR) * delaySlewCoeff,
                                            float_4(-itdMaxStep), float_4(itdMaxStep));
                group.propagation += simd::clamp((group.propagationTarget - group.propagation) * dopplerCoeff,
                                                 float_4(-dopplerMaxStep), float_4(dopplerMaxStep));
                float_4 earL, earR;
                group.delay.read(group.delayL + group.propagation, group.delayR + group.propagation, earL, earR);
                earL *= group.gainL;
                earR *= group.gainR;

                // Head shadow per ear.
                float_4 shadowedL = group.shadowB0L * earL + group.shadowB1L * group.shadowInL - group.shadowA1L * group.shadowOutL;
                float_4 shadowedR = group.shadowB0R * earR + group.shadowB1R * group.shadowInR - group.shadowA1R * group.shadowOutR;
                group.shadowInL = earL; group.shadowOutL = shadowedL;
                group.shadowInR = earR; group.shadowOutR = shadowedR;

                busL += shadowedL;
                busR += shadowedR;
                sendBusL += tone * group.sendL;
                sendBusR += tone * group.sendR;

                group.gainL += group.gainStepL;
                group.gainR += group.gainStepR;
                group.sendL += group.sendStepL;
                group.sendR += group.sendStepR;

                group.level = simd::fmax(simd::fabs(tone) * group.levelGain, group.level * levelReleaseCoeff);
            }
        }

        const float dryL  = busL[0] + busL[1] + busL[2] + busL[3];
        const float dryR  = busR[0] + busR[1] + busR[2] + busR[3];
        const float sendL = sendBusL[0] + sendBusL[1] + sendBusL[2] + sendBusL[3];
        const float sendR = sendBusR[0] + sendBusR[1] + sendBusR[2] + sendBusR[3];

        // -- Headroom ------------------------------------------------------------
        // The source-count gain applies to dry and send alike, so the room
        // balance holds as the pot fills.
        countGainCurrent += (countGainTarget - countGainCurrent) * smoothCoeff;

        // The menu option takes effect here, between samples: the tank is
        // retuned and cleared, its buffers already sized for either rate.
        if (roomFullRate != roomFullRateActive) {
            roomFullRateActive = roomFullRate;
            room.setRate(roomFullRate ? sampleRate : 0.5f * sampleRate);
            roomDecimator.reset();
            roomInterpolator.reset();
            roomHeld = float_4(0.f);
            roomSnapPending = true;
            lastRoomArgs[0] = -1.f;
        }

        float wetL = 0.f, wetR = 0.f;
        if (roomFullRateActive) {
            room.process(sendL * countGainCurrent, sendR * countGainCurrent, wetL, wetR);
        } else {
            // Half-rate room: band-limit the send, run the tank on every other
            // sample, and rebuild the full rate from zero-stuffed output (x2
            // keeps the level).
            const float_4 sendBand = roomDecimator.process(float_4(sendL * countGainCurrent, sendR * countGainCurrent, 0.f, 0.f));
            if (!roomPhase) {
                float tankL = 0.f, tankR = 0.f;
                room.process(sendBand[0], sendBand[1], tankL, tankR);
                roomHeld = float_4(tankL, tankR, 0.f, 0.f);
            }
            const float_4 wet = roomInterpolator.process(roomPhase ? float_4(0.f) : roomHeld * 2.f);
            roomPhase = !roomPhase;
            wetL = wet[0];
            wetR = wet[1];
        }
        wetFollower += (fabsf(wetL) + fabsf(wetR) - wetFollower) * smoothCoeff;

        const float mixL = dryL * countGainCurrent + wetL;
        const float mixR = dryR * countGainCurrent + wetR;
        const float mixPeak = std::max(fabsf(mixL), fabsf(mixR));
        mixEnv  += (mixPeak - mixEnv) * ((mixPeak > mixEnv) ? mixAttackCoeff : mixReleaseCoeff);
        potGain += potGainStep;

        // -- Simmer: volume, then the tube stage ----------------------------------
        volumeCurrent += (volumeTarget - volumeCurrent) * smoothCoeff;
        driveCurrent  += (driveTarget  - driveCurrent)  * smoothCoeff;
        biasCurrent   += (biasTarget   - biasCurrent)   * smoothCoeff;
        makeupCurrent += (makeupTarget - makeupCurrent) * smoothCoeff;
        const bool mainMuted = params[MAIN_MUTE_PARAM].getValue() > 0.5f;
        mainMuteGain += ((mainMuted ? 0.f : 1.f) - mainMuteGain) * mainMuteCoeff;
        lights[MAIN_MUTE_LIGHT].setBrightness(mainMuted ? 1.f : 0.f);

        // Tube stage (see HotPotTube). The static bias offset g(b) is removed,
        // and the output is divided by the small-signal slope g'(b) =
        // (1 + b^2)^-1.5, so low levels pass at unity.
        const float inScale   = volumeCurrent * potGain * driveCurrent / TUBE_HEADROOM_V;
        const float biasSq    = 1.f + biasCurrent * biasCurrent;
        const float biasSqrt  = sqrtf(biasSq);
        const float biasOut   = biasCurrent / biasSqrt;
        const float outScale  = biasSq * biasSqrt * TUBE_HEADROOM_V / driveCurrent * makeupCurrent;
        const float_4 tubeOut = tube.process(float_4(mixL, mixR, 0.f, 0.f), inScale, biasCurrent, biasOut, outScale);

        // Warmth: a gentle post-drive rolloff that closes as drive rises, then
        // the DC the asymmetry leaves behind is taken out.
        warmState += (tubeOut - warmState) * (1.f - warmCoeff);
        dcOutLast  = warmState - dcInLast + dcCoeff * dcOutLast;
        dcInLast   = warmState;
        const float warmL = dcOutLast[0];
        const float warmR = dcOutLast[1];

        float outL = busSatL.process(warmL) * mainMuteGain;
        float outR = busSatR.process(warmR) * mainMuteGain;

        // Non-finite recovery: clear every stateful stage.
        if (!std::isfinite(outL) || !std::isfinite(outR)) {
            outL = outR = 0.f;
            room.clear();
            roomDecimator.reset();
            roomInterpolator.reset();
            roomHeld = float_4(0.f);
            busSatL.reset();
            busSatR.reset();
            wetFollower = 0.f;
            mixEnv      = 0.f;
            potGain     = 1.f;
            potGainStep = 0.f;
            tube.reset();
            warmState = dcInLast = dcOutLast = float_4(0.f);
            for (int r = 0; r < NUM_RAILS; ++r)
                for (int g = 0; g < RAIL_GROUPS; ++g) { groups[r][g].resetState(); groups[r][g].active = false; }
        }

        outputs[OUT_L_OUTPUT].setVoltage(outL);
        outputs[OUT_R_OUTPUT].setVoltage(outR);

        if (++displayCounter >= DISPLAY_DIV) {
            displayCounter = 0;
            publishDisplay();
        }
    }
};

// =============================================================================
// Display -- the pot, seen from above, front up. A map view rather than a
// scale drawing: the ring runs around the rim, the stage is a line across
// the circle in front of center, and the depth rail runs down the middle.
// Each source is drawn at its position along its own track.
// =============================================================================
struct HotPotDisplay : TransparentWidget {
    HotPot* module = nullptr;

    // Rail colors: Stir, Shabu (chili), Fan (yellow).
    const NVGcolor railColor[NUM_RAILS] = {
        nvgRGBf(0.20f, 0.80f, 0.90f),
        nvgRGBf(1.00f, 0.42f, 0.22f),
        nvgRGBf(1.00f, 0.85f, 0.25f),
    };

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) { TransparentWidget::drawLayer(args, layer); return; }

        const float centerX = 0.5f * box.size.x;
        const float centerY = 0.5f * box.size.y;
        const float potRadius   = 0.5f * std::min(box.size.x, box.size.y) - 1.f;
        // Inset so the largest dot (2.8 + 2.4 px) stays well inside the pot.
        const float ringRadius  = potRadius - 9.f;          // Stir track, inside the rim
        const float innerRadius = ringRadius - 7.f;         // Shabu and Fan stay inside Stir
        const float stageOffset = 0.5f * innerRadius;       // Shabu line distance in front of center
        const float stageHalf   = sqrtf(innerRadius * innerRadius - stageOffset * stageOffset);
        const float stageY      = centerY - stageOffset;

        // Fan tracks are drawn to scale from their geometry, sized so the
        // farthest end sits at 85% of innerRadius.
        const float fanPixelsPerMeter = 0.85f * innerRadius
            / sqrtf(FAN_FRONT_HALF_M * FAN_FRONT_HALF_M + FAN_FRONT_Y_M * FAN_FRONT_Y_M);
        const float fanBackX  = FAN_BACK_HALF_M  * fanPixelsPerMeter, fanBackY  = centerY - FAN_BACK_Y_M  * fanPixelsPerMeter;
        const float fanFrontX = FAN_FRONT_HALF_M * fanPixelsPerMeter, fanFrontY = centerY - FAN_FRONT_Y_M * fanPixelsPerMeter;

        HotPotDisplayFrame frame;
        if (module) {
            frame = module->displayFrames[module->displayPublished.load(std::memory_order_acquire)];
        } else {
            // Library preview: a small demo layout, in 1x meters.
            const float demo[6][3] = {
                { -2.8f,  2.8f, 0 }, { 3.5f, -2.0f, 0 },
                { -6.f,   8.f,  1 }, { 0.f,   8.f,  1 }, { 6.f, 8.f, 1 },
                { -6.25f, -3.f, 2 },
            };
            for (int i = 0; i < 6; ++i) {
                HotPotDisplaySource& source = frame.sources[i];
                source.x = demo[i][0]; source.y = demo[i][1]; source.rail = (int)demo[i][2];
                source.level = 3.f; source.active = true; source.isB = (i == 1);
            }
            frame.wetLevel = 1.f;
        }

        // -- Tracks ------------------------------------------------------------
        nvgStrokeWidth(args.vg, 0.8f);
        nvgBeginPath(args.vg);
        nvgCircle(args.vg, centerX, centerY, ringRadius);
        nvgStrokeColor(args.vg, nvgTransRGBAf(railColor[0], 0.45f));
        nvgStroke(args.vg);

        nvgBeginPath(args.vg);
        nvgMoveTo(args.vg, centerX - stageHalf, stageY);
        nvgLineTo(args.vg, centerX + stageHalf, stageY);
        nvgStrokeColor(args.vg, nvgTransRGBAf(railColor[1], 0.45f));
        nvgStroke(args.vg);

        nvgBeginPath(args.vg);
        nvgMoveTo(args.vg, centerX - fanBackX, fanBackY);
        nvgLineTo(args.vg, centerX - fanFrontX, fanFrontY);
        nvgMoveTo(args.vg, centerX + fanBackX, fanBackY);
        nvgLineTo(args.vg, centerX + fanFrontX, fanFrontY);
        nvgStrokeColor(args.vg, nvgTransRGBAf(railColor[2], 0.45f));
        nvgStroke(args.vg);

        // The listener's place at the center is taken by the Skim button.

        // -- Sources, placed along their own track ---------------------------
        for (int i = 0; i < TOTAL_SOURCES; ++i) {
            const HotPotDisplaySource& source = frame.sources[i];
            if (!source.active) continue;
            const int rail = clamp(source.rail, 0, NUM_RAILS - 1);

            float dotX = centerX, dotY = centerY;
            if (rail == 0) {
                const float angle = atan2f(source.x, source.y);   // 0 = front, clockwise
                dotX = centerX + ringRadius * sinf(angle);
                dotY = centerY - ringRadius * cosf(angle);
            } else if (rail == 1) {
                const float along = clamp(source.x / RAIL_HALF_LENGTH_M, -1.f, 1.f);
                dotX = centerX + along * stageHalf;
                dotY = stageY;
            } else {
                // Rear end -> front end of the track on the source's side.
                const float toFront = clamp((source.y - FAN_BACK_Y_M) / (FAN_FRONT_Y_M - FAN_BACK_Y_M), 0.f, 1.f);
                const float side    = (source.x < 0.f) ? -1.f : 1.f;
                dotX = centerX + side * (fanBackX + toFront * (fanFrontX - fanBackX));
                dotY = fanBackY + toFront * (fanFrontY - fanBackY);
            }

            const float brightness = clamp(source.level / 5.f, 0.f, 1.f);
            const float radius = 2.8f + 2.4f * brightness;
            const NVGcolor color = railColor[rail];

            nvgBeginPath(args.vg);
            nvgCircle(args.vg, dotX, dotY, radius);
            const float alpha = 0.25f + 0.75f * brightness;
            if (source.isB) {
                nvgStrokeColor(args.vg, nvgTransRGBAf(color, alpha));
                nvgStrokeWidth(args.vg, 1.4f);
                nvgStroke(args.vg);
            } else {
                nvgFillColor(args.vg, nvgTransRGBAf(color, alpha));
                nvgFill(args.vg);
            }
        }

        TransparentWidget::drawLayer(args, layer);
    }
};

// =============================================================================
// Widget -- 24HP. Coordinates in mm.
// =============================================================================
struct HotPotWidget : ModuleWidget {

    static Vec p(float x, float y) { return mm2px(Vec(x, y)); }

    // Same slider as the rest of the set, with a per-rail handle color.
    struct HotPotSliderBase : app::SvgSlider {
        HotPotSliderBase() {
            setBackgroundSvg(Svg::load(asset::plugin(pluginInstance, "res/components/ShortSlider.svg")));
            setHandleSvg    (Svg::load(asset::plugin(pluginInstance, "res/components/ShortSliderHandle.svg")));
            setHandlePosCentered(math::Vec(10.f, 55.f), math::Vec(10.f, 10.f));
        }
    };
    template <int RED, int GREEN, int BLUE>
    struct HotPotRailLight : GrayModuleLightWidget {
        HotPotRailLight() { addBaseColor(nvgRGB(RED, GREEN, BLUE)); }
    };
    typedef HotPotRailLight< 51, 204, 230> RingLight;    // matches the display's rail colors
    typedef HotPotRailLight<255, 107,  56> StageLight;
    typedef HotPotRailLight<255, 217,  64> DepthLight;

    // Latching version of the set's small button, for the motion mode.
    struct HotPotLatch : TL1105 {
        HotPotLatch() { momentary = false; latch = true; }
    };
    template <typename TL>
    struct HotPotSlider : LightSlider<HotPotSliderBase, VCVSliderLight<TL>> { HotPotSlider() {} };

    template <typename TL>
    void addStrip(HotPot* module, int rail, const float stripX[3],
                  float yButton, float ySlider, float yTrim, float yCV, float yKnobRow, float yInputs) {
        // Buttons: motion mode over the position slider (lit in the rail color
        // when mirrored), mute over the level slider.
        addParam(createParamCentered<HotPotLatch>(p(stripX[0], yButton), module, HotPot::MODE_PARAM + rail));
        addChild(createLightCentered<MediumLight<TL>>(p(stripX[0], yButton), module, HotPot::MODE_LIGHT + rail));
        addParam(createParamCentered<HotPotLatch>(p(stripX[2], yButton), module, HotPot::MUTE_PARAM + rail));
        addChild(createLightCentered<MediumLight<RedLight>>(p(stripX[2], yButton), module, HotPot::MUTE_LIGHT + rail));

        // Sliders: position, spread, level -- each over its trim and CV.
        const int sliderParams[3] = { HotPot::POS_PARAM + rail, HotPot::SPREAD_PARAM + rail, HotPot::LEVEL_PARAM + rail };
        const int trimParams[3]   = { HotPot::POS_ATT_PARAM + rail, HotPot::SPREAD_ATT_PARAM + rail, HotPot::LEVEL_ATT_PARAM + rail };
        const int cvInputs[3]     = { HotPot::POS_CV_INPUT + rail, HotPot::SPREAD_CV_INPUT + rail, HotPot::LEVEL_CV_INPUT + rail };
        for (int c = 0; c < 3; ++c) {
            addParam(createParamCentered<HotPotSlider<TL>>(p(stripX[c], ySlider), module, sliderParams[c]));
            addParam(createParamCentered<Trimpot>(p(stripX[c], yTrim), module, trimParams[c]));
            addInput(createInputCentered<ThemedPJ301MPort>(p(stripX[c], yCV), module, cvInputs[c]));
        }
        // Motion rate as knob, trim, CV across the strip.
        addParam(createParamCentered<RoundSmallBlackKnob>(p(stripX[0], yKnobRow), module, HotPot::MOVE_PARAM + rail));
        addParam(createParamCentered<Trimpot>(p(stripX[1], yKnobRow), module, HotPot::MOVE_ATT_PARAM + rail));
        addInput(createInputCentered<ThemedPJ301MPort>(p(stripX[2], yKnobRow), module, HotPot::MOVE_CV_INPUT + rail));
        // A and B inputs, centered under the strip.
        addInput(createInputCentered<ThemedPJ301MPort>(p(0.5f * (stripX[0] + stripX[1]), yInputs), module, HotPot::IN_A_INPUT + rail));
        addInput(createInputCentered<ThemedPJ301MPort>(p(0.5f * (stripX[1] + stripX[2]), yInputs), module, HotPot::IN_B_INPUT + rail));
    }

    HotPotWidget(HotPot* module) {
        setModule(module);
        box.size = Vec(24 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT);

        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/HotPot.svg"),
            asset::plugin(pluginInstance, "res/HotPot-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        const float panelWidth = 24.f * 5.08f;

        // -- Pot, top center -------------------------------------------------
        const float potSize = 40.f, potTop = 9.f;
        HotPotDisplay* display = createWidget<HotPotDisplay>(p(0.5f * (panelWidth - potSize), potTop));
        display->box.size = p(potSize, potSize);
        display->module   = module;
        addChild(display);

        // Skim sits at the center of the pot, where the listener is.
        const float potCenterX = 0.5f * panelWidth, potCenterY = potTop + 0.5f * potSize;
        addParam(createParamCentered<TL1105>(p(potCenterX, potCenterY), module, HotPot::RESET_PARAM));
        addChild(createLightCentered<MediumLight<RedLight>>(p(potCenterX, potCenterY), module, HotPot::RESET_LIGHT));

        // -- Flanks: knob, trim, CV columns -----------------------------------
        const float yKnob = 17.f, yFlankTrim = 28.5f, yFlankCV = 37.f;
        const float flankPitch = 12.f, flankEdge = 9.f;
        const float leftX[3]  = { flankEdge, flankEdge + flankPitch, flankEdge + 2.f * flankPitch };
        const float rightX[3] = { panelWidth - leftX[2], panelWidth - leftX[1], panelWidth - leftX[0] };

        const int flankParams[6] = { HotPot::ROOM_PARAM, HotPot::DECAY_PARAM, HotPot::HAZE_PARAM,
                                     HotPot::DAMP_PARAM, HotPot::SIZE_PARAM,  HotPot::CUES_PARAM };
        const int flankTrims[6]  = { HotPot::ROOM_ATT_PARAM, HotPot::DECAY_ATT_PARAM, HotPot::HAZE_ATT_PARAM,
                                     HotPot::DAMP_ATT_PARAM, HotPot::SIZE_ATT_PARAM,  HotPot::CUES_ATT_PARAM };
        const int flankInputs[6] = { HotPot::ROOM_CV_INPUT, HotPot::DECAY_CV_INPUT, HotPot::HAZE_CV_INPUT,
                                     HotPot::DAMP_CV_INPUT, HotPot::SIZE_CV_INPUT,  HotPot::CUES_CV_INPUT };
        for (int i = 0; i < 6; ++i) {
            const float x = (i < 3) ? leftX[i] : rightX[i - 3];
            addParam(createParamCentered<RoundBlackKnob>(p(x, yKnob), module, flankParams[i]));
            addParam(createParamCentered<Trimpot>(p(x, yFlankTrim), module, flankTrims[i]));
            addInput(createInputCentered<ThemedPJ301MPort>(p(x, yFlankCV), module, flankInputs[i]));
        }

        // -- Strips: three columns each, wider gaps between strips -----------
        const float stripPitch = 11.f, stripGap = 12.5f;   // column pitch within and between strips
        const float stripStep  = 2.f * stripPitch + stripGap;
        const float stripLeft  = 0.5f * (panelWidth - (3.f * stripStep));   // three strips plus MAIN
        const float yButton = 53.5f, ySlider = 68.5f, yTrim = 80.f, yCV = 88.f;
        const float yKnobRow = 102.f, yInputs = 115.5f;

        float stripX[NUM_RAILS][3];
        for (int r = 0; r < NUM_RAILS; ++r)
            for (int c = 0; c < 3; ++c) stripX[r][c] = stripLeft + r * stripStep + c * stripPitch;

        addStrip<RingLight> (module, 0, stripX[0], yButton, ySlider, yTrim, yCV, yKnobRow, yInputs);
        addStrip<StageLight>(module, 1, stripX[1], yButton, ySlider, yTrim, yCV, yKnobRow, yInputs);
        addStrip<DepthLight>(module, 2, stripX[2], yButton, ySlider, yTrim, yCV, yKnobRow, yInputs);

        // -- MAIN: mute, volume / drive slider, trim and CV, over the stereo outputs --
        const float mainX = stripX[2][2] + stripGap;
        addParam(createParamCentered<HotPotLatch>(p(mainX, yButton), module, HotPot::MAIN_MUTE_PARAM));
        addChild(createLightCentered<MediumLight<RedLight>>(p(mainX, yButton), module, HotPot::MAIN_MUTE_LIGHT));
        addParam(createParamCentered<HotPotSlider<WhiteLight>>(p(mainX, ySlider), module, HotPot::MAIN_PARAM));
        addParam(createParamCentered<Trimpot>(p(mainX, yTrim), module, HotPot::MAIN_ATT_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(p(mainX, yCV), module, HotPot::MAIN_CV_INPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(p(mainX, yKnobRow + 4.f), module, HotPot::OUT_L_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(p(mainX, yInputs), module, HotPot::OUT_R_OUTPUT));
    }

    void appendContextMenu(Menu* menu) override {
        HotPot* module = dynamic_cast<HotPot*>(this->module);
        if (!module) return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Motion shape"));
        const std::vector<std::string> shapeNames = { "Sine", "Triangle", "Stepped" };
        menu->addChild(createIndexPtrSubmenuItem("Stir",  shapeNames, &module->motionShape[0]));
        menu->addChild(createIndexPtrSubmenuItem("Shabu", shapeNames, &module->motionShape[1]));
        menu->addChild(createIndexPtrSubmenuItem("Fan",   shapeNames, &module->motionShape[2]));

        menu->addChild(new MenuSeparator);
        const std::vector<std::string> haasNames = { "Natural", "Wide", "Wider", "Widest (60 ms)" };
        menu->addChild(createIndexPtrSubmenuItem("Haas far-ear delay", haasNames, &module->haasMode));
        menu->addChild(createBoolPtrMenuItem("Doppler", "", &module->doppler));
        menu->addChild(createBoolPtrMenuItem("Full-bandwidth room (more CPU)", "", &module->roomFullRate));
    }
};

Model* modelHotPot = createModel<HotPot, HotPotWidget>("HotPot");