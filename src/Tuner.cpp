////////////////////////////////////////////////////////////
//
//   Tuner
//
//   written by Cody Geary
//   Copyright 2025, MIT License
//
//   A tuner and zero-crossing dynamic scope
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
#include <cmath>
#include <string>
#include "digital_display.hpp"

using namespace rack;
using float_4 = rack::simd::float_4;

template<typename T, size_t Size>
class CircularBuffer {
private:
    T buffer[Size];
public:
    T& operator[](size_t i) { return buffer[i % Size]; }
    const T& operator[](size_t i) const { return buffer[i % Size]; }
    static constexpr size_t size() { return Size; }
};

#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>


// ---------------------------------------------------------------------------
// DecimatedFrequencyTracker
//
// Decimates the input signal before running normalised autocorrelation.
// Simple averaging decimation is sufficient -- the mild HF rolloff does not
// affect fundamental detection.
//
// AC computation is spread evenly across every full-rate audio sample by a
// budgeted state machine.  Each sample performs at most workBudget units of
// work (1 unit = one float_4 correlation iteration), crossing state
// boundaries as needed, so the per-sample cost is flat instead of bursty.
// The mean/DC-removal passes and the peak/candidate scans are chunked under
// the same budget -- no state does a full-buffer pass in a single sample.
//
// Smoothing only runs on confident results, preventing stale or default
// values from leaking into the display when no signal is present.
// ---------------------------------------------------------------------------
struct DecimatedFrequencyTracker {

    // Buffer length is set per-tracker in setDecimation() based on the
    // minimum frequency to detect:
    //   bufferSize = 2 * effectiveSampleRate / minFrequency
    // Audio tracker (8x/48kHz, min ~16Hz):  bufferSize = 2*6000/16 = 750
    // LFO tracker (128x/48kHz, min ~0.5Hz): bufferSize = 2*375/0.5 = 1500
    // Smaller buffer = shorter inner loop = dramatically less CPU.
    static constexpr int MAX_BUFFER_SIZE = 1500;
    int bufferSize = MAX_BUFFER_SIZE;

    float writeBuffer[MAX_BUFFER_SIZE]   = {};
    float processBuffer[MAX_BUFFER_SIZE] = {};
    int   writeIndex = 0;

    float decimAccum = 0.f;
    int   decimCount = 0;
    int   decimationFactor = 8;

    // Incremental AC engine.
    // States: 0=idle, 1=mean, 2=dc-remove, 3=correlate, 4=peak-scan, 5=candidate+finalize
    std::vector<float> ac;
    std::vector<float> win;
    std::vector<float> dcRemovedSignal;
    int   acState = 0;
    int   acLag   = 1;
    int   acHalf  = 0;
    float acMean  = 0.f;
    bool  bufferReady = false;

    int     chunkPos = 0;            // element/lag position within the current state
    float_4 partNum  = float_4(0.f); // partial correlation sums carried across calls
    float_4 partSq0  = float_4(0.f);
    float_4 partSqL  = float_4(0.f);
    float   acGlobalPeak    = -2.f;
    int     acGlobalPeakLag = 1;
    float   acThreshold     = 0.f;
    int     acCandidateLag  = 0;

    // lagsPerCall scales the per-sample work budget; 8 is the baseline where
    // a full analysis completes in about half the buffer refill time.
    int lagsPerCall = 8;

    // workBudget: units of analysis work permitted per full-rate sample.
    // 1 unit = one float_4 correlation iteration = 4 element operations.
    // baseWorkBudget is derived from buffer geometry in setDecimation().
    int baseWorkBudget = 32;
    int workBudget     = 32;

    // analysisSkip: skip this many buffer fills between analyses.
    // 0 = analyse every fill (highest CPU, fastest lock).
    // N = analyse every (N+1)th fill -- directly scales CPU down by N+1.
    // The inner loop cost is unchanged; we simply run it less often.
    int analysisSkip = 0;
    int analysisSkipCounter = 0;

    float decimatedSampleRate = 6000.f;
    int   simdLen      = 0;   // (acHalf / 4) * 4, cached to avoid recomputing per lag
    float lastFreq     = -1.f;
    float smoothedFreq = -1.f;
    float acPeakValue  = 0.f;
    float confidenceFloor = 0.25f;
    float smoothFactor = 0.9995f;

    DecimatedFrequencyTracker() {
        ac.resize(MAX_BUFFER_SIZE / 2, 0.f);
        win.resize(MAX_BUFFER_SIZE / 2, 0.f);
        dcRemovedSignal.resize(MAX_BUFFER_SIZE, 0.f);
        rebuildWindow();
    }

    void setDecimation(int factor, float fullSampleRate, float minFrequency = 16.f) {
        decimationFactor    = std::max(1, factor);
        decimatedSampleRate = fullSampleRate / decimationFactor;
        // Buffer just large enough to detect minFrequency.
        // 2 * effectiveRate / minFreq gives enough lags for 2 full periods.
        bufferSize = std::min(MAX_BUFFER_SIZE,
                     std::max(64, (int)(2.f * decimatedSampleRate / minFrequency)));
        rebuildWindow();
        // ~50ms smoothing time constant regardless of decimation rate
        smoothFactor = expf(-1.f / (0.05f * decimatedSampleRate));

        // Per-sample analysis budget.  Total correlation work is roughly
        // (bufferSize/2) lags * (simdLen/4) float_4 iterations each.  Spread
        // it so a full analysis completes in about half the buffer refill
        // time (bufferSize * decimationFactor full-rate samples).  The 0.5f
        // completion fraction can be hand-tuned: smaller spreads the work
        // thinner (lower peak CPU, slower lock).  The floor of 8 keeps
        // heavily decimated trackers (LFO range) locking promptly even
        // though their refill time would allow a much smaller budget.
        float totalWork   = (float)(bufferSize / 2) * (simdLen / 4);
        float fillSamples = (float)bufferSize * decimationFactor;
        baseWorkBudget = std::max(8, (int)ceilf(totalWork / (0.5f * fillSamples)));
        workBudget     = std::max(1, baseWorkBudget * lagsPerCall / 8);

        // Abort any in-flight analysis -- the buffer geometry just changed.
        acState     = 0;
        bufferReady = false;
        chunkPos    = 0;
        writeIndex  = 0;
        decimAccum  = 0.f;
        decimCount  = 0;
        analysisSkipCounter = 0;
    }

    void setConfidenceFloor(float floor) { confidenceFloor = floor; }
    void setLagsPerCall(int lags) {
        lagsPerCall = std::max(1, lags);
        workBudget  = std::max(1, baseWorkBudget * lagsPerCall / 8);
    }
    void setAnalysisSkip(int skip)       { analysisSkip = std::max(0, skip); }

    void rebuildWindow() {
        int half = bufferSize / 2;
        simdLen = (half / 4) * 4;
        win.resize(half);
        ac.resize(half, 0.f);
        dcRemovedSignal.resize(bufferSize, 0.f);
        for (int i = 0; i < half; ++i)
            win[i] = 0.5f * (1.f - cosf(2.f * (float)M_PI * i / (half - 1)));
    }

    // Returns smoothed Hz or -1 if no confident lock.
    float process(float in) {
        decimAccum += in;
        if (++decimCount >= decimationFactor) {
            writeBuffer[writeIndex++] = decimAccum / decimationFactor;
            decimAccum = 0.f;
            decimCount = 0;
            if (writeIndex >= bufferSize) {
                writeIndex = 0;
                if (acState == 0) {
                    // Skip N buffer fills between analyses to reduce CPU.
                    // Each skipped fill costs nothing -- we just keep writing
                    // into the write buffer and discard it.
                    if (++analysisSkipCounter > analysisSkip) {
                        analysisSkipCounter = 0;
                        std::memcpy(processBuffer, writeBuffer, bufferSize * sizeof(float));
                        bufferReady = true;
                        acHalf      = bufferSize / 2;
                        acMean      = 0.f;
                        chunkPos    = 0;
                        acState     = 1;
                    }
                }
            }
        }

        // Run a small, fixed slice of analysis work on every full-rate sample.
        // Total work per analysis is unchanged; spreading it over every sample
        // (instead of bursting on decimated ticks) flattens the CPU profile.
        if (bufferReady) processChunk();

        // Only smooth when a new decimated sample was just produced --
        // running this every full-rate sample wastes CPU since the value
        // only changes once per decimationFactor samples.
        if (decimCount == 0) {
            if (lastFreq > 0.f) {
                if (smoothedFreq < 0.f)
                    smoothedFreq = lastFreq;  // cold start: snap rather than ramp
                else
                    smoothedFreq = smoothFactor * smoothedFreq + (1.f - smoothFactor) * lastFreq;
            } else {
                smoothedFreq = -1.f;
            }
        }

        return smoothedFreq;
    }

    // Apply a finished analysis result and return to idle.
    // result: >0 confident Hz, -1 noise floor, -2 weak (hold previous)
    void finishAnalysis(float result) {
        if      (result > 0.f)   lastFreq = result;
        else if (result == -1.f) lastFreq = -1.f;
        // result == -2: weak signal, hold lastFreq unchanged
        bufferReady = false;
        acState     = 0;
        chunkPos    = 0;
    }

    // Incremental analysis engine.  Called once per full-rate sample while an
    // analysis is in flight.  Each call consumes at most workBudget units of
    // work (1 unit = one float_4 iteration = 4 element operations), crossing
    // state boundaries as needed, so the per-sample cost stays flat.
    void processChunk() {
        int budget = workBudget;
        while (budget > 0 && acState != 0) {
            switch (acState) {

                case 1: { // accumulate mean of the captured frame
                    int elems = std::min(bufferSize - chunkPos, budget * 4);
                    for (int i = chunkPos; i < chunkPos + elems; ++i)
                        acMean += processBuffer[i];
                    chunkPos += elems;
                    budget   -= (elems + 3) / 4;
                    if (chunkPos >= bufferSize) {
                        acMean  /= bufferSize;
                        chunkPos = 0;
                        acState  = 2;
                    }
                    break;
                }

                case 2: { // subtract mean (DC removal)
                    int elems = std::min(bufferSize - chunkPos, budget * 4);
                    for (int i = chunkPos; i < chunkPos + elems; ++i)
                        dcRemovedSignal[i] = processBuffer[i] - acMean;
                    chunkPos += elems;
                    budget   -= (elems + 3) / 4;
                    if (chunkPos >= bufferSize) {
                        // No need to zero ac[] -- every ac[acLag] is written before being read
                        acLag    = 1;
                        chunkPos = 0;
                        partNum  = float_4(0.f);
                        partSq0  = float_4(0.f);
                        partSqL  = float_4(0.f);
                        acState  = 3;
                    }
                    break;
                }

                case 3: { // normalised autocorrelation, partial lags carried across calls
                    const float* w   = win.data();
                    const float* sig = dcRemovedSignal.data();
                    int iters = std::min((simdLen - chunkPos) / 4, budget);
                    for (int n = 0; n < iters; ++n) {
                        float_4 wi = float_4::load(w   + chunkPos);
                        float_4 a  = float_4::load(sig + chunkPos)         * wi;
                        float_4 b  = float_4::load(sig + chunkPos + acLag) * wi;
                        partNum += a * b;
                        partSq0 += a * a;
                        partSqL += b * b;
                        chunkPos += 4;
                    }
                    budget -= iters;
                    if (chunkPos >= simdLen) {
                        float num = partNum[0] + partNum[1] + partNum[2] + partNum[3];
                        float sq0 = partSq0[0] + partSq0[1] + partSq0[2] + partSq0[3];
                        float sqL = partSqL[0] + partSqL[1] + partSqL[2] + partSqL[3];
                        // Scalar tail for remaining samples of this lag
                        for (int i = simdLen; i < acHalf; ++i) {
                            float a = sig[i]         * w[i];
                            float b = sig[i + acLag] * w[i];
                            num += a * b;
                            sq0 += a * a;
                            sqL += b * b;
                        }
                        ac[acLag] = std::min(1.f, std::max(-1.f,
                                        num / (sqrtf(sq0 * sqL) + 1e-12f)));
                        acLag++;
                        chunkPos = 0;
                        partNum  = float_4(0.f);
                        partSq0  = float_4(0.f);
                        partSqL  = float_4(0.f);
                        budget--;  // tail + normalisation counted as one unit
                        if (acLag >= acHalf) {
                            acGlobalPeak    = -2.f;
                            acGlobalPeakLag = 1;
                            chunkPos        = 1;
                            acState         = 4;
                        }
                    }
                    break;
                }

                case 4: { // global peak scan over ac[1 .. acHalf-1]
                    int last = std::min(acHalf, chunkPos + budget * 4);
                    for (int lag = chunkPos; lag < last; ++lag) {
                        if (ac[lag] > acGlobalPeak) {
                            acGlobalPeak    = ac[lag];
                            acGlobalPeakLag = lag;
                        }
                    }
                    budget  -= (last - chunkPos + 3) / 4;
                    chunkPos = last;
                    if (chunkPos >= acHalf) {
                        acPeakValue = acGlobalPeak;
                        if (acGlobalPeak < confidenceFloor) {
                            finishAnalysis(-1.f);
                            break;
                        }
                        if (acGlobalPeak < confidenceFloor + 0.10f) {
                            finishAnalysis(-2.f);
                            break;
                        }
                        acThreshold = std::min(0.88f, std::max(confidenceFloor + 0.10f,
                                          0.55f + 0.1f * (acGlobalPeak - 0.8f)));
                        acCandidateLag = 0;
                        chunkPos       = 2;
                        acState        = 5;
                    }
                    break;
                }

                case 5: { // first-peak candidate scan, then finalize
                    int last = std::min(acHalf - 1, chunkPos + budget * 4);
                    int lag  = chunkPos;
                    for (; lag < last; ++lag) {
                        if (ac[lag] > ac[lag-1] && ac[lag] >= ac[lag+1]
                            && ac[lag] >= acThreshold) {
                            acCandidateLag = lag;
                            break;
                        }
                    }
                    budget  -= (lag - chunkPos) / 4 + 1;
                    chunkPos = lag;
                    if (acCandidateLag == 0 && chunkPos < acHalf - 1) break;

                    // Finalize: constant-time selection and refinement.
                    int bestLag = (acCandidateLag >= 2) ? acCandidateLag
                                                        : std::max(2, acGlobalPeakLag);

                    // Sub-harmonic correction: harmonic-rich waveforms (square, pulse, saw) produce
                    // strong AC peaks at sub-multiples of the true period (lag/2, lag/3, lag/4).
                    // The first-peak search can land on one of these instead of the fundamental.
                    // If a shorter lag is itself a local peak and nearly as strong, it wins.
                    for (int divisor = 2; divisor <= 4; ++divisor) {
                        int subLag = bestLag / divisor;
                        if (subLag < 2 || subLag >= acHalf - 1) continue;
                        bool isLocalMax   = ac[subLag] > ac[subLag - 1] && ac[subLag] >= ac[subLag + 1];
                        bool isComparable = ac[subLag] >= ac[bestLag] * 0.85f;
                        if (isLocalMax && isComparable) bestLag = subLag;
                    }

                    // Parabolic sub-sample interpolation
                    float freq = -1.f;
                    if (bestLag > 1 && bestLag < acHalf - 1) {
                        float y0    = ac[bestLag - 1];
                        float y1    = ac[bestLag];
                        float y2    = ac[bestLag + 1];
                        float denom = y0 - 2.f * y1 + y2;
                        float shift = (fabsf(denom) > 1e-12f) ? (0.5f * (y0 - y2) / denom) : 0.f;
                        float refinedLag = (float)bestLag + shift;
                        if (refinedLag > 1.f)
                            freq = decimatedSampleRate / refinedLag;
                    }
                    if (!(std::isfinite(freq) && freq > 0.f)) {
                        freq = decimatedSampleRate / (float)bestLag;
                        if (!(std::isfinite(freq) && freq > 0.f)) freq = -1.f;
                    }
                    finishAnalysis(freq);
                    break;
                }
            }
        }
    }
};


// ---------------------------------------------------------------------------
// DualRangeTracker
//
// Two DecimatedFrequencyTrackers in parallel:
//   audioTracker : 4x decimation  -> ~12kHz effective at 48kHz -> ~16Hz to ~6kHz
//   lfoTracker   : 128x decimation -> ~375Hz effective at 48kHz -> ~0.5Hz to ~187Hz
//
// Both run every sample.  The result with higher AC confidence wins, with a
// bias toward the audio tracker to avoid spurious LFO detections on normal
// pitched signals.  The LFO tracker uses a relaxed confidence floor since
// slow periodic signals produce lower AC peaks over a fixed-length window.
//
// audioNyquist is the Nyquist of the audio tracker.  Results within 1Hz of
// this ceiling are spurious alias locks from slow/LFO signals and are
// suppressed -- the caller should treat them as no-lock.
// ---------------------------------------------------------------------------
struct DualRangeTracker {
    DecimatedFrequencyTracker audioTracker;
    DecimatedFrequencyTracker lfoTracker;

    // Nyquist ceiling of the audio tracker, computed in setSampleRate.
    // Results this close to the ceiling are considered spurious alias locks.
    float audioNyquist = 6000.f;

    void setSampleRate(float sr) {
        // Audio tracker: 4x decimation -> 12kHz effective -> ceiling ~6kHz
        // buffer = 2*12000/16 = 1500 samples
        // SIMD float_4 on inner loop keeps CPU cost manageable
        audioTracker.setDecimation(4, sr, 16.f);
        audioTracker.setConfidenceFloor(0.25f);

        // LFO tracker: min 0.5Hz, buffer = 2*375/0.5 = 1500 samples (full)
        lfoTracker.setDecimation(128, sr, 0.5f);
        lfoTracker.setConfidenceFloor(0.15f);

        // Nyquist = effectiveRate / 2 = (sr / decimation) / 2
        audioNyquist = audioTracker.decimatedSampleRate / 2.f;
    }

    void setLagsPerCall(int lags) {
        audioTracker.setLagsPerCall(lags);
        lfoTracker.setLagsPerCall(lags);
    }

    void setAnalysisSkip(int skip) {
        audioTracker.setAnalysisSkip(skip);
        lfoTracker.setAnalysisSkip(skip);
    }

    float process(float in) {
        float audioFreq = audioTracker.process(in);
        float lfoFreq   = lfoTracker.process(in);

        bool audioValid = (audioFreq > 0.f && audioTracker.acPeakValue > 0.35f);
        bool lfoValid   = (lfoFreq   > 0.f && lfoTracker.acPeakValue   > 0.25f);

        if (audioValid && lfoValid)
            return (audioTracker.acPeakValue >= lfoTracker.acPeakValue - 0.15f)
                   ? audioFreq : lfoFreq;
        if (audioValid) return audioFreq;
        if (lfoValid)   return lfoFreq;
        return -1.f;
    }
};


// ---------------------------------------------------------------------------
// PhaseAccumulatorTrigger
//
// Generates display trigger pulses from the known detected frequency rather
// than from raw zero-crossings.  This eliminates node-flickering on waveforms
// with multiple zero-crossings per cycle (square, saw, complex harmonics).
//
// The accumulator advances at currentHz / (sampleRate * numCycles) per sample
// and wraps once per full display window.  On wrap it arms itself and waits
// for the next rising zero-crossing to resync phase, keeping the display
// aligned to the waveform shape.
//
// The resync window is scaled by 1/numCycles so it represents a fixed fraction
// of one single period regardless of how many cycles are displayed.
//
// A safety fallback fires if no zero-crossing appears within half the window,
// handling DC-offset signals or silence gracefully.
// ---------------------------------------------------------------------------
struct PhaseAccumulatorTrigger {
    float phase          = 0.f;
    float phaseIncrement = 0.f;
    bool  armed          = false;
    bool  triggered      = false;

    // Fraction of one single period accepted as resync tolerance on each side.
    static constexpr float RESYNC_WINDOW = 0.15f;

    void update(float currentHz, float sampleRate, float numCycles,
                float signalIn, float prevSignalIn) {
        triggered = false;

        if (currentHz > 0.f)
            phaseIncrement = currentHz / (sampleRate * numCycles);

        phase += phaseIncrement;

        if (phase >= 1.f) {
            phase -= 1.f;
            armed  = true;
        }

        bool risingCrossing = (signalIn >= 0.f && prevSignalIn < 0.f);
        float resyncWindow  = RESYNC_WINDOW / numCycles;

        if (risingCrossing) {
            if (armed) {
                phase     = 0.f;
                armed     = false;
                triggered = true;
            } else if (phase > 1.f - resyncWindow) {
                phase     = 0.f;
                triggered = true;
            }
        }

        // Safety: fire if no zero-crossing appears within half the window
        if (armed && phase > 0.5f) {
            armed     = false;
            triggered = true;
        }
    }
};


struct Tuner : Module {

    enum ParamId {
        OFFSET_PARAM, OFFSET2_PARAM,
        WIDTH_PARAM,  WIDTH2_PARAM,
        GAIN_PARAM,   GAIN2_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        AUDIO_INPUT, AUDIO2_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        FREQ_OUTPUT, FREQ2_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        LIGHTS_LEN
    };

    DualRangeTracker        freqTracker[2];
    PhaseAccumulatorTrigger displayTrigger[2];
    CircularBuffer<float, 1024> waveBuffer[2];

    float sampleRate = 48000.f;

    float currentHz[2]   = {-1.f, -1.f};
    float currentVOct[2] = {0.f, 0.f};
    std::string currentNote[2]    = {"---", "---"};
    std::string centsDeviation[2] = {"---", "---"};

    int   paramReadCounter[2] = {0, 0};
    int   prevSampleIndex[2]  = {0, 0};

    float prevIn[2]          = {0.f, 0.f};
    bool  capturing[2]       = {false, false};
    float captureProgress[2] = {0.f, 0.f};

    float displayGain[2]      = {1.f, 1.f};
    float displayOffset[2]    = {0.f, 0.f};
    float displayNumCycles[2] = {2.f, 2.f};
    float displayIncDenom[2]  = {96000.f, 96000.f};  // numCycles * sampleRate
    bool  inputConnected[2]   = {false, false};

    int  lagsPerCall   = 8;
    int  analysisSkip  = 1;
    bool displayMode   = false;

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
        json_object_set_new(rootJ, "displayMode",   json_boolean(displayMode));
        json_object_set_new(rootJ, "lagsPerCall",   json_integer(lagsPerCall));
        json_object_set_new(rootJ, "analysisSkip",  json_integer(analysisSkip));
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        json_t* j;
        j = json_object_get(rootJ, "displayMode");
        if (j) displayMode = json_boolean_value(j);
        j = json_object_get(rootJ, "lagsPerCall");
        if (j) lagsPerCall = clamp((int)json_integer_value(j), 1, 32);
        j = json_object_get(rootJ, "analysisSkip");
        if (j) analysisSkip = clamp((int)json_integer_value(j), 0, 32);
        applyLagsPerCall();
        applyAnalysisSkip();
    }

    void applyLagsPerCall() {
        for (int ch = 0; ch < 2; ch++)
            freqTracker[ch].setLagsPerCall(lagsPerCall);
    }

    void applyAnalysisSkip() {
        for (int ch = 0; ch < 2; ch++)
            freqTracker[ch].setAnalysisSkip(analysisSkip);
    }

    Tuner() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configInput(AUDIO_INPUT,  "Audio 1");
        configOutput(FREQ_OUTPUT, "Frequency (V/oct) 1");
        configParam(GAIN_PARAM,   0.f,  5.f, 1.f, "Wave Gain 1");
        configParam(OFFSET_PARAM, -5.f, 5.f, 0.f, "Wave Offset 1");
        configParam(WIDTH_PARAM,  1.f,  6.f, 1.f, "Width in Wavelengths 1")->snapEnabled = true;
        paramQuantities[WIDTH_PARAM]->displayMultiplier = 2.0f;

        configInput(AUDIO2_INPUT,  "Audio 2");
        configOutput(FREQ2_OUTPUT, "Frequency (V/oct) 2");
        configParam(GAIN2_PARAM,   0.f,  5.f, 1.f, "Wave Gain 2");
        configParam(OFFSET2_PARAM, -5.f, 5.f, 0.f, "Wave Offset 2");
        configParam(WIDTH2_PARAM,  1.f,  6.f, 1.f, "Width in Wavelengths 2")->snapEnabled = true;
        paramQuantities[WIDTH2_PARAM]->displayMultiplier = 2.0f;

        applyLagsPerCall();
        applyAnalysisSkip();
    }

    void onSampleRateChange() override {
        sampleRate = APP->engine->getSampleRate();
        for (int ch = 0; ch < 2; ch++)
            freqTracker[ch].setSampleRate(sampleRate);
        // Re-apply user settings after setSampleRate resets the trackers
        applyLagsPerCall();
        applyAnalysisSkip();
    }

    void process(const ProcessArgs& args) override {

        for (int ch = 0; ch < 2; ch++) {

            // Read knob and input state every ~2ms to save CPU
            if (++paramReadCounter[ch] >= 100) {
                paramReadCounter[ch]   = 0;
                displayOffset[ch]      = params[OFFSET_PARAM + ch].getValue()-0.05f;
                displayGain[ch]        = params[GAIN_PARAM   + ch].getValue();
                displayNumCycles[ch]   = params[WIDTH_PARAM  + ch].getValue() * 2.f;
                displayIncDenom[ch]    = displayNumCycles[ch] * sampleRate;
                inputConnected[ch]     = inputs[AUDIO_INPUT  + ch].isConnected();
            }

            float in = 0.f;
            if (inputConnected[ch])
                in = clamp(inputs[AUDIO_INPUT + ch].getVoltage(0) * displayGain[ch]
                           + displayOffset[ch], -10.f, 10.f);

            currentHz[ch] = freqTracker[ch].process(in);

            // V/oct output
            if (paramReadCounter[ch] == 0) {
                if (currentHz[ch] > 0.f) {
                    // Standard VCV V/oct: 0V = C4 (261.6255653 Hz)
                    currentVOct[ch] = log2f(currentHz[ch] / dsp::FREQ_C4);
                    outputs[FREQ_OUTPUT + ch].setVoltage(currentVOct[ch]);
                } else {
                    outputs[FREQ_OUTPUT + ch].setVoltage(0.f);
                    currentVOct[ch] = -999.f;
                }
            }

            // --- Waveform display ---
            float scaledSample = in * 0.5f;

            // Phase-accumulator trigger: fires exactly once per numCycles periods,
            // resynced to the nearest rising zero-crossing for waveform alignment.
            displayTrigger[ch].update(currentHz[ch], sampleRate, displayNumCycles[ch], in, prevIn[ch]);
            prevIn[ch] = in;

            if (displayTrigger[ch].triggered) {
                capturing[ch]       = true;
                captureProgress[ch] = 0.f;
                prevSampleIndex[ch] = 0;
            }

            if (capturing[ch]) {
                // Increment in buffer slots per audio sample.
                // numCycles periods map onto 1024 slots.
                float inc = (currentHz[ch] > 0.f)
                            ? 1024.f * currentHz[ch] / displayIncDenom[ch]
                            : 1.f;

                // Saturate one slot past the end so slot 1023 is always reached
                // before the next trigger resets the frame.
                captureProgress[ch] = std::min(captureProgress[ch] + inc, 1024.f);

                int sampleIndex = clamp((int)captureProgress[ch], 0, 1023);
                waveBuffer[ch][sampleIndex] = scaledSample;

                // Interpolate gaps between sparse writes (high-frequency signals
                // may advance several buffer slots per audio sample)
                if (sampleIndex != prevSampleIndex[ch]) {
                    int   fromIdx = prevSampleIndex[ch];
                    int   toIdx   = sampleIndex;
                    float fromVal = waveBuffer[ch][fromIdx];

                    if (fromIdx < toIdx) {
                        int gap = toIdx - fromIdx;
                        for (int i = 1; i < gap; ++i) {
                            float t = (float)i / gap;
                            waveBuffer[ch][fromIdx + i] = fromVal + t * (scaledSample - fromVal);
                        }
                    } else {
                        // Wrap-around interpolation
                        int gapToEnd = 1023 - fromIdx;
                        for (int i = 1; i <= gapToEnd; ++i) {
                            float t = (float)i / (gapToEnd + 1);
                            waveBuffer[ch][fromIdx + i] = fromVal + t * (scaledSample - fromVal);
                        }
                        for (int i = 0; i <= toIdx; ++i) {
                            float t = (float)(i + 1) / (toIdx + 1);
                            waveBuffer[ch][i] = fromVal + t * (scaledSample - fromVal);
                        }
                    }
                    prevSampleIndex[ch] = sampleIndex;
                }
            }
        }
    }
};


struct TunerWidget : ModuleWidget {

    struct WaveDisplay : TransparentWidget {
        Tuner*   module       = nullptr;
        unsigned channelIndex = 0;

        void drawLayer(const DrawArgs& args, int layer) override {
            if (!module || layer != 1) return;

            if (module->displayMode) {
                nvgBeginPath(args.vg);
                nvgRect(args.vg, 0.f, box.size.y / 2.f - 20.f, box.size.x, 40.f);
                nvgFillColor(args.vg, nvgRGB(0x21, 0x21, 0x21));
                nvgFill(args.vg);
                nvgClosePath(args.vg);
                return;
            }

            float centerY = box.size.y / 2.f;
            float scale   = centerY / 5.f;

            nvgBeginPath(args.vg);
            // Skip first and last 4 slots -- these boundary slots are most
            // likely to contain stale data from the previous capture frame.
            for (size_t i = 4; i < 1020; i++) {
                float x = (float)i / 1023.f * box.size.x;
                float y = centerY - module->waveBuffer[channelIndex][i] * scale;
                if (module->currentHz[channelIndex] < 0.f) y = centerY;
                // Clamp to display bounds so stale or wild values never draw off-screen
                y = clamp(y, 0.f, box.size.y);
                if (i == 4) nvgMoveTo(args.vg, x, y);
                else        nvgLineTo(args.vg, x, y);
            }
            nvgStrokeColor(args.vg, nvgRGBAf(0.f, 0.7f, 1.f, 0.9f));
            nvgStrokeWidth(args.vg, 1.2f);
            nvgStroke(args.vg);
        }
    };

    DigitalDisplay* noteDisp  = nullptr;
    DigitalDisplay* centsDisp = nullptr;
    DigitalDisplay* freqDisp  = nullptr;
    WaveDisplay*    waveDisp  = nullptr;

    DigitalDisplay* noteDisp2  = nullptr;
    DigitalDisplay* centsDisp2 = nullptr;
    DigitalDisplay* freqDisp2  = nullptr;
    WaveDisplay*    waveDisp2  = nullptr;

    bool lastDisplayMode = false;

    TunerWidget(Tuner* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Tuner.svg"),
            asset::plugin(pluginInstance, "res/Tuner-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // --- Channel 1 ---
        waveDisp = new WaveDisplay();
        waveDisp->module       = module;
        waveDisp->channelIndex = 0;
        waveDisp->box.pos  = mm2px(Vec(8, 13));
        waveDisp->box.size = mm2px(Vec(29.939 * 2, 32.608));
        addChild(waveDisp);

        noteDisp  = createDigitalDisplay(Vec(box.size.x / 2 - 25 - 20, 40),  "C4");
        centsDisp = createDigitalDisplay(Vec(box.size.x / 2 - 25 + 20, 40),  "0.0%");
        freqDisp  = createDigitalDisplay(Vec(box.size.x / 2 - 25,      120), "261.6 Hz");
        addChild(noteDisp);
        addChild(centsDisp);
        addChild(freqDisp);

        addInput(createInputCentered<PJ301MPort>    (Vec((box.size.x / 6) * 1.f, 170), module, Tuner::AUDIO_INPUT));
        addParam(createParamCentered<RoundBlackKnob>(Vec((box.size.x / 6) * 2.f, 160), module, Tuner::OFFSET_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(Vec((box.size.x / 6) * 3.f, 160), module, Tuner::GAIN_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(Vec((box.size.x / 6) * 4.f, 160), module, Tuner::WIDTH_PARAM));
        addOutput(createOutputCentered<PJ301MPort>  (Vec((box.size.x / 6) * 5.f, 170), module, Tuner::FREQ_OUTPUT));

        // --- Channel 2 ---
        const float channelOffset = 165.f;

        waveDisp2 = new WaveDisplay();
        waveDisp2->module       = module;
        waveDisp2->channelIndex = 1;
        waveDisp2->box.pos  = mm2px(Vec(8, 13 + 25.4f / 75.f * channelOffset));
        waveDisp2->box.size = mm2px(Vec(29.939 * 2, 32.608));
        addChild(waveDisp2);

        noteDisp2  = createDigitalDisplay(Vec(box.size.x / 2 - 25 - 20, 40  + channelOffset), "C4");
        centsDisp2 = createDigitalDisplay(Vec(box.size.x / 2 - 25 + 20, 40  + channelOffset), "0.0%");
        freqDisp2  = createDigitalDisplay(Vec(box.size.x / 2 - 25,      120 + channelOffset), "261.6 Hz");
        addChild(noteDisp2);
        addChild(centsDisp2);
        addChild(freqDisp2);

        addInput(createInputCentered<PJ301MPort>    (Vec((box.size.x / 6) * 1.f, 170 + channelOffset), module, Tuner::AUDIO2_INPUT));
        addParam(createParamCentered<RoundBlackKnob>(Vec((box.size.x / 6) * 2.f, 160 + channelOffset), module, Tuner::OFFSET2_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(Vec((box.size.x / 6) * 3.f, 160 + channelOffset), module, Tuner::GAIN2_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(Vec((box.size.x / 6) * 4.f, 160 + channelOffset), module, Tuner::WIDTH2_PARAM));
        addOutput(createOutputCentered<PJ301MPort>  (Vec((box.size.x / 6) * 5.f, 170 + channelOffset), module, Tuner::FREQ2_OUTPUT));
    }

    DigitalDisplay* createDigitalDisplay(Vec position, std::string initialValue) {
        DigitalDisplay* display = new DigitalDisplay();
        display->box.pos  = position;
        display->box.size = Vec(50, 18);
        display->text     = initialValue;
        display->fgColor  = nvgRGB(208, 140, 89);
        display->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        display->setFontSize(18.f);
        return display;
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);

        Tuner* tunerModule = dynamic_cast<Tuner*>(this->module);
        if (!tunerModule) return;

        menu->addChild(new MenuSeparator());
        menu->addChild(createMenuLabel("Analysis speed"));

        struct SpeedItem : MenuItem {
            Tuner* module;
            int    lags;
            int    skip;
            void onAction(const event::Action& e) override {
                module->lagsPerCall  = lags;
                module->analysisSkip = skip;
                module->applyLagsPerCall();
                module->applyAnalysisSkip();
            }
            void step() override {
                rightText = (module->lagsPerCall  == lags
                             && module->analysisSkip == skip) ? "✔" : "";
                MenuItem::step();
            }
        };

        struct SpeedOption { const char* label; int lags; int skip; };
        const std::vector<SpeedOption> speedOptions = {
            {"Precise  (fast lock, highest CPU)",   8,  0},
            {"Balanced (default)",                  8,  1},
            {"Light    (slower lock, less CPU)",    8,  3},
            {"Minimal  (slowest, least CPU)",       8,  7},
        };
        for (auto& opt : speedOptions) {
            auto* item   = new SpeedItem();
            item->text   = opt.label;
            item->lags   = opt.lags;
            item->skip   = opt.skip;
            item->module = tunerModule;
            menu->addChild(item);
        }

        menu->addChild(new MenuSeparator());

        struct DisplayModeItem : MenuItem {
            Tuner* tunerModule;
            void onAction(const event::Action& e) override {
                tunerModule->displayMode = !tunerModule->displayMode;
            }
            void step() override {
                rightText = tunerModule->displayMode ? "✔" : "";
                MenuItem::step();
            }
        };
        auto* displayModeItem        = new DisplayModeItem();
        displayModeItem->text        = "Large Hz display (disable waveform)";
        displayModeItem->tunerModule = tunerModule;
        menu->addChild(displayModeItem);
    }

    void updateNoteDisplay(Tuner* module, int ch,
                           DigitalDisplay* noteDisp_,
                           DigitalDisplay* centsDisp_,
                           DigitalDisplay* freqDisp_)
    {
        static const char* noteNames[12] = {
            "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
        };

        // Suppress display if locked within 2Hz of the audio tracker's Nyquist --
        // that indicates a spurious alias lock from a slow/LFO signal, not a real pitch.
        float nyquist = module->freqTracker[ch].audioNyquist;
        bool  validHz = module->currentHz[ch] > 0.0001f
                        && module->currentHz[ch] < nyquist - 2.f;

        if (validHz) {
            float midi    = 69.f + 12.f * log2f(module->currentHz[ch] / 440.f);
            int   noteNum = std::lround(midi);
            int   noteIdx = (noteNum + 1200) % 12;
            int   octave  = noteNum / 12 - 1;
            module->currentNote[ch]    = string::f("%s%d", noteNames[noteIdx], octave);
            module->centsDeviation[ch] = string::f("%+0.1f", (midi - noteNum) * 100.f);
        } else {
            module->currentNote[ch]    = "(o)";
            module->centsDeviation[ch] = "(o)";
        }

        noteDisp_->text  = module->currentNote[ch];
        centsDisp_->text = module->centsDeviation[ch];

        if (module->displayMode)
            freqDisp_->text = validHz ? string::f("%.2f Hz", module->currentHz[ch]) : "-=-";
        else
            freqDisp_->text = validHz ? string::f("%.1f Hz", module->currentHz[ch]) : "-=-";
    }

    void step() override {
        Tuner* module = dynamic_cast<Tuner*>(this->module);
        if (!module) return;

        updateNoteDisplay(module, 0, noteDisp,  centsDisp,  freqDisp);
        updateNoteDisplay(module, 1, noteDisp2, centsDisp2, freqDisp2);

        const float baseFreqY1 = 120.f;
        const float baseFreqY2 = 120.f + 165.f;

        // Only update font/position/color when displayMode changes
        if (module->displayMode != lastDisplayMode) {
            lastDisplayMode = module->displayMode;
            if (module->displayMode) {
                freqDisp->setFontSize(36.f);
                freqDisp->box.pos.y = baseFreqY1 - 40.f;
                freqDisp->fgColor   = nvgRGBAf(0.f, 0.7f, 1.f, 0.9f);

                freqDisp2->setFontSize(36.f);
                freqDisp2->box.pos.y = baseFreqY2 - 40.f;
                freqDisp2->fgColor   = nvgRGBAf(0.f, 0.7f, 1.f, 0.9f);
            } else {
                freqDisp->setFontSize(18.f);
                freqDisp->box.pos.y = baseFreqY1;
                freqDisp->fgColor   = nvgRGB(208, 140, 89);

                freqDisp2->setFontSize(18.f);
                freqDisp2->box.pos.y = baseFreqY2;
                freqDisp2->fgColor   = nvgRGB(208, 140, 89);
            }
        }

        ModuleWidget::step();
    }
};

Model* modelTuner = createModel<Tuner, TunerWidget>("Tuner");