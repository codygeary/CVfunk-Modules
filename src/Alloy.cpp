////////////////////////////////////////////////////////////
//
//   Alloy
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   Metal sound synthesizer using Karplus Strong synthesis
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
using namespace rack;

#include <vector>
#include <cmath>
#include <algorithm>

//////////////////////////
// Utility
//////////////////////////
static inline float randf() {
    return 2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f;
}

static float lagrangeInterpolate(float y0, float y1, float y2, float y3, float frac) {
    float x = frac;
    float x_1 = x - 1.0f;
    float x_2 = x - 2.0f;
    float x_3 = x - 3.0f;

    float L0 = (x_1 * x_2 * x_3) * -0.166667f; // -1/6
    float L1 = (x * x_2 * x_3) * 0.5f;
    float L2 = (x * x_1 * x_3) * -0.5f;
    float L3 = (x * x_1 * x_2) * 0.166667f;    // 1/6

    return L0 * y0 + L1 * y1 + L2 * y2 + L3 * y3;
}

struct SecondOrderHPF {
    float x1 = 0, x2 = 0; // previous two inputs
    float y1 = 0, y2 = 0; // previous two outputs
    float a0, a1, a2;     // filter coefficients for the input
    float b1, b2;         // filter coefficients for the output

    SecondOrderHPF() {}

    // Initialize the filter coefficients
    void setCutoffFrequency(float sampleRate, float cutoffFreq) {
        float w0 = 2 * M_PI * cutoffFreq / sampleRate;
        float cosw0 = cos(w0);
        float sinw0 = sin(w0);
        float alpha = sinw0 / 2 * sqrt(2);  // sqrt(2) results in a Butterworth filter

        float a = 1 + alpha;
        a = fmax(a, 0.00001f); //prevent div by zero
        a0 = (1 + cosw0) / 2 / a;
        a1 = -(1 + cosw0) / a;
        a2 = (1 + cosw0) / 2 / a;
        b1 = -2 * cosw0 / a;
        b2 = (1 - alpha) / a;
    }

    // Process the input sample
    float process(float input) {
        float output = a0 * input + a1 * x1 + a2 * x2 - b1 * y1 - b2 * y2;
        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = output;
        return output;
    }
};

//////////////////////////
// Alloy Node
//////////////////////////
struct AlloyNode {
    std::vector<float> buf;
    int writeIndex = 0;
    float delaySec = 0.001f;
    float resonance = 0.9f;
    float damping = 0.01f;
    float lastOut = 0.f;
    float lastInput = 0.f; // ADAA state
    float sampleRate = 48000.f;
    float minDelay = 0.0002f;
    float maxDelay = 0.02f;

    void init(float sr, float maxDelaySec = 0.02f) {
        sampleRate = sr;
        int bufSize = std::max(64, (int)ceilf(maxDelaySec * sr + 4));
        buf.assign(bufSize, 0.f);
        writeIndex = 0;
        lastInput = 0.f;
        lastOut = 0.f;
        maxDelay = (float)((buf.size() - 4) / sampleRate);
    }

    void setDelay(float ds) {
        // Only clamp to buffer-safe range, not minimum
        delaySec = clamp(ds, minDelay, maxDelay);
    }

    float processSample(float input) {
        int bufSize = (int)buf.size();
        if (bufSize < 8) return 0.f;

        // Delay & read position
        float delaySamples = delaySec * sampleRate;
        delaySamples = clamp(delaySamples, 1.f, bufSize - 4.f);

        float readPos = (float)writeIndex - delaySamples;
        while (readPos < 0.f) readPos += bufSize;
        while (readPos >= bufSize) readPos -= bufSize;

        int i0 = ((int)floorf(readPos) - 1 + bufSize) % bufSize;
        float frac = readPos - floorf(readPos);
        float y0 = buf[(i0 + 0) % bufSize];
        float y1 = buf[(i0 + 1) % bufSize];
        float y2 = buf[(i0 + 2) % bufSize];
        float y3 = buf[(i0 + 3) % bufSize];

        float out = lagrangeInterpolate(y0, y1, y2, y3, frac);

        // Overdrive
        float w = input + resonance * out;

        // ADAA saturation (inlined)
        float delta = w - lastInput;
        float sat;
        if (fabs(delta) > 1e-6f) {
            // Antiderivative calculation
            float w2 = w * w;
            float w4 = w2 * w2;
            float w6 = w4 * w2;
            float w8 = w4 * w4;
            float ad_w = w2 * 0.5f - w4 * 0.0833333f + w6 * 0.0222222f - w8 * 0.00674603f;

            float l2 = lastInput * lastInput;
            float l4 = l2 * l2;
            float l6 = l4 * l2;
            float l8 = l4 * l4;
            float ad_l = l2 * 0.5f - l4 * 0.0833333f + l6 * 0.0222222f - l8 * 0.00674603f;

            sat = (ad_w - ad_l) / delta;
        } else {
            // polyTanh
            float w2 = w * w;
            float w3 = w2 * w;
            float w5 = w3 * w2;
            float w7 = w5 * w2;
            sat = w - w3 * 0.333333f + w5 * 0.133333f - w7 * 0.0539683f;
        }

        // Single finite check at the end
        if (!std::isfinite(sat)) sat = 0.f;
        if (!std::isfinite(out)) out = 0.f;

        buf[writeIndex] = sat;
        writeIndex = (writeIndex + 1) % bufSize;
        lastOut = out;
        lastInput = w;

        return out;
    }
};

//////////////////////////
// Module
//////////////////////////
struct Alloy : Module {
    enum ParamIds {
        TENSION_PARAM, TENSION_ATT,
        RESONANCE_PARAM, RESONANCE_ATT,
        NOISE_PARAM, NOISE_ATT,
        SHAPE_PARAM, SHAPE_ATT,
        IMPULSE_PARAM, IMPULSE_ATT,
        OVERDRIVE_PARAM, OVERDRIVE_ATT,
        PITCH_PARAM,
        STRIKE_BUTTON,
        NUM_PARAMS
    };
    enum InputIds {
        AUDIO_INPUT,
        TENSION_IN,
        RESONANCE_IN,
        NOISE_IN,
        SHAPE_IN,
        IMPULSE_IN,
        OVERDRIVE_IN,
        PITCH_IN,
        TRIG_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        AUDIO_OUTPUT_L,
        AUDIO_OUTPUT_R,
        NUM_OUTPUTS
    };
    enum LightIds {
        IMPULSE_LIGHT,
        NUM_LIGHTS
    };

    // --- Voice system ---
    static const int MAX_POLY = 16;
    static const int MAX_NODES = 16;
    int nodeCount = 12; // default

    AlloyNode nodes[MAX_POLY][MAX_NODES];
    float nodeDetune[MAX_POLY][MAX_NODES];

    float sampleRate = 48000.f;

    // Per-voice states
    bool strikeState[MAX_POLY] = {};
    float exciteEnv[MAX_POLY] = {};
    float exciteTime[MAX_POLY] = {};
    float lastOutputL[MAX_POLY] = {};
    float lastOutputR[MAX_POLY] = {};

    float lastInputL[MAX_POLY] = {};
    float lastInputR[MAX_POLY] = {};

    dsp::SchmittTrigger trigInputTrigger[MAX_POLY];
    dsp::SchmittTrigger strikeButtonTrigger[MAX_POLY];

    // --Efficiency--
    int skipCounter = 0;
    int processSkips = 200;
    float tension[16] = {0.f};
    float resonance[16] = {0.5f};
    float shape[16] = {0.f};
    float noise[16] = {0.f};
    float impulse[16] = {0.5f};
    float overdrive[16] = {2.0f};

    // DC protection on input
    SecondOrderHPF hpf[16];

    bool delayMode = false;

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
        json_object_set_new(rootJ, "nodeCount", json_integer(nodeCount));

        json_object_set_new(rootJ, "delayMode", json_boolean(delayMode));
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        json_t* delayModeJ = json_object_get(rootJ, "delayMode");
        if (delayModeJ) {
            delayMode = json_boolean_value(delayModeJ);
        }

        json_t* nodeCountJ = json_object_get(rootJ, "nodeCount");
        if (nodeCountJ) {
            nodeCount = json_integer_value(nodeCountJ);
        }
    }

    Alloy() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        configParam(STRIKE_BUTTON, 0.f, 1.f, 0.f, "Strike");

        configParam(TENSION_PARAM, 0.f, 1.f, 0.5f, "Tension");
        configParam(RESONANCE_PARAM, 0.f, 0.9999f, 0.88f, "Resonance");
        configParam(NOISE_PARAM, 0.f, 1.0f, 0.012f, "Noise");
        configParam(PITCH_PARAM, -4.f, 4.f, 0.f, "Pitch (V/oct)"); // 0V = C4
        configParam(SHAPE_PARAM, -1.f, 1.f, 0.0f, "Shape");
        configParam(IMPULSE_PARAM, 0.f, 1.f, 0.25f, "Impulse Length");
        configParam(OVERDRIVE_PARAM, 0.f, 1.f, 0.0f, "Overdrive Distortion");

        configParam(TENSION_ATT, -1.f, 1.f, 0.0f, "Tension");
        configParam(RESONANCE_ATT, -1.f, 1.f, 0.0f, "Resonance");
        configParam(NOISE_ATT,-1.f, 1.f, 0.0f, "Noise");
        configParam(SHAPE_ATT, -1.f, 1.f, 0.0f, "Shape");
        configParam(IMPULSE_ATT, -1.f, 1.f, 0.0f, "Impulse Length");
        configParam(OVERDRIVE_ATT, -1.f, 1.f, 0.0f, "Overdrive Distortion");

        configInput(TENSION_IN,   "Tension");
        configInput(RESONANCE_IN, "Resonance");
        configInput(NOISE_IN,     "Noise");
        configInput(SHAPE_IN,     "Shape");
        configInput(IMPULSE_IN,   "Impulse");
        configInput(OVERDRIVE_IN,  "Overdrive");
        configInput(PITCH_IN,  "Pitch (V/Oct)");

        configInput(AUDIO_INPUT, "Resonator");

        // Initialize all detune tables
        for (int c = 0; c < MAX_POLY; ++c)
            for (int i = 0; i < MAX_NODES; ++i)
                nodeDetune[c][i] = 1.0f + 0.01f * (float(i) - 8.0f); // center around 8
    }

    void onSampleRateChange() override {
        sampleRate = APP->engine->getSampleRate();

        // Support down to -8V (to cover delayMode’s -4V transpose)
        const float MIN_PITCH_V = -8.0f;
        float maxDelayFromPitch = vOctToDelaySec(MIN_PITCH_V);
        float maxDelay = clamp(maxDelayFromPitch * 1.1f, 0.02f, 0.5f); // allow longer buffer

        for (int c = 0; c < MAX_POLY; ++c) {
            hpf[c].setCutoffFrequency(sampleRate, 30.0f);
            for (int i = 0; i < MAX_NODES; ++i) {
                nodes[c][i].init(sampleRate, maxDelay);
            }
        }
    }

    // Convert V/oct to delay in seconds (same formula)
    float vOctToDelaySec(float vOct) {
        float f = 261.63f * powf(2.f, vOct); // frequency in Hz
        return 1.f / f;                       // delay in seconds
    }

    // shapeNodeDelays preserved exactly
    void shapeNodeDelays(int c, float pitchSec, float timbreShape) {
        timbreShape = clamp(timbreShape, -1.f, 1.f);

        if (timbreShape < 0.f) {
            float chaos = timbreShape * timbreShape;
            float minJitter = 0.00f;
            float maxJitter = 0.4f * chaos;
            float jitterRange = maxJitter - minJitter;

            for (int i = 0; i < nodeCount; ++i) {
                float jitter = minJitter + jitterRange * ((randf() + 1.f) * 0.5f);
                float detunedDelay = pitchSec * (1.f + jitter);
                nodes[c][i].setDelay(detunedDelay);
                nodes[c][i].damping = 0.03f + 0.08f * chaos;
            }
        } else {
            float shape = timbreShape * timbreShape;
            float maxSpread = 0.05f * shape;
            float spreadStep = (nodeCount > 1) ? (2.f * maxSpread) / (nodeCount - 1) : 0.f;

            for (int i = 0; i < nodeCount; ++i) {
                float spread = ((float)i * spreadStep) - maxSpread;
                float detunedDelay = pitchSec * (1.f + spread);
                nodes[c][i].setDelay(detunedDelay);
                nodes[c][i].damping = 0.02f + 0.2f * shape;
            }
        }
    }

    float excitationSample() { return randf(); }

    float applyADAA(float input, float lastInput) {
        float delta = input - lastInput;
        if (fabs(delta) > 1e-6f) {
            float i2 = input * input;
            float i4 = i2 * i2;
            float i6 = i4 * i2;
            float i8 = i4 * i4;
            float ad_i = i2 * 0.5f - i4 * 0.0833333f + i6 * 0.0222222f - i8 * 0.00674603f;

            float l2 = lastInput * lastInput;
            float l4 = l2 * l2;
            float l6 = l4 * l2;
            float l8 = l4 * l4;
            float ad_l = l2 * 0.5f - l4 * 0.0833333f + l6 * 0.0222222f - l8 * 0.00674603f;

            return (ad_i - ad_l) / delta;
        } else {
            return polyTanh(input);
        }
    }

    float polyTanh(float x) {
        float x2 = x * x;
        float x3 = x2 * x;
        float x5 = x3 * x2;
        float x7 = x5 * x2;
        return x - x3 * 0.333333f + x5 * 0.133333f - x7 * 0.0539683f;
    }

    float polySin(float x) {
        float x2 = x * x;
        float x3 = x2 * x;
        float x5 = x3 * x2;
        float x7 = x5 * x2;
        return x - x3 * 0.166667f + x5 * 0.00833333f - x7 * 0.000198413f;
    }

    float polyCos(float x) {
        float x2 = x * x;
        float x4 = x2 * x2;
        float x6 = x4 * x2;
        return 1.0f - x2 * 0.5f + x4 * 0.0416667f - x6 * 0.00138889f;
    }

    // Detect strike per voice (keeps original behavior for single voice)
    bool detectStrikeForVoice(int c) {
        bool trig = false;
        float v = 0.f;
        if (inputs[TRIG_INPUT].isConnected())
            v = inputs[TRIG_INPUT].getPolyVoltage(c);
        if (trigInputTrigger[c].process(v)) trig = true;

        float strikeVal = params[STRIKE_BUTTON].getValue();
        if (strikeButtonTrigger[c].process(strikeVal)) trig = true;

        if (trig) strikeState[c] = true;
        else if (v < 1.f) strikeState[c] = false;

        return trig;
    }

    float getParamValue(int c, int inputId, int attParamId, float paramValue, float scale = 1.f) {
        float in = 0.f;
        if (inputs[inputId].isConnected()) {
            int chCount = inputs[inputId].getChannels();
            if (chCount == 1) {
                // Mono input: broadcast to all voices
                in = inputs[inputId].getVoltage(0) * scale;
            } else {
                // Poly input: read per-voice channel
                in = inputs[inputId].getPolyVoltage(c) * scale;
            }
        }
        float att = params[attParamId].getValue(); // -1..1
        return paramValue + att * in;
    }

    void process(const ProcessArgs& args) override {
        if (sampleRate != APP->engine->getSampleRate())
            onSampleRateChange();

        // --- Determine channel count from all inputs ---
        int pitchCh = inputs[PITCH_IN].getChannels();
        int audioCh = inputs[AUDIO_INPUT].getChannels();
        int trigCh  = inputs[TRIG_INPUT].getChannels();
        int channels = std::max({pitchCh, audioCh, trigCh, 1});
        channels = std::min(channels, MAX_POLY);

        outputs[AUDIO_OUTPUT_L].setChannels(channels);
        outputs[AUDIO_OUTPUT_R].setChannels(channels);

        skipCounter++;

        if (skipCounter > processSkips) {
            // Precompute values for each channel using getParamValue()
            for (int c = 0; c < channels; ++c) {
                tension[c]   = clamp(pow(getParamValue(c, TENSION_IN, TENSION_ATT, params[TENSION_PARAM].getValue()), 2.f) * 0.15f, 0.f, 0.15f);
                resonance[c] = clamp(pow(getParamValue(c, RESONANCE_IN, RESONANCE_ATT, params[RESONANCE_PARAM].getValue()), 0.1f), 0.f, 1.0f);
                shape[c]     = clamp(getParamValue(c, SHAPE_IN, SHAPE_ATT, params[SHAPE_PARAM].getValue()), -1.f, 1.f);
                noise[c]     = clamp(getParamValue(c, NOISE_IN, NOISE_ATT, params[NOISE_PARAM].getValue()), 0.f, 1.f);
                impulse[c]   = clamp(2.f * getParamValue(c, IMPULSE_IN, IMPULSE_ATT, params[IMPULSE_PARAM].getValue()), 0.01f, 2.f);
                overdrive[c] = clamp(2.0f + 20.f * getParamValue(c, OVERDRIVE_IN, OVERDRIVE_ATT, params[OVERDRIVE_PARAM].getValue()), 2.0f, 22.0f);

                float pitchV = params[PITCH_PARAM].getValue();
                if (inputs[PITCH_IN].isConnected())
                    pitchV += inputs[PITCH_IN].getPolyVoltage(c);
                if (delayMode)
                    pitchV -= 4.f;  // delay mode transpose
                float pitchSec = clamp(vOctToDelaySec(pitchV), 0.0002f, 0.5f);
                shapeNodeDelays(c, pitchSec, shape[c]);
            }
            skipCounter = 0;
        }

        // Per-voice processing
        for (int c = 0; c < channels; ++c) {
            // detect strike per voice (uses per-voice jack input; button is global)
            bool struck = detectStrikeForVoice(c);
            if (struck) {
                strikeState[c] = true;
                exciteTime[c] = 0.f;
                exciteEnv[c] = 1.f;
                // randomize detune per voice just like before
                for (int i = 0; i < nodeCount; ++i)
                    nodeDetune[c][i] = 1.f + 0.02f * ((float)rand() / RAND_MAX - 0.5f);
                // re-shape node delays using last stored pitch for voice:
                float pitchV = 0.f;
                if (inputs[PITCH_IN].isConnected())
                    pitchV += inputs[PITCH_IN].getPolyVoltage(c);
                if (delayMode)
                    pitchV -= 4.f;  // same transpose
                float pitchSec = clamp(vOctToDelaySec(pitchV), 0.0002f, 0.5f);
                shapeNodeDelays(c, pitchSec, shape[c]);
            }

            float exciteSample = 0.f;
            if (exciteEnv[c] > 0.f) {
                exciteTime[c] += 1.f / sampleRate;
                float burstLength = impulse[c] * (inputs[PITCH_IN].isConnected()
                    ? clamp(vOctToDelaySec(0.f + inputs[PITCH_IN].getPolyVoltage(c)), 0.0002f, 0.25f)
                    : clamp(vOctToDelaySec(0.f), 0.0002f, 0.25f));
                burstLength = fmax(0.0005f, burstLength);
                if (exciteTime[c] < burstLength) {
                    float t = exciteTime[c] / burstLength;
                    exciteSample = 0.5f * (1.f - polyCos(M_PI * t)) * excitationSample() * exciteEnv[c];
                } else {
                    exciteEnv[c] *= 0.995f;
                    if (exciteEnv[c] < 1e-4f) exciteEnv[c] = 0.f;
                }
            }

            float simmerLevel = exciteEnv[c];

            // process nodes for this voice
            float nodeOutputs[MAX_NODES] = {0.f};
            float nodeCountFloat = (float)nodeCount;
            float invNodeCount = 1.0f / nodeCountFloat;

            float externalAudio = 0.f;
            if (inputs[AUDIO_INPUT].isConnected() ){
                externalAudio += inputs[AUDIO_INPUT].getPolyVoltage(c)*0.1f;
                externalAudio = hpf[c].process(externalAudio);
            }

            for (int i = 0; i < nodeCount; ++i) {
                float nodeExcite = exciteSample * (0.5f + 0.5f * ((float)i * invNodeCount));
                int left = (i - 1 + nodeCount) % nodeCount;
                int right = (i + 1) % nodeCount;
                float tensionTerm = tension[c] * (nodes[c][left].lastOut + nodes[c][right].lastOut - 2.f * nodes[c][i].lastOut);
                float sizzle = noise[c] * randf() * simmerLevel;
                float nodeInput = nodeExcite + tensionTerm + sizzle + externalAudio;
                nodes[c][i].resonance = resonance[c];
                nodeOutputs[i] = nodes[c][i].processSample(nodeInput);
            }

            // stereo mix
            float outL = 0.f, outR = 0.f;
            float panStep = (nodeCount > 1) ? (1.0f / (nodeCount - 1)) : 0.5f;
            for (int i = 0; i < nodeCount; ++i) {
                float pan = (float)i * panStep;
                outL += nodeOutputs[i] * polyCos(M_PI_2 * pan);
                outR += nodeOutputs[i] * polySin(M_PI_2 * pan);
            }

            float maxHeadRoom = 13.14f;
            outL = clamp(4.f * outL * overdrive[c], -maxHeadRoom, maxHeadRoom);
            outR = clamp(4.f * outR * overdrive[c], -maxHeadRoom, maxHeadRoom);
            outL = applyADAA(outL / 10.f, lastOutputL[c]);
            outR = applyADAA(outR / 10.f, lastOutputR[c]);
            lastOutputL[c] = outL;
            lastOutputR[c] = outR;

            outL = clamp(outL * 6.9f, -12.f, 12.f);
            outR = clamp(outR * 6.9f, -12.f, 12.f);

            outputs[AUDIO_OUTPUT_L].setVoltage(outL, c);
            outputs[AUDIO_OUTPUT_R].setVoltage(outR, c);
        } // end per-voice loop
    }
};

//////////////////////
// Widget
//////////////////////
struct AlloyWidget : ModuleWidget {
    AlloyWidget(Alloy* module) {
        setModule(module);

        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Alloy.svg"),
            asset::plugin(pluginInstance, "res/Alloy-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        const Vec knobStartPos = Vec(0, 50);

        const float knobSpacingY = 45.f;

        const float center = box.size.x/2;
        const float offset = box.size.x/6;

        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(center - 2*offset, 0*knobSpacingY )), module, Alloy::AUDIO_INPUT));

        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(center , 0*knobSpacingY )), module, Alloy::PITCH_IN));
        addParam(createParamCentered<RoundHugeBlackKnob>(knobStartPos.plus(Vec(center + 1.5*offset , 0*knobSpacingY)), module, Alloy::PITCH_PARAM));

        addParam(createParamCentered<RoundLargeBlackKnob>(knobStartPos.plus(Vec(center + 0.7*offset, 1.15*knobSpacingY)), module, Alloy::SHAPE_PARAM));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(center - offset, 1*knobSpacingY)), module, Alloy::SHAPE_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(center - 2*offset, 1*knobSpacingY )), module, Alloy::SHAPE_IN));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec(center , 2*knobSpacingY)), module, Alloy::TENSION_PARAM));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(center-offset, 2*knobSpacingY)), module, Alloy::TENSION_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(center - 2*offset, 2*knobSpacingY )), module, Alloy::TENSION_IN));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec(center, 3*knobSpacingY)), module, Alloy::RESONANCE_PARAM));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(center-offset, 3*knobSpacingY)), module, Alloy::RESONANCE_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(center - 2*offset, 3*knobSpacingY )), module, Alloy::RESONANCE_IN));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec(center , 4*knobSpacingY)), module, Alloy::OVERDRIVE_PARAM));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(center-offset, 4*knobSpacingY)), module, Alloy::OVERDRIVE_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(center - 2*offset, 4*knobSpacingY )), module, Alloy::OVERDRIVE_IN));

        addChild(createLightCentered<LargeLight<RedLight>>(knobStartPos.plus(Vec(center - 1.5*offset, 5.5*knobSpacingY)), module, Alloy::IMPULSE_LIGHT));
        addParam(createParamCentered<TL1105>(knobStartPos.plus(Vec(center - 1.5*offset, 5.5*knobSpacingY)), module, Alloy::STRIKE_BUTTON));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(center - 1.5*offset, 6.25*knobSpacingY)), module, Alloy::TRIG_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec(center+2*offset, 5.25*knobSpacingY)), module, Alloy::IMPULSE_PARAM));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(center+offset, 5.25*knobSpacingY)), module, Alloy::IMPULSE_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(center, 5.25*knobSpacingY )), module, Alloy::IMPULSE_IN));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec(center+2*offset, 6.25*knobSpacingY)), module, Alloy::NOISE_PARAM));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(center+offset, 6.25*knobSpacingY)), module, Alloy::NOISE_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(center, 6.25*knobSpacingY )), module, Alloy::NOISE_IN));

        addOutput(createOutputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(center+2*offset, 2.5*knobSpacingY)), module, Alloy::AUDIO_OUTPUT_L));
        addOutput(createOutputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(center+2*offset, 3.5*knobSpacingY)), module, Alloy::AUDIO_OUTPUT_R));
    }

    void draw(const DrawArgs& args) override {
        ModuleWidget::draw(args);
        Alloy* module = dynamic_cast<Alloy*>(this->module);
        if (!module) return;
            module->lights[Alloy::IMPULSE_LIGHT].setBrightness(module->exciteEnv[0]>0.0f ? 1.0f : 0.f);
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
        // Cast to Alloy
        Alloy* alloyModule = dynamic_cast<Alloy*>(module);
        assert(alloyModule);

        // Separator for clarity
        menu->addChild(new MenuSeparator);

        // Node Count Submenu
        struct NodeCountSubMenu : MenuItem {
            Alloy* alloyModule;

            Menu* createChildMenu() override {
                Menu* subMenu = new Menu;
                std::vector<int> counts = {8, 12, 16};

                for (int count : counts) {
                    struct NodeCountItem : MenuItem {
                        Alloy* alloyModule;
                        int nodeCount;
                        void onAction(const event::Action& e) override {
                            // Update the module's node count
                            alloyModule->nodeCount = nodeCount;
                        }
                        void step() override {
                            rightText = (alloyModule->nodeCount == nodeCount) ? "✔" : "";
                            MenuItem::step();
                        }
                    };
                    NodeCountItem* countItem = new NodeCountItem();
                    countItem->text = std::to_string(count) + " Nodes";
                    countItem->alloyModule = alloyModule;
                    countItem->nodeCount = count;
                    subMenu->addChild(countItem);
                }
                return subMenu;
            }
            void step() override {
                rightText = ">"; // Indicate submenu
                MenuItem::step();
            }
        };
        NodeCountSubMenu* nodeCountSubMenu = new NodeCountSubMenu();
        nodeCountSubMenu->text = "Set Node Count";
        nodeCountSubMenu->alloyModule = alloyModule;
        menu->addChild(nodeCountSubMenu);

        struct DelayModeItem : MenuItem {
            Alloy* alloyModule;

            void onAction(const event::Action& e) override {
                // Toggle the delayMode variable in the module
                alloyModule->delayMode = !alloyModule->delayMode;
            }

            void step() override {
                rightText = alloyModule->delayMode ? "✔" : ""; // Show checkmark if true
                MenuItem::step();
            }
        };

        // Create the Poly Output menu item
        DelayModeItem* delayModeItem = new DelayModeItem();
        delayModeItem->text = "Delay effect mode"; // Set menu item text
        delayModeItem->alloyModule = alloyModule;             // Pass the module pointer
        menu->addChild(delayModeItem);                      // Add to context menu

    }
};
Model* modelAlloy = createModel<Alloy, AlloyWidget>("Alloy");