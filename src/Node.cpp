////////////////////////////////////////////////////////////
//
//   Node
//
//   written by Cody Geary
//   Copyright 2025, MIT License
//
//   Two-channel stereo saturating VCA mixer
//
////////////////////////////////////////////////////////////
#include "rack.hpp"
#include "plugin.hpp"

struct Node : Module {
    enum ParamId {
        MUTE1_PARAM, MUTE2_PARAM,
        GAIN1_PARAM, GAIN2_PARAM,
        VOL_PARAM,
        XFADE_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        _1_IN1, _1_IN2,
        _2_IN1, _2_IN2,
        CV1_IN, CV2_IN,
        XFADE_IN,
        INPUTS_LEN
    };
    enum OutputId {
        OUT1, OUT2,
        OUTPUTS_LEN
    };
    enum LightId {
        LIGHT_1_1_L, LIGHT_1_2_L, LIGHT_1_3_L, LIGHT_1_4_L,  LIGHT_1_5_L,
        LIGHT_1_6_L, LIGHT_1_7_L, LIGHT_1_8_L, LIGHT_1_9_L, LIGHT_1_10_L,
        LIGHT_2_1_L, LIGHT_2_2_L, LIGHT_2_3_L, LIGHT_2_4_L,  LIGHT_2_5_L,
        LIGHT_2_6_L, LIGHT_2_7_L, LIGHT_2_8_L, LIGHT_2_9_L, LIGHT_2_10_L,
        LIGHT_1_1_R, LIGHT_1_2_R, LIGHT_1_3_R, LIGHT_1_4_R,  LIGHT_1_5_R,
        LIGHT_1_6_R, LIGHT_1_7_R, LIGHT_1_8_R, LIGHT_1_9_R, LIGHT_1_10_R,
        LIGHT_2_1_R, LIGHT_2_2_R, LIGHT_2_3_R, LIGHT_2_4_R,  LIGHT_2_5_R,
        LIGHT_2_6_R, LIGHT_2_7_R, LIGHT_2_8_R, LIGHT_2_9_R, LIGHT_2_10_R,

        VOL_LIGHT1L, VOL_LIGHT2L, VOL_LIGHT3L, VOL_LIGHT4L, VOL_LIGHT5L,
        VOL_LIGHT6L, VOL_LIGHT7L, VOL_LIGHT8L, VOL_LIGHT9L, VOL_LIGHT10L,
        VOL_LIGHT11L, VOL_LIGHT12L, VOL_LIGHT13L, VOL_LIGHT14L, VOL_LIGHT15L,
        VOL_LIGHT16L, VOL_LIGHT17L, VOL_LIGHT18L, VOL_LIGHT19L, VOL_LIGHT20L,
        VOL_LIGHT1R, VOL_LIGHT2R, VOL_LIGHT3R, VOL_LIGHT4R, VOL_LIGHT5R,
        VOL_LIGHT6R, VOL_LIGHT7R, VOL_LIGHT8R, VOL_LIGHT9R, VOL_LIGHT10R,
        VOL_LIGHT11R, VOL_LIGHT12R, VOL_LIGHT13R, VOL_LIGHT14R, VOL_LIGHT15R,
        VOL_LIGHT16R, VOL_LIGHT17R, VOL_LIGHT18R, VOL_LIGHT19R, VOL_LIGHT20R,
        
        XFADE_LIGHT, MUTE_LIGHT1, MUTE_LIGHT2,   
        LIGHTS_LEN
    };

    int cycleCount = 0;
    float alpha = 0.01f;
    float volTotalL = 0.0f,   volTotalR = 0.0f;
    float Ch1TotalL = 0.0f,   Ch1TotalR = 0.0f;
    float Ch2TotalL = 0.0f,   Ch2TotalR = 0.0f;
    float lastInputL = 0.0f, lastInputR = 0.0f;
    float volume = 0.0f;
    float Ch1L = 0.0f, Ch1R = 0.0f;
    float Ch2L = 0.0f, Ch2R = 0.0f;
    float outL = 0.0f, outR = 0.0f;
    float meterAccumL = 0.f, meterAccumR = 0.f;
    int meterSampleCount = 0;

    //for tracking the mute state of each channel
    dsp::SchmittTrigger mute1Trigger, mute2Trigger;
    bool muteLatch[2] = {false,false};
    bool muteState[2] = {false,false};
    bool muteStatePrevious[2] = {false,false};

    // For mute transition
    float transitionSamples = 100.f; // Number of samples to complete the transition, updated in config to 10ms
    float fadeLevel[2] = {1.0f, 1.0f};
    int transitionCount[2] = {0, 0};  // Array to track transition progress for each channel
    float transitionTime = 10.0f; // 10ms default
    
    bool polyOutput = false; //for polyphonic chaining option
    
    json_t* dataToJson() override {
        json_t* rootJ = json_object();

        // Save polyOutput
        json_object_set_new(rootJ, "polyOutput", json_boolean(polyOutput));

        // Save transitionTime
        json_object_set_new(rootJ, "transitionTime", json_real(transitionTime));
        json_object_set_new(rootJ, "transitionSamples", json_real(transitionSamples));

    
        // Save muteState array
        json_t* muteJ = json_array();
        for (int i = 0; i < 2; i++) {
            json_array_append_new(muteJ, json_boolean(muteState[i]));
        }
        json_object_set_new(rootJ, "muteState", muteJ);
    
        return rootJ;
    }
    
    void dataFromJson(json_t* rootJ) override {    
        // Load polyOutput
        json_t* polyJ = json_object_get(rootJ, "polyOutput");
        if (polyJ) {
            polyOutput = json_boolean_value(polyJ);
        }

        // Load transitionTime
        json_t* transitionTimeJ = json_object_get(rootJ, "transitionTime");
        if (transitionTimeJ) {
            transitionTime = json_real_value(transitionTimeJ);
        }
        json_t* transitionSamplesJ = json_object_get(rootJ, "transitionSamples");
        if (transitionSamplesJ) {
            transitionSamples = json_real_value(transitionSamplesJ);
        }

        // Load muteState array
        json_t* muteJ = json_object_get(rootJ, "muteState");
        if (muteJ) {
            for (int i = 0; i < 2; i++) {
                json_t* valJ = json_array_get(muteJ, i);
                if (valJ) {
                    muteState[i] = json_boolean_value(valJ);
                }
            }
        }
    }

    Node() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(MUTE1_PARAM, 0.f, 1.f, 0.f, "Chan. I Mute");
        configParam(MUTE2_PARAM, 0.f, 1.f, 0.f, "Chan. II Mute");
        configParam(GAIN1_PARAM, 0.f, 10.f, 1.f, "Gain I (0-10x)");
        configParam(GAIN2_PARAM, 0.f, 10.f, 1.f, "Gain II (0-10x)");
        configParam(VOL_PARAM, 0.f, 1.f, 1.0f, "Volume");
        configParam(XFADE_PARAM, -1.f, 1.f, 0.f, "Crossfader (-1=I, 1=II)");

        configInput(_1_IN1, "Chan. I L");
        configInput(_1_IN2, "Chan. I R");
        configInput(_2_IN1, "Chan. II L");
        configInput(_2_IN2, "Chan. II R");
        configInput(XFADE_IN, "Cr.fader CV");
        configInput(CV1_IN, "CV I");
        configInput(CV2_IN, "CV II");

        configOutput(OUT1, "Output L");
        configOutput(OUT2, "Output R");

        transitionSamples = transitionTime * 0.001f * APP->engine->getSampleRate(); // 10 ms * sample rate
    }

    void process(const ProcessArgs& args) override {
        cycleCount++;

        transitionSamples = transitionTime * 0.001f * APP->engine->getSampleRate(); // 10 ms * sample rate
    
        // ===== XFADE CV =====
        float xfadeParam = params[XFADE_PARAM].getValue();
        float xfadeCV = 0.f;
        if (inputs[XFADE_IN].isConnected()) {
            xfadeCV = clamp(inputs[XFADE_IN].getVoltage() * 0.2f, -1.f, 1.f);
            params[XFADE_PARAM].setValue(xfadeCV); // animate param if CV connected
        }
        float xfadeAmount = clamp(xfadeParam + xfadeCV, -1.f, 1.f);
        float channel2Amt = (xfadeAmount + 1.f) * 0.5f;
        float channel1Amt = 1.f - channel2Amt;
    
        // ===== Handle mutes and fade transitions =====
        if (mute1Trigger.process(params[MUTE1_PARAM].getValue())) {
            muteState[0] = !muteState[0];
            transitionCount[0] = transitionSamples;
        }
        if (mute2Trigger.process(params[MUTE2_PARAM].getValue())) {
            muteState[1] = !muteState[1];
            transitionCount[1] = transitionSamples;
        }
        for (int i = 0; i < 2; i++) {
            if (transitionCount[i] > 0) {
                fadeLevel[i] += (muteState[i] ? -1.f : 1.f) / transitionSamples;
                if ((muteState[i] && fadeLevel[i] <= 0.f) || (!muteState[i] && fadeLevel[i] >= 1.f)) {
                    fadeLevel[i] = muteState[i] ? 0.f : 1.f;
                    transitionCount[i] = 0;
                } else {
                    transitionCount[i]--;
                }
            } else {
                fadeLevel[i] = muteState[i] ? 0.f : 1.f;
            }
        }
    
        // ===== Process Channel 1 (stereo poly aware) =====
        int ch1Channels = std::max(inputs[_1_IN1].getChannels(), inputs[_1_IN2].getChannels());
        int cv1Channels = inputs[CV1_IN].getChannels();
        float ch1Lsum = 0.f, ch1Rsum = 0.f;
        for (int c = 0; c < ch1Channels; c++) {
            float inL = (inputs[_1_IN1].getChannels() > c) ? inputs[_1_IN1].getPolyVoltage(c) : 0.f;
            float inR = (inputs[_1_IN2].getChannels() > c) ? inputs[_1_IN2].getPolyVoltage(c) : inL;
    
            float cv = 1.f;
            if (cv1Channels == 1) {
                cv = clamp(inputs[CV1_IN].getPolyVoltage(0) * 0.1f, 0.f, 1.f);
            } else if (cv1Channels > c) {
                cv = clamp(inputs[CV1_IN].getPolyVoltage(c) * 0.1f, 0.f, 1.f);
            } else if (cv1Channels > 1) {
                cv = 0.f;
            }
            float gain = params[GAIN1_PARAM].getValue() * cv;
    
            inL *= fadeLevel[0] * gain;
            inR *= fadeLevel[0] * gain;
    
            ch1Lsum += inL;
            ch1Rsum += inR;
        }
        float in1L = (ch1Channels > 0) ? ch1Lsum / ch1Channels : 0.f;
        float in1R = (ch1Channels > 0) ? ch1Rsum / ch1Channels : 0.f;
    
        // ===== Process Channel 2 (stereo poly aware) =====
        int ch2Channels = std::max(inputs[_2_IN1].getChannels(), inputs[_2_IN2].getChannels());
        int cv2Channels = inputs[CV2_IN].getChannels();
        float ch2Lsum = 0.f, ch2Rsum = 0.f;
        for (int c = 0; c < ch2Channels; c++) {
            float inL = (inputs[_2_IN1].getChannels() > c) ? inputs[_2_IN1].getPolyVoltage(c) : 0.f;
            float inR = (inputs[_2_IN2].getChannels() > c) ? inputs[_2_IN2].getPolyVoltage(c) : inL;
    
            float cv = 1.f;
            if (cv2Channels == 1) {
                cv = clamp(inputs[CV2_IN].getPolyVoltage(0) * 0.1f, 0.f, 1.f);
            } else if (cv2Channels > c) {
                cv = clamp(inputs[CV2_IN].getPolyVoltage(c) * 0.1f, 0.f, 1.f);
            } else if (cv2Channels > 1) {
                cv = 0.f;
            }
            float gain = params[GAIN2_PARAM].getValue() * cv;
    
            inL *= fadeLevel[1] * gain;
            inR *= fadeLevel[1] * gain;
    
            ch2Lsum += inL;
            ch2Rsum += inR;
        }
        float in2L = (ch2Channels > 0) ? ch2Lsum / ch2Channels : 0.f;
        float in2R = (ch2Channels > 0) ? ch2Rsum / ch2Channels : 0.f;
    
        // ===== MIX AND OUTPUT =====
        volume = params[VOL_PARAM].getValue();
        Ch1L = in1L, Ch2L = in2L;
        Ch1R = in1R, Ch2R = in2R;
        outL = Ch1L * channel1Amt + Ch2L * channel2Amt;
        outR = Ch1R * channel1Amt + Ch2R * channel2Amt;
    
        // ===== METERING =====
        float sampleRate = args.sampleRate;
        float scaleFactor = sampleRate / 96000.f;
        float decayRate = pow(0.999f, scaleFactor);
        volTotalL = volTotalL * decayRate + fabs(outL) * (1.f - decayRate);
        volTotalR = volTotalR * decayRate + fabs(outR) * (1.f - decayRate);
        Ch1TotalL = Ch1TotalL * decayRate + fabs(Ch1L) * (1.f - decayRate);
        Ch1TotalR = Ch1TotalR * decayRate + fabs(Ch1R) * (1.f - decayRate);
        Ch2TotalL = Ch2TotalL * decayRate + fabs(Ch2L) * (1.f - decayRate);
        Ch2TotalR = Ch2TotalR * decayRate + fabs(Ch2R) * (1.f - decayRate);
    
        // ===== OUTPUT =====
        if (polyOutput) {
            // Output full poly channels instead of stereo mix
            int polyChannels = std::max(ch1Channels, ch2Channels);
            outputs[OUT1].setChannels(polyChannels);
            outputs[OUT2].setChannels(polyChannels);
        
            for (int c = 0; c < polyChannels; c++) {
                float ch1L = (inputs[_1_IN1].getChannels() > c) ? inputs[_1_IN1].getPolyVoltage(c) : 0.f;
                float ch1R = (inputs[_1_IN2].getChannels() > c) ? inputs[_1_IN2].getPolyVoltage(c) : ch1L;
                float ch2L = (inputs[_2_IN1].getChannels() > c) ? inputs[_2_IN1].getPolyVoltage(c) : 0.f;
                float ch2R = (inputs[_2_IN2].getChannels() > c) ? inputs[_2_IN2].getPolyVoltage(c) : ch2L;
        
                // Apply gains, fades, and crossfade per channel
                float outL = (ch1L * fadeLevel[0] * params[GAIN1_PARAM].getValue() * channel1Amt) +
                             (ch2L * fadeLevel[1] * params[GAIN2_PARAM].getValue() * channel2Amt);
                float outR = (ch1R * fadeLevel[0] * params[GAIN1_PARAM].getValue() * channel1Amt) +
                             (ch2R * fadeLevel[1] * params[GAIN2_PARAM].getValue() * channel2Amt);
        
                // Clamp and ADAA just like stereo mode
                float maxHeadRoom = 13.14f;
                outL = clamp(outL, -maxHeadRoom, maxHeadRoom);
                outR = clamp(outR, -maxHeadRoom, maxHeadRoom);

                float inputL = outL / 10.f; //fix ADAA to be more technically correct
                float inputR = outR / 10.f;
                outL = applyADAA(inputL, lastInputL, args.sampleRate);
                outR = applyADAA(inputR, lastInputR, args.sampleRate);
                lastInputL = inputL;  // Store input, not output
                lastInputR = inputR;
        
                outputs[OUT1].setVoltage(clamp(outL * volume * 6.9f, -10.f, 10.f), c);
                outputs[OUT2].setVoltage(clamp(outR * volume * 6.9f, -10.f, 10.f), c);
            }
        }
        else {
            // ===== ADAA stereo-mix path =====
            float maxHeadRoom = 13.14f;
            outL = clamp(outL, -maxHeadRoom, maxHeadRoom);
            outR = clamp(outR, -maxHeadRoom, maxHeadRoom);
            outL = applyADAA(outL / 10.f, lastInputL, args.sampleRate);
            outR = applyADAA(outR / 10.f, lastInputR, args.sampleRate);
            lastInputL = outL;
            lastInputR = outR;
        
            outputs[OUT1].setVoltage(clamp(outL * volume * 6.9f, -10.f, 10.f));
            outputs[OUT2].setVoltage(clamp(outR * volume * 6.9f, -10.f, 10.f));
        }
    
    }


    float getAverageVoltage(rack::engine::Input& input) {
        int channels = input.getChannels();
        if (channels == 0)
            return 0.f;
    
        float sum = 0.f;
        for (int c = 0; c < channels; ++c) {
            sum += input.getVoltage(c);
        }
        return sum / channels;
    }
 
    float applyADAA(float input, float lastInput, float sampleRate) {
        float delta = input - lastInput;
        if (fabs(delta) > 1e-6) {
            return (antiderivative(input) - antiderivative(lastInput)) / delta;
        } else {
            return polyTanh(input);
        }
    }


    float antiderivative(float x) {
        float x2 = x * x;
        return 0.5f * x2 - (1.0f/12.0f) * x2*x2 + (1.0f/45.0f) * x2*x2*x2 - (17.0f/2520.0f) * x2*x2*x2*x2;
    } 
   
    float polyTanh(float x) {
        float x2 = x * x;
        return x - x * x2 * (1.0f/3.0f - x2 * (2.0f/15.0f - 17.0f/315.0f * x2));
    }
    
    float polySin(float x) {
        float x2 = x * x;
        return x - x * x2 * (1.0f/6.0f - x2 * (1.0f/120.0f - x2 / 5040.0f));
    }
    
    float polyCos(float x) {
        float x2 = x * x;
        return 1.0f - x2 * (0.5f - x2 * (1.0f/24.0f - x2 / 720.0f));
    }

    
};

struct NodeWidget : ModuleWidget {
    NodeWidget(Node* module) {
        setModule(module);

        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Node.svg"),
            asset::plugin(pluginInstance, "res/Node-dark.svg")
        ));

        // Add screws or additional design elements as needed
        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        box.size = Vec(8 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT); // 8HP wide module

        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(11.064, 13.955)), module, Node::_2_IN1));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(11.064, 25.698)), module, Node::_2_IN2));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(23.766, 20.426)), module, Node::CV2_IN));

        addChild(createLightCentered<LargeLight<RedLight>>( mm2px(Vec(23.871, 29.533)), module, Node::MUTE_LIGHT2));
        addParam(createParamCentered<TL1105>(mm2px(Vec(23.871, 29.533)), module, Node::MUTE2_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(17.4, 38.161)), module, Node::GAIN2_PARAM));

        // Add Ring of Lights
        addLightsAroundKnob(module, mm2px(20.755), mm2px(67.399), Node::VOL_LIGHT1R, 20, 22.5f);
        addLightsAroundKnob(module, mm2px(20.755), mm2px(67.399), Node::VOL_LIGHT1L, 20, 26.5f);

        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(20.755, 67.399)), module, Node::VOL_PARAM));

        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(17.4, 96.159)), module, Node::GAIN1_PARAM));
        addChild(createLightCentered<LargeLight<RedLight>>( mm2px(Vec(23.871, 104.786)), module, Node::MUTE_LIGHT1));
        addParam(createParamCentered<TL1105>(mm2px(Vec(23.871, 104.786)), module, Node::MUTE1_PARAM));

        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(11.064, 108.621)), module, Node::_1_IN1));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(11.064, 120.364)), module, Node::_1_IN2));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(23.766, 113.894)), module, Node::CV1_IN));

        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(34.121, 60.929)), module, Node::OUT1));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(34.121, 72.672)), module, Node::OUT2));

        // VU meter 1 (top to bottom)
        const float x1 = 32.636f;
        const float y1_bottom = 13.344f;
        const float y1_top = 42.601f;
        for (int i = 0; i < 10; i++) {
            float t = i / 9.f;  // interpolate between 0 and 1
            float y = y1_top * (1 - t) + y1_bottom * t;
            if (i<5){    
                addChild(createLightCentered<SmallLight<YellowLight>>( mm2px(Vec(x1, y)), module, Node::LIGHT_2_1_L + i));
                addChild(createLightCentered<SmallLight<YellowLight>>( mm2px(Vec(x1+2, y)), module, Node::LIGHT_2_1_R + i));
            } else {
                addChild(createLightCentered<SmallLight<RedLight>>( mm2px(Vec(x1, y)), module, Node::LIGHT_2_1_L + i));
                addChild(createLightCentered<SmallLight<RedLight>>( mm2px(Vec(x1+2, y)), module, Node::LIGHT_2_1_R + i));
            }
        }

        // VU meter 2 (top to bottom)
        const float y2_bottom = 90.185f;
        const float y2_top = 121.442f;
        for (int i = 0; i < 10; i++) {
            float t = i / 9.f;
            float y = y2_top * (1 - t) + y2_bottom * t;

            if (i<5){    
                addChild(createLightCentered<SmallLight<YellowLight>>( mm2px(Vec(x1, y)), module, Node::LIGHT_1_1_L + i));
                addChild(createLightCentered<SmallLight<YellowLight>>( mm2px(Vec(x1+2, y)), module, Node::LIGHT_1_1_R + i));
            } else {
                addChild(createLightCentered<SmallLight<RedLight>>( mm2px(Vec(x1, y)), module, Node::LIGHT_1_1_L + i));
                addChild(createLightCentered<SmallLight<RedLight>>( mm2px(Vec(x1+2, y)), module, Node::LIGHT_1_1_R + i));
            }
        }

        addParam(createLightParamCentered<VCVLightSlider<YellowLight>>(mm2px(Vec(7.198, 53.7+13.304)), module, Node::XFADE_PARAM, Node::XFADE_LIGHT));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.198, 53.7+29.608)), module, Node::XFADE_IN));

    }

    void step() override {
        Node* module = dynamic_cast<Node*>(this->module);
        if (!module) return;

        module->lights[Node::MUTE_LIGHT1].setBrightness(module->muteState[0] ? 1.0f : 0.f);
        module->lights[Node::MUTE_LIGHT2].setBrightness(module->muteState[1] ? 1.0f : 0.f);
        updateLights();
        ModuleWidget::step();
    }

    void updateLights() {
        Node* module = dynamic_cast<Node*>(this->module);
        if (!module) return;
            
            updateSegmentedLights(Node::VOL_LIGHT1L, module->volTotalL*module->volume, 10.0f, 20); //Main Vol
            updateSegmentedLights(Node::VOL_LIGHT1R, module->volTotalR*module->volume, 10.0f, 20);
            updateSegmentedLights(Node::LIGHT_1_1_L, module->Ch1TotalL, 10.0f, 10); //Ch1 VU
            updateSegmentedLights(Node::LIGHT_1_1_R, module->Ch1TotalR, 10.0f, 10);
            updateSegmentedLights(Node::LIGHT_2_1_L, module->Ch2TotalL, 10.0f, 10); //Ch2 VU
            updateSegmentedLights(Node::LIGHT_2_1_R, module->Ch2TotalR, 10.0f, 10);

            module->lights[Node::XFADE_LIGHT].setBrightness( clamp((module->volTotalL+module->volTotalR)/20.f, 0.f, 1.f)); //Crossfader

    }

    void updateSegmentedLights(int startLightId, float totalValue, float maxValue, int numLights) {
        float normalizedValue = totalValue / maxValue;
        int fullLights = static_cast<int>(normalizedValue * numLights);
        float fractionalBrightness = (normalizedValue * numLights) - fullLights;

        for (int i = 0; i < numLights; i++) {
            if (i < fullLights) {
                module->lights[startLightId + i].setBrightness(1.0f); // Full brightness for fully covered segments
            } else if (i == fullLights) {
                module->lights[startLightId + i].setBrightness(fractionalBrightness); // Partial brightness for the last partially covered segment
            } else {
                float dimming = module->lights[startLightId + i].getBrightness();
                module->lights[startLightId + i].setBrightness(dimming * 0.75f);
            }
        }
    }

    void addLightsAroundKnob(Module* module, float knobX, float knobY, int firstLightId, int numLights, float radius) {
        const float startAngle = M_PI*0.7f; // Start angle in radians (8 o'clock on the clock face)
        const float endAngle = 2.0f*M_PI+M_PI*0.3f;   // End angle in radians (4 o'clock on the clock face)

        for (int i = 0; i < numLights; i++) {
            float fraction = (float)i / (numLights - 1); // Fraction that goes from 0 to 1
            float angle = startAngle + fraction * (endAngle - startAngle);
            float x = knobX + radius * cos(angle);
            float y = knobY + radius * sin(angle);

            // Create and add the light
            if (i< .5*numLights){
                addChild(createLightCentered<TinyLight<YellowLight>>(Vec(x, y), module, firstLightId + i));
            } else {
                addChild(createLightCentered<TinyLight<RedLight>>(Vec(x, y), module, firstLightId + i));
            }
        }
    }

    // Generic Quantity for any float member 
    struct FloatMemberQuantity : Quantity {
        Node* module;
        float Node::*member; // pointer-to-member
        std::string label;
        float min, max, def;
        int precision;
    
        FloatMemberQuantity(Node* m, float Node::*mem, std::string lbl,
                            float mn, float mx, float df, int prec = 0)
            : module(m), member(mem), label(lbl), min(mn), max(mx), def(df), precision(prec) {}
    
        void setValue(float v) override { module->*member = clamp(v, min, max); }
        float getValue() override { return module->*member; }
        float getDefaultValue() override { return def; }
        float getMinValue() override { return min; }
        float getMaxValue() override { return max; }
        int getDisplayPrecision() override { return precision; }
    
        std::string getLabel() override { return label; }
        std::string getDisplayValueString() override {
            if (precision == 0)
                return std::to_string((int)std::round(getValue()));
            return string::f("%.*f", precision, getValue());
        }
    };

    void appendContextMenu(Menu* menu) override {
        Node* nodeModule = dynamic_cast<Node*>(this->module);
        assert(nodeModule);
    
        // Separator for new section
        menu->addChild(new MenuSeparator);
    
        // Poly Output Menu Item
        struct PolyOutputItem : MenuItem {
            Node* nodeModule;
    
            void onAction(const event::Action& e) override {
                // Toggle the polyOutput variable in the module
                nodeModule->polyOutput = !nodeModule->polyOutput;
            }
    
            void step() override {
                rightText = nodeModule->polyOutput ? "✔" : ""; // Show checkmark if true
                MenuItem::step();
            }
        };
    
        // Create the Poly Output menu item
        PolyOutputItem* polyOutputItem = new PolyOutputItem();
        polyOutputItem->text = "Output poly instead of mix"; // Set menu item text
        polyOutputItem->nodeModule = nodeModule;             // Pass the module pointer
        menu->addChild(polyOutputItem);                      // Add to context menu
        
        // Envelope polySpan
        auto* fadeSlider = new ui::Slider();
        fadeSlider->quantity = new FloatMemberQuantity(nodeModule, &Node::transitionTime,
            "Mute Fade Time (ms)", 1.f, 2000.f, 19.f, 0);
        fadeSlider->box.size.x = 200.f;
        menu->addChild(fadeSlider);
                
    }


};
Model* modelNode = createModel<Node, NodeWidget>("Node");