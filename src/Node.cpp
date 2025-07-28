#include "rack.hpp"
#include "plugin.hpp"

struct Node : Module {
    enum ParamId {
        MUTE1_PARAM,
        MUTE2_PARAM,
        GAIN1_PARAM,
        GAIN2_PARAM,
        VOL_PARAM,
        XFADE_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        _1_IN1,
        _1_IN2,
        _2_IN1,
        _2_IN2,
        CV1_IN,
        CV2_IN,
        XFADE_IN,
        INPUTS_LEN
    };
    enum OutputId {
        OUT1,
        OUT2,
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
    float volTotalL = 0.0f;
    float volTotalR = 0.0f;
    float Ch1TotalL = 0.0f;
    float Ch1TotalR = 0.0f;
    float Ch2TotalL = 0.0f;
    float Ch2TotalR = 0.0f;
    float lastOutputL = 0.0f;
    float lastOutputR = 0.0f;

    //for tracking the mute state of each channel
    dsp::SchmittTrigger mute1Trigger, mute2Trigger;
    bool muteLatch[2] = {false,false};
    bool muteState[2] = {false,false};
    bool muteStatePrevious[2] = {false,false};

    // For mute transition
    float transitionSamples = 100.f; // Number of samples to complete the transition, updated in config to 10ms
    float fadeLevel[2] = {1.0f, 1.0f};
    int transitionCount[2] = {0, 0};  // Array to track transition progress for each channel


    Node() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(MUTE1_PARAM, 0.f, 1.f, 0.f, "Channel I Mute");
        configParam(MUTE2_PARAM, 0.f, 1.f, 0.f, "Channel II Mute");

        configParam(GAIN1_PARAM, 0.f, 5.f, 1.f, "Gain I (0-5x)");
        configParam(GAIN2_PARAM, 0.f, 5.f, 1.f, "Gain II (0-5x)");
        configParam(VOL_PARAM, 0.f, 1.f, 0.5f, "Volume");
        configParam(XFADE_PARAM, -1.f, 1.f, 0.f, "Crossfader (-1=I, 1=II)");

        configInput(_1_IN1, "Channel I L");
        configInput(_1_IN2, "Channel I R");
        configInput(_2_IN1, "Channel II L");
        configInput(_2_IN2, "Channel II R");
        configInput(XFADE_IN, "Crossfader (-5...5V range)");

        configInput(CV1_IN, "CV I (0...10V range)");
        configInput(CV2_IN, "CV II (0...10V range)");
        configOutput(OUT1, "Output L");
        configOutput(OUT2, "Output R");

        transitionSamples = 0.01 * APP->engine->getSampleRate(); // 10 ms * sample rate
    }

    void process(const ProcessArgs& args) override {
    
        cycleCount++;
        
        // CV GAIN 
        float CV1 = 1.0f;
        if (inputs[CV1_IN].isConnected())
            CV1 = clamp(inputs[CV1_IN].getVoltage() * 0.1f, 0.f, 1.f);
        float gainChannel1 = params[GAIN1_PARAM].getValue() * CV1;
    
        float CV2 = 1.0f;
        if (inputs[CV2_IN].isConnected())
            CV2 = clamp(inputs[CV2_IN].getVoltage() * 0.1f, 0.f, 1.f);
        float gainChannel2 = params[GAIN2_PARAM].getValue() * CV2; 
    
        // XFADE CV 
        float xfadeCV = params[XFADE_PARAM].getValue();
        if (inputs[XFADE_IN].isConnected()) {
            xfadeCV = clamp(inputs[XFADE_IN].getVoltage() * 0.2f, -1.f, 1.f);
            params[XFADE_PARAM].setValue(xfadeCV); //override and animate the param if input is connected   
        }
        
        float xfadeAmount = clamp(params[XFADE_PARAM].getValue() + xfadeCV, -1.f, 1.f);
        float channel2Amt = (xfadeAmount + 1.f) * 0.5f;
        float channel1Amt = 1.f - channel2Amt;
    
        // INPUTS 
        float in1L = 0.f, in1R = 0.f;
        if (inputs[_1_IN1].isConnected() && inputs[_1_IN2].isConnected()) {
            in1L = inputs[_1_IN1].getVoltage();
            in1R = inputs[_1_IN2].getVoltage();
        } else if (inputs[_1_IN1].isConnected()) {
            in1L = in1R = inputs[_1_IN1].getVoltage();
        } else if (inputs[_1_IN2].isConnected()) {
            in1L = in1R = inputs[_1_IN2].getVoltage();
        }
    
        float in2L = 0.f, in2R = 0.f;
        if (inputs[_2_IN1].isConnected() && inputs[_2_IN2].isConnected()) {
            in2L = inputs[_2_IN1].getVoltage();
            in2R = inputs[_2_IN2].getVoltage();
        } else if (inputs[_2_IN1].isConnected()) {
            in2L = in2R = inputs[_2_IN1].getVoltage();
        } else if (inputs[_2_IN2].isConnected()) {
            in2L = in2R = inputs[_2_IN2].getVoltage();
        }

        // Handle Mutes
        if (mute1Trigger.process(params[MUTE1_PARAM].getValue())) {
            muteState[0] = !muteState[0];
            transitionCount[0] = transitionSamples;  // Reset the transition count
            lights[MUTE_LIGHT1].setBrightness(muteState[0] ? 1.0f : 0.f) ;
        }
        if (mute2Trigger.process(params[MUTE2_PARAM].getValue())) {
            muteState[1] = !muteState[1];
            transitionCount[1] = transitionSamples;  // Reset the transition count
            lights[MUTE_LIGHT2].setBrightness(muteState[1] ? 1.0f : 0.f) ;
        }

        for (int i=0; i<2; i++){
            if (transitionCount[i] > 0) {
                if (muteState[i]){
                    fadeLevel[i] += -1.0f/transitionSamples;
                } else {
                    fadeLevel[i] += 1.0f/transitionSamples;
                }
    
                // Clamp fade level at boundaries
                if ((muteState[i] && fadeLevel[i] <= 0.0f) || (!muteState[i] && fadeLevel[i] >= 1.0f)) {
                    fadeLevel[i] = muteState[i] ? 0.0f : 1.0f;
                    transitionCount[i] = 0;  // End transition
                } else {
                    transitionCount[i]--;  // Decrease transition count
                }
            } else {
                // Set fadeLevel to the target value once transition completes
                fadeLevel[i] = muteState[i] ? 0.0f : 1.0f;
            }
        }

        // Fade the signal in or out based on the Mute   
        in1L *= fadeLevel[0]; 
        in1R *= fadeLevel[0];
        in2L *= fadeLevel[1];  
        in2R *= fadeLevel[1];
   
        // MIX AND OUTPUT 
        float volume = params[VOL_PARAM].getValue();
        float Ch1L = in1L * gainChannel1 ;
        float Ch2L = in2L * gainChannel2 ;
        float outL = Ch1L * channel1Amt + Ch2L * channel2Amt;      
        float Ch1R = in1R * gainChannel1 ; 
        float Ch2R = in2R * gainChannel2 ;        
        float outR = Ch1R * channel1Amt + Ch2R * channel2Amt;
 
        // MEASURE VOLUMES
        // Calculate scale factor based on the current sample rate
        float sampleRate = args.sampleRate;
        float scaleFactor =  sampleRate / 96000.0f; // Reference sample rate (96 kHz)
        float decayRate = pow(0.999f, scaleFactor);  // Decay rate adjusted for sample rate
        
        volTotalL = volTotalL * decayRate + fabs(outL) * (1.0f - decayRate);
        volTotalR = volTotalR * decayRate + fabs(outR) * (1.0f - decayRate);
        
        Ch1TotalL = Ch1TotalL * decayRate + fabs(Ch1L) * (1.0f - decayRate);
        Ch1TotalR = Ch1TotalR * decayRate + fabs(Ch1R) * (1.0f - decayRate);
        Ch2TotalL = Ch2TotalL * decayRate + fabs(Ch2L) * (1.0f - decayRate);
        Ch2TotalR = Ch2TotalR * decayRate + fabs(Ch2R) * (1.0f - decayRate);

        // Apply ADAA
        float maxHeadRoom = 13.14f; 
        outL = clamp(outL, -maxHeadRoom, maxHeadRoom);
        outR = clamp(outR, -maxHeadRoom, maxHeadRoom);
        outL = applyADAA(outL/10.f, lastOutputL, sampleRate); 
        outR = applyADAA(outR/10.f, lastOutputR, sampleRate);
        lastOutputL = outL;
        lastOutputR = outR;

        // OUTPUT    
        outputs[OUT1].setVoltage(clamp(outL*volume*6.9f, -10.f, 10.f));
        outputs[OUT2].setVoltage(clamp(outR*volume*6.9f, -10.f, 10.f));

        // Update lights periodically
        updateLights();
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
        float x4 = x2 * x2;
        float x6 = x4 * x2;
        float x8 = x4 * x4;
        return x2 / 2.0f - x4 / 12.0f + x6 / 45.0f - 17.0f * x8 / 2520.0f;
    }

    float polyTanh(float x) {
        float x2 = x * x;       // x^2
        float x3 = x2 * x;      // x^3
        float x5 = x3 * x2;     // x^5
        float x7 = x5 * x2;     // x^7
        return x - x3 / 3.0f + (2.0f * x5) / 15.0f - (17.0f * x7) / 315.0f;
    }
    
    void updateLights() {
        if (++cycleCount >= 2000) { //Save CPU by updating lights infrequently
            
            updateSegmentedLights(VOL_LIGHT1L, volTotalL, 10.0f, 20); //Main Vol
            updateSegmentedLights(VOL_LIGHT1R, volTotalR, 10.0f, 20);

            updateSegmentedLights(LIGHT_1_1_L, Ch1TotalL, 10.0f, 10); //Ch1 VU
            updateSegmentedLights(LIGHT_1_1_R, Ch1TotalR, 10.0f, 10);

            updateSegmentedLights(LIGHT_2_1_L, Ch2TotalL, 10.0f, 10); //Ch2 VU
            updateSegmentedLights(LIGHT_2_1_R, Ch2TotalR, 10.0f, 10);

            lights[XFADE_LIGHT].setBrightness( clamp((volTotalL+volTotalR)/20.f, 0.f, 1.f)); //Crossfader

            cycleCount = 0;
        }
    }

    void updateSegmentedLights(int startLightId, float totalValue, float maxValue, int numLights) {
        float normalizedValue = totalValue / maxValue;
        int fullLights = static_cast<int>(normalizedValue * numLights);
        float fractionalBrightness = (normalizedValue * numLights) - fullLights;

        for (int i = 0; i < numLights; i++) {
            if (i < fullLights) {
                lights[startLightId + i].setBrightness(1.0f); // Full brightness for fully covered segments
            } else if (i == fullLights) {
                lights[startLightId + i].setBrightness(fractionalBrightness); // Partial brightness for the last partially covered segment
            } else {
                float dimming = lights[startLightId + i].getBrightness();
                lights[startLightId + i].setBrightness(dimming * 0.75f);
            }
        }
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

};
Model* modelNode = createModel<Node, NodeWidget>("Node");