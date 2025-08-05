////////////////////////////////////////////////////////////
//
//   Wonk
//
//   written by Cody Geary
//   Copyright 2025, MIT License
//
//   Six-channel clock-synced LFO with wonky feedback
//
////////////////////////////////////////////////////////////

#include "plugin.hpp"

struct Wonk : Module {

    static inline float linearInterpolate(float a, float b, float fraction) {
        return a + fraction * (b - a);
    }

    enum ParamId {
        RATE_ATT,
        RATE_KNOB,
        WONK_KNOB,
        WONK_ATT,
        POS_KNOB,
        NODES_ATT,
        NODES_KNOB,
        MOD_DEPTH_ATT,
        MOD_DEPTH,
        RESET_BUTTON,
        PARAMS_LEN
    };
    enum InputId {
        CLOCK_INPUT,
        RATE_INPUT,
        WONK_INPUT,
        NODES_INPUT,
        MOD_DEPTH_INPUT,
        RESET_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        _1_OUTPUT,
        _2_OUTPUT,
        _3_OUTPUT,
        _4_OUTPUT,
        _5_OUTPUT,
        _6_OUTPUT,
        POLY_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        RESET_LIGHT,
        LIGHTS_LEN
    };

    dsp::SchmittTrigger clockTrigger, resetTrigger, resetButton;
    dsp::Timer syncTimer;

    bool syncPoint = false; 
    float syncInterval = 1.0f;

    float freqHz = 1.0f; //frequency in Hz
    bool firstPulseReceived = false;
    bool firstSync = true; //it is the first sync at the start

    int processSkipper = 0; // updating less important stuff less often
    int processSkips = 100; // frequency of skipped updates
    int processCounter = 0; // Counter to track staggered processing
    bool prevcableConnected = false; //check if a poly cable was plugged into the output
    
    float lfoPhase[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}; // Current LFO phase for each channel
    float prevPhaseResetInput[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}; // Previous envelope input, for peak detection
    float calculateTargetPhase(int channel, float nodePosition, float deltaTime, float place);
    void adjustLFOPhase(int channel, float targetPhase, float envInfluence, float deltaTime);
    float lastTargetVoltages[5] = {0.f, 0.f, 0.f, 0.f, 0.f}; // Initialize with default voltages, assuming start at 0V
    float place[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    float happy_place[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    float lfoOutput[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    float nextChunk[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    int SINprocessCounter = 0; // Counter to track process cycles
    int SkipProcesses = 16; //Number of process cycles to skip for the sine calculation
    float wonkMod[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}; //store the scaled wonkMod output
    float rawwonkMod[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}; //store the raw value
    bool modClockSync = false; //Sync the modulation bus to the master clock instead of stage clock
    float modulationDepth = 5.0f;

    dsp::PulseGenerator syncPulse;

    Wonk() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(RATE_ATT, -1.f, 1.f, 0.f, "Rate Attenuverter");
        configParam(RESET_BUTTON, 0.f, 1.f, 0.f, "Reset Button");
        configParam(RATE_KNOB, -24.0f, 24.f, 1.f, "Rate multiplier (or divider for negative)");
        configParam(WONK_KNOB, 0.f, 1.f, 0.f, "Wonk Intensity");
        configParam(WONK_ATT, -1.f, 1.f, 0.f, "Wonk Input Attenuverter");
        configParam(POS_KNOB, 1.f, 6.f, 1.f, "Wonk Feedback Position");
        configParam(NODES_ATT, -1.f, 1.f, 0.f, "Nodes Atternuverter");
        configParam(NODES_KNOB, -3.f, 3.f, 1.f, "Number of Modulation Nodes");
        configParam(MOD_DEPTH_ATT, -1.f, 1.f, 0.f, "Modulation Depth Attenuverter");
        configParam(MOD_DEPTH, 0.f, 5.f, 5.f, "Modulation Depth");
        configInput(CLOCK_INPUT, "Clock");
        configInput(RESET_INPUT, "Reset");
        configInput(RATE_INPUT, "Rate");
        configInput(WONK_INPUT, "Wonk");
        configInput(NODES_INPUT, "Nodes");
        configInput(MOD_DEPTH_INPUT, "Modulation Depth");
        configOutput(_1_OUTPUT, "1");
        configOutput(_2_OUTPUT, "2");
        configOutput(_3_OUTPUT, "3");
        configOutput(_4_OUTPUT, "4");
        configOutput(_5_OUTPUT, "5");
        configOutput(_6_OUTPUT, "6");
        configOutput(POLY_OUTPUT, "Polyphonic");
    }

    void process(const ProcessArgs& args) override {

        float deltaTime = args.sampleTime;
        syncTimer.process(deltaTime);

        // Clock handling logic
        bool externalClockConnected = inputs[CLOCK_INPUT].isConnected();
        if (externalClockConnected && clockTrigger.process(inputs[CLOCK_INPUT].getVoltage() - 0.1f)) {
            if (firstPulseReceived) {
                syncInterval = syncTimer.time;
                syncTimer.reset();
                firstSync = false;
            }
            firstPulseReceived = true;
        }

        // Resetting Logic
        bool resetConnected = inputs[RESET_INPUT].isConnected();
        syncPoint = false;
        if (resetConnected && resetTrigger.process(inputs[RESET_INPUT].getVoltage() - 0.1f)) syncPoint = true;
        if (resetButton.process(params[RESET_BUTTON].getValue())) syncPoint = true;
        if (syncPoint) syncPulse.trigger(0.2f);
        bool syncActive = syncPulse.process(args.sampleTime);
        if (syncActive) {
            lights[RESET_LIGHT].setBrightness(1.0f);
        } else {
            lights[RESET_LIGHT].setBrightness(0.f);
        }

        // Compute Modulation Depth
        modulationDepth = params[MOD_DEPTH].getValue();
        if (inputs[MOD_DEPTH_INPUT].isConnected()){
            modulationDepth = clamp (inputs[MOD_DEPTH_INPUT].getVoltage() * params[MOD_DEPTH_ATT].getValue() * 0.5f + modulationDepth, 0.f, 5.f); //map 0-10V to 5V.
        } 

        processSkipper++;
        if (processSkipper>=processSkips){
            // Track connection states of each output
            bool cableConnected = outputs[POLY_OUTPUT].isConnected();

            // Re-initialize polyphonic channels only if connections change
            if (cableConnected && !prevcableConnected) {
                outputs[POLY_OUTPUT].setChannels(6);
            }

            // Update previous connection states
            prevcableConnected = cableConnected;
            
            processSkipper = 0;
        }

        if (modulationDepth > 0.0f){ // skip the whole LFO logic if mod depth is 0

            SINprocessCounter++;  //Skip some SINE computations to save CPU  
            float currentOutput[6] = {0.0f}; // Array to store the output voltages
            float rawRate = params[RATE_KNOB].getValue(); 
            if (inputs[RATE_INPUT].isConnected()) {
                rawRate = inputs[RATE_INPUT].getVoltage() * params[RATE_ATT].getValue() + rawRate;
            }
            float rate = 1.0f;
    
            // Compute the effective rate
            if (rawRate >= 1.0f) {
                // Positive values: Multiplier
                rate = rawRate;
            } else if (rawRate <= -1.0f) {
                // Negative values: Divider
                rate = 1.0f / std::abs(rawRate);
            } else {
                // Handle the -1.0 to 1.0 range (e.g., default to 1.0)
                rate = 1.0f;
            }
    
            rate = rate * freqHz; 
    
            float nodePosition = params[NODES_KNOB].getValue();
            if (inputs[NODES_INPUT].isConnected()){
                nodePosition = nodePosition + inputs[NODES_INPUT].getVoltage() * params[NODES_ATT].getValue();
            }
            float modRate[6] = {rate, rate, rate, rate, rate, rate};
            float wonky = params[WONK_KNOB].getValue();
            if (inputs[WONK_INPUT].isConnected()){
                wonky = clamp(inputs[WONK_INPUT].getVoltage()*params[WONK_ATT].getValue() / 10.f + wonky, 0.f, 1.f);
            }
            int wonkPos = static_cast<int>( clamp( roundf(params[POS_KNOB].getValue() - 0.5f), 0.0f, 5.0f ) );
    
            nodePosition = clamp(nodePosition + nodePosition * (wonky * wonkMod[wonkPos] / 25.0f), -3.0f, 3.0f);
            deltaTime = args.sampleTime;
                
            for (int i = 0; i < 6; i++) {
                int adjWonkPos = wonkPos+i;
                if (adjWonkPos >= 6){adjWonkPos -= 6;}
                if (adjWonkPos < 0){adjWonkPos += 6;}
    
                modRate[i] = rate + rate * 0.95f * (wonky * wonkMod[adjWonkPos] / 5.0f);
    
                float basePhase = i / -6.0f;
                float targetPhase = basePhase;
    
                if (nodePosition < -2.0f) {
                    // -3.0 <= nodePosition < -2.0: Bimodal behavior (reversed)
                    float trimodalPhase = (i % 3) / 3.0f; // Calculate trimodal phase negatively
                    float bimodalPhase = (i % 2) / 2.0f; // Calculate bimodal phase negatively
                    float blendFactor = -nodePosition - 2.0f; // Shift to a positive blend factor
                    targetPhase = linearInterpolate(trimodalPhase * -1.0f, bimodalPhase * -1.0f, blendFactor);
                } else if (nodePosition < -1.0f) {
                    // -2.0 <= nodePosition < -1.0: Transition towards trimodal (synchronized)
                    float trimodalPhase = (i % 3) / 3.0f; // Calculate trimodal phase negatively
                    float blendFactor = -nodePosition - 1.0f; // Shift blend factor to ensure synchronization
                    targetPhase = linearInterpolate(basePhase * -1.0f, trimodalPhase * -1.0f, blendFactor); // Synchronized transition
                } else if (nodePosition < 0.0f) {
                    // -1.0 <= nodePosition < 0.0: Hexamodal behavior (reversed)
                    targetPhase = linearInterpolate(0.5f, basePhase * -1.0f, -nodePosition); // Hexamodal reflected negatively
                } else if (nodePosition < 1.0f) {
                    // 0.0 <= nodePosition < 1.0: Hexamodal behavior
                    targetPhase = linearInterpolate(0.5f, basePhase, nodePosition); // Transition to hexamodal
                } else if (nodePosition < 2.0f) {
                    // 1.0 <= nodePosition < 2.0: Transition from hexamodal to trimodal
                    float trimodalPhase = (i % 3) / 3.0f; // Calculate trimodal phase positively
                    float blendFactor = nodePosition - 1.0f; // Shift blend factor to ensure synchronization
                    targetPhase = linearInterpolate(basePhase , trimodalPhase , blendFactor); // Synchronized transition
                } else {
                    // 2.0 <= nodePosition < 3.0: Bimodal behavior
                    float bimodalPhase = (i % 2) / 2.0f; // Calculate bimodal phase positively
                    float trimodalPhase = (i % 3) / 3.0f; // Calculate trimodal phase positively
                    float blendFactor = nodePosition - 2.0f; // Blend factor for interpolation
                    targetPhase = linearInterpolate(trimodalPhase, bimodalPhase, blendFactor); // Blend trimodal behavior
                }
    
                if (syncPoint ){
                    place[i] = 0.0f;
                    lfoPhase[i] = targetPhase;
                }
    
                targetPhase += place[i];
    
                while (targetPhase >= 1.0f) targetPhase -= 1.0f;
                while (targetPhase < 0.0f) targetPhase += 1.0f;
    
                float phaseDiff = targetPhase - lfoPhase[i];
                if (phaseDiff > 0.5f) phaseDiff -= 1.0f;
                if (phaseDiff < -0.5f) phaseDiff += 1.0f;
    
                float placeDiff = 0.f - place[i];
                if (placeDiff > 0.5f) placeDiff -= 1.0f;
                if (placeDiff < -0.5f) placeDiff += 1.0f;
    
                if (syncPoint) {
                    lfoPhase[i] += phaseDiff * 0.2f;
                } else {
                    lfoPhase[i] += phaseDiff * 0.2f;
                }
    
                while (lfoPhase[i] >= 1.0f) lfoPhase[i] -= 1.0f;
                while (lfoPhase[i] < 0.0f) lfoPhase[i] += 1.0f;
    
                lfoPhase[i] += modRate[i] * deltaTime;
                while (lfoPhase[i] >= 1.0f) lfoPhase[i] -= 1.0f;
                while (lfoPhase[i] < 0.0f) lfoPhase[i] += 1.0f;
    
                place[i] += modRate[i] * deltaTime;
                while (place[i] >= 1.0f) place[i] -= 1.0f;
                while (place[i] < 0.0f) place[i] += 1.0f;
    
                if (SINprocessCounter > SkipProcesses) {
                    lfoOutput[i] = 20.0f * sinf(2.0f * M_PI * lfoPhase[i]);
                    nextChunk[i] = lfoOutput[i] - currentOutput[i];
                }
    
                currentOutput[i] += nextChunk[i] * (1.0f / SkipProcesses);
                rawwonkMod[i] = currentOutput[i];
    
                wonkMod[i] = currentOutput[i] * modulationDepth * 0.8f;
                outputs[POLY_OUTPUT].setVoltage(wonkMod[i], i);
                outputs[_1_OUTPUT + i].setVoltage(wonkMod[i]);
    
            }//LFO layers
        }
             
        if (SINprocessCounter > SkipProcesses) { SINprocessCounter = 0; }
            
    }
};

struct WonkWidget : ModuleWidget {

    struct WonkDisplay : TransparentWidget {
        Wonk* module = nullptr;
        int index;        // Index from 0 to 5 for each rectangle
    
        void drawLayer(const DrawArgs& args, int layer) override {
            if (!module)
                return;
    
            if (layer == 1) { // Self-illuminating layer
                // Get the value from wonkMod array
                float value = module->wonkMod[index];
    
                // Clamp the value to -5V to +5V range
                value = clamp( value, -5.0f, 5.0f );
    
                // Determine the color based on the sign of the value
                NVGcolor color;
                if ( value >= 0.0f ) {
                    color = nvgRGBAf(1.0f, 0.4f, 0.0f, 1.0f); // Red for positive values
                } else {
                    color = nvgRGBAf(0.0f, 0.4f, 1.0f, 1.0f); // Blue for negative values
                }
    
                // Calculate the center X position and width scaling factor
                float centerX = box.size.x / 2.0f;
                float widthScale = centerX / 5.0f; // 5V corresponds to half the widget's width
    
                // Calculate the rectangle width based on the value
                float rectWidth = std::fabs(value) * widthScale;
    
                // Determine the X position of the rectangle
                float xPos;
                if (value >= 0.0f) {
                    xPos = centerX; // Positive values extend rightwards
                } else {
                    xPos = centerX - rectWidth; // Negative values extend leftwards
                }
    
                // Draw the rectangle with a subtle glow for backlighting
                nvgBeginPath(args.vg);
                nvgRect(args.vg, xPos, 0.0f, rectWidth, box.size.y*0.9f);
                nvgFillColor(args.vg, color);
                nvgFill(args.vg);
            }
        }    
    };
    
    WonkWidget(Wonk* module) {
        setModule(module);

        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Wonk.svg"),
            asset::plugin(pluginInstance, "res/Wonk-dark.svg")
        ));

        // Add screws or additional design elements as needed
        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(6.954, 14.562)), module, Wonk::CLOCK_INPUT));       
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(6.954 + 12, 14.562)), module, Wonk::RESET_INPUT));
                
        addParam(createParamCentered<TL1105>(mm2px(Vec(6.954+19, 14.562)), module, Wonk::RESET_BUTTON));
        addChild(createLightCentered<LargeLight<RedLight>>(mm2px(Vec(6.954+19, 14.562)), module, (Wonk::RESET_LIGHT)));

        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(37.443, 14.562)), module, Wonk::RATE_INPUT));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(46.31, 14.562)), module, Wonk::RATE_ATT));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(55.177, 14.562)), module, Wonk::RATE_KNOB));

        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(6.954, 95.717)), module, Wonk::WONK_INPUT));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(15.821, 95.717)), module, Wonk::WONK_ATT));        
        addParam(createParamCentered<RoundHugeBlackKnob>(mm2px(Vec(30.48, 94.926)), module, Wonk::WONK_KNOB));
        
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(48.512, 100.019)), module, Wonk::POS_KNOB));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(6.954, 113.958)), module, Wonk::NODES_INPUT));        
        addParam(createParamCentered<Trimpot>(mm2px(Vec(15.821, 113.958)), module, Wonk::NODES_ATT));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(24.689, 113.958)), module, Wonk::NODES_KNOB));

        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(37.443, 113.958)), module, Wonk::MOD_DEPTH_INPUT));        
        addParam(createParamCentered<Trimpot>(mm2px(Vec(46.31, 113.958)), module, Wonk::MOD_DEPTH_ATT));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(55.177, 113.958)), module, Wonk::MOD_DEPTH));

        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(48.512, 30.137)), module, Wonk::_1_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(48.512, 39.465)), module, Wonk::_2_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(48.512, 48.793)), module, Wonk::_3_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(48.512, 58.121)), module, Wonk::_4_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(48.512, 67.449)), module, Wonk::_5_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(48.512, 76.777)), module, Wonk::_6_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(48.512, 86.875)), module, Wonk::POLY_OUTPUT));

        if (module){
            // Coordinates for each widget
            Vec wonkPositions[6] = {
                mm2px(Vec(7.398f, 26.018f)),
                mm2px(Vec(7.398f, 35.293f)),
                mm2px(Vec(7.398f, 44.568f)),
                mm2px(Vec(7.398f, 53.843f)),
                mm2px(Vec(7.398f, 63.118f)),
                mm2px(Vec(7.398f, 72.393f))
            };

            // Size of each widget
            Vec widgetSize = mm2px(Vec(33.642f, 8.829f));

            for (int i = 0; i < 6; i++) {
                WonkDisplay* display = createWidget<WonkDisplay>(wonkPositions[i]);
                display->box.size = widgetSize;
                display->module = module;
                display->index = i;
                addChild(display);
            }
        }
    }  
};

Model* modelWonk = createModel<Wonk, WonkWidget>("Wonk");