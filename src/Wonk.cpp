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
#include "rack.hpp"

//Branchless replacement for fmod
inline __attribute__((always_inline)) float wrap01(float x) { return x - floorf(x); }
inline __attribute__((always_inline)) float wrapPhaseDiff(float x) {  return x - roundf(x); }
inline __attribute__((always_inline)) float linearInterpolate(float a, float b, float fraction) { return a + fraction * (b - a); }
inline __attribute__((always_inline)) float lagrange4(
        float y0, float y1, float y2, float y3, float t
    ) {
        float a = (-t * (t - 1.0f) * (t - 2.0f)) / 6.0f;
        float b = ((t + 1.0f) * (t - 1.0f) * (t - 2.0f)) / 2.0f;
        float c = (-(t + 1.0f) * t * (t - 2.0f)) / 2.0f;
        float d = ((t + 1.0f) * t * (t - 1.0f)) / 6.0f;
        return a * y0 + b * y1 + c * y2 + d * y3;
    }

struct Wonk : Module {

    enum ParamId {
        RATE_ATT, RATE_KNOB,
        WONK_ATT, WONK_KNOB,
        POS_KNOB,
        NODES_ATT, NODES_KNOB,
        MOD_DEPTH_ATT, MOD_DEPTH,
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
        _1_OUTPUT, _2_OUTPUT,
        _3_OUTPUT, _4_OUTPUT,
        _5_OUTPUT, _6_OUTPUT,
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
    float syncInterval = 2.0f;
    float prevSyncInterval = 2.0f;

    float freqHz = 0.5f; //frequency in Hz
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
    float place[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    float happy_place[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

    //For Interpolation
    float lfoOutput[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    float nextChunk[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    int SINprocessCounter = 0; // Counter to track process cycles
    int SkipProcesses = 4; //Number of process cycles to skip for the sine calculation
    float lfoHistory[6][4] = {}; // ring buffer of last 4 sine values per channel
    int lfoHistPos = 0; // 0 to 3 ring buffer position

    float wonkMod[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}; //store the scaled wonkMod output
    float rawwonkMod[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}; //store the raw value
    bool modClockSync = false; //Sync the modulation bus to the master clock instead of stage clock
    float modulationDepth = 5.0f;
    bool resetCondition = false;
    bool syncActive = false;
    dsp::PulseGenerator syncPulse;

    bool unipolarMode = false;


    // JSON Save/Load
    json_t* dataToJson() override {
        json_t* rootJ = json_object();
        json_object_set_new(rootJ, "syncInterval", json_real(syncInterval));

        // Save place array
        json_t* placeArrayJ = json_array();
        for (int i = 0; i < 6; i++) {
            json_array_append_new(placeArrayJ, json_real(place[i]));
        }
        json_object_set_new(rootJ, "place", placeArrayJ);

        // Save lfoPhase array
        json_t* lfoPhaseArrayJ = json_array();
        for (int i = 0; i < 6; i++) {
            json_array_append_new(lfoPhaseArrayJ, json_real(lfoPhase[i]));
        }
        json_object_set_new(rootJ, "lfoPhase", lfoPhaseArrayJ);

        json_object_set_new(rootJ, "unipolarMode", json_boolean(unipolarMode));

        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        json_t* siJ = json_object_get(rootJ, "syncInterval");
        if (siJ)
            syncInterval = (float)json_real_value(siJ);

        // Load place array
        json_t* placeArrayJ = json_object_get(rootJ, "place");
        if (placeArrayJ && json_is_array(placeArrayJ)) {
            size_t i;
            json_t* val;
            json_array_foreach(placeArrayJ, i, val) {
                if (i < 6)
                    place[i] = (float)json_real_value(val);
            }
        }

        // Load lfoPhase array
        json_t* lfoPhaseArrayJ = json_object_get(rootJ, "lfoPhase");
        if (lfoPhaseArrayJ && json_is_array(lfoPhaseArrayJ)) {
            size_t i;
            json_t* val;
            json_array_foreach(lfoPhaseArrayJ, i, val) {
                if (i < 6)
                    lfoPhase[i] = (float)json_real_value(val);
            }
        }

        json_t* unipolarModeJ = json_object_get(rootJ, "unipolarMode");
        if (unipolarModeJ) {
            unipolarMode = json_is_true(unipolarModeJ);
        }

    }

    Wonk() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(RATE_ATT, -1.f, 1.f, 0.f, "Rate Attenuverter");
        configParam(RESET_BUTTON, 0.f, 1.f, 0.f, "Reset Button");
        configParam(RATE_KNOB, -24.0f, 24.f, 1.f, "Rate multiplier (or divider for negative)");
        configParam(WONK_KNOB, 0.f, 1.f, 0.f, "Wonk Intensity");
        configParam(WONK_ATT, -1.f, 1.f, 0.f, "Wonk Input Attenuverter");
        configParam(POS_KNOB, 1.f, 6.f, 1.f, "Wonk Feedback Position")->snapEnabled=true;
        configParam(NODES_ATT, -1.f, 1.f, 0.f, "Nodes Attenuverter");
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

        outputs[POLY_OUTPUT].setChannels(6);
    }

    void onReset(const ResetEvent& e) override {
        // Reset all parameters
        Module::onReset(e);

        syncInterval = 2.0f;
        prevSyncInterval = 2.0f;

        for (int i=0; i<6; i++){
                place[i] = 0.0f; //Reset place to zero
                lfoPhase[i] = i/6.f; //Reset phase to computed node positions
        }
    }

    void process(const ProcessArgs& args) override {
    
        float deltaTime = args.sampleTime;
        syncTimer.process(deltaTime);
    
        resetCondition = false; //remote reset token
        syncPoint = false; //internal reset-sync token
    
        // Clock handling logic
        bool externalClockConnected = inputs[CLOCK_INPUT].isConnected();
        if (externalClockConnected ) {
            float SyncInputVoltage = inputs[CLOCK_INPUT].getVoltage();
    
            if (abs(SyncInputVoltage - 10.42f) < 0.1f) { //RESET VOLTAGE for CVfunk Chain function
                syncPoint = true;
                syncInterval = prevSyncInterval;
                firstSync = true;
                firstPulseReceived = false;
                resetCondition = true;
            }
    
            if (clockTrigger.process(SyncInputVoltage - 0.1f)) {
    
                // Check for special control voltages first
                if (abs(SyncInputVoltage - 10.69f) < 0.1f) {  //ON VOLTAGE for CVfunk Chain function
                    syncPoint = true;
                    syncInterval = prevSyncInterval;
                    firstSync = true;
                    firstPulseReceived = false;
                    return; // Don't process as normal clock
                }
                if (abs(SyncInputVoltage - 10.86f) < 0.1f) {  //OFF VOLTAGE for CVfunk Chain function
                    syncInterval = prevSyncInterval;
                    firstSync = true;
                    firstPulseReceived = false;
                    return; // Don't process as normal clock
                }
    
                // Compute SYNC INTERVAL here
                if (firstPulseReceived) {
                    prevSyncInterval = syncInterval;  //save interval before overwriting it
                    syncInterval = syncTimer.time;
                    syncTimer.reset();
                    firstSync = false;
                }
                firstPulseReceived = true;
            }
        }
    
        freqHz = 1.0f/fmax(syncInterval, 0.0001f); //limit syncInterval to avoid div by zero.
    
        // Resetting Logic
        bool resetConnected = inputs[RESET_INPUT].isConnected();
        if (resetConnected && resetTrigger.process(inputs[RESET_INPUT].getVoltage() - 0.1f)) syncPoint = true; //Sync on Reset Trigger
        if (resetButton.process(params[RESET_BUTTON].getValue())) syncPoint = true; //Sync on Reset Button Press
        if (resetCondition) syncPoint = true; //Reset on chain reset signal
    
        if (syncPoint) syncPulse.trigger(0.2f);
        syncActive = syncPulse.process(args.sampleTime);
    
        // Compute Modulation Depth
        modulationDepth = params[MOD_DEPTH].getValue();
        if (inputs[MOD_DEPTH_INPUT].isConnected()){
            modulationDepth = clamp (
                inputs[MOD_DEPTH_INPUT].getVoltage() * params[MOD_DEPTH_ATT].getValue() * 0.5f + modulationDepth, -5.f, 5.f); //map 0-10V to 5V.
        }
    
        processSkipper++;
        if (processSkipper>=processSkips){
            bool cableConnected = outputs[POLY_OUTPUT].isConnected();
            if (cableConnected) {
                outputs[POLY_OUTPUT].setChannels(6);
            }
            prevcableConnected = cableConnected;
            processSkipper = 0;
        }
    
        SINprocessCounter++;
    
        // Rate
        float rawRate = params[RATE_KNOB].getValue();
        if (inputs[RATE_INPUT].isConnected()) {
            rawRate = inputs[RATE_INPUT].getVoltage() * params[RATE_ATT].getValue() + rawRate;
        }
    
        float rate = 1.0f;
        if (rawRate >= 1.0f) rate = rawRate;
        else if (rawRate <= -1.0f) rate = 1.0f / std::abs(rawRate);
        rate *= freqHz * 0.5f; // Hz
    
        // Wonky / Node position
        float wonky = params[WONK_KNOB].getValue();
        if (inputs[WONK_INPUT].isConnected()){
            wonky = clamp(inputs[WONK_INPUT].getVoltage()*params[WONK_ATT].getValue() / 10.f + wonky, 0.f, 1.f);
        }
        const int wonkPos = static_cast<int>( clamp( roundf(params[POS_KNOB].getValue() - 0.5f), 0.0f, 5.0f ) );
    
        float nodePosition = params[NODES_KNOB].getValue();
        if (inputs[NODES_INPUT].isConnected()){
            nodePosition = nodePosition + inputs[NODES_INPUT].getVoltage() * params[NODES_ATT].getValue();
        }
    
        // Cached scalars
        const float nodePositionFactor = nodePosition - nodePosition * (wonky * 0.9f);
        const float wonkyScale = wonky * 0.95f / 5.0f;
        const float rateScaledDT = rate * deltaTime;
        const float twoPi = 2.0f * M_PI;
        const float wonkModScale = modulationDepth * 0.2f;
        const bool polyConnected = outputs[POLY_OUTPUT].isConnected();
        const bool unipolar = unipolarMode;
        // -----------------------------------------------
    
        for (int i = 0; i < 6; i++) {
            int adjWonkPos = wonkPos + i;
            if (adjWonkPos >= 6) adjWonkPos -= 6;
            if (adjWonkPos < 0) adjWonkPos += 6;
    
            float modRate = rate + rate * wonkyScale * wonkMod[adjWonkPos];
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

            if (syncPoint ){ // SYNC
                place[i] = 0.0f; //Reset place to zero
                lfoPhase[i] = targetPhase; //Reset phase to computed node positions
            }

            targetPhase += place[i];

            targetPhase = wrap01(targetPhase + place[i]); //more efficient wrap logic wrap01 0...1  and wrapPhaseDiff -0.5...0.5

            float phaseDiff = wrapPhaseDiff(targetPhase - lfoPhase[i]);

            lfoPhase[i] = wrap01(lfoPhase[i] + phaseDiff * 0.2f);
            lfoPhase[i] = wrap01(lfoPhase[i] + modRate * deltaTime);
            place[i]    = wrap01(place[i]    + modRate * deltaTime);

            if (SINprocessCounter > SkipProcesses) {
                // Compute new sine, store in ring buffer
                for (int i = 0; i < 6; ++i) {
                    float newSin = 5.0f * sinf(2.0f * M_PI * lfoPhase[i]); //Real sine is important for feedback circuit
                    lfoHistory[i][lfoHistPos] = newSin;
                }
                SINprocessCounter = 0;
                lfoHistPos = (lfoHistPos + 1) % 4; //Store the LFO history for interpolation use. Increment and rotate vals of 0...3
            }

            float t = static_cast<float>(SINprocessCounter) / static_cast<float>(SkipProcesses);

            // Recall History for Lagrange interpolation
            float y0 = lfoHistory[i][(lfoHistPos + 0) % 4]; // Ring buffer of 4
            float y1 = lfoHistory[i][(lfoHistPos + 1) % 4];
            float y2 = lfoHistory[i][(lfoHistPos + 2) % 4];
            float y3 = lfoHistory[i][(lfoHistPos + 3) % 4];

            float interpolated = lagrange4(y0, y1, y2, y3, t); // Smooth interpolation

            wonkMod[i] = interpolated * (modulationDepth * 0.2f); // modDepth can be up to 5V, normalize to 1 here

            float outputVal = unipolarMode ? wonkMod[i] + modulationDepth : wonkMod[i];

            if (outputs[POLY_OUTPUT].isConnected()) {
                outputs[POLY_OUTPUT].setVoltage(outputVal, i);
            }

            outputs[_1_OUTPUT + i].setVoltage(outputVal);

        }//LFO layers

        if (SINprocessCounter > SkipProcesses) { SINprocessCounter = 0; }

    }

};

struct WonkWidget : ModuleWidget {

    struct WonkDisplay : TransparentWidget {
        Wonk* module = nullptr;
        int index;        // Index from 0 to 15 for each rectangle

        void drawLayer(const DrawArgs& args, int layer) override {
            if (layer == 1) { // Self-illuminating layer
                float fake[6] = {-4.f, 3.f, -1.f, 5.f, 2.f, 4.f};
                float value = fake[index]; // fake data

                bool unipolar = module && module->unipolarMode;

                if (module) {
                    value = unipolar ? module->wonkMod[index] + module->modulationDepth : module->wonkMod[index];
                }

                NVGcolor color;
                if (unipolar) {
                    color = nvgRGB(208, 140, 89); // gold
                } else {
                    color = (value >= 0.0f)
                        ? nvgRGBAf(1.0f, 0.4f, 0.0f, 1.0f)   // orange for positive
                        : nvgRGBAf(0.0f, 0.4f, 1.0f, 1.0f);  // blue for negative
                }

                float rectWidth;
                float xPos;

                if (unipolar) {
                    // Map 0–10V across full width
                    float widthScale = box.size.x / 10.f;
                    rectWidth = value * widthScale;
                    xPos = 0.f; // start at left edge
                } else {
                    // Map -5–+5V across split center
                    float centerX = box.size.x / 2.0f;
                    float widthScale = centerX / 5.0f;
                    rectWidth = std::fabs(value) * widthScale;

                    if (value >= 0.0f)
                        xPos = centerX;            // rightwards
                    else
                        xPos = centerX - rectWidth; // leftwards
                }

                // Draw the rectangle
                nvgBeginPath(args.vg);
                nvgRect(args.vg, xPos, 0.0f, rectWidth, box.size.y * 0.9f);
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

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);

        Wonk* wonkModule = dynamic_cast<Wonk*>(module);
        assert(wonkModule);

        // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());

        // Unipolar mode menu item (toggle)
        struct UnipolarItem : MenuItem {
            Wonk* module;
            void onAction(const event::Action& e) override {
                module->unipolarMode = !module->unipolarMode;
            }
            void step() override {
                rightText = module->unipolarMode ? "✔" : "";
                MenuItem::step();
            }
        };

        UnipolarItem* unipolarItem = new UnipolarItem();
        unipolarItem->text = "Unipolar mode (0-10V)";
        unipolarItem->module = wonkModule;
        menu->addChild(unipolarItem);

    }

#if defined(METAMODULE)
    // For MM, use step(), because overriding draw() will allocate a module-sized pixel buffer
    void step() override {
#else
    void draw(const DrawArgs& args) override {
        ModuleWidget::draw(args);
#endif
        Wonk* module = dynamic_cast<Wonk*>(this->module);
        if (!module) return;

        if (module->syncActive) {
            module->lights[Wonk::RESET_LIGHT].setBrightness(1.0f);
        } else {
            module->lights[Wonk::RESET_LIGHT].setBrightness(0.f);
        }

        if (module->unipolarMode){
            module->paramQuantities[Wonk::MOD_DEPTH]->displayMultiplier = 2.f;
            } else {
            module->paramQuantities[Wonk::MOD_DEPTH]->displayMultiplier = 1.f;
        }
    }
};

Model* modelWonk = createModel<Wonk, WonkWidget>("Wonk");