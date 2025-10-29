////////////////////////////////////////////////////////////
//
//   Picus
//
//   written by Cody Geary
//   Copyright 2025, MIT License
//
//   Seven-stage clock divider and multipler trigger burst sequencer.
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
#include "digital_display.hpp"
using namespace rack;
#include <random>
#include <map>
#include <vector>

const int STAGES=7; 
const int PATTERNS=10;  

struct Picus : Module {
    enum ParamIds {
        //Clock Divider
        X1D_BUTTON, X1U_BUTTON, Y1D_BUTTON, Y1U_BUTTON,
        X2D_BUTTON, X2U_BUTTON, Y2D_BUTTON, Y2U_BUTTON,
        X3D_BUTTON, X3U_BUTTON, Y3D_BUTTON, Y3U_BUTTON,
        X4D_BUTTON, X4U_BUTTON, Y4D_BUTTON, Y4U_BUTTON,
        X5D_BUTTON, X5U_BUTTON, Y5D_BUTTON, Y5U_BUTTON,
        X6D_BUTTON, X6U_BUTTON, Y6D_BUTTON, Y6U_BUTTON,
        X7D_BUTTON, X7U_BUTTON, Y7D_BUTTON, Y7U_BUTTON,
        PAT_1_BUTTON, PAT_2_BUTTON, PAT_3_BUTTON, PAT_4_BUTTON, PAT_5_BUTTON,
        PAT_6_BUTTON, PAT_7_BUTTON, PAT_8_BUTTON, PAT_9_BUTTON, PAT_10_BUTTON,
        STAGE_1_BUTTON, STAGE_2_BUTTON, STAGE_3_BUTTON, STAGE_4_BUTTON,
        STAGE_5_BUTTON, STAGE_6_BUTTON, STAGE_7_BUTTON,
        PATTERN_KNOB, PATTERN_ATT, ON_SWITCH, RESET_BUTTON,
        NUM_PARAMS
    };
    enum InputIds {
        CLOCK_INPUT, RESET_INPUT, PATTERN_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        DON_OUTPUT, KA_OUTPUT, END_OUTPUT,
        NUM_OUTS
    };
    enum LightIds {
        PAT_1_BIG_LIGHT, PAT_2_BIG_LIGHT, PAT_3_BIG_LIGHT, PAT_4_BIG_LIGHT, PAT_5_BIG_LIGHT,
        PAT_6_BIG_LIGHT, PAT_7_BIG_LIGHT, PAT_8_BIG_LIGHT, PAT_9_BIG_LIGHT, PAT_10_BIG_LIGHT,

        PAT_1_MED_LIGHT, PAT_2_MED_LIGHT, PAT_3_MED_LIGHT, PAT_4_MED_LIGHT, PAT_5_MED_LIGHT,
        PAT_6_MED_LIGHT, PAT_7_MED_LIGHT, PAT_8_MED_LIGHT, PAT_9_MED_LIGHT, PAT_10_MED_LIGHT,

        PAT_1_SMALL_LIGHT, PAT_2_SMALL_LIGHT, PAT_3_SMALL_LIGHT, PAT_4_SMALL_LIGHT, PAT_5_SMALL_LIGHT,
        PAT_6_SMALL_LIGHT, PAT_7_SMALL_LIGHT, PAT_8_SMALL_LIGHT, PAT_9_SMALL_LIGHT, PAT_10_SMALL_LIGHT,

        STAGE_1A_LIGHT, STAGE_1B_LIGHT, STAGE_1C_LIGHT, STAGE_1D_LIGHT,
        STAGE_2A_LIGHT, STAGE_2B_LIGHT, STAGE_2C_LIGHT, STAGE_2D_LIGHT,
        STAGE_3A_LIGHT, STAGE_3B_LIGHT, STAGE_3C_LIGHT, STAGE_3D_LIGHT,
        STAGE_4A_LIGHT, STAGE_4B_LIGHT, STAGE_4C_LIGHT, STAGE_4D_LIGHT,
        STAGE_5A_LIGHT, STAGE_5B_LIGHT, STAGE_5C_LIGHT, STAGE_5D_LIGHT,
        STAGE_6A_LIGHT, STAGE_6B_LIGHT, STAGE_6C_LIGHT, STAGE_6D_LIGHT,
        STAGE_7A_LIGHT, STAGE_7B_LIGHT, STAGE_7C_LIGHT, STAGE_7D_LIGHT,

        STAGE_1_LIGHT, STAGE_2_LIGHT, STAGE_3_LIGHT,
        STAGE_4_LIGHT, STAGE_5_LIGHT, STAGE_6_LIGHT, STAGE_7_LIGHT,
        DON_LIGHT, KA_LIGHT, END_LIGHT,
        NUM_LIGHTS
    };

    dsp::SchmittTrigger clockTrigger, resetTrigger, resetButtonTrigger;
    dsp::SchmittTrigger xDownTriggers[STAGES], xUpTriggers[STAGES], yDownTriggers[STAGES], yUpTriggers[STAGES];
    dsp::SchmittTrigger patternTrigger[PATTERNS], stageTriggers[STAGES];
    int patternState[PATTERNS] = {0,0,0,0,0,0,0,0,0,0}; //0=don 1=ka 2=off
    int patternStages = PATTERNS; //total number of stages
    int patternIndex = 0; //stage in the pattern loop of 10 steps

    dsp::Timer syncTimer, beatTimer;

    bool syncPoint = false; //keep track of when the sync point is for linking the clocks correct
    float syncInterval = 1.0f;

    bool firstPulseReceived = false;
    bool firstSync = true;
    int currentStage = 0;
    int selectedStage = 0; //button-stage selector
    float multiply[STAGES] = {1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f};
    float divide[STAGES] = {1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f};
    bool resyncFlag[STAGES] = {false,false,false,false,false,false,false}; 
    int beatCount = 0;
    float beatInterval = 1.f;
    float playMode = 0.f;
    bool endPulseAtStage = true;
    bool patternReset = false;
    bool resetCondition = false;
    float lastPlayMode = 1.0f;
    bool blinkDON = false;
    bool blinkKA = false;
    bool blinkEND = false;

    int inputSkipper = 0;
    int inputSkipsTotal = 100; //only process button presses every 1/100 steps as it takes way too much CPU

    dsp::PulseGenerator DonPulse, KaPulse, EndPulse;

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
    
        json_t* patternJ = json_array();
        for (int i = 0; i < PATTERNS; i++) {
            json_array_append_new(patternJ, json_integer(patternState[i]));
        }
        json_object_set_new(rootJ, "patternState", patternJ);
    
        json_object_set_new(rootJ, "currentStage", json_integer(currentStage));
    
        json_object_set_new(rootJ, "endPulseAtStage", json_boolean(endPulseAtStage));
        json_object_set_new(rootJ, "patternReset", json_boolean(patternReset));
    
        json_object_set_new(rootJ, "selectedStage", json_integer(selectedStage));
    
        json_t* multiplyJ = json_array();
        for (int i = 0; i < STAGES; i++) {
            json_array_append_new(multiplyJ, json_real(multiply[i]));
        }
        json_object_set_new(rootJ, "multiply", multiplyJ);
    
        json_t* divideJ = json_array();
        for (int i = 0; i < STAGES; i++) {
            json_array_append_new(divideJ, json_real(divide[i]));
        }
        json_object_set_new(rootJ, "divide", divideJ);
    
        return rootJ;
    }
    
    void dataFromJson(json_t* rootJ) override {
        // Load patternState[10]
        json_t* patternJ = json_object_get(rootJ, "patternState");
        if (patternJ && json_is_array(patternJ)) {
            for (size_t i = 0; i < json_array_size(patternJ) && i < PATTERNS; i++) {
                json_t* val = json_array_get(patternJ, i);
                if (json_is_integer(val)) {
                    patternState[i] = json_integer_value(val);
                }
            }
        }
    
        json_t* curStageJ = json_object_get(rootJ, "currentStage");
        if (curStageJ && json_is_integer(curStageJ)) {
            currentStage = json_integer_value(curStageJ);
        }
    
        json_t* selStageJ = json_object_get(rootJ, "selectedStage");
        if (selStageJ && json_is_integer(selStageJ)) {
            selectedStage = json_integer_value(selStageJ);
        }
    
        json_t* endPulseAtStageJ = json_object_get(rootJ, "endPulseAtStage");
        if (endPulseAtStageJ) {
            endPulseAtStage = json_boolean_value(endPulseAtStageJ);
        }
    
        json_t* patternResetJ = json_object_get(rootJ, "patternReset");
        if (patternResetJ) {
            patternReset = json_boolean_value(patternResetJ);
        }
    
        json_t* multiplyJ = json_object_get(rootJ, "multiply");
        if (multiplyJ && json_is_array(multiplyJ)) {
            for (size_t i = 0; i < json_array_size(multiplyJ) && i < STAGES; i++) {
                json_t* val = json_array_get(multiplyJ, i);
                if (json_is_number(val)) {
                    multiply[i] = json_number_value(val);
                }
            }
        }
    
        // Load divide[7]
        json_t* divideJ = json_object_get(rootJ, "divide");
        if (divideJ && json_is_array(divideJ)) {
            for (size_t i = 0; i < json_array_size(divideJ) && i < STAGES; i++) {
                json_t* val = json_array_get(divideJ, i);
                if (json_is_number(val)) {
                    divide[i] = json_number_value(val);
                }
            }
        }
    }

    Picus() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTS, NUM_LIGHTS);

        for (int i=0; i<STAGES; i++){
            configParam(X1D_BUTTON+4*i, 0.f, 1.f, 0.f, "X Down Ch." + std::to_string(i+1) );
            configParam(X1U_BUTTON+4*i, 0.f, 1.f, 0.f, "X Up Ch." + std::to_string(i+1) );
            configParam(Y1D_BUTTON+4*i, 0.f, 1.f, 0.f, "Y Down Ch." + std::to_string(i+1) );
            configParam(Y1U_BUTTON+4*i, 0.f, 1.f, 0.f, "Y Up Ch." + std::to_string(i+1) );
        }
        configParam(RESET_BUTTON, 0.f, 1.f, 0.f, "Reset");

        for (int i = 0; i < PATTERNS; ++i) {
            configParam(PAT_1_BUTTON + i, 0.f, 1.f, 0.f, "Pulse Pattern " + std::to_string(i+1));
        }
        for (int i = 0; i < STAGES; ++i) {
            configParam(STAGE_1_BUTTON + i, 0.f, 1.f, 0.f, "Stage Select " + std::to_string(i+1));
        }

        configParam(PATTERN_KNOB, 0.f, 10.f, 5.f, "Pattern");
        configParam(PATTERN_ATT, -1.f, 1.f, 1.f, "Pattern Input Attenuator");

        configSwitch(ON_SWITCH, 0.0, 2.0, 1.0, "Play Mode", {"OFF", "ON", "ONE-SHOT"});
#if defined(METAMODULE)
        configInput(CLOCK_INPUT, "Clock Input");
#else
        configInput(CLOCK_INPUT, "Clock Input \n (Also accepts CHAIN link from Hammer) \n");
#endif
        configInput(RESET_INPUT, "Reset");
        configInput(PATTERN_INPUT, "Pattern Length");

        configOutput(DON_OUTPUT, "Don Drum Trigger");
        configOutput(KA_OUTPUT, "Ka Drum Trigger");
        configOutput(END_OUTPUT, "End of Stage/Sequence Trigger");

    }

   void onReset(const ResetEvent& e) override {
        // Reset all parameters
        Module::onReset(e);
    
        // Reset custom state variables
        for (int i = 0; i < STAGES; ++i) {
            multiply[i] = 1.0f;
            divide[i] = 1.0f;
        }
    
        for (int i = 0; i < PATTERNS; ++i) {
            patternState[i] = 0;
        }
    }
    
    void onRandomize(const RandomizeEvent& e) override {

        params[PATTERN_KNOB].setValue(PATTERNS*random::uniform());

        // Randomize custom state variables
        for (int i = 0; i < STAGES; ++i) {
            multiply[i] = random::uniform() * 12.0f;     // 0–32
            divide[i] = random::uniform() * 8.0f + 1.0;        // 1–9
        }
    
        for (int i = 0; i < PATTERNS; ++i) {
            patternState[i] = random::u32() % 3;         // 0, 1, or 2
        }
    } 

    void process(const ProcessArgs& args) override {

        //Process ON/OFF Switch
        playMode = params[ON_SWITCH].getValue();
        if (lastPlayMode == 0.f) lastPlayMode = 1.f;  //this mode can only be 1 or 2, but default is 1
        
        float deltaTime = args.sampleTime;
        syncTimer.process(deltaTime);
        beatTimer.process(deltaTime);

        // Clock handling logic
        syncPoint = false; // reset syncing flag
        bool externalClockConnected = inputs[CLOCK_INPUT].isConnected();
        if (externalClockConnected){
        
            float ClockInputVoltage = inputs[CLOCK_INPUT].getVoltage();

            if (abs(ClockInputVoltage - 10.42f) < 0.1f) { //RESET VOLTAGE for CVfunk Chain function
                resetCondition = true;
            } else { resetCondition = false;}
        
            if (clockTrigger.process(ClockInputVoltage - 0.1f) ) {
                if (abs(ClockInputVoltage - 10.69f) < 0.1f) {  //ON VOLTAGE for CVfunk Chain function
                    if (playMode>0.f){
                        lastPlayMode = playMode;
                        paramQuantities[ON_SWITCH]->setDisplayValue(playMode);
                    } else {
                        playMode = lastPlayMode;
                        paramQuantities[ON_SWITCH]->setDisplayValue(playMode);
                    }
                    return; // Don't process as normal clock
                }

                if (abs(ClockInputVoltage - 10.86f) < 0.1f) {  //OFF VOLTAGE for CVfunk Chain function
                    if (playMode>0.f){
                        lastPlayMode = playMode;  //if already playing save the current mode as the toggle
                        playMode = 0.f;  //turn OFF
                        paramQuantities[ON_SWITCH]->setDisplayValue(playMode);
                    } else {
                        playMode = 0.f;  //turn OFF
                        paramQuantities[ON_SWITCH]->setDisplayValue(playMode);
                    }
                    return; // Don't process as normal clock
                }
                
                if (firstPulseReceived) {
                    syncInterval = syncTimer.time;
                    syncTimer.reset();
                    syncPoint = true;
                    firstSync = false;
                }
                firstPulseReceived = true;
            }
        }

        // Process Pattern Knob
        patternStages = params[PATTERN_KNOB].getValue(); //initial value set by knob
        if (inputs[PATTERN_INPUT].isConnected()){
            patternStages += params[PATTERN_ATT].getValue()*inputs[PATTERN_INPUT].getVoltage();
        }
        patternStages = ceilf(patternStages); //round up
        patternStages = clamp(patternStages, 1, 10); //keep in strict bounds
        if (patternIndex >= patternStages) patternIndex = 0;

        // Process Ratio Buttons
        inputSkipper++;
        if (inputSkipper > inputSkipsTotal){ //Process button inputs infrequently to reduce CPU load. 
        
            for (int i = 0; i < 7; i++) {
                if (xDownTriggers[i].process(params[X1D_BUTTON + i * 4].getValue())) { multiply[i] -= 1.f; resyncFlag[i]=true;}
                if (xUpTriggers[i].process(params[X1U_BUTTON + i * 4].getValue()))   { multiply[i] += 1.f; resyncFlag[i]=true;}
                if (yDownTriggers[i].process(params[Y1D_BUTTON + i * 4].getValue())) { divide[i] -= 1.f; resyncFlag[i]=true;}
                if (yUpTriggers[i].process(params[Y1U_BUTTON + i * 4].getValue()))   { divide[i] += 1.f; resyncFlag[i]=true;}
                multiply[i] = clamp(multiply[i],0.0f,99.0f);
                divide[i] = clamp(divide[i],0.0f, 99.0f); // divide[i] can be zero! So we have to be careful to actually de-activate the stage when we set it to zero.
    
                //Process Stage Selection Buttons
                if (stageTriggers[i].process(params[STAGE_1_BUTTON + i].getValue())) selectedStage = i;
            }
            
            inputSkipper = 0;            
        }
        divide[0] = clamp(divide[0],1.f,99.f); // The top stage cannot be turned off, limited to 1 instead of 0.
                                              // if (divide[i]==0.f) the stage is OFF
                                              // if (multiply[i]==0.f) the stage is muted.

        // Handle Stage Selection Syncing Priority
        if (syncPoint && (currentStage != selectedStage) && playMode > 0.f){
            beatCount = 0;
            currentStage = selectedStage;
            beatTimer.reset(); // <-- Here Reset the Beat Timer
            syncPoint = false; //reset sync flag
            if (endPulseAtStage) EndPulse.trigger(0.001f);
            blinkEND = true;
            patternIndex++;
            if (patternReset) patternIndex=0;
            if (patternIndex >= patternStages) patternIndex = 0;
            if (patternState[patternIndex]==0) DonPulse.trigger(0.001f);
            if (patternState[patternIndex]==1) KaPulse.trigger(0.001f);
        }

        // Stage Advancing
        if (syncPoint && playMode > 0.f){
            beatCount++;
            if (firstSync) beatCount = 0; // Reset beat count on the first clock sync
            int stageLength = static_cast<int>(divide[currentStage]);
            if (beatCount >= stageLength ){
                beatCount = 0;
                currentStage++;
                selectedStage++;
                beatTimer.reset(); // <-- Reset the Beat Timer
                if (endPulseAtStage) {EndPulse.trigger(0.001f); blinkEND = true;}
                patternIndex++;
                if (patternReset) patternIndex=0;
                if (patternIndex >= patternStages) patternIndex = 0;
                if (patternState[patternIndex]==0) DonPulse.trigger(0.001f);
                if (patternState[patternIndex]==1) KaPulse.trigger(0.001f);

                // Advance to next active stage
                for (int i = 0; i < STAGES; i++) { 
                    if (currentStage >= STAGES) { //Stage Wrap-Around Point
                        currentStage = 0;
                        if (!endPulseAtStage) { EndPulse.trigger(0.001f); blinkEND = true; }                                           
                        if (playMode == 2.0) { //one-shot mode
                            paramQuantities[ON_SWITCH]->setDisplayValue(0.0f);
                            playMode = 0.f;
                            lastPlayMode = 2.0;
                        }
                    }
                    if (divide[currentStage] != 0) break;
                    currentStage++;
                }
                // Keep selectedStage in sync
                selectedStage = currentStage;
            }
        }

        // Beat Computing
        if (divide[currentStage]>0.f && multiply[currentStage]>0.f && (!firstSync) && playMode > 0.f){
            if (syncPoint || resyncFlag[currentStage]){
                resyncFlag[currentStage] = false;
                beatInterval = (divide[currentStage]*syncInterval)/multiply[currentStage];
            }
            if (beatTimer.time >= beatInterval && playMode > 0.f && externalClockConnected){
                beatTimer.reset();
                patternIndex++;
                if (patternIndex >= patternStages) patternIndex = 0;
                if (patternState[patternIndex]==0) DonPulse.trigger(0.001f);
                if (patternState[patternIndex]==1) KaPulse.trigger(0.001f);
            }
        }

        //Beat Outputs      
        bool DonActive = false;
        bool KaActive = false;
        bool EndActive = false;
        
        if (DonPulse.process(args.sampleTime)){ DonActive = true; blinkDON = true;}
        if (KaPulse.process(args.sampleTime)) { KaActive = true; blinkKA = true;}
        if (EndPulse.process(args.sampleTime)){ EndActive = true; blinkEND = true;}

        KaPulse.process(args.sampleTime);
        EndPulse.process(args.sampleTime);

        if (divide[currentStage]>0.f && multiply[currentStage]>0.f && playMode > 0.f ){
            outputs[DON_OUTPUT].setVoltage(DonActive ? 10.f : 0.f);
            outputs[KA_OUTPUT].setVoltage(KaActive ? 10.f : 0.f);
        }
        outputs[END_OUTPUT].setVoltage(EndActive ? 10.f : 0.f);

        // Handle Reset Input
        bool reset = resetButtonTrigger.process(params[RESET_BUTTON].getValue()); //initial value set by knob
        if (inputs[RESET_INPUT].isConnected()){
            if(resetTrigger.process(inputs[RESET_INPUT].getVoltage()-0.1f)) reset = true;
        }
        if (reset || resetCondition){
            currentStage = 0;
            selectedStage = 0;
            beatTimer.reset();
            firstPulseReceived = false;
            patternIndex = 0;
            EndPulse.trigger(0.001f);

            if (lastPlayMode == 2.0f){   
                if (playMode>0.f){
                    lastPlayMode = playMode;
                    paramQuantities[ON_SWITCH]->setDisplayValue(playMode);
                } else {
                    playMode = lastPlayMode;
                    paramQuantities[ON_SWITCH]->setDisplayValue(playMode);
                }
            }
        }

        // Handle Pattern Buttons
        for (int i = 0; i < PATTERNS; i++){
            if(patternTrigger[i].process( params[PAT_1_BUTTON+i].getValue())){
                patternState[i]++;
                if (patternState[i]>2) patternState[i]=0;
            }
            
        }

    }//end process
};

struct PicusWidget : ModuleWidget {
    DigitalDisplay* ratioDisplays[STAGES] = {nullptr};

    PicusWidget(Picus* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Picus.svg"),
            asset::plugin(pluginInstance, "res/Picus-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // Clock Divider Increments
        float xoffset = 5.5f;
        float yoffset = -14.2f;
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(14.974f+xoffset, 49.329f +yoffset)), module, (Picus::STAGE_1A_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(21.452f+xoffset, 49.329f +yoffset)), module, (Picus::STAGE_1B_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(43.533f+xoffset, 49.329f +yoffset)), module, (Picus::STAGE_1C_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(50.011f+xoffset, 49.329f +yoffset)), module, (Picus::STAGE_1D_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(14.974f+xoffset, 59.482f +yoffset)), module, (Picus::STAGE_2A_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(21.452f+xoffset, 59.482f +yoffset)), module, (Picus::STAGE_2B_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(43.533f+xoffset, 59.482f +yoffset)), module, (Picus::STAGE_2C_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(50.011f+xoffset, 59.482f +yoffset)), module, (Picus::STAGE_2D_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(14.974f+xoffset, 69.739f +yoffset)), module, (Picus::STAGE_3A_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(21.452f+xoffset, 69.739f +yoffset)), module, (Picus::STAGE_3B_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(43.533f+xoffset, 69.739f +yoffset)), module, (Picus::STAGE_3C_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(50.011f+xoffset, 69.739f +yoffset)), module, (Picus::STAGE_3D_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(14.974f+xoffset, 80.011f +yoffset)), module, (Picus::STAGE_4A_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(21.452f+xoffset, 80.011f +yoffset)), module, (Picus::STAGE_4B_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(43.533f+xoffset, 80.011f +yoffset)), module, (Picus::STAGE_4C_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(50.011f+xoffset, 80.011f +yoffset)), module, (Picus::STAGE_4D_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(14.974f+xoffset, 90.319f +yoffset)), module, (Picus::STAGE_5A_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(21.452f+xoffset, 90.319f +yoffset)), module, (Picus::STAGE_5B_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(43.533f+xoffset, 90.319f +yoffset)), module, (Picus::STAGE_5C_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(50.011f+xoffset, 90.319f +yoffset)), module, (Picus::STAGE_5D_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(14.974f+xoffset, 100.583f+yoffset)), module, (Picus::STAGE_6A_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(21.452f+xoffset, 100.583f+yoffset)), module, (Picus::STAGE_6B_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(43.533f+xoffset, 100.583f+yoffset)), module, (Picus::STAGE_6C_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(50.011f+xoffset, 100.583f+yoffset)), module, (Picus::STAGE_6D_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(14.974f+xoffset, 110.85f+yoffset)), module, (Picus::STAGE_7A_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(21.452f+xoffset, 110.85f+yoffset)), module, (Picus::STAGE_7B_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(43.533f+xoffset, 110.85f+yoffset)), module, (Picus::STAGE_7C_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(50.011f+xoffset, 110.85f+yoffset)), module, (Picus::STAGE_7D_LIGHT)));

        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(4.974f+xoffset, 49.329f +yoffset)), module, (Picus::STAGE_1_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(4.974f+xoffset, 59.482f +yoffset)), module, (Picus::STAGE_2_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(4.974f+xoffset, 69.739f +yoffset)), module, (Picus::STAGE_3_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(4.974f+xoffset, 80.011f +yoffset)), module, (Picus::STAGE_4_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(4.974f+xoffset, 90.319f +yoffset)), module, (Picus::STAGE_5_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(4.974f+xoffset, 100.583f+yoffset)), module, (Picus::STAGE_6_LIGHT)));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(4.974f+xoffset, 110.85f+yoffset)), module, (Picus::STAGE_7_LIGHT)));

        addParam(createParamCentered<TL1105>(mm2px(Vec(4.974f+xoffset, 49.329f +yoffset)), module, Picus::STAGE_1_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(4.974f+xoffset, 59.482f +yoffset)), module, Picus::STAGE_2_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(4.974f+xoffset, 69.739f +yoffset)), module, Picus::STAGE_3_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(4.974f+xoffset, 80.011f +yoffset)), module, Picus::STAGE_4_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(4.974f+xoffset, 90.319f +yoffset)), module, Picus::STAGE_5_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(4.974f+xoffset, 100.583f+yoffset)), module, Picus::STAGE_6_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(4.974f+xoffset, 110.85f+yoffset)), module, Picus::STAGE_7_BUTTON));

        addParam(createParamCentered<TL1105>(mm2px(Vec(14.974f+xoffset, 49.329f +yoffset)), module, Picus::X1D_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(21.452f+xoffset, 49.329f +yoffset)), module, Picus::X1U_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(43.533f+xoffset, 49.329f +yoffset)), module, Picus::Y1D_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(50.011f+xoffset, 49.329f +yoffset)), module, Picus::Y1U_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(14.974f+xoffset, 59.482f +yoffset)), module, Picus::X2D_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(21.452f+xoffset, 59.482f +yoffset)), module, Picus::X2U_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(43.533f+xoffset, 59.482f +yoffset)), module, Picus::Y2D_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(50.011f+xoffset, 59.482f +yoffset)), module, Picus::Y2U_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(14.974f+xoffset, 69.739f +yoffset)), module, Picus::X3D_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(21.452f+xoffset, 69.739f +yoffset)), module, Picus::X3U_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(43.533f+xoffset, 69.739f +yoffset)), module, Picus::Y3D_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(50.011f+xoffset, 69.739f +yoffset)), module, Picus::Y3U_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(14.974f+xoffset, 80.011f +yoffset)), module, Picus::X4D_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(21.452f+xoffset, 80.011f +yoffset)), module, Picus::X4U_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(43.533f+xoffset, 80.011f +yoffset)), module, Picus::Y4D_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(50.011f+xoffset, 80.011f +yoffset)), module, Picus::Y4U_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(14.974f+xoffset, 90.319f +yoffset)), module, Picus::X5D_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(21.452f+xoffset, 90.319f +yoffset)), module, Picus::X5U_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(43.533f+xoffset, 90.319f +yoffset)), module, Picus::Y5D_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(50.011f+xoffset, 90.319f +yoffset)), module, Picus::Y5U_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(14.974f+xoffset, 100.583f+yoffset)), module, Picus::X6D_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(21.452f+xoffset, 100.583f+yoffset)), module, Picus::X6U_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(43.533f+xoffset, 100.583f+yoffset)), module, Picus::Y6D_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(50.011f+xoffset, 100.583f+yoffset)), module, Picus::Y6U_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(14.974f+xoffset, 110.85f+yoffset)), module, Picus::X7D_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(21.452f+xoffset, 110.85f+yoffset)), module, Picus::X7U_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(43.533f+xoffset, 110.85f+yoffset)), module, Picus::Y7D_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(50.011f+xoffset, 110.85f+yoffset)), module, Picus::Y7U_BUTTON));

        addParam(createParamCentered<CKSSThreeHorizontal>(mm2px(Vec(7, 108)), module, Picus::ON_SWITCH));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7,  115)), module, Picus::CLOCK_INPUT));
        addParam(createParamCentered<TL1105>(mm2px(Vec(17,  108)), module, Picus::RESET_BUTTON));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(17,  115)), module, Picus::RESET_INPUT));

        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(32, 115)), module, Picus::DON_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(42, 115)), module, Picus::KA_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(52, 115)), module, Picus::END_OUTPUT));

        addChild(createLightCentered<SmallLight<WhiteLight>>(mm2px(Vec(32-4, 115-4)), module, (Picus::DON_LIGHT)));
        addChild(createLightCentered<SmallLight<BlueLight>>(mm2px(Vec(42-4, 115-4)), module, (Picus::KA_LIGHT)));
        addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(52-4, 115-4)), module, (Picus::END_LIGHT)));

        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(68,  115)), module, Picus::PATTERN_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(68, 95)), module, Picus::PATTERN_KNOB));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(68, 105)), module, Picus::PATTERN_ATT));

        // Ratio Displays Initialization
        for (int i = 0; i < 7; i++) {
            ratioDisplays[i] = createDigitalDisplay(mm2px(Vec(24.f+xoffset, 46.365f + (float)i * 10.386f + yoffset)), "3:4");
            addChild(ratioDisplays[i]);
        }

        float bufferSpace = 17.0f;
        for (int i=0; i<10; i++){
            addParam(createParamCentered<TL1105>(mm2px(Vec(10.f + (box.size.x-2*bufferSpace)*i/30, 16.5f)), module, Picus::PAT_1_BUTTON+i));
            addChild(createLightCentered<TinyLight<RedLight>>(mm2px(Vec(10.f + (box.size.x-2*bufferSpace)*i/30, 16.5f )), module, (Picus::PAT_1_BIG_LIGHT+i+20)));
            addChild(createLightCentered<SmallLight<BlueLight>>(mm2px(Vec(10.f + (box.size.x-2*bufferSpace)*i/30, 16.5f )), module, (Picus::PAT_1_BIG_LIGHT+i+10)));
            addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(10.f + (box.size.x-2*bufferSpace)*i/30, 16.5f )), module, (Picus::PAT_1_BIG_LIGHT+i)));
        }
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
 
        Picus* picusModule = dynamic_cast<Picus*>(module);
        assert(picusModule); // Ensure the cast succeeds

         // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());
     
        struct GateOutputMenuItem : MenuItem {
            Picus* picusModule;
            void onAction(const event::Action& e) override {
                picusModule->endPulseAtStage = !picusModule->endPulseAtStage;
            }
            void step() override {
                rightText = picusModule->endPulseAtStage ? "stage end ✔" : "sequence end ✔";
                MenuItem::step();
            }
        };
    
        GateOutputMenuItem* gateOutputItem = new GateOutputMenuItem();
        gateOutputItem->text = "END outputs pulse at ";
        gateOutputItem->picusModule = picusModule;
        menu->addChild(gateOutputItem);

        struct PatternResetMenuItem : MenuItem {
            Picus* picusModule;
            void onAction(const event::Action& e) override {
                picusModule->patternReset = !picusModule->patternReset;
            }
            void step() override {
                rightText = picusModule->patternReset ? "at stage end ✔" : "only upon reset ✔";
                MenuItem::step();
            }
        };
    
        PatternResetMenuItem* PatternResetItem = new PatternResetMenuItem();
        PatternResetItem->text = "Pattern resets ";
        PatternResetItem->picusModule = picusModule;
        menu->addChild(PatternResetItem);

    }

    void draw(const DrawArgs& args) override {
        ModuleWidget::draw(args);
        Picus* module = dynamic_cast<Picus*>(this->module);
        if (!module) return;

        // Update ratio displays
        for (int i = 0; i < 7; i++) {
            if (ratioDisplays[i]) {
                char ratioText[16];
                snprintf(ratioText, sizeof(ratioText), "%d:%d", static_cast<int>(module->multiply[i]), static_cast<int>(module->divide[i]));
                if (module->currentStage != i){
                    ratioDisplays[i]->fgColor = nvgRGB(154, 105, 65); // Dimmed text
                } else {
                    ratioDisplays[i]->fgColor = nvgRGB(208, 140, 89); // Gold color text
                }
                if (module->divide[i]!=0){
                    ratioDisplays[i]->text = ratioText;
                } else {
                    ratioDisplays[i]->text = "off";
                }
            }
        }
        
        // Handle LIGHTS
        for (int i = 0; i < 10; i++){
            if (i < module->patternStages){
                if (module->patternState[i] == 0){ //state=0 main
                    module->lights[Picus::PAT_1_BIG_LIGHT + i + 10].setBrightness( 0.f); //blue-off
                    module->lights[Picus::PAT_1_BIG_LIGHT + i + 20].setBrightness( 0.f); //red-off
                    if (i == module->patternIndex){
                        module->lights[Picus::PAT_1_BIG_LIGHT + i].setBrightness( 1.f); //large white(0-9)
                    } else {
                        module->lights[Picus::PAT_1_BIG_LIGHT + i].setBrightness( 0.25f);
                    }
                } else if (module->patternState[i] == 1){ //state=1 accent
                    module->lights[Picus::PAT_1_BIG_LIGHT + i].setBrightness( 0.f); //white-off
                    module->lights[Picus::PAT_1_BIG_LIGHT + i + 20].setBrightness( 0.f); //red-off
                    if (i == module->patternIndex){
                        module->lights[Picus::PAT_1_BIG_LIGHT + i + 10].setBrightness( 1.f);  //medium blue(10-19)
                    } else {
                        module->lights[Picus::PAT_1_BIG_LIGHT + i + 10].setBrightness( 0.25f);
                    }
                } else { //state=2 off
                    module->lights[Picus::PAT_1_BIG_LIGHT + i].setBrightness( 0.f); //white-off
                    module->lights[Picus::PAT_1_BIG_LIGHT + i + 10].setBrightness( 0.f); //blue-off
                    if (i == module->patternIndex){
                        module->lights[Picus::PAT_1_BIG_LIGHT + i + 20].setBrightness( 1.f); //small red(20-29)
                    } else {
                        module->lights[Picus::PAT_1_BIG_LIGHT + i + 20].setBrightness( 0.25f);
                    }
                }
            } else {
                    module->lights[Picus::PAT_1_BIG_LIGHT + i].setBrightness( 0.f); //off
                    module->lights[Picus::PAT_1_BIG_LIGHT + i + 10].setBrightness( 0.f);
                    module->lights[Picus::PAT_1_BIG_LIGHT + i + 20].setBrightness( 0.f);
            }
        }

        // Stage Lights
        for (int i = 0; i < 28; i++){ // blank out 7 stages * 4 buttons
            module->lights[Picus::STAGE_1A_LIGHT + i].setBrightness( 0.f);
        }
        for (int i = 0; i < 7; i++){ //7 stage lights
            module->lights[Picus::STAGE_1_LIGHT + i].setBrightness( 0.f); 
        }

        module->selectedStage = clamp(module->selectedStage, 0, 6); //clamp stage in case bad data loaded from JSON
        module->lights[Picus::STAGE_1_LIGHT + module->selectedStage].setBrightness( 1.0f); //illuminate selected stage.
        // Compute Current Stage
        module->lights[Picus::STAGE_1A_LIGHT + 4*module->currentStage].setBrightness( 0.3f); //illuminate current stage
        module->lights[Picus::STAGE_1B_LIGHT + 4*module->currentStage].setBrightness( 0.3f);
        module->lights[Picus::STAGE_1C_LIGHT + 4*module->currentStage].setBrightness( 0.3f);
        module->lights[Picus::STAGE_1D_LIGHT + 4*module->currentStage].setBrightness( 0.3f);        

        if (module->blinkDON){
            module->blinkDON= false; //reset blink token
            module->lights[Picus::DON_LIGHT].setBrightness( 1.f);
        } else {
            float dim = module->lights[Picus::DON_LIGHT].getBrightness();
            module->lights[Picus::DON_LIGHT].setBrightness( dim * .8f);
        }          

        if (module->blinkKA){
            module->blinkKA = false; //reset blink token
            module->lights[Picus::KA_LIGHT].setBrightness( 1.f);
        } else {
            float dim = module->lights[Picus::KA_LIGHT].getBrightness();
            module->lights[Picus::KA_LIGHT].setBrightness( dim * .8f);
        }          

        if (module->blinkEND){
            module->blinkEND = false; //reset blink token
            module->lights[Picus::END_LIGHT].setBrightness( 1.f);
        } else {
            float dim = module->lights[Picus::END_LIGHT].getBrightness();
            module->lights[Picus::END_LIGHT].setBrightness( dim * .8f);
        }          
    }  

    DigitalDisplay* createDigitalDisplay(Vec position, std::string initialValue) {
        DigitalDisplay* display = new DigitalDisplay();
        display->box.pos = position;
        display->box.size = Vec(50, 18);
        display->text = initialValue;
        display->fgColor = nvgRGB(208, 140, 89); // Gold color text
        display->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        display->setFontSize(14.0f);
        return display;
    }

};

Model* modelPicus = createModel<Picus, PicusWidget>("Picus");