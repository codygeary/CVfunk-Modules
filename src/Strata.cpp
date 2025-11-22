////////////////////////////////////////////////////////////
//
//   Strata
//
//   written by Cody Geary
//   Copyright 2025, MIT License
//
//   A layered step sequencer with rhythmic generator
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
#include "digital_display.hpp"
#include <array>
#include <string>
#include <vector>
#include <cmath>

using namespace rack;
#include <random>
#include <map>

const int STAGES=8; 
const int PATTERNS=24;  

static inline int randomInt(int minVal, int maxVal) {
    return minVal + (random::u32() % (maxVal - minVal + 1));
}

struct Strata : Module {
    // Parameters
    enum ParamId {
    
        //Knobs with save state
        SEQ_1_KNOB, SEQ_2_KNOB, SEQ_3_KNOB, SEQ_4_KNOB, 
        SEQ_5_KNOB, SEQ_6_KNOB, SEQ_7_KNOB, SEQ_8_KNOB, 
        SEMI_1_KNOB, SEMI_2_KNOB, SEMI_3_KNOB, SEMI_4_KNOB, 
        SEMI_5_KNOB, SEMI_6_KNOB, SEMI_7_KNOB,
        OCT_1_KNOB, OCT_2_KNOB, OCT_3_KNOB, OCT_4_KNOB, 

        //Buttons with save state
        SEQ_1_BUTTON, SEQ_2_BUTTON, SEQ_3_BUTTON, SEQ_4_BUTTON, 
        SEQ_5_BUTTON, SEQ_6_BUTTON, SEQ_7_BUTTON, SEQ_8_BUTTON, 
        SEMI_1_BUTTON, SEMI_2_BUTTON, SEMI_3_BUTTON, SEMI_4_BUTTON, 
        SEMI_5_BUTTON, SEMI_6_BUTTON, SEMI_7_BUTTON,
        OCT_1_BUTTON, OCT_2_BUTTON, OCT_3_BUTTON, OCT_4_BUTTON, 

        //Beat Buttons
        STAGE_1_BEATS_UP, STAGE_2_BEATS_UP, STAGE_3_BEATS_UP, STAGE_4_BEATS_UP, 
        STAGE_5_BEATS_UP, STAGE_6_BEATS_UP, STAGE_7_BEATS_UP, STAGE_8_BEATS_UP,
        SEMI_BEATS_UP, OCT_BEATS_UP, 

        STAGE_1_BEATS_DOWN, STAGE_2_BEATS_DOWN, STAGE_3_BEATS_DOWN, STAGE_4_BEATS_DOWN, 
        STAGE_5_BEATS_DOWN, STAGE_6_BEATS_DOWN, STAGE_7_BEATS_DOWN, STAGE_8_BEATS_DOWN, 
        SEMI_BEATS_DOWN, OCT_BEATS_DOWN, 

        STAGE_1_STEPS_UP, STAGE_2_STEPS_UP, STAGE_3_STEPS_UP, STAGE_4_STEPS_UP, 
        STAGE_5_STEPS_UP, STAGE_6_STEPS_UP, STAGE_7_STEPS_UP, STAGE_8_STEPS_UP, 
        SEMI_STEPS_UP, OCT_STEPS_UP,

        STAGE_1_STEPS_DOWN, STAGE_2_STEPS_DOWN, STAGE_3_STEPS_DOWN, STAGE_4_STEPS_DOWN, 
        STAGE_5_STEPS_DOWN, STAGE_6_STEPS_DOWN, STAGE_7_STEPS_DOWN, STAGE_8_STEPS_DOWN, 
        SEMI_STEPS_DOWN, OCT_STEPS_DOWN,

        //Pattern Buttons        
        PATTERN_1_BUTTON, PATTERN_2_BUTTON, PATTERN_3_BUTTON, PATTERN_4_BUTTON, 
        PATTERN_5_BUTTON, PATTERN_6_BUTTON, PATTERN_7_BUTTON, PATTERN_8_BUTTON, 
        PATTERN_9_BUTTON, PATTERN_10_BUTTON, PATTERN_11_BUTTON, PATTERN_12_BUTTON, 
        PATTERN_13_BUTTON, PATTERN_14_BUTTON, PATTERN_15_BUTTON, PATTERN_16_BUTTON, 
        PATTERN_17_BUTTON, PATTERN_18_BUTTON, PATTERN_19_BUTTON, PATTERN_20_BUTTON, 
        PATTERN_21_BUTTON, PATTERN_22_BUTTON, PATTERN_23_BUTTON, PATTERN_24_BUTTON, 

        PATTERN_KNOB,
        MAIN_SWITCH, SEMI_SWITCH, OCT_SWITCH,
        RESET_BUTTON, ON_SWITCH,
        LAYER_1_BUTTON, LAYER_2_BUTTON, LAYER_3_BUTTON, LAYER_4_BUTTON,
        LAYER_NEXT_BUTTON,
                
        OFFSET_PARAM,                      
        PARAMS_LEN
    };

    // Inputs
    enum InputId {
        CLOCK_INPUT, RESET_INPUT,
        OFFSET_INPUT, LAYER_INPUT, PATTERN_INPUT,        
        INPUTS_LEN
    };

    // Outputs
    enum OutputId {
        GATE_OUTPUT, INV_GATE_OUTPUT, MAIN_OUTPUT,
        OUTPUTS_LEN
    };
    enum class PasteMode {
        KnobsAndBeats,  // copy knobs + beats
        BeatsOnly,      // copy beats only
        KnobsOnly       // copy knobs only
    };
    
    PasteMode copyMode = PasteMode::KnobsAndBeats; // default

    // Lights
    enum LightId {

        SEQ_1_LIGHT_R, SEQ_2_LIGHT_R, SEQ_3_LIGHT_R, SEQ_4_LIGHT_R, 
        SEQ_5_LIGHT_R, SEQ_6_LIGHT_R, SEQ_7_LIGHT_R, SEQ_8_LIGHT_R, 
        SEMI_1_LIGHT_R, SEMI_2_LIGHT_R, SEMI_3_LIGHT_R, SEMI_4_LIGHT_R, 
        SEMI_5_LIGHT_R, SEMI_6_LIGHT_R, SEMI_7_LIGHT_R,
        OCT_1_LIGHT_R, OCT_2_LIGHT_R, OCT_3_LIGHT_R, OCT_4_LIGHT_R, 

        SEQ_1_LIGHT_G, SEQ_2_LIGHT_G, SEQ_3_LIGHT_G, SEQ_4_LIGHT_G, 
        SEQ_5_LIGHT_G, SEQ_6_LIGHT_G, SEQ_7_LIGHT_G, SEQ_8_LIGHT_G, 
        SEMI_1_LIGHT_G, SEMI_2_LIGHT_G, SEMI_3_LIGHT_G, SEMI_4_LIGHT_G, 
        SEMI_5_LIGHT_G, SEMI_6_LIGHT_G, SEMI_7_LIGHT_G,
        OCT_1_LIGHT_G, OCT_2_LIGHT_G, OCT_3_LIGHT_G, OCT_4_LIGHT_G, 

        SEQ_1_LIGHT_B, SEQ_2_LIGHT_B, SEQ_3_LIGHT_B, SEQ_4_LIGHT_B, 
        SEQ_5_LIGHT_B, SEQ_6_LIGHT_B, SEQ_7_LIGHT_B, SEQ_8_LIGHT_B, 
        SEMI_1_LIGHT_B, SEMI_2_LIGHT_B, SEMI_3_LIGHT_B, SEMI_4_LIGHT_B, 
        SEMI_5_LIGHT_B, SEMI_6_LIGHT_B, SEMI_7_LIGHT_B,
        OCT_1_LIGHT_B, OCT_2_LIGHT_B, OCT_3_LIGHT_B, OCT_4_LIGHT_B, 

        SEQ_1_LIGHT_Y, SEQ_2_LIGHT_Y, SEQ_3_LIGHT_Y, SEQ_4_LIGHT_Y, 
        SEQ_5_LIGHT_Y, SEQ_6_LIGHT_Y, SEQ_7_LIGHT_Y, SEQ_8_LIGHT_Y, 
        SEMI_1_LIGHT_Y, SEMI_2_LIGHT_Y, SEMI_3_LIGHT_Y, SEMI_4_LIGHT_Y, 
        SEMI_5_LIGHT_Y, SEMI_6_LIGHT_Y, SEMI_7_LIGHT_Y,
        OCT_1_LIGHT_Y, OCT_2_LIGHT_Y, OCT_3_LIGHT_Y, OCT_4_LIGHT_Y, 

        SEQ_1_LIGHT_W, SEQ_2_LIGHT_W, SEQ_3_LIGHT_W, SEQ_4_LIGHT_W, 
        SEQ_5_LIGHT_W, SEQ_6_LIGHT_W, SEQ_7_LIGHT_W, SEQ_8_LIGHT_W, 
        SEMI_1_LIGHT_W, SEMI_2_LIGHT_W, SEMI_3_LIGHT_W, SEMI_4_LIGHT_W, 
        SEMI_5_LIGHT_W, SEMI_6_LIGHT_W, SEMI_7_LIGHT_W,
        OCT_1_LIGHT_W, OCT_2_LIGHT_W, OCT_3_LIGHT_W, OCT_4_LIGHT_W, 

        STAGE_1_LIGHT, STAGE_2_LIGHT, STAGE_3_LIGHT, STAGE_4_LIGHT, 
        STAGE_5_LIGHT, STAGE_6_LIGHT, STAGE_7_LIGHT, STAGE_8_LIGHT, 

        PATTERN_1_LIGHT_Y, PATTERN_2_LIGHT_Y, PATTERN_3_LIGHT_Y, PATTERN_4_LIGHT_Y, 
        PATTERN_5_LIGHT_Y, PATTERN_6_LIGHT_Y, PATTERN_7_LIGHT_Y, PATTERN_8_LIGHT_Y, 
        PATTERN_9_LIGHT_Y, PATTERN_10_LIGHT_Y, PATTERN_11_LIGHT_Y, PATTERN_12_LIGHT_Y, 
        PATTERN_13_LIGHT_Y, PATTERN_14_LIGHT_Y, PATTERN_15_LIGHT_Y, PATTERN_16_LIGHT_Y, 
        PATTERN_17_LIGHT_Y, PATTERN_18_LIGHT_Y, PATTERN_19_LIGHT_Y, PATTERN_20_LIGHT_Y, 
        PATTERN_21_LIGHT_Y, PATTERN_22_LIGHT_Y, PATTERN_23_LIGHT_Y, PATTERN_24_LIGHT_Y, 

        PATTERN_1_LIGHT_B, PATTERN_2_LIGHT_B, PATTERN_3_LIGHT_B, PATTERN_4_LIGHT_B, 
        PATTERN_5_LIGHT_B, PATTERN_6_LIGHT_B, PATTERN_7_LIGHT_B, PATTERN_8_LIGHT_B, 
        PATTERN_9_LIGHT_B, PATTERN_10_LIGHT_B, PATTERN_11_LIGHT_B, PATTERN_12_LIGHT_B, 
        PATTERN_13_LIGHT_B, PATTERN_14_LIGHT_B, PATTERN_15_LIGHT_B, PATTERN_16_LIGHT_B, 
        PATTERN_17_LIGHT_B, PATTERN_18_LIGHT_B, PATTERN_19_LIGHT_B, PATTERN_20_LIGHT_B, 
        PATTERN_21_LIGHT_B, PATTERN_22_LIGHT_B, PATTERN_23_LIGHT_B, PATTERN_24_LIGHT_B, 

        PATTERN_1_LIGHT_W, PATTERN_2_LIGHT_W, PATTERN_3_LIGHT_W, PATTERN_4_LIGHT_W, 
        PATTERN_5_LIGHT_W, PATTERN_6_LIGHT_W, PATTERN_7_LIGHT_W, PATTERN_8_LIGHT_W, 
        PATTERN_9_LIGHT_W, PATTERN_10_LIGHT_W, PATTERN_11_LIGHT_W, PATTERN_12_LIGHT_W, 
        PATTERN_13_LIGHT_W, PATTERN_14_LIGHT_W, PATTERN_15_LIGHT_W, PATTERN_16_LIGHT_W, 
        PATTERN_17_LIGHT_W, PATTERN_18_LIGHT_W, PATTERN_19_LIGHT_W, PATTERN_20_LIGHT_W, 
        PATTERN_21_LIGHT_W, PATTERN_22_LIGHT_W, PATTERN_23_LIGHT_W, PATTERN_24_LIGHT_W, 

        LAYER_1_LIGHT, LAYER_2_LIGHT, LAYER_3_LIGHT, LAYER_4_LIGHT,
        INV_LIGHT, GATE_LIGHT,
        
        LIGHTS_LEN
    };

    dsp::SchmittTrigger clockTrigger, resetTrigger, resetButtonTrigger, layerTrigger[6];
    dsp::SchmittTrigger xDownTriggers[STAGES+2], xUpTriggers[STAGES+2], yDownTriggers[STAGES+2], yUpTriggers[STAGES+2];
    dsp::SchmittTrigger patternTrigger[PATTERNS];
    int patternState[PATTERNS][4] = {
        {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
        {1,1,1,1}, {1,1,1,1}, {1,1,1,1}, {1,1,1,1},
        {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
        {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
        {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
        {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}
    };
    int copiedPatternState[PATTERNS] = {0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    float patternKnob[4] = {8.f, 8.f, 8.f, 8.f};
    float copiedPatternKnob = 8.f;
    int patternStages = PATTERNS; //total number of stages
    int patternIndex = 0; //stage in the pattern loop of 10 steps

    dsp::Timer syncTimer, beatTimer, beatTimer_semi, beatTimer_oct;

    bool syncPoint = false; //keep track of when the sync point is for linking the clocks correct
    float syncInterval = 1.0f;

    bool firstPulseReceived = false;
    bool firstSync = true;
    int currentStage = 0;
    int selectedStage = 0; //button-stage selector
    float multiply[10][4] = {{1.f,1.f,1.f,1.f},{1.f,1.f,1.f,1.f},{1.f,1.f,1.f,1.f},{1.f,1.f,1.f,1.f},
                                {1.f,1.f,1.f,1.f},{1.f,1.f,1.f,1.f},{1.f,1.f,1.f,1.f},{1.f,1.f,1.f,1.f},
                                {1.f,1.f,1.f,1.f},{1.f,1.f,1.f,1.f}};
    float divide[10][4] = {{1.f,1.f,1.f,1.f},{1.f,1.f,1.f,1.f},{1.f,1.f,1.f,1.f},{1.f,1.f,1.f,1.f},
                                {1.f,1.f,1.f,1.f},{1.f,1.f,1.f,1.f},{1.f,1.f,1.f,1.f},{1.f,1.f,1.f,1.f},
                                {1.f,1.f,1.f,1.f},{1.f,1.f,1.f,1.f}};

    float copiedMultiply[10] = {1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f};
    float copiedDivide[10] = {1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f};

    bool resyncFlag[10] = {false,false,false,false,false,false,false,false,false,false}; 
    int beatCount = 0;
    int beatCountSemi = 0;
    int beatCountOct = 0;
    float beatInterval = 1.f;
    float beatInterval_semi = 1.f;
    float beatInterval_oct = 1.f;
    
    bool endPulseAtStage = true;
    bool patternReset = false;
    bool resetCondition = false;
    bool blinkDON = false;
    bool blinkKA = false;
    bool blinkEND = false;
    int subBeatCount = 0;
    int subBeatCount_semi = 0;
    int subBeatCount_oct = 0;

    int inputSkipper = 0;
    int inputSkipsTotal = 100; //only process button presses every 1/100 steps as it takes way too much CPU
    float playMode = 0.f;
    float lastPlayMode = 1.0f;
    bool resetArmed = false;

    dsp::PulseGenerator DonPulse, KaPulse, EndPulse;

    // Note Sequencer Handling   
    bool noteSampled = false;
    int leftStage = 0;
    int rightStage = 0;
    int semiStage = 0;
    int octStage =  0;        
    int strataLayer = 0;
    int previousStrataLayer = 0;
    int activeStage = 0;
    
    // Hold Note input signals
    float currentNote = 0.f;
    float currentSemi = 0.f;
    float currentOffset = 0.f;
    float currentOct = 0.f;
    float currentOutput = 0.f;

    float knobStates[19][4] = {{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},
                                {0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},
                                {0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},
                                {0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},
                                {0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f}};
    float switchStates[3][4] = {{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f}};
    bool buttonStates[19][4] = {{true, true, true, true},{true, true, true, true},{true, true, true, true},{true, true, true, true},
                                {true, true, true, true},{true, true, true, true},{true, true, true, true},{true, true, true, true},
                                {true, true, true, true},{true, true, true, true},{true, true, true, true},{true, true, true, true},
                                {true, true, true, true},{true, true, true, true},{true, true, true, true},{true, true, true, true},
                                {true, true, true, true},{true, true, true, true},{true, true, true, true}};
    float finalNotes[19] = {0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    float copiedKnobStates[19] = {0.f};  
    bool copiedButtonStates[19] = {true};
    float copiedSwitchStates[3] = {0.f};
    bool copyBufferFilled = false;

    bool displayUpdate = false; //flag for updating the knob states from memory
    dsp::SchmittTrigger buttonTrigger[19], stageTrigger[19];
    bool DonSample = false;
    bool KaSample = false;
    bool StageSample = false;
    
    float sequenceDir = 1.f;
    bool layerCVmode = false;
    bool copyCVonly = false;
    bool initializing = true; //load Knob positions from JSON.

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
   
        // Save knobStates
        json_t* knobStatesJ = json_array();
        for (int i = 0; i < 19; i++) {
            json_t* rowJ = json_array();
            for (int z = 0; z < 4; z++) {
                json_array_append_new(rowJ, json_real(knobStates[i][z]));
            }
            json_array_append_new(knobStatesJ, rowJ);
        }
        json_object_set_new(rootJ, "knobStates", knobStatesJ);

        // Save switchStates
        json_t* switchStatesJ = json_array();
        for (int i = 0; i < 3; i++) {
            json_t* rowJ = json_array();
            for (int z = 0; z < 4; z++) {
                json_array_append_new(rowJ, json_real(switchStates[i][z]));
            }
            json_array_append_new(switchStatesJ, rowJ);
        }
        json_object_set_new(rootJ, "switchStates", switchStatesJ);


        // Save finalNotes
        json_t* finalNotesJ = json_array();
        for (int i = 0; i < 19; i++) {
            json_array_append_new(finalNotesJ, json_real(finalNotes[i]));
        }
        json_object_set_new(rootJ, "finalNotes", finalNotesJ);

        // Save buttonStates
        json_t* buttonStatesJ = json_array();
        for (int i = 0; i < 19; i++) {
            json_t* rowJ = json_array();
            for (int z = 0; z < 4; z++) {
                json_array_append_new(rowJ, json_boolean(buttonStates[i][z]));
            }
            json_array_append_new(buttonStatesJ, rowJ);
        }
        json_object_set_new(rootJ, "buttonStates", buttonStatesJ);
    
        // Save patternState[PATTERNS][4]
        json_t* patternJ = json_array();
        for (int i = 0; i < PATTERNS; i++) {
            json_t* rowJ = json_array();
            for (int z = 0; z < 4; z++) {
                json_array_append_new(rowJ, json_integer(patternState[i][z]));
            }
            json_array_append_new(patternJ, rowJ);
        }
        json_object_set_new(rootJ, "patternState", patternJ);
    
        json_object_set_new(rootJ, "currentStage", json_integer(currentStage));
        json_object_set_new(rootJ, "leftStage", json_integer(leftStage));
        json_object_set_new(rootJ, "rightStage", json_integer(rightStage));
        json_object_set_new(rootJ, "semiStage", json_integer(semiStage));
        json_object_set_new(rootJ, "octStage", json_integer(octStage));
        json_object_set_new(rootJ, "strataLayer", json_integer(strataLayer));
    
        json_object_set_new(rootJ, "endPulseAtStage", json_boolean(endPulseAtStage));
        json_object_set_new(rootJ, "patternReset", json_boolean(patternReset));
        json_object_set_new(rootJ, "layerCVmode", json_boolean(layerCVmode));
        json_object_set_new(rootJ, "copyCVonly", json_boolean(copyCVonly));
    
        json_object_set_new(rootJ, "selectedStage", json_integer(selectedStage));

        json_object_set_new(rootJ, "copyMode", json_integer(static_cast<int>(copyMode)));

        json_object_set_new(rootJ, "playMode", json_real(playMode));
        json_object_set_new(rootJ, "lastPlayMode", json_real(lastPlayMode));

        // --- Save multiply[10][4] ---
        json_t* multiplyJ = json_array();
        for (int i = 0; i < 10; i++) {
            json_t* rowJ = json_array();
            for (int z = 0; z < 4; z++) {
                json_array_append_new(rowJ, json_real(multiply[i][z]));
            }
            json_array_append_new(multiplyJ, rowJ);
        }
        json_object_set_new(rootJ, "multiply", multiplyJ);
        
        // --- Save divide[10][4] ---
        json_t* divideJ = json_array();
        for (int i = 0; i < 10; i++) {
            json_t* rowJ = json_array();
            for (int z = 0; z < 4; z++) {
                json_array_append_new(rowJ, json_real(divide[i][z]));
            }
            json_array_append_new(divideJ, rowJ);
        }
        json_object_set_new(rootJ, "divide", divideJ);

        return rootJ;
    }
    
    void dataFromJson(json_t* rootJ) override {

        // Load knobStates
        json_t* knobStatesJ = json_object_get(rootJ, "knobStates");
        if (knobStatesJ) {
            for (int i = 0; i < 19; i++) {
                json_t* rowJ = json_array_get(knobStatesJ, i);
                if (rowJ) {
                    for (int z = 0; z < 4; z++) {
                        json_t* valueJ = json_array_get(rowJ, z);
                        if (valueJ) {
                            knobStates[i][z] = json_real_value(valueJ);
                        }
                    }
                }
            }
        }

        // Load switchStates
        json_t* switchStatesJ = json_object_get(rootJ, "switchStates");
        if (switchStatesJ) {
            for (int i = 0; i < 3; i++) {
                json_t* rowJ = json_array_get(switchStatesJ, i);
                if (rowJ) {
                    for (int z = 0; z < 4; z++) {
                        json_t* valueJ = json_array_get(rowJ, z);
                        if (valueJ) {
                            switchStates[i][z] = json_real_value(valueJ);
                        }
                    }
                }
            }
        }


        // Load finalNotes
        json_t* finalNotesJ = json_object_get(rootJ, "finalNotes");
        if (finalNotesJ) {
            for (int i = 0; i < 19; i++) {
                json_t* valueJ = json_array_get(finalNotesJ, i);
                if (valueJ) {
                    finalNotes[i] = json_real_value(valueJ);
                }
            }
        }

        // Load buttonStates
        json_t* buttonStatesJ = json_object_get(rootJ, "buttonStates");
        if (buttonStatesJ) {
            for (int i = 0; i < 19; i++) {
                json_t* rowJ = json_array_get(buttonStatesJ, i);
                if (rowJ) {
                    for (int z = 0; z < 4; z++) {
                        json_t* valueJ = json_array_get(rowJ, z);
                        if (valueJ) {
                            buttonStates[i][z] = json_boolean_value(valueJ);
                        }
                    }
                }
            }
        }

        // Load patternState with backward compatibility
        json_t* patternJ = json_object_get(rootJ, "patternState");
        if (patternJ && json_is_array(patternJ)) {
        
            json_t* first = json_array_get(patternJ, 0);
        
            bool oldFormat = first && json_is_integer(first);     // OLD: [PATTERNS]
            bool newFormat = first && json_is_array(first);        // NEW: [PATTERNS][4]
        
            if (oldFormat) {
                // OLD FORMAT: patternState[PATTERNS]
                for (size_t i = 0; i < json_array_size(patternJ) && i < PATTERNS; i++) {
                    json_t* valJ = json_array_get(patternJ, i);
                    if (json_is_integer(valJ)) {
                        int v = json_integer_value(valJ);
                        // Copy same value to all 4 layers
                        for (int z = 0; z < 4; z++) {
                            patternState[i][z] = v;
                        }
                    }
                }
        
            } else if (newFormat) {
                // NEW FORMAT: patternState[PATTERNS][4]
                for (size_t i = 0; i < json_array_size(patternJ) && i < PATTERNS; i++) {
                    json_t* rowJ = json_array_get(patternJ, i);
                    if (rowJ && json_is_array(rowJ)) {
                        for (size_t z = 0; z < json_array_size(rowJ) && z < 4; z++) {
                            json_t* valJ = json_array_get(rowJ, z);
                            if (json_is_integer(valJ)) {
                                patternState[i][z] = json_integer_value(valJ);
                            }
                        }
                    }
                }
            }
        }

        //Load PasteMode enum
        json_t* copyModeJ = json_object_get(rootJ, "copyMode");
        if (copyModeJ && json_is_integer(copyModeJ)) {
            int v = json_integer_value(copyModeJ);
            if (v >= 0 && v <= 2)
                copyMode = static_cast<PasteMode>(v);
        }
    
        json_t* curStageJ = json_object_get(rootJ, "currentStage");
        if (curStageJ && json_is_integer(curStageJ)) {
            currentStage = json_integer_value(curStageJ);
        }
    
        json_t* selStageJ = json_object_get(rootJ, "selectedStage");
        if (selStageJ && json_is_integer(selStageJ)) {
            selectedStage = json_integer_value(selStageJ);
        }

        json_t* leftStageJ = json_object_get(rootJ, "leftStage");
        if (leftStageJ && json_is_integer(leftStageJ)) {
            leftStage = json_integer_value(leftStageJ);
        }
        json_t* rightStageJ = json_object_get(rootJ, "rightStage");
        if (rightStageJ && json_is_integer(rightStageJ)) {
            rightStage = json_integer_value(rightStageJ);
        }
        json_t* semiStageJ = json_object_get(rootJ, "semiStage");
        if (semiStageJ && json_is_integer(semiStageJ)) {
            semiStage = json_integer_value(semiStageJ);
        }
        json_t* octStageJ = json_object_get(rootJ, "octStage");
        if (octStageJ && json_is_integer(octStageJ)) {
            octStage = json_integer_value(octStageJ);
        }
        json_t* strataLayerJ = json_object_get(rootJ, "strataLayer");
        if (strataLayerJ && json_is_integer(strataLayerJ)) {
            strataLayer = json_integer_value(strataLayerJ);
        }
    
        json_t* endPulseAtStageJ = json_object_get(rootJ, "endPulseAtStage");
        if (endPulseAtStageJ) {
            endPulseAtStage = json_boolean_value(endPulseAtStageJ);
        }
    
        json_t* patternResetJ = json_object_get(rootJ, "patternReset");
        if (patternResetJ) {
            patternReset = json_boolean_value(patternResetJ);
        }

        json_t* layerCVmodeJ = json_object_get(rootJ, "layerCVmode");
        if (layerCVmodeJ) {
            layerCVmode = json_boolean_value(layerCVmodeJ);
        }

        json_t* copyCVonlyJ = json_object_get(rootJ, "copyCVonly");
        if (copyCVonlyJ) {
            copyCVonly = json_boolean_value(copyCVonlyJ);
        }
   
        // --- Load multiply with backward compatibility ---
        json_t* multiplyJ = json_object_get(rootJ, "multiply");
        if (multiplyJ && json_is_array(multiplyJ)) {
        
            // Detect OLD format: multiply[10]
            json_t* first = json_array_get(multiplyJ, 0);
            bool oldFormat = first && json_is_number(first);
        
            if (oldFormat) {
                // OLD FORMAT: multiply[10]
                for (size_t i = 0; i < json_array_size(multiplyJ) && i < 10; i++) {
                    json_t* valJ = json_array_get(multiplyJ, i);
                    if (json_is_number(valJ)) {
                        float v = json_number_value(valJ);
                        // Fill ALL 4 layers with the same value
                        for (int z = 0; z < 4; z++) {
                            multiply[i][z] = v;
                        }
                    }
                }
        
            } else {
                // NEW FORMAT: multiply[10][4]
                for (size_t i = 0; i < json_array_size(multiplyJ) && i < 10; i++) {
                    json_t* rowJ = json_array_get(multiplyJ, i);
                    if (rowJ && json_is_array(rowJ)) {
                        for (size_t z = 0; z < json_array_size(rowJ) && z < 4; z++) {
                            json_t* valJ = json_array_get(rowJ, z);
                            if (json_is_number(valJ)) {
                                multiply[i][z] = json_number_value(valJ);
                            }
                        }
                    }
                }
            }
        }
 
         // --- Load divide with backward compatibility ---
        json_t* divideJ = json_object_get(rootJ, "divide");
        if (divideJ && json_is_array(divideJ)) {
            
            // Detect OLD format: divide[10]
            json_t* first = json_array_get(divideJ, 0);
            bool oldFormat = first && json_is_number(first);
        
            if (oldFormat) {
                // OLD FORMAT: divide[10]
                for (size_t i = 0; i < json_array_size(divideJ) && i < 10; i++) {
                    json_t* valJ = json_array_get(divideJ, i);
                    if (json_is_number(valJ)) {
                        float v = json_number_value(valJ);
                        // Fill all 4 layers
                        for (int z = 0; z < 4; z++) {
                            divide[i][z] = v;
                        }
                    }
                }
            } 
            else {
                // NEW FORMAT: divide[10][4]
                for (size_t i = 0; i < json_array_size(divideJ) && i < 10; i++) {
                    json_t* rowJ = json_array_get(divideJ, i);
                    if (rowJ && json_is_array(rowJ)) {
                        for (size_t z = 0; z < json_array_size(rowJ) && z < 4; z++) {
                            json_t* valJ = json_array_get(rowJ, z);
                            if (json_is_number(valJ)) {
                                divide[i][z] = json_number_value(valJ);
                            }
                        }
                    }
                }
            }
        }
 
        json_t* playModeJ = json_object_get(rootJ, "playMode");
        if (playModeJ && json_is_number(playModeJ)) {
            playMode = json_number_value(playModeJ);
        }
    
        json_t* lastPlayModeJ = json_object_get(rootJ, "lastPlayMode");
        if (lastPlayModeJ && json_is_number(lastPlayModeJ)) {
            lastPlayMode = json_number_value(lastPlayModeJ);
        }
    }

    Strata() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
    
        // === MAIN SEQUENCER KNOBS (–2..2 V/oct) ===
        for (int i = 0; i < 4; i++) {
            configParam(SEQ_1_KNOB + i, -2.f, 2.f, 0.f, string::f("Stage L%d Pitch (V)", i + 1) );
        }
        for (int i = 0; i < 4; i++) {
            configParam(SEQ_5_KNOB + i, -2.f, 2.f, 0.f, string::f("Stage L%d Pitch (V)", i + 1) );
        }
    
        // === MAIN SEQUENCER BUTTONS ===
        for (int i = 0; i < 4; i++) { 
            configButton(SEQ_1_BUTTON + i, string::f("Stage L%d Enable", i + 1));
        }
        for (int i = 0; i < 4; i++) { 
            configButton(SEQ_5_BUTTON + i, string::f("Stage R%d Enable", i + 1));
        }
    
        // === SEMITONE KNOBS (–12..12 semitones) ===
        for (int i = 0; i < 7; i++) {
            configParam(SEMI_1_KNOB + i,  -12.f, 12.f, 0.f, string::f("Semitone %d", i + 1))->snapEnabled=true;
        }
    
        // === SEMITONE BUTTONS ===
        for (int i = 0; i < 7; i++) {
            configButton(SEMI_1_BUTTON + i, string::f("Semitone %d Enable", i + 1));
        }
    
        // === OCTAVE KNOBS (–2..2) ===
        for (int i = 0; i < 4; i++) {
            configParam(OCT_1_KNOB + i, -3.f, 3.f, 0.f, string::f("Octave %d", i + 1))->snapEnabled=true;
        }
    
        // === OCTAVE BUTTONS ===
        for (int i = 0; i < 4; i++) {
            configButton(OCT_1_BUTTON + i,  string::f("Octave %d Enable", i + 1));
        }
    
        // === BEATS UP BUTTONS (per stage + semi + octave) ===
        for (int i = 0; i < 8; i++)
            configButton(STAGE_1_BEATS_UP + i,  string::f("Stage %d Beats +", i + 1));
        configButton(SEMI_BEATS_UP, "Semitone Beats +");
        configButton(OCT_BEATS_UP,  "Octave Beats +");
    
        // === BEATS DOWN BUTTONS ===
        for (int i = 0; i < 8; i++)
            configButton(STAGE_1_BEATS_DOWN + i, string::f("Stage %d Beats –", i + 1));
        configButton(SEMI_BEATS_DOWN, "Semitone Beats –");
        configButton(OCT_BEATS_DOWN,  "Octave Beats –");
    
        // === STEPS UP BUTTONS ===
        for (int i = 0; i < 8; i++)
            configButton(STAGE_1_STEPS_UP + i, string::f("Stage %d Steps +", i + 1));
        configButton(SEMI_STEPS_UP, "Semitone Steps +");
        configButton(OCT_STEPS_UP,  "Octave Steps +");
    
        // === STEPS DOWN BUTTONS ===
        for (int i = 0; i < 8; i++)
            configButton(STAGE_1_STEPS_DOWN + i, string::f("Stage %d Steps –", i + 1));
        configButton(SEMI_STEPS_DOWN, "Semitone Steps –");
        configButton(OCT_STEPS_DOWN,  "Octave Steps –");
    
        // === PATTERN SELECT BUTTONS (1–24) ===
        for (int i = 0; i < 24; i++) {
            configButton(PATTERN_1_BUTTON + i, string::f("Pattern %d", i + 1));
        }
    
        // === PATTERN LENGTH KNOB ===
        configParam(PATTERN_KNOB,  1.f, 24.f, 8.f, "Pattern Length")->snapEnabled=true;;
    
        // === SWITCHES ===
        configSwitch(MAIN_SWITCH, 0.f, 2.f, 0.f, "Main Seq. Mode", {"Stage", "Hybrid", "Step"} );
        configSwitch(SEMI_SWITCH, 0.f, 2.f, 0.f, "Semitone Seq. Dir.", {"Fwd", "Ping-Pong", "Rev"} );
        configSwitch(OCT_SWITCH,  0.f, 2.f, 1.f, "Octave Dest.", {"Left", "Both", "Right"} );
        configSwitch(ON_SWITCH, 0.0, 2.0, 1.0, "Play Mode", {"Off", "On", "One-Shot"} );
    
        configButton(RESET_BUTTON, "Reset Button");
    
        for (int i = 0; i < 4; i++)
            configButton(LAYER_1_BUTTON + i, string::f("Layer %d Sel.", i + 1));
        configButton(LAYER_NEXT_BUTTON, "Next Layer");
    
        // === OFFSET PARAMETER ===
        configParam(OFFSET_PARAM,  -2.f, 2.f, 0.f,  "Global Offset");
    
        // === INPUTS ===
        configInput(CLOCK_INPUT, "Clock");
        configInput(RESET_INPUT, "Reset");
        configInput(OFFSET_INPUT, "Offset CV");
        configInput(LAYER_INPUT, "Layer CV");
        configInput(PATTERN_INPUT, "Pattern Len. CV");
    
        // === OUTPUTS ===
        configOutput(GATE_OUTPUT, "Gate");
        configOutput(INV_GATE_OUTPUT, "Inv. Gate");
        configOutput(MAIN_OUTPUT, "Main Output");
    }

    void onReset(const ResetEvent& e) override {
        // Reset all parameters
        Module::onReset(e);
    
        // Reset custom state variables
        for (int i = 0; i < 10; i++) {
            for (int z = 0; z < 4; z++) {    
                multiply[i][z] = 1.0f;
                divide[i][z] = 1.0f;
            }
        }
    
        for (int i = 0; i < PATTERNS; i++) {
            int v = (i >= 4 && i <= 7) ? 1 : 0;
            for (int z = 0; z < 4; z++) {
                patternState[i][z] = v;
            }
        }        

        // Reset knobStates to all zeros
        for (int x = 0; x < 19; x++) {
            for (int z = 0; z < 4; z++) {
                knobStates[x][z] = 0.0f;
            }
        }

        // Reset buttonStates to all true
        for (int x = 0; x < 19; x++) {
            for (int z = 0; z < 4; z++) {
                buttonStates[x][z] = true;
            }
        }

        // Reset switchStates
        for (int z = 0; z < 4; z++) {
            switchStates[0][z] = 0.0f;
            switchStates[1][z] = 0.0f;
            switchStates[2][z] = 1.0f;
        }

        // Reset finalNotes to all zeros
        for (int i = 0; i < 19; i++) {
            finalNotes[i] = 0.0f;
        }
        
        // Reset parameter knobs to 0
        for (int i = 0; i < 19; i++) {
            params[SEQ_1_KNOB + i].setValue(0.0f);
        }
        
    }
    
    void onRandomize(const RandomizeEvent& e) override {
        
        // --- PATTERN_KNOB (int 1…24) ---
        {
            int p = 1 + (random::u32() % PATTERNS);   // PATTERNS = 24
            params[PATTERN_KNOB].setValue((float)p);
        }
    
        // --- MULTIPLY / DIVIDE (10 entries each) ---
        for (int i = 0; i < 10; i++) {
            float m = random::uniform() * 12.f;       // 0…12
            float d = 1.f + random::uniform() * 8.f;  // 1…9
            multiply[i][strataLayer] = m;
            divide[i][strataLayer]   = d;
        }
    
        // --- PATTERN STATES (0,1,2) ---
        for (int i = 0; i < PATTERNS; i++)
            patternState[i][strataLayer] = (int)(random::u32() % 3);
    
        // --- SEQ_1…SEQ_8_KNOB  (–2…2 V in 1/12 increments) ---
        for (int i = 0; i < 8; i++) {
            int steps = randomInt(-24, 24);          // -24…24 semitones
            float v = steps / 12.f;
            params[SEQ_1_KNOB + i].setValue(v);
        }
    
        // --- SEMI_1…SEMI_7_KNOB  (int –12…12) ---
        for (int i = 0; i < 7; i++) {
            int v = randomInt(-12, 12);
            params[SEMI_1_KNOB + i].setValue((float)v);
        }
    
        // --- OCT_1…OCT_4_KNOB (int –2…2) ---
        for (int i = 0; i < 4; i++) {
            int v = randomInt(-2, 2);
            params[OCT_1_KNOB + i].setValue((float)v);
        }
    
        // --- SWITCHES (MAIN / SEMI / OCT) (0…2) ---
        params[MAIN_SWITCH].setValue((float)randomInt(0,2));
        params[SEMI_SWITCH].setValue((float)randomInt(0,2));
        params[OCT_SWITCH].setValue((float)randomInt(0,2));
        
        //Button randomization with at least one active button per group        
        // Randomize all buttons first
        for (int i = 0; i < 19; i++) {
            buttonStates[i][strataLayer] = (random::uniform() < 0.5f);
        }
    
        // Then force each group to have at least one true:
        auto ensureOne = [&](int start, int count) {
            bool any = false;
            for (int i = 0; i < count; i++) {
                if (buttonStates[start + i][strataLayer]) {
                    any = true;
                    break;
                }
            }
            if (!any) {
                int pick = start + (random::u32() % count);
                buttonStates[pick][strataLayer] = true;
            }
        };
    
        ensureOne(0, 4);   // Group 0–3
        ensureOne(4, 4);   // Group 4–7
        ensureOne(8, 8);   // Group 8–15
        ensureOne(16, 3);  // Group 16–18        
                
    }

    void process(const ProcessArgs& args) override {
    
        noteSampled = false;
        DonSample = false;
        KaSample = false;
        StageSample = false;

        //Process ON/OFF Switch
        playMode = params[ON_SWITCH].getValue();
        if (lastPlayMode == 0.f) lastPlayMode = 1.f;  //this mode can only be 1 or 2, but default is 1
        
        float deltaTime = args.sampleTime;
        syncTimer.process(deltaTime);
        beatTimer.process(deltaTime);
        beatTimer_semi.process(deltaTime);
        beatTimer_oct.process(deltaTime);

        float mainSwitch = params[MAIN_SWITCH].getValue();
        float semiSwitch = params[SEMI_SWITCH].getValue();
        float octSwitch = params[OCT_SWITCH].getValue();

        if (initializing){
            //Update Knob Ranges
            for (int i = 0; i < 19; i++) {
                    paramQuantities[SEQ_1_KNOB + i]->setValue( knobStates[i][strataLayer] );  // Recall stored knob value    
                    // Apply clamping to avoid out-of-bounds values
                    finalNotes[i] = clamp(params[SEQ_1_KNOB + i].getValue(), -10.f, 10.f);
            }  
            for (int i = 0; i < 3; i++) {
                    paramQuantities[MAIN_SWITCH + i]->setValue( switchStates[i][strataLayer] );  // Recall stored switch value    
            }                        
            initializing = false;
        } 

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
                
                // --- Clock pulse detected ---
                if (resetArmed) {
                    // --- first clock after a reset ---
                    resetArmed = false;
                    firstPulseReceived = true;
                    firstSync = true;
                    syncPoint = true;        // trigger immediately
                    syncTimer.reset();
                    beatTimer.reset();
                    beatCount = 0;
                    beatCountSemi = 0;
                    beatCountOct = 0;
                    subBeatCount = 0;
                    subBeatCount_semi = 0;
                    subBeatCount_oct = 0;
                
                    // produce the first beat now
                    if (playMode > 0.f) {
                        patternIndex = 0;  // always start from step 1
                        if (patternState[patternIndex][strataLayer] == 0) {DonPulse.trigger(beatInterval/2); DonSample = true; noteSampled = true;}
                        if (patternState[patternIndex][strataLayer] == 1) {KaPulse.trigger(beatInterval/2); KaSample = true; noteSampled = true;}
                    }
                
                } else if (!firstPulseReceived) {
                    // --- normal initial start ---
                    firstPulseReceived = true;
                    firstSync = true;
                    syncPoint = true;
                    syncTimer.reset();
                    beatTimer.reset();
                    beatCount = 0;
                    beatCountSemi = 0;
                    beatCountOct = 0;
                    subBeatCount = 0;
                    subBeatCount_semi = 0;
                    subBeatCount_oct = 0;
                
                } else {
                    // --- all subsequent pulses ---
                    syncInterval = syncTimer.time;
                    syncTimer.reset();
                    syncPoint = true;
                    firstSync = false;
                }
            }
        }

        // Process Pattern Knob
        patternStages = params[PATTERN_KNOB].getValue(); //initial value set by knob
        if (inputs[PATTERN_INPUT].isConnected()){
            patternStages += inputs[PATTERN_INPUT].getVoltage();
        }
        patternStages = ceilf(patternStages); //round up
        patternStages = clamp(patternStages, 1, PATTERNS); //keep in strict bounds
        if (patternIndex >= patternStages) patternIndex = 0;

        // Process Ratio Buttons
        inputSkipper++;
        if (inputSkipper > inputSkipsTotal){ //Process button inputs infrequently to reduce CPU load. 
        
            for (int i = 0; i < STAGES+2; i++) {
                if (xDownTriggers[i].process(params[STAGE_1_BEATS_DOWN + i].getValue())) { multiply[i][strataLayer] -= 1.f; resyncFlag[i]=true;}
                if (xUpTriggers[i].process(params[STAGE_1_BEATS_UP + i].getValue()))   { multiply[i][strataLayer] += 1.f; resyncFlag[i]=true;}
                if (yDownTriggers[i].process(params[STAGE_1_STEPS_DOWN + i].getValue())) { divide[i][strataLayer] -= 1.f; resyncFlag[i]=true;}
                if (yUpTriggers[i].process(params[STAGE_1_STEPS_UP + i].getValue()))   { divide[i][strataLayer] += 1.f; resyncFlag[i]=true;}
                multiply[i][strataLayer] = clamp(multiply[i][strataLayer],0.0f,99.0f);
                divide[i][strataLayer] = clamp(divide[i][strataLayer],0.0f, 99.0f); // divide[i][strataLayer] can be zero! So we have to be careful to actually de-activate the stage when we set it to zero.
    
            }
            
            inputSkipper = 0;            
        }
        divide[0][strataLayer] = clamp(divide[0][strataLayer],1.f,99.f); // The top stage cannot be turned off, limited to 1 instead of 0.
                                              // if (divide[i][strataLayer]==0.f) the stage is OFF
                                              // if (multiply[i][strataLayer]==0.f) the stage is muted.

        // Stage Advancing
        if (syncPoint && playMode > 0.f){
            beatCount++;
            beatCountSemi++;
            beatCountOct++;

            if (firstSync) {
                beatCount = 0; // Reset beat count on the first clock sync
                beatCountSemi = 0;
                beatCountOct = 0;    
            }
            int stageLength = static_cast<int>(divide[currentStage][strataLayer]);
            

            if (beatCount >= stageLength ){
                beatCount = 0;

                currentStage++;
                selectedStage++;
                beatTimer.reset(); // <-- Reset the Beat Timer

                // Advance to next active stage
                for (int i = 0; i < STAGES; i++) { 
                    if (currentStage >= STAGES) { //Stage Wrap-Around Point
                        currentStage = 0;
                        if (playMode == 2.0) { //one-shot mode
                            paramQuantities[ON_SWITCH]->setDisplayValue(0.0f);
                            playMode = 0.f;
                            lastPlayMode = 2.0;
                        }
                    }
                    if (divide[currentStage][strataLayer] == 0) currentStage++; //skip stages that are off
                }
                // Keep selectedStage in sync
                selectedStage = currentStage;

                EndPulse.trigger(0.001f); 
                blinkEND = true; 
                StageSample = true;

                if (multiply[currentStage][strataLayer]>0){
                    beatInterval = (divide[currentStage][strataLayer] * syncInterval) / multiply[currentStage][strataLayer];
                    beatInterval = fmax(0.001f, beatInterval); //minimum pulsewidth
                } 

                patternIndex++;
                if (patternReset) patternIndex=0;
                if (patternIndex >= patternStages) patternIndex = 0;
                if (patternState[patternIndex][strataLayer]==0) {
                    DonPulse.trigger(beatInterval); 
                    DonSample = true;
                    noteSampled = true;
                }
                if (patternState[patternIndex][strataLayer]==1) {
                    KaPulse.trigger(beatInterval); 
                    KaSample = true;
                    noteSampled = true;
                }

            } 
            
            if (beatCountSemi >= static_cast<int>(divide[8][strataLayer])) {
                beatCountSemi = 0;
                beatTimer_semi.reset();
            
                // --- Build list of active semitone steps ---
                int active[7];
                int activeCount = 0;
                for (int s = 0; s < 7; s++)
                    if (buttonStates[s + 8][strataLayer])
                        active[activeCount++] = s;
            
                if (activeCount == 0)
                    return;
            
                // Find current index
                int idx = 0;
                for (int i = 0; i < activeCount; i++)
                    if (active[i] == semiStage) { idx = i; break; }
            
                // Determine direction
                if (semiSwitch == 0) sequenceDir = +1;
                else if (semiSwitch == 2) sequenceDir = -1;
            
                // Advance the sequence (main beat)
                if (semiSwitch == 1) { // ping-pong
                    idx += sequenceDir;
                    if (idx >= activeCount) { idx = activeCount - 2; sequenceDir = -1; }
                    else if (idx < 0)      { idx = 1; sequenceDir = +1; }
                }
                else {  // forward/reverse wrap
                    idx += sequenceDir;
                    if (idx >= activeCount) idx = 0;
                    if (idx < 0)            idx = activeCount - 1;
                }
            
                if (divide[8][strataLayer]>0 && multiply[8][strataLayer]>0) semiStage = active[idx];
            }
          
            if (beatCountOct >= static_cast<int>(divide[9][strataLayer])) {
                beatCountOct = 0;
                beatTimer_oct.reset();
                
                if (divide[9][strataLayer] > 0.f && multiply[9][strataLayer] > 0.f && playMode > 0.f) { octStage++; }//advance octave sequencer  
                if (octStage>3) octStage = 0;      
            }
        }
        
        // Beat Computing (sub-beats within each stage)
        if (divide[currentStage][strataLayer] > 0.f && multiply[currentStage][strataLayer] > 0.f && playMode > 0.f) {
        
            if ((syncPoint && beatCount == 0) || resyncFlag[currentStage]) {
                resyncFlag[currentStage] = false;
                // Total duration of this stage / number of sub-beats
                beatInterval = (divide[currentStage][strataLayer] * syncInterval) / multiply[currentStage][strataLayer];
                beatInterval = fmax(0.001f, beatInterval);
                
                beatTimer.reset();
                subBeatCount = 0;
            }
        
            if (beatTimer.time >= beatInterval && playMode > 0.f && externalClockConnected) {
                beatTimer.reset();
                subBeatCount++;
        
                // Only produce sub-beats for intermediate positions — the last sub-beat is skipped so the stage advance triggers
                if (subBeatCount < multiply[currentStage][strataLayer]) {
                    if (mainSwitch >= 1 ) patternIndex++; //increment patternIndex based on the mode setting
                    if (patternIndex >= patternStages)
                        patternIndex = 0;
        
                    if (patternState[patternIndex][strataLayer] == 0){
                        DonPulse.trigger(beatInterval); 
                        if (mainSwitch == 2) DonSample = true;
                        noteSampled = true;
                    }
                    if (patternState[patternIndex][strataLayer] == 1){
                        KaPulse.trigger(beatInterval); 
                        if (mainSwitch == 2) KaSample = true;
                        noteSampled = true;
                    }
                }
            }
        }

        // Beat Computing for Semi and Oct Sequencers
        if (divide[8][strataLayer] > 0.f && multiply[8][strataLayer] > 0.f && playMode > 0.f) {
            if ((syncPoint && beatCountSemi == 0) || resyncFlag[8]) {
                resyncFlag[8] = false;
                // Total duration of this stage / number of sub-beats
                beatInterval_semi = (divide[8][strataLayer] * syncInterval) / multiply[8][strataLayer];
                beatTimer_semi.reset();
                subBeatCount_semi = 0;
            }
        
            if (beatTimer_semi.time >= beatInterval_semi && playMode > 0.f && externalClockConnected) {
                beatTimer_semi.reset();
                subBeatCount_semi++;
                if (subBeatCount_semi < multiply[8][strataLayer]) {                    
                    semiStage++; //advance semitone sequencer  
                    if (semiStage>6) semiStage = 0;     
                } 
            }
        }

        if (divide[9][strataLayer] > 0.f && multiply[9][strataLayer] > 0.f && playMode > 0.f) {        
            if ((syncPoint && beatCountOct == 0) || resyncFlag[9]) {
                resyncFlag[9] = false;
                // Total duration of this stage / number of sub-beats
                beatInterval_oct = (divide[9][strataLayer] * syncInterval) / multiply[9][strataLayer];
                beatTimer_oct.reset();
                subBeatCount_oct = 0;
            }
       
            if (beatTimer_oct.time >= beatInterval_oct && playMode > 0.f && externalClockConnected) {
                beatTimer_oct.reset();
                subBeatCount_oct++;
                if (subBeatCount_oct < multiply[9][strataLayer]) {               
                    octStage++; //advance oct sequencer      
                    if (octStage>3) octStage = 0;  
                }
            }
        }

        // Advance to next active octave stage
        for (int i = 0; i < 4; i++) { 
            if (octStage >= 4) { //Stage Wrap-Around Point
                octStage = 0;
            }
            if (buttonStates[octStage+8+7][strataLayer]<1) octStage++; //skip stages that are off
        }

        //Beat Outputs      
        bool DonActive = false;
        bool KaActive = false;
        
        if (DonPulse.process(args.sampleTime)){ 
            DonActive = true; 
            blinkDON = true;
        }
        if (KaPulse.process(args.sampleTime)) { 
            KaActive = true; 
            blinkKA = true;
        }
        
        // DON SAMPLE 
        if (DonSample) {
            if (activeStage < 4) {
                int start = leftStage;
                for (int i = 1; i <= 4; i++) {
                    int next = (start + i) % 4;
                    if (buttonStates[next][strataLayer] >= 1) { leftStage = next; break; }
                }
            } else {
                int start = rightStage;
                for (int i = 1; i <= 4; i++) {
                    int next = (start + i) % 4;
                    if (buttonStates[next + 4][strataLayer] >= 1) { rightStage = next; break; }
                }
            }
            activeStage = leftStage;
        }
        
        // KA SAMPLE 
        if (KaSample) {
            if (activeStage > 3) {
                int start = rightStage;
                for (int i = 1; i <= 4; i++) {
                    int next = (start + i) % 4;
                    if (buttonStates[next + 4][strataLayer] >= 1) { rightStage = next; break; }
                }
            } else {
                int start = leftStage;
                for (int i = 1; i <= 4; i++) {
                    int next = (start + i) % 4;
                    if (buttonStates[next][strataLayer] >= 1) { leftStage = next; break; }
                }
            }
            activeStage = rightStage + 4;
        }

        DonPulse.process(args.sampleTime);
        KaPulse.process(args.sampleTime);
        
        bool BeatActive = DonActive || KaActive;

        if (divide[currentStage][strataLayer]>0.f && multiply[currentStage][strataLayer]>0.f && playMode > 0.f ){
            outputs[GATE_OUTPUT].setVoltage(BeatActive ? 10.f : 0.f);
            outputs[INV_GATE_OUTPUT].setVoltage(BeatActive ? 0.f : 10.f);
        } else {
            outputs[GATE_OUTPUT].setVoltage(0.f);
            outputs[INV_GATE_OUTPUT].setVoltage(10.f);
        }
        

        // Handle Reset Input
        bool reset = resetButtonTrigger.process(params[RESET_BUTTON].getValue()); //initial value set by knob
        if (inputs[RESET_INPUT].isConnected()){
            if(resetTrigger.process(inputs[RESET_INPUT].getVoltage()-0.1f)) reset = true;
        }

        if (reset || resetCondition) {
            currentStage = 0;
            selectedStage = 0;
            beatTimer.reset();
            patternIndex = 0; 
        
            clockTrigger.reset();
            syncTimer.reset();
            syncPoint = false;
        
            firstPulseReceived = false;
            firstSync = false;
        
            subBeatCount = 0;
            beatCount = 0;
            resetArmed = true;   

            leftStage = 0;
            rightStage = 0;
            activeStage = 0;
            semiStage = 0;
            octStage =  0;        
            //strataLayer = 0; //Don't reset the layer, as sometimes we don't want that.
        
            if (lastPlayMode == 2.0f) {   
                if (playMode > 0.f) {
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
            if(patternTrigger[i].process( params[PATTERN_1_BUTTON+i].getValue())){
                patternState[i][strataLayer]++;
                if (patternState[i][strataLayer]>2) patternState[i][strataLayer]=0;
            }            
        }
 
        // Handle Layer Buttons
        if(inputs[LAYER_INPUT].isConnected()){
            if (layerCVmode){
                strataLayer = (int)roundf(clamp(inputs[LAYER_INPUT].getVoltage(), 0.f, 10.f) );
                strataLayer = (strataLayer + 4) % 4;
            } else {
                for (int i = 0; i < 4; i++){
                    if(layerTrigger[i].process( params[LAYER_1_BUTTON+i].getValue())){
                        strataLayer = i;
                    }            
                }
                
                if(layerTrigger[4].process(params[LAYER_NEXT_BUTTON].getValue())){
                    strataLayer++;
                    if (strataLayer>3) strataLayer=0;
                }

                if(layerTrigger[5].process(inputs[LAYER_INPUT].getVoltage())){ //Also advance on CV trigger
                    strataLayer++;
                    if (strataLayer>3) strataLayer=0;
                }
            }
        } else {
            for (int i = 0; i < 4; i++){
                if(layerTrigger[i].process( params[LAYER_1_BUTTON+i].getValue())){
                    strataLayer = i;
                }            
            }
            
            if(layerTrigger[4].process(params[LAYER_NEXT_BUTTON].getValue())){
                strataLayer++;
                if (strataLayer>3) strataLayer=0;
            }
        }
        
        // Detect Layer change and flag display update
        if (strataLayer != previousStrataLayer) {
            displayUpdate = true;
            previousStrataLayer = strataLayer;
        }      
        
        // Deal with knob parameter saving and recall
        for (int i= 0; i < 19; i++) {

            if (displayUpdate) {
                paramQuantities[SEQ_1_KNOB + i]->setValue( knobStates[i][strataLayer] );  // Recall stored knob value
            } else {
                knobStates[i][strataLayer] = params[SEQ_1_KNOB + i].getValue();  // Save current knob state
            }

            if (stageTrigger[i].process(params[SEQ_1_BUTTON + i].getValue())) {
            
                // ----- GROUP 0-3 -----
                if (i >= 0 && i <= 3) {
                    int active = 0;
                    for (int j = 0; j < 4; j++)
                        if (buttonStates[j][strataLayer]) active++;
            
                    if (buttonStates[i][strataLayer] && active == 1) {
                        // block turning off the last one
                    } else {
                        buttonStates[i][strataLayer] = !buttonStates[i][strataLayer];
                    }
                }
            
                // ----- GROUP 4-7 -----
                else if (i >= 4 && i <= 7) {
                    int active = 0;
                    for (int j = 4; j < 8; j++)
                        if (buttonStates[j][strataLayer]) active++;
            
                    if (buttonStates[i][strataLayer] && active == 1) {
                        // block turning off the last one
                    } else {
                        buttonStates[i][strataLayer] = !buttonStates[i][strataLayer];
                    }
                }
            
                // ----- GROUP 8-15 -----
                else if (i >= 8 && i <= 15) {
                    int active = 0;
                    for (int j = 8; j < 16; j++)
                        if (buttonStates[j][strataLayer]) active++;
            
                    if (buttonStates[i][strataLayer] && active == 1) {
                        // block turning off the last one
                    } else {
                        buttonStates[i][strataLayer] = !buttonStates[i][strataLayer];
                    }
                }
            
                // ----- GROUP 16-19 -----
                else if (i >= 16 && i <= 19) {
                    int active = 0;
                    for (int j = 16; j < 20; j++)
                        if (buttonStates[j][strataLayer]) active++;
            
                    if (buttonStates[i][strataLayer] && active == 1) {
                        // block turning off the last one
                    } else {
                        buttonStates[i][strataLayer] = !buttonStates[i][strataLayer];
                    }
                }
            }

            finalNotes[i] = params[SEQ_1_KNOB + i].getValue();            
        }

        //Save and Recall Switch
        for (int i= 0; i < 3; i++) {
            if (displayUpdate) {
                paramQuantities[MAIN_SWITCH + i]->setValue( switchStates[i][strataLayer] );  // Recall stored knob value
            } else {
                switchStates[i][strataLayer] = params[MAIN_SWITCH + i].getValue();  // Save current knob state
            }
        }
        
        //Save and Recall Pattern knob
        if (displayUpdate) {
            paramQuantities[PATTERN_KNOB]->setValue( patternKnob[strataLayer] );  // Recall stored knob value
        } else {
            patternKnob[strataLayer] = params[PATTERN_KNOB].getValue();  // Save current knob state
        }
        
        // Reset display update flag after processing
        if (displayUpdate) {
            displayUpdate = false;
        }
        
        if (noteSampled){
            float rootNote = params[SEQ_1_KNOB + activeStage].getValue();
            float offsetSeq = params[SEMI_1_KNOB + semiStage].getValue()/12.f;
            float offsetCV = inputs[OFFSET_INPUT].isConnected() ? inputs[OFFSET_INPUT].getVoltage() : 0.f;
            float globalOffset = params[OFFSET_PARAM].getValue();
            float octOffset = params[OCT_1_KNOB + octStage].getValue();   
            
            if (octSwitch == 0){ //left sequencer only
                if (activeStage>3) octOffset = 0.f;
            } else if (octSwitch == 1){ //both sequencers
                // offsetSeq unchanged
            } else if (octSwitch ==2){ //right sequencer only
                if (activeStage<4) octOffset = 0.f;
            }
 
            if (!(divide[8][strataLayer] > 0.f )) offsetSeq = 0.f;
            if (!(divide[9][strataLayer] > 0.f )) octOffset = 0.f;
            
            float finalValue = clamp(rootNote + offsetSeq + offsetCV + octOffset + globalOffset, -10.f, 10.f );
            float quantizedNote = std::roundf(finalValue * 12.f) / 12.f;  // Quantize to nearest semitone
            
            
                        
            outputs[MAIN_OUTPUT].setVoltage(quantizedNote);
        }
                
    }//end process
};

struct StrataWidget : ModuleWidget {
    DigitalDisplay* noteDisplays[8] = {nullptr};
    DigitalDisplay* semiDisplays[7] = {nullptr};
    DigitalDisplay* octDisplays[4] = {nullptr};
    DigitalDisplay* ratioBeatsDisplays[10] = {nullptr};
    DigitalDisplay* ratioStagesDisplays[10] = {nullptr};
    DigitalDisplay* outputDisplay = {nullptr};

    StrataWidget(Strata* module) {
        setModule(module);

        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Strata.svg"),
            asset::plugin(pluginInstance, "res/Strata-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        float xStart = box.size.x*0.07736;
        float xUnit = box.size.x*0.09434;
        float yStart = 50.f;
        float yUnit = 18.5f;
        float xGap = 15.f; //spacing of buttons from center line
        float leftPad = 8.f;
        
        float posX = 0.f;
        float posY = 0.f;
        float dispX = -0.3f; //offset for display
        float dispY = -0.4f; //offset for displays

        // Clock and Reset Input/Buttons
        posX = xStart - xGap + leftPad;
        posY = yStart-10.f;
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(posX, posY-10.f ), module, Strata::PATTERN_INPUT));
        posY += 1.75*yUnit;
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(posX, posY ), module, Strata::CLOCK_INPUT));
        posY += 1.75*yUnit;
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(posX, posY ), module, Strata::RESET_INPUT));
               
        posX = xStart + xGap + leftPad;
        posY = yStart-10.f ;
        addParam(createParamCentered<RoundBlackKnob>(Vec(posX+xGap, posY-10.f ), module, Strata::PATTERN_KNOB));
        posY += 1.75*yUnit+ 1.75*yUnit;
        addParam(createParamCentered<TL1105>(Vec(posX-2, posY ), module, Strata::RESET_BUTTON));

        // Switches
        posX = xStart + xUnit + xGap * 0.75f;
        posY = yStart + (6 + 4.5f * 0) * yUnit + 5.0f;
        posY -= 0.25f * yUnit;     // Only switch 0 has this offset
        addParam(createParamCentered<CKSSThree>(Vec(posX, posY), module, Strata::MAIN_SWITCH + 0));
    
        posX = xStart + xUnit + xGap * 0.75f;
        posY = yStart + (6 + 4.5f * 1) * yUnit + 5.0f;
        addParam(createParamCentered<CKSSThree>(Vec(posX, posY), module, Strata::MAIN_SWITCH + 1));
    
        posX = xStart + xUnit + xGap * 0.75f;
        posY = yStart + (6 + 4.5f * 2) * yUnit + 5.0f;
        addParam(createParamCentered<CKSSThree>(Vec(posX, posY), module, Strata::MAIN_SWITCH + 2));

        // Layers Box
        posX = xStart  + leftPad;
        posY = yStart + (4.5)*yUnit;
        addParam(createParamCentered<TL1105>(Vec(posX, posY +.25*yUnit)           , module, Strata::LAYER_1_BUTTON));        
        addParam(createParamCentered<TL1105>(Vec(posX, posY + 1.666*yUnit), module, Strata::LAYER_2_BUTTON));        
        addParam(createParamCentered<TL1105>(Vec(posX, posY + 3.083*yUnit), module, Strata::LAYER_3_BUTTON));        
        addParam(createParamCentered<TL1105>(Vec(posX, posY + 4.5*yUnit), module, Strata::LAYER_4_BUTTON));        
        addChild(createLightCentered<LargeLight<BlueLight>>   (Vec(posX, posY + .25*yUnit )           , module, Strata::LAYER_1_LIGHT));
        addChild(createLightCentered<LargeLight<YellowLight>> (Vec(posX, posY + 1.666*yUnit), module, Strata::LAYER_2_LIGHT));
        addChild(createLightCentered<LargeLight<RedLight>>  (Vec(posX, posY + 3.083*yUnit), module, Strata::LAYER_3_LIGHT));
        addChild(createLightCentered<LargeLight<GreenLight>>(Vec(posX, posY + 4.5*yUnit), module, Strata::LAYER_4_LIGHT));
        // Layer CV
        posX = xStart + xGap + leftPad;
        posY = yStart + 10.5*yUnit;
        addParam(createParamCentered<TL1105>(Vec(posX, posY ), module, Strata::LAYER_NEXT_BUTTON));        
        posX = xStart - xGap + leftPad;
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(posX, posY ), module, Strata::LAYER_INPUT));

        // Offset
        posX = xStart + xGap + leftPad;
        posY = yStart + 13*yUnit;
        addParam(createParamCentered<Trimpot>(Vec(posX, posY ), module, Strata::OFFSET_PARAM));
        posX = xStart - xGap + leftPad;
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(posX, posY ), module, Strata::OFFSET_INPUT));

        // On/Off
        posX = xStart + leftPad;
        posY = yStart + 16*yUnit;
        addParam(createParamCentered<CKSSThreeHorizontal>(Vec(posX, posY ), module, Strata::ON_SWITCH));

        // Beat Sequencer        
        for (int i = 0; i<8; i++){
            posX = xStart + (i+2)*xUnit;
            posY = yStart-12.f + yUnit;
            addParam(createParamCentered<TL1105>(Vec(posX-xGap*.6, posY ), module, Strata::STAGE_1_BEATS_UP + i));
            addParam(createParamCentered<TL1105>(Vec(posX-xGap*.6, posY + 2*yUnit), module, Strata::STAGE_1_BEATS_DOWN + i));
            
            addParam(createParamCentered<TL1105>(Vec(posX+xGap*.6, posY ), module, Strata::STAGE_1_STEPS_UP + i));
            addParam(createParamCentered<TL1105>(Vec(posX+xGap*.6, posY + 2*yUnit), module, Strata::STAGE_1_STEPS_DOWN + i));

            ratioBeatsDisplays[i] = createDigitalDisplay(( Vec(posX-xGap*0.6 + (dispX)*xUnit, posY + (1+dispY)*yUnit ) ), "1");
            addChild(ratioBeatsDisplays[i]);

            ratioStagesDisplays[i] = createDigitalDisplay((Vec(posX+xGap*0.6 + (dispX)*xUnit, posY + (1+dispY)*yUnit ) ), "1");
            addChild(ratioStagesDisplays[i]);

            addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(posX, posY + 2.6*yUnit ), module, Strata::STAGE_1_LIGHT + i));
        }

        float patSpace = xUnit*.333333;
        //Pattern Buttons
        for (int i=0; i<24; i++){
            posX = xStart + 1.67*xUnit + i*patSpace;
            posY = yStart-12.f ;
            addParam(createParamCentered<TL1105>(Vec(posX, posY ), module, Strata::PATTERN_1_BUTTON + i));
            addChild(createLightCentered<SmallLight<WhiteLight>>(Vec(posX-patSpace/5, posY ), module, Strata::PATTERN_1_LIGHT_Y + i));
            addChild(createLightCentered<SmallLight<WhiteLight>>(Vec(posX+patSpace/5, posY ), module, Strata::PATTERN_1_LIGHT_B + i));            
            addChild(createLightCentered<LargeLight<WhiteLight>>(Vec(posX, posY ), module, Strata::PATTERN_1_LIGHT_W + i));            
        }

        //semitones beat
        posX = xStart + (2)*xUnit;
        posY = yStart + 10.*yUnit - 5.0;
        addParam(createParamCentered<TL1105>(Vec(posX-xGap*.6, posY ), module, Strata::SEMI_BEATS_UP ));
        addParam(createParamCentered<TL1105>(Vec(posX-xGap*.6, posY + 2*yUnit), module, Strata::SEMI_BEATS_DOWN ));
        addParam(createParamCentered<TL1105>(Vec(posX+xGap*.6, posY ), module, Strata::SEMI_STEPS_UP ));
        addParam(createParamCentered<TL1105>(Vec(posX+xGap*.6, posY + 2*yUnit), module, Strata::SEMI_STEPS_DOWN ));

        ratioBeatsDisplays[8] = createDigitalDisplay(( Vec(posX-xGap*0.6 + (dispX)*xUnit, posY + (1+dispY)*yUnit -2) ), "1");
        addChild(ratioBeatsDisplays[8]);
        ratioStagesDisplays[8] = createDigitalDisplay((Vec(posX+xGap*0.6 + (dispX)*xUnit, posY + (1+dispY)*yUnit -2) ), "8");
        addChild(ratioStagesDisplays[8]);

        //octaves beat
        posX = xStart + (2)*xUnit;
        posY = yStart + 14.5*yUnit - 5.0;
        addParam(createParamCentered<TL1105>(Vec(posX-xGap*.6, posY ), module, Strata::OCT_BEATS_UP ));
        addParam(createParamCentered<TL1105>(Vec(posX-xGap*.6, posY + 2*yUnit), module, Strata::OCT_BEATS_DOWN ));
        addParam(createParamCentered<TL1105>(Vec(posX+xGap*.6, posY ), module, Strata::OCT_STEPS_UP ));
        addParam(createParamCentered<TL1105>(Vec(posX+xGap*.6, posY + 2*yUnit), module, Strata::OCT_STEPS_DOWN ));

        ratioBeatsDisplays[9] = createDigitalDisplay(( Vec(posX-xGap*0.6 + (dispX)*xUnit, posY + (1+dispY)*yUnit -2) ), "1");
        addChild(ratioBeatsDisplays[9]);
        ratioStagesDisplays[9] = createDigitalDisplay((Vec(posX+xGap*0.6 + (dispX)*xUnit, posY + (1+dispY)*yUnit -2) ), "1");
        addChild(ratioStagesDisplays[9]);

        // Main Sequencer        
        for (int i = 0; i<8; i++){
            posX = xStart + (i+2)*xUnit;
            posY = yStart + 6*yUnit;

            addParam(createParamCentered<RoundLargeBlackKnob>(Vec(posX, posY  ), module, Strata::SEQ_1_KNOB + i));
            addParam(createParamCentered<TL1105>(Vec(posX, posY ), module, Strata::SEQ_1_BUTTON + i));

            addChild(createLightCentered<LargeLight<RedLight>>   (Vec(posX, posY ), module, Strata::SEQ_1_LIGHT_R + i));
            addChild(createLightCentered<LargeLight<GreenLight>> (Vec(posX, posY ), module, Strata::SEQ_1_LIGHT_G + i));
            addChild(createLightCentered<LargeLight<BlueLight>>  (Vec(posX, posY ), module, Strata::SEQ_1_LIGHT_B + i));
            addChild(createLightCentered<LargeLight<YellowLight>>(Vec(posX, posY ), module, Strata::SEQ_1_LIGHT_Y + i));

            addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(posX, posY + 1.5*yUnit ), module, Strata::SEQ_1_LIGHT_W + i));

            posX = xStart + (i+2+dispX)*xUnit;
            posY = yStart + (4.5+dispY)*yUnit;
            noteDisplays[i] = createDigitalDisplay((Vec(posX, posY)), "C4");
            addChild(noteDisplays[i]);
        }    

        // Semitone Offsets Sequencer
        for (int i = 0; i<7; i++){
            posX = xStart + (i+3)*xUnit;
            posY = yStart + 10.5*yUnit;

            addParam(createParamCentered<RoundLargeBlackKnob>(Vec(posX, posY  ), module, Strata::SEMI_1_KNOB + i));
            addParam(createParamCentered<TL1105>(Vec(posX, posY ), module, Strata::SEMI_1_BUTTON + i));

            addChild(createLightCentered<LargeLight<RedLight>>   (Vec(posX, posY ), module, Strata::SEMI_1_LIGHT_R + i));
            addChild(createLightCentered<LargeLight<GreenLight>> (Vec(posX, posY ), module, Strata::SEMI_1_LIGHT_G + i));
            addChild(createLightCentered<LargeLight<BlueLight>>  (Vec(posX, posY ), module, Strata::SEMI_1_LIGHT_B + i));
            addChild(createLightCentered<LargeLight<YellowLight>>(Vec(posX, posY ), module, Strata::SEMI_1_LIGHT_Y + i));

            addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(posX, posY + 1.5*yUnit ), module, Strata::SEMI_1_LIGHT_W + i));

            posX = xStart + (i+3+dispX)*xUnit;
            posY = yStart + (9+dispY)*yUnit;
            semiDisplays[i] = createDigitalDisplay((Vec(posX, posY)), "+0");
            addChild(semiDisplays[i]);
        }    

        // Octaves Sequencer
        for (int i = 0; i<4; i++){
            posX = xStart + (i+3)*xUnit;
            posY = yStart + 15*yUnit;
            addParam(createParamCentered<RoundLargeBlackKnob>(Vec(posX, posY  ), module, Strata::OCT_1_KNOB + i));
            addParam(createParamCentered<TL1105>(Vec(posX, posY ), module, Strata::OCT_1_BUTTON + i));

            addChild(createLightCentered<LargeLight<RedLight>>   (Vec(posX, posY ), module, Strata::OCT_1_LIGHT_R + i));
            addChild(createLightCentered<LargeLight<GreenLight>> (Vec(posX, posY ), module, Strata::OCT_1_LIGHT_G + i));
            addChild(createLightCentered<LargeLight<BlueLight>>  (Vec(posX, posY ), module, Strata::OCT_1_LIGHT_B + i));
            addChild(createLightCentered<LargeLight<YellowLight>>(Vec(posX, posY ), module, Strata::OCT_1_LIGHT_Y + i));

            addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(posX, posY + 1.5*yUnit ), module, Strata::OCT_1_LIGHT_W + i));

            posX = xStart + (i+3+dispX)*xUnit;
            posY = yStart + (13.5+dispY)*yUnit;
            octDisplays[i] = createDigitalDisplay((Vec(posX, posY)), "+0");
            addChild(octDisplays[i]);
        }    

        // Outputs
        posX = xStart + 8*xUnit;
        posY = yStart + 16*yUnit;
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(posX, posY ), module, Strata::GATE_OUTPUT));
        posX += 2*xGap;       
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(posX, posY ), module, Strata::MAIN_OUTPUT));
        posX -= 4*xGap;        
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(posX, posY ), module, Strata::INV_GATE_OUTPUT));
        
        posX = xStart + (8+dispX)*xUnit;
        posY = yStart + (13.75+dispY)*yUnit;
        outputDisplay = createDigitalDisplay((Vec(posX, posY)), "C4");
        addChild(outputDisplay);

    }

    void step() override {
        Strata* module = dynamic_cast<Strata*>(this->module);
        if (!module) return;

        // Update ratio displays
        for (int i = 0; i < STAGES+2; i++) {
        
            DigitalDisplay* beatsDisp  = ratioBeatsDisplays[i];
            DigitalDisplay* stagesDisp = ratioStagesDisplays[i];
        
            if (beatsDisp && stagesDisp) {
        
                // Colors: dimmed or gold depending on whether this is the current stage
                NVGcolor dimColor  = nvgRGB(154, 105, 65);
                NVGcolor goldColor = nvgRGB(208, 140, 89);
                bool isCurrent = (module->currentStage == i);
        
                beatsDisp->fgColor  =
                stagesDisp->fgColor = isCurrent ? goldColor : dimColor;
        
                // Read values
                int num = static_cast<int>(module->multiply[i][module->strataLayer]);
                int den = static_cast<int>(module->divide[i][module->strataLayer]);
        
                // Assign text
                if (den != 0) {
                    // Standard ratio
                    char numText[8];
                    char denText[8];
                    snprintf(numText, sizeof(numText), "%d", num);
                    snprintf(denText, sizeof(denText), "%d", den);
        
                    beatsDisp->text  = numText;   // numerator
                    stagesDisp->text = denText;   // denominator
                }
                else {
                    // Denominator = 0 → show "off"
                    beatsDisp->text  = "-";
                    stagesDisp->text = "-";
                }
            }
        }        
        // Handle LIGHTS
        for (int i = 0; i < PATTERNS; i++){
            if (i < module->patternStages){
                if (module->patternState[i][module->strataLayer] == 0){ //state=0 left
                    module->lights[Strata::PATTERN_1_LIGHT_Y + i].setBrightness( 0.f); 
                    module->lights[Strata::PATTERN_1_LIGHT_B + i].setBrightness( 0.f); 
                    module->lights[Strata::PATTERN_1_LIGHT_W + i].setBrightness( 0.f); 
                    if (i == module->patternIndex){
                        module->lights[Strata::PATTERN_1_LIGHT_Y + i].setBrightness( 1.f); 
                        module->lights[Strata::PATTERN_1_LIGHT_W + i].setBrightness( 0.7f);
                    } else {
                        module->lights[Strata::PATTERN_1_LIGHT_Y + i].setBrightness( 0.5f);
                        module->lights[Strata::PATTERN_1_LIGHT_W + i].setBrightness( 0.2f);
                    }
                } else if (module->patternState[i][module->strataLayer] == 1){ //state=1 right
                    module->lights[Strata::PATTERN_1_LIGHT_Y + i].setBrightness( 0.f); 
                    module->lights[Strata::PATTERN_1_LIGHT_B + i].setBrightness( 0.f); 
                    module->lights[Strata::PATTERN_1_LIGHT_W + i].setBrightness( 0.f); 
                    if (i == module->patternIndex){
                        module->lights[Strata::PATTERN_1_LIGHT_B + i].setBrightness( 1.f); 
                        module->lights[Strata::PATTERN_1_LIGHT_W + i].setBrightness( 0.7f);
                    } else {
                        module->lights[Strata::PATTERN_1_LIGHT_B + i].setBrightness( 0.5f);
                        module->lights[Strata::PATTERN_1_LIGHT_W + i].setBrightness( 0.2f);
                    }
                } else { //state=2 off
                    module->lights[Strata::PATTERN_1_LIGHT_Y + i].setBrightness( 0.f); 
                    module->lights[Strata::PATTERN_1_LIGHT_B + i].setBrightness( 0.f); 
                    module->lights[Strata::PATTERN_1_LIGHT_W + i].setBrightness( 0.f); 
                    if (i == module->patternIndex){
                        module->lights[Strata::PATTERN_1_LIGHT_Y + i].setBrightness( 0.f); 
                        module->lights[Strata::PATTERN_1_LIGHT_B + i].setBrightness( 0.f); 
                        module->lights[Strata::PATTERN_1_LIGHT_W + i].setBrightness( 0.7f);
                    } else {
                        module->lights[Strata::PATTERN_1_LIGHT_Y + i].setBrightness( 0.f); 
                        module->lights[Strata::PATTERN_1_LIGHT_B + i].setBrightness( 0.f); 
                        module->lights[Strata::PATTERN_1_LIGHT_W + i].setBrightness( 0.05f);

                    }
                }
            } else {
                    module->lights[Strata::PATTERN_1_LIGHT_Y + i].setBrightness( 0.f); 
                    module->lights[Strata::PATTERN_1_LIGHT_B + i].setBrightness( 0.f); 
                    module->lights[Strata::PATTERN_1_LIGHT_W + i].setBrightness( 0.f); 
            }
        }

        // Stage Lights
        for (int i = 0; i < STAGES; i++){ 
            module->lights[Strata::STAGE_1_LIGHT + i].setBrightness( 0.f); 
        }

        module->selectedStage = clamp(module->selectedStage, 0, STAGES-1); //clamp stage in case bad data loaded from JSON
        module->lights[Strata::STAGE_1_LIGHT + module->selectedStage].setBrightness( 1.0f); //illuminate selected stage.

        if (module->blinkDON){
            module->blinkDON= false; //reset blink token
            module->lights[Strata::GATE_LIGHT].setBrightness( 1.f);
        }        

        if (module->blinkKA){
            module->blinkKA = false; //reset blink token
            module->lights[Strata::GATE_LIGHT].setBrightness( 1.f);
        } 
        
        float dim = module->lights[Strata::GATE_LIGHT].getBrightness();
        module->lights[Strata::GATE_LIGHT].setBrightness( dim * .8f);
        
        // Sequencer Lights
        for (int i=0; i<19; i++){       
            if (module->strataLayer==0){
                module->lights[Strata::SEQ_1_LIGHT_R + i].setBrightness(0);
                module->lights[Strata::SEQ_1_LIGHT_G + i].setBrightness(0);
                module->lights[Strata::SEQ_1_LIGHT_B + i].setBrightness(module->buttonStates[i][0] ? 1.f : 0.f);
                module->lights[Strata::SEQ_1_LIGHT_Y + i].setBrightness(0);
            } else if (module->strataLayer==1){
                module->lights[Strata::SEQ_1_LIGHT_R + i].setBrightness(0);
                module->lights[Strata::SEQ_1_LIGHT_G + i].setBrightness(0);
                module->lights[Strata::SEQ_1_LIGHT_B + i].setBrightness(0);
                module->lights[Strata::SEQ_1_LIGHT_Y + i].setBrightness(module->buttonStates[i][1] ? 1.f : 0.f);
            } else if (module->strataLayer==2){
                module->lights[Strata::SEQ_1_LIGHT_R + i].setBrightness(module->buttonStates[i][2] ? 1.f : 0.f);
                module->lights[Strata::SEQ_1_LIGHT_G + i].setBrightness(0);
                module->lights[Strata::SEQ_1_LIGHT_B + i].setBrightness(0);
                module->lights[Strata::SEQ_1_LIGHT_Y + i].setBrightness(0);
            } else {
                module->lights[Strata::SEQ_1_LIGHT_R + i].setBrightness(0);
                module->lights[Strata::SEQ_1_LIGHT_G + i].setBrightness(module->buttonStates[i][3] ? 1.f : 0.f);
                module->lights[Strata::SEQ_1_LIGHT_B + i].setBrightness(0);
                module->lights[Strata::SEQ_1_LIGHT_Y + i].setBrightness(0);
            }             
            float brightness = 0.f;
            if (i<4){ //left sequencer
                if (i==module->activeStage){
                    brightness = 1.f;
                } else if (i==module->leftStage){
                    brightness = 0.15f;
                } else {
                    brightness = 0.f;
                }
                module->lights[Strata::SEQ_1_LIGHT_W + i].setBrightness( brightness );                              
            } else if (i<8) { //right sequencer
                if (i==module->activeStage){
                    brightness = 1.f;
                } else if (i-4==module->rightStage){
                    brightness = 0.15f;
                } else {
                    brightness = 0.f;
                }
                module->lights[Strata::SEQ_1_LIGHT_W + i].setBrightness( brightness ); 
            } else if (i<15) { //semitones
                if (i-8 == module->semiStage){
                    brightness = 1.0f;
                } else {
                    brightness = 0.f;
                }
                if (module->divide[8][module->strataLayer]==0) brightness = 0;
                module->lights[Strata::SEQ_1_LIGHT_W + i].setBrightness( brightness );                 
            } else {  //octaves
                if (i-15 == module->octStage){
                    brightness = 1.0f;
                } else {
                    brightness = 0.f;
                }
                if ( module->divide[9][module->strataLayer]==0) brightness = 0;
                module->lights[Strata::SEQ_1_LIGHT_W + i].setBrightness( brightness );                 
            
            }            
        }
            
        if (module->strataLayer==0){
            module->lights[Strata::LAYER_1_LIGHT].setBrightness(1);
            module->lights[Strata::LAYER_2_LIGHT].setBrightness(0);
            module->lights[Strata::LAYER_3_LIGHT].setBrightness(0);
            module->lights[Strata::LAYER_4_LIGHT].setBrightness(0);
        } else if (module->strataLayer==1){
            module->lights[Strata::LAYER_1_LIGHT].setBrightness(0);
            module->lights[Strata::LAYER_2_LIGHT].setBrightness(1);
            module->lights[Strata::LAYER_3_LIGHT].setBrightness(0);
            module->lights[Strata::LAYER_4_LIGHT].setBrightness(0);
        } else if (module->strataLayer==2){
            module->lights[Strata::LAYER_1_LIGHT].setBrightness(0);
            module->lights[Strata::LAYER_2_LIGHT].setBrightness(0);
            module->lights[Strata::LAYER_3_LIGHT].setBrightness(1);
            module->lights[Strata::LAYER_4_LIGHT].setBrightness(0);
        } else {
            module->lights[Strata::LAYER_1_LIGHT].setBrightness(0);
            module->lights[Strata::LAYER_2_LIGHT].setBrightness(0);
            module->lights[Strata::LAYER_3_LIGHT].setBrightness(0);
            module->lights[Strata::LAYER_4_LIGHT].setBrightness(1);
        } 
        
            
        // Update note displays
        for (int i = 0; i < 8; i++) {
            if (noteDisplays[i]) {
                float pitchVoltage = module->finalNotes[i];
        
                // Convert to semitones
                float exactSemi = pitchVoltage * 12.f;
                int totalSemi   = std::roundf(exactSemi);
        
                // Correct octave using mathematical floor
                int octave = std::floor(totalSemi / 12.f);
        
                // Semitone normalized to 0..11
                int semitone = ((totalSemi % 12) + 12) % 12;
        
                // Shift to your reference (0V = C4)
                octave += 4;
        
                // Note names
                static const char* names[12] =
                    { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
        
                char buf[8];
                snprintf(buf, sizeof(buf), "%s%d", names[semitone], octave);
        
                noteDisplays[i]->text = buf;
            }
        }

        // --- SEMITONE DISPLAYS (7) ---
        for (int s = 0; s < 7; s++) {
            if (semiDisplays[s]) {
                int val = std::roundf(module->finalNotes[8 + s]);
        
                char buf[8];
                snprintf(buf, sizeof(buf), "%+d", val);
        
                semiDisplays[s]->text = buf;
            }
        }

        // --- OCTAVE DISPLAYS (4) ---
        for (int o = 0; o < 4; o++) {
            if (octDisplays[o]) {
                int val = std::roundf(module->finalNotes[15 + o]);
        
                char buf[8];
                snprintf(buf, sizeof(buf), "%+d", val);
        
                octDisplays[o]->text = buf;
            }
        }

        //Output Note display
        if (outputDisplay) {
            float pitchVoltage = module->outputs[Strata::MAIN_OUTPUT].getVoltage();
    
            // Convert to semitones
            float exactSemi = pitchVoltage * 12.f;

            int totalSemi = std::roundf(exactSemi);
            
            // True mathematical floor for octave:
            int octave = std::floor(totalSemi / 12.f);
            
            // Semitone as 0–11:
            int semitone = ((totalSemi % 12) + 12) % 12;
            
            // Reference shift (0V = C4)
            octave += 4;
            
            static const char* names[12] =
                { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
            
            char buf[8];
            snprintf(buf, sizeof(buf), "%s%d", names[semitone], octave);
            
            outputDisplay->text = buf;
        }


        ModuleWidget::step();         
    }  

    DigitalDisplay* createDigitalDisplay(Vec position, std::string initialValue) {
        DigitalDisplay* display = new DigitalDisplay();
        display->box.pos = position;
        display->box.size = Vec(28.32, 17.76);
        display->text = initialValue;
        display->fgColor = nvgRGB(208, 140, 89); // Gold color text
        display->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        display->setFontSize(14.0f);
        return display;
    }
    
    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
    
        Strata* strataModule = dynamic_cast<Strata*>(module);
        assert(strataModule); // Ensure the cast succeeds
    
        // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());
     
        // Copy Layer menu item
        struct CopyLayerMenuItem : MenuItem {
            Strata* strataModule;
            void onAction(const event::Action& e) override {
                for (int i = 0; i < 19; i++) {
                    strataModule->copiedKnobStates[i] = strataModule->knobStates[i][strataModule->strataLayer];
                    strataModule->copiedButtonStates[i] = strataModule->buttonStates[i][strataModule->strataLayer];                    
                }
                for (int i = 0; i < 3; i++) {                
                    strataModule->copiedSwitchStates[i] = strataModule->switchStates[i][strataModule->strataLayer];
                }
                
                for (int i = 0; i < 10; i++) { //always copy everything, even if we don't paste all layers
                    strataModule->copiedDivide[i] = strataModule->divide[i][strataModule->strataLayer];
                    strataModule->copiedMultiply[i] = strataModule->multiply[i][strataModule->strataLayer];
                } 
                for (int i=0; i<PATTERNS; i++){
                    strataModule->copiedPatternState[i] = strataModule->patternState[i][strataModule->strataLayer];
                }
                strataModule->copiedPatternKnob = strataModule->patternKnob[strataModule->strataLayer];                              
                strataModule->copyBufferFilled = true;
            }
            void step() override {
                rightText = strataModule->copyBufferFilled ? "✔" : "";
                MenuItem::step();
            }
        };
        
        CopyLayerMenuItem* copyLayerItem = new CopyLayerMenuItem();
        copyLayerItem->text = "Copy Layer";
        copyLayerItem->strataModule = strataModule;
        menu->addChild(copyLayerItem);
        
        // Paste Layer menu item
        struct PasteLayerMenuItem : MenuItem {
            Strata* strataModule;
            void onAction(const event::Action& e) override {
                if (!strataModule->copyBufferFilled) return;
                switch (strataModule->copyMode) {
                    case Strata::PasteMode::KnobsAndBeats:
                        for (int i = 0; i < 19; i++) { //KNOBS
                            strataModule->knobStates[i][strataModule->strataLayer] = strataModule->copiedKnobStates[i];
                            strataModule->buttonStates[i][strataModule->strataLayer] = strataModule->copiedButtonStates[i];
                        }
                        for (int i = 0; i < 3; i++) { //KNOBS-SWITCHES
                            strataModule->switchStates[i][strataModule->strataLayer] = strataModule->copiedSwitchStates[i];
                        }                                
                        for (int i = 0; i < 10; i++) { //BEATS
                            strataModule->divide[i][strataModule->strataLayer] = strataModule->copiedDivide[i];
                            strataModule->multiply[i][strataModule->strataLayer] = strataModule->copiedMultiply[i];
                        }  
                        for (int i = 0; i < PATTERNS; i++) { //BEATS-PATTERNS
                            strataModule->patternState[i][strataModule->strataLayer] = strataModule->copiedPatternState[i];
                        }
                        strataModule->patternKnob[strataModule->strataLayer] = strataModule->copiedPatternKnob;
                        break;
                
                    case Strata::PasteMode::KnobsOnly:
                        for (int i = 0; i < 19; i++) { //KNOBS
                            strataModule->knobStates[i][strataModule->strataLayer] = strataModule->copiedKnobStates[i];
                            strataModule->buttonStates[i][strataModule->strataLayer] = strataModule->copiedButtonStates[i];
                        }
                        for (int i = 0; i < 3; i++) { //KNOBS-SWITCHES
                            strataModule->switchStates[i][strataModule->strataLayer] = strataModule->copiedSwitchStates[i];
                        }                                
                        break;
                
                    case Strata::PasteMode::BeatsOnly:
                        for (int i = 0; i < 10; i++) { //BEATS
                            strataModule->divide[i][strataModule->strataLayer] = strataModule->copiedDivide[i];
                            strataModule->multiply[i][strataModule->strataLayer] = strataModule->copiedMultiply[i];
                        }  
                        for (int i = 0; i < PATTERNS; i++) { //BEATS-PATTERNS
                            strataModule->patternState[i][strataModule->strataLayer] = strataModule->copiedPatternState[i];
                        }  
                        strataModule->patternKnob[strataModule->strataLayer] = strataModule->copiedPatternKnob;
                        break;
                }

                strataModule->displayUpdate = true;
            }
            void step() override {
                rightText = strataModule->copyBufferFilled ? "Ready" : "Empty";
                MenuItem::step();
            }
        };
        
        PasteLayerMenuItem* pasteLayerItem = new PasteLayerMenuItem();
        pasteLayerItem->text = "Paste Layer";
        pasteLayerItem->strataModule = strataModule;
        menu->addChild(pasteLayerItem);

        // Paste to All Layers menu item (updated to use PasteMode)
        struct PasteAllLayersMenuItem : MenuItem {
            Strata* strataModule;
            void onAction(const event::Action& e) override {
                if (!strataModule->copyBufferFilled) return;
        
                for (int z = 0; z < 4; z++) {
                    switch (strataModule->copyMode) {
                        case Strata::PasteMode::KnobsAndBeats:
                            for (int i = 0; i < 19; i++) {
                                strataModule->knobStates[i][z] = strataModule->copiedKnobStates[i];
                                strataModule->buttonStates[i][z] = strataModule->copiedButtonStates[i];
                            }
                            for (int i = 0; i < 3; i++) {
                                strataModule->switchStates[i][z] = strataModule->copiedSwitchStates[i];
                            }
                            for (int i = 0; i < 10; i++) {
                                strataModule->divide[i][z] = strataModule->copiedDivide[i];
                                strataModule->multiply[i][z] = strataModule->copiedMultiply[i];
                            }
                            for (int i = 0; i < PATTERNS; i++) {
                                strataModule->patternState[i][z] = strataModule->copiedPatternState[i];
                            }
                            strataModule->patternKnob[z] = strataModule->copiedPatternKnob;
                            break;
        
                        case Strata::PasteMode::KnobsOnly:
                            for (int i = 0; i < 19; i++) {
                                strataModule->knobStates[i][z] = strataModule->copiedKnobStates[i];
                                strataModule->buttonStates[i][z] = strataModule->copiedButtonStates[i];
                            }
                            for (int i = 0; i < 3; i++) {
                                strataModule->switchStates[i][z] = strataModule->copiedSwitchStates[i];
                            }
                            break;
        
                        case Strata::PasteMode::BeatsOnly:
                            for (int i = 0; i < 10; i++) {
                                strataModule->divide[i][z] = strataModule->copiedDivide[i];
                                strataModule->multiply[i][z] = strataModule->copiedMultiply[i];
                            }
                            for (int i = 0; i < PATTERNS; i++) {
                                strataModule->patternState[i][z] = strataModule->copiedPatternState[i];
                            }
                            strataModule->patternKnob[z] = strataModule->copiedPatternKnob;
                            break;
                    }
                }
        
                strataModule->displayUpdate = true;
            }
        
            void step() override {
                rightText = strataModule->copyBufferFilled ? "Ready" : "Empty";
                MenuItem::step();
            }
        };
        
        PasteAllLayersMenuItem* pasteAllLayersItem = new PasteAllLayersMenuItem();
        pasteAllLayersItem->text = "Paste to All Layers";
        pasteAllLayersItem->strataModule = strataModule;
        menu->addChild(pasteAllLayersItem);


        // --- Separator for the new toggles ---
        menu->addChild(new MenuSeparator());
        
        struct PasteModeMenuItem : MenuItem {
            Strata* strataModule;
            Strata::PasteMode mode;
        
            void onAction(const event::Action& e) override {
                strataModule->copyMode = mode;
            }
        
            void step() override {
                rightText = (strataModule->copyMode == mode) ? "✔" : "";
                MenuItem::step();
            }
        };
        
        // Add to menu
        menu->addChild(new MenuSeparator());
        MenuLabel* label = new MenuLabel();
        label->text = "Paste Mode";
        menu->addChild(label);
        
        PasteModeMenuItem* knobsAndBeatsItem = new PasteModeMenuItem();
        knobsAndBeatsItem->text = "Knobs + Beats";
        knobsAndBeatsItem->strataModule = strataModule;
        knobsAndBeatsItem->mode = Strata::PasteMode::KnobsAndBeats;
        menu->addChild(knobsAndBeatsItem);
        
        PasteModeMenuItem* knobsOnlyItem = new PasteModeMenuItem();
        knobsOnlyItem->text = "Knobs Only";
        knobsOnlyItem->strataModule = strataModule;
        knobsOnlyItem->mode = Strata::PasteMode::KnobsOnly;
        menu->addChild(knobsOnlyItem);
        
        PasteModeMenuItem* beatsOnlyItem = new PasteModeMenuItem();
        beatsOnlyItem->text = "Beats Only";
        beatsOnlyItem->strataModule = strataModule;
        beatsOnlyItem->mode = Strata::PasteMode::BeatsOnly;
        menu->addChild(beatsOnlyItem);

        // --- Separator ---
        menu->addChild(new MenuSeparator());

        // Toggle: Layer CV Mode
        struct LayerCVModeMenuItem : MenuItem {
            Strata* strataModule;
            void onAction(const event::Action& e) override {
                strataModule->layerCVmode = !strataModule->layerCVmode;
            }
            void step() override {
                rightText = strataModule->layerCVmode ? "✔" : "";
                MenuItem::step();
            }
        };
        
        auto* layerCVmodeItem = new LayerCVModeMenuItem();
        layerCVmodeItem->text = "Layer Advance by CV (1V/layer)";
        layerCVmodeItem->strataModule = strataModule;
        menu->addChild(layerCVmodeItem);
        
    } 
          
};
Model* modelStrata = createModel<Strata, StrataWidget>("Strata");