////////////////////////////////////////////////////////////
//
//   Cartesia
//
//   written by Cody Geary
//   Copyright 2025, MIT License
//
//   A 4x4x4 sequencer for both quantized and non-quantized voltages, 
//   gates per step, and polyphonic outputs with stacked z-layer options.
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
#include "digital_display.hpp"
#include <array>
#include <string>
#include <vector>
#include <cmath>

struct Cartesia : Module {
    // Parameters
    enum ParamId {
        // Slice buttons
        SLICE1BUTTON_PARAM, SLICE2BUTTON_PARAM, SLICE3BUTTON_PARAM, SLICE4BUTTON_PARAM,

        // Knobs (4x4 grid x,y)
        KNOB00_PARAM, KNOB10_PARAM, KNOB20_PARAM, KNOB30_PARAM,
        KNOB01_PARAM, KNOB11_PARAM, KNOB21_PARAM, KNOB31_PARAM,
        KNOB02_PARAM, KNOB12_PARAM, KNOB22_PARAM, KNOB32_PARAM,
        KNOB03_PARAM, KNOB13_PARAM, KNOB23_PARAM, KNOB33_PARAM,

        // Stage Buttons
        KNOB00_BUTTON, KNOB10_BUTTON, KNOB20_BUTTON, KNOB30_BUTTON,
        KNOB01_BUTTON, KNOB11_BUTTON, KNOB21_BUTTON, KNOB31_BUTTON,
        KNOB02_BUTTON, KNOB12_BUTTON, KNOB22_BUTTON, KNOB32_BUTTON,
        KNOB03_BUTTON, KNOB13_BUTTON, KNOB23_BUTTON, KNOB33_BUTTON,

        // Other control buttons
        XFWDBUTTON_PARAM, YFWDBUTTON_PARAM, ZFWDBUTTON_PARAM,
        SCANFWDBUTTON_PARAM, QUANTIZEBUTTON_PARAM, ONBUTTON_PARAM,
        RESETBUTTON_PARAM, RANDOMBUTTON_PARAM, POLYKNOB_PARAM,

        // Range controls
        MIN_PARAM, RANGE_PARAM, MINATT_PARAM, RANGEATT_PARAM,

        PARAMS_LEN
    };

    // Inputs
    enum InputId {
        XCV_INPUT, XREV_INPUT, XFWD_INPUT,
        YCV_INPUT, YREV_INPUT, YFWD_INPUT,
        ZCV_INPUT, ZREV_INPUT, ZFWD_INPUT,

        MINCV_INPUT, RANGECV_INPUT, SCANCV_INPUT,
        SCANREV_INPUT, SCANFWD_INPUT,
        ONOFF_INPUT, RESET_INPUT, RANDOM_INPUT, OFFSET_INPUT,

        INPUTS_LEN
    };

    // Outputs
    enum OutputId {
        RESET_OUTPUT, TRIGGER_OUTPUT, GATEOUT_OUTPUT, INVGATEOUT_OUTPUT,
        OUTPUT_OUTPUT,

        OUTPUTS_LEN
    };

    // Lights (4x4x4 LED Grid + other indicators)
    enum LightId {
        // LED Grid (x, y, z positioning)
        LED000_LIGHT, LED001_LIGHT, LED002_LIGHT, LED003_LIGHT,
        LED010_LIGHT, LED011_LIGHT, LED012_LIGHT, LED013_LIGHT,
        LED020_LIGHT, LED021_LIGHT, LED022_LIGHT, LED023_LIGHT,
        LED030_LIGHT, LED031_LIGHT, LED032_LIGHT, LED033_LIGHT,

        LED100_LIGHT, LED101_LIGHT, LED102_LIGHT, LED103_LIGHT,
        LED110_LIGHT, LED111_LIGHT, LED112_LIGHT, LED113_LIGHT,
        LED120_LIGHT, LED121_LIGHT, LED122_LIGHT, LED123_LIGHT,
        LED130_LIGHT, LED131_LIGHT, LED132_LIGHT, LED133_LIGHT,

        LED200_LIGHT, LED201_LIGHT, LED202_LIGHT, LED203_LIGHT,
        LED210_LIGHT, LED211_LIGHT, LED212_LIGHT, LED213_LIGHT,
        LED220_LIGHT, LED221_LIGHT, LED222_LIGHT, LED223_LIGHT,
        LED230_LIGHT, LED231_LIGHT, LED232_LIGHT, LED233_LIGHT,

        LED300_LIGHT, LED301_LIGHT, LED302_LIGHT, LED303_LIGHT,
        LED310_LIGHT, LED311_LIGHT, LED312_LIGHT, LED313_LIGHT,
        LED320_LIGHT, LED321_LIGHT, LED322_LIGHT, LED323_LIGHT,
        LED330_LIGHT, LED331_LIGHT, LED332_LIGHT, LED333_LIGHT,

        // Stage indicator lights
        STAGE00_LIGHT, STAGE10_LIGHT, STAGE20_LIGHT, STAGE30_LIGHT,
        STAGE01_LIGHT, STAGE11_LIGHT, STAGE21_LIGHT, STAGE31_LIGHT,
        STAGE02_LIGHT, STAGE12_LIGHT, STAGE22_LIGHT, STAGE32_LIGHT,
        STAGE03_LIGHT, STAGE13_LIGHT, STAGE23_LIGHT, STAGE33_LIGHT,

        // Knob indicator LIGHTs
        KNOB00_LIGHT_R, KNOB10_LIGHT_R, KNOB20_LIGHT_R, KNOB30_LIGHT_R,
        KNOB01_LIGHT_R, KNOB11_LIGHT_R, KNOB21_LIGHT_R, KNOB31_LIGHT_R,
        KNOB02_LIGHT_R, KNOB12_LIGHT_R, KNOB22_LIGHT_R, KNOB32_LIGHT_R,
        KNOB03_LIGHT_R, KNOB13_LIGHT_R, KNOB23_LIGHT_R, KNOB33_LIGHT_R,

        KNOB00_LIGHT_G, KNOB10_LIGHT_G, KNOB20_LIGHT_G, KNOB30_LIGHT_G,
        KNOB01_LIGHT_G, KNOB11_LIGHT_G, KNOB21_LIGHT_G, KNOB31_LIGHT_G,
        KNOB02_LIGHT_G, KNOB12_LIGHT_G, KNOB22_LIGHT_G, KNOB32_LIGHT_G,
        KNOB03_LIGHT_G, KNOB13_LIGHT_G, KNOB23_LIGHT_G, KNOB33_LIGHT_G,

        KNOB00_LIGHT_B, KNOB10_LIGHT_B, KNOB20_LIGHT_B, KNOB30_LIGHT_B,
        KNOB01_LIGHT_B, KNOB11_LIGHT_B, KNOB21_LIGHT_B, KNOB31_LIGHT_B,
        KNOB02_LIGHT_B, KNOB12_LIGHT_B, KNOB22_LIGHT_B, KNOB32_LIGHT_B,
        KNOB03_LIGHT_B, KNOB13_LIGHT_B, KNOB23_LIGHT_B, KNOB33_LIGHT_B,

        KNOB00_LIGHT_Y, KNOB10_LIGHT_Y, KNOB20_LIGHT_Y, KNOB30_LIGHT_Y,
        KNOB01_LIGHT_Y, KNOB11_LIGHT_Y, KNOB21_LIGHT_Y, KNOB31_LIGHT_Y,
        KNOB02_LIGHT_Y, KNOB12_LIGHT_Y, KNOB22_LIGHT_Y, KNOB32_LIGHT_Y,
        KNOB03_LIGHT_Y, KNOB13_LIGHT_Y, KNOB23_LIGHT_Y, KNOB33_LIGHT_Y,

        SLICE1BUTTON_LIGHT, SLICE2BUTTON_LIGHT, SLICE3BUTTON_LIGHT, SLICE4BUTTON_LIGHT,
        QUANTIZEBUTTON_LIGHT, ONBUTTON_LIGHT,

        POLY1_LIGHT, POLY2_LIGHT, POLY3_LIGHT, POLY4_LIGHT,

        LIGHTS_LEN
    };

    // Global Vars
    int xStage = 0;
    int yStage = 0;
    int zStage = 0;
    int previousXStage = 0;
    int previousYStage = 0;
    int previousZStage = 0;
    bool isSampled = true;    //Sample and Hold active steps
    bool displayUpdate = true; //track if the display need updating
    bool initializing = true;

    float knobStates[16][4] = {{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},
                                {0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},
                                {0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},
                                {0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f},{0.f,0.f,0.f,0.f}};

    bool buttonStates[16][4] = {{true, true, true, true},{true, true, true, true},{true, true, true, true},{true, true, true, true},
                                {true, true, true, true},{true, true, true, true},{true, true, true, true},{true, true, true, true},
                                {true, true, true, true},{true, true, true, true},{true, true, true, true},{true, true, true, true},
                                {true, true, true, true},{true, true, true, true},{true, true, true, true},{true, true, true, true}};
    float finalNotes[16] = {0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f, 0.0f};

    bool sequenceRunning = true;
    bool quantize = true;
    int polyLevels = 1;
    float knobMin = 0.f;
    float knobRange = 5.f;

    dsp::SchmittTrigger onTrigger, resetTrigger, randomTrigger, quantizeTrigger,
                        xRevTrigger, xFwdTrigger, xButtonTrigger,
                        yRevTrigger, yFwdTrigger, yButtonTrigger,
                        zRevTrigger, zFwdTrigger, zButtonTrigger,
                        scanRevTrigger, scanFwdTrigger, onButtonTrigger,
                        scanButtonTrigger, randomButtonTrigger, resetButtonTrigger,
                        sliceTrigger[4], stageTrigger[16];

    dsp::PulseGenerator resetPulse, triggerPulse;
    DigitalDisplay* minDisplay = nullptr;
    DigitalDisplay* maxDisplay = nullptr;
    DigitalDisplay* noteDisplays[16] = {nullptr};

    //For Copy/Paste function
    float copiedKnobStates[16] = {0.f};  
    bool copiedButtonStates[16] = {true};
    bool copyBufferFilled = false;
    bool gateTriggerEnabled = false;

    json_t* dataToJson() override {
        json_t* rootJ = json_object();

        json_object_set_new(rootJ, "knobMin", json_real(knobMin));
        json_object_set_new(rootJ, "knobRange", json_real(knobRange));

        // Save knobStates
        json_t* knobStatesJ = json_array();
        for (int i = 0; i < 16; i++) {
            json_t* rowJ = json_array();
            for (int z = 0; z < 4; z++) {
                json_array_append_new(rowJ, json_real(knobStates[i][z]));
            }
            json_array_append_new(knobStatesJ, rowJ);
        }
        json_object_set_new(rootJ, "knobStates", knobStatesJ);

        // Save finalNotes
        json_t* finalNotesJ = json_array();
        for (int i = 0; i < 16; i++) {
            json_array_append_new(finalNotesJ, json_real(finalNotes[i]));
        }
        json_object_set_new(rootJ, "finalNotes", finalNotesJ);

        // Save buttonStates
        json_t* buttonStatesJ = json_array();
        for (int i = 0; i < 16; i++) {
            json_t* rowJ = json_array();
            for (int z = 0; z < 4; z++) {
                json_array_append_new(rowJ, json_boolean(buttonStates[i][z]));
            }
            json_array_append_new(buttonStatesJ, rowJ);
        }
        json_object_set_new(rootJ, "buttonStates", buttonStatesJ);

        // Save sequenceRunning
        json_object_set_new(rootJ, "sequenceRunning", json_boolean(sequenceRunning));

        // Save quantize
        json_object_set_new(rootJ, "quantize", json_boolean(quantize));

        // Save gate/trigger toggle setting
        json_object_set_new(rootJ, "gateTriggerEnabled", json_boolean(gateTriggerEnabled));

        // Save S/H setting
        json_object_set_new(rootJ, "isSampled", json_boolean(isSampled));

        // Save copy buffer
        json_object_set_new(rootJ, "copyBufferFilled", json_boolean(copyBufferFilled));

        // Store stage positions (xStage, yStage, zStage)
        json_object_set_new(rootJ, "xStage", json_integer(xStage));
        json_object_set_new(rootJ, "yStage", json_integer(yStage));
        json_object_set_new(rootJ, "zStage", json_integer(zStage));

        json_t* copyKnobJ = json_array();
        for (int i = 0; i < 16; i++) {
            json_array_append_new(copyKnobJ, json_real(copiedKnobStates[i]));
        }
        json_object_set_new(rootJ, "copiedKnobStates", copyKnobJ);
        
        json_t* copyButtonJ = json_array();
        for (int i = 0; i < 16; i++) {
            json_array_append_new(copyButtonJ, json_boolean(copiedButtonStates[i]));
        }
        json_object_set_new(rootJ, "copiedButtonStates", copyButtonJ);

        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {

        json_t *knobMinJ = json_object_get(rootJ, "knobMin");
        if (knobMinJ) {
            knobMin = json_real_value(knobMinJ);
        }
    
        json_t *knobRangeJ = json_object_get(rootJ, "knobRange");
        if (knobRangeJ) {
            knobRange = json_real_value(knobRangeJ);
        }

        // Load knobStates
        json_t* knobStatesJ = json_object_get(rootJ, "knobStates");
        if (knobStatesJ) {
            for (int i = 0; i < 16; i++) {
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

        // Load finalNotes
        json_t* finalNotesJ = json_object_get(rootJ, "finalNotes");
        if (finalNotesJ) {
            for (int i = 0; i < 16; i++) {
                json_t* valueJ = json_array_get(finalNotesJ, i);
                if (valueJ) {
                    finalNotes[i] = json_real_value(valueJ);
                }
            }
        }

        // Load buttonStates
        json_t* buttonStatesJ = json_object_get(rootJ, "buttonStates");
        if (buttonStatesJ) {
            for (int i = 0; i < 16; i++) {
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

        // Load sequenceRunning
        json_t* sequenceRunningJ = json_object_get(rootJ, "sequenceRunning");
        if (sequenceRunningJ) {
            sequenceRunning = json_boolean_value(sequenceRunningJ);
        }

        // Load quantize
        json_t* quantizeJ = json_object_get(rootJ, "quantize");
        if (quantizeJ) {
            quantize = json_boolean_value(quantizeJ);
        }

        //Load gate/trigger toggle setting
        json_t* gateTriggerJ = json_object_get(rootJ, "gateTriggerEnabled");
        if (gateTriggerJ) {
            gateTriggerEnabled = json_boolean_value(gateTriggerJ);
        }

        // Load copy buffer bool
        json_t* copyBufferFilledJ = json_object_get(rootJ, "copyBufferFilled");
        if (copyBufferFilledJ) {
            copyBufferFilled = json_boolean_value(copyBufferFilledJ);
        }

        // Load S/H Setting
        json_t* isSampledJ = json_object_get(rootJ, "isSampled");
        if (isSampledJ) {
            isSampled = json_boolean_value(isSampledJ);
        }

        // Load stage positions (xStage, yStage, zStage)
        json_t *xStageJ = json_object_get(rootJ, "xStage");
        if (xStageJ) {
            xStage = json_integer_value(xStageJ);
        }
    
        json_t *yStageJ = json_object_get(rootJ, "yStage");
        if (yStageJ) {
            yStage = json_integer_value(yStageJ);
        }
    
        json_t *zStageJ = json_object_get(rootJ, "zStage");
        if (zStageJ) {
            zStage = json_integer_value(zStageJ);
        }
 
        json_t* copyKnobJ = json_object_get(rootJ, "copiedKnobStates");
        if (json_is_array(copyKnobJ)) {
            for (int i = 0; i < 16; i++) {
                json_t* valJ = json_array_get(copyKnobJ, i);
                if (json_is_real(valJ)) {
                    copiedKnobStates[i] = json_real_value(valJ);
                }
            }
        }
        
        json_t* copyButtonJ = json_object_get(rootJ, "copiedButtonStates");
        if (json_is_array(copyButtonJ)) {
            for (int i = 0; i < 16; i++) {
                json_t* valJ = json_array_get(copyButtonJ, i);
                if (json_is_boolean(valJ)) {
                    copiedButtonStates[i] = json_boolean_value(valJ);
                }
            }
        }
      
        displayUpdate = true;
    }

    Cartesia() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        // Configure buttons
        configParam(SLICE1BUTTON_PARAM, 0.f, 1.f, 0.f, "Slice 1");
        configParam(SLICE2BUTTON_PARAM, 0.f, 1.f, 0.f, "Slice 2");
        configParam(SLICE3BUTTON_PARAM, 0.f, 1.f, 0.f, "Slice 3");
        configParam(SLICE4BUTTON_PARAM, 0.f, 1.f, 0.f, "Slice 4");

        // Configure knobs (4x4 grid)
        for (int x = 0; x < 4; x++) {
            for (int y = 0; y < 4; y++) {
                int i = x * 4 + y;  // Calculate the index in a row-major order
                configParam(KNOB00_PARAM + i, 0.f, 1.f, 0.5f, "Knob " + std::to_string(x) + "," + std::to_string(y));
                configParam(KNOB00_BUTTON + i, 0.f, 1.f, 0.5f, "Gate " + std::to_string(x) + "," + std::to_string(y));
            }
        }

        // Configure other controls
        configParam(XFWDBUTTON_PARAM, 0.f, 1.f, 0.f, "X Fwd");
        configParam(YFWDBUTTON_PARAM, 0.f, 1.f, 0.f, "Y Fwd");
        configParam(ZFWDBUTTON_PARAM, 0.f, 1.f, 0.f, "Z Fwd");
        configParam(SCANFWDBUTTON_PARAM, 0.f, 1.f, 0.f, "Scan Layer Fwd");
        configParam(QUANTIZEBUTTON_PARAM, 0.f, 1.f, 0.f, "Quantize Knobs");
        configParam(ONBUTTON_PARAM, 0.f, 1.f, 0.f, "ON/OFF");
        configParam(RESETBUTTON_PARAM, 0.f, 1.f, 0.f, "Reset");
        configParam(RANDOMBUTTON_PARAM, 0.f, 1.f, 0.f, "Random");

        // Configure range parameters
        configParam(MIN_PARAM, -10.f, 10.f, 0.f, "Knob Min");
        configParam(RANGE_PARAM, 1.f, 20.f, 5.f, "Knob Range (V)");
        configParam(MINATT_PARAM, -1.f, 1.f, 0.f, "Knob Min Attenuverter");
        configParam(RANGEATT_PARAM, -1.f, 1.f, 0.f, "Range Attenuverter");
        configParam(POLYKNOB_PARAM, 1.f, 4.f, 1.f, "Z Poly")->snapEnabled=true;

        // Configure inputs
        configInput(XCV_INPUT, "X Axis CV");
        configInput(XREV_INPUT, "X Reverse Trigger");
        configInput(XFWD_INPUT, "X Forward Trigger");
        
        configInput(YCV_INPUT, "Y Axis CV");
        configInput(YREV_INPUT, "Y Reverse Trigger");
        configInput(YFWD_INPUT, "Y Forward Trigger");
        
        configInput(ZCV_INPUT, "Z Axis CV");
        configInput(ZREV_INPUT, "Z Reverse Trigger");
        configInput(ZFWD_INPUT, "Z Forward Trigger");
        
        configInput(MINCV_INPUT, "Minimum Step CV");
        configInput(RANGECV_INPUT, "Step Range CV");
        configInput(SCANCV_INPUT, "Scan Position CV");
        configInput(SCANREV_INPUT, "Scan Reverse Trigger");
        configInput(SCANFWD_INPUT, "Scan Forward Trigger");
        
        configInput(ONOFF_INPUT, "ON/OFF Trigger");
        configInput(RESET_INPUT, "Reset Trigger");
        configInput(RANDOM_INPUT, "Randomize Trigger");
        configInput(OFFSET_INPUT, "Offset CV");
        
        // Configure outputs
        configOutput(RESET_OUTPUT, "Reset Trigger Out");
        configOutput(TRIGGER_OUTPUT, "Step Trigger Out");
        configOutput(GATEOUT_OUTPUT, "Step Gate Out");
        configOutput(INVGATEOUT_OUTPUT, "Inverted Step Gate Out");
        configOutput(OUTPUT_OUTPUT, "Main Sequencer Output");

    }

    void onRandomize(const RandomizeEvent& e) override {
        // Randomize only the parameter knobs (KNOB00_PARAM to KNOB33_PARAM)
        for (int i = 0; i < 16; i++) {
            params[KNOB00_PARAM + i].setValue(random::uniform()); // Random value between 0.0 and 1.0
        }

        // Randomize buttonStates only for the current zStage
        for (int i = 0; i < 16; i++) {
            buttonStates[i][zStage] = (random::uniform() < 0.5f); // 50% chance to be true or false
        }
    }

    void onReset() override {
        // Reset knobStates to all zeros
        for (int x = 0; x < 16; x++) {
            for (int z = 0; z < 4; z++) {
                knobStates[x][z] = 0.0f;
            }
        }

        // Reset buttonStates to all true
        for (int x = 0; x < 16; x++) {
            for (int z = 0; z < 4; z++) {
                buttonStates[x][z] = true;
            }
        }

        // Reset finalNotes to all zeros
        for (int i = 0; i < 16; i++) {
            finalNotes[i] = 0.0f;
        }

        // Reset other variables
        sequenceRunning = true;
        quantize = true;
        polyLevels = 1;
        knobMin = 0.f;
        knobRange = 5.f;

        // Reset parameter knobs to 0
        for (int i = 0; i < 16; i++) {
            params[KNOB00_PARAM + i].setValue(0.0f);
        }
    }

    void process(const ProcessArgs& args) override {

        if (inputs[ONOFF_INPUT].isConnected()) {
            if (onTrigger.process(inputs[ONOFF_INPUT].getVoltage())) {
                sequenceRunning = !sequenceRunning;
            }
        }
        if (onButtonTrigger.process(params[ONBUTTON_PARAM].getValue())) {
            sequenceRunning = !sequenceRunning;
        }

        knobMin = params[MIN_PARAM].getValue();
        if(inputs[MINCV_INPUT].isConnected()){
           knobMin += params[MINATT_PARAM].getValue()*inputs[MINCV_INPUT].getVoltage();
        }
        knobMin = clamp(knobMin, -10.f, 10.f);

        knobRange = params[RANGE_PARAM].getValue();
        if(inputs[RANGECV_INPUT].isConnected()){
           knobRange += params[RANGEATT_PARAM].getValue()*inputs[RANGECV_INPUT].getVoltage();
        }
        knobRange = fmin(knobRange, 10.f-knobMin);
        knobRange = clamp(knobRange, 1.f, 20.f);

        if (initializing){
            //Update Knob Ranges
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 4; y++) {
                    int i = y * 4 + x;
                    paramQuantities[KNOB00_PARAM + i]->displayOffset = knobMin;
                    paramQuantities[KNOB00_PARAM + i]->displayMultiplier = knobRange;
                }
            }
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 4; y++) {
                    int i = y * 4 + x;  // Row-major index calculation
                    paramQuantities[KNOB00_PARAM + i]->setValue( knobStates[i][zStage] );  // Recall stored knob value    

                    // Apply clamping to avoid out-of-bounds values
                    finalNotes[i] = clamp(params[KNOB00_PARAM + i].getValue() * knobRange + knobMin, -10.f, 10.f);
                }
            }            
            initializing = false;
        }

        if (quantizeTrigger.process(params[QUANTIZEBUTTON_PARAM].getValue())) {
            quantize = !quantize;
        }

        bool scanConnected = false;
        // Process XY raster scan movement based on scan CV input (0-10V, 16 stages)
        if (inputs[SCANCV_INPUT].isConnected() && sequenceRunning) {
            scanConnected = true;
            float scanVoltage = inputs[SCANCV_INPUT].getVoltage();
            int scanStage = clamp(static_cast<int>(floor(scanVoltage / (10.0f / 16.0f))), 0, 15);

            xStage = scanStage % 4;  // X cycles from 0-3 within each Y group
            yStage = scanStage / 4;  // Y changes every 4 X cycles

        } else {
            if (scanButtonTrigger.process(params[SCANFWDBUTTON_PARAM].getValue())) {
                xStage++;
                if (xStage > 3) { xStage = 0; yStage++; }
            }
            if (scanFwdTrigger.process(inputs[SCANFWD_INPUT].getVoltage()) && sequenceRunning) {
                xStage++;
                if (xStage > 3) { xStage = 0; yStage++; }
            }
            if (scanRevTrigger.process(inputs[SCANREV_INPUT].getVoltage()) && sequenceRunning) {
                xStage--;
                if (xStage < 0) { xStage = 3; yStage--; }
            }
        }

        // Process X-axis movement
        if (inputs[XCV_INPUT].isConnected() && sequenceRunning && !scanConnected) {
            float xVoltage = inputs[XCV_INPUT].getVoltage();
            xStage = clamp(static_cast<int>(floor(xVoltage / 2.5f)), 0, 3);
        } else {
            if (xButtonTrigger.process(params[XFWDBUTTON_PARAM].getValue())) { xStage++; }
            if (sequenceRunning){
                if (xRevTrigger.process(inputs[XREV_INPUT].getVoltage())) { xStage--; }
                if (xFwdTrigger.process(inputs[XFWD_INPUT].getVoltage())) { xStage++; }
            }
        }

        // Process Y-axis movement
        if (inputs[YCV_INPUT].isConnected() && sequenceRunning && !scanConnected) {
            float yVoltage = inputs[YCV_INPUT].getVoltage();
            yStage = clamp(static_cast<int>(floor(yVoltage / 2.5f)), 0, 3);
        } else {
            if (yButtonTrigger.process(params[YFWDBUTTON_PARAM].getValue())) { yStage++; }
            if (sequenceRunning){
                if (yRevTrigger.process(inputs[YREV_INPUT].getVoltage())) { yStage--; }
                if (yFwdTrigger.process(inputs[YFWD_INPUT].getVoltage())) { yStage++; }
            }
        }

        // Process Z-axis movement
        if (inputs[ZCV_INPUT].isConnected() && sequenceRunning && !scanConnected) {
            float zVoltage = inputs[ZCV_INPUT].getVoltage();
            zStage = clamp(static_cast<int>(floor(zVoltage / 2.5f)), 0, 3);
        } else {
            if (zButtonTrigger.process(params[ZFWDBUTTON_PARAM].getValue())) { zStage++; }
            if (sequenceRunning){
                if (zRevTrigger.process(inputs[ZREV_INPUT].getVoltage())) { zStage--; }
                if (zFwdTrigger.process(inputs[ZFWD_INPUT].getVoltage())) { zStage++; }
            }
        }

        // Process RANDOM inputs
        if (inputs[RANDOM_INPUT].isConnected() && sequenceRunning) {
            if (randomTrigger.process(inputs[RANDOM_INPUT].getVoltage())) {
                xStage = random::u32() % 4;  // Random value between 0-3
                yStage = random::u32() % 4;
                zStage = random::u32() % 4;
            }
        }
        if (randomButtonTrigger.process(params[RANDOMBUTTON_PARAM].getValue())) {
            xStage = random::u32() % 4;
            yStage = random::u32() % 4;
            zStage = random::u32() % 4;
        }

        // Process SLICE Change
        for (int i=0; i<4; i++){
            if (sliceTrigger[i].process(params[SLICE1BUTTON_PARAM + i].getValue())) {
                zStage = i;
            }
        }

        double deltaTime = args.sampleTime;

        // Process RESET inputs
        if (inputs[RESET_INPUT].isConnected()) {
            if (resetTrigger.process(inputs[RESET_INPUT].getVoltage()) && sequenceRunning) {
                xStage = 0;
                yStage = 0;
                zStage = 0;
                resetPulse.trigger(0.001f);
            }            
        }
        
        if (resetButtonTrigger.process(params[RESETBUTTON_PARAM].getValue())) {
                xStage = 0;
                yStage = 0;
                zStage = 0;
                resetPulse.trigger(0.001f);                
        }

        // Wrap stage values to stay within 0 - 3 (ensuring no unintended division)
        xStage = (xStage + 4) % 4;
        yStage = (yStage + 4) % 4;
        zStage = (zStage + 4) % 4;

        // Detect Z-layer (slice) change and flag display update
        if (zStage != previousZStage) {
            triggerPulse.trigger(0.001f);
            displayUpdate = true;
            previousZStage = zStage;
        }

        if ( (xStage != previousXStage) || (yStage != previousYStage) || (zStage != previousZStage) ){
            triggerPulse.trigger(0.001f);
            previousXStage = xStage;
            previousYStage = yStage;
            previousZStage = zStage;
        }

        // Deal with knob parameter saving and recall
        for (int x = 0; x < 4; x++) {
            for (int y = 0; y < 4; y++) {
                int i = y * 4 + x;  // Row-major index calculation

                if (displayUpdate) {
                    paramQuantities[KNOB00_PARAM + i]->setValue( knobStates[i][zStage] );  // Recall stored knob value
                } else {
                    knobStates[i][zStage] = params[KNOB00_PARAM + i].getValue();  // Save current knob state
                }

                if (stageTrigger[i].process(params[KNOB00_BUTTON + i].getValue())){
                    buttonStates[i][zStage] = !buttonStates[i][zStage];
                }

                // Apply clamping to avoid out-of-bounds values
                finalNotes[i] = clamp(params[KNOB00_PARAM + i].getValue() * knobRange + knobMin, -10.f, 10.f);
            }
        }

        // Reset display update flag after processing
        if (displayUpdate) {
            displayUpdate = false;
        }

        // Process Polyphonic Output Handling
        polyLevels = params[POLYKNOB_PARAM].getValue();
        outputs[OUTPUT_OUTPUT].setChannels(polyLevels);
        outputs[GATEOUT_OUTPUT].setChannels(polyLevels);
        outputs[INVGATEOUT_OUTPUT].setChannels(polyLevels);
        for (int i = 0; i < polyLevels; i++) {
            int wrappedZ = (zStage + i) % 4;  // Ensuring valid Z index
            float layerNote = clamp(knobStates[yStage * 4 + xStage][wrappedZ] * knobRange + knobMin, -10.f, 10.f);

            if (inputs[OFFSET_INPUT].isConnected()){ //Add Offset Input, if connected
                layerNote = clamp(layerNote + inputs[OFFSET_INPUT].getVoltage(), -10.f, 10.f);
            }

            if (quantize) {
                float quantizedNote = std::roundf(layerNote * 12.f) / 12.f;  // Quantize to nearest semitone
                if (isSampled){
                    if (buttonStates[xStage+4*yStage][wrappedZ]){
                        outputs[OUTPUT_OUTPUT].setVoltage(quantizedNote, i);                  
                    }
                } else {
                    outputs[OUTPUT_OUTPUT].setVoltage(quantizedNote, i);
                }
            } else {
                if (isSampled){
                    if (buttonStates[xStage+4*yStage][wrappedZ]){
                        outputs[OUTPUT_OUTPUT].setVoltage(layerNote, i);                  
                    }
                } else {
                    outputs[OUTPUT_OUTPUT].setVoltage(layerNote, i);
                }
            }

            // Gate and Inverted Gate Logic
            if (buttonStates[xStage + 4 * yStage][wrappedZ]) {
                if (gateTriggerEnabled) {
                    // If gateTriggerEnabled, output a trigger instead of a gate
                    outputs[GATEOUT_OUTPUT].setVoltage(triggerPulse.process(deltaTime) ? 10.f : 0.f, i);
                } else {
                    // Normal gate output
                    outputs[GATEOUT_OUTPUT].setVoltage(10.f, i);
                }
                outputs[INVGATEOUT_OUTPUT].setVoltage(0.f, i);
            } else {
                outputs[GATEOUT_OUTPUT].setVoltage(0.f, i);
                outputs[INVGATEOUT_OUTPUT].setVoltage(10.f, i);
            }
        }
        
        //Trigger Outputs
        outputs[RESET_OUTPUT].setVoltage(resetPulse.process(deltaTime) ? 10.f : 0.f);
        outputs[TRIGGER_OUTPUT].setVoltage(triggerPulse.process(deltaTime) ? 10.f : 0.f);
        
    }
};

struct CartesiaWidget : ModuleWidget {
    CartesiaWidget(Cartesia* module) {
        setModule(module);

        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Cartesia.svg"),
            asset::plugin(pluginInstance, "res/Cartesia-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        float scale = 1.04f;  //Compensate for the error in importing svg through helper.py

        //Slice Buttons
        addParam(createParamCentered<LEDButton>(Vec(scale*47.28,   scale*40.08), module, Cartesia::SLICE1BUTTON_PARAM));
        addParam(createParamCentered<LEDButton>(Vec(scale*94.292,  scale*40.08), module, Cartesia::SLICE2BUTTON_PARAM));
        addParam(createParamCentered<LEDButton>(Vec(scale*141.304, scale*40.08), module, Cartesia::SLICE3BUTTON_PARAM));
        addParam(createParamCentered<LEDButton>(Vec(scale*188.316, scale*40.08), module, Cartesia::SLICE4BUTTON_PARAM));

        // Slice Lights
        addChild(createLightCentered<MediumLight<RedLight>>(Vec(scale*47.28,   scale*40.08), module, Cartesia::SLICE1BUTTON_LIGHT));
        addChild(createLightCentered<MediumLight<GreenLight>>(Vec(scale*94.292,  scale*40.08), module, Cartesia::SLICE2BUTTON_LIGHT));
        addChild(createLightCentered<MediumLight<BlueLight>>(Vec(scale*141.304, scale*40.08), module, Cartesia::SLICE3BUTTON_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(scale*188.316, scale*40.08), module, Cartesia::SLICE4BUTTON_LIGHT));

        //Main Knobs
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(scale*249.463, scale*67.39  ), module, Cartesia::KNOB00_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(scale*307.762, scale*67.39  ), module, Cartesia::KNOB10_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(scale*366.062, scale*67.39  ), module, Cartesia::KNOB20_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(scale*424.361, scale*67.39  ), module, Cartesia::KNOB30_PARAM));

        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(scale*249.463, scale*130.414), module, Cartesia::KNOB01_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(scale*307.762, scale*130.414), module, Cartesia::KNOB11_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(scale*366.062, scale*130.414), module, Cartesia::KNOB21_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(scale*424.361, scale*130.414), module, Cartesia::KNOB31_PARAM));

        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(scale*249.463, scale*192.335), module, Cartesia::KNOB02_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(scale*307.762, scale*192.335), module, Cartesia::KNOB12_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(scale*366.062, scale*192.335), module, Cartesia::KNOB22_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(scale*424.361, scale*192.335), module, Cartesia::KNOB32_PARAM));

        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(scale*249.463, scale*256.447), module, Cartesia::KNOB03_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(scale*307.762, scale*256.447), module, Cartesia::KNOB13_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(scale*366.062, scale*256.447), module, Cartesia::KNOB23_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(scale*424.361, scale*256.447), module, Cartesia::KNOB33_PARAM));

        //Stage Buttons
        addParam(createParamCentered<TL1105>(Vec(scale*249.463, scale*67.39  ), module, Cartesia::KNOB00_BUTTON));
        addParam(createParamCentered<TL1105>(Vec(scale*307.762, scale*67.39  ), module, Cartesia::KNOB10_BUTTON));
        addParam(createParamCentered<TL1105>(Vec(scale*366.062, scale*67.39  ), module, Cartesia::KNOB20_BUTTON));
        addParam(createParamCentered<TL1105>(Vec(scale*424.361, scale*67.39  ), module, Cartesia::KNOB30_BUTTON));

        addParam(createParamCentered<TL1105>(Vec(scale*249.463, scale*130.414), module, Cartesia::KNOB01_BUTTON));
        addParam(createParamCentered<TL1105>(Vec(scale*307.762, scale*130.414), module, Cartesia::KNOB11_BUTTON));
        addParam(createParamCentered<TL1105>(Vec(scale*366.062, scale*130.414), module, Cartesia::KNOB21_BUTTON));
        addParam(createParamCentered<TL1105>(Vec(scale*424.361, scale*130.414), module, Cartesia::KNOB31_BUTTON));

        addParam(createParamCentered<TL1105>(Vec(scale*249.463, scale*192.335), module, Cartesia::KNOB02_BUTTON));
        addParam(createParamCentered<TL1105>(Vec(scale*307.762, scale*192.335), module, Cartesia::KNOB12_BUTTON));
        addParam(createParamCentered<TL1105>(Vec(scale*366.062, scale*192.335), module, Cartesia::KNOB22_BUTTON));
        addParam(createParamCentered<TL1105>(Vec(scale*424.361, scale*192.335), module, Cartesia::KNOB32_BUTTON));

        addParam(createParamCentered<TL1105>(Vec(scale*249.463, scale*256.447), module, Cartesia::KNOB03_BUTTON));
        addParam(createParamCentered<TL1105>(Vec(scale*307.762, scale*256.447), module, Cartesia::KNOB13_BUTTON));
        addParam(createParamCentered<TL1105>(Vec(scale*366.062, scale*256.447), module, Cartesia::KNOB23_BUTTON));
        addParam(createParamCentered<TL1105>(Vec(scale*424.361, scale*256.447), module, Cartesia::KNOB33_BUTTON));

        //Main Control Grid
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(scale*38.88, scale*175.2  ), module, Cartesia::XCV_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(scale*38.88, scale*206.202), module, Cartesia::YCV_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(scale*38.88, scale*237.771), module, Cartesia::ZCV_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(scale*38.88, scale*270.228), module, Cartesia::SCANCV_INPUT));

        addInput(createInputCentered<ThemedPJ301MPort>(Vec(scale*72.928, scale*175.2  ), module, Cartesia::XREV_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(scale*72.928, scale*206.202), module, Cartesia::YREV_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(scale*72.928, scale*237.771), module, Cartesia::ZREV_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(scale*72.928, scale*270.228), module, Cartesia::SCANREV_INPUT));

        addInput(createInputCentered<ThemedPJ301MPort>(Vec(scale*106.976, scale*175.2  ), module, Cartesia::XFWD_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(scale*106.976, scale*206.202), module, Cartesia::YFWD_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(scale*106.976, scale*237.771), module, Cartesia::ZFWD_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(scale*106.976, scale*270.228), module, Cartesia::SCANFWD_INPUT));

        addParam(createParamCentered<TL1105>(Vec(scale*132.022, scale*175.2  ), module, Cartesia::XFWDBUTTON_PARAM));
        addParam(createParamCentered<TL1105>(Vec(scale*132.022, scale*206.202), module, Cartesia::YFWDBUTTON_PARAM));
        addParam(createParamCentered<TL1105>(Vec(scale*132.022, scale*237.772), module, Cartesia::ZFWDBUTTON_PARAM));
        addParam(createParamCentered<TL1105>(Vec(scale*132.022, scale*270.229), module, Cartesia::SCANFWDBUTTON_PARAM));

        //Range Controls
        addParam(createParamCentered<RoundBlackKnob>(Vec(scale*162.823, scale*192.91), module, Cartesia::MIN_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(Vec(scale*195.182, scale*192.91), module, Cartesia::RANGE_PARAM));
        addParam(createParamCentered<Trimpot>(Vec(scale*163.463, scale*222.273), module, Cartesia::MINATT_PARAM));
        addParam(createParamCentered<Trimpot>(Vec(scale*195.501, scale*222.273), module, Cartesia::RANGEATT_PARAM));

        addInput(createInputCentered<ThemedPJ301MPort>(Vec(scale*162.823, scale*248.042), module, Cartesia::MINCV_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(scale*195.182, scale*248.042), module, Cartesia::RANGECV_INPUT));

        //On Off Reset
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(scale*38.88,   scale*325.181), module, Cartesia::ONOFF_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(scale*85.404,  scale*325.181), module, Cartesia::RESET_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(scale*132.072, scale*325.181), module, Cartesia::RANDOM_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(scale*179.475, scale*325.181), module, Cartesia::OFFSET_INPUT));

        addParam(createParamCentered<TL1105>(Vec(scale*85.404,  scale*302.332), module, Cartesia::RESETBUTTON_PARAM));
        addParam(createParamCentered<TL1105>(Vec(scale*132.072, scale*302.332), module, Cartesia::RANDOMBUTTON_PARAM));

        addParam(createParamCentered<LEDButton>(Vec(scale*38.88,   scale*302.332), module, Cartesia::ONBUTTON_PARAM));
        addParam(createParamCentered<LEDButton>(Vec(scale*179.475, scale*285.052), module, Cartesia::QUANTIZEBUTTON_PARAM));

        //Button Lights
        addChild(createLightCentered<MediumLight<RedLight>>(Vec(scale*38.88,   scale*302.332), module, Cartesia::ONBUTTON_LIGHT));
        addChild(createLightCentered<MediumLight<RedLight>>(Vec(scale*179.475, scale*285.052), module, Cartesia::QUANTIZEBUTTON_LIGHT));

        //Outputs
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(scale*245.698, scale*319.61 ), module, Cartesia::RESET_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(scale*282.418, scale*319.61 ), module, Cartesia::TRIGGER_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(scale*355.856, scale*319.61 ), module, Cartesia::GATEOUT_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(scale*392.575, scale*319.61 ), module, Cartesia::INVGATEOUT_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(scale*429.294, scale*319.61 ), module, Cartesia::OUTPUT_OUTPUT));
        addParam(createParamCentered<Trimpot>(Vec(scale*319.137, scale*322.855), module, Cartesia::POLYKNOB_PARAM));

        //LED Map grid
        addChild(createLightCentered<MediumLight<RedLight   >>(Vec(scale*29.04,   scale*53.52 ), module, Cartesia::LED000_LIGHT));
        addChild(createLightCentered<MediumLight<GreenLight >>(Vec(scale*74.587,  scale*53.52 ), module, Cartesia::LED001_LIGHT));
        addChild(createLightCentered<MediumLight<BlueLight  >>(Vec(scale*121.53,  scale*53.52 ), module, Cartesia::LED002_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(scale*167.971, scale*53.52 ), module, Cartesia::LED003_LIGHT));

        addChild(createLightCentered<MediumLight<RedLight   >>(Vec(scale*39.9,   scale*60.0   ), module, Cartesia::LED100_LIGHT));
        addChild(createLightCentered<MediumLight<GreenLight >>(Vec(scale*85.447, scale*60.0   ), module, Cartesia::LED101_LIGHT));
        addChild(createLightCentered<MediumLight<BlueLight  >>(Vec(scale*132.39, scale*60.0   ), module, Cartesia::LED102_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(scale*178.83, scale*60.72  ), module, Cartesia::LED103_LIGHT));

        addChild(createLightCentered<MediumLight<RedLight   >>(Vec(scale*50.55,  scale*66.48  ), module, Cartesia::LED200_LIGHT));
        addChild(createLightCentered<MediumLight<GreenLight >>(Vec(scale*96.097, scale*66.48  ), module, Cartesia::LED201_LIGHT));
        addChild(createLightCentered<MediumLight<BlueLight  >>(Vec(scale*143.04, scale*66.48  ), module, Cartesia::LED202_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(scale*189.48, scale*66.48  ), module, Cartesia::LED203_LIGHT));

        addChild(createLightCentered<MediumLight<RedLight   >>(Vec(scale*61.166,  scale*72.96 ), module, Cartesia::LED300_LIGHT));
        addChild(createLightCentered<MediumLight<GreenLight >>(Vec(scale*106.713, scale*72.96 ), module, Cartesia::LED301_LIGHT));
        addChild(createLightCentered<MediumLight<BlueLight  >>(Vec(scale*153.657, scale*72.96 ), module, Cartesia::LED302_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(scale*200.097, scale*72.96 ), module, Cartesia::LED303_LIGHT));

        addChild(createLightCentered<MediumLight<RedLight   >>(Vec(scale*29.04,   scale*70.912), module, Cartesia::LED010_LIGHT));
        addChild(createLightCentered<MediumLight<GreenLight >>(Vec(scale*74.587,  scale*70.912), module, Cartesia::LED011_LIGHT));
        addChild(createLightCentered<MediumLight<BlueLight  >>(Vec(scale*121.53,  scale*70.912), module, Cartesia::LED012_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(scale*167.971, scale*70.912), module, Cartesia::LED013_LIGHT));

        addChild(createLightCentered<MediumLight<RedLight   >>(Vec(scale*39.9,   scale*77.392 ), module, Cartesia::LED110_LIGHT));
        addChild(createLightCentered<MediumLight<GreenLight >>(Vec(scale*85.447, scale*77.392 ), module, Cartesia::LED111_LIGHT));
        addChild(createLightCentered<MediumLight<BlueLight  >>(Vec(scale*132.39, scale*77.392 ), module, Cartesia::LED112_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(scale*178.83, scale*78.112 ), module, Cartesia::LED113_LIGHT));

        addChild(createLightCentered<MediumLight<RedLight   >>(Vec(scale*50.55,  scale*83.872  ), module, Cartesia::LED210_LIGHT));
        addChild(createLightCentered<MediumLight<GreenLight >>(Vec(scale*96.097, scale*83.872  ), module, Cartesia::LED211_LIGHT));
        addChild(createLightCentered<MediumLight<BlueLight  >>(Vec(scale*143.04, scale*83.872  ), module, Cartesia::LED212_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(scale*189.48, scale*83.872  ), module, Cartesia::LED213_LIGHT));

        addChild(createLightCentered<MediumLight<RedLight   >>(Vec(scale*61.166,  scale*90.352 ), module, Cartesia::LED310_LIGHT));
        addChild(createLightCentered<MediumLight<GreenLight >>(Vec(scale*106.713, scale*90.352 ), module, Cartesia::LED311_LIGHT));
        addChild(createLightCentered<MediumLight<BlueLight  >>(Vec(scale*153.657, scale*90.352 ), module, Cartesia::LED312_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(scale*200.097, scale*90.352 ), module, Cartesia::LED313_LIGHT));

        addChild(createLightCentered<MediumLight<RedLight   >>(Vec(scale*29.04,   scale*88.303 ), module, Cartesia::LED020_LIGHT));
        addChild(createLightCentered<MediumLight<GreenLight >>(Vec(scale*74.587,  scale*88.303 ), module, Cartesia::LED021_LIGHT));
        addChild(createLightCentered<MediumLight<BlueLight  >>(Vec(scale*121.53,  scale*88.303 ), module, Cartesia::LED022_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(scale*167.971, scale*88.303 ), module, Cartesia::LED023_LIGHT));

        addChild(createLightCentered<MediumLight<RedLight   >>(Vec(scale*39.9,   scale*94.784  ), module, Cartesia::LED120_LIGHT));
        addChild(createLightCentered<MediumLight<GreenLight >>(Vec(scale*85.447, scale*94.784  ), module, Cartesia::LED121_LIGHT));
        addChild(createLightCentered<MediumLight<BlueLight  >>(Vec(scale*132.39, scale*94.784  ), module, Cartesia::LED122_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(scale*178.83, scale*95.504  ), module, Cartesia::LED123_LIGHT));

        addChild(createLightCentered<MediumLight<RedLight   >>(Vec(scale*50.55,  scale*101.263 ), module, Cartesia::LED220_LIGHT));
        addChild(createLightCentered<MediumLight<GreenLight >>(Vec(scale*96.097, scale*101.263 ), module, Cartesia::LED221_LIGHT));
        addChild(createLightCentered<MediumLight<BlueLight  >>(Vec(scale*143.04, scale*101.263 ), module, Cartesia::LED222_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(scale*189.48, scale*101.263 ), module, Cartesia::LED223_LIGHT));

        addChild(createLightCentered<MediumLight<RedLight   >>(Vec(scale*61.166,  scale*107.743), module, Cartesia::LED320_LIGHT));
        addChild(createLightCentered<MediumLight<GreenLight >>(Vec(scale*106.713, scale*107.743), module, Cartesia::LED321_LIGHT));
        addChild(createLightCentered<MediumLight<BlueLight  >>(Vec(scale*153.657, scale*107.743), module, Cartesia::LED322_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(scale*200.097, scale*107.743), module, Cartesia::LED323_LIGHT));

        addChild(createLightCentered<MediumLight<RedLight   >>(Vec(scale*29.04,   scale*105.695), module, Cartesia::LED030_LIGHT));
        addChild(createLightCentered<MediumLight<GreenLight >>(Vec(scale*74.587,  scale*105.695), module, Cartesia::LED031_LIGHT));
        addChild(createLightCentered<MediumLight<BlueLight  >>(Vec(scale*121.53,  scale*105.695), module, Cartesia::LED032_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(scale*167.971, scale*105.695), module, Cartesia::LED033_LIGHT));

        addChild(createLightCentered<MediumLight<RedLight   >>(Vec(scale*39.9,   scale*112.175 ), module, Cartesia::LED130_LIGHT));
        addChild(createLightCentered<MediumLight<GreenLight >>(Vec(scale*85.447, scale*112.175 ), module, Cartesia::LED131_LIGHT));
        addChild(createLightCentered<MediumLight<BlueLight  >>(Vec(scale*132.39, scale*112.175 ), module, Cartesia::LED132_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(scale*178.83, scale*112.895 ), module, Cartesia::LED133_LIGHT));

        addChild(createLightCentered<MediumLight<RedLight   >>(Vec(scale*50.55,  scale*118.655  ), module, Cartesia::LED230_LIGHT));
        addChild(createLightCentered<MediumLight<GreenLight >>(Vec(scale*96.097, scale*118.655 ), module, Cartesia::LED231_LIGHT));
        addChild(createLightCentered<MediumLight<BlueLight  >>(Vec(scale*143.04, scale*118.655 ), module, Cartesia::LED232_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(scale*189.48, scale*118.655 ), module, Cartesia::LED233_LIGHT));

        addChild(createLightCentered<MediumLight<RedLight   >>(Vec(scale*61.166,  scale*125.135 ), module, Cartesia::LED330_LIGHT));
        addChild(createLightCentered<MediumLight<GreenLight >>(Vec(scale*106.713, scale*125.135), module, Cartesia::LED331_LIGHT));
        addChild(createLightCentered<MediumLight<BlueLight  >>(Vec(scale*153.657, scale*125.135), module, Cartesia::LED332_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(scale*200.097, scale*125.135), module, Cartesia::LED333_LIGHT));

        //STAGE Lights
        addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(scale*266.304, scale*82.444 ), module, Cartesia::STAGE00_LIGHT));
        addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(scale*324.603, scale*82.444 ), module, Cartesia::STAGE10_LIGHT));
        addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(scale*382.902, scale*82.444 ), module, Cartesia::STAGE20_LIGHT));
        addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(scale*441.202, scale*82.444 ), module, Cartesia::STAGE30_LIGHT));
        addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(scale*266.304, scale*145.468), module, Cartesia::STAGE01_LIGHT));
        addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(scale*324.603, scale*145.468), module, Cartesia::STAGE11_LIGHT));
        addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(scale*382.902, scale*145.468), module, Cartesia::STAGE21_LIGHT));
        addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(scale*441.202, scale*145.468), module, Cartesia::STAGE31_LIGHT));
        addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(scale*266.304, scale*207.389), module, Cartesia::STAGE02_LIGHT));
        addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(scale*324.603, scale*207.389), module, Cartesia::STAGE12_LIGHT));
        addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(scale*382.902, scale*207.389), module, Cartesia::STAGE22_LIGHT));
        addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(scale*441.202, scale*207.389), module, Cartesia::STAGE32_LIGHT));
        addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(scale*266.304, scale*271.501), module, Cartesia::STAGE03_LIGHT));
        addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(scale*324.603, scale*271.501), module, Cartesia::STAGE13_LIGHT));
        addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(scale*382.902, scale*271.501), module, Cartesia::STAGE23_LIGHT));
        addChild(createLightCentered<MediumLight<WhiteLight>>(Vec(scale*441.202, scale*271.501), module, Cartesia::STAGE33_LIGHT));

        //RGBY Main Knob Lights
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(scale*249.463, scale*67.39  ), module, Cartesia::KNOB00_LIGHT_R));
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(scale*307.762, scale*67.39  ), module, Cartesia::KNOB10_LIGHT_R));
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(scale*366.062, scale*67.39  ), module, Cartesia::KNOB20_LIGHT_R));
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(scale*424.361, scale*67.39  ), module, Cartesia::KNOB30_LIGHT_R));
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(scale*249.463, scale*130.414), module, Cartesia::KNOB01_LIGHT_R));
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(scale*307.762, scale*130.414), module, Cartesia::KNOB11_LIGHT_R));
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(scale*366.062, scale*130.414), module, Cartesia::KNOB21_LIGHT_R));
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(scale*424.361, scale*130.414), module, Cartesia::KNOB31_LIGHT_R));
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(scale*249.463, scale*192.335), module, Cartesia::KNOB02_LIGHT_R));
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(scale*307.762, scale*192.335), module, Cartesia::KNOB12_LIGHT_R));
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(scale*366.062, scale*192.335), module, Cartesia::KNOB22_LIGHT_R));
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(scale*424.361, scale*192.335), module, Cartesia::KNOB32_LIGHT_R));
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(scale*249.463, scale*256.447), module, Cartesia::KNOB03_LIGHT_R));
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(scale*307.762, scale*256.447), module, Cartesia::KNOB13_LIGHT_R));
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(scale*366.062, scale*256.447), module, Cartesia::KNOB23_LIGHT_R));
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(scale*424.361, scale*256.447), module, Cartesia::KNOB33_LIGHT_R));

        addChild(createLightCentered<LargeLight<GreenLight>>(Vec(scale*249.463, scale*67.39  ), module, Cartesia::KNOB00_LIGHT_G));
        addChild(createLightCentered<LargeLight<GreenLight>>(Vec(scale*307.762, scale*67.39  ), module, Cartesia::KNOB10_LIGHT_G));
        addChild(createLightCentered<LargeLight<GreenLight>>(Vec(scale*366.062, scale*67.39  ), module, Cartesia::KNOB20_LIGHT_G));
        addChild(createLightCentered<LargeLight<GreenLight>>(Vec(scale*424.361, scale*67.39  ), module, Cartesia::KNOB30_LIGHT_G));
        addChild(createLightCentered<LargeLight<GreenLight>>(Vec(scale*249.463, scale*130.414), module, Cartesia::KNOB01_LIGHT_G));
        addChild(createLightCentered<LargeLight<GreenLight>>(Vec(scale*307.762, scale*130.414), module, Cartesia::KNOB11_LIGHT_G));
        addChild(createLightCentered<LargeLight<GreenLight>>(Vec(scale*366.062, scale*130.414), module, Cartesia::KNOB21_LIGHT_G));
        addChild(createLightCentered<LargeLight<GreenLight>>(Vec(scale*424.361, scale*130.414), module, Cartesia::KNOB31_LIGHT_G));
        addChild(createLightCentered<LargeLight<GreenLight>>(Vec(scale*249.463, scale*192.335), module, Cartesia::KNOB02_LIGHT_G));
        addChild(createLightCentered<LargeLight<GreenLight>>(Vec(scale*307.762, scale*192.335), module, Cartesia::KNOB12_LIGHT_G));
        addChild(createLightCentered<LargeLight<GreenLight>>(Vec(scale*366.062, scale*192.335), module, Cartesia::KNOB22_LIGHT_G));
        addChild(createLightCentered<LargeLight<GreenLight>>(Vec(scale*424.361, scale*192.335), module, Cartesia::KNOB32_LIGHT_G));
        addChild(createLightCentered<LargeLight<GreenLight>>(Vec(scale*249.463, scale*256.447), module, Cartesia::KNOB03_LIGHT_G));
        addChild(createLightCentered<LargeLight<GreenLight>>(Vec(scale*307.762, scale*256.447), module, Cartesia::KNOB13_LIGHT_G));
        addChild(createLightCentered<LargeLight<GreenLight>>(Vec(scale*366.062, scale*256.447), module, Cartesia::KNOB23_LIGHT_G));
        addChild(createLightCentered<LargeLight<GreenLight>>(Vec(scale*424.361, scale*256.447), module, Cartesia::KNOB33_LIGHT_G));

        addChild(createLightCentered<LargeLight<BlueLight>>(Vec(scale*249.463, scale*67.39  ), module, Cartesia::KNOB00_LIGHT_B));
        addChild(createLightCentered<LargeLight<BlueLight>>(Vec(scale*307.762, scale*67.39  ), module, Cartesia::KNOB10_LIGHT_B));
        addChild(createLightCentered<LargeLight<BlueLight>>(Vec(scale*366.062, scale*67.39  ), module, Cartesia::KNOB20_LIGHT_B));
        addChild(createLightCentered<LargeLight<BlueLight>>(Vec(scale*424.361, scale*67.39  ), module, Cartesia::KNOB30_LIGHT_B));
        addChild(createLightCentered<LargeLight<BlueLight>>(Vec(scale*249.463, scale*130.414), module, Cartesia::KNOB01_LIGHT_B));
        addChild(createLightCentered<LargeLight<BlueLight>>(Vec(scale*307.762, scale*130.414), module, Cartesia::KNOB11_LIGHT_B));
        addChild(createLightCentered<LargeLight<BlueLight>>(Vec(scale*366.062, scale*130.414), module, Cartesia::KNOB21_LIGHT_B));
        addChild(createLightCentered<LargeLight<BlueLight>>(Vec(scale*424.361, scale*130.414), module, Cartesia::KNOB31_LIGHT_B));
        addChild(createLightCentered<LargeLight<BlueLight>>(Vec(scale*249.463, scale*192.335), module, Cartesia::KNOB02_LIGHT_B));
        addChild(createLightCentered<LargeLight<BlueLight>>(Vec(scale*307.762, scale*192.335), module, Cartesia::KNOB12_LIGHT_B));
        addChild(createLightCentered<LargeLight<BlueLight>>(Vec(scale*366.062, scale*192.335), module, Cartesia::KNOB22_LIGHT_B));
        addChild(createLightCentered<LargeLight<BlueLight>>(Vec(scale*424.361, scale*192.335), module, Cartesia::KNOB32_LIGHT_B));
        addChild(createLightCentered<LargeLight<BlueLight>>(Vec(scale*249.463, scale*256.447), module, Cartesia::KNOB03_LIGHT_B));
        addChild(createLightCentered<LargeLight<BlueLight>>(Vec(scale*307.762, scale*256.447), module, Cartesia::KNOB13_LIGHT_B));
        addChild(createLightCentered<LargeLight<BlueLight>>(Vec(scale*366.062, scale*256.447), module, Cartesia::KNOB23_LIGHT_B));
        addChild(createLightCentered<LargeLight<BlueLight>>(Vec(scale*424.361, scale*256.447), module, Cartesia::KNOB33_LIGHT_B));

        addChild(createLightCentered<LargeLight<YellowLight>>(Vec(scale*249.463, scale*67.39  ), module, Cartesia::KNOB00_LIGHT_Y));
        addChild(createLightCentered<LargeLight<YellowLight>>(Vec(scale*307.762, scale*67.39  ), module, Cartesia::KNOB10_LIGHT_Y));
        addChild(createLightCentered<LargeLight<YellowLight>>(Vec(scale*366.062, scale*67.39  ), module, Cartesia::KNOB20_LIGHT_Y));
        addChild(createLightCentered<LargeLight<YellowLight>>(Vec(scale*424.361, scale*67.39  ), module, Cartesia::KNOB30_LIGHT_Y));
        addChild(createLightCentered<LargeLight<YellowLight>>(Vec(scale*249.463, scale*130.414), module, Cartesia::KNOB01_LIGHT_Y));
        addChild(createLightCentered<LargeLight<YellowLight>>(Vec(scale*307.762, scale*130.414), module, Cartesia::KNOB11_LIGHT_Y));
        addChild(createLightCentered<LargeLight<YellowLight>>(Vec(scale*366.062, scale*130.414), module, Cartesia::KNOB21_LIGHT_Y));
        addChild(createLightCentered<LargeLight<YellowLight>>(Vec(scale*424.361, scale*130.414), module, Cartesia::KNOB31_LIGHT_Y));
        addChild(createLightCentered<LargeLight<YellowLight>>(Vec(scale*249.463, scale*192.335), module, Cartesia::KNOB02_LIGHT_Y));
        addChild(createLightCentered<LargeLight<YellowLight>>(Vec(scale*307.762, scale*192.335), module, Cartesia::KNOB12_LIGHT_Y));
        addChild(createLightCentered<LargeLight<YellowLight>>(Vec(scale*366.062, scale*192.335), module, Cartesia::KNOB22_LIGHT_Y));
        addChild(createLightCentered<LargeLight<YellowLight>>(Vec(scale*424.361, scale*192.335), module, Cartesia::KNOB32_LIGHT_Y));
        addChild(createLightCentered<LargeLight<YellowLight>>(Vec(scale*249.463, scale*256.447), module, Cartesia::KNOB03_LIGHT_Y));
        addChild(createLightCentered<LargeLight<YellowLight>>(Vec(scale*307.762, scale*256.447), module, Cartesia::KNOB13_LIGHT_Y));
        addChild(createLightCentered<LargeLight<YellowLight>>(Vec(scale*366.062, scale*256.447), module, Cartesia::KNOB23_LIGHT_Y));
        addChild(createLightCentered<LargeLight<YellowLight>>(Vec(scale*424.361, scale*256.447), module, Cartesia::KNOB33_LIGHT_Y));

        //Poly Lights
        addChild(createLightCentered<SmallLight<RedLight>>(Vec(scale*308.688, scale*308.325), module, Cartesia::POLY1_LIGHT));
        addChild(createLightCentered<SmallLight<RedLight>>(Vec(scale*316.138, scale*308.325), module, Cartesia::POLY2_LIGHT));
        addChild(createLightCentered<SmallLight<RedLight>>(Vec(scale*323.588, scale*308.325), module, Cartesia::POLY3_LIGHT));
        addChild(createLightCentered<SmallLight<RedLight>>(Vec(scale*331.039, scale*308.325), module, Cartesia::POLY4_LIGHT));

        //Text Displays
        if (module) {
            // (Vec(28.32, 17.76))  display size
            module->minDisplay = createDigitalDisplay((Vec(scale*149.520, scale*154.62)), "Min");
            addChild(module->minDisplay);

            module->maxDisplay = createDigitalDisplay((Vec(scale*180.310, scale*154.62)), "Max");
            addChild(module->maxDisplay);

            for (int i=0; i<4; i++){
                for (int j=0; j<4; j++){
                    module->noteDisplays[i+j*4] = createDigitalDisplay((Vec(scale*235.512 + scale*58.06*i, scale*30.475 + scale*63.0*j)), "C4");
                    addChild(module->noteDisplays[i+j*4]);
                }
            }
        }

    }

    void draw(const DrawArgs& args) override {
        ModuleWidget::draw(args);
        Cartesia* module = dynamic_cast<Cartesia*>(this->module);
        if (!module) return;

        // Update note displays
        for (int i = 0; i < 16; i++) {
            if (module->noteDisplays[i]) {
                if (module->quantize){
                    // Compute the note name of the note and the octave
                    float pitchVoltage = module->finalNotes[i];
                    int octave = static_cast<int>(pitchVoltage + 4);  // The integer part represents the octave

                    // Calculate the semitone
                    double fractionalPart = fmod(pitchVoltage, 1.0);
                    int semitone = std::roundf(fractionalPart * 12);
                    semitone = (semitone % 12 + 12) % 12;  // Ensure it's a valid semitone (0 to 11)

                    // Define the note names
                    const char* noteNames[12] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
                    const char* noteName = noteNames[semitone];

                    // Create a buffer to hold the full note name
                    char fullNote[7];  // Enough space for note name + octave + null terminator
                    snprintf(fullNote, sizeof(fullNote), "%s%d", noteName, octave);  // Combine note and octave

                    module->noteDisplays[i]->text = fullNote;
                } else {
                    char fullNote[7];
                    snprintf(fullNote, sizeof(fullNote), "%.2f", module->finalNotes[i]);
                    module->noteDisplays[i]->text = fullNote;
                }
            }
        }

        // Update Min Max display
        if (module->minDisplay){
            if (module->quantize){
                // Compute the note name of the note and the octave
                float pitchVoltage = module->knobMin;
                int octave = static_cast<int>(pitchVoltage + 4);  // The integer part represents the octave

                // Calculate the semitone
                double fractionalPart = fmod(pitchVoltage, 1.0);
                int semitone = std::roundf(fractionalPart * 12);
                semitone = (semitone % 12 + 12) % 12;  // Ensure it's a valid semitone (0 to 11)

                // Define the note names
                const char* noteNames[12] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
                const char* noteName = noteNames[semitone];

                // Create a buffer to hold the full note name
                char fullNote[7];  // Enough space for note name + octave + null terminator
                snprintf(fullNote, sizeof(fullNote), "%s%d", noteName, octave);  // Combine note and octave

                module->minDisplay->text = fullNote;
            } else {
                char rangeDisp[7];
                snprintf(rangeDisp, sizeof(rangeDisp), "%.1f", module->knobMin);
                module->minDisplay->text = rangeDisp;
            }
        }

        if (module->maxDisplay){
            if (module->quantize){
                // Compute the note name of the note and the octave
                float pitchVoltage = fmin(module->knobMin + module->knobRange, 10.f);
                int octave = static_cast<int>(pitchVoltage + 4);  // The integer part represents the octave

                // Calculate the semitone
                double fractionalPart = fmod(pitchVoltage, 1.0);
                int semitone = std::roundf(fractionalPart * 12);
                semitone = (semitone % 12 + 12) % 12;  // Ensure it's a valid semitone (0 to 11)

                // Define the note names
                const char* noteNames[12] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
                const char* noteName = noteNames[semitone];

                // Create a buffer to hold the full note name
                char fullNote[7];  // Enough space for note name + octave + null terminator
                snprintf(fullNote, sizeof(fullNote), "%s%d", noteName, octave);  // Combine note and octave

                module->maxDisplay->text = fullNote;
            } else {
                char rangeDisp[7];
                float knobMax = fmin(module->knobMin + module->knobRange, 10.f);
                snprintf(rangeDisp, sizeof(rangeDisp), "%.1f", knobMax);
                module->maxDisplay->text = rangeDisp;
            }
        }

        //Update Quantize Light
        if(module->quantize){
            module->lights[Cartesia::QUANTIZEBUTTON_LIGHT].setBrightness(1.0f);
        } else {
            module->lights[Cartesia::QUANTIZEBUTTON_LIGHT].setBrightness(0.0f);
        }

        //Update Stage Light and Map Lights
        for (int x = 0; x < 4; x++) {
            for (int y = 0; y < 4; y++) {
                for (int z = 0; z < 4; z++) {
                    int i = (x * 16) + (y * 4) + z;
        
                    module->lights[Cartesia::LED000_LIGHT + i].setBrightness(0.0f);
        
                    // If this is the active Z stage
                    if (z == module->zStage) {
                        if (x == module->xStage && y == module->yStage) {
                            module->lights[Cartesia::LED000_LIGHT + i].setBrightness(1.0f);
                            module->lights[Cartesia::STAGE00_LIGHT + 4 * y + x].setBrightness(0.5f);
                            module->lights[Cartesia::SLICE1BUTTON_LIGHT + z].setBrightness(1.0f);
                        } else {
                            module->lights[Cartesia::LED000_LIGHT + i].setBrightness(0.12f);
                            module->lights[Cartesia::STAGE00_LIGHT + 4 * y + x].setBrightness(0.0f);
                        }
                    } else {
                        module->lights[Cartesia::SLICE1BUTTON_LIGHT + z].setBrightness(0.0f);
                    }

                    // Apply Z-stack polyphony, making sure to wrap around at 4
                    for (int p = 1; p < module->polyLevels; p++) {  // Start at 1 to avoid double-lighting zStage itself
                        int zWrapped = (module->zStage + p) % 4;
                        if (z == zWrapped && x == module->xStage && y == module->yStage) {
                            module->lights[Cartesia::LED000_LIGHT + i].setBrightness(0.25f);
                        }
                    }
                }
            }
        }


        //Update Knob Ranges
        for (int x = 0; x < 4; x++) {
            for (int y = 0; y < 4; y++) {
                int i = y * 4 + x;
                module->paramQuantities[Cartesia::KNOB00_PARAM + i]->displayOffset = module->knobMin;
                module->paramQuantities[Cartesia::KNOB00_PARAM + i]->displayMultiplier = module->knobRange;
            }
        }

        // Update Knob Lights
        for (int x = 0; x < 4; x++) {
            for (int y = 0; y < 4; y++) {
                int i = y * 4 + x;

                // Calculate base index for lights
                int baseIndex = Cartesia::KNOB00_LIGHT_R + i;

                // Loop through all 4 zLayers and set brightness
                for (int z = 0; z < 4; z++) {
                    int lightIndex = baseIndex + (z * 16);

                    if (z == module->zStage) {
                        // If this is the active zStage, set brightness based on button state
                        module->lights[lightIndex].setBrightness(module->buttonStates[i][module->zStage] ? 1.0f : 0.00f);
                    } else {
                        // Turn off all other zStage lights
                        module->lights[lightIndex].setBrightness(0.0f);
                    }
                }
            }
        }

        //Update Poly Lights
        for (int i=0; i<4; i++){
            if (i < module->polyLevels){
                module->lights[Cartesia::POLY1_LIGHT + i].setBrightness(1.0f);
            } else {
                module->lights[Cartesia::POLY1_LIGHT + i].setBrightness(0.0f);
            }
         }

         //Update ON light
         if (module->sequenceRunning){
             module->lights[Cartesia::ONBUTTON_LIGHT].setBrightness(1.0f);
         } else {
             module->lights[Cartesia::ONBUTTON_LIGHT].setBrightness(0.0f);
         }
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
    
        Cartesia* cartesiaModule = dynamic_cast<Cartesia*>(module);
        assert(cartesiaModule); // Ensure the cast succeeds
    
        // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());
     
        // Copy Layer menu item
        struct CopyLayerMenuItem : MenuItem {
            Cartesia* cartesiaModule;
            void onAction(const event::Action& e) override {
                for (int i = 0; i < 16; i++) {
                    cartesiaModule->copiedKnobStates[i] = cartesiaModule->knobStates[i][cartesiaModule->zStage];
                    cartesiaModule->copiedButtonStates[i] = cartesiaModule->buttonStates[i][cartesiaModule->zStage];
                }
                cartesiaModule->copyBufferFilled = true;
            }
            void step() override {
                rightText = cartesiaModule->copyBufferFilled ? "✔" : "";
                MenuItem::step();
            }
        };
        
        CopyLayerMenuItem* copyLayerItem = new CopyLayerMenuItem();
        copyLayerItem->text = "Copy Layer";
        copyLayerItem->cartesiaModule = cartesiaModule;
        menu->addChild(copyLayerItem);
        
        // Paste Layer menu item
        struct PasteLayerMenuItem : MenuItem {
            Cartesia* cartesiaModule;
            void onAction(const event::Action& e) override {
                if (!cartesiaModule->copyBufferFilled) return;
                for (int i = 0; i < 16; i++) {
                    cartesiaModule->knobStates[i][cartesiaModule->zStage] = cartesiaModule->copiedKnobStates[i];
                    cartesiaModule->buttonStates[i][cartesiaModule->zStage] = cartesiaModule->copiedButtonStates[i];
                }
                cartesiaModule->displayUpdate = true;
            }
            void step() override {
                rightText = cartesiaModule->copyBufferFilled ? "Ready" : "Empty";
                MenuItem::step();
            }
        };
        
        PasteLayerMenuItem* pasteLayerItem = new PasteLayerMenuItem();
        pasteLayerItem->text = "Paste Layer";
        pasteLayerItem->cartesiaModule = cartesiaModule;
        menu->addChild(pasteLayerItem);

        // Paste to All Layers menu item
        struct PasteAllLayersMenuItem : MenuItem {
            Cartesia* cartesiaModule;
            void onAction(const event::Action& e) override {
                if (!cartesiaModule->copyBufferFilled) return;
                for (int z = 0; z < 4; z++) {
                    for (int i = 0; i < 16; i++) {
                        cartesiaModule->knobStates[i][z] = cartesiaModule->copiedKnobStates[i];
                        cartesiaModule->buttonStates[i][z] = cartesiaModule->copiedButtonStates[i];
                    }
                }
                cartesiaModule->displayUpdate = true;
            }
            void step() override {
                rightText = cartesiaModule->copyBufferFilled ? "Ready" : "Empty";
                MenuItem::step();
            }
        };
        
        PasteAllLayersMenuItem* pasteAllLayersItem = new PasteAllLayersMenuItem();
        pasteAllLayersItem->text = "Paste to All Layers";
        pasteAllLayersItem->cartesiaModule = cartesiaModule;
        menu->addChild(pasteAllLayersItem);

        // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());
     
        // Sample and Hold CV controls Active Step menu item
        struct SampleAndHoldMenuItem : MenuItem {
            Cartesia* cartesiaModule;
            void onAction(const event::Action& e) override {
                // Trigger the Sample and Hold functionality based on the active step
                cartesiaModule->isSampled = !cartesiaModule->isSampled;
            }
            void step() override {
                // You could also add a visual indicator here if you need to, like a checkmark.
                rightText = cartesiaModule->isSampled ? "✔" : "";
                MenuItem::step();
            }
        };
    
        SampleAndHoldMenuItem* sampleAndHoldItem = new SampleAndHoldMenuItem();
        sampleAndHoldItem->text = "Sample and Hold Active Step";
        sampleAndHoldItem->cartesiaModule = cartesiaModule;
        menu->addChild(sampleAndHoldItem);

        // Toggle Gate Trigger Output
        struct GateTriggerMenuItem : MenuItem {
            Cartesia* cartesiaModule;
            void onAction(const event::Action& e) override {
                cartesiaModule->gateTriggerEnabled = !cartesiaModule->gateTriggerEnabled;
            }
            void step() override {
                rightText = cartesiaModule->gateTriggerEnabled ? "✔" : "";
                MenuItem::step();
            }
        };
        
        GateTriggerMenuItem* gateTriggerItem = new GateTriggerMenuItem();
        gateTriggerItem->text = "Enable Triggers from Gate Outputs";
        gateTriggerItem->cartesiaModule = cartesiaModule;
        menu->addChild(gateTriggerItem);
        
    }    
};
Model* modelCartesia = createModel<Cartesia, CartesiaWidget>("Cartesia");