////////////////////////////////////////////////////////////
//
//   Hex Mod
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   Six phase related LFOs
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"

using namespace rack;

float linearInterpolate(float a, float b, float fraction) {
    return a + fraction * (b - a);
}

struct HexMod : Module {
    enum ParamIds {
        RATE_KNOB,
        NODE_KNOB,
        RATE_ATT_KNOB,
        NODE_ATT_KNOB,
        NUM_PARAMS
    };
    enum InputIds {
        ENV_INPUT_1,
        ENV_INPUT_2,
        ENV_INPUT_3,
        ENV_INPUT_4,
        ENV_INPUT_5,
        ENV_INPUT_6,
        RATE_INPUT,
        NODE_INPUT,
        SYNC_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        LFO_OUTPUT_1,
        LFO_OUTPUT_2,
        LFO_OUTPUT_3,
        LFO_OUTPUT_4,
        LFO_OUTPUT_5,
        LFO_OUTPUT_6,
        NUM_OUTPUTS
    };
    enum LightIds {
        // Positive (Red) LEDs for each output
        LFO_POS_LED_1, LFO_POS_LED_2, LFO_POS_LED_3, LFO_POS_LED_4, LFO_POS_LED_5,
        LFO_POS_LED_6, LFO_POS_LED_7, LFO_POS_LED_8, LFO_POS_LED_9, LFO_POS_LED_10,
        LFO_POS_LED_11, LFO_POS_LED_12, LFO_POS_LED_13, LFO_POS_LED_14, LFO_POS_LED_15,
        LFO_POS_LED_16, LFO_POS_LED_17, LFO_POS_LED_18, LFO_POS_LED_19, LFO_POS_LED_20,
        LFO_POS_LED_21, LFO_POS_LED_22, LFO_POS_LED_23, LFO_POS_LED_24, LFO_POS_LED_25,
        LFO_POS_LED_26, LFO_POS_LED_27, LFO_POS_LED_28, LFO_POS_LED_29, LFO_POS_LED_30,

        // Negative (Blue) LEDs for each output
        LFO_NEG_LED_1, LFO_NEG_LED_2, LFO_NEG_LED_3, LFO_NEG_LED_4, LFO_NEG_LED_5,
        LFO_NEG_LED_6, LFO_NEG_LED_7, LFO_NEG_LED_8, LFO_NEG_LED_9, LFO_NEG_LED_10,
        LFO_NEG_LED_11, LFO_NEG_LED_12, LFO_NEG_LED_13, LFO_NEG_LED_14, LFO_NEG_LED_15,
        LFO_NEG_LED_16, LFO_NEG_LED_17, LFO_NEG_LED_18, LFO_NEG_LED_19, LFO_NEG_LED_20,
        LFO_NEG_LED_21, LFO_NEG_LED_22, LFO_NEG_LED_23, LFO_NEG_LED_24, LFO_NEG_LED_25,
        LFO_NEG_LED_26, LFO_NEG_LED_27, LFO_NEG_LED_28, LFO_NEG_LED_29, LFO_NEG_LED_30,
        
        //INPUT-OUTPUT LIGHTS
        IN_LED_1, IN_LED_2, IN_LED_3, IN_LED_4, IN_LED_5, IN_LED_6,
        OUT_LED_1a, OUT_LED_2a, OUT_LED_3a, OUT_LED_4a, OUT_LED_5a, OUT_LED_6a,
        OUT_LED_1b, OUT_LED_2b, OUT_LED_3b, OUT_LED_4b, OUT_LED_5b, OUT_LED_6b,
        OUT_LED_1c, OUT_LED_2c, OUT_LED_3c, OUT_LED_4c, OUT_LED_5c, OUT_LED_6c,
        OUT_LED_1d, OUT_LED_2d, OUT_LED_3d, OUT_LED_4d, OUT_LED_5d, OUT_LED_6d,
        
        NUM_LIGHTS
    };

    // Initialize timer dsps
    dsp::Timer SyncTimer;

    // Initialize variables for trigger detection
    dsp::SchmittTrigger SyncTrigger;

    bool lightsEnabled = true;
    bool syncEnabled = false;
    bool synclinkEnabled = true;
    bool voctEnabled = false;

    float lfoPhase[6] = {0.0f}; // Current LFO phase for each channel
    float prevPhaseResetInput[6] = {}; // Previous envelope input, for peak detection

    // Function declarations
    void process(const ProcessArgs& args) override;
    float calculateTargetPhase(int channel, float NodePosition, float deltaTime, float place);
    void adjustLFOPhase(int channel, float targetPhase, float envInfluence, float deltaTime);
    void updateLEDs(int channel, float voltage);
    
    float lastTargetVoltages[5] = {0.f, 0.f, 0.f, 0.f, 0.f}; // Initialize with default voltages, assuming start at 0V
  
    float place[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    float happy_place[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

    bool firstClockPulse = true;
    bool clockSyncPulse = false;
    bool risingState[6] = {}; // Initialize all channels as falling initially
    bool latch[6] = {}; // Initialize all latches

    float lfoOutput[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    float nextChunk[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}; //measure next voltage step to subdivide
    
    int LEDprocessCounter = 0; // Counter to track process cycles
    int SINprocessCounter = 0; // Counter to track process cycles
    int SkipProcesses = 4; //Number of process cycles to skip for the big calculation

    float lastConnectedInputVoltage = 0.0f;
    float SyncInterval = 2; //default to 2hz

    // Serialization method to save module state
    json_t* dataToJson() override {
        json_t* rootJ = json_object();

        json_object_set_new(rootJ, "lightsEnabled", json_boolean(lightsEnabled));
        json_object_set_new(rootJ, "syncEnabled", json_boolean(syncEnabled));
        json_object_set_new(rootJ, "synclinkEnabled", json_boolean(synclinkEnabled));
        json_object_set_new(rootJ, "SyncInterval", json_real(SyncInterval));
        json_object_set_new(rootJ, "voctEnabled", json_boolean(voctEnabled));

        // Serialize lfoOutput array
        json_t* lfoOutputJ = json_array();
        for (int i = 0; i < 6; i++) {
            json_array_append_new(lfoOutputJ, json_real(lfoOutput[i]));
        }
        json_object_set_new(rootJ, "lfoOutput", lfoOutputJ);

        // Serialize place array
        json_t* placeJ = json_array();
        for (int i = 0; i < 6; i++) {
            json_array_append_new(placeJ, json_real(place[i]));
        }
        json_object_set_new(rootJ, "place", placeJ);


        return rootJ;
    }

    // Deserialization method to load module state
    void dataFromJson(json_t* rootJ) override {
        // Load the state of lightsEnabled
        json_t* lightsEnabledJ = json_object_get(rootJ, "lightsEnabled");
        if (lightsEnabledJ) {
            lightsEnabled = json_is_true(lightsEnabledJ);
        }

        // Load the state of syncEnabled
        json_t* syncEnabledJ = json_object_get(rootJ, "syncEnabled");
        if (syncEnabledJ) {
            syncEnabled = json_is_true(syncEnabledJ);
        }
 
        // Load the state of voctEnabled
        json_t* voctEnabledJ = json_object_get(rootJ, "voctEnabled");
        if (voctEnabledJ) {
            voctEnabled = json_is_true(voctEnabledJ);
        }

        // Load the state of syncEnabled
        json_t* synclinkEnabledJ = json_object_get(rootJ, "synclinkEnabled");
        if (synclinkEnabledJ) {
            synclinkEnabled = json_is_true(synclinkEnabledJ);
        }
        
        // Load the state of SyncInterval
        json_t* SyncIntervalJ = json_object_get(rootJ, "SyncInterval");
        if (SyncIntervalJ) {
            SyncInterval = json_number_value(SyncIntervalJ);
        }   
        
        // Deserialize lfoOutput array
        json_t* lfoOutputJ = json_object_get(rootJ, "lfoOutput");
        if (lfoOutputJ) {
            for (int i = 0; i < 6; i++) {
                json_t* valueJ = json_array_get(lfoOutputJ, i);
                if (valueJ) {
                    lfoOutput[i] = json_number_value(valueJ);
                }
            }
        }                        
        // Deserialize place array
        json_t* placeJ = json_object_get(rootJ, "place");
        if (placeJ) {
            for (int i = 0; i < 6; i++) {
                json_t* valueJ = json_array_get(placeJ, i);
                if (valueJ) {
                    place[i] = json_number_value(valueJ);
                }
            }
        }                        
    }

     HexMod() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        // Initialize knob parameters with a reasonable range and default values
        configParam(RATE_KNOB, 0.0f, 10.0f, 2.0f, "Rate, Hz"); // 
        configParam(NODE_KNOB, 0.0f, 3.0f, 0.0f, "Node Distribution"); // 0: Hexagonal, 1: Unison, 2: Bimodal, 3: Trimodal

        configParam(RATE_ATT_KNOB, -1.0f, 1.0f, 0.1f, "Rate Attenuation"); // 
        configParam(NODE_ATT_KNOB, -1.0f, 1.0f, 0.1f, "Node Attenuation"); // 

        for (int i = 0; i < 6; i++) {
            configInput(ENV_INPUT_1 + i, "Trigger/Gate " + std::to_string(i + 1));
        }
        configInput(RATE_INPUT, "Rate CV");
        configInput(NODE_INPUT, "Node Distribution CV");
        configInput(SYNC_INPUT, "Sync");
        
        lightsEnabled = true; // Default to true
        synclinkEnabled = true;

        for (int i = 0; i < 6; i++) {
            configOutput(LFO_OUTPUT_1 + i, "LFO " + std::to_string(i + 1));
        }
    }
};

void HexMod::process(const ProcessArgs& args) {

    float deltaTime = args.sampleTime; 
    LEDprocessCounter++;  
    SINprocessCounter++;  

    //PROCESS INPUTS

    // Calculate the rate from the RATE_KNOB and any RATE_INPUT CV
    float rate = params[RATE_KNOB].getValue();
    if (inputs[RATE_INPUT].isConnected()) {
        rate += inputs[RATE_INPUT].getVoltage()*params[RATE_ATT_KNOB].getValue(); // CV adds to the rate
    }    
 
	if (voctEnabled){
	    rate = clamp(rate, -10.f, 10.0f); 
	    rate = 261.625565 * pow(2.0, rate);
	} else {
	   rate = clamp(rate, 0.01f, 10.0f); 
	}

    // Calculate target phase based on Node knob
    float NodePosition = params[NODE_KNOB].getValue();
    if (inputs[NODE_INPUT].isConnected()) {
        NodePosition += inputs[NODE_INPUT].getVoltage()*params[NODE_ATT_KNOB].getValue(); // CV adds to the position
    }
    NodePosition = clamp(NodePosition, 0.0f, 3.0f); 

    // Process clock sync input
    float SyncInputVoltage;

    if (inputs[SYNC_INPUT].isConnected()) {
        // Get the voltage from the SYNC input
        SyncInputVoltage = inputs[SYNC_INPUT].getVoltage();

        // Accumulate time in the timer
        SyncTimer.process(args.sampleTime);

        // Check if the Sync Trigger condition is met
        if (SyncTrigger.process(SyncInputVoltage)) {
            if (!firstClockPulse){
                SyncInterval = SyncTimer.time; // Get the accumulated time since the last reset
                SyncTimer.reset(); // Reset the timer for the next trigger interval measurement
        
                if (synclinkEnabled) {
                    clockSyncPulse = true;
                }
            } else {
                SyncTimer.reset(); // Reset the timer for the next trigger interval measurement
                firstClockPulse = false;
            }           
        }

        if (syncEnabled) {
            rate *= 1 / SyncInterval; // Rate knob becomes a multiplier when Sync is patched
        } else {
            rate = 1 / SyncInterval; // Rate knob is deactivated when Sync is patched
        }
    }

    
    for (int i = 0; i < 6; i++) {
        // Gate/trigger to Phase Reset input
        float PhaseResetInput;
        
        // If the current input is connected, use it and update lastConnectedInputVoltage
        if (inputs[ENV_INPUT_1 + i].isConnected()) {
            PhaseResetInput = inputs[ENV_INPUT_1 + i].getVoltage();
            lastConnectedInputVoltage = PhaseResetInput;
        } else {
            // If not connected, use the last connected input's voltage
            PhaseResetInput = lastConnectedInputVoltage;
        }
        
        if (PhaseResetInput < 0.0001f){latch[i]= true; }
        PhaseResetInput = clamp(PhaseResetInput, 0.0f, 10.0f);
 
       // Check if the envelope is rising or falling with hysteresis
        if (risingState[i]) {
            // If it was rising, look for a significant drop before considering it falling
            if (PhaseResetInput < prevPhaseResetInput[i]) {
                risingState[i] = false; // Now it's falling
            }
        } else {
            // If it was falling, look for a significant rise before considering it rising
            if (PhaseResetInput > prevPhaseResetInput[i]) {
                risingState[i] = true; // Now it's rising
                lights[IN_LED_1+i].setBrightness(1.0f);        
                lights[OUT_LED_1a+i].setBrightness(1.0f);        
                lights[OUT_LED_1b+i].setBrightness(1.0f);        
                lights[OUT_LED_1c+i].setBrightness(1.0f);        
                lights[OUT_LED_1d+i].setBrightness(1.0f);        
            }
        }

        float basePhase = i / -6.0f; // Starting with hexagonal distribution    
        float targetPhase = basePhase; // Default to base phase

        /////////////////////
        // NODE positioning logic
        //
        if (NodePosition < 1.0f) {
            // Unison
            targetPhase = linearInterpolate(basePhase, 0.5f, NodePosition);
        } else if (NodePosition < 2.0f) {
            // Bimodal distribution
            float bimodalPhase = (i % 2) / 2.0f;
            float dynamicFactor = -1.0f*(NodePosition - 1.0f)*((i+1.0f)/2.0f);
            targetPhase = linearInterpolate(0.5f, bimodalPhase*dynamicFactor, NodePosition - 1.0f);
        } else {
            float bimodalPhase = (i % 2) / 2.0f;
            float trimodalPhase = (i % 3) / 3.0f;
            float blendFactor = NodePosition - 2.0f; // Gradually changes from 0 to 1 as NodePosition goes from 2.0 to 3.0
            float adjustedTrimodalPhase = trimodalPhase;
            adjustedTrimodalPhase = linearInterpolate(bimodalPhase, trimodalPhase, blendFactor*1.0f );         
            targetPhase = adjustedTrimodalPhase;
        }    
        targetPhase += place[i];
        
        while (targetPhase >= 1.0f) targetPhase -= 1.0f;
        while (targetPhase < 0.0f) targetPhase += 1.0f;

        float phaseDiff = targetPhase - lfoPhase[i];
        // Ensure phaseDiff is within the -0.5 to 0.5 range to find the shortest path
        if (phaseDiff > 0.5f) phaseDiff -= 1.0f;
        if (phaseDiff < -0.5f) phaseDiff += 1.0f;

        if (synclinkEnabled){
            if (clockSyncPulse){        
                lfoPhase[i] += phaseDiff;
            } else {
                lfoPhase[i] += phaseDiff*(0.2f*(rate/1000.f)    ) - 0.199*pow((PhaseResetInput/10.0f),0.01f)*(rate/1000.f)   ;
            }            
        }else{
            //Phase returns to the correct spot, rate determined by PhaseGate
            lfoPhase[i] += phaseDiff*(0.2f*(rate/1000.f)   ) - 0.199*pow((PhaseResetInput/10.0f),0.01f)*(rate/1000.f)    ;
        }

        // Ensure phase is within [0, 1)
        while (lfoPhase[i] >= 1.0f) lfoPhase[i] -= 1.0f;
        while (lfoPhase[i] < 0.0f) lfoPhase[i] += 1.0f;

        // Update the LFO phase based on the rate
        lfoPhase[i] += rate * deltaTime ;        
        if (lfoPhase[i] >= 1.0f) lfoPhase[i] -= 1.0f; // Wrap the phase

        place[i] += rate * deltaTime;
 
        if (place[i] >= 1.0f) place[i] -= 1.0f; // Wrap 

        // Reset LFO phase to 0 at the peak of the envelope
        if ((risingState[i] && latch[i]) || (clockSyncPulse)) {
            if(!clockSyncPulse){
                lfoPhase[i] = 0.0f;
            }
            place[i] = 0.0f;
            latch[i]= false;
        } 

        float currentOutput = outputs[LFO_OUTPUT_1 + i].getVoltage();
        if (SINprocessCounter > SkipProcesses) {
            // Generate LFO output using the sine function and the adjusted phase
            lfoOutput[i] = 5.0f * sinf(2.0f * M_PI * lfoPhase[i]);
            nextChunk[i] = lfoOutput[i]-currentOutput;
        }
    
        // Since we process 1/N samples, linearly interpolate the rest
        currentOutput += nextChunk[i] * 1/SkipProcesses;

        //Output Voltage
        outputs[LFO_OUTPUT_1 + i].setVoltage(currentOutput);
        if (lightsEnabled) {
            if (LEDprocessCounter > 1500) {
                // Update LEDs based on LFO output
                updateLEDs(i, lfoOutput[i]);

                float brightness = lights[IN_LED_1+i].getBrightness(); 
                lights[IN_LED_1+i].setBrightness(brightness*0.9f);        
                lights[OUT_LED_1a+i].setBrightness(brightness*0.9f);        
                lights[OUT_LED_1b+i].setBrightness(brightness*0.9f);        
                lights[OUT_LED_1c+i].setBrightness(brightness*0.9f);        
                lights[OUT_LED_1d+i].setBrightness(brightness*0.9f);        
            }
        } else {
            for (int i = 0; i < NUM_LIGHTS; i++) {lights[i].setBrightness(0);}
        }
              
        prevPhaseResetInput[i] = PhaseResetInput;
    }
   
    if (LEDprocessCounter > 1500) {LEDprocessCounter=0;    }
    if (SINprocessCounter > SkipProcesses) {SINprocessCounter=0;    }
    clockSyncPulse=false;
}

void HexMod::updateLEDs(int channel, float voltage) {
    // Ensure we do not exceed the array bounds
    if (channel < 0 || channel >= 6) return;

    // Calculate the starting index for red and blue LEDs of this channel
    int redStartIndex = LFO_POS_LED_1 + channel * 5;
    int blueStartIndex = LFO_NEG_LED_1 + channel * 5;

    // Update LEDs for the channel
    for (int i = 0; i < 5; i++) { // 5 LEDs for each polarity
        // Calculate the index for the current LED within the channel
        int redIndex = redStartIndex + i;
        int blueIndex = blueStartIndex + i;

        // Safety check to prevent out-of-bounds access
        if (redIndex >= NUM_LIGHTS || blueIndex >= NUM_LIGHTS) continue;

        // Update red LEDs based on positive voltage
        lights[redIndex].setBrightness(clamp(voltage - i, 0.0f, 1.0f));

        // Update blue LEDs based on negative voltage
        lights[blueIndex].setBrightness(clamp((-voltage) - i, 0.0f, 1.0f));
    }
}

struct HexModWidget : ModuleWidget {
    HexModWidget(HexMod* module) {
        setModule(module);

        setPanel(createPanel(
                asset::plugin(pluginInstance, "res/HexMod.svg"),
                asset::plugin(pluginInstance, "res/HexMod-dark.svg")
            ));

        // Add screws or additional design elements as needed
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // Envelope Inputs at the top
        const Vec inputStartPos = Vec(15+10, 30);
        const float inputSpacing = 31.0f;
            addInput(createInput<PJ301MPort>(inputStartPos.plus(Vec(inputSpacing * 0, 0)), module, HexMod::ENV_INPUT_1 + 0));
            addChild(createLight<TinyLight<YellowLight>>(inputStartPos.plus(Vec(inputSpacing * 0+11, 27)), module, HexMod::IN_LED_1 + 0));

            addInput(createInput<PJ301MPort>(inputStartPos.plus(Vec(inputSpacing * 1, 0)), module, HexMod::ENV_INPUT_1 + 1));
            addChild(createLight<TinyLight<YellowLight>>(inputStartPos.plus(Vec(inputSpacing * 1+11, 27)), module, HexMod::IN_LED_1 + 1));

            addInput(createInput<PJ301MPort>(inputStartPos.plus(Vec(inputSpacing * 2, 0)), module, HexMod::ENV_INPUT_1 + 2));
            addChild(createLight<TinyLight<YellowLight>>(inputStartPos.plus(Vec(inputSpacing * 2+11, 27)), module, HexMod::IN_LED_1 + 2));

            addInput(createInput<PJ301MPort>(inputStartPos.plus(Vec(inputSpacing * 3, 0)), module, HexMod::ENV_INPUT_1 + 3));
            addChild(createLight<TinyLight<YellowLight>>(inputStartPos.plus(Vec(inputSpacing * 3+11, 27)), module, HexMod::IN_LED_1 + 3));

            addInput(createInput<PJ301MPort>(inputStartPos.plus(Vec(inputSpacing * 4, 0)), module, HexMod::ENV_INPUT_1 + 4));
            addChild(createLight<TinyLight<YellowLight>>(inputStartPos.plus(Vec(inputSpacing * 4+11, 27)), module, HexMod::IN_LED_1 + 4));

            addInput(createInput<PJ301MPort>(inputStartPos.plus(Vec(inputSpacing * 5, 0)), module, HexMod::ENV_INPUT_1 + 5));
            addChild(createLight<TinyLight<YellowLight>>(inputStartPos.plus(Vec(inputSpacing * 5+11, 27)), module, HexMod::IN_LED_1 + 5));

        // Hexagon of Outputs and LED strips in the center
        const Vec hexCenter = Vec(mm2px(37), mm2px(55)); // Center of the hexagon
        const float hexRadius = 67.0f; // Adjusted radius for output placement
        const float jackOffset = 20.0f; // Additional radius offset for jacks
        for (int i = 0; i < 6; i++) {
            float angle = M_PI / 3.0 * (i+3);
            Vec jackPos = Vec(hexCenter.x + cos(angle) * (hexRadius + jackOffset), hexCenter.y + sin(angle) * (hexRadius + jackOffset));
            jackPos = jackPos.minus(Vec(8, 8)); // Offset the jack position 

            addOutput(createOutput<PJ301MPort>(jackPos, module, HexMod::LFO_OUTPUT_1 + i));

            Vec outputPos = Vec(hexCenter.x + cos(angle) * hexRadius, hexCenter.y + sin(angle) * hexRadius); // Original position for LED calculations

            // Calculate the direction vector from the output jack towards the hex center and its perpendicular vector for staggering
            Vec dir = hexCenter.minus(outputPos).normalize();
            Vec staggerDir = Vec(-dir.y, dir.x); // Perpendicular vector for staggering

            // Determine the start and end points for LED placement
            Vec ledStartPos = outputPos;
            Vec ledEndPos = hexCenter.minus(dir.mult(hexRadius * 0.15f)); // Bring LEDs closer to the center

            // Calculate the increment vector for LED placement
            Vec increment = ledEndPos.minus(ledStartPos).div(9); // Dividing by 9 as we have 10 LEDs in total

            // Add LED strips for each output along the spoke
            for (int j = 0; j < 10; j++) {
                Vec ledPos = ledStartPos.plus(increment.mult(j+0.5));
                Vec staggeredLedPos = ledPos.plus(staggerDir.mult((10-j+1)*0.3f * ((j % 2) * 2 - 1))); // Staggering alternate LEDs

                if (j < 5) {
                    // For the first 5 LEDs in each set, use red LEDs
                    addChild(createLight<SmallLight<RedLight>>(staggeredLedPos, module, HexMod::LFO_POS_LED_1 + i * 5 + (4-j) ));
                } else {
                    // For the next 5 LEDs in each set, use blue LEDs
                    // Since LFO_NEG_LED_1 starts immediately after the last red LED of the last channel, calculate the offset accordingly
                    addChild(createLight<TinyLight<BlueLight>>(staggeredLedPos, module, HexMod::LFO_NEG_LED_1 + i * 5 + (j - 5)));
                }                
            }
            
            //Add OUT_LEDs just to the side of the zero-point of each spoke:
            addChild(createLight<TinyLight<YellowLight>>(ledStartPos.plus(increment.mult(-2)).plus(staggerDir.mult(23)), module, HexMod::OUT_LED_1a + i));
            addChild(createLight<TinyLight<YellowLight>>(ledStartPos.plus(increment.mult(-2)).plus(staggerDir.mult(-23)), module, HexMod::OUT_LED_1b + i));
            addChild(createLight<TinyLight<YellowLight>>(ledStartPos.plus(increment.mult(-1.0)).plus(staggerDir.mult(34.5)), module, HexMod::OUT_LED_1c + i));
            addChild(createLight<TinyLight<YellowLight>>(ledStartPos.plus(increment.mult(-1.0)).plus(staggerDir.mult(-34.5)), module, HexMod::OUT_LED_1d + i));
            
        }

        // Row of knobs at the bottom, with attenuators and CV inputs
        const Vec knobStartPos = Vec(21, 268);
        const float knobSpacing = 152.0f;
        addParam(createParam<RoundBlackKnob>(knobStartPos, module, HexMod::RATE_KNOB));
        addParam(createParam<RoundBlackKnob>(knobStartPos.plus(Vec( knobSpacing, 0)), module, HexMod::NODE_KNOB));

        addParam(createParam<Trimpot>(knobStartPos.plus(Vec(0+5, 41)), module, HexMod::RATE_ATT_KNOB));
        addParam(createParam<Trimpot>(knobStartPos.plus(Vec( knobSpacing+5, 41)), module, HexMod::NODE_ATT_KNOB));

        addInput(createInput<PJ301MPort>(knobStartPos.plus(Vec(0+2, 63)), module, HexMod::RATE_INPUT));
        addInput(createInput<PJ301MPort>(knobStartPos.plus(Vec(knobSpacing+2, 63)), module, HexMod::NODE_INPUT));

        addInput(createInput<PJ301MPort>(knobStartPos.plus(Vec(0.5*knobSpacing+2, 40)), module, HexMod::SYNC_INPUT));

    }
    
    void appendContextMenu(Menu* menu) {
        ModuleWidget::appendContextMenu(menu);

        HexMod* hexMod = dynamic_cast<HexMod*>(module);
        assert(hexMod);

        // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator);

        // Lights enabled/disabled menu item
        struct LightsEnabledItem : MenuItem {
            HexMod* hexMod;
            void onAction(const event::Action& e) override {
                hexMod->lightsEnabled = !hexMod->lightsEnabled;
            }
            void step() override {
                rightText = hexMod->lightsEnabled ? "✔" : "";
                MenuItem::step();
            }
        };

        LightsEnabledItem* lightsItem = new LightsEnabledItem;
        lightsItem->text = "Enable Lights";
        lightsItem->hexMod = hexMod;
        menu->addChild(lightsItem);

        // Sync enabled/disabled menu item
        struct SyncEnabledItem : MenuItem {
            HexMod* hexMod;
            void onAction(const event::Action& e) override {
                hexMod->syncEnabled = !hexMod->syncEnabled;
            }
            void step() override {
                rightText = hexMod->syncEnabled ? "✔" : "";
                MenuItem::step();
            }
        };

        SyncEnabledItem* syncItem = new SyncEnabledItem;
        syncItem->text = "Rate multiplies the Sync Input"; 
        syncItem->hexMod = hexMod;
        menu->addChild(syncItem);

        // Sync and Phase not linked
        struct SyncLinkEnabledItem : MenuItem {
            HexMod* hexMod;
            void onAction(const event::Action& e) override {
                hexMod->synclinkEnabled = !hexMod->synclinkEnabled;
            }
            void step() override {
                rightText = hexMod->synclinkEnabled ? "✔" : "";
                MenuItem::step();
            }
        };

        SyncLinkEnabledItem* synclinkItem = new SyncLinkEnabledItem;
        synclinkItem->text = "Sync locks both Clock and Phase"; 
        synclinkItem->hexMod = hexMod;
        menu->addChild(synclinkItem);

        // Sync and Phase not linked
        struct VOctEnabledItem : MenuItem {
            HexMod* hexMod;
            void onAction(const event::Action& e) override {
                hexMod->voctEnabled = !hexMod->voctEnabled;
            }
            void step() override {
                rightText = hexMod->voctEnabled ? "✔" : "";
                MenuItem::step();
            }
        };

        VOctEnabledItem* voctItem = new VOctEnabledItem;
        voctItem->text = "Rate input take v/oct (for audio rate)"; 
        voctItem->hexMod = hexMod;
        menu->addChild(voctItem);

    }
    
};

Model* modelHexMod = createModel<HexMod, HexModWidget>("HexMod");