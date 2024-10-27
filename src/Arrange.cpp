////////////////////////////////////////////////////////////
//
//   Arrange
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   Seven channel sequencer
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
#include "digital_display.hpp"

using namespace rack;

struct DiscreteRoundBlackKnob : RoundBlackKnob { 
    void onDragEnd(const DragEndEvent& e) override {
        ParamQuantity* paramQuantity = getParamQuantity();
        
        if (paramQuantity) {
            // Get the raw value from the knob
            float rawValue = paramQuantity->getValue();
            
            // Round the value to the nearest integer
            float discreteValue = round(rawValue);
            
            // Set the snapped value
            paramQuantity->setValue(discreteValue);
        }
        
        // Call the base class implementation to ensure proper behavior
        RoundBlackKnob::onDragEnd(e);
    }
};

// Define our Module derived from Rack's Module class
struct Arrange : Module {
    enum ParamIds {
        STAGE_SELECT,
        MAX_STAGES,
        FORWARD_BUTTON,
        BACKWARDS_BUTTON,
        RESET_BUTTON,
        CHAN_1_BUTTON, CHAN_2_BUTTON, CHAN_3_BUTTON, CHAN_4_BUTTON, CHAN_5_BUTTON, CHAN_6_BUTTON, CHAN_7_BUTTON, 
        CHAN_1_KNOB, CHAN_2_KNOB, CHAN_3_KNOB, CHAN_4_KNOB, CHAN_5_KNOB, CHAN_6_KNOB, CHAN_7_KNOB,
        REC_BUTTON,        
        NUM_PARAMS
    };
    enum InputIds {
        RESET_INPUT,
        FORWARD_INPUT,
        BACKWARDS_INPUT,
        REC_INPUT,
        CHAN_1_INPUT, CHAN_2_INPUT, CHAN_3_INPUT, CHAN_4_INPUT, CHAN_5_INPUT, CHAN_6_INPUT, CHAN_7_INPUT, 
        NUM_INPUTS
    };
    enum OutputIds {
        CHAN_1_OUTPUT, CHAN_2_OUTPUT, CHAN_3_OUTPUT, CHAN_4_OUTPUT, CHAN_5_OUTPUT, CHAN_6_OUTPUT, CHAN_7_OUTPUT, 
        NUM_OUTPUTS
    };
    enum LightIds {
        CHAN_1_LIGHT, CHAN_2_LIGHT, CHAN_3_LIGHT, CHAN_4_LIGHT, CHAN_5_LIGHT, CHAN_6_LIGHT, CHAN_7_LIGHT,
        CHAN_1_LIGHT_B, CHAN_2_LIGHT_B, CHAN_3_LIGHT_B, CHAN_4_LIGHT_B, CHAN_5_LIGHT_B, CHAN_6_LIGHT_B, CHAN_7_LIGHT_B,
        REC_LIGHT, 
        NUM_LIGHTS
    };

    DigitalDisplay* digitalDisplay = nullptr;
    DigitalDisplay* chanDisplays[7] = {nullptr};
   
    dsp::SchmittTrigger resetTrigger, forwardTrigger, backwardTrigger, recTrigger, forwardInput, backwardInput, recInput, resetInput;
    dsp::SchmittTrigger channelButtonTriggers[7];
    dsp::PulseGenerator pulseGens[7]; 
 
    int currentStage = 0;
    int maxStages = 16;
    float maxNumStages = 16.0f; //float version of maxStages
    int prevMaxStages = -1;
    int channelButton[7] = {0}; // store button press state for each channel (0, 1, 2)
    float outputValues[2048][7] = {{0.0f}}; // 2D array to store output values for each stage and channel
                                            // array size is defaulted to the largest possible size used in the module.
    int lengthMultiplier = 1.0f;
    bool initializingFlag = true;
    int maxSequenceLength = 128;  //default to 128
    int prevMaxSequenceLength = 128;
    bool resizeEvent = false;
    
    // To store the current state for the latch es   
    bool resetLatched = false;     
    bool forwardLatched = false;  
    bool backwardLatched = false; 
    // Previous state variables to detect rising edges
    bool prevResetState = false; 
    bool prevForwardState = false; 
    bool prevBackwardState = false;
    bool recordLatched = false;
    bool prevRecordState = false;
    bool computedProb[7] = {false};
    bool enablePolyOut = false;
    bool prevEnablePolyOut = false;
    bool stopRecordAtEnd = false;

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
    
        // Store channelButton array as a JSON array
        json_t* channelButtonJ = json_array();
        for (int i = 0; i < 7; i++) {
            json_array_append_new(channelButtonJ, json_integer(channelButton[i]));
        }
        json_object_set_new(rootJ, "channelButton", channelButtonJ);
    
        // Store outputValues 2D array as a JSON array of arrays
        json_t* outputValuesJ = json_array();
        for (int stage = 0; stage < 2048; stage++) {
            json_t* stageArrayJ = json_array();
            for (int channel = 0; channel < 7; channel++) {
                json_array_append_new(stageArrayJ, json_real(outputValues[stage][channel]));
            }
            json_array_append_new(outputValuesJ, stageArrayJ);
        }
        json_object_set_new(rootJ, "outputValues", outputValuesJ);
    
        // Store recordLatched and prevRecordState as JSON booleans
        json_object_set_new(rootJ, "recordLatched", json_boolean(recordLatched));
        json_object_set_new(rootJ, "prevRecordState", json_boolean(prevRecordState));
        json_object_set_new(rootJ, "stopRecordAtEnd", json_boolean(stopRecordAtEnd));

    
        // Store computedProb array as a JSON array
        json_t* computedProbJ = json_array();
        for (int i = 0; i < 7; i++) {
            json_array_append_new(computedProbJ, json_boolean(computedProb[i]));
        }
        json_object_set_new(rootJ, "computedProb", computedProbJ);
    
        // Store enablePolyOut as a JSON boolean
        json_object_set_new(rootJ, "enablePolyOut", json_boolean(enablePolyOut));

        // Store maxSequenceLength as a JSON integer
        json_object_set_new(rootJ, "maxSequenceLength", json_integer(maxSequenceLength)); 
   
        return rootJ;
    }
        
    void dataFromJson(json_t* rootJ) override {
        // Load channelButton array
        json_t* channelButtonJ = json_object_get(rootJ, "channelButton");
        if (channelButtonJ) {
            for (int i = 0; i < 7; i++) {
                json_t* buttonJ = json_array_get(channelButtonJ, i);
                if (buttonJ)
                    channelButton[i] = json_integer_value(buttonJ);
            }
        }
    
        // Load outputValues 2D array
        json_t* outputValuesJ = json_object_get(rootJ, "outputValues");
        if (outputValuesJ) {
            for (int stage = 0; stage < 2048; stage++) {
                json_t* stageArrayJ = json_array_get(outputValuesJ, stage);
                if (stageArrayJ) {
                    for (int channel = 0; channel < 7; channel++) {
                        json_t* valueJ = json_array_get(stageArrayJ, channel);
                        if (valueJ)
                            outputValues[stage][channel] = json_number_value(valueJ);
                    }
                }
            }
        }
    
        // Load recordLatched and prevRecordState
        json_t* recordLatchedJ = json_object_get(rootJ, "recordLatched");
        if (recordLatchedJ) {
            recordLatched = json_is_true(recordLatchedJ);
        }
    
        json_t* prevRecordStateJ = json_object_get(rootJ, "prevRecordState");
        if (prevRecordStateJ) {
            prevRecordState = json_is_true(prevRecordStateJ);
        }

        json_t* stopRecordAtEndJ = json_object_get(rootJ, "stopRecordAtEnd");
        if (stopRecordAtEndJ) {
            stopRecordAtEnd = json_is_true(stopRecordAtEndJ);
        }
 
        // Load computedProb array
        json_t* computedProbJ = json_object_get(rootJ, "computedProb");
        if (computedProbJ) {
            for (int i = 0; i < 7; i++) {
                json_t* probJ = json_array_get(computedProbJ, i);
                if (probJ)
                    computedProb[i] = json_is_true(probJ);
            }
        }
    
        // Load enablePolyOut
        json_t* enablePolyOutJ = json_object_get(rootJ, "enablePolyOut");
        if (enablePolyOutJ) {
            enablePolyOut = json_is_true(enablePolyOutJ);
        }
        
        // Load maxSequenceLength
        json_t* maxSequenceLengthJ = json_object_get(rootJ, "maxSequenceLength");
        if (maxSequenceLengthJ) {
            maxSequenceLength = json_integer_value(maxSequenceLengthJ); // Set maxSequenceLength
        }     
    }

    Arrange() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        configParam(MAX_STAGES, 0.f, 128.f, 16.f, "Max Stages");
        configParam(STAGE_SELECT, 0.f, 1.f, 0.f, "Stage");

        // Button parameters
        configParam(FORWARD_BUTTON, 0.f, 1.f, 0.f, "Forward");
        configParam(BACKWARDS_BUTTON, 0.f, 1.f, 0.f, "Backward");
        configParam(RESET_BUTTON, 0.f, 1.f, 0.f, "Reset");
        configParam(REC_BUTTON, 0.f, 1.f, 0.f, "Record");
        configParam(CHAN_1_BUTTON, 0.f, 1.f, 0.f, "Channel 1 Mode");
        configParam(CHAN_2_BUTTON, 0.f, 1.f, 0.f, "Channel 2 Mode");
        configParam(CHAN_3_BUTTON, 0.f, 1.f, 0.f, "Channel 3 Mode");
        configParam(CHAN_4_BUTTON, 0.f, 1.f, 0.f, "Channel 4 Mode");
        configParam(CHAN_5_BUTTON, 0.f, 1.f, 0.f, "Channel 5 Mode");
        configParam(CHAN_6_BUTTON, 0.f, 1.f, 0.f, "Channel 6 Mode");
        configParam(CHAN_7_BUTTON, 0.f, 1.f, 0.f, "Channel 7 Mode");
    
        // Knob parameters for each channel
        configParam(CHAN_1_KNOB, -10.f, 10.f, 0.f, "Channel 1");
        configParam(CHAN_2_KNOB, -10.f, 10.f, 0.f, "Channel 2");
        configParam(CHAN_3_KNOB, -10.f, 10.f, 0.f, "Channel 3");
        configParam(CHAN_4_KNOB, -10.f, 10.f, 0.f, "Channel 4");
        configParam(CHAN_5_KNOB, -10.f, 10.f, 0.f, "Channel 5");
        configParam(CHAN_6_KNOB, -10.f, 10.f, 0.f, "Channel 6");
        configParam(CHAN_7_KNOB, -10.f, 10.f, 0.f, "Channel 7");
    
        // Configure inputs
        configInput(RESET_INPUT, "Reset");
        configInput(FORWARD_INPUT, "Forward");
        configInput(BACKWARDS_INPUT, "Backward");
        configInput(REC_INPUT, "Record");
        configInput(CHAN_1_INPUT, "Channel 1");
        configInput(CHAN_2_INPUT, "Channel 2");
        configInput(CHAN_3_INPUT, "Channel 3");
        configInput(CHAN_4_INPUT, "Channel 4");
        configInput(CHAN_5_INPUT, "Channel 5");
        configInput(CHAN_6_INPUT, "Channel 6");
        configInput(CHAN_7_INPUT, "Channel 7");
    
        // Configure outputs
        configOutput(CHAN_1_OUTPUT, "Channel 1");
        configOutput(CHAN_2_OUTPUT, "Channel 2");
        configOutput(CHAN_3_OUTPUT, "Channel 3");
        configOutput(CHAN_4_OUTPUT, "Channel 4");
        configOutput(CHAN_5_OUTPUT, "Channel 5");
        configOutput(CHAN_6_OUTPUT, "Channel 6");
        configOutput(CHAN_7_OUTPUT, "Channel 7");
    }

    void onRandomize(const RandomizeEvent& e) override {
        // Randomize only the channel knob parameters
        for (int i = 0; i < 7; i++) {
            params[CHAN_1_KNOB + i].setValue(random::uniform()*10-5);  //randomizes to -5...5V
        }
    }

    void onReset(const ResetEvent& e) override {
        for (int stage = 0; stage < maxSequenceLength; stage++) {  // Reset values only
            for (int channel = 0; channel < 7; channel++) {
                outputValues[stage][channel] = 0.0f;  // Reset each element to 0.0f
            }
        }
        for (int i = 0; i < 7; i++) {
            params[CHAN_1_KNOB + i].setValue(0.f);  //set param knobs to zero
        }       
    }

    void process(const ProcessArgs &args) override {    

        if (initializingFlag) {
            for (int i = 0; i < 7; i++) {
                // Recall the output values for the current stage and set them to the knobs
                float recalledValue = outputValues[currentStage][i]; // Get the stored value for the current stage
                paramQuantities[CHAN_1_KNOB + i]->setDisplayValue(recalledValue); 
            }
            initializingFlag = false;
        }

        // Store the current stage before processing the buttons
        int previousStage = currentStage;
        bool resizeEvent = false;

        // Calculate maxNumStages
        maxNumStages = params[MAX_STAGES].getValue()*lengthMultiplier;
        if (maxNumStages < 1) { maxNumStages = 1; } // Fewest stages allowed is 1 
        
        // Check if maxSequenceLength has changed
        if (prevMaxSequenceLength != maxSequenceLength) {
            // Sequence length has changed 
            lengthMultiplier = static_cast<int>(maxSequenceLength / 128);
            paramQuantities[MAX_STAGES]->displayMultiplier = lengthMultiplier;
            if (currentStage < maxNumStages) {currentStage = static_cast<int>(maxNumStages);}
            prevMaxSequenceLength = maxSequenceLength;
            resizeEvent = true;
        } 

        maxStages = static_cast<int>(maxNumStages);
        
        currentStage = round(params[STAGE_SELECT].getValue() * (maxStages - 1));
        
        // Dynamically reconfigure the Stage knob based on the Max, if Max changes
        if (maxStages != prevMaxStages) {
            paramQuantities[STAGE_SELECT]->setDisplayValue(0.f);
            paramQuantities[STAGE_SELECT]->displayMultiplier = (maxStages - 1);            
            prevMaxStages = maxStages;        
            resizeEvent = true;
        }
    
        // Handle button press for Forward
        if (forwardTrigger.process(params[FORWARD_BUTTON].getValue())) {
            currentStage++; // Increment stage
            if (currentStage >= maxStages) {
                currentStage = 0;  // Loop back to the start if needed
                if (stopRecordAtEnd && recordLatched){
                    recordLatched = false;
                }
            }
            paramQuantities[STAGE_SELECT]->setDisplayValue(currentStage);
        } else if (inputs[FORWARD_INPUT].isConnected()) {
            bool forwardCurrentState = forwardInput.process(inputs[FORWARD_INPUT].getVoltage());
            if (forwardCurrentState && !prevForwardState) { // Rising edge detected
                currentStage++; // Increment stage

                if (currentStage >= maxStages) {
                    currentStage = 0;  // Loop back to the start if needed
                    if (stopRecordAtEnd && recordLatched){
                        recordLatched = false;
                    }
                }
                paramQuantities[STAGE_SELECT]->setDisplayValue(currentStage);
            }
            prevForwardState = forwardCurrentState; // Update previous state
        }
   
        // Handle button press for Backward
        if (backwardTrigger.process(params[BACKWARDS_BUTTON].getValue())) {
            currentStage--;
            if (currentStage < 0) {
                currentStage = maxStages - 1;  // Wrap around to the last stage
            }
            paramQuantities[STAGE_SELECT]->setDisplayValue(currentStage);
        } else if (inputs[BACKWARDS_INPUT].isConnected()) {
            bool backwardCurrentState = backwardInput.process(inputs[BACKWARDS_INPUT].getVoltage());
            if (backwardCurrentState && !prevBackwardState) { // Rising edge detected
                currentStage--;
                if (currentStage < 0) {
                    currentStage = maxStages - 1;  // Wrap around to the last stage
                }
                paramQuantities[STAGE_SELECT]->setDisplayValue(currentStage);
            }
            prevBackwardState = backwardCurrentState; // Update previous state
        } 
 
        // Handle button press for Reset last
        if (resetTrigger.process(params[RESET_BUTTON].getValue())) {
            currentStage = 0;  // Reset to the first stage
            paramQuantities[STAGE_SELECT]->setDisplayValue(currentStage);
        } else if (inputs[RESET_INPUT].isConnected()) {
            bool resetCurrentState = resetInput.process(inputs[RESET_INPUT].getVoltage());
            if (resetCurrentState && !prevResetState) { // Rising edge detected
                currentStage = 0;  // Reset to the first stage
                paramQuantities[STAGE_SELECT]->setDisplayValue(currentStage);
            }
            prevResetState = resetCurrentState; // Update previous state
        }
                    
        // If the current stage has changed, recall values for the knobs
        if ( (currentStage != previousStage) || resizeEvent){
            for (int i = 0; i < 7; i++) {
                // Recall the output values for the current stage and set them to the knobs
                float recalledValue = outputValues[currentStage][i]; // Get the stored value for the current stage
                paramQuantities[CHAN_1_KNOB + i]->setDisplayValue(recalledValue); 
            
                // Generate a random value for each gate based on the probability
                float randVal = random::uniform(); // Generate a random number between 0 and 1
                if (randVal < ((recalledValue+10.f)/20.f) ) {
                    computedProb[i] = true; // High gate (10V) if random value is less than input probability
                    pulseGens[i].trigger(0.001f);  // Trigger a 1ms pulse (0.001 seconds)
                } else {
                    computedProb[i] = false;  // Low gate (0V) if random value is greater than input probability
                }
            }          
        }
        
        // Check for toggle button states and cycle them through three modes
        for (int i = 0; i < 7; i++) {
            // Use a Schmitt Trigger to detect the rising edge
            if (channelButtonTriggers[i].process(params[CHAN_1_BUTTON + i].getValue())) {
                channelButton[i] = (channelButton[i] + 1) % 3; // Cycle through 0, 1, 2
            }
        }

        bool recordStateChange = false;

        // Handle button press for Record
        if (recTrigger.process(params[REC_BUTTON].getValue())) {
            recordLatched = !recordLatched;  // Toggle the recording state
        } else if (inputs[REC_INPUT].isConnected()) {
            bool recCurrentState = recInput.process(inputs[REC_INPUT].getVoltage());
            if (recCurrentState && !prevRecordState) {  // Rising edge detected
                recordLatched = !recordLatched;  // Toggle the recording state
                recordStateChange = true;
            }
            prevRecordState = recCurrentState;  // Update previous state
        }       
        // Update the REC_LIGHT based on recordLatched state
        lights[REC_LIGHT].setBrightness(recordLatched ? 1.0f : 0.0f);

        // Process the final outputs and recording of inputs
        if (recordLatched || recordStateChange){       
            // Check if the channel has polyphonic input
            int inputChannels[7] = {0};   // Number of polyphonic channels for Input CV inputs
            
            // Arrays to store the current input signals and connectivity status
            int activeInputChannel[7] = {-1};   // Stores the number of the previous active channel for the Input CVs 
            //initialize all active channels with -1, indicating nothing connected.
         
            // Scan all inputs to determine the polyphony
            for (int i = 0; i < 7; i++) {                
                if (inputs[CHAN_1_INPUT + i].isConnected()) {
                    inputChannels[i] = inputs[CHAN_1_INPUT + i].getChannels();
                    activeInputChannel[i] = i;
                } else if (i > 0 && activeInputChannel[i-1] != -1) {
                    if (inputChannels[activeInputChannel[i-1]] > (i - activeInputChannel[i-1])) {
                        activeInputChannel[i] = activeInputChannel[i-1]; // Carry over the active channel
                    } else {
                        activeInputChannel[i] = -1; // No valid polyphonic channel to carry over
                    }
                } else {
                    activeInputChannel[i] = -1; // Explicitly reset if not connected
                }
            }       
                   
            //Prepare output and store into array
            for (int i = 0; i < 7; i++){
                // Measure inputs
                float knobVal = params[CHAN_1_KNOB + i].getValue(); //default to the knob value with no inputs
                float inputVal = knobVal; 
            
                if (activeInputChannel[i]==i) {
                    inputVal = inputs[CHAN_1_INPUT + i].getPolyVoltage(0);
                } else if (activeInputChannel[i] > -1){
                    // Now we compute which channel we need to grab
                    int diffBetween = i - activeInputChannel[i];
                    int currentChannelMax =  inputChannels[activeInputChannel[i]] ;    
                    if (currentChannelMax - diffBetween > 0) {    //If we are before the last poly channel
                        inputVal = inputs[CHAN_1_INPUT + activeInputChannel[i]].getPolyVoltage(diffBetween); 
                    }
                }
        
                if (channelButton[i]==0){
                    outputs[CHAN_1_OUTPUT + i].setVoltage(inputVal);//output the unmodified value
                } else if (channelButton[i]==1){               
                    //quantize the outputValue voltage by rounding to the nearest 1/12
                    inputVal = round(inputVal * 12.0f) / 12.0f;                  
                    outputs[CHAN_1_OUTPUT + i].setVoltage(inputVal);//output the quantized value
                } else if (channelButton[i] == 2) {            
                    // Check if the pulse is still high
                    if (pulseGens[i].process(args.sampleTime)) {
                        outputs[CHAN_1_OUTPUT + i].setVoltage(10.f);  // Output a 10V trigger
                    } else {
                        outputs[CHAN_1_OUTPUT + i].setVoltage(0.f);   // Otherwise, output 0V
                    }
                }  
                
                if (knobVal != inputVal){  //only update knobs to the set value if they are different
                    paramQuantities[CHAN_1_KNOB + i]->setDisplayValue(inputVal); 
                }
        
                //Store the output value in the 2D array for the current stage
                if (currentStage >= 0 && currentStage < 128) {
                    outputValues[currentStage][i] = inputVal; // Store input value at the current stage for the respective channel
                }
            } 
        } else {
            for (int i = 0; i < 7; i++){         
                float inputVal = params[CHAN_1_KNOB + i].getValue(); //else we recall the values and set the knobs       
                
                if (channelButton[i]==0){
                    outputs[CHAN_1_OUTPUT + i].setVoltage(inputVal);//output the unmodified value
                } else if (channelButton[i]==1){               
                    //quantize the outputValue voltage by rounding to the nearest 1/12
                    inputVal = round(inputVal * 12.0f) / 12.0f;                  
                    outputs[CHAN_1_OUTPUT + i].setVoltage(inputVal);//output the quantized value
                } else if (channelButton[i] == 2) {            
                    // Check if the pulse is still high
                    if (pulseGens[i].process(args.sampleTime)) {
                        outputs[CHAN_1_OUTPUT + i].setVoltage(10.f);  // Output a 10V trigger
                    } else {
                        outputs[CHAN_1_OUTPUT + i].setVoltage(0.f);   // Otherwise, output 0V
                    }
                }  
            }                    
        } 
        
        // Detect if the enablePolyOut state has changed
        if (enablePolyOut != prevEnablePolyOut) {
            if (enablePolyOut) {
                // Update tooltips to reflect polyphonic output
                configOutput(CHAN_1_OUTPUT, "Poly Channel 1");
            } else {
                // Revert tooltips to reflect monophonic output
                configOutput(CHAN_1_OUTPUT, "Channel 1");
            }
        
            // Update the previous state to the current state
            prevEnablePolyOut = enablePolyOut;
        }
        
        // Ensure correct number of channels is set based on the current state of enablePolyOut
        if (enablePolyOut) {
            outputs[CHAN_1_OUTPUT].setChannels(7);  // Set the number of channels to 7
            // Set the polyphonic voltage for the first output
            for ( int part = 1; part < 7; part++) {
                outputs[CHAN_1_OUTPUT].setVoltage(outputs[CHAN_1_OUTPUT + part].getVoltage(), part);  // Set voltage for the polyphonic channels
            }
        } else {
            outputs[CHAN_1_OUTPUT].setChannels(1);  // Set the number of channels to 1
        }

    }//void process
        
};

struct ProgressDisplay : TransparentWidget {
    Arrange* module;

    void drawLayer(const DrawArgs& args, int layer) override {
        if (!module || layer != 1) return;  // Only draw on the correct layer

        // Make sure we have a valid drawing area
        if (box.size.x <= 0 || box.size.y <= 0) return;  // Prevent any drawing if size is invalid

        // Clear the drawing area
        nvgBeginPath(args.vg);
        nvgRect(args.vg, 0, 0, box.size.x, box.size.y);
        nvgFillColor(args.vg, nvgRGBA(0, 0, 0, 0));  // Transparent background
        nvgFill(args.vg);

        // Variables for drawing
        int dotsToMake = module->maxStages;
        int currentDot = module->currentStage;
        float inactiveDotRadius = 2.0f;  // Inactive dot size, half of the active dot size
        float activeDotRadius = 4.0f;  // Active dot size (current stage)
        float yPosition = box.size.y * 0.5f;  // Centered vertically
        float dotSpacing = box.size.x / dotsToMake;  // Space between dots

        // Safety checks
        if (dotsToMake <= 0) dotsToMake = 1;  // Avoid division by zero
        
        // Draw the dots
        for (int i = 0; i < dotsToMake; i++) {
            float xPosition = i * dotSpacing + dotSpacing / 2;  // Center the dots within each segment

            nvgBeginPath(args.vg);
            
            // Check if this dot represents the current stage
            if (i == currentDot) {
                // Draw the current stage dot (active, larger size)
                nvgCircle(args.vg, xPosition, yPosition, activeDotRadius);
                nvgFillColor(args.vg, nvgRGBA(255, 255, 255, 255));  // Bright white color for active dot
            } else {
                // Draw inactive dots (smaller size)
                nvgCircle(args.vg, xPosition, yPosition, inactiveDotRadius);
                nvgFillColor(args.vg, nvgRGBA(100, 100, 100, 255));  // Light grey color for inactive dots
            }
            
            nvgFill(args.vg);
        }
    }
    
};

struct ArrangeWidget : ModuleWidget {
    ArrangeWidget(Arrange* module) {
        setModule(module);

        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Arrange.svg"),
            asset::plugin(pluginInstance, "res/Arrange-dark.svg")
        ));

        // Add screws or additional design elements as needed
        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        box.size = Vec(15 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT); 

        // Configure and add the first digital display
        DigitalDisplay* digitalDisplay = new DigitalDisplay();
        digitalDisplay->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        digitalDisplay->box.pos = Vec(41.5 + 25, 34); // Position on the module
        digitalDisplay->box.size = Vec(100, 18); // Size of the display
        digitalDisplay->text = "Stage : Max"; // Initial text
        digitalDisplay->fgColor = nvgRGB(208, 140, 89); // White color text
        digitalDisplay->textPos = Vec(0, 15); // Text position
        digitalDisplay->setFontSize(16.0f); // Set the font size as desired
        addChild(digitalDisplay);

        if (module) {
            module->digitalDisplay = digitalDisplay; // Link the module to the display
        }

        // Create and add the ProgressBar Display
        ProgressDisplay* progressDisplay = createWidget<ProgressDisplay>(Vec(46.5 + 25, 50)); // Positioning
        progressDisplay->box.size = Vec(90, 25); // Size of the display widget
        progressDisplay->module = module;
        addChild(progressDisplay);

        // Knobs
        addParam(createParamCentered<RoundBlackKnob>(Vec(20 + 25, 50), module, Arrange::STAGE_SELECT));
        addParam(createParamCentered<RoundBlackKnob>(Vec(160 + 25, 50), module, Arrange::MAX_STAGES));

        addParam(createParamCentered<TL1105>                  (Vec(45, 90), module, Arrange::REC_BUTTON));
        addInput(createInputCentered<ThemedPJ301MPort>        (Vec(20 , 90), module, Arrange::REC_INPUT));
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(45, 90), module, Arrange::REC_LIGHT));


        addParam(createParamCentered<TL1105>                  (Vec(100 , 90), module, Arrange::BACKWARDS_BUTTON));
        addInput(createInputCentered<ThemedPJ301MPort>        (Vec(75 , 90), module, Arrange::BACKWARDS_INPUT));

        addParam(createParamCentered<TL1105>                  (Vec(130 , 90), module, Arrange::FORWARD_BUTTON));
        addInput(createInputCentered<ThemedPJ301MPort>        (Vec(155 , 90), module, Arrange::FORWARD_INPUT));

        addParam(createParamCentered<TL1105>                  (Vec(185 , 90), module, Arrange::RESET_BUTTON));
        addInput(createInputCentered<ThemedPJ301MPort>        (Vec(210, 90), module, Arrange::RESET_INPUT));

        float initialYPos = 135; 
        float spacing = 35; 
        for (int i = 0; i < 7; ++i) {
            float yPos = initialYPos + i * spacing; // Adjusted positioning and spacing

            addInput(createInputCentered<ThemedPJ301MPort>        (Vec(20 , yPos), module, Arrange::CHAN_1_INPUT + i));
            addParam(createParamCentered<TL1105>                  (Vec(20 + 30, yPos), module, Arrange::CHAN_1_BUTTON + i));
            addChild(createLightCentered<LargeLight<BlueLight>>(Vec(20 + 30, yPos), module, Arrange::CHAN_1_LIGHT + i));
            addChild(createLightCentered<LargeLight<YellowLight>>(Vec(20 + 30, yPos), module, Arrange::CHAN_1_LIGHT_B + i));
            addParam(createParamCentered<RoundBlackKnob>          (Vec(50 + 35, yPos), module, Arrange::CHAN_1_KNOB + i));

            if (module) {
                // Ratio Displays Initialization
                module->chanDisplays[i] = createDigitalDisplay(Vec(75 + 40, yPos -  10), "Ready");
                addChild(module->chanDisplays[i]);
            }
 
            addOutput(createOutputCentered<ThemedPJ301MPort>    (Vec(157 + 45, yPos), module, Arrange::CHAN_1_OUTPUT + i));
        }
    }

    void draw(const DrawArgs& args) override {
        ModuleWidget::draw(args);
        Arrange* module = dynamic_cast<Arrange*>(this->module);
        if (!module) return;

        // Update Stage progress display
        if (module->digitalDisplay) {
            module->digitalDisplay->text =  std::to_string(module->currentStage + 1) + " / " + std::to_string(module->maxStages);
        }

        // Update channel quantizer displays
        for (int i = 0; i < 7; i++) {

            if (module->chanDisplays[i]) {
                if (module->channelButton[i]==0) {

                    char buffer[32]; // Create a buffer to hold the formatted string
                    float value = module->outputs[Arrange::CHAN_1_OUTPUT + i].getVoltage(); // Get the value
                    
                    // Format the value to 3 significant figures
                    snprintf(buffer, sizeof(buffer), "%.3f", value);
                    
                    // Set the formatted text
                    std::string voltageDisplay = std::string(buffer) + " V";
                    module->chanDisplays[i]->text = voltageDisplay;  
                    module->lights[Arrange::CHAN_1_LIGHT + i].setBrightness(0.0f); 
                    module->lights[Arrange::CHAN_1_LIGHT_B + i].setBrightness(0.0f); 
                
                } else if (module->channelButton[i]==1){
                    // Compute the note name of the note and the octave
                    float pitchVoltage = module->outputs[Arrange::CHAN_1_OUTPUT + i].getVoltage();
                    int octave = static_cast<int>(pitchVoltage + 4);  // The integer part represents the octave
                    
                    // Calculate the semitone
                    double fractionalPart = fmod(pitchVoltage, 1.0);
                    int semitone = round(fractionalPart * 12);
                    semitone = (semitone % 12 + 12) % 12;  // Ensure it's a valid semitone (0 to 11)
            
                    // Define the note names
                    const char* noteNames[12] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
                    const char* noteName = noteNames[semitone];
                    
                    // Create a buffer to hold the full note name
                    char fullNote[7];  // Enough space for note name + octave + null terminator
                    snprintf(fullNote, sizeof(fullNote), "%s%d", noteName, octave);  // Combine note and octave
                
                    module->chanDisplays[i]->text = fullNote;  
                    module->lights[Arrange::CHAN_1_LIGHT + i].setBrightness(1.0f); 
                    module->lights[Arrange::CHAN_1_LIGHT_B + i].setBrightness(0.0f); 
                    
                } else if (module->channelButton[i]==2){

                    // Get the current probability value (clamped between 0.0 and 1.0)
                    float probability =  (module->outputValues[module->currentStage][i] + 10.f)/20.f  ;
                    
                    // Convert the probability to a percentage (0 to 100)
                    int percentage = static_cast<int>(probability * 100.0f);
            
                    // Create a buffer to hold the percentage display
                    char percentageBuffer[8];  // Enough space for "100%" + null terminator
                    snprintf(percentageBuffer, sizeof(percentageBuffer), "%d%%", percentage);  // Format as a percentage
                    
                    // Set the percentage display
                    module->chanDisplays[i]->text = percentageBuffer;  
                    
                    // Set the brightness for the secondary light (CHAN_1_LIGHT_B) in this mode
                    module->lights[Arrange::CHAN_1_LIGHT + i].setBrightness(0.0f); 
                    module->lights[Arrange::CHAN_1_LIGHT_B + i].setBrightness(1.0f); 
                   
                }               
            }
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

    // Update the context menu structure and ensure correct function calling
    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
        
        // Cast the module to Arrange
        Arrange* arrangeModule = dynamic_cast<Arrange*>(module);
        assert(arrangeModule);
        
        // Separator for new section
        menu->addChild(new MenuSeparator);
        
        // Max Sequence Length Submenu
        struct MaxSequenceLengthSubMenu : MenuItem {
            Arrange* arrangeModule;
    
            Menu* createChildMenu() override {
                Menu* subMenu = new Menu;
                std::vector<int> lengths = {128, 256, 512, 1024, 2048}; 
                for (int length : lengths) {
                    struct MaxSequenceLengthItem : MenuItem {
                        Arrange* arrangeModule;
                        int sequenceLength;
    
                        void onAction(const event::Action& e) override {
                            // Set the new max sequence length in the module
                            arrangeModule->maxSequenceLength = sequenceLength;

                        }
    
                        void step() override {
                            rightText = (arrangeModule->maxSequenceLength == sequenceLength) ? "✔" : "";
                            MenuItem::step();
                        }
                    };
    
                    MaxSequenceLengthItem* lengthItem = new MaxSequenceLengthItem();
                    lengthItem->text = std::to_string(length);
                    lengthItem->arrangeModule = arrangeModule; // Pass the module to the item
                    lengthItem->sequenceLength = length; // Set the sequence length
                    subMenu->addChild(lengthItem);
                }
                return subMenu;
            }
        };
    
        MaxSequenceLengthSubMenu* maxLengthSubMenu = new MaxSequenceLengthSubMenu();
        maxLengthSubMenu->text = "Set Max Sequence Length";
        maxLengthSubMenu->arrangeModule = arrangeModule; // Pass the module to the submenu
        menu->addChild(maxLengthSubMenu);

        // Separator for new section
        menu->addChild(new MenuSeparator);
        
        // Stop Record At End Menu Item
        struct StopRecordAtEndItem : MenuItem {
            Arrange* arrangeModule;
        
            void onAction(const event::Action& e) override {
                // Toggle the stopRecordAtEnd variable in the module
                arrangeModule->stopRecordAtEnd = !arrangeModule->stopRecordAtEnd;
            }
        
            void step() override {
                rightText = arrangeModule->stopRecordAtEnd ? "✔" : ""; // Show checkmark if true
                MenuItem::step();
            }
        };
        
        // Create the Stop Record At End menu item
        StopRecordAtEndItem* stopRecordAtEndItem = new StopRecordAtEndItem();
        stopRecordAtEndItem->text = "Stop Record At End"; // Set menu item text
        stopRecordAtEndItem->arrangeModule = arrangeModule; // Pass the module to the item
        menu->addChild(stopRecordAtEndItem); // Add the item to the menu

        // Separator for new section
        menu->addChild(new MenuSeparator);
        
        // Enable Poly Output
        struct EnablePolyOutItem : MenuItem {
            Arrange* arrangeModule;
        
            void onAction(const event::Action& e) override {
                // Toggle the enablePolyOut variable in the module
                arrangeModule->enablePolyOut = !arrangeModule->enablePolyOut;
            }
        
            void step() override {
                rightText = arrangeModule->enablePolyOut ? "✔" : ""; // Show checkmark if true
                MenuItem::step();
            }
        };
        
        // Create the Enable Poly Out menu item
        EnablePolyOutItem* enablePolyOutItem = new EnablePolyOutItem();
        enablePolyOutItem->text = "Enable Poly Out"; // Set menu item text
        enablePolyOutItem->arrangeModule = arrangeModule; // Pass the module to the item
        menu->addChild(enablePolyOutItem); // Add the item to the menu

    }    
};

Model* modelArrange = createModel<Arrange, ArrangeWidget>("Arrange");