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
        NUM_PARAMS
    };
    enum InputIds {
        RESET_INPUT,
        FORWARD_INPUT,
        BACKWARDS_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        CHAN_1_OUTPUT, CHAN_2_OUTPUT, CHAN_3_OUTPUT, CHAN_4_OUTPUT, CHAN_5_OUTPUT, CHAN_6_OUTPUT, CHAN_7_OUTPUT, 
        NUM_OUTPUTS
    };
    enum LightIds {
        CHAN_1_LIGHT, CHAN_2_LIGHT, CHAN_3_LIGHT, CHAN_4_LIGHT, CHAN_5_LIGHT, CHAN_6_LIGHT, CHAN_7_LIGHT, 
        NUM_LIGHTS
    };

    DigitalDisplay* digitalDisplay = nullptr;
    DigitalDisplay* chanDisplays[7] = {nullptr};
   
    dsp::SchmittTrigger resetTrigger, forwardTrigger, backwardTrigger;
    dsp::SchmittTrigger channelButtonTriggers[7];
    int currentStage = 0;
    int maxStages = 16;
    int prevMaxStages = -1;
    bool channelButton[7] = {false}; // store button press state for each channel
    float outputValues[128][7] = {{0.0f}}; // 2D array to store output values for each stage and channel
    bool initializingFlag = true;
    // To store the current state for the latch es   
    bool resetLatched = false;     
    bool forwardLatched = false;  
    bool backwardLatched = false; 
    // Previous state variables to detect rising edges
    bool prevResetState = false; 
    bool prevForwardState = false; 
    bool prevBackwardState = false;

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
    
        // Store channelButton array as a JSON array
        json_t* channelButtonJ = json_array();
        for (int i = 0; i < 7; i++) {
            json_array_append_new(channelButtonJ, json_boolean(channelButton[i]));
        }
        json_object_set_new(rootJ, "channelButton", channelButtonJ);
    
        // Store outputValues 2D array as a JSON array of arrays
        json_t* outputValuesJ = json_array();
        for (int stage = 0; stage < 128; stage++) {
            json_t* stageArrayJ = json_array();
            for (int channel = 0; channel < 7; channel++) {
                json_array_append_new(stageArrayJ, json_real(outputValues[stage][channel]));
            }
            json_array_append_new(outputValuesJ, stageArrayJ);
        }
        json_object_set_new(rootJ, "outputValues", outputValuesJ);
    
        return rootJ;
    }
    
    void dataFromJson(json_t* rootJ) override {
        // Load channelButton array
        json_t* channelButtonJ = json_object_get(rootJ, "channelButton");
        if (channelButtonJ) {
            for (int i = 0; i < 7; i++) {
                json_t* buttonJ = json_array_get(channelButtonJ, i);
                if (buttonJ)
                    channelButton[i] = json_boolean_value(buttonJ);
            }
        }
    
        // Load outputValues 2D array
        json_t* outputValuesJ = json_object_get(rootJ, "outputValues");
        if (outputValuesJ) {
            for (int stage = 0; stage < 128; stage++) {
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
    }

    Arrange() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        configParam(MAX_STAGES, 1.f, 128.f, 16.f, "Max Stages");
        configParam(STAGE_SELECT, 0.f, 1.f, 0.f, "Stage");

        // Button parameters
        configParam(FORWARD_BUTTON, 0.f, 1.f, 0.f, "Forward");
        configParam(BACKWARDS_BUTTON, 0.f, 1.f, 0.f, "Backward");
        configParam(RESET_BUTTON, 0.f, 1.f, 0.f, "Reset");
        configParam(CHAN_1_BUTTON, 0.f, 1.f, 0.f, "Channel 1 Voct/V");
        configParam(CHAN_2_BUTTON, 0.f, 1.f, 0.f, "Channel 2 Voct/V");
        configParam(CHAN_3_BUTTON, 0.f, 1.f, 0.f, "Channel 3 Voct/V");
        configParam(CHAN_4_BUTTON, 0.f, 1.f, 0.f, "Channel 4 Voct/V");
        configParam(CHAN_5_BUTTON, 0.f, 1.f, 0.f, "Channel 5 Voct/V");
        configParam(CHAN_6_BUTTON, 0.f, 1.f, 0.f, "Channel 6 Voct/V");
        configParam(CHAN_7_BUTTON, 0.f, 1.f, 0.f, "Channel 7 Voct/V");
    
        // Knob parameters for each channel
        configParam(CHAN_1_KNOB, -10.f, 10.f, 0.f, "Channel 1 Knob");
        configParam(CHAN_2_KNOB, -10.f, 10.f, 0.f, "Channel 2 Knob");
        configParam(CHAN_3_KNOB, -10.f, 10.f, 0.f, "Channel 3 Knob");
        configParam(CHAN_4_KNOB, -10.f, 10.f, 0.f, "Channel 4 Knob");
        configParam(CHAN_5_KNOB, -10.f, 10.f, 0.f, "Channel 5 Knob");
        configParam(CHAN_6_KNOB, -10.f, 10.f, 0.f, "Channel 6 Knob");
        configParam(CHAN_7_KNOB, -10.f, 10.f, 0.f, "Channel 7 Knob");
    
        // Configure inputs
        configInput(RESET_INPUT, "Reset");
        configInput(FORWARD_INPUT, "Forward");
        configInput(BACKWARDS_INPUT, "Backward");
    
        // Configure outputs
        configOutput(CHAN_1_OUTPUT, "Channel 1 Output");
        configOutput(CHAN_2_OUTPUT, "Channel 2 Output");
        configOutput(CHAN_3_OUTPUT, "Channel 3 Output");
        configOutput(CHAN_4_OUTPUT, "Channel 4 Output");
        configOutput(CHAN_5_OUTPUT, "Channel 5 Output");
        configOutput(CHAN_6_OUTPUT, "Channel 6 Output");
        configOutput(CHAN_7_OUTPUT, "Channel 7 Output");
    }

    void onRandomize(const RandomizeEvent& e) override {
        // Randomize only the channel knob parameters
        for (int i = 0; i < 7; i++) {
            params[CHAN_1_KNOB + i].setValue(random::uniform()*10-5);
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

        currentStage = floor(params[STAGE_SELECT].getValue()*(maxStages-1));
        if (currentStage < 0){ currentStage = 0; }
        if (currentStage > maxStages-1){ currentStage = maxStages-1; }

        maxStages = floor( params[MAX_STAGES].getValue() );
        if ( maxStages <= 0 ) {maxStages = 1;} //Fewest stages allowed is 1 

        // Dynamically reconfigure the Stage knob based on the Max, if Max changes
        if (maxStages != prevMaxStages){
            paramQuantities[STAGE_SELECT]->setDisplayValue(currentStage/maxStages);
            paramQuantities[STAGE_SELECT]->displayMultiplier = (maxStages-1);            
            prevMaxStages = maxStages;
            resizeEvent = true;
        }
  
        // Handle button press for Reset
        if (resetTrigger.process(params[RESET_BUTTON].getValue())) {
            currentStage = 0;  // Reset to the first stage
            paramQuantities[STAGE_SELECT]->setDisplayValue(currentStage);
        } else if (inputs[RESET_INPUT].isConnected()) {
            bool resetCurrentState = inputs[RESET_INPUT].getVoltage() > 0.05f;
            if (resetCurrentState && !prevResetState) { // Rising edge detected
                currentStage = 0;  // Reset to the first stage
                paramQuantities[STAGE_SELECT]->setDisplayValue(currentStage);
            }
            prevResetState = resetCurrentState; // Update previous state
        }

    
        // Handle button press for Forward
        if (forwardTrigger.process(params[FORWARD_BUTTON].getValue())) {
            currentStage++;
            if (currentStage >= maxStages) {
                currentStage = 0;  // Loop back to the start if needed
            }
            paramQuantities[STAGE_SELECT]->setDisplayValue(currentStage);
        } else if (inputs[FORWARD_INPUT].isConnected()) {
            bool forwardCurrentState = inputs[FORWARD_INPUT].getVoltage() > 0.05f;
            if (forwardCurrentState && !prevForwardState) { // Rising edge detected
                currentStage++;
                if (currentStage >= maxStages) {
                    currentStage = 0;  // Loop back to the start if needed
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
            bool backwardCurrentState = inputs[BACKWARDS_INPUT].getVoltage() > 0.05f;
            if (backwardCurrentState && !prevBackwardState) { // Rising edge detected
                currentStage--;
                if (currentStage < 0) {
                    currentStage = maxStages - 1;  // Wrap around to the last stage
                }
                paramQuantities[STAGE_SELECT]->setDisplayValue(currentStage);
            }
            prevBackwardState = backwardCurrentState; // Update previous state
        } 
                     
        // If the current stage has changed, recall values for the knobs
        if ( (currentStage != previousStage) || resizeEvent){
            for (int i = 0; i < 7; i++) {
                // Recall the output values for the current stage and set them to the knobs
                float recalledValue = outputValues[currentStage][i]; // Get the stored value for the current stage
                paramQuantities[CHAN_1_KNOB + i]->setDisplayValue(recalledValue); 
            }
        }

        // Check for toggle button states and toggle them
        for (int i = 0; i < 7; i++) {
            // Use a Schmitt Trigger to detect the rising edge
            if (channelButtonTriggers[i].process(params[CHAN_1_BUTTON + i].getValue())) {
                channelButton[i] = !channelButton[i]; // Toggle the channel button state
                if (channelButton[i]){
                    float outputValue = round(params[CHAN_1_KNOB + i].getValue() * 12.0f) / 12.0f;
                    paramQuantities[CHAN_1_KNOB + i]->setDisplayValue(outputValue);

                    // Store the output value in the 2D array for the current stage
                    if (currentStage >= 0 && currentStage < 128) {
                        outputValues[currentStage][i] = outputValue; // Store output value at the current stage for the respective channel
                    }
                }
            }            
        }

        //Prepare output and store into array
        for (int i = 0; i < 7; i++){
            float outputValue = params[CHAN_1_KNOB + i].getValue();

            if (channelButton[i]){
                //quantize the outputValue voltage by rounding to the nearest 1/12
                outputValue = round(outputValue * 12.0f) / 12.0f;
            }

            outputs[CHAN_1_OUTPUT + i].setVoltage(outputValue);

            // Store the output value in the 2D array for the current stage
            if (currentStage >= 0 && currentStage < 128) {
                outputValues[currentStage][i] = outputValue; // Store output value at the current stage for the respective channel
            }
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

        box.size = Vec(12 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT); // 8HP wide screen

        // Configure and add the first digital display
        DigitalDisplay* digitalDisplay = new DigitalDisplay();
        digitalDisplay->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        digitalDisplay->box.pos = Vec(41.5, 34); // Position on the module
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
        ProgressDisplay* progressDisplay = createWidget<ProgressDisplay>(Vec(46.5, 50)); // Positioning
        progressDisplay->box.size = Vec(90, 25); // Size of the display widget
        progressDisplay->module = module;
        addChild(progressDisplay);

        // Knobs
        addParam(createParamCentered<RoundBlackKnob>(Vec(20, 50), module, Arrange::STAGE_SELECT));
        addParam(createParamCentered<DiscreteRoundBlackKnob>(Vec(160, 50), module, Arrange::MAX_STAGES));

        addParam(createParamCentered<TL1105>                  (Vec(45, 90), module, Arrange::RESET_BUTTON));
        addInput(createInputCentered<ThemedPJ301MPort>        (Vec(20, 90), module, Arrange::RESET_INPUT));

        addParam(createParamCentered<TL1105>                  (Vec(100, 90), module, Arrange::BACKWARDS_BUTTON));
        addInput(createInputCentered<ThemedPJ301MPort>        (Vec(75, 90), module, Arrange::BACKWARDS_INPUT));

        addParam(createParamCentered<TL1105>                  (Vec(135, 90), module, Arrange::FORWARD_BUTTON));
        addInput(createInputCentered<ThemedPJ301MPort>        (Vec(160, 90), module, Arrange::FORWARD_INPUT));

        float initialYPos = 135; 
        float spacing = 35; 
        for (int i = 0; i < 7; ++i) {
            float yPos = initialYPos + i * spacing; // Adjusted positioning and spacing

            addParam(createParamCentered<TL1105>                  (Vec(20, yPos), module, Arrange::CHAN_1_BUTTON + i));
            addChild(createLightCentered<LargeLight<BlueLight>>(Vec(20, yPos), module, Arrange::CHAN_1_LIGHT + i));
            addParam(createParamCentered<RoundBlackKnob>          (Vec(50, yPos), module, Arrange::CHAN_1_KNOB + i));

            if (module) {
                // Ratio Displays Initialization
                module->chanDisplays[i] = createDigitalDisplay(Vec(75, yPos -  10), "Ready");
                addChild(module->chanDisplays[i]);
            }
 
            addOutput(createOutputCentered<ThemedPJ301MPort>    (Vec(157, yPos), module, Arrange::CHAN_1_OUTPUT + i));
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
                if (module->channelButton[i]) {
                
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
                } else {
                    char buffer[32]; // Create a buffer to hold the formatted string
                    float value = module->params[Arrange::CHAN_1_KNOB + i].getValue(); // Get the value
                    
                    // Format the value to 3 significant figures
                    snprintf(buffer, sizeof(buffer), "%.3f", value);
                    
                    // Set the formatted text
                    std::string voltageDisplay = std::string(buffer) + " V";
                    module->chanDisplays[i]->text = voltageDisplay;  
                    module->lights[Arrange::CHAN_1_LIGHT + i].setBrightness(0.0f); 
                    
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
  
};

Model* modelArrange = createModel<Arrange, ArrangeWidget>("Arrange");