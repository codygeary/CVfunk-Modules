////////////////////////////////////////////////////////////
//
//   Morta
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   A single knob macro-controller
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
#include "digital_display.hpp" // Include the DigitalDisplay header
using namespace rack;

struct Morta : Module {
    enum ParamIds {
        MASTER_KNOB,
        RANGE_KNOB,
        RANGE_TRIMPOT,
        MAIN_GAIN,
        MAIN_OFFSET,
        NUM_PARAMS
    };
    enum InputIds {
        MAIN_INPUT,
        RANGE_CV_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        OUTPUT_1_1, OUTPUT_1_2, OUTPUT_1_3, OUTPUT_1_4,
        OUTPUT_2_1, OUTPUT_2_2, OUTPUT_2_3, OUTPUT_2_4,
        OUTPUT_3_1, OUTPUT_3_2, OUTPUT_3_3, OUTPUT_3_4,
        OUTPUT_4_1, OUTPUT_4_2, OUTPUT_4_3, OUTPUT_4_4,
        MAIN_OUTPUT,
        NUM_OUTPUTS
    };
   enum LightIds {    
        NUM_LIGHTS
    };

    std::atomic<bool> isEditing[1]; //For the master smart knob

    float inputValue = 0.f;
    float displayValue = 0.0f;
    DigitalDisplay* voltDisplay = nullptr;

    Morta() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        // Configure the parameters
        configParam(MASTER_KNOB, -10.0f, 10.0f, 0.0f, "Master Knob");
        configParam(RANGE_KNOB, 0.0f, 10.0f, 5.0f, "Range Knob");
        configParam(RANGE_TRIMPOT, -1.0f, 1.0f, 0.0f, "Range Attenuvertor");

        configParam(MAIN_GAIN, -2.0f, 2.0f, 1.0f, "Input Gain");
        configParam(MAIN_OFFSET, -10.0f, 10.0f, 0.0f, "Input Offset");

        // Configure the inputs
        configInput(MAIN_INPUT, "Main");
        configInput(RANGE_CV_INPUT, "Range CV");
        
        configOutput(OUTPUT_1_1, "0→1" );
        configOutput(OUTPUT_1_2, "0→5" );
        configOutput(OUTPUT_1_3, "0→10");
        configOutput(OUTPUT_1_4, "0→R" );

        configOutput(OUTPUT_2_1, "-1→1" );
        configOutput(OUTPUT_2_2, "-5→5" );
        configOutput(OUTPUT_2_3, "-10→10");
        configOutput(OUTPUT_2_4, "-R→R" );

        configOutput(OUTPUT_3_1, "1→-1" );
        configOutput(OUTPUT_3_2, "5→-5" );
        configOutput(OUTPUT_3_3, "10→-10");
        configOutput(OUTPUT_3_4, "R→-R" );

        configOutput(OUTPUT_4_1, "1→0" );
        configOutput(OUTPUT_4_2, "5→0" );
        configOutput(OUTPUT_4_3, "10→0");
        configOutput(OUTPUT_4_4, "R→0" );
      
        isEditing[0] = false; // Initialize editing state to false    
    }

    void process(const ProcessArgs &args) override {
    
        // Determine the number of channels based on the RANGE_CV_INPUT or MAIN_INPUT
        int numChannels = std::max(inputs[RANGE_CV_INPUT].getChannels(), inputs[MAIN_INPUT].getChannels());
        if (numChannels == 0) numChannels = 1;  // Default to mono if no inputs are connected
    
        // Set the output channels to match the input channels
        for (int i = 0; i < 16; i++) {
            outputs[OUTPUT_1_1 + i].setChannels(numChannels);
        }
        outputs[MAIN_OUTPUT].setChannels(numChannels);
    
        // Check if RANGE_CV_INPUT is monophonic
        bool isRangeCVMonophonic = inputs[RANGE_CV_INPUT].isConnected() && (inputs[RANGE_CV_INPUT].getChannels() == 1);
    
        // Get the monophonic RANGE_CV value if applicable
        float rangeCVMonoValue = isRangeCVMonophonic ? inputs[RANGE_CV_INPUT].getVoltage(0) : 0.0f;
    
        for (int c = 0; c < numChannels; c++) {
    
            // Override master knob value with input if input is connected
            float topChannelVoltage = inputs[MAIN_INPUT].getVoltage(0);
            topChannelVoltage = topChannelVoltage*params[MAIN_GAIN].getValue() + params[MAIN_OFFSET].getValue();
            topChannelVoltage = clamp(topChannelVoltage, -10.f, 10.f);            
        
            if (inputs[MAIN_INPUT].isConnected()) {
                if (!isEditing[0]){
                    params[MASTER_KNOB].setValue(topChannelVoltage);
                    displayValue = inputs[MAIN_INPUT].getVoltage(0);
                    displayValue = displayValue*params[MAIN_GAIN].getValue() + params[MAIN_OFFSET].getValue();
                    displayValue = clamp(displayValue, -10.f, 10.f);

                } else {
                    displayValue = params[MASTER_KNOB].getValue();                           
                }
            } else {
                displayValue = params[MASTER_KNOB].getValue();           
            }
    
            // Handle range for the fourth column, applying monophonic CV if applicable
            float rangeCV = inputs[RANGE_CV_INPUT].isConnected() ? 
                            (isRangeCVMonophonic ? rangeCVMonoValue : inputs[RANGE_CV_INPUT].getVoltage(c)) : 0.0f;
            float customRange = params[RANGE_KNOB].getValue() + rangeCV * params[RANGE_TRIMPOT].getValue();


            float inputValue = 0.0f;

            if (inputs[MAIN_INPUT].isConnected()) {
                if (!isEditing[0]){
                    inputValue = inputs[MAIN_INPUT].getVoltage(c);
                    inputValue = inputValue*params[MAIN_GAIN].getValue() + params[MAIN_OFFSET].getValue();
                    inputValue = clamp(inputValue, -10.f, 10.f);
                    
                } else {
                    inputValue = params[MASTER_KNOB].getValue();          
                    inputValue = inputValue*params[MAIN_GAIN].getValue() + params[MAIN_OFFSET].getValue();
                    inputValue = clamp(inputValue, -10.f, 10.f);
                }
            } else {
                inputValue = params[MASTER_KNOB].getValue();
            }
  
            // Compute the scaled values for each mode
            float scaledValues[4][4] = {
                {inputValue / 20.0f + 0.5f, inputValue / 4.0f + 2.5f, inputValue / 2.0f + 5.f, (inputValue / 20.0f + 0.5f) * customRange}, // Unipolar 0 to Max
                {inputValue / 10.0f , inputValue / 2.0f , inputValue  , (inputValue / 10.0f) * customRange }, // Bipolar -Max to Max
                {-inputValue / 10.0f, -inputValue / 2.0f , -inputValue , (-inputValue / 10.0f)  * customRange }, // Inverted Bipolar -Max to Max
                {0.5f - inputValue / 20.0f, 2.5f - inputValue / 4.0f, 5.0f - inputValue / 2.0f, customRange - (inputValue / 20.0f + 0.5f) * customRange} // Positive Inverted Max to 0
            };
    
            // Update outputs for each channel
            for (int row = 0; row < 4; row++) {
                for (int col = 0; col < 4; col++) {
                    outputs[OUTPUT_1_1 + row * 4 + col].setVoltage(scaledValues[row][col], c);
                }
            }
    
            // Set the main output for this channel
            outputs[MAIN_OUTPUT].setVoltage(inputValue, c);
        }
    }

};

struct MortaWidget : ModuleWidget {

    //Define a SmartKnob that tracks if we are turning it
    template <typename BaseKnob>
    struct SmartKnob : BaseKnob {
        void onDragStart(const event::DragStart& e) override {
            if (ParamQuantity* paramQuantity = this->getParamQuantity()) {
                if (Morta* module = dynamic_cast<Morta*>(paramQuantity->module)) {
                    int index = paramQuantity->paramId - Morta::MASTER_KNOB; //instance of 1st smart knob in the group
                    if (index >= 0 && index < 1) { //for 1 smart knobs
                        module->isEditing[index].store(true);
                    }
                }
            }
            BaseKnob::onDragStart(e);
        }
    
        void onDragEnd(const event::DragEnd& e) override {
            if (ParamQuantity* paramQuantity = this->getParamQuantity()) {
                if (Morta* module = dynamic_cast<Morta*>(paramQuantity->module)) {
                    int index = paramQuantity->paramId - Morta::MASTER_KNOB;
                    if (index >= 0 && index < 1) { //for 1 smart knobs
                        module->isEditing[index].store(false);
                    }
                }
            }
            BaseKnob::onDragEnd(e);
        }
    };
    
    // Type aliases to apply 'Smart' to all the knob types we use
    using SmartRoundBlackKnob = SmartKnob<RoundBlackKnob>;
    using SmartTrimpot = SmartKnob<Trimpot>;
    using SmartRoundLargeBlackKnob = SmartKnob<RoundLargeBlackKnob>;
    using SmartRoundHugeBlackKnob = SmartKnob<RoundHugeBlackKnob>;


    MortaWidget(Morta* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Morta.svg"),
            asset::plugin(pluginInstance, "res/Morta-dark.svg")
        ));

        // Screws
        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // Main input and output at the top
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(box.size.x / 2 - 50, 70), module, Morta::MAIN_INPUT));

        addParam(createParamCentered<Trimpot>(Vec(box.size.x / 2 - 50, 45), module, Morta::MAIN_GAIN));
        addParam(createParamCentered<Trimpot>(Vec(box.size.x / 2 - 50, 95), module, Morta::MAIN_OFFSET));


        // Central giant knob
        addParam(createParamCentered<SmartRoundHugeBlackKnob>(Vec(box.size.x / 2, 70), module, Morta::MASTER_KNOB));

        // CV input, trimpot, and knob for the range control at the bottom
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(box.size.x / 2 + 30, 155), module, Morta::RANGE_CV_INPUT));
        addParam(createParamCentered<Trimpot>(Vec(box.size.x / 2, 155), module, Morta::RANGE_TRIMPOT));
        addParam(createParamCentered<RoundBlackKnob>(Vec(box.size.x / 2 - 30, 155), module, Morta::RANGE_KNOB));

        // 4x4 matrix of outputs at the bottom
        for (int row = 0; row < 4; row++) {
            for (int col = 0; col < 4; col++) {
                float x = box.size.x / 5 * (col + 0.5f) + box.size.x / 10 ;
                float y = 210 + row * 40;
                addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(x, y), module, Morta::OUTPUT_1_1 + row * 4 + col));
            }
        }

        //Main CV Output
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(box.size.x/2 + 55, 119), module, Morta::MAIN_OUTPUT));

        if (module) {
            // Volt Display Initialization
            module->voltDisplay = createDigitalDisplay(Vec(box.size.x / 2 - 25, 110), "0.000 V");
            addChild(module->voltDisplay);
        }

    }

    void draw(const DrawArgs& args) override {
        ModuleWidget::draw(args);
        Morta* module = dynamic_cast<Morta*>(this->module);
        if (!module) return;
    
        // Update BPM and Swing displays
        if (module->voltDisplay) {
            char voltText[16];
            
            // Display the input value for the top channel (channel 0)
            snprintf(voltText, sizeof(voltText), "%.3f V", module->displayValue); 
            
            module->voltDisplay->text = voltText;
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

Model* modelMorta = createModel<Morta, MortaWidget>("Morta");