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
        NUM_OUTPUTS
    };
   enum LightIds {    
        NUM_LIGHTS
    };

    float inputValue = 0.f;
    DigitalDisplay* voltDisplay = nullptr;

    Morta() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        // Configure the parameters
        configParam(MASTER_KNOB, -10.0f, 10.0f, 0.0f, "Master Knob");
        configParam(RANGE_KNOB, 0.0f, 10.0f, 5.0f, "Range Knob");
        configParam(RANGE_TRIMPOT, -1.0f, 1.0f, 0.0f, "Range Attenuvertor");

        // Configure the inputs
        configInput(MAIN_INPUT, "Main");
        configInput(RANGE_CV_INPUT, "Range CV");

        // Configure the outputs
        for (int i = 0; i < 16; i++) {
            configOutput(OUTPUT_1_1 + i, "Output " + std::to_string(i / 4 + 1) + "," + std::to_string(i % 4 + 1));
        }
    }

    void process(const ProcessArgs &args) override {

        // Override master knob value with input if input is connected
        if (inputs[MAIN_INPUT].isConnected()) {
            params[MASTER_KNOB].setValue(inputs[MAIN_INPUT].getVoltage());
        }

        // Handle range for the fourth column
        float rangeCV = inputs[RANGE_CV_INPUT].isConnected() ? inputs[RANGE_CV_INPUT].getVoltage() : 0.0f;
        float customRange = params[RANGE_KNOB].getValue() + rangeCV * params[RANGE_TRIMPOT].getValue();

        inputValue = params[MASTER_KNOB].getValue(); 

        // Compute the scaled values
        float scaledValues[4][4] = {
            {inputValue / 20.0f + 0.5f, inputValue / 4.0f + 2.5f, inputValue / 2.0f + 5.f, (inputValue / 20.0f + 0.5f) * customRange}, // Unipolar 0 to Max
            {inputValue / 10.0f , inputValue / 2.0f , inputValue  , (inputValue / 10.0f) * customRange }, // Bipolar -Max to Max
            {-inputValue / 10.0f, -inputValue / 2.0f , -inputValue , (-inputValue / 10.0f)  * customRange }, // Inverted Bipolar -Max to Max
            {0.5f - inputValue / 20.0f, 2.5f - inputValue / 4.0f, 5.0f - inputValue / 2.0f, customRange - (inputValue / 20.0f + 0.5f) * customRange} // Positive Inverted Max to 0
        };

        // Update outputs
        for (int row = 0; row < 4; row++) {
            for (int col = 0; col < 4; col++) {
                outputs[OUTPUT_1_1 + row * 4 + col].setVoltage(scaledValues[row][col]);
            }
        }
    }
};

struct MortaWidget : ModuleWidget {
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

        // Central giant knob
        addParam(createParamCentered<RoundHugeBlackKnob>(Vec(box.size.x / 2, 70), module, Morta::MASTER_KNOB));

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
            snprintf(voltText, sizeof(voltText), "%.3f V", module->inputValue);
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