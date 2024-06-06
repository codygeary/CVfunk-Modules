////////////////////////////////////////////////////////////
//
//   Nona
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   Nine-stage CV mixing utility
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
using namespace rack;

struct Nona : Module {
    enum ParamIds {
        GAIN_KNOB_1, GAIN_KNOB_2, GAIN_KNOB_3,
        GAIN_KNOB_4, GAIN_KNOB_5, GAIN_KNOB_6,
        GAIN_KNOB_7, GAIN_KNOB_8, GAIN_KNOB_9,
        OFFSET_KNOB_1, OFFSET_KNOB_2, OFFSET_KNOB_3,
        OFFSET_KNOB_4, OFFSET_KNOB_5, OFFSET_KNOB_6,
        OFFSET_KNOB_7, OFFSET_KNOB_8, OFFSET_KNOB_9,
        NUM_PARAMS
    };
    enum InputIds {
        INPUT_1, INPUT_2, INPUT_3,     
        INPUT_4, INPUT_5, INPUT_6,      
        INPUT_7, INPUT_8, INPUT_9,      
        NUM_INPUTS
    };
    enum OutputIds {
        OUTPUT_1, OUTPUT_2, OUTPUT_3,
        OUTPUT_4, OUTPUT_5, OUTPUT_6,
        OUTPUT_7, OUTPUT_8, OUTPUT_9,
       NUM_OUTPUTS
    };
    enum LightIds {    
        NUM_LIGHTS
    };

    Nona() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        for (int i = 0; i < 9; i++) {
            configInput(INPUT_1 + i, "In " + std::to_string(i + 1));
        }
        for (int i = 0; i < 9; i++) {
            configOutput(OUTPUT_1 + i, "Out " + std::to_string(i + 1));
        }
        for (int i = 0; i < 9; i++) {
            configParam(GAIN_KNOB_1 + i, -2.0f, 2.0f, 0.0f, "Gain " + std::to_string(i + 1));
        }
        for (int i = 0; i < 9; i++) {
            configParam(OFFSET_KNOB_1 + i, -5.0f, 5.0f, 0.0f,  "Offset " + std::to_string(i + 1));
        }
    }

    void process(const ProcessArgs &args) override {
        // Initialize arrays to store the input voltages, gain, and offset values
        float inputVoltages[9] = {0.f};
        float gains[9] = {0.f};
        float offsets[9] = {0.f};
        float outputVoltages[9] = {0.f};
        bool outputActive[9] = {false};

        // Read all inputs, gains, and offsets first
        for (int i = 0; i < 9; i++) {
            if (inputs[INPUT_1 + i].isConnected()) {
                inputVoltages[i] = inputs[INPUT_1 + i].getVoltage();
            }
            if (outputs[OUTPUT_1 + i].isConnected()) {
                outputActive[i] = true;
            }
            gains[i] = params[GAIN_KNOB_1 + i].getValue();
            offsets[i] = params[OFFSET_KNOB_1 + i].getValue();
        }

        // Process each stage: Calculate the raw output voltage for each stage
        for (int i = 0; i < 9; i++) {
            outputVoltages[i] = inputVoltages[i] * gains[i] + offsets[i];
        }

        // Process each output with normalling
        for (int i = 0; i < 9; i++) {
            float outputMix = 0.f;
            for (int k = i; k >= 0; k--) {
                if (outputActive[k] && k != i) {
                    break;
                }
                outputMix += outputVoltages[k];
            }
            outputMix = clamp (outputMix, -10.f, 10.f);
            if (outputActive[i]) {
                outputs[OUTPUT_1 + i].setVoltage(outputMix);
            } else {
                outputs[OUTPUT_1 + i].setVoltage(0.f);
            }
        }
    }
};

struct NonaWidget : ModuleWidget {
    NonaWidget(Nona* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Nona.svg"),
            asset::plugin(pluginInstance, "res/Nona-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        for (int i = 0; i < 9; i++) {
            addInput(createInputCentered<ThemedPJ301MPort>           (Vec( 22, 35 + i * 38 ), module, Nona::INPUT_1 + i));
            addParam(createParamCentered<RoundSmallBlackKnob>             (Vec( 57, 40 + i * 38 ), module, Nona::GAIN_KNOB_1 + i));
            addParam(createParamCentered<RoundSmallBlackKnob>             (Vec( 92, 40 + i * 38 ), module, Nona::OFFSET_KNOB_1 + i));
            addOutput(createOutputCentered<ThemedPJ301MPort>         (Vec( 127, 35 + i * 38 ), module, Nona::OUTPUT_1 + i));
        }
    }
};

Model* modelNona = createModel<Nona, NonaWidget>("Nona");