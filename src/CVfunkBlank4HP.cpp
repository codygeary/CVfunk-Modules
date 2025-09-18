////////////////////////////////////////////////////////////
//
//   CVfunkBlank4HP
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//
////////////////////////////////////////////////////////////

#include "plugin.hpp"

struct CVfunkBlank4HP : Module {

    enum ParamId {
        PARAMS_LEN
    };
    enum InputId {
        INPUTS_LEN
    };
    enum OutputId {
        OUTPUTS_LEN
    };
    enum LightId {
        LIGHTS_LEN
    };

    CVfunkBlank4HP() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
    }

    void process(const ProcessArgs& args) override {
     
    }

};

struct CVfunkBlank4HPWidget : ModuleWidget {

    CVfunkBlank4HPWidget(CVfunkBlank4HP* module) {
        setModule(module);
        
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/CVfunkBlank4HP.svg"),
            asset::plugin(pluginInstance, "res/CVfunkBlank4HP-dark.svg")
        ));
        
        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

    }
};

Model* modelCVfunkBlank4HP = createModel<CVfunkBlank4HP, CVfunkBlank4HPWidget>("CVfunkBlank4HP");