////////////////////////////////////////////////////////////
//
//   Clpy
//
//   written by Cody Geary
//   Copyright 2025, MIT License
//
//   A wave-shaper that folds and clips to a target CV
//
////////////////////////////////////////////////////////////

#include "plugin.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

#include "Filter6pButter.h"
#define OVERSAMPLING_FACTOR 4

class OverSamplingShaper {
public:
    OverSamplingShaper() {
        // Configure 6-pole filters for oversampling / decimation
        interpolatingFilter.setCutoffFreq(1.f / (OVERSAMPLING_FACTOR * 4));
        decimatingFilter.setCutoffFreq(1.f / (OVERSAMPLING_FACTOR * 4));
    }

    float process(float input, float clipValue, bool symmetric, bool oversamplingEnabled) {
        if (!oversamplingEnabled) {
            lastClipValue = clipValue;
            return processShape(input, clipValue, symmetric);
        }

        // Linear interpolation of clip CV over the oversampled block
        float clipStep = (clipValue - lastClipValue) / OVERSAMPLING_FACTOR;
        float currentClip = lastClipValue;

        // Upsample into a temporary buffer
        float buffer[OVERSAMPLING_FACTOR];
        for (int i = 0; i < OVERSAMPLING_FACTOR; ++i) {
            float oversampledInput = (i == 0) ? input * OVERSAMPLING_FACTOR : 0.f;
            oversampledInput = interpolatingFilter.process(oversampledInput);

            currentClip += clipStep;
            buffer[i] = processShape(oversampledInput, currentClip, symmetric);
        }

        // Decimate the buffer back to single output sample
        float out = 0.f;
        for (int i = 0; i < OVERSAMPLING_FACTOR; ++i) {
            out = decimatingFilter.process(buffer[i]);
        }

        lastClipValue = clipValue;
        return out;
    }

private:
    float processShape(float input, float clipValue, bool symmetric) {
        return 5.f * waveshape(input * 0.2f, clipValue, symmetric);
    }

    inline float waveshape(float x, float C, bool symmetric) {
        constexpr float a = 0.926605548037825f;
        float core = polySin(x) * fastExpf(-4.f * x * x / (3.14159265f * 3.14159265f));
        float t = clamp((fabsf(x) - a) / (3.14159265f - a), 0.f, 1.f);
        t = t * t * (3.f - 2.f * t);
        float tail = symmetric ? ((x >= 0.0f) ? C : -C) : C;
        return core * (1.f - t) + tail * t;
    }

    float polySin(float x) {
        const float twoPi = 2.f * M_PI;
        x = fmod(x + M_PI, twoPi);
        if (x < 0.f) x += twoPi;
        x -= M_PI;

        float x2 = x*x, x3 = x*x2, x5 = x3*x2, x7 = x5*x2, x9 = x7*x2;
        return x - x3/6.f + x5/120.f - x7/5040.f + x9/362880.f;
    }

    inline float fastExpf(float x) {
        x = fmaxf(-10.f, fminf(10.f, x));
        return 1.f + x*(1.f + x*(0.499705f + x*(0.1687389f + x*(0.0366899f + x*0.0061537f))));
    }

    Filter6PButter interpolatingFilter;
    Filter6PButter decimatingFilter;
    float lastClipValue = 0.f; // Clip CV memory for interpolation
};

struct Clpy : Module {
    enum ParamId {
        GAIN_PARAM, GAIN_ATT_PARAM,
        CLIP_PARAM, CLIP_ATT_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        GAIN_INPUT,
        CLIP_L_INPUT, CLIP_R_INPUT,
        INL_INPUT,
        INR_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        OUTL_OUTPUT,
        OUTR_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        LIGHTS_LEN
    };

    bool symmetric = false;
    static constexpr float fourDivPiSqrd = 4.0f / (3.14159265f * 3.14159265f);

    // Initialize Butterworth filter for oversampling
    OverSamplingShaper shaperL[16];  // Instance of the oversampling and shaping processor
    OverSamplingShaper shaperR[16];  // Instance of the oversampling and shaping processor
    Filter6PButter butterworthFilter;  // Butterworth filter instance
    bool isSupersamplingEnabled = false;  // Enable supersampling is off by default

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
        json_object_set_new(rootJ, "symmetric", json_boolean(symmetric));
        json_object_set_new(rootJ, "isSupersamplingEnabled", json_boolean(isSupersamplingEnabled));
        return rootJ;
    }
    
    void dataFromJson(json_t* rootJ) override {
        json_t* divJ = json_object_get(rootJ, "symmetric");
        if (divJ)
            symmetric = json_boolean_value(divJ);

        json_t* isSupersamplingEnabledJ = json_object_get(rootJ, "isSupersamplingEnabled");
        if (isSupersamplingEnabledJ)
            isSupersamplingEnabled = json_boolean_value(isSupersamplingEnabledJ);
    }
    
    Clpy() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(GAIN_PARAM, 0.f, 10.f, 1.f, "Gain");
        configParam(CLIP_PARAM, -5.f, 5.f, 0.f, "Clip");
        configParam(GAIN_ATT_PARAM, -1.f, 1.f, 0.f, "Gain Att.");
        configParam(CLIP_ATT_PARAM, -1.f, 1.f, 0.f, "Clip Att.");
        configInput(GAIN_INPUT, "Gain CV");
        configInput(CLIP_L_INPUT, "Clip L CV");
        configInput(CLIP_R_INPUT, "Clip R CV");
        configInput(INL_INPUT, "In L");
        configInput(INR_INPUT, "In R");
        configOutput(OUTL_OUTPUT, "Out L");
        configOutput(OUTR_OUTPUT, "Out R");
    }

    void process(const ProcessArgs& args) override {
        int inChannels = std::max(inputs[INL_INPUT].getChannels(), inputs[INR_INPUT].getChannels());
        if (inChannels == 0) inChannels = 1;
    
        int gainChannels = inputs[GAIN_INPUT].getChannels();
        int clipLChannels = inputs[CLIP_L_INPUT].getChannels();
        int clipRChannels = inputs[CLIP_R_INPUT].getChannels();
    
        outputs[OUTL_OUTPUT].setChannels(inChannels);
        outputs[OUTR_OUTPUT].setChannels(inChannels);
    
        // Read attenuverters
        float gainAtt = params[GAIN_ATT_PARAM].getValue();  
        float clipAtt = params[CLIP_ATT_PARAM].getValue();  
    
        for (int c = 0; c < inChannels; c++) {
            // Audio inputs
            float inL = (inputs[INL_INPUT].getChannels() > c) ? inputs[INL_INPUT].getPolyVoltage(c) : 0.f;
            float inR = (inputs[INR_INPUT].getChannels() > c) ? inputs[INR_INPUT].getPolyVoltage(c) : inL;
    
            // Gain CV
            float gainCV = 0.f;
            if (gainChannels > c) gainCV = inputs[GAIN_INPUT].getPolyVoltage(c);
            else if (gainChannels == 1) gainCV = inputs[GAIN_INPUT].getPolyVoltage(0);

            // Apply attenuverter
            gainCV *= gainAtt;
    
            float gain = clamp(params[GAIN_PARAM].getValue() + gainCV, 0.f, 10.f);
    
            inL *= gain*0.5f;
            inR *= gain*0.5f;
    
            // Clip / asymptote L/R with auto-normalization
            float clipL = 0.0f;
            float clipR = 0.0f;
    
            if (clipLChannels > c) clipL = 0.2f * inputs[CLIP_L_INPUT].getPolyVoltage(c);
            else if (clipLChannels == 1) clipL = 0.2f * inputs[CLIP_L_INPUT].getPolyVoltage(0);
    
            if (clipRChannels > c) clipR = 0.2f * inputs[CLIP_R_INPUT].getPolyVoltage(c);
            else if (clipRChannels == 1) clipR = 0.2f * inputs[CLIP_R_INPUT].getPolyVoltage(0);
    
            // Apply attenuverter
            clipL *= clipAtt;
            clipR *= clipAtt;

            float clipParam = 0.2f * params[CLIP_PARAM].getValue();
            clipL += clipParam;
            clipR += clipParam;
    
            // Clamp final clip values
            clipL = clamp(clipL, -10.f, 10.f);
            clipR = clamp(clipR, -10.f, 10.f);
            
            clipL *= 0.56f;
            clipR *= 0.56f;
    
            // Auto-normalize
            if (clipLChannels == 0 && clipRChannels > 0) clipL = clipR;
            if (clipRChannels == 0 && clipLChannels > 0) clipR = clipL;
    
            // Apply waveshaper with optional supersampling
            float outL = shaperL[c].process(inL, clipL, symmetric, isSupersamplingEnabled);
            float outR = shaperR[c].process(inR, clipR, symmetric, isSupersamplingEnabled);
    
            // Output
            outputs[OUTL_OUTPUT].setVoltage(clamp(outL*1.77f, -10.f, 10.f), c);
            outputs[OUTR_OUTPUT].setVoltage(clamp(outR*1.77f, -10.f, 10.f), c);
        }
    }

};

struct ClpyWidget : ModuleWidget {
    ClpyWidget(Clpy* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Clpy.svg"),
            asset::plugin(pluginInstance, "res/Clpy-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        float gainPos = 87.f;
        addParam(createParamCentered<RoundBlackKnob>    (Vec( box.size.x/2.f, gainPos + 0  ), module, Clpy::GAIN_PARAM));
        addParam(createParamCentered<Trimpot>           (Vec( box.size.x/2.f, gainPos + 28 ), module, Clpy::GAIN_ATT_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>  (Vec( box.size.x/2.f, gainPos + 52 ), module, Clpy::GAIN_INPUT));

        float clipPos = 195.f;
        addParam(createParamCentered<RoundBlackKnob>    (Vec( box.size.x/2.f, clipPos + 0  ), module, Clpy::CLIP_PARAM));
        addParam(createParamCentered<Trimpot>           (Vec( box.size.x/2.f, clipPos + 28 ), module, Clpy::CLIP_ATT_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>  (Vec( box.size.x/2.f-12, clipPos + 52 ), module, Clpy::CLIP_L_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>  (Vec( box.size.x/2.f+12, clipPos + 52 ), module, Clpy::CLIP_R_INPUT));


        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(6.211, 12.002)), module, Clpy::INL_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(14.109, 12.002)), module, Clpy::INR_INPUT));

        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(6.211, 101.669)), module, Clpy::OUTL_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(14.109, 101.669)), module, Clpy::OUTR_OUTPUT));
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
    
        Clpy* clpyModule = dynamic_cast<Clpy*>(this->module);
        if (!clpyModule)
            return;
    
        menu->addChild(new MenuSeparator());
    
        struct SymmetricItem : MenuItem {
            Clpy* module;
            void onAction(const event::Action& e) override {
                module->symmetric = !module->symmetric;
            }
            void step() override {
                text = module->symmetric ? "Clipping Mode:   Convergent   ✔Symmetric"
                                         : "Clipping Mode:  ✔Convergent    Symmetric";
                MenuItem::step();
            }
        };
    
        auto* divItem = new SymmetricItem();
        divItem->module = clpyModule;
        menu->addChild(divItem);

        menu->addChild(new MenuSeparator());
    
        struct SupersampleItem : MenuItem {
            Clpy* module;
            void onAction(const event::Action& e) override {
                module->isSupersamplingEnabled = !module->isSupersamplingEnabled;
            }
            void step() override {
                text = "Supersampling";
                rightText = module->isSupersamplingEnabled ? "✔" : "";
                MenuItem::step();
            }
        };
    
        auto* supersampleItem = new SupersampleItem();
        supersampleItem->module = clpyModule;
        menu->addChild(supersampleItem);

    }
};
Model* modelClpy = createModel<Clpy, ClpyWidget>("Clpy");
