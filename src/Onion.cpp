////////////////////////////////////////////////////////////
//
//   Onion
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   A polyphonic CV utility. Outputs ranges of sliders as layered CV.
//
////////////////////////////////////////////////////////////

#include "plugin.hpp"

const int ONION_LAYERS=18; //total weave patterns

struct Onion : Module {

    enum ParamIds {
        LAYERS_PARAM,
        DEPTH_PARAM,
        BIPOLAR_PARAM,
        LAYER_1_PARAM, LAYER_2_PARAM, LAYER_3_PARAM, LAYER_4_PARAM,
        LAYER_5_PARAM, LAYER_6_PARAM, LAYER_7_PARAM, LAYER_8_PARAM, 
        LAYER_9_PARAM, LAYER_10_PARAM, LAYER_11_PARAM, LAYER_12_PARAM,
        LAYER_13_PARAM, LAYER_14_PARAM, LAYER_15_PARAM, LAYER_16_PARAM,
        LAYER_17_PARAM, LAYER_18_PARAM,
        NUM_PARAMS
    };
    enum InputIds {
        DEPTH_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        LAYER_1_OUTPUT, LAYER_2_OUTPUT, LAYER_3_OUTPUT, LAYER_4_OUTPUT, 
        LAYER_5_OUTPUT, LAYER_6_OUTPUT, LAYER_7_OUTPUT, LAYER_8_OUTPUT,
        LAYER_9_OUTPUT, LAYER_10_OUTPUT, LAYER_11_OUTPUT, LAYER_12_OUTPUT,
        LAYER_13_OUTPUT, LAYER_14_OUTPUT, LAYER_15_OUTPUT, LAYER_16_OUTPUT,
        LAYER_17_OUTPUT, LAYER_18_OUTPUT,
        NUM_OUTPUTS
    };
    enum LightIds {
        LAYER_1_LIGHT, LAYER_2_LIGHT, LAYER_3_LIGHT, LAYER_4_LIGHT,
        LAYER_5_LIGHT, LAYER_6_LIGHT, LAYER_7_LIGHT, LAYER_8_LIGHT,
        LAYER_9_LIGHT, LAYER_10_LIGHT, LAYER_11_LIGHT, LAYER_12_LIGHT,
        LAYER_13_LIGHT, LAYER_14_LIGHT, LAYER_15_LIGHT, LAYER_16_LIGHT,
        LAYER_17_LIGHT, LAYER_18_LIGHT,
        NUM_LIGHTS
    };

    int outputLayers = 1;
    float layers[ONION_LAYERS] = {0.0f};
    float depth = 10.f;
    float modDepth = 10.f;
    float depthInput = 0.f;
    float polarity = 1.f;
    float out[ONION_LAYERS] = {0.0f};
    float prevPolarity = 1.f;
 
    json_t* dataToJson() override {
        json_t* rootJ = json_object();
    
        // Save prevPolarity
        json_object_set_new(rootJ, "prevPolarity", json_real(prevPolarity));
    
        return rootJ;
    }
    
    void dataFromJson(json_t* rootJ) override {
        if (!rootJ)
            return;
    
        // Load prevPolarity
        json_t* jp = json_object_get(rootJ, "prevPolarity");
        if (jp)
            prevPolarity = json_number_value(jp);
    }
    
    Onion() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    
        // Configure sliders and outputs using a loop
        for (int i = 0; i < ONION_LAYERS; i++) {
            std::string label = "Layer " + std::to_string(i + 1);
            configParam(LAYER_1_PARAM + i, -1.f, 1.f, 0.0f, label);
            std::string outLabel = "Layer " + std::to_string(i + 1) ;
            configOutput(LAYER_1_OUTPUT + i, outLabel);
        }

        configInput(DEPTH_INPUT, "CV Depth");
        configParam(LAYERS_PARAM, 1.f, 16.f, 1.f, "Onion Layers")->snapEnabled=true;
        configParam(DEPTH_PARAM, 0.0f, 10.f, 10.f, "CV Depth");
        configParam(BIPOLAR_PARAM, 0.f, 1.f, 1.f, "Bipolar");

        for (int layer = 0; layer < ONION_LAYERS; layer++) {
            paramQuantities[LAYER_1_PARAM + layer]->setDisplayValue(0.f);
        }        

    }

    void onReset(const ResetEvent& e) override {
        for (int layer = 0; layer < ONION_LAYERS; layer++) {
            paramQuantities[LAYER_1_PARAM + layer]->setDisplayValue(0.f);
        }        
    }
        
    void process(const ProcessArgs& args) override {
    
        depth = params[DEPTH_PARAM].getValue();
        depthInput = (inputs[DEPTH_INPUT].isConnected()) ? inputs[DEPTH_INPUT].getVoltage() : 0.f;
        modDepth = clamp(depth+depthInput, -10.f, 10.f); //range for modulation CV
        depth = clamp(modDepth, 0.f, 10.f); // depth of the slider
                                            // Note: In my testing VCV doesn't seem to care if the slider ranges 0-0
        
#ifdef METAMODULE
        outputLayers = 1;
#else
        outputLayers = (int)params[LAYERS_PARAM].getValue();
#endif
        
        polarity = params[BIPOLAR_PARAM].getValue();
                
        for (int layer=0; layer<ONION_LAYERS; layer++){
            layers[layer]=params[LAYER_1_PARAM + layer].getValue();
        }
                
        if (polarity != prevPolarity) { // move sliders to adjusted positions when polarity changes
            if (polarity < 0.5f) { // mono -> bipolar
                for (int layer = 0; layer < ONION_LAYERS; layer++) {
                    layers[layer] = layers[layer] * 2.f - 1.f;  // map 0..1 -> -1..1
                    paramQuantities[LAYER_1_PARAM + layer]->setValue(layers[layer]);
                }
            } else { // bipolar -> mono
                for (int layer = 0; layer < ONION_LAYERS; layer++) {
                    layers[layer] = (layers[layer] + 1.f) * 0.5f;  // map -1..1 -> 0..1
                    paramQuantities[LAYER_1_PARAM + layer]->setValue(layers[layer]);
                }
            }
            prevPolarity = polarity; // remember new state
        }

        for (int layer=0; layer<ONION_LAYERS; layer++){
            float value = layers[layer];
            if (polarity<0.5f){ value = (value+1.f)*0.5f; } //adjust for polarity
            value *= modDepth;
            out[layer] = value;
        }

        // Individual outputs per channel
        for (int i = 0; i < ONION_LAYERS; i++) {
            if (outputs[LAYER_1_OUTPUT + i].isConnected()) {
                outputs[LAYER_1_OUTPUT + i].setChannels(outputLayers);
                for (int ch = 0; ch < outputLayers; ch++) {
                    int idx = (i + ch) % ONION_LAYERS;
                    outputs[LAYER_1_OUTPUT + i].setVoltage(out[idx], ch);
                }
            } else {
                outputs[LAYER_1_OUTPUT + i].setChannels(0);
            }
        }

    }
};

struct OnionWidget : ModuleWidget {
    
    struct CVfunkSlider : app::SvgSlider {
        CVfunkSlider() {
            setBackgroundSvg(Svg::load(asset::plugin(pluginInstance, "res/components/ShortSlider.svg")));
            setHandleSvg(Svg::load(asset::plugin(pluginInstance, "res/components/ShortSliderHandle.svg")));
    
            // Update these to match new shorter SVG dimensions
            setHandlePosCentered(
                math::Vec(10.f, 55.f),  // bottom center
                math::Vec(10.f, 10.f)   // top center
            );
        }
    };

    template <typename TLightBase = YellowLight>
    struct FunkLightSlider : LightSlider<CVfunkSlider, VCVSliderLight<TLightBase>> {
        FunkLightSlider() {}
    };
    
    OnionWidget(Onion* module) {
        setModule(module);

        setPanel(createPanel(
                asset::plugin(pluginInstance, "res/Onion.svg"),
                asset::plugin(pluginInstance, "res/Onion-dark.svg")
            ));

        // Add screws or additional design elements as needed
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        addParam(createParamCentered<CKSS>(Vec(box.size.x/2.f - 50.f, 49.f), module, Onion::BIPOLAR_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(box.size.x/2.f-15, 45.f), module, Onion::LAYERS_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(box.size.x/2.f+25,45.f),module, Onion::DEPTH_INPUT));
        addParam(createParamCentered<RoundBlackKnob>(Vec(box.size.x/2.f + 50, 45.f), module, Onion::DEPTH_PARAM));

        float xStart = -5.f;
        float xPos = xStart;
        float yPos = 118.f;
        float spacing = 23.f;
        // Loop through each channel
        
        for (int i = 0; i < ONION_LAYERS; i++) {
            if (i==6) { yPos += 95.f; xPos=xStart;}
            if (i==12) { yPos += 95.f; xPos=xStart;}
            xPos +=  spacing;

            // Volume slider with light
            addParam(createLightParamCentered<FunkLightSlider<YellowLight>>(Vec(xPos, yPos), module, Onion::LAYER_1_PARAM + i, Onion::LAYER_1_LIGHT+ i));

            // VCA CV input
            addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(xPos, yPos + 40), module, Onion::LAYER_1_OUTPUT + i));
        
        }
    }

    void step() override {
        Onion* module = dynamic_cast<Onion*>(this->module);
        if (!module) return;

        float depth = module->depth;
        if (module->polarity <0.5f) depth *= 0.5f;

        for (int layer = 0; layer<ONION_LAYERS; layer++){
            float brightness = fabs(module->out[layer]/depth);
            module->lights[Onion::LAYER_1_LIGHT + layer].setBrightness(brightness*brightness);

            if (module->polarity < 0.5f){
                module->paramQuantities[Onion::LAYER_1_PARAM + layer]->displayOffset = depth; 
                module->paramQuantities[Onion::LAYER_1_PARAM + layer]->defaultValue = -1.0f; 
                               
            } else {
                module->paramQuantities[Onion::LAYER_1_PARAM + layer]->displayOffset = 0.0f;                   
                module->paramQuantities[Onion::LAYER_1_PARAM + layer]->defaultValue = 0.0f; 
            }
            
            module->paramQuantities[Onion::LAYER_1_PARAM + layer]->displayMultiplier = depth;
            
            // Dynamically update output labels with wrap-around
            for (int layer = 0; layer < ONION_LAYERS; layer++) {
                int startLayer = layer + 1;
                int endLayer = (layer + module->outputLayers - 1) % ONION_LAYERS + 1;
            
                std::string label;
                
            #ifdef METAMODULE
                //No polyphony on MM, so no need to rename the layer tooltips.
            #else
                if (module->outputLayers == 0 || startLayer == endLayer) {
                    // Single layer Ñ just show it
                    label = "Layer " + std::to_string(endLayer);
                } else if (endLayer > startLayer) {
                    // Normal range without wrap
                    label = "Layer " + std::to_string(startLayer) + "-" + std::to_string(endLayer);
                } else {
                    // Wrapped around
                    // First part: from startLayer to ONION_LAYERS
                    if (startLayer == ONION_LAYERS) {
                        label = "Layer " + std::to_string(ONION_LAYERS);
                    } else {
                        label = "Layer " + std::to_string(startLayer) + "-" + std::to_string(ONION_LAYERS);
                    }
            
                    // Second part: from 1 to endLayer
                    if (endLayer == 1) {
                        label += ", 1";
                    } else if (endLayer > 1) {
                        label += ", 1-" + std::to_string(endLayer);
                    }
                }
            
                module->configOutput(Onion::LAYER_1_OUTPUT + layer, label);
            #endif

            }

        } 
        ModuleWidget::step();                  
    }
};

Model* modelOnion = createModel<Onion, OnionWidget>("Onion");
