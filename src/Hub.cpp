////////////////////////////////////////////////////////////
//
//   Hub
//
//   written by Cody Geary
//   Copyright 2026, MIT License
//
//   A dual knob macro-controller
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
#include "digital_display.hpp" 
using namespace rack;

const int CHANNELS=2;
const float YOFFSET = 171.f;

struct Hub : Module {
    enum ParamIds {
        MASTER_KNOB_I,
        VCA_GAIN_I,
        VCA_ATT_I,
        MAIN_GAIN_I,
        MAIN_OFFSET_I,

        MASTER_KNOB_II,
        VCA_GAIN_II,
        VCA_ATT_II,
        MAIN_GAIN_II,
        MAIN_OFFSET_II,

        NUM_PARAMS
    };
    enum InputIds {
        MAIN_INPUT_I,
        VCA_CV_INPUT_I,

        MAIN_INPUT_II,
        VCA_CV_INPUT_II,

        NUM_INPUTS
    };
    enum OutputIds {
        MAIN_OUTPUT_I,
        MAIN_OUTPUT_II,
        NUM_OUTPUTS
    };
   enum LightIds {
        HUB_I_1, HUB_I_2, HUB_I_3, HUB_I_4, HUB_I_5, HUB_I_6, HUB_I_7, HUB_I_8,
        HUB_I_9, HUB_I_10, HUB_I_11, HUB_I_12, HUB_I_13, HUB_I_14, HUB_I_15, HUB_I_16,
        
        HUB_II_1, HUB_II_2, HUB_II_3, HUB_II_4, HUB_II_5, HUB_II_6, HUB_II_7, HUB_II_8,
        HUB_II_9, HUB_II_10, HUB_II_11, HUB_II_12, HUB_II_13, HUB_II_14, HUB_II_15, HUB_II_16,

        HUB_IB_1, HUB_IB_2, HUB_IB_3, HUB_IB_4, HUB_IB_5, HUB_IB_6, HUB_IB_7, HUB_IB_8,
        HUB_IB_9, HUB_IB_10, HUB_IB_11, HUB_IB_12, HUB_IB_13, HUB_IB_14, HUB_IB_15, HUB_IB_16,
        
        HUB_IIB_1, HUB_IIB_2, HUB_IIB_3, HUB_IIB_4, HUB_IIB_5, HUB_IIB_6, HUB_IIB_7, HUB_IIB_8,
        HUB_IIB_9, HUB_IIB_10, HUB_IIB_11, HUB_IIB_12, HUB_IIB_13, HUB_IIB_14, HUB_IIB_15, HUB_IIB_16,
        NUM_LIGHTS
    };

    std::atomic<bool> isEditing[CHANNELS]; //For the master smart knob

    // Channel I state
    float inputValue_I = 0.f;
    float scaledValue_I = 0.f;
    float displayValue_I = 0.0f;
    int numChannelsI = 1;

    // Channel II state
    float inputValue_II = 0.f;
    float scaledValue_II = 0.f;
    float displayValue_II = 0.0f;
    int numChannelsII = 1;

    Hub() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        // Configure I parameters
        configParam(MASTER_KNOB_I, -10.0f, 10.0f, 0.0f, "Master Knob I");
        configParam(VCA_GAIN_I, 0.0f, 10.0f, 10.0f, "Output Range I");
        configParam(VCA_ATT_I, -1.0f, 1.0f, 1.0f, "Range Att. I");
        configParam(MAIN_GAIN_I, -2.0f, 2.0f, 1.0f, "Input I Att.");
        configParam(MAIN_OFFSET_I, -10.0f, 10.0f, 0.0f, "Input I Offset Bias");

        // Configure II parameters
        configParam(MASTER_KNOB_II, -10.0f, 10.0f, 0.0f, "Master Knob II");
        configParam(VCA_GAIN_II, 0.0f, 10.0f, 10.0f, "Output Range II");
        configParam(VCA_ATT_II, -1.0f, 1.0f, 1.0f, "Range Att. II");
        configParam(MAIN_GAIN_II, -2.0f, 2.0f, 1.0f, "Input II Att.");
        configParam(MAIN_OFFSET_II, -10.0f, 10.0f, 0.0f, "Input II Offset Bias");

        // Configure I inputs/outputs
        configInput(MAIN_INPUT_I, "In I");
        configInput(VCA_CV_INPUT_I, "Range CV I");
        configOutput(MAIN_OUTPUT_I, "Out I" );

        // Configure II inputs/outputs
        configInput(MAIN_INPUT_II, "In II");
        configInput(VCA_CV_INPUT_II, "Range CV II");
        configOutput(MAIN_OUTPUT_II, "Out II" );
      
        isEditing[0] = false;   
        isEditing[1] = false;   
    }

    void process(const ProcessArgs &args) override {
        // ---- Process I ----
        {
            int numChannelsI = std::max(inputs[VCA_CV_INPUT_I].getChannels(), inputs[MAIN_INPUT_I].getChannels());
            if (numChannelsI == 0) numChannelsI = 1;
            outputs[MAIN_OUTPUT_I].setChannels(numChannelsI);

            bool isRangeCV_Mono = inputs[VCA_CV_INPUT_I].isConnected() && (inputs[VCA_CV_INPUT_I].getChannels() == 1);
            float rangeCV_MonoValue = isRangeCV_Mono ? inputs[VCA_CV_INPUT_I].getVoltage(0) : 0.0f;

            for (int c = 0; c < numChannelsI; c++) {
                float rangeCV = inputs[VCA_CV_INPUT_I].isConnected() ? 
                                (isRangeCV_Mono ? rangeCV_MonoValue : inputs[VCA_CV_INPUT_I].getVoltage(c)) : 10.0f;
                float VCA_AMP = params[VCA_GAIN_I].getValue() * 0.1f * rangeCV * 0.1f * params[VCA_ATT_I].getValue();

                float topChannelVoltage = inputs[MAIN_INPUT_I].getVoltage(0);
                topChannelVoltage = topChannelVoltage*params[MAIN_GAIN_I].getValue() + params[MAIN_OFFSET_I].getValue();
                topChannelVoltage = topChannelVoltage * VCA_AMP;
                topChannelVoltage = clamp(topChannelVoltage, -10.f, 10.f);            

                if (inputs[MAIN_INPUT_I].isConnected() && c==0) {
                    if (!isEditing[0]){
                        params[MASTER_KNOB_I].setValue(topChannelVoltage);
                        inputValue_I = inputs[MAIN_INPUT_I].getVoltage(0);
                        scaledValue_I = inputValue_I*params[MAIN_GAIN_I].getValue() + params[MAIN_OFFSET_I].getValue();
                    } else {
                        scaledValue_I = params[MASTER_KNOB_I].getValue();                           
                    }
                } else {
                    scaledValue_I = params[MASTER_KNOB_I].getValue();           
                }
                
                displayValue_I = clamp(scaledValue_I*VCA_AMP,-10.f, 10.f);           

                float chanValue = 0.0f;
                if (inputs[MAIN_INPUT_I].isConnected()) {
                    if (!isEditing[0]){
                        chanValue = inputs[MAIN_INPUT_I].getVoltage(c);
                        chanValue = chanValue*params[MAIN_GAIN_I].getValue() + params[MAIN_OFFSET_I].getValue();
                        chanValue = clamp(chanValue*VCA_AMP, -10.f, 10.f);
                    } else {
                        chanValue = params[MASTER_KNOB_I].getValue();          
                        chanValue = chanValue*params[MAIN_GAIN_I].getValue() + params[MAIN_OFFSET_I].getValue();
                        chanValue = clamp(chanValue*VCA_AMP, -10.f, 10.f);                    
                    }
                } else {
                    chanValue = params[MASTER_KNOB_I].getValue()*VCA_AMP;
                }
                outputs[MAIN_OUTPUT_I].setVoltage(chanValue , c);
            }
        }

        // ---- Process II ----
        {
            numChannelsII = std::max(inputs[VCA_CV_INPUT_II].getChannels(), inputs[MAIN_INPUT_II].getChannels());
            if (numChannelsII== 0) numChannelsII = 1;
            outputs[MAIN_OUTPUT_II].setChannels(numChannelsII);

            bool isRangeCV_Mono = inputs[VCA_CV_INPUT_II].isConnected() && (inputs[VCA_CV_INPUT_II].getChannels() == 1);
            float rangeCV_MonoValue = isRangeCV_Mono ? inputs[VCA_CV_INPUT_II].getVoltage(0) : 0.0f;

            for (int c = 0; c < numChannelsII; c++) {
                float rangeCV = inputs[VCA_CV_INPUT_II].isConnected() ? 
                                (isRangeCV_Mono ? rangeCV_MonoValue : inputs[VCA_CV_INPUT_II].getVoltage(c)) : 10.0f;
                float VCA_AMP = params[VCA_GAIN_II].getValue() * 0.1f * rangeCV * 0.1f * params[VCA_ATT_II].getValue();

                float topChannelVoltage = inputs[MAIN_INPUT_II].getVoltage(0);
                topChannelVoltage = topChannelVoltage*params[MAIN_GAIN_II].getValue() + params[MAIN_OFFSET_II].getValue();
                topChannelVoltage = topChannelVoltage * VCA_AMP;
                topChannelVoltage = clamp(topChannelVoltage, -10.f, 10.f);            

                if (inputs[MAIN_INPUT_II].isConnected() && c==0) {
                    if (!isEditing[1]){
                        params[MASTER_KNOB_II].setValue(topChannelVoltage);
                        inputValue_II = inputs[MAIN_INPUT_II].getVoltage(0);
                        scaledValue_II = inputValue_II*params[MAIN_GAIN_II].getValue() + params[MAIN_OFFSET_II].getValue();
                    } else {
                        scaledValue_II = params[MASTER_KNOB_II].getValue();                           
                    }
                } else {
                    scaledValue_II = params[MASTER_KNOB_II].getValue();           
                }
                
                displayValue_II = clamp(scaledValue_II*VCA_AMP,-10.f, 10.f);           

                float chanValue = 0.0f;
                if (inputs[MAIN_INPUT_II].isConnected()) {
                    if (!isEditing[1]){
                        chanValue = inputs[MAIN_INPUT_II].getVoltage(c);
                        chanValue = chanValue*params[MAIN_GAIN_II].getValue() + params[MAIN_OFFSET_II].getValue();
                        chanValue = clamp(chanValue*VCA_AMP, -10.f, 10.f);
                    } else {
                        chanValue = params[MASTER_KNOB_II].getValue();          
                        chanValue = chanValue*params[MAIN_GAIN_II].getValue() + params[MAIN_OFFSET_II].getValue();
                        chanValue = clamp(chanValue*VCA_AMP, -10.f, 10.f);                    
                    }
                } else {
                    chanValue = params[MASTER_KNOB_II].getValue()*VCA_AMP;
                }
                outputs[MAIN_OUTPUT_II].setVoltage(chanValue , c);
            }
        }
    }
};

struct HubWidget : ModuleWidget {
    DigitalDisplay* voltDisplay_I = nullptr;
    DigitalDisplay* voltDisplay_II = nullptr;

    template <typename BaseKnob>
    struct SmartKnob : BaseKnob {
        void onDragStart(const event::DragStart& e) override {
            if (ParamQuantity* paramQuantity = this->getParamQuantity()) {
                if (Hub* module = dynamic_cast<Hub*>(paramQuantity->module)) {
                    int index = -1;
                    if (paramQuantity->paramId == Hub::MASTER_KNOB_I) index = 0;
                    else if (paramQuantity->paramId == Hub::MASTER_KNOB_II) index = 1;
                    if (index != -1) {
                        module->isEditing[index].store(true);
                    }
                }
            }
            BaseKnob::onDragStart(e);
        }
    
        void onDragEnd(const event::DragEnd& e) override {
            if (ParamQuantity* paramQuantity = this->getParamQuantity()) {
                if (Hub* module = dynamic_cast<Hub*>(paramQuantity->module)) {
                    int index = -1;
                    if (paramQuantity->paramId == Hub::MASTER_KNOB_I) index = 0;
                    else if (paramQuantity->paramId == Hub::MASTER_KNOB_II) index = 1;
                    if (index != -1) {
                        module->isEditing[index].store(false);
                    }
                }
            }
            BaseKnob::onDragEnd(e);
        }
    };
    
    using SmartRoundBlackKnob = SmartKnob<RoundBlackKnob>;
    using SmartTrimpot = SmartKnob<Trimpot>;
    using SmartRoundLargeBlackKnob = SmartKnob<RoundLargeBlackKnob>;
    using SmartRoundHugeBlackKnob = SmartKnob<RoundHugeBlackKnob>;

    HubWidget(Hub* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Hub.svg"),
            asset::plugin(pluginInstance, "res/Hub-dark.svg")
        ));

        // Screws
        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // ==== TOP (I) ====
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(box.size.x / 2 - 50, 70), module, Hub::MAIN_INPUT_I));
        addParam(createParamCentered<Trimpot>(Vec(box.size.x / 2 - 50, 45), module, Hub::MAIN_GAIN_I));
        addParam(createParamCentered<Trimpot>(Vec(box.size.x / 2 - 50, 95), module, Hub::MAIN_OFFSET_I));
        addParam(createParamCentered<SmartRoundHugeBlackKnob>(Vec(box.size.x / 2, 70), module, Hub::MASTER_KNOB_I));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(box.size.x / 2 + 30, 155), module, Hub::VCA_CV_INPUT_I));
        addParam(createParamCentered<Trimpot>(Vec(box.size.x / 2, 155), module, Hub::VCA_ATT_I));
        addParam(createParamCentered<RoundBlackKnob>(Vec(box.size.x / 2 - 30, 155), module, Hub::VCA_GAIN_I));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(box.size.x/2 + 55, 119), module, Hub::MAIN_OUTPUT_I));

        for (int i=0; i<8; i++){            
            float stepsize = 6;
            addChild(createLightCentered<SmallLight<YellowLight>>(Vec(box.size.x/2 + 53, 119-i*stepsize - 34), module, Hub::HUB_I_1+i*2));
            addChild(createLightCentered<SmallLight<YellowLight>>(Vec(box.size.x/2 + 53 + stepsize, 119-i*stepsize - 34 - stepsize/2), module, Hub::HUB_I_2+i*2));
            addChild(createLightCentered<TinyLight<WhiteLight>>(Vec(box.size.x/2 + 53, 119-i*stepsize - 34), module, Hub::HUB_IB_1+i*2));
            addChild(createLightCentered<TinyLight<WhiteLight>>(Vec(box.size.x/2 + 53 + stepsize, 119-i*stepsize - 34 - stepsize/2), module, Hub::HUB_IB_2+i*2));
        }

        voltDisplay_I = createDigitalDisplay(Vec(box.size.x / 2 - 25, 110), "0.000 V");
        addChild(voltDisplay_I);

        // ==== BOTTOM (II) ====
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(box.size.x / 2 - 50, 70 + YOFFSET), module, Hub::MAIN_INPUT_II));
        addParam(createParamCentered<Trimpot>(Vec(box.size.x / 2 - 50, 45 + YOFFSET), module, Hub::MAIN_GAIN_II));
        addParam(createParamCentered<Trimpot>(Vec(box.size.x / 2 - 50, 95 + YOFFSET), module, Hub::MAIN_OFFSET_II));
        addParam(createParamCentered<SmartRoundHugeBlackKnob>(Vec(box.size.x / 2, 70 + YOFFSET), module, Hub::MASTER_KNOB_II));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(box.size.x / 2 + 30, 155 + YOFFSET), module, Hub::VCA_CV_INPUT_II));
        addParam(createParamCentered<Trimpot>(Vec(box.size.x / 2, 155 + YOFFSET), module, Hub::VCA_ATT_II));
        addParam(createParamCentered<RoundBlackKnob>(Vec(box.size.x / 2 - 30, 155 + YOFFSET), module, Hub::VCA_GAIN_II));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(box.size.x/2 + 55, 119 + YOFFSET), module, Hub::MAIN_OUTPUT_II));

        for (int i=0; i<8; i++){            
            float stepsize = 6;
            addChild(createLightCentered<SmallLight<YellowLight>>(Vec(box.size.x/2 + 53, 119-i*stepsize - 34 + YOFFSET), module, Hub::HUB_II_1+i*2));
            addChild(createLightCentered<SmallLight<YellowLight>>(Vec(box.size.x/2 + 53 + stepsize, 119-i*stepsize - 34 - stepsize/2 + YOFFSET), module, Hub::HUB_II_2+i*2));
            addChild(createLightCentered<TinyLight<WhiteLight>>(Vec(box.size.x/2 + 53, 119-i*stepsize - 34 + YOFFSET), module, Hub::HUB_IIB_1+i*2));
            addChild(createLightCentered<TinyLight<WhiteLight>>(Vec(box.size.x/2 + 53 + stepsize, 119-i*stepsize - 34 - stepsize/2 + YOFFSET), module, Hub::HUB_IIB_2+i*2));
        }

        voltDisplay_II = createDigitalDisplay(Vec(box.size.x / 2 - 25, 110 + YOFFSET), "0.000 V");
        addChild(voltDisplay_II);
    }

    void step() override {
        Hub* module = dynamic_cast<Hub*>(this->module);
        if (!module) return;
    
        if (voltDisplay_I) {
            char voltText[16];
            snprintf(voltText, sizeof(voltText), "%.3f V", module->displayValue_I);             
            voltDisplay_I->text = voltText;
        }
        if (voltDisplay_II) {
            char voltText[16];
            snprintf(voltText, sizeof(voltText), "%.3f V", module->displayValue_II);             
            voltDisplay_II->text = voltText;
        }

        for (int i=0; i<16 ; i++){
            module->lights[Hub::HUB_I_1+i].setBrightness(0.f);
            module->lights[Hub::HUB_II_1+i].setBrightness(0.f);  
            module->lights[Hub::HUB_IB_1+i].setBrightness(0.f);
            module->lights[Hub::HUB_IIB_1+i].setBrightness(0.f);                    
        }       
               
        for (int i=0; i<module->outputs[Hub::MAIN_OUTPUT_I].getChannels(); i++){
            float val1 = module->outputs[Hub::MAIN_OUTPUT_I].getVoltage(i)*0.1f;
            if (val1>0){
                module->lights[Hub::HUB_I_1+i].setBrightness(val1); //for positive vals
            } else {
                module->lights[Hub::HUB_IB_1+i].setBrightness(-val1); //for negative vals
            }
        }

        for (int i=0; i<module->outputs[Hub::MAIN_OUTPUT_II].getChannels(); i++){
            float val2 = module->outputs[Hub::MAIN_OUTPUT_II].getVoltage(i)*0.1f;
            if (val2>0){
                module->lights[Hub::HUB_II_1+i].setBrightness(val2);
            } else {
                module->lights[Hub::HUB_IIB_1+i].setBrightness(-val2);
            }
        } 
        ModuleWidget::step();
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

Model* modelHub = createModel<Hub, HubWidget>("Hub");
