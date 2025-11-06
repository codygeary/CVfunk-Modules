////////////////////////////////////////////////////////////
//
//   Rat
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   A ratio CV generator with interesting non-linear knob scaling
//
////////////////////////////////////////////////////////////
#include "rack.hpp"
#include "plugin.hpp"
#include "digital_display.hpp" 

// fast integer power for small integer exponents (n >= 1)
inline float fastPowInt(float x, int n) {
    if (n <= 1) return x;
    float y = x;
    for (int i = 1; i < n; ++i) y *= x;
    return y;
}

// compute processed absolute ratio (the value multiplied to frequency)
//
// Inputs:
//   rawRatio : the user + CV ratio combined (can be negative for divisor case)
//   expo     : non-linearity parameter (1..5 typical)
// Returns: positive multiplier (e.g. 2.0 => +1 octave)
inline float computeProcessedAbsRatio(float rawRatio, float expo, bool CVlock) {
    // - treat rawRatio in (-inf..-1] and [1..inf) as direct multipliers
    // - rawRatio in (-1,1) -> maps to 1.0
    float ratio = 1.0f;
    if (rawRatio >= 1.0f) {
        ratio = rawRatio;
    } else if (rawRatio <= -1.0f) {
        ratio = rawRatio;
    } else {
        ratio = 1.0f;
    }

    float absRatio = std::fabs(ratio);

    if (CVlock) {
        // Snap directly to nearest half-integer (0.5 step)
        float snapped = std::round(absRatio * 2.0f) * 0.5f;
        return clamp(snapped, 0.5f, 16.0f);  // keep safe range
    }

    int baseRatio = (int)(absRatio * 2.0f);
    float remainder = 2.0f * absRatio - baseRatio; // in [0,1)

    // Use integer expo if it's near integer 1..8 for fast path, otherwise powf
    int expoInt = (int)std::lround(expo);
    bool useFastInt = (std::fabs(expo - expoInt) < 1e-6f) && (expoInt >= 1) && (expoInt <= 8);

    float mapped;
    if (remainder < 0.5f) {
        float x = remainder; // 0..0.5
        float v = useFastInt ? fastPowInt(x, expoInt) : powf(x, expo);
        mapped = baseRatio + v;
    } else {
        float x = 1.0f - remainder; // 0..0.5
        float v = useFastInt ? fastPowInt(x, expoInt) : powf(x, expo);
        mapped = (baseRatio + 1) - v;
    }

    // divide by 2 to return to original ratio scale and clamp
    float out = mapped * 0.5f;
    return clamp(out, 0.5f, 16.0f);
}


struct Rat : Module {
    enum ParamId {
        RATIO_PARAM,
        RATIO_ATT_PARAM,
        LOCK_BUTTON,
        PARAMS_LEN
    };
    enum InputId {
        OSCI_INPUT,
        RATIO_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        OSCII_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        LOCK_BUTTON_LIGHT,
        LIGHTS_LEN
    };

    dsp::SchmittTrigger lockButton;  
    bool CVlock = false;
        
    float ratioValue[16] = {1.0f};
    float dispRatio = 1.0f;
    float expo = 5.0f;
    
    // Serialization method to save module state
    json_t* dataToJson() override {
        json_t* rootJ = json_object();
    
        // Save expo
        json_object_set_new(rootJ, "expo", json_real(expo));
    
        return rootJ;
    }
    
    // Deserialization method to load module state
    void dataFromJson(json_t* rootJ) override {
     
        // Load floats
        json_t* expoJ = json_object_get(rootJ, "expo");
        if (expoJ) expo = json_number_value(expoJ);
    }    
    
    Rat() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(RATIO_PARAM, -16.0f, 16.0f, 1.0f, "Ratio");
        configParam(RATIO_ATT_PARAM, -1.0f, 1.0f, 1.0f, "Ratio Att.");
        configInput(OSCI_INPUT, "Osc I V/Oct In");
        configInput(RATIO_INPUT, "Ratio In");
        configOutput(OSCII_OUTPUT, "Osc II V/Oct Out");
        configParam(LOCK_BUTTON, 0.0, 1.0, 0.0, "Lock-on Ratio" );
     }

    void process(const ProcessArgs& args) override {

        if (lockButton.process(params[LOCK_BUTTON].getValue())){
            CVlock = !CVlock;
        }

        // Cache some cheap booleans and values
        const bool oscConnected = inputs[OSCI_INPUT].isConnected();
        const bool ratioConnected = inputs[RATIO_INPUT].isConnected();
    
        // Channel handling: follow original (channels based on OSCI input)
        int numChannels = std::max(inputs[OSCI_INPUT].getChannels(), 1);
        outputs[OSCII_OUTPUT].setChannels(numChannels);
    
        // Is ratio input monophonic? If so, we can compute non-linear mapping once
        const bool isRatioMonophonic = ratioConnected && (inputs[RATIO_INPUT].getChannels() == 1);
    
        // Base param (read once)
        float baseParamRatio = params[RATIO_PARAM].getValue();
        float ratioAtt = params[RATIO_ATT_PARAM].getValue();
        float expoLocal = expo; // the UI-controlled non-linearity
    
        // Prepare storage for per-channel processed absRatio and log2(absRatio)
        // We'll keep them in local arrays (small fixed size)
        static float processedAbs[16];
        static float log2Processed[16];
    
        // If ratio is monophonic (or disconnected), compute once and replicate for all channels
        if (!ratioConnected || isRatioMonophonic) {
            float rawRatio = baseParamRatio;
            if (ratioConnected) {
                float ratioMonoValue = inputs[RATIO_INPUT].getVoltage(0);
                rawRatio += ratioMonoValue * ratioAtt;
            }
    
            // Compute processed absolute multiplier
            float absR = 1.0f;

            absR = computeProcessedAbsRatio(rawRatio, expoLocal, CVlock);
    
            // Save display value on channel 0 
            dispRatio = std::abs(absR);
    
            // If original effective ratio < 1.0 we must invert:
            float ratioForInvert = 1.0f;
            if (rawRatio >= 1.0f) ratioForInvert = rawRatio;
            else if (rawRatio <= -1.0f) ratioForInvert = rawRatio;
            else ratioForInvert = 1.0f;
    
            bool needsInvert = (ratioForInvert < 1.0f);
    
            float finalAbs = needsInvert ? (1.0f / absR) : absR;
            finalAbs = clamp(finalAbs, 1.0f / 16.0f, 16.0f);
    
            // fill arrays and compute log2 once
            // Use dsp::approxLog2 if available; fallback to log2f
            for (int c = 0; c < numChannels; ++c) {
                processedAbs[c] = finalAbs;
                log2Processed[c] = log2f(finalAbs);
                ratioValue[c] = rawRatio;
            }            
        } else {
            // ratioConnected and poly (per-channel). Compute per-channel processedAbs and log2.
            for (int c = 0; c < numChannels; ++c) {
                float rawRatio = baseParamRatio;
                // ratio input poly: one value per channel
                rawRatio += inputs[RATIO_INPUT].getVoltage(c) * ratioAtt;
    
                float absR = 1.0f;
                
                absR = computeProcessedAbsRatio(rawRatio, expoLocal, CVlock);
      
                // preserve dispRatio as first channel's display
                if (c == 0) dispRatio = std::abs(absR);
    
                // Determine invert
                float ratioForInvert = 1.0f;
                if (rawRatio >= 1.0f) ratioForInvert = rawRatio;
                else if (rawRatio <= -1.0f) ratioForInvert = rawRatio;
                else ratioForInvert = 1.0f;
    
                bool needsInvert = (ratioForInvert < 1.0f);
                float finalAbs = needsInvert ? (1.0f / absR) : absR;
                finalAbs = clamp(finalAbs, 1.0f / 16.0f, 16.0f);
    
                processedAbs[c] = finalAbs;
                log2Processed[c] = log2f(finalAbs);
                ratioValue[c] = rawRatio;
            }
        }
    
        // Now the audio/CV conversion per channel:
        // outCV = inputVal + log2(absRatio)
        if (oscConnected) {
            // poly or mono OSCI; 
            for (int c = 0; c < numChannels; ++c) {
                float inputVal = inputs[OSCI_INPUT].getVoltage(c); // v/oct in
                float outCV = inputVal + log2Processed[c];
                outputs[OSCII_OUTPUT].setVoltage(outCV, c);
            }
        } else {
            // OSCI not connected: inputVal == 0 for all channels; but we still set channels
            for (int c = 0; c < numChannels; ++c) {
                float outCV = log2Processed[c];
                outputs[OSCII_OUTPUT].setVoltage(outCV, c);
            }
        }
    }
};
struct RatWidget : ModuleWidget {
    DigitalDisplay* ratioDisplay = nullptr;
    RatWidget(Rat* module) {
        setModule(module);
        
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Rat.svg"),
            asset::plugin(pluginInstance, "res/Rat-dark.svg")
        ));
        
        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        addInput(createInputCentered<ThemedPJ301MPort>(Vec( box.size.x/2.f, 65  ), module, Rat::OSCI_INPUT));

        float ratPos = 197.f;
        addParam(createParamCentered<RoundBlackKnob>    (Vec( box.size.x/2.f, ratPos + 0  ), module, Rat::RATIO_PARAM));
        addParam(createParamCentered<Trimpot>           (Vec( box.size.x/2.f, ratPos + 30 ), module, Rat::RATIO_ATT_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>  (Vec( box.size.x/2.f, ratPos + 55 ), module, Rat::RATIO_INPUT));

        ratioDisplay = createDigitalDisplay(Vec(box.size.x / 2 - 25, 107), "1.00:1");
        addChild(ratioDisplay);

        float lockPos = 139.f;
        addParam(createParamCentered<TL1105>(Vec( box.size.x/2.f, lockPos  ), module, Rat::LOCK_BUTTON));
        addChild(createLightCentered<SmallLight<RedLight>>(Vec( box.size.x/2.f, lockPos  ), module, Rat::LOCK_BUTTON_LIGHT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec( box.size.x/2.f, 317  ), module, Rat::OSCII_OUTPUT));

    }

    void step() override {
        Rat* module = dynamic_cast<Rat*>(this->module);
        if (!module) return;
    
        if (ratioDisplay) {      
            if (module->CVlock){         
                if (abs(module->dispRatio - roundf(module->dispRatio)) > 0.0f){   //if we have a non-round number
                    if (module->ratioValue[0] < 1.0f) {
                        float dispVal = int(2.0f*module->dispRatio);
                        ratioDisplay->text = string::f("2:%.0f", dispVal);
                    } else {
                        float dispVal = int(2.0f*module->dispRatio);
                        ratioDisplay->text = string::f("%.0f:2", dispVal);
                    }
                } else {
                    if (module->ratioValue[0] < 1.0f) {
                        ratioDisplay->text = string::f("1:%.0f", module->dispRatio);
                    } else {
                        ratioDisplay->text = string::f("%.0f:1", module->dispRatio);
                    }
                }
            } else {
                if (module->ratioValue[0] < 1.0f) {
                    ratioDisplay->text = string::f("1:%.3f", module->dispRatio);
                } else {
                    ratioDisplay->text = string::f("%.3f:1", module->dispRatio);
                }            
            }
        }
        
        module->lights[Rat::LOCK_BUTTON_LIGHT].setBrightness(module->CVlock ? 1.0f : 0.0f );
        ModuleWidget::step(); 
    }    

    // Generic Quantity for any float member 
    struct FloatMemberQuantity : Quantity {
        Rat* module;
        float Rat::*member; // pointer-to-member
        std::string label;
        float min, max, def;
        int precision;
    
        FloatMemberQuantity(Rat* m, float Rat::*mem, std::string lbl,
                            float mn, float mx, float df, int prec = 0)
            : module(m), member(mem), label(lbl), min(mn), max(mx), def(df), precision(prec) {}
    
        void setValue(float v) override { module->*member = clamp(v, min, max); }
        float getValue() override { return module->*member; }
        float getDefaultValue() override { return def; }
        float getMinValue() override { return min; }
        float getMaxValue() override { return max; }
        int getDisplayPrecision() override { return precision; }
    
        std::string getLabel() override { return label; }
        std::string getDisplayValueString() override {
            if (precision == 0)
                return std::to_string((int)std::round(getValue()));
            return string::f("%.*f", precision, getValue());
        }
    };

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
        Rat* m = dynamic_cast<Rat*>(module);
        assert(m);

        menu->addChild(new MenuSeparator());
        menu->addChild(createMenuLabel("Ratio Non-Linearity"));
    
        // Non-Linearity
        auto* spanSlider = new ui::Slider();
        spanSlider->quantity = new FloatMemberQuantity(m, &Rat::expo,
            "Non-Linearity", 1.f, 5.f, 5.f, 1);
        spanSlider->box.size.x = 200.f;
        menu->addChild(spanSlider);
    }


    DigitalDisplay* createDigitalDisplay(Vec position, std::string initialValue) {
        DigitalDisplay* display = new DigitalDisplay();
        display->box.pos = position;
        display->box.size = Vec(50, 18);
        display->text = initialValue;
        display->fgColor = nvgRGB(208, 140, 89); // Gold color text
        display->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        display->setFontSize(10.0f);
        return display;
    }   
};
Model* modelRat = createModel<Rat, RatWidget>("Rat");