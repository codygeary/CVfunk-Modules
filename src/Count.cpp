////////////////////////////////////////////////////////////
//
//   Count
//
//   written by Cody Geary
//   Copyright 2025, MIT License
//
//   A counter with display
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
#include "digital_display.hpp"
using namespace rack;
#include <random>
#include <map>
#include <vector>

// Accepted chars (num only)
static const std::string VALID_CHARS = "1234567890";

// Hard limit: 14-digit maximum (all 9s)
static const long long MAX_LIMIT = 99999999999999LL; // 14 digits
static const long long MAX_COUNT_LIMIT = 99999999999999LL; // 14 digits of 9

struct Count : Module {
    enum ParamIds {
        UP_BUTTON, DOWN_BUTTON, RESET_BUTTON,
        LOOP_SWITCH, PHASE_SWITCH, RESET_POINT_SWITCH,
        NUM_PARAMS
    };
    enum InputIds {
        UP_INPUT, DOWN_INPUT, RESET_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        COUNT_OUTPUT,
        NUM_OUTS
    };
    enum LightIds {
        NUM_LIGHTS
    };

    std::string inputText = "1";
    std::string previnputText = "1";

    dsp::SchmittTrigger upTrigger, downTrigger, resetTrigger, upButtonTrigger, downButtonTrigger, resetButtonTrigger;

    long long maxCount = 16;
    bool phaseMode = false;
    int resetPoint = 0;
    long long currentNumber = 0;

    json_t* dataToJson() override {
        json_t* rootJ = json_object();

        json_object_set_new(rootJ, "maxCount", json_integer(maxCount));
        json_object_set_new(rootJ, "currentNumber", json_integer(currentNumber));

        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        json_t* maxCountJ = json_object_get(rootJ, "maxCount");
        if (maxCountJ) {
            maxCount = json_integer_value(maxCountJ);
            if (maxCount < 1) maxCount = 1;
            if (maxCount > MAX_LIMIT) maxCount = MAX_LIMIT;
            inputText = std::to_string(maxCount);
            previnputText = inputText;
        }

        json_t* currentNumberJ = json_object_get(rootJ, "currentNumber");
        if (currentNumberJ)
            currentNumber = json_integer_value(currentNumberJ);
    }

    Count() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTS, NUM_LIGHTS);

        configParam(UP_BUTTON, 0.f, 1.f, 0.f, "Up Button");
        configParam(DOWN_BUTTON, 0.f, 1.f, 0.f, "Down Button");
        configParam(RESET_BUTTON, 0.f, 1.f, 0.f, "Reset Button");

        configSwitch(LOOP_SWITCH, 0.0, 2.0, 2.0, "Loop Logic", {"Stop", "Infinite", "Loop"});
        configSwitch(PHASE_SWITCH, 0.0, 1.0, 1.0, "Gate / Phase Mode", {"Phase", "Gate"});
        configSwitch(RESET_POINT_SWITCH, 0.0, 2.0, 0.0, "Reset Point", {"0", "Center", "End"});

        configInput(UP_INPUT, "Up");
        configInput(DOWN_INPUT, "Down");
        configInput(RESET_INPUT, "Reset");

        configOutput(COUNT_OUTPUT, "Outputs Gate at Loop Point or Start/End, or Phase.");
    }

    void process(const ProcessArgs& args) override {

        
        // --- Parse maxCount safely ---
        if (inputText != previnputText) {
            if (inputText.empty()) {
                maxCount = 1; // fallback
            } else {
                try {
                    std::string trimmed = inputText;  
                    long long parsed = std::stoll(trimmed);
        
                    if (parsed < 1) parsed = 1;
                    if (parsed > MAX_COUNT_LIMIT) parsed = MAX_COUNT_LIMIT;
        
                    maxCount = parsed;
        
                    // keep text box in sync if clamped
                    std::string corrected = std::to_string(maxCount);
                    if (corrected != inputText) {
                        inputText = corrected;
                    }
                } catch (...) {
                    maxCount = 1;
                }
            }
            previnputText = inputText;
        }
  
        if (maxCount > MAX_COUNT_LIMIT) maxCount = MAX_COUNT_LIMIT;

        
        // --- Up events ---
        if (upTrigger.process(inputs[UP_INPUT].getVoltage()) ||
            upButtonTrigger.process(params[UP_BUTTON].getValue())) {
            currentNumber++;
            if (currentNumber > maxCount) {
                switch ((int)params[LOOP_SWITCH].getValue()) {
                    case 0: currentNumber = maxCount; break; // Stop
                    case 1: break;                         // Infinite
                    case 2: currentNumber = 1; break;      // Loop
                }
            }
        }

        // --- Down events ---
        if (downTrigger.process(inputs[DOWN_INPUT].getVoltage()) ||
            downButtonTrigger.process(params[DOWN_BUTTON].getValue())) {
            currentNumber--;
            if (currentNumber < 1) {
                switch ((int)params[LOOP_SWITCH].getValue()) {
                    case 0: currentNumber = 0; break;       // Stop
                    case 1: break;                          // Infinite
                    case 2: currentNumber = maxCount; break;// Loop
                }
            }
        }

        // --- Reset events ---
        if (resetTrigger.process(inputs[RESET_INPUT].getVoltage() - 0.1f) ||
            resetButtonTrigger.process(params[RESET_BUTTON].getValue())) {
            switch ((int)params[RESET_POINT_SWITCH].getValue()) {
                case 0: currentNumber = 0; break;                  // Reset to 0
                case 1: currentNumber = maxCount / 2; break;       // Reset to center
                case 2: currentNumber = maxCount; break;           // Reset to end
            }
        }

        // --- Output behavior ---
        float out = 0.f;
        int loopMode = (int)params[LOOP_SWITCH].getValue(); // 0=Stop,1=Infinite,2=Loop
        int phaseMode = (int)params[PHASE_SWITCH].getValue();

        if (phaseMode == 0) { // Phase
            if (maxCount > 0) {
                long long phaseNum = currentNumber;
                if (loopMode == 1) {
                    // wrap into range 0..maxCount
                    phaseNum = ((phaseNum % maxCount) + maxCount) % maxCount;
                }
                out = 10.f * (float)phaseNum / (float)maxCount;
            }
        } else { // Gate
            if (currentNumber == 0 || currentNumber == maxCount)
                out = 10.f;
        }

        outputs[COUNT_OUTPUT].setVoltage(out);
    }
};

struct inputTextField : rack::ui::TextField {
    Count* module = nullptr;
    bool settingText = false;  // guard against recursive text changes

    inputTextField(Count* mod) : module(mod) {
        multiline = false;
        placeholder = "Enter Max Count";
    }

    static std::string sanitizeSequence(const std::string& input) {
        std::string out;
        for (char c : input) {
            if (VALID_CHARS.find(c) != std::string::npos) {
                out += c;
            } else {
                out += "0";
            }
        }
        return out;
    }

    void updateText(const std::string& newText, int desiredCursor = -1, int desiredSelection = -1) {
        if (settingText) return;
        settingText = true;
    
        std::string safe = sanitizeSequence(newText);
        text = safe;
    
        // Clamp cursor and selection
        if (desiredCursor < 0) desiredCursor = (int)safe.size();
        if (desiredSelection < 0) desiredSelection = desiredCursor;
    
        cursor = std::min(desiredCursor, (int)safe.size());
        selection = std::min(desiredSelection, (int)safe.size());
    
        // Ensure cursor <= selection
        if (cursor > selection) cursor = selection;
    
        if (module)
            module->inputText = safe;
    
        settingText = false;
    }
    
    void processSanitize() {
        if (!module) return;
    
        int oldCursor = cursor;
        int oldSelection = selection;
        std::string oldText = text;
    
        std::string safe = sanitizeSequence(oldText);
    
        if (safe != oldText) {
            // Count valid characters up to old cursor and selection
            int newCursor = std::min(oldCursor, (int)safe.size());
            int newSelection = std::min(oldSelection, (int)safe.size());
    
            updateText(safe, newCursor, newSelection);
        } else {
            module->inputText = safe;
        }
    }


    void onSelectKey(const SelectKeyEvent& e) override {
        rack::ui::TextField::onSelectKey(e);
        processSanitize();
    }

    void onButton(const ButtonEvent& e) override {
        rack::ui::TextField::onButton(e);
        processSanitize();
    }
};


struct CountWidget : ModuleWidget {
    inputTextField* input = nullptr;
    DigitalDisplay* countDisplay = nullptr;

    CountWidget(Count* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Count.svg"),
            asset::plugin(pluginInstance, "res/Count-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        countDisplay = createDigitalDisplay(Vec(box.size.x / 2 - 25, 75), "0" );
        addChild(countDisplay);

        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(box.size.x/2,    170), module, Count::COUNT_OUTPUT));

        addParam(createParamCentered<TL1105>(Vec(box.size.x/2+43, 220), module, Count::UP_BUTTON));
        addParam(createParamCentered<TL1105>(Vec(box.size.x/2-43, 220), module, Count::DOWN_BUTTON));
        addParam(createParamCentered<TL1105>(Vec(box.size.x/2,    220), module, Count::RESET_BUTTON));

        addInput(createInputCentered<ThemedPJ301MPort>(Vec(box.size.x/2+43, 245), module, Count::UP_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(box.size.x/2-43, 245), module, Count::DOWN_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(box.size.x/2,    245), module, Count::RESET_INPUT));

        addParam(createParamCentered<CKSSThree>(Vec(box.size.x/2.f-43, 170), module, Count::LOOP_SWITCH));
        addParam(createParamCentered<CKSS>(Vec(box.size.x/2.f+43, 170), module, Count::PHASE_SWITCH));

        addParam(createParamCentered<CKSSThreeHorizontal>(Vec(box.size.x/2.f, 293), module, Count::RESET_POINT_SWITCH));

        // --- Max Count text entry widget ---
        input = new inputTextField(module);
        input->box.pos = Vec(box.size.x/2.0f-55, 325);
        input->box.size = (Vec(box.size.x-40.f, 20.f));
        if (module)
            input->text = module->inputText;
        addChild(input);

    }

    void draw(const DrawArgs& args) override {
        ModuleWidget::draw(args);
        Count* module = dynamic_cast<Count*>(this->module);
        if (!module) return;

          // --- keep inputTextField in sync with maxCount ---
        if (input && input->text != module->inputText) {
            input->updateText(module->inputText);
        }

        std::string numStr = std::to_string(module->currentNumber);
        int digits = (int)numStr.size();
        if (digits < 1) digits = 1;

        // Width of the display box 
        float availableWidth = countDisplay->box.size.x * 2.0f;

        // Estimate font size: scale so the digits fill the box width
        float fontSize = availableWidth / (digits * 0.6f);

        // Clamp so it doesnÕt get too huge or too small
        fontSize = clamp(fontSize, 8.f, 120.f);

        countDisplay->text = numStr;
        countDisplay->setFontSize(fontSize);
    }

    DigitalDisplay* createDigitalDisplay(Vec position, std::string initialValue) {
        DigitalDisplay* display = new DigitalDisplay();
        display->box.pos = position;
        display->box.size = Vec(50, 18);
        display->text = initialValue;
        display->fgColor = nvgRGB(208, 140, 89); // Gold color text
        display->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        display->setFontSize(120.f);
        return display;
    }
};
Model* modelCount = createModel<Count, CountWidget>("Count");