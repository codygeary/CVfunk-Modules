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
        LOOP_SWITCH, RESET_POINT_SWITCH,
        NUM_PARAMS
    };
    enum InputIds {
        UP_INPUT, DOWN_INPUT, RESET_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        COUNT_OUTPUT,
        PHASE_OUTPUT,
        NUM_OUTS
    };
    enum LightIds {
        NUM_LIGHTS
    };

    std::string inputText = "16";
    std::string previnputText = "16";

    dsp::SchmittTrigger upTrigger, downTrigger, resetTrigger, upButtonTrigger, downButtonTrigger, resetButtonTrigger;

    long long maxCount = 16;     // number of steps, >= 1
    bool phaseMode = false;
    bool prevPhaseMode = false;
    int resetPoint = 0;
    long long currentNumber = 1; // start at 1 by default (one-based)
    bool zeroBased = false;
    bool increasing = true; //keep track of direction

    json_t* dataToJson() override {
        json_t* rootJ = json_object();

        json_object_set_new(rootJ, "maxCount", json_integer(maxCount));
        json_object_set_new(rootJ, "currentNumber", json_integer(currentNumber));
        json_object_set_new(rootJ, "zeroBased", json_boolean(zeroBased));

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

        json_t* zeroBasedJ = json_object_get(rootJ, "zeroBased");
        if (zeroBasedJ) {
            zeroBased = json_is_true(zeroBasedJ);
        }

    }

    Count() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTS, NUM_LIGHTS);

        configParam(UP_BUTTON, 0.f, 1.f, 0.f, "Up Button");
        configParam(DOWN_BUTTON, 0.f, 1.f, 0.f, "Down Button");
        configParam(RESET_BUTTON, 0.f, 1.f, 0.f, "Reset Button");

        configSwitch(LOOP_SWITCH, 0.0, 2.0, 2.0, "Loop Logic", {"Stop", "Unbounded", "Loop"});
        configSwitch(RESET_POINT_SWITCH, 0.0, 2.0, 0.0, "Reset Point", {"0", "Center", "End"});

        configInput(UP_INPUT, "Up");
        configInput(DOWN_INPUT, "Down");
        configInput(RESET_INPUT, "Reset");

        configOutput(COUNT_OUTPUT, "High Gate at Loop Point or upon reaching Start/End");
        configOutput(PHASE_OUTPUT, "Stepped-Phase 0-10V");

    }

    void process(const ProcessArgs& args) override {

        // --- Parse maxCount safely when text changed ---
        if (inputText != previnputText) {
            if (inputText.empty()) {
                maxCount = 1; // fallback
            } else {
#if defined(METAMODULE)
                // Lightweight version for MetaModule
                long long parsed = atoll(inputText.data());
#else
                // Safer, validated version (no exceptions, checks input validity)
                char* end = nullptr;
                long long parsed = std::strtoll(inputText.c_str(), &end, 10);
                if (end == inputText.c_str() || *end != '\0') {
                    // Invalid input — reset safely
                    parsed = 1;
                }
#endif
                // Clamp to valid range
                if (parsed < 1) parsed = 1;
                if (parsed > MAX_COUNT_LIMIT) parsed = MAX_COUNT_LIMIT;
        
                maxCount = parsed;
        
                // Keep text box in sync if clamped
                std::string corrected = std::to_string(maxCount);
                if (corrected != inputText) {
                    inputText = corrected;
                }
            }
            previnputText = inputText;
        }

        if (maxCount > MAX_COUNT_LIMIT) maxCount = MAX_COUNT_LIMIT;

        // Precompute useful bounds from zeroBased and maxCount
        // stepCount = number of steps (>=1)
        long long stepCount = maxCount;
        long long lowerBound = zeroBased ? 0LL : 1LL;
        long long upperBound = zeroBased ? (stepCount - 1LL) : stepCount;

        // --- Up events ---
        if (upTrigger.process(inputs[UP_INPUT].getVoltage()) ||
            upButtonTrigger.process(params[UP_BUTTON].getValue())) {
            currentNumber++;
            long long ub = upperBound;
            if (currentNumber > ub) {
                switch ((int)params[LOOP_SWITCH].getValue()) {
                    case 0: // Stop: clamp to upperBound
                        currentNumber = ub;
                        break;
                    case 1: // Infinite: do nothing (allow grow)
                        break;
                    case 2: // Loop: wrap to lowerBound
                        currentNumber = lowerBound;
                        break;
                }
            }
            increasing = true;
        }

        // --- Down events ---
        if (downTrigger.process(inputs[DOWN_INPUT].getVoltage()) ||
            downButtonTrigger.process(params[DOWN_BUTTON].getValue())) {
            currentNumber--;
            long long lb = lowerBound;
            if (currentNumber < lb) {
                switch ((int)params[LOOP_SWITCH].getValue()) {
                    case 0: // Stop: clamp to lowerBound
                        currentNumber = lb;
                        break;
                    case 1: // Infinite: do nothing
                        break;
                    case 2: // Loop: wrap to upperBound
                        currentNumber = upperBound;
                        break;
                }
            }
            increasing = false;
        }

        // --- Reset events ---
        if (resetTrigger.process(inputs[RESET_INPUT].getVoltage() - 0.1f) ||
            resetButtonTrigger.process(params[RESET_BUTTON].getValue())) {
            switch ((int)params[RESET_POINT_SWITCH].getValue()) {
                case 0: // Reset to lowerBound (0 or 1)
                    currentNumber = lowerBound;
                    break;
                case 1: { // Reset to center: midpoint between lowerBound and upperBound
                    long long mid = (lowerBound + upperBound) / 2LL;
                    currentNumber = mid;
                } break;
                case 2: // Reset to end (upper bound)
                    currentNumber = upperBound;
                    break;
            }
        }

        // --- Output behavior ---
        float out = 0.f;
        float phase = 0.f;
        int loopMode = (int)params[LOOP_SWITCH].getValue(); // 0=Stop,1=Infinite,2=Loop

        if (stepCount > 0) {
            // For wrapping in phase mode, modulo by stepCount (the number of steps)
            long long phaseNum = currentNumber;
            if (loopMode == 1) {
                phaseNum = ((phaseNum % stepCount) + stepCount) % stepCount;
            }

            // Choose divisor so the last step maps to 10V:
            // - one-based: divisor = stepCount (1..stepCount -> 10*stepCount/stepCount = 10V)
            // - zero-based: divisor = max(1, stepCount - 1) so 0..(stepCount-1) maps and top -> 10V
            long long divisor = zeroBased ? std::max(1LL, stepCount - 1LL) : stepCount;

            // Convert to voltage
            phase = 10.f * (float)phaseNum / (float)divisor;
        }

        if (loopMode > 0){ //not stop mode
            if (zeroBased) {
                if (currentNumber == 0) out = 10.f;
            } else {
                if (currentNumber == maxCount) out = 10.f;
            }
        } else { //stop mode
            int baseLine = zeroBased ? 0 : 1;
            int topLine = zeroBased ? maxCount-1 : maxCount;
            if (!increasing && currentNumber == baseLine) out = 10.f;
            if (increasing && currentNumber == topLine) out = 10.f;           
        }

        outputs[COUNT_OUTPUT].setVoltage(out);
        outputs[PHASE_OUTPUT].setVoltage(phase);

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
                // non-digit -> replace with '0' to keep caret positions easier for the user
                out += '0';
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

        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(box.size.x/2 +43, 170), module, Count::PHASE_OUTPUT));

        addParam(createParamCentered<CKSSThreeHorizontal>(Vec(box.size.x/2.f, 293), module, Count::RESET_POINT_SWITCH));

        // --- Max Count text entry widget ---
        input = new inputTextField(module);
        input->box.pos = Vec(box.size.x/2.0f-55, 325);
        input->box.size = (Vec(box.size.x-40.f, 20.f));
        if (module)
            input->text = module->inputText;
        addChild(input);

    }

#if defined(METAMODULE)
    // For MM, use step(), because overriding draw() will allocate a module-sized pixel buffer
    void step() override {
#else
    void draw(const DrawArgs& args) override {
        ModuleWidget::draw(args);
#endif
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

        // Clamp so it doesn’t get too huge or too small
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

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);

        // Use a different name to avoid shadowing
        Count* countModule = dynamic_cast<Count*>(this->module);
        if (!countModule) return;

        // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());

        // Zero-based mode menu item (toggle)
        struct ZeroBasedItem : MenuItem {
            Count* module;
            void onAction(const event::Action& e) override {
                module->zeroBased = !module->zeroBased;
            }
            void step() override {
                rightText = module->zeroBased ? "✔" : "";
                MenuItem::step();
            }
        };

        ZeroBasedItem* zeroItem = new ZeroBasedItem();
        zeroItem->text = "Zero-based counting (start at 0)";
        zeroItem->module = countModule;
        menu->addChild(zeroItem);
    }

};
Model* modelCount = createModel<Count, CountWidget>("Count");
