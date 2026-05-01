////////////////////////////////////////////////////////////
//
//   Steps
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   A window comparator with step interval sequencer
//
////////////////////////////////////////////////////////////


#include "plugin.hpp"

struct Steps : Module {

    enum ParamId {
        BIAS_PARAM,
        RANGE_PARAM,
        STEP_PARAM,
        BIAS_ATT,
        RANGE_ATT,
        STEP_ATT,
        COMPARATOR_ATT,
        TRIGGER_BUTTON_PARAM,
        RESET_BUTTON_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        COMPARATOR_INPUT,
        BIAS_INPUT,
        RANGE_INPUT,
        INVERT_INPUT,
        STEP_INPUT,
        TRIGGER_INPUT,
        RESET_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        COMPARATOR_UP_OUTPUT,
        COMPARATOR_DN_OUTPUT,
        STEPPER_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        UP_LIGHT,
        DOWN_LIGHT,
        SAMPLE_LIGHT,
        OUT1_LIGHT,
        OUT2_LIGHT,
        OUT3_LIGHT,
        OUT4_LIGHT,
        OUT5_LIGHT,
        OUT6_LIGHT,
        OUT7_LIGHT,
        OUT8_LIGHT,
        OUT9_LIGHT,
        OUT10_LIGHT,
        LIGHTS_LEN
    };

    // ── Context-menu options ──────────────────────────────────────────────────
    bool invertIsLatch  = false;  // false = momentary (default), true = latch toggle
    int  polyOverride   = 0;      // 0 = Auto, 1-16 = locked channel count
    int  quantDivisions = 0;      // divisions per volt; 0 = no quantization

    // ── Per-channel stepper state ─────────────────────────────────────────────
    static constexpr int MAX_POLY = 16;
    float step_mix[MAX_POLY] = {};

    // ── DSP helpers ───────────────────────────────────────────────────────────
    dsp::SchmittTrigger triggerIn[MAX_POLY];
    dsp::SchmittTrigger resetIn[MAX_POLY];
    dsp::BooleanTrigger triggerBtn;
    dsp::BooleanTrigger resetBtn;

    // Per-channel latch-invert state and previous edge
    bool invertLatch[MAX_POLY]    = {};
    bool prevInvertHigh[MAX_POLY] = {};

    Steps() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(BIAS_PARAM,           -5.0f, 5.0f,  1.0f,        "Bias");
        configParam(RANGE_PARAM,           0.0f, 10.0f, 3.0f,        "Range");
        configParam(STEP_PARAM,           -1.0f, 1.0f,  0.41666666f, "Step Size");
        configParam(BIAS_ATT,             -1.0f, 1.0f,  1.0f,        "Bias Att.");
        configParam(RANGE_ATT,            -1.0f, 1.0f,  1.0f,        "Range Att.");
        configParam(STEP_ATT,             -1.0f, 1.0f,  1.0f,        "Step Size Att.");
        configParam(COMPARATOR_ATT,       -1.0f, 1.0f,  1.0f,        "Comp. Input Att.");
        configParam(TRIGGER_BUTTON_PARAM,  0.0f, 1.0f,  0.0f,        "Trigger");
        configParam(RESET_BUTTON_PARAM,    0.0f, 1.0f,  0.0f,        "Reset");

#ifdef METAMODULE
        configInput(COMPARATOR_INPUT, "Comparator");
#else
        configInput(COMPARATOR_INPUT, "Comparator In (Input breaks normal)");
#endif
        configInput(BIAS_INPUT,    "Bias CV");
        configInput(RANGE_INPUT,   "Range CV");
        configInput(INVERT_INPUT,  "Invert Gate");
        configInput(STEP_INPUT,    "Step Size CV");
        configInput(TRIGGER_INPUT, "Trigger");
        configInput(RESET_INPUT,   "Reset");

        configOutput(COMPARATOR_UP_OUTPUT, "Comparator Above");
        configOutput(COMPARATOR_DN_OUTPUT, "Comparator Below");
        configOutput(STEPPER_OUTPUT,       "Stepper");

        // Initialise all accumulators to the bias default
        for (int c = 0; c < MAX_POLY; c++)
            step_mix[c] = params[BIAS_PARAM].getValue();
    }

    // ── Quantization helper ───────────────────────────────────────────────────
    // Snaps v to the nearest grid point at (1/quantDivisions) V intervals.
    float quantize(float v) const {
        if (quantDivisions <= 0) return v;
        float d = (float)quantDivisions;
        return std::round(v * d) / d;
    }

    // ── Save / restore ────────────────────────────────────────────────────────
    json_t* dataToJson() override {
        json_t* rootJ = json_object();

        json_t* smArr = json_array();
        for (int c = 0; c < MAX_POLY; c++)
            json_array_append_new(smArr, json_real(step_mix[c]));
        json_object_set_new(rootJ, "step_mix",       smArr);
        json_object_set_new(rootJ, "invertIsLatch",  json_boolean(invertIsLatch));
        json_object_set_new(rootJ, "polyOverride",   json_integer(polyOverride));
        json_object_set_new(rootJ, "quantDivisions", json_integer(quantDivisions));
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        json_t* smArr = json_object_get(rootJ, "step_mix");
        if (smArr && json_is_array(smArr)) {
            for (int c = 0; c < MAX_POLY && c < (int)json_array_size(smArr); c++)
                step_mix[c] = (float)json_number_value(json_array_get(smArr, c));
        }
        json_t* il = json_object_get(rootJ, "invertIsLatch");
        if (il) invertIsLatch = json_boolean_value(il);

        json_t* po = json_object_get(rootJ, "polyOverride");
        if (po) polyOverride = clamp((int)json_integer_value(po), 0, MAX_POLY);

        json_t* qd = json_object_get(rootJ, "quantDivisions");
        if (qd) quantDivisions = std::max(0, (int)json_integer_value(qd));
    }

    void process(const ProcessArgs& args) override {

        // ── Global (mono) parameters ──────────────────────────────────────────
        float bias = params[BIAS_PARAM].getValue()
                   + (inputs[BIAS_INPUT].isConnected()
                      ? inputs[BIAS_INPUT].getVoltage() * params[BIAS_ATT].getValue()
                      : 0.f);
        float range = params[RANGE_PARAM].getValue()
                    + (inputs[RANGE_INPUT].isConnected()
                       ? inputs[RANGE_INPUT].getVoltage() * params[RANGE_ATT].getValue()
                       : 0.f);
        float stepSize = params[STEP_PARAM].getValue()
                       + (inputs[STEP_INPUT].isConnected()
                          ? inputs[STEP_INPUT].getVoltage() * params[STEP_ATT].getValue()
                          : 0.f);

        bias     = clamp(bias, -10.f, 10.f);
        range    = clamp(std::abs(range), 0.f, 10.f);
        stepSize = clamp(stepSize, -5.f, 5.f);

        // ── Channel count ─────────────────────────────────────────────────────
        // polyOverride == 0 → auto from cables; else locked count.
        // When locked count exceeds cable channels, getPolyVoltage() naturally
        // telegraphs channel 0 downward — so a mono trigger fires all channels.
        int channels;
        if (polyOverride > 0) {
            channels = polyOverride;
        } else {
            channels = std::max({
                inputs[TRIGGER_INPUT].getChannels(),
                inputs[COMPARATOR_INPUT].getChannels(),
                1
            });
        }

        // ── Manual button edges (shared, fire all channels) ───────────────────
        bool btnTrig  = triggerBtn.process(params[TRIGGER_BUTTON_PARAM].getValue() > 0.f);
        bool btnReset = resetBtn.process(params[RESET_BUTTON_PARAM].getValue() > 0.f);

        // ── Output channel setup ──────────────────────────────────────────────
        outputs[STEPPER_OUTPUT].setChannels(channels);
        outputs[COMPARATOR_UP_OUTPUT].setChannels(channels);
        outputs[COMPARATOR_DN_OUTPUT].setChannels(channels);

        // ── Per-channel processing ────────────────────────────────────────────
        float displayComparatorOutput = 0.f;
        bool  externalComp = inputs[COMPARATOR_INPUT].isConnected();

        for (int c = 0; c < channels; c++) {

            // ── Invert ───────────────────────────────────────────────────────
            float invertV = inputs[INVERT_INPUT].isConnected()
                          ? inputs[INVERT_INPUT].getPolyVoltage(c)
                          : 0.f;

            bool invert;
            if (invertIsLatch) {
                bool high = invertV > 1.f;
                if (high && !prevInvertHigh[c])
                    invertLatch[c] = !invertLatch[c];
                prevInvertHigh[c] = high;
                invert = invertLatch[c];
            } else {
                invert = invertV > 0.f;
            }

            float effectiveStep = invert ? -stepSize : stepSize;

            // ── Comparator ───────────────────────────────────────────────────
            float compIn = externalComp
                         ? inputs[COMPARATOR_INPUT].getPolyVoltage(c)
                           * params[COMPARATOR_ATT].getValue()
                         : step_mix[c];

            float comparatorOutput = 0.f;
            float correction       = 0.f;

            if (externalComp) {
                if      (compIn > bias + 0.5f * range) comparatorOutput = -5.f;
                else if (compIn < bias - 0.5f * range) comparatorOutput =  5.f;
                // correction stays 0 — external input disconnects the normal
            } else {
                if (effectiveStep >= 0.f) {
                    if      (compIn >= bias + 0.5f * range) { comparatorOutput = -5.f; correction = -range; }
                    else if (compIn <  bias - 0.5f * range) { comparatorOutput =  5.f; correction =  range; }
                } else {
                    if      (compIn >  bias + 0.5f * range) { comparatorOutput = -5.f; correction = -range; }
                    else if (compIn <= bias - 0.5f * range) { comparatorOutput =  5.f; correction =  range; }
                }
            }

            // ── Trigger / Reset ───────────────────────────────────────────────
            bool doTrig  = triggerIn[c].process(
                               inputs[TRIGGER_INPUT].getPolyVoltage(c), 0.1f, 1.f)
                         || btnTrig;
            bool doReset = resetIn[c].process(
                               inputs[RESET_INPUT].getPolyVoltage(c), 0.1f, 1.f)
                         || btnReset;

            if (doReset) {
                step_mix[c] = externalComp ? 0.f : bias;
            } else if (doTrig) {
                step_mix[c] += (correction != 0.f) ? correction : effectiveStep;
                step_mix[c]  = clamp(step_mix[c], -10.f, 10.f);
            }

            // Quantize only at the output — step_mix stays as a true float
            // accumulator so sub-quantum steps continue to accumulate correctly.
            outputs[STEPPER_OUTPUT].setVoltage(quantize(step_mix[c]), c);
            outputs[COMPARATOR_UP_OUTPUT].setVoltage(comparatorOutput < 0.f ? 10.f : 0.f, c);
            outputs[COMPARATOR_DN_OUTPUT].setVoltage(comparatorOutput > 0.f ? 10.f : 0.f, c);

            if (c == 0) displayComparatorOutput = comparatorOutput;
        }

        // ── Lights (driven by channel 0) ──────────────────────────────────────
        bool didTrigOrReset = triggerIn[0].isHigh() || resetIn[0].isHigh()
                            || btnTrig || btnReset;
        lights[SAMPLE_LIGHT].setSmoothBrightness(didTrigOrReset ? 1.f : 0.f, args.sampleTime);

        if (displayComparatorOutput > 0.f) {
            lights[UP_LIGHT].setSmoothBrightness(0.f, args.sampleTime);
            lights[DOWN_LIGHT].setSmoothBrightness(1.f, args.sampleTime);
        } else if (displayComparatorOutput < 0.f) {
            lights[UP_LIGHT].setSmoothBrightness(1.f, args.sampleTime);
            lights[DOWN_LIGHT].setSmoothBrightness(0.f, args.sampleTime);
        } else {
            lights[UP_LIGHT].setSmoothBrightness(0.f, args.sampleTime);
            lights[DOWN_LIGHT].setSmoothBrightness(0.f, args.sampleTime);
        }

        // LED bargraph — step_mix[0] position within the current window
        int led_level = externalComp
            ? (int)std::floor(((step_mix[0] + 10.f) / 20.f) * 10.f)
            : (int)std::floor(((step_mix[0] - (bias - 0.5f * range)) / range) * 10.f);
        led_level = clamp(led_level, 0, 10);

        for (int i = 0; i < 10; ++i) {
            if (i < led_level) {
                lights[OUT1_LIGHT + i].setSmoothBrightness(1.f, args.sampleTime);
            } else {
                float b = lights[OUT1_LIGHT + i].getBrightness();
                b = (0.9f - 0.05f * i) * b;
                lights[OUT1_LIGHT + i].setSmoothBrightness(b, args.sampleTime);
            }
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  Widget
// ─────────────────────────────────────────────────────────────────────────────
struct StepsWidget : ModuleWidget {

    StepsWidget(Steps* module) {
        setModule(module);

        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Steps.svg"),
            asset::plugin(pluginInstance, "res/Steps-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(12.978, 49.183)), module, Steps::BIAS_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(37.219, 49.183)), module, Steps::RANGE_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(12.978, 78.965)), module, Steps::STEP_PARAM));

        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.3,    28.408)), module, Steps::COMPARATOR_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(19.545, 28.408)), module, Steps::BIAS_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(32.159, 28.408)), module, Steps::RANGE_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.3,    94.974)), module, Steps::INVERT_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.3,   112.263)), module, Steps::STEP_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(19.545,112.263)), module, Steps::TRIGGER_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(32.159,112.263)), module, Steps::RESET_INPUT));

        addParam(createParamCentered<Trimpot>(mm2px(Vec(19.545, 28.408-8)), module, Steps::BIAS_ATT));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(32.159, 28.408-8)), module, Steps::RANGE_ATT));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(7.3,   112.263-8)), module, Steps::STEP_ATT));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(7.3,    28.408-8)), module, Steps::COMPARATOR_ATT));

        addParam(createParamCentered<TL1105>(mm2px(Vec(19.545, 112.263-8)), module, Steps::TRIGGER_BUTTON_PARAM));
        addParam(createParamCentered<TL1105>(mm2px(Vec(32.159, 112.263-8)), module, Steps::RESET_BUTTON_PARAM));

        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(44.445, 19.632)), module, Steps::COMPARATOR_UP_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(44.426, 28.485)), module, Steps::COMPARATOR_DN_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(44.426,112.263)), module, Steps::STEPPER_OUTPUT));

        addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(39.075, 21.719)), module, Steps::UP_LIGHT));
        addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(39.19,  31.283)), module, Steps::DOWN_LIGHT));
        addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(45.726, 78.466)), module, Steps::SAMPLE_LIGHT));

        addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(40.923,106.773)), module, Steps::OUT1_LIGHT));
        addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(40.923,103.628)), module, Steps::OUT2_LIGHT));
        addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(40.923,100.483)), module, Steps::OUT3_LIGHT));
        addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(40.923, 97.338)), module, Steps::OUT4_LIGHT));
        addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(40.923, 94.192)), module, Steps::OUT5_LIGHT));
        addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(40.923, 91.047)), module, Steps::OUT6_LIGHT));
        addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(40.923, 87.902)), module, Steps::OUT7_LIGHT));
        addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(40.923, 84.757)), module, Steps::OUT8_LIGHT));
        addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(40.923, 81.612)), module, Steps::OUT9_LIGHT));
        addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(40.923, 78.466)), module, Steps::OUT10_LIGHT));
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
        Steps* m = dynamic_cast<Steps*>(module);
        if (!m) return;

        menu->addChild(new MenuSeparator);

        // ── Poly channel count slider ─────────────────────────────────────────
        struct PolyQuantity : Quantity {
            Steps* m;
            void  setValue(float v) override {
                m->polyOverride = clamp((int)std::round(v), 0, Steps::MAX_POLY);
            }
            float getValue() override            { return (float)m->polyOverride; }
            float getDefaultValue() override     { return 0.f; }
            float getMinValue() override         { return 0.f; }
            float getMaxValue() override         { return (float)Steps::MAX_POLY; }
            int   getDisplayPrecision() override { return 0; }
            std::string getLabel() override      { return "Poly Channels"; }
            std::string getDisplayValueString() override {
                return m->polyOverride == 0
                    ? "Auto"
                    : std::to_string(m->polyOverride);
            }
        };
        menu->addChild(createMenuLabel("Poly Channel Count"));
        auto* polySl     = new ui::Slider();
        auto* polyQ      = new PolyQuantity();
        polyQ->m         = m;
        polySl->quantity = polyQ;
        polySl->box.size.x = 200.f;
        menu->addChild(polySl);

        menu->addChild(new MenuSeparator);

        // ── Quantization submenu ──────────────────────────────────────────────
        struct QuantItem : MenuItem {
            Steps* m;
            int    divs;
            void onAction(const event::Action& e) override { m->quantDivisions = divs; }
            void step() override {
                rightText = (m->quantDivisions == divs) ? "✔" : "";
                MenuItem::step();
            }
        };
        struct QuantMenu : MenuItem {
            Steps* m;
            Menu* createChildMenu() override {
                Menu* sub = new Menu;
                const std::pair<const char*, int> entries[] = {
                    { "None (default)",             0  },
                    { "Octaves — 1-EDO  (1 V)",     1  },
                    { "9-EDO   (1/9 V)",             9  },
                    { "12-EDO  — Semitones",         12 },
                    { "17-EDO  (1/17 V)",            17 },
                    { "19-EDO  (1/19 V)",            19 },
                    { "22-EDO  (1/22 V)",            22 },
                    { "24-EDO  — Quarter-tones",     24 },
                    { "31-EDO  (1/31 V)",            31 },
                    { "41-EDO  (1/41 V)",            41 },
                    { "53-EDO  (1/53 V)",            53 },
                };
                for (auto& e : entries) {
                    auto* it = new QuantItem;
                    it->text = e.first;
                    it->m    = m;
                    it->divs = e.second;
                    sub->addChild(it);
                }
                return sub;
            }
            void step() override {
                // Show active scale name as right-text summary
                const std::pair<int, const char*> names[] = {
                    {  0, "None"  }, {  1, "Octaves" }, {  9, "9-EDO"  },
                    { 12, "12-EDO"}, { 17, "17-EDO"  }, { 19, "19-EDO" },
                    { 22, "22-EDO"}, { 24, "24-EDO"  }, { 31, "31-EDO" },
                    { 41, "41-EDO"}, { 53, "53-EDO"  },
                };
                rightText = ">";
                for (auto& n : names) {
                    if (m->quantDivisions == n.first) {
                        rightText = std::string(n.second) + " >";
                        break;
                    }
                }
                MenuItem::step();
            }
        };
        auto* qm = new QuantMenu;
        qm->text = "Quantization";
        qm->m    = m;
        menu->addChild(qm);

        menu->addChild(new MenuSeparator);

        // ── Invert input mode ─────────────────────────────────────────────────
        struct InvertModeItem : MenuItem {
            Steps* m;
            void onAction(const event::Action& e) override { m->invertIsLatch = !m->invertIsLatch; }
            void step() override {
                rightText = m->invertIsLatch ? "Latch ✔" : "Momentary";
                MenuItem::step();
            }
        };
        auto* im = new InvertModeItem;
        im->text = "Invert Input Mode";
        im->m    = m;
        menu->addChild(im);
    }
};

Model* modelSteps = createModel<Steps, StepsWidget>("Steps");
