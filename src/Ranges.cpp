////////////////////////////////////////////////////////////
//
//   Ranges
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   Divides two input sequences into a range of voltages
//
////////////////////////////////////////////////////////////


#include "plugin.hpp"

struct Ranges : Module {
    enum ParamId {
        TOP_PARAM,
        BOTTOM_PARAM,
        TOP_ATT_PARAM,
        BOTTOM_ATT_PARAM,
        DIVISIONS_PARAM,
        NUM_PARAMS
    };
    enum InputId {
        TOP_INPUT,
        BOTTOM_INPUT,
        DIVISIONS_INPUT,
        NUM_INPUTS
    };
    enum OutputId {
        OUT1_OUTPUT, OUT2_OUTPUT, OUT3_OUTPUT,
        OUT4_OUTPUT, OUT5_OUTPUT, OUT6_OUTPUT,
        OUT7_OUTPUT, OUT8_OUTPUT, OUT9_OUTPUT,
        OUT10_OUTPUT, OUT11_OUTPUT, OUT12_OUTPUT,
        OUT13_OUTPUT,
        NUM_OUTPUTS
    };
    enum LightId {
        OUT1_LIGHT, OUT2_LIGHT, OUT3_LIGHT,
        OUT4_LIGHT, OUT5_LIGHT, OUT6_LIGHT,
        OUT7_LIGHT, OUT8_LIGHT, OUT9_LIGHT,
        OUT10_LIGHT, OUT11_LIGHT, OUT12_LIGHT,
        OUT13_LIGHT,
        NUM_LIGHTS
    };

    // Display state — written by process(), read by draw()
    float displayStart      = 0.f;
    float displayEnd        = 0.f;
    int   displayDivisions  = 1;
    int   displayChannels   = 1;

    Ranges() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
        configParam(TOP_PARAM,        -10.f, 10.f, 0.f, "Top");
        configParam(BOTTOM_PARAM,     -10.f, 10.f, 0.f, "Bottom");
        configParam(TOP_ATT_PARAM,     -1.f,  1.f, 0.f, "Top Att.");
        configParam(BOTTOM_ATT_PARAM,  -1.f,  1.f, 0.f, "Bottom Att.");
        configParam(DIVISIONS_PARAM,   0.f,  11.f, 1.f, "Divisions");

        configInput(TOP_INPUT,       "Top");
        configInput(BOTTOM_INPUT,    "Bottom");
        configInput(DIVISIONS_INPUT, "Divisions");

        for (int i = 0; i < 13; ++i)
            configLight(OUT1_LIGHT + i, "Output Active Indicator");

        for (int i = 0; i < 13; ++i)
            configOutput(OUT1_OUTPUT + i, "Range " + std::to_string(i + 1));
    }

    void process(const ProcessArgs& args) override {

        // ── Polyphony channel count ──────────────────────────────────────────
        // Drive channel count from the top/bottom CV inputs; fall back to 1.
        int channels = std::max({
            inputs[TOP_INPUT].getChannels(),
            inputs[BOTTOM_INPUT].getChannels(),
            1
        });

        // ── Per-output channel setup ─────────────────────────────────────────
        // We compute divisions from channel 0 of the divisions input (or the
        // knob alone) since all outputs share one division count.
        float divCV  = inputs[DIVISIONS_INPUT].getVoltage(); // ch 0 / mono
        int divisions = 1 + static_cast<int>(
            floor(params[DIVISIONS_PARAM].getValue() + 2.4f * divCV));
        divisions = clamp(divisions, 0, 12);

        // ── Process each output jack ─────────────────────────────────────────
        for (int i = 0; i < 13; ++i) {
            if (i < divisions + 1) {
                // Active output — set polyphonic voltages
                outputs[OUT1_OUTPUT + i].setChannels(channels);

                for (int c = 0; c < channels; ++c) {
                    float start = params[TOP_PARAM].getValue()
                                + params[TOP_ATT_PARAM].getValue()
                                * inputs[TOP_INPUT].getPolyVoltage(c);
                    float end   = params[BOTTOM_PARAM].getValue()
                                + params[BOTTOM_ATT_PARAM].getValue()
                                * inputs[BOTTOM_INPUT].getPolyVoltage(c);

                    start = clamp(start, -10.f, 10.f);
                    end   = clamp(end,   -10.f, 10.f);

                    float step    = divisions > 0 ? (end - start) / divisions : 0.f;
                    float voltage = start + step * i;
                    outputs[OUT1_OUTPUT + i].setVoltage(voltage, c);
                }
                lights[OUT1_LIGHT + i].setBrightness(1.f);
            } else {
                // Inactive output — silence all channels
                outputs[OUT1_OUTPUT + i].setChannels(1);
                outputs[OUT1_OUTPUT + i].setVoltage(0.f);
                lights[OUT1_LIGHT + i].setBrightness(0.f);
            }
        }

        // ── Update display state (channel 0 representative values) ──────────
        {
            float start = params[TOP_PARAM].getValue()
                        + params[TOP_ATT_PARAM].getValue()
                        * inputs[TOP_INPUT].getPolyVoltage(0);
            float end   = params[BOTTOM_PARAM].getValue()
                        + params[BOTTOM_ATT_PARAM].getValue()
                        * inputs[BOTTOM_INPUT].getPolyVoltage(0);
            displayStart     = clamp(start, -10.f, 10.f);
            displayEnd       = clamp(end,   -10.f, 10.f);
            displayDivisions = divisions;
            displayChannels  = channels;
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  RangesDisplay  — vertical voltage-range bar with gold tick marks
// ─────────────────────────────────────────────────────────────────────────────
struct RangesDisplay : TransparentWidget {
    Ranges* module = nullptr;

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) { TransparentWidget::drawLayer(args, layer); return; }

        const float w = box.size.x;
        const float h = box.size.y;

        // Values — use neutral defaults when no module (browser / preview)
        float start     = module ? module->displayStart     :  5.f;
        float end       = module ? module->displayEnd       : -5.f;
        int   divisions = module ? module->displayDivisions :  3;
        int   active    = divisions + 1;

        // ── Voltage range bar ────────────────────────────────────────────────
        // Map ±10 V to the full widget height; top = +10 V, bottom = −10 V
        const float barX  = 0.f;
        const float barW  = w;
        const float vMin  = -10.f, vRange = 20.f;

        auto vToY = [&](float v) -> float {
            return h - (v - vMin) / vRange * h;
        };

        float yTop = vToY(std::max(start, end));
        float yBot = vToY(std::min(start, end));
        float barH = std::max(yBot - yTop, 1.f);

        // Background track (dark grey)
        nvgBeginPath(args.vg);
        nvgRoundedRect(args.vg, barX, 0, barW, h, 2.f);
        nvgFillColor(args.vg, nvgRGBAf(0.1f, 0.1f, 0.1f, 1.0f));
        nvgFill(args.vg);

        // Active range fill (gold)
        nvgBeginPath(args.vg);
        nvgRoundedRect(args.vg, barX, yTop, barW, barH, 2.f);
        nvgFillColor(args.vg, nvgRGBAf(208.f/255.f, 140.f/255.f, 89.f/255.f, 0.75f));
        nvgFill(args.vg);

        // Division tick marks (bright gold lines across the bar)
        if (active > 1) {
            float vStep = (end - start) / divisions;
            for (int i = 0; i < active; ++i) {
                float ty = vToY(start + vStep * i);
                nvgBeginPath(args.vg);
                nvgMoveTo(args.vg, barX,        ty);
                nvgLineTo(args.vg, barX + barW, ty);
                nvgStrokeColor(args.vg, nvgRGB(0, 0, 0));
                nvgStrokeWidth(args.vg, 2.0f);
                nvgStroke(args.vg);
            }
        }

        TransparentWidget::drawLayer(args, layer);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  Widget
// ─────────────────────────────────────────────────────────────────────────────
struct RangesWidget : ModuleWidget {
    RangesWidget(Ranges* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Ranges.svg"),
            asset::plugin(pluginInstance, "res/Ranges-dark.svg")
        ));

        box.size = Vec(8 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT);

        // Screws
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // ── Left Section ─────────────────────────────────────────────────────
        addParam(createParam<RoundBlackKnob>(mm2px(Vec(5, 12)),  module, Ranges::TOP_PARAM));
        addParam(createParam<Trimpot>       (mm2px(Vec(7, 24)),  module, Ranges::TOP_ATT_PARAM));
        addInput(createInput<ThemedPJ301MPort>(mm2px(Vec(6, 32)),module, Ranges::TOP_INPUT));

        addParam(createParam<RoundBlackKnob>(mm2px(Vec(5, 52)),  module, Ranges::BOTTOM_PARAM));
        addParam(createParam<Trimpot>       (mm2px(Vec(7, 64)),  module, Ranges::BOTTOM_ATT_PARAM));
        addInput(createInput<ThemedPJ301MPort>(mm2px(Vec(6, 72)),module, Ranges::BOTTOM_INPUT));

        addParam(createParam<RoundBlackKnob>(mm2px(Vec(5,  97)), module, Ranges::DIVISIONS_PARAM));
        addInput(createInput<ThemedPJ301MPort>(mm2px(Vec(6, 109)),module, Ranges::DIVISIONS_INPUT));

        // ── Range display — slim gold bar on the right of the gap ───────────
        {
            auto* display = createWidget<RangesDisplay>(mm2px(Vec(20.0f, 16.f)));
            display->box.size = mm2px(Vec(2.f, 100.f));
            display->module   = module;
            addChild(display);
        }

        // ── Right Section — 13 outputs ────────────────────────────────────────
        for (int i = 0; i < 13; ++i) {
            float yPos = 13.f + i * 8.f;
            addChild(createLight<SmallLight<RedLight>>(
                mm2px(Vec(23, yPos + 3)), module, Ranges::OUT1_LIGHT + i));
            addOutput(createOutput<ThemedPJ301MPort>(
                mm2px(Vec(26, yPos)),    module, Ranges::OUT1_OUTPUT + i));
        }
    }
};

Model* modelRanges = createModel<Ranges, RangesWidget>("Ranges");
