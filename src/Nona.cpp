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

    bool mainVCA = false;

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
        json_object_set_new(rootJ, "mainVCA", json_boolean(mainVCA));
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        json_t* mainVCAJ = json_object_get(rootJ, "mainVCA");
        if (mainVCAJ) {
            mainVCA = json_is_true(mainVCAJ);
        }
    }

    Nona() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        configInput(INPUT_1, "Master VCA / Input 1");
        for (int i = 1; i < 9; i++) {
            configInput(INPUT_1 + i, "Input " + std::to_string(i + 1));
        }

        configOutput(OUTPUT_1, "Master VCA / Output 1");
        for (int i = 1; i < 9; i++) {
            configOutput(OUTPUT_1 + i, "Output " + std::to_string(i + 1));
        }

        configParam(GAIN_KNOB_1, -2.0f, 2.0f, 0.0f, "Gain 1", " x");
        for (int i = 1; i < 9; i++) {
            configParam(GAIN_KNOB_1 + i, -2.0f, 2.0f, 0.0f, "Gain " + std::to_string(i + 1), " x");
        }

        configParam(OFFSET_KNOB_1, -10.0f, 10.0f, 0.0f, "Offset 1", " V");
        for (int i = 1; i < 9; i++) {
            configParam(OFFSET_KNOB_1 + i, -10.0f, 10.0f, 0.0f, "Offset " + std::to_string(i + 1), " V");
        }
    }

    void process(const ProcessArgs &args) override {
        // Determine poly channel counts per input
        int numCh[9];
        for (int i = 0; i < 9; i++) {
            numCh[i] = inputs[INPUT_1 + i].isConnected()
                       ? std::max(1, inputs[INPUT_1 + i].getChannels())
                       : 1;
        }

        float gains[9], offsets[9];
        bool outputActive[9];
        for (int i = 0; i < 9; i++) {
            gains[i]        = params[GAIN_KNOB_1 + i].getValue();
            offsets[i]      = params[OFFSET_KNOB_1 + i].getValue();
            outputActive[i] = outputs[OUTPUT_1 + i].isConnected();
        }

        // Compute per-poly-channel master gain if mainVCA is active
        const int maxCh = 16;
        float masterGain[maxCh];
        for (int c = 0; c < maxCh; c++) masterGain[c] = 1.0f;

        if (mainVCA) {
            int mch = numCh[0];
            for (int c = 0; c < mch; c++) {
                float v = inputs[INPUT_1].isConnected()
                          ? inputs[INPUT_1].getVoltage(c)
                          : 0.0f;
                masterGain[c] = clamp(v * gains[0] + offsets[0], -20.f, 20.f)*0.1f;
            }
            // Output row 1 processed signal directly
            if (outputActive[0]) {
                outputs[OUTPUT_1].setChannels(mch);
                for (int c = 0; c < mch; c++) {
                    outputs[OUTPUT_1].setVoltage(masterGain[c]*10.f, c);
                }
            }
        }

        // Compute processed voltage for every row
        float processed[9][maxCh] = {};
        int rowCh[9] = {};
        int startRow = mainVCA ? 1 : 0;

        for (int i = startRow; i < 9; i++) {
            rowCh[i] = numCh[i];
            for (int c = 0; c < rowCh[i]; c++) {
                float v = inputs[INPUT_1 + i].isConnected()
                          ? inputs[INPUT_1 + i].getVoltage(c)
                          : 0.0f;
                processed[i][c] = v * gains[i] + offsets[i];
                if (mainVCA) {
                    int mc = std::min(c, numCh[0] - 1);
                    processed[i][c] *= masterGain[mc];
                }
            }
        }

        // Normalling: each output sums from its row back to the last active output above it
        for (int i = startRow; i < 9; i++) {
            int outCh = rowCh[i];
            for (int k = i - 1; k >= startRow; k--) {
                if (outputActive[k] && k != i) break;
                outCh = std::max(outCh, rowCh[k]);
            }
            outCh = std::max(outCh, 1);

            float outputMix[maxCh] = {};
            for (int k = i; k >= startRow; k--) {
                if (outputActive[k] && k != i) break;
                for (int c = 0; c < rowCh[k]; c++) {
                    outputMix[c] += processed[k][c];
                }
            }

            if (outputActive[i]) {
                outputs[OUTPUT_1 + i].setChannels(outCh);
                for (int c = 0; c < outCh; c++) {
                    outputs[OUTPUT_1 + i].setVoltage(clamp(outputMix[c], -10.f, 10.f), c);
                }
            } else {
                outputs[OUTPUT_1 + i].setChannels(1);
                outputs[OUTPUT_1 + i].setVoltage(0.f);
            }
        }

        // When mainVCA is off, handle row 0 the same as the rest
        if (!mainVCA) {
            int outCh = std::max(rowCh[0], 1);
            if (outputActive[0]) {
                outputs[OUTPUT_1].setChannels(outCh);
                for (int c = 0; c < outCh; c++) {
                    outputs[OUTPUT_1].setVoltage(clamp(processed[0][c], -10.f, 10.f), c);
                }
            } else {
                outputs[OUTPUT_1].setChannels(1);
                outputs[OUTPUT_1].setVoltage(0.f);
            }
        }
    }
};

// Draws a slightly "banana" curved rounded-rectangle highlight.
struct VCARowWidget : TransparentWidget {
    Nona* nonaModule;

    VCARowWidget(Nona* module, Vec pos, Vec size) : nonaModule(module) {
        box.pos  = pos;
        box.size = size;
    }

    // Build a rounded-rect with curved (sagging) top/bottom
    void buildPath(NVGcontext* vg, float inset, float r, float sag) {
        float w = box.size.x;
        float h = box.size.y;

        float left   = inset;
        float right  = w - inset;
        float top    = inset;
        float bottom = h - inset;

        float k = 0.55228475f; // circle approx

        nvgBeginPath(vg);

        // Start top-left corner
        nvgMoveTo(vg, left + r, top);

        // ---- TOP EDGE (curved down) ----
        float cx = w * 0.5f;
        nvgBezierTo(vg,
            cx - (cx - (left + r)) * 0.5f, top + sag,
            cx + (right - r - cx) * 0.5f, top + sag,
            right - r, top
        );

        // ---- TOP-RIGHT CORNER ----
        nvgBezierTo(vg,
            right - r + r * k, top,
            right, top + r - r * k,
            right, top + r
        );

        // ---- RIGHT SIDE ----
        nvgLineTo(vg, right, bottom - r);

        // ---- BOTTOM-RIGHT CORNER ----
        nvgBezierTo(vg,
            right, bottom - r + r * k,
            right - r + r * k, bottom,
            right - r, bottom
        );

        // ---- BOTTOM EDGE (curved down) ----
        nvgBezierTo(vg,
            cx + (right - r - cx) * 0.5f, bottom + sag,
            cx - (cx - (left + r)) * 0.5f, bottom + sag,
            left + r, bottom
        );

        // ---- BOTTOM-LEFT CORNER ----
        nvgBezierTo(vg,
            left + r - r * k, bottom,
            left, bottom - r + r * k,
            left, bottom - r
        );

        // ---- LEFT SIDE ----
        nvgLineTo(vg, left, top + r);

        // ---- TOP-LEFT CORNER ----
        nvgBezierTo(vg,
            left, top + r - r * k,
            left + r - r * k, top,
            left + r, top
        );

        nvgClosePath(vg);
    }

    void draw(const DrawArgs& args) override {
        if (!nonaModule || !nonaModule->mainVCA) return;

        const float r   = 15.f;
        const float sag = 4.0f; // small = subtle banana

        // Fill
        buildPath(args.vg, 1.f, r, sag);
        nvgFillColor(args.vg, nvgRGBA(33, 33, 33, 255));
        nvgFill(args.vg);

        // Outer glow
        buildPath(args.vg, 1.f, r + 3.f, sag);
        nvgStrokeColor(args.vg, nvgRGBA(208, 140, 89, 35));
        nvgStrokeWidth(args.vg, 8.0f);
        nvgStroke(args.vg);

        // Mid glow
        buildPath(args.vg, 2.f, r + 1.f, sag);
        nvgStrokeColor(args.vg, nvgRGBA(208, 140, 89, 80));
        nvgStrokeWidth(args.vg, 3.0f);
        nvgStroke(args.vg);

        // Inner stroke
        buildPath(args.vg, 3.f, r, sag);
        nvgStrokeColor(args.vg, nvgRGBA(208, 140, 89, 210));
        nvgStrokeWidth(args.vg, 1.5f);
        nvgStroke(args.vg);
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

        // VCA row highlight -- added first so it renders behind ports, knobs, and cables
        addChild(new VCARowWidget(module, Vec(4, 16.5f), Vec(box.size.x - 8, 37)));

        for (int i = 0; i < 9; i++) {
            addInput(createInputCentered<ThemedPJ301MPort>   (Vec( 22, 35 + i * 38 ), module, Nona::INPUT_1 + i));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec( 57, 40 + i * 38 ), module, Nona::GAIN_KNOB_1 + i));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec( 92, 40 + i * 38 ), module, Nona::OFFSET_KNOB_1 + i));
            addOutput(createOutputCentered<ThemedPJ301MPort> (Vec(127, 35 + i * 38 ), module, Nona::OUTPUT_1 + i));
        }
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);

        Nona* NonaModule = dynamic_cast<Nona*>(module);
        if (!NonaModule) return;

        menu->addChild(new MenuSeparator());

        struct MainVCAMenuItem : MenuItem {
            Nona* NonaModule;
            void onAction(const event::Action& e) override {
                NonaModule->mainVCA = !NonaModule->mainVCA;
            }
            void step() override {
                rightText = NonaModule->mainVCA ? "✔" : "";
                MenuItem::step();
            }
        };

        MainVCAMenuItem* mainVCAItem = new MainVCAMenuItem();
        mainVCAItem->text = "Row 1 as Master VCA";
        mainVCAItem->NonaModule = NonaModule;
        menu->addChild(mainVCAItem);
    }
};

Model* modelNona = createModel<Nona, NonaWidget>("Nona");