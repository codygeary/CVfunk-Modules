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
    
        // Save the state of mainVCA
        json_object_set_new(rootJ, "mainVCA", json_boolean(mainVCA));
    
        return rootJ;
    }
    
    void dataFromJson(json_t* rootJ) override {
    
        // Load the state of mainVCA
        json_t* mainVCAJ = json_object_get(rootJ, "mainVCA");
        if (mainVCAJ) {
            mainVCA = json_is_true(mainVCAJ);
        }                
    }

    Nona() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        for (int i = 0; i < 9; i++) {
            configInput(INPUT_1 + i, "In " + std::to_string(i + 1));
        }
        for (int i = 0; i < 9; i++) {
            configOutput(OUTPUT_1 + i, "Out " + std::to_string(i + 1));
        }
        for (int i = 0; i < 9; i++) {
            configParam(GAIN_KNOB_1 + i, -2.0f, 2.0f, 0.0f, "Gain " + std::to_string(i + 1));
        }
        for (int i = 0; i < 9; i++) {
            configParam(OFFSET_KNOB_1 + i, -5.0f, 5.0f, 0.0f,  "Offset " + std::to_string(i + 1));
        }
    }

    void process(const ProcessArgs &args) override {
        // Initialize arrays to store the input voltages, gain, and offset values
        float inputVoltages[9] = {0.f};
        float gains[9] = {0.f};
        float offsets[9] = {0.f};
        float outputVoltages[9] = {0.f};
        bool outputActive[9] = {false};
    
        // Read all inputs, gains, and offsets first
        for (int i = 0; i < 9; i++) {
            if (inputs[INPUT_1 + i].isConnected()) {
                inputVoltages[i] = inputs[INPUT_1 + i].getVoltage();
            }
            if (outputs[OUTPUT_1 + i].isConnected()) {
                outputActive[i] = true;
            }
            gains[i] = params[GAIN_KNOB_1 + i].getValue();
            offsets[i] = params[OFFSET_KNOB_1 + i].getValue();
        }
    
        // Read the master gain control if mainVCA is active
        float masterGain = 1.0f;
        if (mainVCA) {
            // Compute the master gain similar to the top channel
            if (inputs[INPUT_1].isConnected()) {
                masterGain = inputVoltages[0] * gains[0] + offsets[0];
            } else {
                masterGain = offsets[0]; // Just use offset if no input is present
            }
            masterGain = clamp(masterGain, -10.f, 10.f); // Ensure the master gain stays within a reasonable range
        }
    
        // Separate processing for the top channel (index 0)
        float topChannelOutput = 0.0f;
        if (mainVCA) {
            // Process the top channel separately
            topChannelOutput = inputVoltages[0] * gains[0] + offsets[0];
            topChannelOutput = clamp(topChannelOutput, -10.f, 10.f);
    
            // Output the top channel value directly to its own output
            if (outputs[OUTPUT_1].isConnected()) {
                outputs[OUTPUT_1].setVoltage(topChannelOutput);
            }
        }
    
        // Initialize mix for the remaining channels
        float mixOutput = 0.f;
    
        // Process each stage: Calculate the raw output voltage for each stage
        for (int i = 1; i < 9; i++) {  // Start from channel 1
            outputVoltages[i] = inputVoltages[i] * gains[i] + offsets[i];
        }
    
        // Apply master gain to all stages if mainVCA is active
        if (mainVCA) {
            for (int i = 1; i < 9; i++) {
                outputVoltages[i] *= masterGain;
            }
        }
    
        // Process each output with normalling
        for (int i = 1; i < 9; i++) {  // Start from channel 1
            float outputMix = 0.f;
            for (int k = i; k >= 1; k--) {  // Adjust for normalisation
                if (outputActive[k] && k != i) {
                    break;
                }
                outputMix += outputVoltages[k];
            }
            outputMix = clamp(outputMix, -10.f, 10.f);
            if (outputs[OUTPUT_1 + i].isConnected()) {
                outputs[OUTPUT_1 + i].setVoltage(outputMix);
            } else {
                outputs[OUTPUT_1 + i].setVoltage(0.f);
            }
        }
    
        // Output the master VCA's scaled signal (if active)
        if (mainVCA && outputs[OUTPUT_2].isConnected()) {  // Assuming OUTPUT_2 is where the master VCA output should go
            outputs[OUTPUT_2].setVoltage(topChannelOutput * masterGain);
        }
    }
};

struct SplineWidget : TransparentWidget {
    Nona* NonaModule;

    SplineWidget() {}

    void draw(const DrawArgs &args) override {
        if (!NonaModule || !NonaModule->mainVCA) {
            return;  // Do not draw the spline if mainVCA is false
        }

        // Define the three points for the arc
        Vec p1 = Vec(0, 52);
        Vec p2 = Vec(box.size.x / 2, 66);
        Vec p3 = Vec(box.size.x, 52);

        // Calculate control points for a cubic Bezier approximation of a circular arc
        float k = 4.0 * (sqrt(2) - 1.0) / 3.0;  // Coefficient to approximate circular arc

        Vec c1 = Vec(p1.x + k * (p2.x - p1.x), p1.y + k * (p2.y - p1.y));
        Vec c2 = Vec(p3.x - k * (p3.x - p2.x), p3.y - k * (p3.y - p2.y));

        // Draw the black line segment under the grey curve
        nvgBeginPath(args.vg);
        
        nvgMoveTo(args.vg, box.size.x - 38, 53.5);
        nvgLineTo(args.vg, box.size.x-7.5, 53.5);

        nvgStrokeColor(args.vg, nvgRGBA(45, 45, 45, 255));  // Set color of the line segment
        nvgStrokeWidth(args.vg, 15);  // Set thickness of the line segment
        nvgStroke(args.vg);

        // Draw the cubic Bezier curve
        nvgBeginPath(args.vg);
        nvgMoveTo(args.vg, p1.x, p1.y);  // Start at the first point
        nvgBezierTo(args.vg, c1.x, c1.y, c2.x, c2.y, p3.x, p3.y);  // Cubic Bezier curve

        nvgStrokeColor(args.vg, nvgRGBA(100, 100, 100, 255));  // Set color of the arc
        nvgStrokeWidth(args.vg, 6.0);  // Set thickness of the arc
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

        for (int i = 0; i < 9; i++) {
            addInput(createInputCentered<ThemedPJ301MPort>           (Vec( 22, 35 + i * 38 ), module, Nona::INPUT_1 + i));
            addParam(createParamCentered<RoundSmallBlackKnob>             (Vec( 57, 40 + i * 38 ), module, Nona::GAIN_KNOB_1 + i));
            addParam(createParamCentered<RoundSmallBlackKnob>             (Vec( 92, 40 + i * 38 ), module, Nona::OFFSET_KNOB_1 + i));
            addOutput(createOutputCentered<ThemedPJ301MPort>         (Vec( 127, 35 + i * 38 ), module, Nona::OUTPUT_1 + i));
        }
        
         // Add the SplineWidget
        SplineWidget* splineWidget = new SplineWidget();
        splineWidget->NonaModule = module;
        splineWidget->box.pos = Vec(0, 0);  // Set the position of the spline at the top
        splineWidget->box.size = Vec(box.size.x, 30);  // Set the width and height of the spline widget
        addChild(splineWidget);
       
    }
    
    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
    
        Nona* NonaModule = dynamic_cast<Nona*>(module);
        assert(NonaModule); // Ensure the cast succeeds
    
        // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());
            
        // MainVCA menu item
        struct MainVCAMenuItem : MenuItem {
            Nona* NonaModule;
            void onAction(const event::Action& e) override {
                // Toggle the "Main VCA" mode
                NonaModule->mainVCA = !NonaModule->mainVCA;
            }
            void step() override {
                // Update the display to show a checkmark when the mode is active
                rightText = NonaModule->mainVCA ? "✔" : "";
                MenuItem::step();
            }
        };
        
        // Create the MainVCA menu item and add it to the menu
        MainVCAMenuItem* mainVCAItem = new MainVCAMenuItem();
        mainVCAItem->text = "First row acts as a master VCA for all channels";
        mainVCAItem->NonaModule = NonaModule;
        menu->addChild(mainVCAItem);        
    }
     
};

Model* modelNona = createModel<Nona, NonaWidget>("Nona");