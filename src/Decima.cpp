////////////////////////////////////////////////////////////
//
//   Decima
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   Ten-stage gate sequencer
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
using namespace rack;

struct Decima : Module {
    enum ParamIds {
        BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4, BUTTON_5,
        BUTTON_6, BUTTON_7, BUTTON_8, BUTTON_9, BUTTON_10,
        STEP_BUTTON_1, STEP_BUTTON_2, STEP_BUTTON_3, STEP_BUTTON_4, STEP_BUTTON_5,
        STEP_BUTTON_6, STEP_BUTTON_7, STEP_BUTTON_8, STEP_BUTTON_9, STEP_BUTTON_10,
        PROB_1, PROB_2, PROB_3, PROB_4, PROB_5,
        PROB_6, PROB_7, PROB_8, PROB_9, PROB_10,
        NUM_PARAMS
    };
    enum InputIds {
        CLOCK_IN,
        RESET_IN,
        DIR_IN,      
        NUM_INPUTS
    };
    enum OutputIds {
        GATE_1, GATE_2, GATE_3, GATE_4, GATE_5, 
        GATE_6, GATE_7, GATE_8, GATE_9, GATE_10,
        OUTPUT, INV_OUTPUT,
        NUM_OUTPUTS
    };
    enum LightIds {   
        BUTTON_LIGHT_1, BUTTON_LIGHT_2, BUTTON_LIGHT_3, BUTTON_LIGHT_4, BUTTON_LIGHT_5,
        BUTTON_LIGHT_6, BUTTON_LIGHT_7, BUTTON_LIGHT_8, BUTTON_LIGHT_9, BUTTON_LIGHT_10,
        STAGE_LIGHT_1, STAGE_LIGHT_2, STAGE_LIGHT_3, STAGE_LIGHT_4, STAGE_LIGHT_5, 
        STAGE_LIGHT_6, STAGE_LIGHT_7, STAGE_LIGHT_8, STAGE_LIGHT_9, STAGE_LIGHT_10, 
        NUM_LIGHTS
    };

    int step = 0;
    bool forward = true;
    dsp::SchmittTrigger clockTrigger, resetTrigger, directionTrigger;
    dsp::SchmittTrigger buttonTrigger[10];
    dsp::SchmittTrigger stepButtonTrigger[10];
    bool stepActive[10] = {false};
    dsp::Timer SyncTimer;
    bool firstClockPulse = true;
    float PulseLength = 1.0f;
    bool trigger = true;
    bool manualStageSelect = false;
    bool probGateEnabled = false;

    json_t* dataToJson() override {
        json_t* rootJ = json_object();

        // Store stepActive array
        json_t* stepActiveJ = json_array();
        for (int i = 0; i < 10; i++) {
            json_array_append_new(stepActiveJ, json_boolean(stepActive[i]));
        }
        json_object_set_new(rootJ, "stepActive", stepActiveJ);
        json_object_set_new(rootJ, "probGateEnabled", json_boolean(probGateEnabled));


        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        // Retrieve stepActive array
        json_t* stepActiveJ = json_object_get(rootJ, "stepActive");
        if (stepActiveJ) {
            for (int i = 0; i < 10; i++) {
                json_t* stepActiveElementJ = json_array_get(stepActiveJ, i);
                if (stepActiveElementJ) {
                    stepActive[i] = json_is_true(stepActiveElementJ);
                }
            }
        }
        
        json_t* probGateEnabledJ = json_object_get(rootJ, "probGateEnabled");
        if (probGateEnabledJ) probGateEnabled = json_is_true(probGateEnabledJ);        
        
    }

    Decima() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        for (int i = 0; i < 10; i++) {
            configParam(BUTTON_1 + i, 0.0f, 1.0f, 0.0f, "Button " + std::to_string(i + 1));
            configParam(STEP_BUTTON_1 + i, 0.0f, 1.0f, 0.0f, "Step Select " + std::to_string(i + 1));
            configParam(PROB_1 + i, 0.0f, 1.0f, 1.0f, "Step Probability " + std::to_string(i + 1));
            configLight(BUTTON_LIGHT_1 + i, "Button Light " + std::to_string(i + 1));
            configLight(STAGE_LIGHT_1 + i, "Stage Light " + std::to_string(i + 1));
        }
        configInput(CLOCK_IN, "Clock In");
        configInput(RESET_IN, "Reset In");
        configInput(DIR_IN, "Direction In");

        for (int i = 0; i < 10; i++) {
            configOutput(GATE_1 + i, "Gate " + std::to_string(i + 1));
        }
        configOutput(OUTPUT, "Output");
        configOutput(INV_OUTPUT, "Inverted Output");
    }

    void onReset(const ResetEvent& e) override {
        // Reset all parameters
        Module::onReset(e);
        // Reset custom state variables
        for (int i=0; i<10; i++){
            stepActive[i] = {false};
        }
    }

    void process(const ProcessArgs &args) override {
        // Handle reset button and input
        bool reset = (inputs[RESET_IN].isConnected() && resetTrigger.process(inputs[RESET_IN].getVoltage()));

        if (reset) {
            step = 0; // Reset step
			float probability = params[PROB_1 + step].getValue();
			trigger = (random::uniform() < probability);
			SyncTimer.reset(); // Reset the timer for the next trigger interval measurement
			firstClockPulse = false; // Ensure firstClockPulse is reset
       }

        // Handle direction input
        if (inputs[DIR_IN].isConnected()) {
            if (inputs[DIR_IN].getVoltage()>0.0f) {
                forward = false;
            } else {
                forward = true;
            }
        } else {
            forward = true;
        }

        // Process timer
        float deltaTime = args.sampleTime;
        SyncTimer.process(deltaTime);

        // Toggle the active state of the step when the button is pressed
        for (int i = 0; i < 10; i++) {
            if (buttonTrigger[i].process(params[BUTTON_1 + i].getValue())) {
                stepActive[i] = !stepActive[i];
            }
        }

        // Handle clock input
        if (inputs[CLOCK_IN].isConnected()) {
            if (clockTrigger.process(inputs[CLOCK_IN].getVoltage())) {
                if (!manualStageSelect) {
                    if (forward) {
                        step = (step + 1) % 10;
                    } else {
                        step = (step - 1 + 10) % 10;
                    }

                    // Compute step probabilities and advance step
                    float probability = params[PROB_1 + step].getValue();
                    trigger = (random::uniform() < probability);

                } else {
                    // Skip advancing the step and reset the flag
                    manualStageSelect = false; 
                }

                if (!firstClockPulse) {
                     PulseLength = 0.5f * SyncTimer.time; // Get the accumulated time since the last reset and compute pulse length
                }
                SyncTimer.reset(); // Reset the timer for the next trigger interval measurement
                firstClockPulse = false;
            }
        }

        // Jump to step if step button is pushed
        for (int i = 0; i < 10; i++) {
            if (stepButtonTrigger[i].process(params[STEP_BUTTON_1 + i].getValue())) {
                step = i;
                manualStageSelect = true;
            }
        }

        // Update gate outputs and lights
        for (int i = 0; i < 10; i++) {
            bool active = stepActive[i];
            lights[BUTTON_LIGHT_1 + i].setBrightness(active ? 1.0f : 0.0f);
            lights[STAGE_LIGHT_1 + i].setBrightness(step == i ? 1.0f : 0.0f);

			if (probGateEnabled) {
				if (step == i && stepActive[i] && trigger) {
					outputs[GATE_1 + i].setVoltage(10.0f);
				} else {
					outputs[GATE_1 + i].setVoltage(0.0f);
				}
			
			} else {
				if (step == i) {
					outputs[GATE_1 + i].setVoltage(10.0f);
				} else {
					outputs[GATE_1 + i].setVoltage(0.0f);
				}
			}    
            
        }
      
        if (!manualStageSelect) { 
            if (stepActive[step] && (SyncTimer.time < PulseLength)) {
                if (trigger) { // Only output if the step is triggered by probability setting
                    outputs[OUTPUT].setVoltage(10.0f);
                    outputs[INV_OUTPUT].setVoltage(0.0f);
                } else {
                    outputs[OUTPUT].setVoltage(0.0f);
                    outputs[INV_OUTPUT].setVoltage(10.0f); // If no trigger, then start the inverse gate
                }
            } 
            else if (!stepActive[step] && (SyncTimer.time < PulseLength)) { // Step inactive by button setting
                outputs[OUTPUT].setVoltage(0.0f);
                outputs[INV_OUTPUT].setVoltage(10.0f);
            }
            else if (SyncTimer.time >= PulseLength) { // End of Pulse zero all gates
                outputs[OUTPUT].setVoltage(0.0f);
                outputs[INV_OUTPUT].setVoltage(0.0f);
            }
        } else {
            // Suppress the output voltages
            outputs[OUTPUT].setVoltage(0.0f);
            outputs[INV_OUTPUT].setVoltage(0.0f);
        }
    }
};

struct DecimaWidget : ModuleWidget {
    DecimaWidget(Decima* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Decima.svg"),
            asset::plugin(pluginInstance, "res/Decima-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // Position the inputs for Clock and Reset above the buttons
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(box.size.x / 2 - 40, 42), module, Decima::CLOCK_IN));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(box.size.x / 2 , 42), module, Decima::DIR_IN));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(box.size.x / 2 + 40, 42), module, Decima::RESET_IN));

        // Position the buttons, lights, and outputs
        for (int i = 0; i < 10; i++) {
            float y = 80 + i * 25; // Spacing the buttons and lights vertically

            // Stage lights to the left of the buttons
            addParam(createParamCentered<LEDButton>(Vec(22, y), module, Decima::STEP_BUTTON_1 + i));
            addChild(createLightCentered<LargeLight<RedLight>>(Vec(22, y), module, Decima::STAGE_LIGHT_1 + i));

            // Buttons
            addParam(createParamCentered<LEDButton>(Vec(57, y - 5), module, Decima::BUTTON_1 + i));
            addChild(createLightCentered<MediumLight<GreenLight>>(Vec(57, y - 5), module, Decima::BUTTON_LIGHT_1 + i));

            //Probability knob
            addParam(createParamCentered<Trimpot> (Vec( 92, y - 5), module,  Decima::PROB_1 + i));

            // Gate outputs to the right of the buttons
            addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(127, y), module, Decima::GATE_1 + i));
        }

        // Position the main output under the buttons
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(57, 338), module, Decima::OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(92, 338), module, Decima::INV_OUTPUT));
    }
    
	void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);

        Decima* decimaModule = dynamic_cast<Decima*>(module);
        assert(decimaModule); // Ensure the cast succeeds

        // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());

        // Retriggering enabled/disabled menu item
        struct ProbgateEnabledItem : MenuItem {
            Decima* decimaModule;
            void onAction(const event::Action& e) override {
                decimaModule->probGateEnabled = !decimaModule->probGateEnabled;
            }
            void step() override {
                rightText = decimaModule->probGateEnabled ? "✔" : "";
                MenuItem::step();
            }
        };

        ProbgateEnabledItem* probGateItem = new ProbgateEnabledItem();
        probGateItem->text = "Active step outputs to Gate output";
        probGateItem->decimaModule = decimaModule; // Ensure we're setting the module
        menu->addChild(probGateItem);
    }

    
};

Model* modelDecima = createModel<Decima, DecimaWidget>("Decima");