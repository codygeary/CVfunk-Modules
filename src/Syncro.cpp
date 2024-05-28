#include "rack.hpp"
#include "plugin.hpp"
#include "digital_display.hpp" // Include the DigitalDisplay header
#include <iomanip>
#include <sstream>

using namespace rack;

struct Syncro : Module {
    enum ParamIds {
        CLOCK_KNOB, CLOCK_ATT,
        SWING_KNOB, SWING_ATT,
        FILL_KNOB, FILL_ATT,
        WIDTH_KNOB, WIDTH_ATT,
        ROTATE_KNOB, ROTATE_ATT,
        MULTIPLY_KNOB_1, MULTIPLY_KNOB_2, MULTIPLY_KNOB_3, MULTIPLY_KNOB_4,
        MULTIPLY_KNOB_5, MULTIPLY_KNOB_6, MULTIPLY_KNOB_7, MULTIPLY_KNOB_8,
        DIVIDE_KNOB_1, DIVIDE_KNOB_2, DIVIDE_KNOB_3, DIVIDE_KNOB_4,
        DIVIDE_KNOB_5, DIVIDE_KNOB_6, DIVIDE_KNOB_7, DIVIDE_KNOB_8,
        FILL_BUTTON_1, FILL_BUTTON_2, FILL_BUTTON_3, FILL_BUTTON_4,
        FILL_BUTTON_5, FILL_BUTTON_6, FILL_BUTTON_7, FILL_BUTTON_8,
        ON_OFF_BUTTON, RESET_BUTTON,
        NUM_PARAMS
    };
    enum InputIds {
        CLOCK_INPUT, SWING_INPUT,
        FILL_INPUT, WIDTH_INPUT, ROTATE_INPUT,      
        EXT_CLOCK_INPUT, ON_OFF_INPUT, RESET_INPUT,
        FILL_INPUT_1, FILL_INPUT_2, FILL_INPUT_3, FILL_INPUT_4,
        FILL_INPUT_5, FILL_INPUT_6, FILL_INPUT_7, FILL_INPUT_8,        
        NUM_INPUTS
    };
    enum OutputIds {
        CLOCK_OUTPUT, INV_CLOCK_OUTPUT,
        CLOCK_OUTPUT_1, INV_CLOCK_OUTPUT_1,
        CLOCK_OUTPUT_2, INV_CLOCK_OUTPUT_2,
        CLOCK_OUTPUT_3, INV_CLOCK_OUTPUT_3,
        CLOCK_OUTPUT_4, INV_CLOCK_OUTPUT_4,
        CLOCK_OUTPUT_5, INV_CLOCK_OUTPUT_5,
        CLOCK_OUTPUT_6, INV_CLOCK_OUTPUT_6,
        CLOCK_OUTPUT_7, INV_CLOCK_OUTPUT_7,
        CLOCK_OUTPUT_8, INV_CLOCK_OUTPUT_8,
        NUM_OUTPUTS
    };
    enum LightIds {
        CLOCK_LIGHT, INV_CLOCK_LIGHT,
        CLOCK_LIGHT_1, INV_CLOCK_LIGHT_1,
        CLOCK_LIGHT_2, INV_CLOCK_LIGHT_2,
        CLOCK_LIGHT_3, INV_CLOCK_LIGHT_3,
        CLOCK_LIGHT_4, INV_CLOCK_LIGHT_4,
        CLOCK_LIGHT_5, INV_CLOCK_LIGHT_5,
        CLOCK_LIGHT_6, INV_CLOCK_LIGHT_6,
        CLOCK_LIGHT_7, INV_CLOCK_LIGHT_7,
        CLOCK_LIGHT_8, INV_CLOCK_LIGHT_8,
        FILL_LIGHT_1, FILL_LIGHT_2, FILL_LIGHT_3, FILL_LIGHT_4, 
        FILL_LIGHT_5, FILL_LIGHT_6, FILL_LIGHT_7, FILL_LIGHT_8,      
        FILL_INDICATE_1, FILL_INDICATE_2, FILL_INDICATE_3, FILL_INDICATE_4, 
        FILL_INDICATE_5, FILL_INDICATE_6, FILL_INDICATE_7, FILL_INDICATE_8,  
        ON_OFF_LIGHT, RESET_LIGHT,    
        NUM_LIGHTS
    };

    DigitalDisplay* bpmDisplay = nullptr;
    DigitalDisplay* swingDisplay = nullptr;
    DigitalDisplay* ratioDisplays[8] = {nullptr};

    Light* fillLights[8];
    Light* gateStateLights[18]; // 9 outputs with regular and inverted gate state indicators

    dsp::Timer SyncTimer;
    dsp::Timer SwingTimer;
    float SwingPhase = 0.0f;
	dsp::Timer ClockTimer[9];  // Array to store timers for each clock
    dsp::SchmittTrigger SyncTrigger;
	dsp::SchmittTrigger resetTrigger;
	dsp::SchmittTrigger onOffTrigger;
	dsp::SchmittTrigger onOffButtonTrigger;

	
	bool sequenceRunning = true;

    float lastClockTime = -1.0f;
    float warpedTime = 1.0f;
    float bpm = 120.0f;
    int displayUpdateCounter = 0;
    float phase = 0.0f;
	float multiply[9] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}; 
	float divide[9] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};   
	float ratio[9] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};   
	float disp_multiply[9] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}; 
	float disp_divide[9] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};   
	bool resyncFlag[9] = {false};
    bool firstClockPulse = true;
    float SyncInterval = 0.0f;
    float clockRate = 120.0f;
	float phases[9] = {0.0f};  // Array to store phases for each clock
	bool fill[9] = {false}; // Array to track the fill state for each channel
    int fillGlobal = 0;

    Syncro() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        // Configure parameters
        configParam(CLOCK_KNOB, 0.1f, 360.0f, 120.0f, "Clock Rate", " BPM");
        configParam(SWING_KNOB, -99.0f, 99.0f, 0.0f, "Swing", " %");
        configParam(MULTIPLY_KNOB_1, 1.0f, 128.0f, 1.0f, "Multiply 1");
        configParam(MULTIPLY_KNOB_2, 1.0f, 128.0f, 1.0f, "Multiply 2");
        configParam(MULTIPLY_KNOB_3, 1.0f, 128.0f, 1.0f, "Multiply 3");
        configParam(MULTIPLY_KNOB_4, 1.0f, 128.0f, 1.0f, "Multiply 4");
        configParam(MULTIPLY_KNOB_5, 1.0f, 128.0f, 1.0f, "Multiply 5");
        configParam(MULTIPLY_KNOB_6, 1.0f, 128.0f, 1.0f, "Multiply 6");
        configParam(MULTIPLY_KNOB_7, 1.0f, 128.0f, 1.0f, "Multiply 7");
        configParam(MULTIPLY_KNOB_8, 1.0f, 128.0f, 1.0f, "Multiply 8");
        configParam(DIVIDE_KNOB_1, 1.0f, 128.0f, 1.0f, "Divide 1");
        configParam(DIVIDE_KNOB_2, 1.0f, 128.0f, 1.0f, "Divide 2");
        configParam(DIVIDE_KNOB_3, 1.0f, 128.0f, 1.0f, "Divide 3");
        configParam(DIVIDE_KNOB_4, 1.0f, 128.0f, 1.0f, "Divide 4");
        configParam(DIVIDE_KNOB_5, 1.0f, 128.0f, 1.0f, "Divide 5");
        configParam(DIVIDE_KNOB_6, 1.0f, 128.0f, 1.0f, "Divide 6");
        configParam(DIVIDE_KNOB_7, 1.0f, 128.0f, 1.0f, "Divide 7");
        configParam(DIVIDE_KNOB_8, 1.0f, 128.0f, 1.0f, "Divide 8");
        configParam(FILL_KNOB, 0.0f, 8.0f, 0.0f, "Fill");
        configParam(WIDTH_KNOB, 0.0f, 1.0f, 0.5f, "Gate Width");
        configParam(ROTATE_KNOB, -1.0f, 1.0f, 0.0f, "Rotate");

        // Configure inputs and outputs
        configInput(EXT_CLOCK_INPUT, "External Clock");
        configInput(RESET_INPUT, "Reset");
        configInput(ON_OFF_INPUT, "ON/OFF");
        configOutput(CLOCK_OUTPUT, "Main Clock");
        configOutput(INV_CLOCK_OUTPUT, "Inverted Main Clock");
        configOutput(CLOCK_OUTPUT_1, "Clock 1");
        configOutput(INV_CLOCK_OUTPUT_1, "Inverted Clock 1");
        configOutput(CLOCK_OUTPUT_2, "Clock 2");
        configOutput(INV_CLOCK_OUTPUT_2, "Inverted Clock 2");
        configOutput(CLOCK_OUTPUT_3, "Clock 3");
        configOutput(INV_CLOCK_OUTPUT_3, "Inverted Clock 3");
        configOutput(CLOCK_OUTPUT_4, "Clock 4");
        configOutput(INV_CLOCK_OUTPUT_4, "Inverted Clock 4");
        configOutput(CLOCK_OUTPUT_5, "Clock 5");
        configOutput(INV_CLOCK_OUTPUT_5, "Inverted Clock 5");
        configOutput(CLOCK_OUTPUT_6, "Clock 6");
        configOutput(INV_CLOCK_OUTPUT_6, "Inverted Clock 6");
        configOutput(CLOCK_OUTPUT_7, "Clock 7");
        configOutput(INV_CLOCK_OUTPUT_7, "Inverted Clock 7");
        configOutput(CLOCK_OUTPUT_8, "Clock 8");
        configOutput(INV_CLOCK_OUTPUT_8, "Inverted Clock 8");

        // Initialize fill LEDs
        for (int i = 1; i < 9; i++) {
            fillLights[i] = new Light();
            configLight(FILL_LIGHT_1 + i, "Fill Light " + std::to_string(i + 1));
        }

        // Initialize gate state lights
        for (int i = 0; i < 18; i++) {
            gateStateLights[i] = new Light();
            configLight(CLOCK_LIGHT + i, "Gate State Light " + std::to_string(i + 1));
        }
    }

    void process(const ProcessArgs &args) override {
        float swing = params[SWING_KNOB].getValue() + (inputs[SWING_INPUT].isConnected() ? 10.f * inputs[SWING_INPUT].getVoltage() * params[SWING_ATT].getValue() : 0.0f);   
        swing = clamp (swing, -99.f, 99.f);     
        float width = params[WIDTH_KNOB].getValue() + (inputs[WIDTH_INPUT].isConnected() ? 0.1f * inputs[WIDTH_INPUT].getVoltage() * params[WIDTH_ATT].getValue() : 0.0f);
        width = clamp (width, 0.01f, 0.99f);
        float rotate = params[ROTATE_KNOB].getValue() + (inputs[ROTATE_INPUT].isConnected() ? 0.2f * inputs[ROTATE_INPUT].getVoltage() * params[ROTATE_ATT].getValue() : 0.0f);
        int clockRotate = static_cast<int>(round(fmod(-8.0f*rotate, 8.0f)));
        float deltaTime = args.sampleTime;
        float actualTime = deltaTime;
        bool isExtClock = inputs[EXT_CLOCK_INPUT].isConnected();
  
  		SwingPhase = SwingTimer.time/(120.f/bpm) ;      
        if (SwingPhase >= 1.0f){
           SwingTimer.reset();
        }

        deltaTime *= 1.0f + (swing / 100.0f * sinf(2.0f * M_PI * SwingPhase));

        // Check for on/off input or on/off button
        bool onOffCondition = false;
        if (inputs[ON_OFF_INPUT].isConnected()) {
            onOffCondition = onOffTrigger.process(inputs[ON_OFF_INPUT].getVoltage()) || onOffButtonTrigger.process(params[ON_OFF_BUTTON].getValue() > 0.1f);
        } else {
            onOffCondition = onOffButtonTrigger.process(params[ON_OFF_BUTTON].getValue());    
        }
  
        if (onOffCondition) {
            sequenceRunning = !sequenceRunning; // Toggle sequenceRunning
        }
  
        lights[ON_OFF_LIGHT].setBrightness(sequenceRunning ? 1.0f : 0.0f);        

        if (!sequenceRunning) {
            deltaTime = 0.f;
            for (int i = 0; i < 9; i++) {
                ClockTimer[i].reset();
            }               
        }
        
        // Process timers
        SyncTimer.process(actualTime);
        SwingTimer.process(deltaTime);

		// Update fill state
		for (int i = 1; i < 9; i++) {
			fill[i-1] = (params[FILL_BUTTON_1 + i - 1].getValue() > 0.1f) || (inputs[FILL_INPUT_1 + i - 1].getVoltage() > 0.1f);
		}

        if (inputs[RESET_INPUT].isConnected()){
        }
        
        // Process clock sync input
        if (isExtClock) {
            float SyncInputVoltage = inputs[EXT_CLOCK_INPUT].getVoltage();
            
            if (SyncTrigger.process(SyncInputVoltage)) {
                if (!firstClockPulse) {
                    SyncInterval = SyncTimer.time; // Get the accumulated time since the last reset
                }
                SyncTimer.reset(); // Reset the timer for the next trigger interval measurement
                firstClockPulse = false;
            }
            if (SyncInterval > 0) {
                bpm = 60.f / SyncInterval;
            } else {
                bpm = 120.f;  //div by zero protection, default to 120bpm
            }                       
        } else {
            // Calculate phase increment
            bpm = params[CLOCK_KNOB].getValue() + (inputs[CLOCK_INPUT].isConnected() ? inputs[CLOCK_INPUT].getVoltage() * params[CLOCK_ATT].getValue() : 0.0f);
        }
 
		// Check for reset input or reset button
		bool resetCondition = false;
		bool lightOn = false;
		if (inputs[RESET_INPUT].isConnected()){
		   resetCondition = resetTrigger.process(inputs[RESET_INPUT].getVoltage()) || params[RESET_BUTTON].getValue() > 0.1f;
	       lightOn = inputs[RESET_INPUT].getVoltage() > 0.001f || params[RESET_BUTTON].getValue() > 0.1f ;
	    } else {
		   resetCondition = params[RESET_BUTTON].getValue() > 0.1f;	    
	       lightOn = params[RESET_BUTTON].getValue() > 0.1f ;
	    }
		
		if (resetCondition) {
			for (int i = 0; i < 9; i++) {
				ClockTimer[i].reset();
			}			
		}
		
		lights[RESET_LIGHT].setBrightness(lightOn ? 1.0f : 0.0f);		    

		fillGlobal = static_cast<int>(round(params[FILL_KNOB].getValue() + (inputs[FILL_INPUT].isConnected() ? inputs[FILL_INPUT].getVoltage() * params[FILL_ATT].getValue() : 0.0f)));               

		for (int i = 0; i < 9; i++) {
			ClockTimer[i].process(deltaTime);
			
			if (ClockTimer[i].time >= (60.0f / (bpm * ratio[i]))) {
				ClockTimer[i].reset();
				if (i < 1) {
					for (int j = 1; j < 9; j++) {
						if (resyncFlag[j]) {
							ClockTimer[j].reset();
							resyncFlag[j] = false;
						}

						int index = (clockRotate + j - 1) % 8;
						if (index < 0) {
							index += 8; // Adjust for negative values to wrap around correctly
						}

						multiply[j] = round(params[MULTIPLY_KNOB_1 + index].getValue()) + (fill[index] ? fillGlobal : 0);
						divide[j] = round(params[DIVIDE_KNOB_1 + index].getValue());
						if (fill[index]) {
							resyncFlag[j] = true;
						}
					}
				}
			}

			if (i > 0) {
				ratio[i] = multiply[i] / divide[i];
			}
			// Apply swing as a global adjustment to the phase increment
			phases[i] = ClockTimer[i].time/(60.0f / ( bpm * ratio[i] ));

			// Determine the output state based on pulse width
			bool highState = phases[i] < width;
						
            if(sequenceRunning){
                outputs[CLOCK_OUTPUT + 2 * i].setVoltage(highState ? 5.0f : 0.0f);
                outputs[CLOCK_OUTPUT + 2 * i + 1].setVoltage(highState ? 0.0f : 5.0f);
                lights[CLOCK_LIGHT + 2 * i].setBrightness(highState ? 1.0f : 0.0f);
                lights[CLOCK_LIGHT + 2 * i + 1].setBrightness(highState ? 0.0f : 1.0f);
            } else{
                outputs[CLOCK_OUTPUT + 2 * i].setVoltage(0.f);
                outputs[CLOCK_OUTPUT + 2 * i + 1].setVoltage(0.f);
                lights[CLOCK_LIGHT + 2 * i].setBrightness( 0.0f);
                lights[CLOCK_LIGHT + 2 * i + 1].setBrightness(0.0f);
            }

		}		 
                                 
        displayUpdateCounter++;
        if (displayUpdateCounter >= (1.0f / deltaTime / 30.0f)) { // Update 30 times per second
            displayUpdateCounter = 0;
            // Update BPM and Swing displays
            if (bpmDisplay) {
                std::stringstream bpmStream;
                bpmStream << std::fixed << std::setprecision(1) << bpm;
                bpmDisplay->text = bpmStream.str();
            }

            if (swingDisplay) {
                float swingValue = params[SWING_KNOB].getValue();
                std::stringstream swingStream;
                swingStream << std::fixed << std::setprecision(1) << swingValue << "%";
                swingDisplay->text = swingStream.str();
            }

			// Update ratio displays
			for (int i = 1; i < 9; i++) {
				int index = (clockRotate + i - 1) % 8;
				if (index < 0) {
					index += 8; // Adjust for negative values to wrap around correctly
				}
				index += 1; // Convert from 0-based to 1-based index

				disp_multiply[i] = round(params[MULTIPLY_KNOB_1 + index - 1].getValue()) + (fill[index - 1] ? fillGlobal : 0);
				disp_divide[i] = round(params[DIVIDE_KNOB_1 + index - 1].getValue());
				if (ratioDisplays[i-1]) {
					std::string text = std::to_string(static_cast<int>(disp_multiply[i])) + ":" + std::to_string(static_cast<int>(disp_divide[i]));
					if (index == 1) { // Check if the current index corresponds to the rotated position
						text = "• " + text;
					}
					ratioDisplays[i-1]->text = text;
				}
			}

            for (int i = 0; i < 8; i++){
                if (i < fillGlobal){
                    lights[FILL_LIGHT_1 + i].setBrightness(1.0f);                 
                } else {
                    lights[FILL_LIGHT_1 + i].setBrightness(0.0f);                                 
                }
                if (fill[i]){ 
                    lights[FILL_INDICATE_1 + i].setBrightness(1.0f);                                 
                } else {
                    lights[FILL_INDICATE_1 + i].setBrightness(0.0f);                                               
                }
            }            
        }
    }
};

struct SyncroWidget : ModuleWidget {
    SyncroWidget(Syncro* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Syncro-dark.svg"),
            asset::plugin(pluginInstance, "res/Syncro-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        addParam(createParamCentered<RoundBlackKnob>  (Vec(55,     80), module, Syncro::CLOCK_KNOB));
        addParam(createParamCentered<Trimpot>         (Vec(81.25,  80), module, Syncro::CLOCK_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(103.58, 80), module, Syncro::CLOCK_INPUT));

        // Add fill LEDs
        for (int i = 0; i < 8; i++) {
            addChild(createLight<SmallLight<YellowLight>>(Vec(42 + i * 10, 120 ), module, Syncro::FILL_LIGHT_1 + i));
        }

        addParam(createParamCentered<RoundBlackKnob>  (Vec(55,     145), module, Syncro::FILL_KNOB));
        addParam(createParamCentered<Trimpot>         (Vec(81.25,  145), module, Syncro::FILL_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(103.58, 145), module, Syncro::FILL_INPUT));

        addParam(createParamCentered<RoundBlackKnob>      (Vec( 30, 200+ 0  ), module, Syncro::SWING_KNOB));
        addParam(createParamCentered<Trimpot>             (Vec( 30, 200+ 30 ), module, Syncro::SWING_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>    (Vec( 30, 200+ 55 ), module, Syncro::SWING_INPUT));

        addParam(createParamCentered<RoundBlackKnob>      (Vec( 80, 200+ 0  ), module, Syncro::ROTATE_KNOB));
        addParam(createParamCentered<Trimpot>             (Vec( 80, 200+ 30 ), module, Syncro::ROTATE_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>    (Vec( 80, 200+ 55 ), module, Syncro::ROTATE_INPUT));

        addParam(createParamCentered<RoundBlackKnob>      (Vec( 130, 200+ 0  ), module, Syncro::WIDTH_KNOB));
        addParam(createParamCentered<Trimpot>             (Vec( 130, 200+ 30 ), module, Syncro::WIDTH_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>    (Vec( 130, 200+ 55 ), module, Syncro::WIDTH_INPUT));

        addInput(createInputCentered<ThemedPJ301MPort>    (Vec(30,330), module, Syncro::EXT_CLOCK_INPUT));

        addParam(createParamCentered<TL1105>              (Vec(80,305), module, Syncro::ON_OFF_BUTTON));
		addChild(createLightCentered<MediumLight<YellowLight>>(Vec(80,305), module, Syncro::ON_OFF_LIGHT ));
        addInput(createInputCentered<ThemedPJ301MPort>    (Vec(80,330), module, Syncro::ON_OFF_INPUT));

        addParam(createParamCentered<TL1105>              (Vec(130,305), module, Syncro::RESET_BUTTON));
		addChild(createLightCentered<MediumLight<YellowLight>>(Vec(130,305), module, Syncro::RESET_LIGHT ));
        addInput(createInputCentered<ThemedPJ301MPort>    (Vec(130,330), module, Syncro::RESET_INPUT));


        for (int i = 0; i < 8; i++) {
            // Add Multiply and Divide Knobs
            addParam(createParamCentered<Trimpot>             (Vec( 165, 35 + 38 + i * 38 ), module, Syncro::MULTIPLY_KNOB_1 + i));
            addParam(createParamCentered<Trimpot>             (Vec( 195, 35 + 38 + i * 38 ), module, Syncro::DIVIDE_KNOB_1 + i));

            addParam(createParamCentered<TL1105>              (Vec(280, 35 + 38 + i * 38 ), module, Syncro::FILL_BUTTON_1 + i));
            addChild(createLightCentered<MediumLight<YellowLight>>    (Vec(280, 35 + 38 + i * 38 ), module, Syncro::FILL_INDICATE_1 + i ));
            addInput(createInputCentered<ThemedPJ301MPort>    (Vec(300, 35 + 38 + i * 38 ), module, Syncro::FILL_INPUT_1 + i));
        }

        for (int i = 0; i < 9; i++) {
            // Add gate state lights
            addChild(createLight<SmallLight<YellowLight>>(Vec(320, 33 + i * 38), module, Syncro::CLOCK_LIGHT + 2*i));
            addChild(createLight<SmallLight<YellowLight>>(Vec(350, 33 + i * 38), module, Syncro::CLOCK_LIGHT + 2*i + 1));

            // Add gate outputs
            addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(338,35 + i * 38), module, Syncro::CLOCK_OUTPUT + 2*i));
            addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(368,35 + i * 38), module, Syncro::CLOCK_OUTPUT + 2*i + 1));
        }

        // Add DigitalDisplays for BPM and Swing
        if (module) {
            module->bpmDisplay = new DigitalDisplay();
            addDigitalDisplay(module->bpmDisplay, Vec(27, 28)); 

            module->swingDisplay = new DigitalDisplay();
            addDigitalDisplay(module->swingDisplay, Vec(90, 28));

            // Add ratio displays
            for (int i = 0; i < 8; i++) {
                module->ratioDisplays[i] = new DigitalDisplay();
                addDigitalDisplay(module->ratioDisplays[i], Vec(215, 65 + i * 38));
            }
        }       
    }

    void addDigitalDisplay(DigitalDisplay* display, Vec position) {
        display->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        display->box.pos = position;
        display->box.size = Vec(50, 18);
        display->text = "0"; // Default text
        display->fgColor = nvgRGB(208, 140, 89); // Gold color text
        display->setFontSize(14.0f);
        addChild(display);
    }
};

Model* modelSyncro = createModel<Syncro, SyncroWidget>("Syncro");

