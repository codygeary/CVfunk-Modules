////////////////////////////////////////////////////////////
//
//   Syncro
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   Clock ratio generator with 8 channels
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
#include "digital_display.hpp" // Include the DigitalDisplay header
using namespace rack;
#include <cmath>

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
        ON_OFF_LIGHT,    
        NUM_LIGHTS
    };

    DigitalDisplay* phasorDisplay = nullptr;
    DigitalDisplay* bpmDisplay = nullptr;
    DigitalDisplay* swingDisplay = nullptr;
    DigitalDisplay* ratioDisplays[8] = {nullptr};

    Light fillLights[8];
    Light gateStateLights[18];

    dsp::Timer SyncTimer;
    dsp::Timer SwingTimer;
    dsp::Timer ClockTimer[9];  // Array to store timers for each clock
 
    dsp::SchmittTrigger SyncTrigger;
    dsp::SchmittTrigger resetTrigger;
    dsp::SchmittTrigger onOffTrigger;
    dsp::SchmittTrigger onOffButtonTrigger;

    float SwingPhase = 0.0f;
    float lastClockTime = -1.0f;
    float bpm = 120.0f;
    float phase = 0.0f;
    float multiply[9] = {1.0f}; 
    float divide[9] = {1.0f};   
    float ratio[9] = {1.0f};   
    float disp_multiply[9] = {1.0f}; 
    float disp_divide[9] = {1.0f};   
    float SyncInterval = 1.0f;
    float clockRate = 120.0f;
    float phases[9] = {0.0f};  // Array to store phases for each clock
    float tempPhases[9] = {0.0f};
    float swing = 0.f;

    int clockRotate = 0;
    int swingCount = 0;
    int fillGlobal = 0;
    int masterClockCycle = 0;

    bool fill[9] = {false}; // Array to track the fill state for each channel
    bool resyncFlag[9] = {false};
    bool firstClockPulse = true;
    bool sequenceRunning = true;
    bool phasorMode = false;
    bool clockCVAsVoct = false;
    bool clockCVAsBPM = true;  
    bool resetPulse = false;

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
        json_object_set_new(rootJ, "sequenceRunning", json_boolean(sequenceRunning));
        json_object_set_new(rootJ, "phasorMode", json_boolean(phasorMode));
        json_object_set_new(rootJ, "clockCVAsVoct", json_boolean(clockCVAsVoct));
        json_object_set_new(rootJ, "clockCVAsBPM", json_boolean(clockCVAsBPM));  
        return rootJ;
    }
    
    void dataFromJson(json_t* rootJ) override {
        json_t* sequenceRunningJ = json_object_get(rootJ, "sequenceRunning");
        if (sequenceRunningJ) {
            sequenceRunning = json_is_true(sequenceRunningJ);
        }
    
        json_t* phasorModeJ = json_object_get(rootJ, "phasorMode");
        if (phasorModeJ) {
            phasorMode = json_is_true(phasorModeJ);
        }
    
        json_t* clockCVAsVoctJ = json_object_get(rootJ, "clockCVAsVoct");
        if (clockCVAsVoctJ) {
            clockCVAsVoct = json_is_true(clockCVAsVoctJ);
        }
    
        json_t* clockCVAsBPMJ = json_object_get(rootJ, "clockCVAsBPM");
        if (clockCVAsBPMJ) {
            clockCVAsBPM = json_is_true(clockCVAsBPMJ);
        }
    
    }

    Syncro() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    
        // Configure parameters
        configParam(CLOCK_KNOB, 0.000001f, 480.0f, 120.0f, "Clock Rate", " BPM");
        configParam(CLOCK_ATT, -1.f, 1.f, 0.0f, "Clock Attenuvertor");
        configParam(SWING_KNOB, -99.0f, 99.0f, 0.0f, "Swing", " %");
        configParam(SWING_ATT, -1.f, 1.f, 0.0f, "Swing Attenuvertor");
        configParam(MULTIPLY_KNOB_1, 0.0f, 128.0f, 1.0f, "Multiply 1")->snapEnabled=true;
        configParam(MULTIPLY_KNOB_2, 0.0f, 128.0f, 1.0f, "Multiply 2")->snapEnabled=true;
        configParam(MULTIPLY_KNOB_3, 0.0f, 128.0f, 1.0f, "Multiply 3")->snapEnabled=true;
        configParam(MULTIPLY_KNOB_4, 0.0f, 128.0f, 1.0f, "Multiply 4")->snapEnabled=true;
        configParam(MULTIPLY_KNOB_5, 0.0f, 128.0f, 1.0f, "Multiply 5")->snapEnabled=true;
        configParam(MULTIPLY_KNOB_6, 0.0f, 128.0f, 1.0f, "Multiply 6")->snapEnabled=true;
        configParam(MULTIPLY_KNOB_7, 0.0f, 128.0f, 1.0f, "Multiply 7")->snapEnabled=true;
        configParam(MULTIPLY_KNOB_8, 0.0f, 128.0f, 1.0f, "Multiply 8")->snapEnabled=true;
        configParam(DIVIDE_KNOB_1, 1.0f, 256.0f, 1.0f, "Divide 1")->snapEnabled=true;
        configParam(DIVIDE_KNOB_2, 1.0f, 256.0f, 1.0f, "Divide 2")->snapEnabled=true;
        configParam(DIVIDE_KNOB_3, 1.0f, 256.0f, 1.0f, "Divide 3")->snapEnabled=true;
        configParam(DIVIDE_KNOB_4, 1.0f, 256.0f, 1.0f, "Divide 4")->snapEnabled=true;
        configParam(DIVIDE_KNOB_5, 1.0f, 256.0f, 1.0f, "Divide 5")->snapEnabled=true;
        configParam(DIVIDE_KNOB_6, 1.0f, 256.0f, 1.0f, "Divide 6")->snapEnabled=true;
        configParam(DIVIDE_KNOB_7, 1.0f, 256.0f, 1.0f, "Divide 7")->snapEnabled=true;
        configParam(DIVIDE_KNOB_8, 1.0f, 256.0f, 1.0f, "Divide 8")->snapEnabled=true;
        configParam(FILL_KNOB, 0.0f, 8.0f, 3.0f, "Fill")->snapEnabled=true;
        configParam(FILL_ATT, -1.0f, 1.0f, 0.0f, "Fill Attenuvertor");
        configParam(WIDTH_KNOB, 0.0f, 1.0f, 0.5f, "Gate Width");
        configParam(WIDTH_ATT, -1.0f, 1.0f, 0.0f, "Gate Width Attenuvertor");
        configParam(ROTATE_KNOB, -1.0f, 1.0f, 0.0f, "Rotate");
        configParam(ROTATE_ATT, -1.0f, 1.0f, 0.0f, "Rotate Atenuvertor");

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

        configInput(CLOCK_INPUT , "Clock" );
        configInput(SWING_INPUT , "Swing" );
        configInput(FILL_INPUT , "Fill" );
        configInput(WIDTH_INPUT , "Pulse Width" );
        configInput(ROTATE_INPUT , "Rotation" );
        configParam(RESET_BUTTON, 0.0, 1.0, 0.0, "Reset" );
        configParam(ON_OFF_BUTTON, 0.0, 1.0, 0.0, "On / Off " );

        // Initialize fill buttons
        for (int i = 0; i < 8; i++) {
            configParam(FILL_BUTTON_1 + i, 0.0, 1.0, 0.0, "Fill Button " + std::to_string(i + 1));
        }

        // Initialize fill inputs
        for (int i = 0; i < 8; i++) {
            configInput(FILL_INPUT_1 + i, "Fill " + std::to_string(i + 1));
        }

        // Initialize fill LEDs
        for (int i = 0; i < 8; i++) {
            configLight(FILL_LIGHT_1 + i, "Fill " + std::to_string(i + 1));
        }

        // Initialize gate state lights
        for (int i = 0; i < 18; i++) {
            configLight(CLOCK_LIGHT + i, "Gate State " + std::to_string(i + 1));
        }
    }

    void process(const ProcessArgs &args) override {
        swing = params[SWING_KNOB].getValue() + (inputs[SWING_INPUT].isConnected() ? 10.f * inputs[SWING_INPUT].getVoltage() * params[SWING_ATT].getValue() : 0.0f);
        swing = clamp(swing, -99.f, 99.f);
        float width = params[WIDTH_KNOB].getValue() + (inputs[WIDTH_INPUT].isConnected() ? 0.1f * inputs[WIDTH_INPUT].getVoltage() * params[WIDTH_ATT].getValue() : 0.0f);
        width = clamp(width, 0.001f, 0.999f);
        float rotate = params[ROTATE_KNOB].getValue() + (inputs[ROTATE_INPUT].isConnected() ? 0.1f*inputs[ROTATE_INPUT].getVoltage() * params[ROTATE_ATT].getValue() : 0.0f);
        clockRotate = static_cast<int>(std::roundf(fmod(-8.0f * rotate, 8.0f)));

        // Process clock sync input
        if (inputs[EXT_CLOCK_INPUT].isConnected()) {
            float SyncInputVoltage = inputs[EXT_CLOCK_INPUT].getVoltage();

            if (SyncTrigger.process(SyncInputVoltage)) {
                if (!firstClockPulse) {
                     SyncInterval = SyncTimer.time; // Get the accumulated time since the last reset
                }
                SyncTimer.reset(); // Reset the timer for the next trigger interval measurement
                phases[0] = 0.0f;
                ClockTimer[0].reset();
                resetPulse = true;

                firstClockPulse = false;
            }
            bpm = (SyncInterval > 0) ? (60.f / SyncInterval) : 120.f; // Use default 120 BPM if SyncInterval is zero

        } else {
            // Calculate phase increment
            if ( clockCVAsVoct && inputs[CLOCK_INPUT].isConnected() ) {
                float input_v_oct = inputs[CLOCK_INPUT].getVoltage();//ignore the attenuator in this mode
                // Process input_v_oct into BPM using V/Oct scale centered on 120 BPM
                bpm = 120.f * powf(2.f, input_v_oct);
            } else {
                bpm = params[CLOCK_KNOB].getValue() + (inputs[CLOCK_INPUT].isConnected() ? 10.f * inputs[CLOCK_INPUT].getVoltage() * params[CLOCK_ATT].getValue() : 0.0f);
            }
        }

        double deltaTime = args.sampleTime;

        // Process timers
        SyncTimer.process(deltaTime);
        SwingTimer.process(deltaTime);

        // Compute swing
        SwingPhase = SwingTimer.time / (120.0 / bpm);
        deltaTime *= (1.0 + (swing / 100.0) * cos(2.0 * M_PI * SwingPhase));

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

        // Update fill state
        for (int i = 1; i < 9; i++) {
            fill[i-1] = (params[FILL_BUTTON_1 + i - 1].getValue() > 0.1f) || (inputs[FILL_INPUT_1 + i - 1].getVoltage() > 0.1f);
        }

        // Calculate the Nyquist frequency and the maximum BPM considering a 128x multiplier
        float maxBPM = (args.sampleRate * 60.0f) / 256.0f;

        // Clamp BPM based on the Nyquist frequency
        bpm = clamp(bpm, 0.000001f, maxBPM);

        // Check for reset input or reset button
        bool resetCondition = (inputs[RESET_INPUT].isConnected() && resetTrigger.process(inputs[RESET_INPUT].getVoltage())) || (params[RESET_BUTTON].getValue() > 0.1f);

        if (resetCondition) {
            for (int i = 0; i < 9; i++) {
                ClockTimer[i].reset();
                outputs[CLOCK_OUTPUT + 2 * i].setVoltage(0.f);
                outputs[CLOCK_OUTPUT + 2 * i + 1].setVoltage(0.f);
                lights[CLOCK_LIGHT + 2 * i].setBrightness(0.0f);
                lights[CLOCK_LIGHT + 2 * i + 1].setBrightness(0.0f);
            }
        }

        fillGlobal = static_cast<int>(std::roundf(params[FILL_KNOB].getValue() + (inputs[FILL_INPUT].isConnected() ? inputs[FILL_INPUT].getVoltage() * params[FILL_ATT].getValue() : 0.0f)));

        // Calculate the LCM for each clock
        int lcmWithMaster[9];
        for (int i = 1; i < 9; ++i) {
            int num = static_cast<int>(std::roundf(multiply[i]));
            int denom = static_cast<int>(std::roundf(divide[i]));
            simplifyRatio(num, denom);
            lcmWithMaster[i] = lcm(denom, 1); // Master clock is 1:1

            // Ensure lcmWithMaster[i] is not zero to prevent division by zero
            if (lcmWithMaster[i] == 0) {
                lcmWithMaster[i] = 1; // Default to 1 to avoid division by zero
            }
        }

        for (int i = 0; i < 9; i++) {
            ClockTimer[i].process(deltaTime);

            if (ratio[i] <= 0.f) {
                ratio[i] = 1.0f; // div by zero safety
            }

            if (i < 1){  //Swing clock reset logic
                if ( inputs[EXT_CLOCK_INPUT].isConnected() ) {
                      if (resetPulse){
                        swingCount++;
                        if (swingCount > 1.f){
                            SwingTimer.reset();
                            swingCount = 0;
                        }
                        resetPulse = false;
                    }
                } else {
					if ( ClockTimer[0].time >= (60.0f / (bpm ) ) ){
						swingCount++;
						if (swingCount > 1.f){
							SwingTimer.reset();
							swingCount = 0;
						}
					}
				}
			}

			if (ClockTimer[i].time >= (60.0f / (bpm * ratio[i]))) {
			
				ClockTimer[i].reset();
				
				if (i < 1) {  // Master clock reset point
					masterClockCycle++;
					// Rotate phases
					for (int k = 1; k < 9; k++) {
						int newIndex = (k + clockRotate) % 8;
						if (newIndex < 0) {
							newIndex += 8; // Adjust for negative values to wrap around correctly
						}
						tempPhases[newIndex + 1] = phases[k];
					}
					for (int k = 1; k < 9; k++) {
						phases[k] = tempPhases[k];
					}

					for (int j = 1; j < 9; j++) {
						if (masterClockCycle % lcmWithMaster[j] == 0) {
						   ClockTimer[j].reset();
						}

						if (resyncFlag[j]) {
						   ClockTimer[j].reset();
						   resyncFlag[j] = false;
						}

						int index = (clockRotate + j - 1) % 8;
						if (index < 0) {
						   index += 8; // Adjust for negative values to wrap around correctly
						}

						multiply[j] = std::roundf(params[MULTIPLY_KNOB_1 + index].getValue()) + (fill[j-1] ? fillGlobal : 0);
						divide[j] = std::roundf(params[DIVIDE_KNOB_1 + index].getValue());
						if (divide[j] <= 0) {
							divide[j] = 1.0f; // Now safe to use divide[j] for divisions
						}
						ratio[j] = multiply[j] / divide[j]; 

						if (fill[j] || ratio[j] != multiply[j] / divide[j]) {
							resyncFlag[j] = true;
						}
					}
				}
			}

            // Apply swing as a global adjustment to the phase increment
            if (bpm <= 0) bpm = 1.0f;  // Ensure bpm is positive and non-zero
            if (ratio[i] <= 0) ratio[i] = 1.0f;  // Ensure ratio is positive and non-zero

            float phaseDenominator = 60.0f / (bpm * ratio[i]);
            phases[i] = ClockTimer[i].time / phaseDenominator;  

            // Determine the output state based on pulse width
            bool highState = phases[i] < width;

            if (sequenceRunning) {
                if (phasorMode){

                    // Compute phase offset from pulse width input
                    float phase_offset = params[WIDTH_KNOB].getValue() + 
										 (inputs[WIDTH_INPUT].isConnected() ? 0.1f * inputs[WIDTH_INPUT].getVoltage() * params[WIDTH_ATT].getValue() : 0.0f);
                    phase_offset = clamp(phase_offset, 0.f, 1.0f);

                    // Calculate adjusted phase and use fmod for safe modulo operation
                    float adjusted_phase = fmod(phases[i] + phase_offset, 1.0f);

                    // Ensure adjusted_phase is within 0 to 1 range
                    if (adjusted_phase < 0.0f) {
                        adjusted_phase += 1.0f;
                    }

                    if (multiply[i]>0){
                        outputs[CLOCK_OUTPUT + 2 * i].setVoltage(phases[i]*10.f);
                        outputs[CLOCK_OUTPUT + 2 * i + 1].setVoltage( adjusted_phase*10.f );
                        lights[CLOCK_LIGHT + 2 * i].setBrightness(phases[i]);
                        lights[CLOCK_LIGHT + 2 * i + 1].setBrightness(1-phases[i]);
                    } else {
                        outputs[CLOCK_OUTPUT + 2 * i].setVoltage(0.f);
                        outputs[CLOCK_OUTPUT + 2 * i + 1].setVoltage(10.f);
                        lights[CLOCK_LIGHT + 2 * i].setBrightness(0.f);
                        lights[CLOCK_LIGHT + 2 * i + 1].setBrightness(10.f);
                    }
                } else {
                    if (multiply[i]>0){
                        outputs[CLOCK_OUTPUT + 2 * i].setVoltage(highState ? 10.0f : 0.0f);
                        outputs[CLOCK_OUTPUT + 2 * i + 1].setVoltage(highState ? 0.0f : 10.0f);
                        lights[CLOCK_LIGHT + 2 * i].setBrightness(highState ? 1.0f : 0.0f);
                        lights[CLOCK_LIGHT + 2 * i + 1].setBrightness(highState ? 0.0f : 1.0f);
                    } else {
                        outputs[CLOCK_OUTPUT + 2 * i].setVoltage( 0.0f);
                        outputs[CLOCK_OUTPUT + 2 * i + 1].setVoltage( 10.0f);
                        lights[CLOCK_LIGHT + 2 * i].setBrightness( 0.0f);
                        lights[CLOCK_LIGHT + 2 * i + 1].setBrightness( 1.0f);                
                    }

                }

            } else {
                outputs[CLOCK_OUTPUT + 2 * i].setVoltage(0.f);
                outputs[CLOCK_OUTPUT + 2 * i + 1].setVoltage(0.f);
                lights[CLOCK_LIGHT + 2 * i].setBrightness(0.0f);
                lights[CLOCK_LIGHT + 2 * i + 1].setBrightness(0.0f);
            }
        }

        if (deltaTime <= 0) {
            deltaTime = 1.0f / 48000.0f;  // Assume a default sample rate if deltaTime is zero to avoid division by zero
        }
    }
    
    int gcd(int a, int b) {
        while (b != 0) {
            int t = b;
            b = a % b;
            a = t;
        }
        return a; 
    }

    int lcm(int a, int b) {
        if (a == 0 || b == 0) {
            return 0; 
        }
        int gcd_value = gcd(a, b);
        return (gcd_value != 0) ? (a / gcd_value) * b : 0;
    }

    void simplifyRatio(int& numerator, int& denominator) {
        if (denominator == 0) {
            numerator = 0; 
            return;
        }
        int g = gcd(numerator, denominator);
        if (g != 0) { 
            numerator /= g;
            denominator /= g;
        }
    }
            
};

struct SyncroWidget : ModuleWidget {

    SyncroWidget(Syncro* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Syncro.svg"),
            asset::plugin(pluginInstance, "res/Syncro-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        addParam(createParamCentered<RoundBlackKnob>  (Vec(55,     80), module, Syncro::CLOCK_KNOB));
        addParam(createParamCentered<Trimpot>         (Vec(81.25,  80), module, Syncro::CLOCK_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(103.58, 80), module, Syncro::CLOCK_INPUT));

        // Add fill LEDs
        for (int i = 0; i < 8; i++) {
            addChild(createLight<SmallLight<RedLight>>(Vec(42 + i * 10, 120 ), module, Syncro::FILL_LIGHT_1 + i));
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
        addInput(createInputCentered<ThemedPJ301MPort>    (Vec(130,330), module, Syncro::RESET_INPUT));

        for (int i = 0; i < 8; i++) {
            // Add Multiply and Divide Knobs
            addParam(createParamCentered<Trimpot>             (Vec( 165, 35 + 38 + i * 38 ), module, Syncro::MULTIPLY_KNOB_1 + i));
            addParam(createParamCentered<Trimpot>             (Vec( 190, 35 + 38 + i * 38 ), module, Syncro::DIVIDE_KNOB_1 + i));

            addParam(createParamCentered<TL1105>              (Vec(280, 35 + 38 + i * 38 ), module, Syncro::FILL_BUTTON_1 + i));
            addChild(createLightCentered<SmallLight<YellowLight>>    (Vec(280, 35 + 38 + i * 38 ), module, Syncro::FILL_INDICATE_1 + i ));
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

        if (module) {
            // BPM Display Initialization
            module->bpmDisplay = createDigitalDisplay(Vec(27, 28), "120.0");
            addChild(module->bpmDisplay);

            // Swing Display Initialization
            module->swingDisplay = createDigitalDisplay(Vec(90, 28), "0.0%");
            addChild(module->swingDisplay);

            // Phasor Display Initialization
            module->phasorDisplay = createDigitalDisplay(Vec(230, 26), "");
            addChild(module->phasorDisplay);

            // Ratio Displays Initialization
            for (int i = 0; i < 8; i++) {
                module->ratioDisplays[i] = createDigitalDisplay(Vec(210, 65 + i * 38), "1:1");
                addChild(module->ratioDisplays[i]);
            }
        }
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
    
        // Cast the module to Syncro and check if the cast is successful
        Syncro* syncroModule = dynamic_cast<Syncro*>(module);
        if (!syncroModule) return;
    
        // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());
    
        // Phasor Mode enabled/disabled menu item
        struct PhasorEnabledItem : MenuItem {
            Syncro* syncroModule;
            void onAction(const event::Action& e) override {
                syncroModule->phasorMode = !syncroModule->phasorMode;
            }
            void step() override {
                rightText = syncroModule->phasorMode ? "✔" : "";
                MenuItem::step();
            }
        };
    
        PhasorEnabledItem* phasorItem = new PhasorEnabledItem();
        phasorItem->text = "Phasor Mode";
        phasorItem->syncroModule = syncroModule; 
        menu->addChild(phasorItem);
    
        // Clock CV as V/oct enabled/disabled menu item
        struct ClockCVAsVoctItem : MenuItem {
            Syncro* syncroModule;
            void onAction(const event::Action& e) override {
                syncroModule->clockCVAsVoct = !syncroModule->clockCVAsVoct;
                if (syncroModule->clockCVAsVoct) {
                    syncroModule->clockCVAsBPM = false;
                }
            }
            void step() override {
                rightText = syncroModule->clockCVAsVoct ? "✔" : "";
                MenuItem::step();
            }
        };
    
        ClockCVAsVoctItem* clockItem = new ClockCVAsVoctItem();
        clockItem->text = "Clock CV as V/oct";
        clockItem->syncroModule = syncroModule; 
        menu->addChild(clockItem);
    
        // Clock CV is 1V/10BPM enabled/disabled menu item
        struct ClockCVAsBPMItem : MenuItem {
            Syncro* syncroModule;
            void onAction(const event::Action& e) override {
                syncroModule->clockCVAsBPM = !syncroModule->clockCVAsBPM;
                if (syncroModule->clockCVAsBPM) {
                    syncroModule->clockCVAsVoct = false;
                }
            }
            void step() override {
                rightText = syncroModule->clockCVAsBPM ? "✔" : "";
                MenuItem::step();
            }
        };
    
        ClockCVAsBPMItem* bpmItem = new ClockCVAsBPMItem();
        bpmItem->text = "Clock CV is 1V/10BPM";
        bpmItem->syncroModule = syncroModule; 
        menu->addChild(bpmItem);
    
    }

    void draw(const DrawArgs& args) override {
        ModuleWidget::draw(args);
        Syncro* module = dynamic_cast<Syncro*>(this->module);
        if (!module) return;

        // Update BPM and Swing displays
        if (module->bpmDisplay) {
            char bpmText[16];
            if (module->clockCVAsVoct) {
                snprintf(bpmText, sizeof(bpmText), "▸%.1f", module->bpm);//symbol indicates v/oct mode
            } else {
                snprintf(bpmText, sizeof(bpmText), "%.1f", module->bpm);
            }
            module->bpmDisplay->text = bpmText;
        }

        if (module->swingDisplay) {
            char swingText[16];
            snprintf(swingText, sizeof(swingText), "%.1f%%", module->swing);
            module->swingDisplay->text = swingText;
        }

        if (module->phasorDisplay) {
            if (module->phasorMode){
                module->phasorDisplay->text = "Phasor Mode";
            } else {
                module->phasorDisplay->text = "";
            }
        }

        // Update ratio displays
        for (int i = 1; i < 9; i++) {
            int index = (module->clockRotate + i - 1) % 8;
            if (index < 0) {
                index += 8; // Adjust for negative values to wrap around correctly
            }

            module->disp_multiply[i] = round(module->params[Syncro::MULTIPLY_KNOB_1 + index].getValue()) + (module->fill[i-1] ? module->fillGlobal : 0);
            module->disp_divide[i] = round(module->params[Syncro::DIVIDE_KNOB_1 + index].getValue());
            if (module->ratioDisplays[i-1]) {
                char ratioText[16];
                snprintf(ratioText, sizeof(ratioText), "%d:%d", static_cast<int>(module->disp_multiply[i]), static_cast<int>(module->disp_divide[i]));
                if (index == 0) { // Check if the current index corresponds to the rotated position
                    module->ratioDisplays[i-1]->text = "▸" + std::string(ratioText);
                } else {
                    module->ratioDisplays[i-1]->text = ratioText;
                }
            }
        }

        for (int i = 0; i < 8; i++) {
            if (i < module->fillGlobal) {
                module->lights[Syncro::FILL_LIGHT_1 + i].setBrightness(1.0f);
            } else {
                module->lights[Syncro::FILL_LIGHT_1 + i].setBrightness(0.0f);
            }
            if (module->fill[i]) {
                module->lights[Syncro::FILL_INDICATE_1 + i].setBrightness(1.0f);
            } else {
                module->lights[Syncro::FILL_INDICATE_1 + i].setBrightness(0.0f);
            }
        }
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

Model* modelSyncro = createModel<Syncro, SyncroWidget>("Syncro");