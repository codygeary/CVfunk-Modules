#include "rack.hpp"
#include "plugin.hpp"
#include "digital_display.hpp"
using namespace rack;
#include <random>
#include <map>
#include <vector>

const int CHANNELS=8; 

struct Hammer : Module {
    enum ParamIds {
        //Clock Divider
        X1D_BUTTON, X1U_BUTTON, Y1D_BUTTON, Y1U_BUTTON,
        X2D_BUTTON, X2U_BUTTON, Y2D_BUTTON, Y2U_BUTTON,
        X3D_BUTTON, X3U_BUTTON, Y3D_BUTTON, Y3U_BUTTON,
        X4D_BUTTON, X4U_BUTTON, Y4D_BUTTON, Y4U_BUTTON,
        X5D_BUTTON, X5U_BUTTON, Y5D_BUTTON, Y5U_BUTTON,
        X6D_BUTTON, X6U_BUTTON, Y6D_BUTTON, Y6U_BUTTON,
        X7D_BUTTON, X7U_BUTTON, Y7D_BUTTON, Y7U_BUTTON,
        X8D_BUTTON, X8U_BUTTON, Y8D_BUTTON, Y8U_BUTTON,

        CLOCK_KNOB, CLOCK_ATT,
        ROTATE_KNOB, ROTATE_ATT,
        ON_OFF_BUTTON, RESET_BUTTON,  
        SWING_KNOB,        
        NUM_PARAMS
    };
    enum InputIds {
        CLOCK_INPUT,
        ROTATE_INPUT,      
        EXT_CLOCK_INPUT, ON_OFF_INPUT, RESET_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        CLOCK_OUTPUT, CLOCK_OUTPUT_1, CLOCK_OUTPUT_2, CLOCK_OUTPUT_3, 
        CLOCK_OUTPUT_4, CLOCK_OUTPUT_5,CLOCK_OUTPUT_6, CLOCK_OUTPUT_7, 
        CLOCK_OUTPUT_8, POLY_OUTPUT, CHAIN_OUTPUT,
        NUM_OUTS
    };
    enum LightIds {
        CLOCK_LIGHT, CLOCK_LIGHT_1, CLOCK_LIGHT_2, CLOCK_LIGHT_3, 
        CLOCK_LIGHT_4, CLOCK_LIGHT_5, CLOCK_LIGHT_6, CLOCK_LIGHT_7, 
        CLOCK_LIGHT_8, 
        ON_OFF_LIGHT, 
        NUM_LIGHTS
    };

    dsp::SchmittTrigger xDownTriggers[CHANNELS], xUpTriggers[CHANNELS], yDownTriggers[CHANNELS], yUpTriggers[CHANNELS];

    DigitalDisplay* phasorDisplay = nullptr;
    DigitalDisplay* bpmDisplay = nullptr;
    DigitalDisplay* swingDisplay = nullptr;
    DigitalDisplay* ratioDisplays[8] = {nullptr};

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
    int outputIndex[8] = {0,1,2,3,4,5,6,7};

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

    bool lastResetState = false;
    bool lastSequenceRunning = true;
    dsp::PulseGenerator chainReset, chainOnOff, clockPulse;

    int inputSkipper = 0;
    int inputSkipsTotal = 100; //only process button presses every 1/100 steps as it takes way too much CPU

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
    
        json_t* multiplyJ = json_array();
        for (int i = 0; i < 9; i++) {
            json_array_append_new(multiplyJ, json_real(multiply[i]));
        }
        json_object_set_new(rootJ, "multiply", multiplyJ);
    
        json_t* divideJ = json_array();
        for (int i = 0; i < 9; i++) {
            json_array_append_new(divideJ, json_real(divide[i]));
        }
        json_object_set_new(rootJ, "divide", divideJ);
   
        json_object_set_new(rootJ, "sequenceRunning", json_boolean(sequenceRunning));
        json_object_set_new(rootJ, "phasorMode", json_boolean(phasorMode));
        json_object_set_new(rootJ, "clockCVAsVoct", json_boolean(clockCVAsVoct));
        json_object_set_new(rootJ, "clockCVAsBPM", json_boolean(clockCVAsBPM));      
        return rootJ;
    }
    
    void dataFromJson(json_t* rootJ) override {
    
        json_t* multiplyJ = json_object_get(rootJ, "multiply");
        if (multiplyJ && json_is_array(multiplyJ)) {
            for (size_t i = 0; i < json_array_size(multiplyJ) && i < 9; i++) {
                json_t* val = json_array_get(multiplyJ, i);
                if (json_is_number(val)) {
                    multiply[i] = json_number_value(val);
                }
            }
        }
    
        // Load divide[8]
        json_t* divideJ = json_object_get(rootJ, "divide");
        if (divideJ && json_is_array(divideJ)) {
            for (size_t i = 0; i < json_array_size(divideJ) && i < 9; i++) {
                json_t* val = json_array_get(divideJ, i);
                if (json_is_number(val)) {
                    divide[i] = json_number_value(val);
                }
            }
        }

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

    Hammer() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTS, NUM_LIGHTS);

        configParam(X1D_BUTTON, 0.f, 1.f, 0.f, "X1 Down");
        configParam(X1U_BUTTON, 0.f, 1.f, 0.f, "X1 Up");
        configParam(Y1D_BUTTON, 0.f, 1.f, 0.f, "Y1 Down");
        configParam(Y1U_BUTTON, 0.f, 1.f, 0.f, "Y1 Up");
        configParam(X2D_BUTTON, 0.f, 1.f, 0.f, "X2 Down");
        configParam(X2U_BUTTON, 0.f, 1.f, 0.f, "X2 Up");
        configParam(Y2D_BUTTON, 0.f, 1.f, 0.f, "Y2 Down");
        configParam(Y2U_BUTTON, 0.f, 1.f, 0.f, "Y2 Up");
        configParam(X3D_BUTTON, 0.f, 1.f, 0.f, "X3 Down");
        configParam(X3U_BUTTON, 0.f, 1.f, 0.f, "X3 Up");
        configParam(Y3D_BUTTON, 0.f, 1.f, 0.f, "Y3 Down");
        configParam(Y3U_BUTTON, 0.f, 1.f, 0.f, "Y3 Up");
        configParam(X4D_BUTTON, 0.f, 1.f, 0.f, "X4 Down");
        configParam(X4U_BUTTON, 0.f, 1.f, 0.f, "X4 Up");
        configParam(Y4D_BUTTON, 0.f, 1.f, 0.f, "Y4 Down");
        configParam(Y4U_BUTTON, 0.f, 1.f, 0.f, "Y4 Up");
        configParam(X5D_BUTTON, 0.f, 1.f, 0.f, "X5 Down");
        configParam(X5U_BUTTON, 0.f, 1.f, 0.f, "X5 Up");
        configParam(Y5D_BUTTON, 0.f, 1.f, 0.f, "Y5 Down");
        configParam(Y5U_BUTTON, 0.f, 1.f, 0.f, "Y5 Up");
        configParam(X6D_BUTTON, 0.f, 1.f, 0.f, "X6 Down");
        configParam(X6U_BUTTON, 0.f, 1.f, 0.f, "X6 Up");
        configParam(Y6D_BUTTON, 0.f, 1.f, 0.f, "Y6 Down");
        configParam(Y6U_BUTTON, 0.f, 1.f, 0.f, "Y6 Up");
        configParam(X7D_BUTTON, 0.f, 1.f, 0.f, "X7 Down");
        configParam(X7U_BUTTON, 0.f, 1.f, 0.f, "X7 Up");
        configParam(Y7D_BUTTON, 0.f, 1.f, 0.f, "Y7 Down");
        configParam(Y7U_BUTTON, 0.f, 1.f, 0.f, "Y7 Up");
        configParam(X8D_BUTTON, 0.f, 1.f, 0.f, "X8 Down");
        configParam(X8U_BUTTON, 0.f, 1.f, 0.f, "X8 Up");
        configParam(Y8D_BUTTON, 0.f, 1.f, 0.f, "Y8 Down");
        configParam(Y8U_BUTTON, 0.f, 1.f, 0.f, "Y8 Up");

        // Configure parameters
        configParam(CLOCK_KNOB, 0.000001f, 480.0f, 120.0f, "Clock Rate", " BPM");
        configParam(CLOCK_ATT, -1.f, 1.f, 0.0f, "Clock Attenuvertor");
        configParam(ROTATE_KNOB, -1.0f, 1.0f, 0.0f, "Rotate");
        configParam(ROTATE_ATT, -1.0f, 1.0f, 0.0f, "Rotate Atenuvertor");
        configParam(SWING_KNOB, -99.0f, 99.0f, 0.0f, "Swing", " %");

        // Configure inputs and outputs
        configInput(EXT_CLOCK_INPUT, "External Clock");
        configInput(RESET_INPUT, "Reset");
        configInput(ON_OFF_INPUT, "ON/OFF");
        configOutput(CLOCK_OUTPUT, "Main Clock");
        configOutput(CLOCK_OUTPUT_1, "Clock 1");
        configOutput(CLOCK_OUTPUT_2, "Clock 2");
        configOutput(CLOCK_OUTPUT_3, "Clock 3");
        configOutput(CLOCK_OUTPUT_4, "Clock 4");
        configOutput(CLOCK_OUTPUT_5, "Clock 5");
        configOutput(CLOCK_OUTPUT_6, "Clock 6");
        configOutput(CLOCK_OUTPUT_7, "Clock 7");
        configOutput(CLOCK_OUTPUT_8, "Clock 8");
        configOutput(POLY_OUTPUT, "Poly Clock Out \n Ch 1-8 Clock Gate Outs \n Ch 9-16 Inverted Gate Outs");
        configOutput(CHAIN_OUTPUT, "Chains to the EXT CLOCK of other Hammers to control them.");

        configInput(CLOCK_INPUT , "Clock" );
        configInput(ROTATE_INPUT , "Rotation" );
        configParam(RESET_BUTTON, 0.0, 1.0, 0.0, "Reset" );
        configParam(ON_OFF_BUTTON, 0.0, 1.0, 0.0, "On / Off " );        
    }

    void process(const ProcessArgs& args) override {
        swing = params[SWING_KNOB].getValue();
        swing = clamp(swing, -99.f, 99.f);

        float rotate = params[ROTATE_KNOB].getValue() + (inputs[ROTATE_INPUT].isConnected() ? 0.1f*inputs[ROTATE_INPUT].getVoltage() * params[ROTATE_ATT].getValue() : 0.0f);
        clockRotate = static_cast<int>(std::roundf(fmod(-8.0f * rotate, 8.0f)));

        bool resetCondition = false;
        // Process clock sync input
        if (inputs[EXT_CLOCK_INPUT].isConnected()) {
            float SyncInputVoltage = inputs[EXT_CLOCK_INPUT].getVoltage();

            if (abs(SyncInputVoltage - 10.42f) < 0.1f) {
                resetCondition = true;
            } else { resetCondition = false;}
            

            if (SyncTrigger.process(SyncInputVoltage)) {
                // Check for special control voltages first
                if (abs(SyncInputVoltage - 10.82f) < 0.1f) {
                    // Toggle On/Off signal received from chain
                    sequenceRunning = !sequenceRunning;
                    return; // Don't process as normal clock
                }
                
                // Normal clock processing
                if (!firstClockPulse) {
                     SyncInterval = SyncTimer.time; // Get the accumulated time since the last reset
                }
//                 clockPulse.trigger(0.001f);
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

        // Calculate the Nyquist frequency and the maximum BPM considering a 128x multiplier
        float maxBPM = (args.sampleRate * 60.0f) / 256.0f;

        // Clamp BPM based on the Nyquist frequency
        bpm = clamp(bpm, 0.000001f, maxBPM);

        // Check for reset input or reset button
        if (!resetCondition) resetCondition = (inputs[RESET_INPUT].isConnected() && resetTrigger.process(inputs[RESET_INPUT].getVoltage())) || (params[RESET_BUTTON].getValue() > 0.1f);

        if (resetCondition) {
            for (int i = 0; i < 9; i++) {
                ClockTimer[i].reset();
                outputs[CLOCK_OUTPUT + i].setVoltage(0.f);
                lights[CLOCK_LIGHT + i].setBrightness(0.0f);
            }
        }

        inputSkipper++;
        if (inputSkipper > inputSkipsTotal){
            // Process Ratio Buttons
            for (int i = 0; i < CHANNELS; i++) { 
                if (xDownTriggers[i].process(params[X1D_BUTTON + i * 4].getValue())) { multiply[i+1] -= 1.f; resyncFlag[i+1]=true;}
                if (xUpTriggers[i].process(params[X1U_BUTTON + i * 4].getValue()))   { multiply[i+1] += 1.f; resyncFlag[i+1]=true;}
                if (yDownTriggers[i].process(params[Y1D_BUTTON + i * 4].getValue())) { divide[i+1] -= 1.f; resyncFlag[i+1]=true;}
                if (yUpTriggers[i].process(params[Y1U_BUTTON + i * 4].getValue()))   { divide[i+1] += 1.f; resyncFlag[i+1]=true;}
                multiply[i+1] = clamp(multiply[i+1],0.0f, 99.0f);
                divide[i+1] = clamp(divide[i+1],1.0f, 99.0f); 
                ratio[i+1] = multiply[i+1] / divide[i+1];  
            }
            inputSkipper = 0;
        }
   
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

// 						ratio[j] = multiply[index] / divide[index]; 

					}
				}
			}

            if (bpm <= 0) bpm = 1.0f;  // Ensure bpm is positive and non-zero
            if (ratio[i] <= 0) ratio[i] = 1.0f;  // Ensure ratio is positive and non-zero

            float phaseDenominator = 60.0f / (bpm * ratio[i]);
            phases[i] = ClockTimer[i].time / phaseDenominator;  

            // Determine the output state based on pulse width
            bool highState = phases[i] < 0.5f;

            int rot = (clockRotate + i - 1) % 8;
            if (rot < 0) {
               rot += 8; // Adjust for negative values to wrap around correctly
            }

            if (sequenceRunning) {
                if (phasorMode){

                    // Compute phase offset from pulse width input
                    float phase_offset = 0.5f;
                    phase_offset = clamp(phase_offset, 0.f, 1.0f);

                    // Calculate adjusted phase and use fmod for safe modulo operation
                    float adjusted_phase = fmod(phases[rot+1] + phase_offset, 1.0f);

                    // Ensure adjusted_phase is within 0 to 1 range
                    if (adjusted_phase < 0.0f) {
                        adjusted_phase += 1.0f;
                    }

                    if (multiply[rot+1]>0){
                        outputs[CLOCK_OUTPUT + i].setVoltage(phases[i]*10.f);
                        lights[CLOCK_LIGHT + i].setBrightness(phases[i]);
                    } else {
                        outputs[CLOCK_OUTPUT + i].setVoltage(0.f);
                        lights[CLOCK_LIGHT + i].setBrightness(0.f);
                    }
                } else {
                    if (multiply[rot+1]>0){
                        outputs[CLOCK_OUTPUT + i].setVoltage((highState) ? 10.0f : 0.0f);
                        lights[CLOCK_LIGHT + i].setBrightness((highState) ? 1.0f : 0.0f);
                    } else {
                        outputs[CLOCK_OUTPUT + i].setVoltage( 0.0f);
                        lights[CLOCK_LIGHT + i].setBrightness( 0.0f);
                    }

                }

            } else {
                outputs[CLOCK_OUTPUT + i].setVoltage(0.f);
                lights[CLOCK_LIGHT + i].setBrightness(0.0f);
            }
        }


//         // Generate 16-channel poly output
//         // Channels 1-8: Clock outputs (rotated)
//         // Channels 9-16: Inverted clock outputs (rotated)
//         if (outputs[POLY_OUTPUT].isConnected()) {
//             outputs[POLY_OUTPUT].setChannels(16);
//             
//             for (int i = 1; i < 9; i++) { // Process channels 1-8
//                 // Calculate which physical output this logical channel uses
//                 int outputIndex = i;
//                 if (i > 0) { // Don't rotate the master clock (channel 0)
//                     outputIndex = ((i - 1 - clockRotate + 8) % 8) + 1; // Rotate channels 1-8 (fixed direction and wrapping)
//                 }
//                 
//                 // Get the voltage from the corresponding physical output
//                 float voltage = outputs[CLOCK_OUTPUT + outputIndex].getVoltage();
//                 
//                 // Set poly channels: 0-7 for normal, 8-15 for inverted
//                 outputs[POLY_OUTPUT].setVoltage(voltage, i - 1);           // Channels 0-7
//                 outputs[POLY_OUTPUT].setVoltage(voltage > 5.0f ? 0.0f : 10.0f, i + 7); // Channels 8-15 (inverted)
//             }
//         }  

        if (outputs[CHAIN_OUTPUT].isConnected()) {
            
            // Check for reset condition change
            bool currentResetState = resetCondition;
            if (currentResetState && !lastResetState) {
                // Reset just triggered - send special voltage for 10ms
                chainReset.trigger(0.001f);
            }
            // Check for on/off state change  
            else if (sequenceRunning != lastSequenceRunning) {
                // On/Off state changed - send special voltage for 10ms
                chainOnOff.trigger(0.001f);
            }
            lastResetState = currentResetState;
            lastSequenceRunning = sequenceRunning;
        }

        //Chain Outputs
        bool ResetActive = params[RESET_BUTTON].getValue()>0.1f;
        bool OnOffActive = chainOnOff.process(args.sampleTime);
        bool ClockPulseActive = clockPulse.process(args.sampleTime);
 
        outputs[CHAIN_OUTPUT].setVoltage(0.0f);
        if (ClockPulseActive) outputs[CHAIN_OUTPUT].setVoltage(5.0f);
        if (ResetActive) outputs[CHAIN_OUTPUT].setVoltage(10.42f);
        if (OnOffActive) outputs[CHAIN_OUTPUT].setVoltage(10.82f);
       
        
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

struct HammerWidget : ModuleWidget {

    HammerWidget(Hammer* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Hammer.svg"),
            asset::plugin(pluginInstance, "res/Hammer-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // Clock Divider Increments
        float xoffset = 5.5f;
        float yoffset = -17.f;
        
        // X and Y positions for buttons (relative, same for all rows)
        constexpr float x1d = 14.974f;
        constexpr float x1u = 21.452f;
        constexpr float y1d = 43.533f;
        constexpr float y1u = 50.011f;
        constexpr float outputX = 66.f;
        
        // Y positions for each row (just the vertical positions of each row group)
        float yPositions[] = {
            49.329f, 59.482f, 69.739f, 80.011f, 90.319f, 100.583f, 110.85f, 121.117f
        };
        
        for (int i = 0; i < 8; i++) {
            float y = yPositions[i] + yoffset;        
            addParam(createParamCentered<TL1105>(mm2px(Vec(x1d + xoffset, y)), module, Hammer::X1D_BUTTON + i * 4));
            addParam(createParamCentered<TL1105>(mm2px(Vec(x1u + xoffset, y)), module, Hammer::X1U_BUTTON + i * 4));
            addParam(createParamCentered<TL1105>(mm2px(Vec(y1d + xoffset, y)), module, Hammer::Y1D_BUTTON + i * 4));
            addParam(createParamCentered<TL1105>(mm2px(Vec(y1u + xoffset, y)), module, Hammer::Y1U_BUTTON + i * 4));
            addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(outputX, y)), module, Hammer::CLOCK_OUTPUT_1 + i));
            addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(outputX-4, y-4)), module, Hammer::CLOCK_LIGHT_1 + i));
            
        }

        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(9, 32.4)), module, Hammer::CLOCK_OUTPUT)); //Main Clock
        addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(9-4, 32.4-4)), module, Hammer::CLOCK_LIGHT)); //Main Clock light

        addParam(createParamCentered<TL1105>(mm2px(Vec(9,  92)), module, Hammer::ON_OFF_BUTTON));      
        addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(9,  92)), module, Hammer::ON_OFF_LIGHT ));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(9,  100)), module, Hammer::ON_OFF_INPUT));

        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(9,  115)), module, Hammer::EXT_CLOCK_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(21,  115)), module, Hammer::RESET_INPUT));
        addParam(createParamCentered<TL1105>(mm2px(Vec(27.5,  115)), module, Hammer::RESET_BUTTON));

        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(52, 115)), module, Hammer::CHAIN_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(66, 115)), module, Hammer::POLY_OUTPUT));

        addParam(createParamCentered<RoundBlackKnob>  (Vec(140 ,   42), module, Hammer::CLOCK_KNOB));
        addParam(createParamCentered<Trimpot>         (Vec(165.25, 42), module, Hammer::CLOCK_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(188.58, 42), module, Hammer::CLOCK_INPUT));

        addParam(createParamCentered<RoundBlackKnob>  (Vec(27 ,142), module, Hammer::SWING_KNOB));

        addParam(createParamCentered<RoundBlackKnob>  (Vec(27 ,195), module, Hammer::ROTATE_KNOB));
        addParam(createParamCentered<Trimpot>         (Vec(27, 195+25.25), module, Hammer::ROTATE_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(27, 195 + 48.58), module, Hammer::ROTATE_INPUT));

        if (module){
            // BPM Display Initialization
            module->bpmDisplay = createDigitalDisplay(Vec(20, 40), "120.0");
            addChild(module->bpmDisplay);

            // Ratio Displays Initialization
            for (int i = 0; i < CHANNELS; i++) {
                module->ratioDisplays[i] = createDigitalDisplay(mm2px(Vec(24.f+xoffset, 46.365f + (float)i * 10.386f + yoffset)), "1:1");
                addChild(module->ratioDisplays[i]);
            }
        }
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
    
        // Cast the module to Hammer and check if the cast is successful
        Hammer* syncroModule = dynamic_cast<Hammer*>(module);
        if (!syncroModule) return;
    
        // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());
    
        // Phasor Mode enabled/disabled menu item
        struct PhasorEnabledItem : MenuItem {
            Hammer* syncroModule;
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
            Hammer* syncroModule;
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
            Hammer* syncroModule;
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
        Hammer* module = dynamic_cast<Hammer*>(this->module);
        if (!module) return;


        // Update ratio displays
        for (int i = 1; i < 9; i++) {
            int index = (module->clockRotate + i - 1) % 8;
            if (index < 0) {
                index += 8; // Adjust for negative values to wrap around correctly
            }

            module->disp_multiply[i] = module->multiply[index+1];
            module->disp_divide[i] = module->divide[index+1];
            if (module->ratioDisplays[i-1]) {
                char ratioText[16];
                snprintf(ratioText, sizeof(ratioText), "%d:%d", static_cast<int>(module->disp_multiply[i]), static_cast<int>(module->disp_divide[i]));
                if (index == 0) { // Check if the current index corresponds to the rotated position
                    if (module->disp_multiply[i]>0){
                        module->ratioDisplays[i-1]->text = "▸" + std::string(ratioText);
                    } else {
                        module->ratioDisplays[i-1]->text = "off";
                    }


                } else {
                    if (module->disp_multiply[i]>0){
                        module->ratioDisplays[i-1]->text = std::string(ratioText);
                    } else {
                        module->ratioDisplays[i-1]->text = "off";
                    }
                }
            }
        }

        // Update BPM display
        if (module->bpmDisplay) {
            char bpmText[16];
            if (module->clockCVAsVoct) {
                snprintf(bpmText, sizeof(bpmText), "▸%.1f", module->bpm);//symbol indicates v/oct mode
            } else {
                snprintf(bpmText, sizeof(bpmText), "%.1f", module->bpm);
            }
            module->bpmDisplay->text = bpmText;
        }

        if (module->phasorDisplay) {
            if (module->phasorMode){
                module->phasorDisplay->text = "Phasor Mode";
            } else {
                module->phasorDisplay->text = "";
            }
        }
        
        for (int i=0; i<8; i++){
            if (module->sequenceRunning) {   
                if (module->phasorMode){    
                    if (module->multiply[i]>0){
                        module->lights[Hammer::CLOCK_LIGHT + module->outputIndex[i]].setBrightness(module->phases[i]);
                    } else {
                        module->lights[Hammer::CLOCK_LIGHT + module->outputIndex[i]].setBrightness(0.f);
                    }
                } else {
                    if (module->multiply[i]>0){
                        module->lights[Hammer::CLOCK_LIGHT + module->outputIndex[i]].setBrightness( (module->phases[i]>0.5f) ? 1.0f : 0.0f);
                    } else {
                        module->lights[Hammer::CLOCK_LIGHT + module->outputIndex[i]].setBrightness( 0.0f);
                    }                    
                }
              
            } else { //sequence is off
                module->lights[Hammer::CLOCK_LIGHT + i].setBrightness(0.0f);
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

Model* modelHammer = createModel<Hammer, HammerWidget>("Hammer");