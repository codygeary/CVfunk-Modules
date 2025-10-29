////////////////////////////////////////////////////////////
//
//   Hammer
//
//   written by Cody Geary
//   Copyright 2025, MIT License
//
//   Eight-channel rotating clock divider and multipler with optional phase outputs.
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
#include "digital_display.hpp"
using namespace rack;
#include <random>
#include <map>
#include <vector>

const int CHANNELS=8;  //8 Clock Rotation channels plus 1 Main clock 

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

    dsp::Timer SyncTimer;
    dsp::Timer SwingTimer;
    dsp::Timer ClockTimer[CHANNELS+1];  // Array to store timers for each clock

    dsp::SchmittTrigger SyncTrigger;
    dsp::SchmittTrigger resetTrigger;
    dsp::SchmittTrigger onOffTrigger;
    dsp::SchmittTrigger onOffButtonTrigger;

    float SwingPhase = 0.0f;
    float lastClockTime = -1.0f;
    float bpm = 120.0f;
    float phase = 0.0f;
    float multiply[CHANNELS+1] = {1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f};
    float divide[CHANNELS+1] = {1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f};
    float ratio[CHANNELS+1] = {1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f};
    float disp_multiply[CHANNELS+1] = {1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f};
    float disp_divide[CHANNELS+1] = {1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f};
    float SyncInterval = 1.0f;
    float PrevSyncInterval = 1.0f;
    float clockRate = 120.0f;
    float phases[CHANNELS+1] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};  // Array to store phases for each clock
    float tempPhases[CHANNELS+1] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    float swing = 0.f;
    int outputIndex[CHANNELS] = {0,1,2,3,4,5,6,7}; //For clock rotation function

    int clockRotate = 0;
    int swingCount = 0;
    int fillGlobal = 0;
    int masterClockCycle = 0;

    //for sample-based clocking
    uint64_t masterSampleCounter = 0; // counts audio samples between master ticks
    uint64_t processSampleCounter = 0; // counts process cycles
    uint64_t masterClockLength = (uint64_t)std::round(APP->engine->getSampleRate());
    uint64_t swingOffsetSamples = 0;
    double masterClockError = 0.0; // accumulates fractional sample drift


    bool resyncFlag[CHANNELS+1] = {false,false,false,false,false,false,false,false,false};
    bool firstClockPulse = true;
    bool sequenceRunning = true;
    bool phasorMode = false;
    bool clockCVAsVoct = false;
    bool clockCVAsBPM = true;
    bool resetPulse = false;
    bool syncPoint = false;

    bool onOffCondition = false;
    bool resetCondition = false;
    bool remoteOFF = false;
    bool remoteON = false;

    bool lastResetState = false;
    bool lastSequenceRunning = true;
    dsp::PulseGenerator chainReset, chainON, chainOFF, clockPulse, resetPulseGen;

    int inputSkipper = 0;
    int inputSkipsTotal = 100; //only process button presses every 1/100 steps as it takes way too much CPU

    json_t* dataToJson() override {
        json_t* rootJ = json_object();

        json_t* multiplyJ = json_array();
        for (int i = 0; i < (CHANNELS+1); i++) {
            json_array_append_new(multiplyJ, json_real(multiply[i]));
        }
        json_object_set_new(rootJ, "multiply", multiplyJ);

        json_t* divideJ = json_array();
        for (int i = 0; i < (CHANNELS+1); i++) {
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
            for (size_t i = 0; i < json_array_size(multiplyJ) && i < (CHANNELS+1); i++) {
                json_t* val = json_array_get(multiplyJ, i);
                if (json_is_number(val)) {
                    multiply[i] = json_number_value(val);
                }
            }
        }

        // Load divide[N]
        json_t* divideJ = json_object_get(rootJ, "divide");
        if (divideJ && json_is_array(divideJ)) {
            for (size_t i = 0; i < json_array_size(divideJ) && i < (CHANNELS+1); i++) {
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

        for (int i=0; i<CHANNELS; i++){
            configParam(X1D_BUTTON+4*i, 0.f, 1.f, 0.f, "X Down Ch." + std::to_string(i+1) );
            configParam(X1U_BUTTON+4*i, 0.f, 1.f, 0.f, "X Up Ch." + std::to_string(i+1) );
            configParam(Y1D_BUTTON+4*i, 0.f, 1.f, 0.f, "Y Down Ch." + std::to_string(i+1) );
            configParam(Y1U_BUTTON+4*i, 0.f, 1.f, 0.f, "Y Up Ch." + std::to_string(i+1) );
        }
        
        // Configure parameters
        configParam(CLOCK_KNOB, 0.000001f, 480.0f, 120.0f, "Clock Rate", " BPM");
        configParam(CLOCK_ATT, -1.f, 1.f, 0.0f, "Clock Attenuverter");
        configParam(ROTATE_KNOB, -1.0f, 1.0f, 0.0f, "Rotate");
        configParam(ROTATE_ATT, -1.0f, 1.0f, 0.0f, "Rotate Atenuverter");
        configParam(SWING_KNOB, -99.0f, 99.0f, 0.0f, "Swing", " %");

#ifdef METAMODULE
        configInput(EXT_CLOCK_INPUT, "Ext. Clock Input");
#else
        configInput(EXT_CLOCK_INPUT, "Ext. Clock Input \n (Also accepts CHAIN from Hammer.) \n");
#endif
        configInput(RESET_INPUT, "Reset");
        configInput(ON_OFF_INPUT, "ON/OFF");
        configOutput(CLOCK_OUTPUT, "Main Clock");
        for (int i=0; i<CHANNELS; i++){        
            configOutput(CLOCK_OUTPUT_1+i, "Clock " + std::to_string(i+1) );
        }
#ifdef METAMODULE
        configOutput(POLY_OUTPUT, "Poly Clock Out");
        configOutput(CHAIN_OUTPUT, "Chain");
#else
        configOutput(POLY_OUTPUT, "Poly Clock Out \n Ch 1-8 Clock Gate Outs \n Ch 9-16 Inverted Gate Outs");
        configOutput(CHAIN_OUTPUT, "(CHAIN links to CLOCK input of Hammer or Picus.)\n Chain");
#endif

        configInput(CLOCK_INPUT, "Clock" );
        configInput(ROTATE_INPUT , "Rotation" );
        configParam(RESET_BUTTON, 0.0, 1.0, 0.0, "Reset" );
        configParam(ON_OFF_BUTTON, 0.0, 1.0, 0.0, "On / Off " );
    }

    void onReset(const ResetEvent& e) override {
        // Reset all parameters
        Module::onReset(e);

        // Reset custom state variables
        for (int i = 1; i < (CHANNELS+1); ++i) {
            multiply[i] = 1.0f;
            divide[i] = 1.0f;
        }
    }

    void onRandomize(const RandomizeEvent& e) override {
        // Randomize custom state variables (integers to match UI steps)
        for (int i = 1; i < (CHANNELS+1); ++i) {
            // multiply 0..12 inclusive (integer)
            multiply[i] = std::floor(random::uniform() * 13.0f);
            // divide 1..9 inclusive (integer)
            divide[i] = 1 + static_cast<int>(std::floor(random::uniform() * 9.0f));
            ratio[i] = multiply[i] / divide[i];
            resyncFlag[i] = true;
        }
    }

    void process(const ProcessArgs& args) override {

        masterSampleCounter++;
        processSampleCounter++;
        double deltaTime = args.sampleTime;

        // Process timers
        SyncTimer.process(deltaTime);
        SwingTimer.process(deltaTime);

        // Process Swing and Rotate Inputs
        swing = params[SWING_KNOB].getValue();

        float rotate = params[ROTATE_KNOB].getValue()+ (
                    inputs[ROTATE_INPUT].isConnected() 
                    ? 0.1f*inputs[ROTATE_INPUT].getVoltage() * params[ROTATE_ATT].getValue() 
                    : 0.0f );
        float rotationRange = static_cast<float>(CHANNELS); //float version of CHANNELS for use in rotation
        clockRotate = static_cast<int>(std::roundf(fmod(-rotationRange * rotate, rotationRange)));

        // Flags for resetting and on/off switch
        onOffCondition = false;
        resetCondition = false;
        syncPoint = false;

        // Process clock sync input
        if (inputs[EXT_CLOCK_INPUT].isConnected()) {
            float SyncInputVoltage = inputs[EXT_CLOCK_INPUT].getVoltage();

            if (abs(SyncInputVoltage - 10.42f) < 0.1f) { //RESET VOLTAGE for CVfunk Chain function
                resetCondition = true;
                resetPulseGen.trigger(args.sampleTime);  //1 sample Trigger Pulse for Chain Connection                
            } else { resetCondition = false;}

            if (SyncTrigger.process(SyncInputVoltage-0.1f)) {
                syncPoint = true;
                
                // Check for special control voltages first
                if (abs(SyncInputVoltage - 10.69f) < 0.1f) {  //ON VOLTAGE for CVfunk Chain function
                    // Toggle On/Off signal received from chain
                    remoteON = true;
                    return; // Don't process as normal clock
                }
                if (abs(SyncInputVoltage - 10.86f) < 0.1f) {  //OFF VOLTAGE for CVfunk Chain function
                    // Toggle On/Off signal received from chain
                    remoteOFF = true;
                    return; // Don't process as normal clock
                }

                if (!firstClockPulse) {
                    PrevSyncInterval = SyncInterval;  //save interval before overwriting it
                    SyncInterval = SyncTimer.time; // Get the accumulated time since the last reset

                    if (masterSampleCounter > 0) {
                        // args.sampleRate * 60 / samples == BPM
                        bpm = static_cast<float>((args.sampleRate * 60.0) / static_cast<double>(masterSampleCounter));
                        masterClockLength = masterSampleCounter;
                    }
                }
                masterSampleCounter = 0; // restart counting for next tick

                SyncTimer.reset(); // Reset the timer for the next trigger interval measurement
                
                clockPulse.trigger(args.sampleTime);  //Trigger Pulse for Chain Connection
                phases[0] = 0.0f;
                ClockTimer[0].reset();
                resetPulse = true;
                firstClockPulse = false;
            }

        } else { // Internally clock instead
            if (clockCVAsVoct && inputs[CLOCK_INPUT].isConnected()) {
                float input_v_oct = inputs[CLOCK_INPUT].getVoltage();
                bpm = 120.f * powf(2.f, input_v_oct);
            } else {
                bpm = params[CLOCK_KNOB].getValue() +
                      (inputs[CLOCK_INPUT].isConnected()
                           ? 10.f * inputs[CLOCK_INPUT].getVoltage() * params[CLOCK_ATT].getValue()
                           : 0.0f);
            
                // Compute ideal samples per cycle as double for precision
                double exactSamples = (args.sampleRate * 60.0) / static_cast<double>(bpm);
                double integerPart;
                double fractionalPart = modf(exactSamples, &integerPart);
            
                // Accumulate fractional error
                masterClockError += fractionalPart;
            
                // Adjust integer part when enough fractional error has built up
                if (masterClockError >= 1.0) {
                    integerPart += 1.0;
                    masterClockError -= 1.0;
                } else if (masterClockError <= -1.0) {
                    integerPart -= 1.0;
                    masterClockError += 1.0;
                }
            
                masterClockLength = static_cast<uint64_t>(integerPart);
            } 
        }

        // Compute swing
        SwingPhase = SwingTimer.time / (120.0 / bpm);
        if (swing != 0.f) deltaTime *= (1.0 + (swing / 100.0) * cos(2.0 * M_PI * SwingPhase));
        if (swing != 0.f) swingOffsetSamples = (swing/100.0) * masterClockLength * 0.5f * cos(2.0 * M_PI * SwingPhase);

        // Check for on/off input or on/off button
        if (inputs[ON_OFF_INPUT].isConnected()) {
            onOffCondition = onOffTrigger.process(inputs[ON_OFF_INPUT].getVoltage()) || onOffButtonTrigger.process(params[ON_OFF_BUTTON].getValue() ) ;
        } else {
            onOffCondition = onOffButtonTrigger.process(params[ON_OFF_BUTTON].getValue());
        }

        // Process ON/OFF Toggling
        if (remoteON || remoteOFF || onOffCondition) {
            sequenceRunning = !sequenceRunning; // Toggle sequenceRunning

            if (remoteON) sequenceRunning = true; //remote directly controls instead of toggle
            if (remoteOFF) sequenceRunning = false;
            
            remoteON = false;
            remoteOFF = false; //reset tokens

            if (sequenceRunning){ //sequencer just started again
                for (int i = 0; i < (CHANNELS+1); i++) {
                    ClockTimer[i].reset();
                    phases[i] = 0.f; // reset clock phases
                }
                SyncTimer.reset();
                SwingTimer.reset();
                swingCount = 0;
                masterClockCycle = 0; //reset LCM alignment
                firstClockPulse = true;
                SyncInterval = PrevSyncInterval; //Recall previous sync interval to avoid using a bad starting timing
            }
        }

        // Check for reset input or reset button
        if (!resetCondition) resetCondition = (inputs[RESET_INPUT].isConnected() && resetTrigger.process(inputs[RESET_INPUT].getVoltage())) || (params[RESET_BUTTON].getValue() > 0.1f);

        if (resetCondition) {
            for (int i = 0; i < (CHANNELS+1); i++) {
                ClockTimer[i].reset();
                outputs[CLOCK_OUTPUT + i].setVoltage(0.f);
                lights[CLOCK_LIGHT + i].setBrightness(0.0f);
                phases[i] = 0.f; // reset clock phases
            }
            SwingTimer.reset();
            swingCount = 0;
            masterClockCycle = 0; //reset LCM alignment
            firstClockPulse = true;
            SyncInterval = PrevSyncInterval;
        }

        inputSkipper++;
        if (inputSkipper > inputSkipsTotal){ //Process button inputs infrequently to reduce CPU load. 
            // Process Ratio Buttons
            for (int i = 0; i < CHANNELS; i++) {

                int srcIndex;
                // ensure rotation wraps cleanly in both directions
                int tmp = (clockRotate + i ) % CHANNELS; // tmp in 0..CHANNELS-1
                if (tmp < 0) tmp += CHANNELS;                 // handle negative rotation
                srcIndex = tmp + 1;                           // convert to 1..CHANNELS

                if (xDownTriggers[i].process(params[X1D_BUTTON + i * 4].getValue())) { multiply[srcIndex] -= 1.f; resyncFlag[srcIndex]=true;}
                if (xUpTriggers[i].process(params[X1U_BUTTON + i * 4].getValue()))   { multiply[srcIndex] += 1.f; resyncFlag[srcIndex]=true;}
                if (yDownTriggers[i].process(params[Y1D_BUTTON + i * 4].getValue())) { divide[srcIndex] -= 1.f; resyncFlag[srcIndex]=true;}
                if (yUpTriggers[i].process(params[Y1U_BUTTON + i * 4].getValue()))   { divide[srcIndex] += 1.f; resyncFlag[srcIndex]=true;}
                multiply[srcIndex] = clamp(multiply[srcIndex],0.0f, 99.0f);
                divide[srcIndex] = clamp(divide[srcIndex],1.0f, 99.0f);
                ratio[srcIndex] = multiply[srcIndex] / divide[srcIndex];
            }
            inputSkipper = 0;
        }

        // Calculate the LCM for each clock
        int lcmWithMaster[(CHANNELS+1)];
        for (int i = 1; i < (CHANNELS+1); ++i) {
            int num = static_cast<int>(std::roundf(multiply[i]));
            int denom = static_cast<int>(std::roundf(divide[i]));
            simplifyRatio(num, denom);
            lcmWithMaster[i] = lcm(denom, 1); // Master clock is 1:1

            // Ensure lcmWithMaster[i] is not zero to prevent division by zero
            if (lcmWithMaster[i] == 0) {
                lcmWithMaster[i] = 1; // Default to 1 to avoid division by zero
            }
        }

        // Initialize polyphonic output
        bool polyConnected = false;
        if (outputs[POLY_OUTPUT].isConnected()) {
            outputs[POLY_OUTPUT].setChannels(16);
            polyConnected = true;
        } else {
            polyConnected = false;
        }

        //Process each clock channel
        for (int i = 0; i < (CHANNELS+1); i++) {
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

            if ( (ClockTimer[i].time >= (60.0f / (bpm * ratio[i]))) && i>0) ClockTimer[i].reset();  //Process Channels via timer for continuous swing
            
            if ( (processSampleCounter >= (masterClockLength)) || syncPoint ){  //Process Master via samples for higher precision where it counts most  //OR at Sync Points!
                processSampleCounter = 0;
    
                ClockTimer[0].reset(); //Process master clock with straight sample-based clock and swing offset
                if (i == 0) {  // Master clock reset point
                    masterClockCycle++;
                    clockPulse.trigger(args.sampleTime);  //Trigger Pulse for Chain Connection
                    // Rotate phases
                    for (int k = 1; k < (CHANNELS+1); k++) {
                        int newIndex = (k + clockRotate) % CHANNELS;
                        if (newIndex < 0) {
                            newIndex += CHANNELS; // Adjust for negative values to wrap around correctly
                        }
                        tempPhases[newIndex + 1] = phases[k];
                    }
                    for (int k = 1; k < (CHANNELS+1); k++) {
                        phases[k] = tempPhases[k];
                    }

                    for (int j = 1; j < (CHANNELS+1); j++) {
                        if (masterClockCycle % lcmWithMaster[j] == 0) {
                           ClockTimer[j].reset();
                        }

                        if (resyncFlag[j]) {
                           ClockTimer[j].reset();
                           resyncFlag[j] = false;
                        }
                    }
                }
            }

            if (bpm <= 0) bpm = 1.0f;  // Ensure bpm is positive and non-zero
            if (ratio[i] <= 0) ratio[i] = 1.0f;  // Ensure ratio is positive and non-zero

            // compute normalized phase (ClockTimer reset ensures phase < 1.0)
            float phaseDenominator = 60.0f / (bpm * ratio[i]);
            phases[i] = ClockTimer[i].time / phaseDenominator;
            
            // Compute rotation source index: src is channel index in 1..CHANNELS for i>0
            // This section is more CPU efficient than the previous method using fmod
            int srcIndex;
            if (i == 0) {
                srcIndex = 0; // master channel never rotates
            } else {
                // ensure rotation wraps cleanly in both directions
                int tmp = (clockRotate + (i - 1)) % CHANNELS; // tmp in 0..CHANNELS-1
                if (tmp < 0) tmp += CHANNELS;                 // handle negative rotation
                srcIndex = tmp + 1;                           // convert to 1..CHANNELS
            }
           
            // highState read uses already-normalized phases
            bool highState = (i == 0) ? (phases[0] < 0.5f) : (phases[srcIndex] < 0.5f);
 
            if (sequenceRunning) {
                if (phasorMode) {
                    // Phasor mode: compute adjusted phase only when needed (and avoid fmod)
                    float phase_offset = 0.5f;
    
                    if (multiply[srcIndex] > 0.0f || i == 0) {
                        // fetch the source phase
                        float srcPhase = (i == 0) ? phases[0] : phases[srcIndex];
                        float adjusted_phase = srcPhase + phase_offset;
                        // adjusted_phase should normally be < 2.0; subtract 1.0 if it crossed wrap
                        if (adjusted_phase >= 1.0f) adjusted_phase -= 1.0f;
                        // write outputs
                        outputs[CLOCK_OUTPUT + i].setVoltage(srcPhase * 10.0f);
                        if (polyConnected && i > 0) {
                            outputs[POLY_OUTPUT].setVoltage(srcPhase * 10.0f, i - 1);           // Channels 0-7
                            outputs[POLY_OUTPUT].setVoltage(adjusted_phase * 10.0f, i + 7);     // Channels 8-15 (inverted)
                        }
                    } else {
                        outputs[CLOCK_OUTPUT + i].setVoltage(0.f);
                        if (polyConnected && i > 0) {
                            outputs[POLY_OUTPUT].setVoltage(0.f, i - 1);
                            outputs[POLY_OUTPUT].setVoltage(10.0f, i + 7); // inverted
                        }
                    }
                } else {
                    if (multiply[srcIndex] > 0.0f || i == 0) {
                        outputs[CLOCK_OUTPUT + i].setVoltage(highState ? 10.0f : 0.0f);
                        if (polyConnected && i > 0) {
                            outputs[POLY_OUTPUT].setVoltage(highState ? 10.0f : 0.0f, i - 1);
                            outputs[POLY_OUTPUT].setVoltage(highState ? 0.0f : 10.0f, i + (CHANNELS - 1));
                        }
                    } else {
                        outputs[CLOCK_OUTPUT + i].setVoltage(0.0f);
                        if (polyConnected && i > 0) {
                            outputs[POLY_OUTPUT].setVoltage(0.0f, i - 1);
                            outputs[POLY_OUTPUT].setVoltage(10.f, i + (CHANNELS - 1));
                        }
                    }
                }
            } else {
                outputs[CLOCK_OUTPUT + i].setVoltage(0.f);
            }
        } 
        
        if (outputs[CHAIN_OUTPUT].isConnected()) {

            // Check for reset condition change
            bool currentResetState = resetCondition;
            if (currentResetState && !lastResetState) {
                chainReset.trigger(args.sampleTime);
            }
            // Check for on/off state change
            else if (sequenceRunning != lastSequenceRunning) {
                if (sequenceRunning) {
                    chainON.trigger(args.sampleTime);
                } else {
                    chainOFF.trigger(args.sampleTime);
                }
            }
            lastResetState = currentResetState;
            lastSequenceRunning = sequenceRunning;
        }

        //Chain Outputs
        bool ResetActive = (params[RESET_BUTTON].getValue()>0.1f || resetPulseGen.process(args.sampleTime) );
        bool OnActive = chainON.process(args.sampleTime);
        bool OffActive = chainOFF.process(args.sampleTime);

        bool ClockPulseActive = clockPulse.process(args.sampleTime);

        outputs[CHAIN_OUTPUT].setVoltage(0.0f);
        if (ClockPulseActive) outputs[CHAIN_OUTPUT].setVoltage(5.0f);
        if (ResetActive) outputs[CHAIN_OUTPUT].setVoltage(10.42f);
        if (OnActive) outputs[CHAIN_OUTPUT].setVoltage(10.69f);
        if (OffActive) outputs[CHAIN_OUTPUT].setVoltage(10.86f);

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
    DigitalDisplay* phasorDisplay = nullptr;
    DigitalDisplay* bpmDisplay = nullptr;
    DigitalDisplay* swingDisplay = nullptr;
    DigitalDisplay* ratioDisplays[CHANNELS] = {nullptr};


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

        for (int i = 0; i < CHANNELS; i++) {
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

        addParam(createParamCentered<TL1105>(mm2px(Vec(9,  108)), module, Hammer::ON_OFF_BUTTON));
        addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(9,  108)), module, Hammer::ON_OFF_LIGHT ));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(9,  115)), module, Hammer::ON_OFF_INPUT));

        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(38,  115)), module, Hammer::EXT_CLOCK_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(21,  115)), module, Hammer::RESET_INPUT));
        addParam(createParamCentered<TL1105>(mm2px(Vec(27.5,  115)), module, Hammer::RESET_BUTTON));

        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(52, 115)), module, Hammer::CHAIN_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(66, 115)), module, Hammer::POLY_OUTPUT));

        addParam(createParamCentered<RoundBlackKnob>  (Vec(140 ,   42), module, Hammer::CLOCK_KNOB));
        addParam(createParamCentered<Trimpot>         (Vec(165.25, 42), module, Hammer::CLOCK_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(188.58, 42), module, Hammer::CLOCK_INPUT));

        addParam(createParamCentered<RoundBlackKnob>  (Vec(27 ,142+15), module, Hammer::SWING_KNOB));

        addParam(createParamCentered<RoundBlackKnob>  (Vec(27 ,195+30), module, Hammer::ROTATE_KNOB));
        addParam(createParamCentered<Trimpot>         (Vec(27, 195+25.25+30), module, Hammer::ROTATE_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(27, 195 + 48.58+30), module, Hammer::ROTATE_INPUT));

        // BPM Display Initialization
        bpmDisplay = createDigitalDisplay(Vec(19, 40), "120.0", 16.f);
        addChild(bpmDisplay);

        phasorDisplay = createDigitalDisplay(Vec(32, 48), "", 7.f);
        addChild(phasorDisplay);

        // Ratio Displays Initialization
        for (int i = 0; i < CHANNELS; i++) {
            ratioDisplays[i] = createDigitalDisplay(mm2px(Vec(24.f+xoffset, 46.365f + (float)i * 10.386f + yoffset)), "1:1", 14.f);
            addChild(ratioDisplays[i]);
        }        
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
    
        Hammer* hammerModule = dynamic_cast<Hammer*>(module);
        if (!hammerModule) return;
    
        // -------- Phasor & Clock CV menu items --------
        menu->addChild(new MenuSeparator());
    
        struct PhasorEnabledItem : MenuItem {
            Hammer* hammerModule;
            void onAction(const event::Action&) override {
                hammerModule->phasorMode = !hammerModule->phasorMode;
            }
            void step() override {
                rightText = hammerModule->phasorMode ? "✔" : "";
                MenuItem::step();
            }
        };
        PhasorEnabledItem* phasorItem = new PhasorEnabledItem();
        phasorItem->text = "Phasor Mode";
        phasorItem->hammerModule = hammerModule;
        menu->addChild(phasorItem);
    
        struct ClockCVAsVoctItem : MenuItem {
            Hammer* hammerModule;
            void onAction(const event::Action&) override {
                hammerModule->clockCVAsVoct = !hammerModule->clockCVAsVoct;
                if (hammerModule->clockCVAsVoct) hammerModule->clockCVAsBPM = false;
            }
            void step() override {
                rightText = hammerModule->clockCVAsVoct ? "✔" : "";
                MenuItem::step();
            }
        };
        ClockCVAsVoctItem* clockItem = new ClockCVAsVoctItem();
        clockItem->text = "Clock CV as V/oct";
        clockItem->hammerModule = hammerModule;
        menu->addChild(clockItem);
    
        struct ClockCVAsBPMItem : MenuItem {
            Hammer* hammerModule;
            void onAction(const event::Action&) override {
                hammerModule->clockCVAsBPM = !hammerModule->clockCVAsBPM;
                if (hammerModule->clockCVAsBPM) hammerModule->clockCVAsVoct = false;
            }
            void step() override {
                rightText = hammerModule->clockCVAsBPM ? "✔" : "";
                MenuItem::step();
            }
        };
        ClockCVAsBPMItem* bpmItem = new ClockCVAsBPMItem();
        bpmItem->text = "Clock CV is 1V/10BPM";
        bpmItem->hammerModule = hammerModule;
        menu->addChild(bpmItem);
    
        menu->addChild(new MenuSeparator());
    
        // ------------ Channel Multiply/Divide -------------
        menu->addChild(createMenuLabel("Channel Multiply/Divide"));
    
        // Quantity that updates the module's multiply/divide and keeps ratio & resyncFlag in sync.
        struct ChannelFloatQuantity : Quantity {
            Hammer* module;
            int idx;            // index into module arrays (1..8)
            std::string label;
            float minV, maxV;
            int precision;
            ChannelFloatQuantity(Hammer* m, int i, std::string lbl, float mn, float mx, int prec = 0)
                : module(m), idx(i), label(lbl), minV(mn), maxV(mx), precision(prec) {}
            void setValue(float v) override {
                float cv = clamp(v, minV, maxV);
                // Determine whether label contains "Multiply" or "Divide" to write the right array
                if (label.rfind("Multiply", 0) == 0) {
                    module->multiply[idx] = cv;
                } else {
                    module->divide[idx] = cv;
                }
                // Keep ratio and resync in sync
                if (module->divide[idx] == 0.0f) module->divide[idx] = 1.0f;
                module->ratio[idx] = module->multiply[idx] / module->divide[idx];
                module->resyncFlag[idx] = true;
            }
            float getValue() override {
                if (label.rfind("Multiply", 0) == 0) return module->multiply[idx];
                return module->divide[idx];
            }
            float getDefaultValue() override { return getValue(); }
            float getMinValue() override { return minV; }
            float getMaxValue() override { return maxV; }
            int getDisplayPrecision() override { return precision; }
            std::string getLabel() override { return label; }
            std::string getDisplayValueString() override {
                if (precision == 0) return std::to_string((int)std::round(getValue()));
                return string::f("%.*f", precision, getValue());
            }
        };
    
        // Helper to add the two sliders (multiply/divide) for a channel submenu.
        auto addChannelSliders = [hammerModule](Menu* parent, int channel0based) {
            // Map 0..7 -> module indexes 1..8
            int idx = channel0based + 1;
    
            // Multiply slider
            auto* mulSlider = new ui::Slider();
            mulSlider->quantity = new ChannelFloatQuantity(hammerModule, idx, "Multiply", 0.0f, 99.0f, 0);
            mulSlider->box.size.x = 200.f;
            parent->addChild(mulSlider);
    
            // Divide slider
            auto* divSlider = new ui::Slider();
            divSlider->quantity = new ChannelFloatQuantity(hammerModule, idx, "Divide", 1.0f, 99.0f, 0);
            divSlider->box.size.x = 200.f;
            parent->addChild(divSlider);
        };
    
        // Create 8 channel submenus (Channels 1..8)
        for (int i = 0; i < CHANNELS; i++) {
            menu->addChild(createSubmenuItem(
                "Channel " + std::to_string(i + 1),
                "",
                [=](Menu* sub) { addChannelSliders(sub, i); }
            ));
        }
    
        menu->addChild(new MenuSeparator());
    }


    void draw(const DrawArgs& args) override {
        ModuleWidget::draw(args);
        Hammer* module = dynamic_cast<Hammer*>(this->module);
        if (!module) return;

        // Update ratio displays
        for (int i = 1; i < (CHANNELS+1); i++) {
            int index = (module->clockRotate + i - 1) % CHANNELS;
            if (index < 0) {
                index += CHANNELS; // Adjust for negative values to wrap around correctly
            }

            module->disp_multiply[i] = module->multiply[index+1];
            module->disp_divide[i] = module->divide[index+1];
            if (ratioDisplays[i-1]) {
                char ratioText[16];
                snprintf(ratioText, sizeof(ratioText), "%d:%d", static_cast<int>(module->disp_multiply[i]), static_cast<int>(module->disp_divide[i]));
                if (index == 0) { // Check if the current index corresponds to the rotated position
                    if (module->disp_multiply[i]>0){
                        ratioDisplays[i-1]->text = "▸" + std::string(ratioText);
                    } else {
                        ratioDisplays[i-1]->text = "▸off";
                    }

                } else {
                    if (module->disp_multiply[i]>0){
                        ratioDisplays[i-1]->text = std::string(ratioText);
                    } else {
                        ratioDisplays[i-1]->text = "off";
                    }
                }
            }
        }

        // Update BPM display
        if (bpmDisplay) {
            char bpmText[16];
            if (module->clockCVAsVoct) {
                float bpmRounded = std::round(module->bpm * 10.0f) / 10.0f;
                snprintf(bpmText, sizeof(bpmText), "▸%.1f", bpmRounded);
            } else {
                float bpmRounded = std::round(module->bpm * 10.0f) / 10.0f;
                snprintf(bpmText, sizeof(bpmText), "%.1f", bpmRounded);
            }
            bpmDisplay->text = bpmText;
        }

        if (phasorDisplay) {
            if (module->phasorMode){
                phasorDisplay->text = "Phasor Mode";
            } else {
                phasorDisplay->text = "";
            }
        }

        for (int i=0; i<(CHANNELS+1); i++){
            module->lights[Hammer::CLOCK_LIGHT + i].setBrightness( module->outputs[Hammer::CLOCK_OUTPUT + i].getVoltage()*0.1f );
        }

        module->lights[Hammer::ON_OFF_LIGHT].setBrightness(module->sequenceRunning ? 1.0f : 0.0f);

    }

    DigitalDisplay* createDigitalDisplay(Vec position, std::string initialValue, float fontSizeFloat) {
        DigitalDisplay* display = new DigitalDisplay();
        display->box.pos = position;
        display->box.size = Vec(50, 18);
        display->text = initialValue;
        display->fgColor = nvgRGB(208, 140, 89); // Gold color text
        display->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        display->setFontSize(fontSizeFloat);
        return display;
    }
};

Model* modelHammer = createModel<Hammer, HammerWidget>("Hammer");