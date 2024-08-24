////////////////////////////////////////////////////////////
//
//   StepWave
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   8-step sequencer with variable shape, beats per step, and rhythmic displacement
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
using namespace rack;

template<typename T, size_t Size>
class CircularBuffer {
private:
    T buffer[Size];
    size_t index = 0;

public:
    CircularBuffer() {
        // Initialize buffer to zero
        for (size_t i = 0; i < Size; ++i) buffer[i] = T{};
    }

    void push(T value) {
        buffer[index] = value;
        index = (index + 1) % Size;
    }

    T& operator[](size_t i) {
        return buffer[(index + i) % Size];
    }

    const T& operator[](size_t i) const {
        return buffer[(index + i) % Size];
    }

    static constexpr size_t size() {
        return Size;
    }
};

struct StepWave : Module {
    enum ParamIds {
        STEP_1_VAL, STEP_2_VAL, STEP_3_VAL, STEP_4_VAL,
        STEP_5_VAL, STEP_6_VAL, STEP_7_VAL, STEP_8_VAL,
        STEP_1_BEATS, STEP_2_BEATS, STEP_3_BEATS, STEP_4_BEATS,
        STEP_5_BEATS, STEP_6_BEATS, STEP_7_BEATS, STEP_8_BEATS,
        STEP_1_SHAPE, STEP_2_SHAPE, STEP_3_SHAPE, STEP_4_SHAPE,
        STEP_5_SHAPE, STEP_6_SHAPE, STEP_7_SHAPE, STEP_8_SHAPE,
        STEP_1_2_DISPLACE, STEP_2_3_DISPLACE, STEP_3_4_DISPLACE, STEP_4_5_DISPLACE,
        STEP_5_6_DISPLACE, STEP_6_7_DISPLACE, STEP_7_8_DISPLACE,  
        STEP_1_BUTTON, STEP_2_BUTTON, STEP_3_BUTTON, STEP_4_BUTTON,
        STEP_5_BUTTON, STEP_6_BUTTON, STEP_7_BUTTON, STEP_8_BUTTON,
        SLEW_PARAM,     
        ON_OFF_BUTTON, RESET_BUTTON,
        NUM_PARAMS
    };
    enum InputIds {
        CLOCK_INPUT,        
        STEP_1_IN_VAL, STEP_2_IN_VAL, STEP_3_IN_VAL, STEP_4_IN_VAL,
        STEP_5_IN_VAL, STEP_6_IN_VAL, STEP_7_IN_VAL, STEP_8_IN_VAL,
        STEP_1_2_DISPLACE_IN, STEP_2_3_DISPLACE_IN, STEP_3_4_DISPLACE_IN, STEP_4_5_DISPLACE_IN,
        STEP_5_6_DISPLACE_IN, STEP_6_7_DISPLACE_IN, STEP_7_8_DISPLACE_IN,  
        SLEW_INPUT,
        ON_OFF_INPUT, RESET_INPUT,                     
        NUM_INPUTS
    };
    enum OutputIds {
        CV_OUTPUT, GATE_OUTPUT,
        STEP_1_GATE_OUT, STEP_2_GATE_OUT, STEP_3_GATE_OUT, STEP_4_GATE_OUT, 
        STEP_5_GATE_OUT, STEP_6_GATE_OUT, STEP_7_GATE_OUT, STEP_8_GATE_OUT, 
        NUM_OUTPUTS
    };
    enum LightIds {
        STEP_1_VAL_LIGHT, STEP_2_VAL_LIGHT, STEP_3_VAL_LIGHT, STEP_4_VAL_LIGHT,
        STEP_5_VAL_LIGHT, STEP_6_VAL_LIGHT, STEP_7_VAL_LIGHT, STEP_8_VAL_LIGHT,
        STEP_1_GATE_LIGHT, STEP_2_GATE_LIGHT, STEP_3_GATE_LIGHT, STEP_4_GATE_LIGHT, 
        STEP_5_GATE_LIGHT, STEP_6_GATE_LIGHT, STEP_7_GAT_LIGHTE, STEP_8_GATE_LIGHT, 
        ON_OFF_LIGHT,    
        NUM_LIGHTS
    };

    // For Clocking 
    dsp::Timer SyncTimer;
    dsp::Timer ClockTimerA; 
    dsp::Timer ClockTimerB; 
    dsp::SchmittTrigger SyncTrigger;
    float lastClockTime = -1.0f;
    float SyncInterval[2] = {1.0f/60.f, 1.0f};
    bool firstClockPulse = true;
    bool resetPulse = false;

    // For Reset and ON/OFF
    dsp::SchmittTrigger resetTrigger;
    dsp::SchmittTrigger onOffTrigger;
    dsp::SchmittTrigger onOffButtonTrigger;
    bool sequenceRunning = true;

    // For each stage
    int currentStage[2] = {0,0};
    float stepValues[8] = {0.0f};
    float sampledStepValue[8] =  {0.0f};
    float sampledDispStepValue[8] =  {0.0f};
    float stageDuration[2] = {1.0f, 1.0f};
    float currentShape[2] = {0.f, 0.f};
    int numBeats = 0;
    dsp::SchmittTrigger stepButtonTrigger[8];
    float frameLength[2] = {1.0f,1.0f};
    float splitTime[2] = {0.5f,0.5f};
    float normallizedStageProgress[2] = {0.0f,0.0f};
    float previousStagesLength[2] = {0.0f,0.0f};
    float slewedVoltage[2] = {0.0f,0.0f};
    float normallizedSplitTime[2] = {0.0f, 0.0f};
    float currentTime[2] = {0.f, 0.f};
    float finalCV[2] = {0.0f,0.0f};        

    //For the display
    CircularBuffer<float, 1024> waveBuffers[3];
    float oscPhase[2] = {0.0f}; // Current oscillator phase for each channel

    // For the output
    dsp::SlewLimiter slewLimiterA; 
    dsp::SlewLimiter slewLimiterB; 
    float lastTargetVoltage[2] = {0.f,0.f};
    bool trackCV = false;
    bool shapeBeats = false;
    bool waitForReset = true;
    
    json_t* dataToJson() override {
        json_t* rootJ = json_object();
    
        // Save the state of sequenceRunning
        json_object_set_new(rootJ, "sequenceRunning", json_boolean(sequenceRunning));
    
        // Save the state of trackCV
        json_object_set_new(rootJ, "trackCV", json_boolean(trackCV));
    
        // Save the state of shapeBeats
        json_object_set_new(rootJ, "shapeBeats", json_boolean(shapeBeats));
    
        return rootJ;
    }
    
    void dataFromJson(json_t* rootJ) override {
        // Load the state of sequenceRunning
        json_t* sequenceRunningJ = json_object_get(rootJ, "sequenceRunning");
        if (sequenceRunningJ) {
            sequenceRunning = json_is_true(sequenceRunningJ);
        }
    
        // Load the state of trackCV
        json_t* trackCVJ = json_object_get(rootJ, "trackCV");
        if (trackCVJ) {
            trackCV = json_is_true(trackCVJ);
        }
    
        // Load the state of shapeBeats
        json_t* shapeBeatsJ = json_object_get(rootJ, "shapeBeats");
        if (shapeBeatsJ) {
            shapeBeats = json_is_true(shapeBeatsJ);
        }                
    }

    StepWave() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        configParam(STEP_1_VAL, -5.f, 5.f, 0.0f, "Stage 1 Value");
        configParam(STEP_2_VAL, -5.f, 5.f, 0.0f, "Stage 2 Value");
        configParam(STEP_3_VAL, -5.f, 5.f, 0.0f, "Stage 3 Value");
        configParam(STEP_4_VAL, -5.f, 5.f, 0.0f, "Stage 4 Value");
        configParam(STEP_5_VAL, -5.f, 5.f, 0.0f, "Stage 5 Value");
        configParam(STEP_6_VAL, -5.f, 5.f, 0.0f, "Stage 6 Value");
        configParam(STEP_7_VAL, -5.f, 5.f, 0.0f, "Stage 7 Value");
        configParam(STEP_8_VAL, -5.f, 5.f, 0.0f, "Stage 8 Value");

        configParam(STEP_1_BEATS, 1.f, 10.f, 1.0f, "Stage 1 Beats");
        configParam(STEP_2_BEATS, 1.f, 10.f, 1.0f, "Stage 2 Beats");
        configParam(STEP_3_BEATS, 1.f, 10.f, 1.0f, "Stage 3 Beats");
        configParam(STEP_4_BEATS, 1.f, 10.f, 1.0f, "Stage 4 Beats");
        configParam(STEP_5_BEATS, 1.f, 10.f, 1.0f, "Stage 5 Beats");
        configParam(STEP_6_BEATS, 1.f, 10.f, 1.0f, "Stage 6 Beats");
        configParam(STEP_7_BEATS, 1.f, 10.f, 1.0f, "Stage 7 Beats");
        configParam(STEP_8_BEATS, 1.f, 10.f, 1.0f, "Stage 8 Beats");
 
        configParam(STEP_1_SHAPE, 1.f, 12.f, 1.0f, "Stage 1 Shape");
        configParam(STEP_2_SHAPE, 1.f, 12.f, 1.0f, "Stage 2 Shape");
        configParam(STEP_3_SHAPE, 1.f, 12.f, 1.0f, "Stage 3 Shape");
        configParam(STEP_4_SHAPE, 1.f, 12.f, 1.0f, "Stage 4 Shape");
        configParam(STEP_5_SHAPE, 1.f, 12.f, 1.0f, "Stage 5 Shape");
        configParam(STEP_6_SHAPE, 1.f, 12.f, 1.0f, "Stage 6 Shape");
        configParam(STEP_7_SHAPE, 1.f, 12.f, 1.0f, "Stage 7 Shape");
        configParam(STEP_8_SHAPE, 1.f, 12.f, 1.0f, "Stage 8 Shape");

        configParam(STEP_1_2_DISPLACE, -5.f, 5.f, 0.0f, "Rhythmic Displacement 1-2");
        configParam(STEP_2_3_DISPLACE, -5.f, 5.f, 0.0f, "Rhythmic Displacement 2-3");
        configParam(STEP_3_4_DISPLACE, -5.f, 5.f, 0.0f, "Rhythmic Displacement 3-4");
        configParam(STEP_4_5_DISPLACE, -5.f, 5.f, 0.0f, "Rhythmic Displacement 4-5");
        configParam(STEP_5_6_DISPLACE, -5.f, 5.f, 0.0f, "Rhythmic Displacement 5-6");
        configParam(STEP_6_7_DISPLACE, -5.f, 5.f, 0.0f, "Rhythmic Displacement 6-7");
        configParam(STEP_7_8_DISPLACE, -5.f, 5.f, 0.0f, "Rhythmic Displacement 7-8");

        configParam(SLEW_PARAM, 0.f, 1.f, 0.0f, "Slew");

    }

    void process(const ProcessArgs &args) override {

        // Process clock sync input
        if (inputs[CLOCK_INPUT].isConnected()) {
            float SyncInputVoltage = inputs[CLOCK_INPUT].getVoltage();

            if (SyncTrigger.process(SyncInputVoltage)) {
                if (!firstClockPulse) {
                     SyncInterval[1] = SyncTimer.time; // Get the accumulated time since the last reset                         }
                }
                SyncTimer.reset(); // Reset the timer for the next trigger interval measurement
                resetPulse = true;
                firstClockPulse = false;
            }
        } 

        // Process timers
        float deltaTimeA = args.sampleTime; //for the display clock
        float deltaTimeB = args.sampleTime; //for the synced clock

        SyncTimer.process(deltaTimeB);

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
            deltaTimeB = 0.f;
            ClockTimerB.reset();
            firstClockPulse = true;
        }

        // Check for reset input or reset button
        bool resetCondition = (inputs[RESET_INPUT].isConnected() && resetTrigger.process(inputs[RESET_INPUT].getVoltage())) || (params[RESET_BUTTON].getValue() > 0.1f);    
        if (resetCondition) {
            ClockTimerB.reset();
            currentStage[1] = 0;
        } 
            
        for (int j=0; j<2; j++){ //Cycle through the two different clock layers
              
            if (currentStage[j] == 0) { 
                previousStagesLength[j] = 0.0f;  // Reset at the beginning of the sequence
            }
            
            // Override and animate stage level controls if external CV connected
            for (int i = 0; i<8; i++){
                if (inputs[STEP_1_IN_VAL + i].isConnected() && sequenceRunning) {
                    stepValues[i] = clamp(inputs[STEP_1_IN_VAL + i].getVoltage(),-5.0f, 5.0f);
                    params[STEP_1_VAL + i].setValue(stepValues[i]);
                } else {
                    stepValues[i] = params[STEP_1_VAL + i].getValue();        
                }
            }
            
            //Compute rhythmic offsets
            if (currentStage[j] == 0){
                float displacementZero = clamp ( params[STEP_1_2_DISPLACE].getValue() + inputs[STEP_1_2_DISPLACE_IN].getVoltage(), -5.f, 5.f);
                stageDuration[j] = ((displacementZero / 10.f) + 1) * SyncInterval[j];
            } else if (currentStage[j] > 0 && currentStage[j] < 7) {
                float displacementOne = clamp ( params[STEP_1_2_DISPLACE + currentStage[j]].getValue() + inputs[STEP_1_2_DISPLACE_IN + currentStage[j]].getVoltage(), -5.f, 5.f);
                float displacementTwo = clamp ( params[STEP_1_2_DISPLACE + currentStage[j] - 1].getValue() + inputs[STEP_1_2_DISPLACE_IN + currentStage[j] - 1].getVoltage(), -5.f, 5.f);        
                stageDuration[j] = ((displacementOne / 10.f ) - (displacementTwo / 10.f ) + 1) * SyncInterval[j];
            } else {
                float displacementLast = clamp ( params[STEP_7_8_DISPLACE].getValue() + inputs[STEP_7_8_DISPLACE_IN].getVoltage(), -5.f, 5.f);
                stageDuration[j] = SyncInterval[j] * (1 - displacementLast / 10.f );
            }        

            // Ensure stageDuration[j] does not become too small
            const float minStageDuration = 0.0001f; // Minimum value to prevent division by zero
            stageDuration[j] = fmax(stageDuration[j], minStageDuration);
    
            if (j==0){        
                ClockTimerA.process(deltaTimeA);
                currentTime[j] = ClockTimerA.time;
                normallizedStageProgress[j] = currentTime[j]/stageDuration[j];
            } else {
                ClockTimerB.process(deltaTimeB);
                currentTime[j] = ClockTimerB.time;
                normallizedStageProgress[j] = currentTime[j]/stageDuration[j];        
            }


            if (j==1 && resetPulse && currentStage[1]==7) { //require a reset pulse to loop sequence
                resetPulse = false;
                ClockTimerB.reset();
                currentStage[1] += 1; 
            } else if (j==1 && currentStage[1]<7){
                resetPulse = false;
            }

            
            if (currentTime[j] >= stageDuration[j]){
                if (j==0){
                    ClockTimerA.reset();  //Reset master channel
                } else {
                    ClockTimerB.reset();
                }
                                
                if (j==0){
                    currentStage[j] += 1; //always advance the display layer
                } else if (j==1 && currentStage[1]<7){
                    currentStage[j] += 1;
                }
                
                if (currentStage[j] > 7) {currentStage[j] = 0;} 
                
                sampledStepValue[currentStage[j]] = stepValues[currentStage[j]];
                currentShape[j] = params[STEP_1_SHAPE + currentStage[j]].getValue();
    
                //Set all the Beats params to an integer and fix the knob at stage end
                for (int i = 0; i<8; i++){
                    int intBeats = int(params[STEP_1_BEATS + i].getValue());
                    params[STEP_1_BEATS + i].setValue(intBeats);
                }
                
                previousStagesLength[j] += stageDuration[j] / SyncInterval[j]; // Accumulate the duration of each stage normalized to SyncInterval[j]
                normallizedStageProgress[j] = 0; //reset the current stage progress meter
                
                numBeats = floor( params[STEP_1_BEATS + currentStage[j]].getValue() );                
            }
    
            if (j==1){//only jump to button for the sequencer layer
                // Jump to step if step button is pushed
                for (int i = 0; i < 8; i++) {
                    if (stepButtonTrigger[i].process(params[STEP_1_BUTTON + i].getValue())) {
                        currentStage[j] = i;
                        sampledStepValue[currentStage[j]] = stepValues[currentStage[j]];
                        currentShape[j] = params[STEP_1_SHAPE + currentStage[j]].getValue();
                        numBeats = floor(params[STEP_1_BEATS + currentStage[j]].getValue());
                
                        // Reset stage progress
                        normallizedStageProgress[j] = 0.0f;
                
                        // Recalculate the total length of all stages up to the jump spot
                        previousStagesLength[j] = 0.0f;
                
                        for (int k = 0; k < currentStage[j]; k++) {
                            float displacementCurrent = clamp(params[STEP_1_2_DISPLACE + k].getValue() + inputs[STEP_1_2_DISPLACE_IN + k].getVoltage(), -5.f, 5.f);
                            float displacementPrevious = (k > 0) ? clamp(params[STEP_1_2_DISPLACE + k - 1].getValue() + inputs[STEP_1_2_DISPLACE_IN + k - 1].getVoltage(), -5.f, 5.f) : 0.f;
                            
                            // Calculate the duration of the current stage
                            float currentStageDuration = ((displacementCurrent / 10.f) - (displacementPrevious / 10.f) + 1) * SyncInterval[j];
                            
                            // Accumulate the lengths
                            previousStagesLength[j] += currentStageDuration / SyncInterval[j];
                        }
                
                        // After computing previousStagesLength, set the current stage duration based on its displacement
                        float displacementCurrent = clamp(params[STEP_1_2_DISPLACE + currentStage[j]].getValue() + inputs[STEP_1_2_DISPLACE_IN + currentStage[j]].getVoltage(), -5.f, 5.f);
                        float displacementPrevious = (currentStage[j] > 0) ? clamp(params[STEP_1_2_DISPLACE + currentStage[j] - 1].getValue() + inputs[STEP_1_2_DISPLACE_IN + currentStage[j] - 1].getVoltage(), -5.f, 5.f) : 0.f;
                        
                        stageDuration[j] = ((displacementCurrent / 10.f) - (displacementPrevious / 10.f) + 1) * SyncInterval[j];
            
                        ClockTimerB.reset();                        
                    }
                }
            }
    
            if (j==1){ //only output stage lights for the synced clock
                // Stage Lights
                for (int i=0; i<8; i++){
                    if (currentStage[j] == i && sequenceRunning){
                        outputs[STEP_1_GATE_OUT + i ].setVoltage(5.0);
                        lights[STEP_1_GATE_LIGHT + i].setBrightness(1.0);
                        lights[STEP_1_VAL_LIGHT + i].setBrightness(1.0);                
                    } else if (currentStage[j] == i){
                        outputs[STEP_1_GATE_OUT + i ].setVoltage(0.0);
                        lights[STEP_1_GATE_LIGHT + i].setBrightness(0.5);
                        lights[STEP_1_VAL_LIGHT + i].setBrightness(0.25);
                    } else {
                        outputs[STEP_1_GATE_OUT + i ].setVoltage(0.0);
                        lights[STEP_1_GATE_LIGHT + i].setBrightness(0.0);
                        lights[STEP_1_VAL_LIGHT + i].setBrightness(0.25);            
                    }
                }
            }    
            
            //CV and Gate computation
            if (shapeBeats){
                numBeats = floor( params[STEP_1_BEATS + currentStage[j]].getValue() );     
            } else { numBeats = 1;}   
                    
            frameLength[j] = stageDuration[j]/numBeats;
            splitTime[j] = currentTime[j];
            while( splitTime[j] > stageDuration[j]/numBeats ) { //Compute subdivisions for beats
                splitTime[j] -= stageDuration[j]/numBeats;
            }
            normallizedSplitTime[j] = splitTime[j]/(stageDuration[j]/numBeats);
    
            if (currentShape[j] == 1.f) { // Rectangle Shape
                if (!trackCV) {
                    finalCV[j] = sampledStepValue[currentStage[j]];  
                } else {
                    finalCV[j] = stepValues[currentStage[j]]; 
                }
            } 
            else if (currentShape[j] > 1.f && currentShape[j] <= 2.f) { // Morphing to Right-Leaning Sawtooth
                float sawFactor = normallizedSplitTime[j];
                float morphFactor = (currentShape[j] - 1.f); // From 0 to 1 as currentShape goes from 1 to 2
            
                if (!trackCV) {
                    finalCV[j] = sampledStepValue[currentStage[j]] * (1 - morphFactor) + sampledStepValue[currentStage[j]] * morphFactor * sawFactor;
                } else {
                    finalCV[j] = stepValues[currentStage[j]] * (1 - morphFactor) + stepValues[currentStage[j]] * morphFactor * sawFactor;
                }
            }
            else if (currentShape[j] > 2.f && currentShape[j] <= 3.f) { // Morphing from Right-Leaning Triangle to Left-Leaning Triangle
                float morphFactor = currentShape[j] - 2.f; // Ranges from 0 to 1 as currentShape goes from 2 to 3
            
                // Calculate the peak position based on the morph factor
                float peakPosition = 1.f - morphFactor; // Moves from 1 (right) to 0 (left)
            
                // Calculate the triangle wave based on the peak position
                float triangleValue;
                if (normallizedSplitTime[j] <= peakPosition) {
                    triangleValue = normallizedSplitTime[j] / peakPosition; // Rising slope
                } else {
                    triangleValue = (1.f - normallizedSplitTime[j]) / (1.f - peakPosition); // Falling slope
                }
            
                // Apply the triangle wave to the CV
                if (!trackCV) {
                    finalCV[j] = sampledStepValue[currentStage[j]] * triangleValue;
                } else {
                    finalCV[j] = stepValues[currentStage[j]] * triangleValue;
                }
            }
            else if (currentShape[j] > 3.f && currentShape[j] <= 4.f) { // Morphing from Left-Leaning Triangle to Rectangle
                float morphFactor = currentShape[j] - 3.f; // Ranges from 0 to 1 as currentShape goes from 3 to 4
            
                // Triangle to rectangle morphing logic
                if (normallizedSplitTime[j] < 1.f - morphFactor) {
                    // Left part of the shape, which is still the triangle
                    finalCV[j] = sampledStepValue[currentStage[j]] * (1.f - normallizedSplitTime[j] / (1.f - morphFactor));
                } else {
                    // Right part morphing to a rectangle
                    finalCV[j] = 0.f;
                }
            }        
            else if (currentShape[j] > 4.f && currentShape[j] <= 5.f) { // Pulse Width Modulation from Left-Sided PWM to Rectangle
                float morphFactor = currentShape[j] - 4.f; // Ranges from 0 to 1 as currentShape goes from 4 to 5
                
                // Calculate the pulse width position
                float pulseWidth = morphFactor; // Moves from narrow pulse (0) to full width (1)
                
                // Generate the PWM shape based on pulse width
                if (normallizedSplitTime[j] <= pulseWidth) {
                    finalCV[j] = sampledStepValue[currentStage[j]]; // High part of the pulse
                } else {
                    finalCV[j] = 0.f; // Low part of the pulse
                }
            }
            else if (currentShape[j] > 5.f && currentShape[j] <= 6.f) { // Morphing from Rectangle to Sine Wave
                float morphFactor = currentShape[j] - 5.f; // Ranges from 0 to 1 as currentShape goes from 5 to 6
                
                // Generate a sine wave and interpolate it with the rectangle
                float sineValue = (sinf(2.f * M_PI * normallizedSplitTime[j]));
                if (!trackCV) {
                    finalCV[j] = sampledStepValue[currentStage[j]] * ((1 - morphFactor) + sineValue * morphFactor);
                } else {
                    finalCV[j] = stepValues[currentStage[j]] * ((1 - morphFactor) + sineValue * morphFactor);
                }
            }
            else if (currentShape[j] > 6.f && currentShape[j] <= 7.f) { // Morphing from Sine Wave to Square Wave
                float morphFactor = currentShape[j] - 6.f; // Ranges from 0 to 1 as currentShape goes from 6 to 7
                
                // Generate a square wave and interpolate it with the sine wave
                float squareValue = (normallizedSplitTime[j] < 0.5f) ? 1.f : -1.f;
                float sineValue = (sinf(2.f * M_PI * normallizedSplitTime[j]));
                if (!trackCV) {
                    finalCV[j] = sampledStepValue[currentStage[j]] * ((1 - morphFactor) * sineValue + morphFactor * squareValue);
                } else {
                    finalCV[j] = stepValues[currentStage[j]] * ((1 - morphFactor) * sineValue + morphFactor * squareValue);
                }
            }
            else if (currentShape[j] > 7.f && currentShape[j] <= 8.f) { // Morphing from Square Wave to Inverted Sawtooth
                float morphFactor = currentShape[j] - 7.f; // Ranges from 0 to 1 as currentShape goes from 7 to 8
                
                // Generate an inverted sawtooth wave and interpolate it with the square wave
                float squareValue = (normallizedSplitTime[j] < 0.5f) ? 1.f : -1.f;
                float sawtoothValue = 1.f - 2*normallizedSplitTime[j];
                if (!trackCV) {
                    finalCV[j] = sampledStepValue[currentStage[j]] * ((1 - morphFactor) * squareValue + morphFactor * sawtoothValue);
                } else {
                    finalCV[j] = stepValues[currentStage[j]] * ((1 - morphFactor) * squareValue + morphFactor * sawtoothValue);
                }
            }
            else if (currentShape[j] > 8.f && currentShape[j] <= 9.f) { // Morphing from Inverted Sawtooth to Triangle
                float morphFactor = currentShape[j] - 8.f; // Ranges from 0 to 1 as currentShape goes from 8 to 9
                
                // Generate a triangle wave and interpolate it with the inverted sawtooth
                float sawtoothValue = 1.f - 2*normallizedSplitTime[j];
                float triangleValue = (normallizedSplitTime[j] < 0.5f) ? (1-4.f * normallizedSplitTime[j]) : (1-4.f * (1.f - normallizedSplitTime[j]));
                if (!trackCV) {
                    finalCV[j] = sampledStepValue[currentStage[j]] * ((1 - morphFactor) * sawtoothValue + morphFactor * triangleValue);
                } else {
                    finalCV[j] = stepValues[currentStage[j]] * ((1 - morphFactor) * sawtoothValue + morphFactor * triangleValue);
                }
            }
            else if (currentShape[j] > 9.f && currentShape[j] <= 10.f) { // Morphing from Triangle to Logarithmic Ramp
                float morphFactor = currentShape[j] - 9.f; // Ranges from 0 to 1 as currentShape goes from 9 to 10
                
                // Generate a logarithmic ramp and interpolate it with the triangle
                float triangleValue = (normallizedSplitTime[j] < 0.5f) ? (1 - 4.f * normallizedSplitTime[j]) : (1 - 4.f * (1.f - normallizedSplitTime[j]));
                float logRampValue = 1.f - 2.f * logf(1.f + 9.f * normallizedSplitTime[j]) / logf(10.f);
                
                if (!trackCV) {
                    finalCV[j] = sampledStepValue[currentStage[j]] * ((1 - morphFactor) * triangleValue + morphFactor * logRampValue);
                } else {
                    finalCV[j] = stepValues[currentStage[j]] * ((1 - morphFactor) * triangleValue + morphFactor * logRampValue);
                }
            }
            else if (currentShape[j] > 10.f && currentShape[j] <= 11.f) { // Morphing from Logarithmic Ramp to Bell Curve
                float morphFactor = currentShape[j] - 10.f; // Ranges from 0 to 1 as currentShape goes from 10 to 11
                
                // Generate a logarithmic ramp and interpolate it with the bell curve
                float logRampValue = 1.f - 2.f * logf(1.f + 9.f * normallizedSplitTime[j]) / logf(10.f);
                
                // Bell Curve: Gaussian function centered around 0.5
                float bellCurveValue = expf(-50.f * powf(normallizedSplitTime[j] - 0.5f, 2.f));
                
                // Interpolate between logarithmic ramp and bell curve
                float interpolatedValue = (1.f - morphFactor) * logRampValue + morphFactor * bellCurveValue;
                
                // Apply the interpolated value to finalCV[j]
                if (!trackCV) {
                    finalCV[j] = sampledStepValue[currentStage[j]] * interpolatedValue;
                } else {
                    finalCV[j] = stepValues[currentStage[j]] * interpolatedValue;
                }
            }
            else if (currentShape[j] > 11.f && currentShape[j] <= 12.f) { // Morphing from Bell Curve to Flat-Topped
                float morphFactor = currentShape[j] - 11.f; // Ranges from 0 to 1 as currentShape goes from 11 to 12
                
                // Bell Curve: Gaussian function centered around 0.5
                float bellCurveValue = expf(-50.f * powf(normallizedSplitTime[j] - 0.5f, 2.f));
                
                // Flat-Topped Waveform: Linear ramp with flat top
                float flatTopValue = normallizedSplitTime[j] < 0.3f ? 
                                     (normallizedSplitTime[j] / 0.3f) : 
                                     (normallizedSplitTime[j] > 0.7f ? 
                                     (1.f - (normallizedSplitTime[j] - 0.7f) / 0.3f) : 
                                     1.f);
                
                // Interpolate between bell curve and flat-topped waveform
                float interpolatedValue = (1.f - morphFactor) * bellCurveValue + morphFactor * flatTopValue;
                
                // Apply the interpolated value to finalCV[j]
                if (!trackCV) {
                    finalCV[j] = sampledStepValue[currentStage[j]] * interpolatedValue;
                } else {
                    finalCV[j] = stepValues[currentStage[j]] * interpolatedValue;
                }
            }
        
            //Compute Slew and output CV voltages
            float slewRate = clamp(params[SLEW_PARAM].getValue() + inputs[SLEW_INPUT].getVoltage()/10.f, 0.f, 1.f); 
    
            if (slewRate >0){    
                // Calculate the absolute voltage difference from the last target
                float voltageDifference = fabs(finalCV[j] - lastTargetVoltage[j]);
        
                // Adjust slewSpeed based on the voltage difference and trigger interval
                // Ensure triggerInterval is non-zero to avoid division by zero
                float adjustedTriggerInterval = fmax( stageDuration[j], 1e-8f);
                float slewSpeed = voltageDifference / adjustedTriggerInterval; // Voltage difference per second
        
                // Apply the SLEW_PARAM knob to scale the slewSpeed, adding 1e-6 to avoid division by zero
                slewSpeed *= 1.0f / (slewRate + 1e-8f);
                if (j==0){
                    // Set the rise and fall speeds of the slew limiter to the calculated slew speed
                    slewLimiterA.setRiseFall(slewSpeed, slewSpeed);
            
                    // Process the target voltage through the slew limiter
                    slewedVoltage[j] = slewLimiterA.process(args.sampleTime, finalCV[j]);
                } else {
                    // Set the rise and fall speeds of the slew limiter to the calculated slew speed
                    slewLimiterB.setRiseFall(slewSpeed, slewSpeed);
            
                    // Process the target voltage through the slew limiter
                    slewedVoltage[j] = slewLimiterB.process(args.sampleTime, finalCV[j]);
                }
                if (j==1){
                    outputs[CV_OUTPUT].setVoltage(slewedVoltage[j]);
                }
                lastTargetVoltage[j] = slewedVoltage[j];    
            } else {
                if (j==1){
                    outputs[CV_OUTPUT].setVoltage(finalCV[j]);        
                }
                lastTargetVoltage[j] = finalCV[j];            
            }
            
            //Main Gate Output
    
            numBeats = floor( params[STEP_1_BEATS + currentStage[j]].getValue() );                
            frameLength[j] = stageDuration[j]/numBeats;
            splitTime[j] = currentTime[j];
            while( splitTime[j] > stageDuration[j]/numBeats ) {
                splitTime[j] -= stageDuration[j]/numBeats;
            }            
            
            float gateCV = 0.f;
            if (splitTime[j] < frameLength[j]/2.f){
                gateCV = 5.0f;
            } else {
                gateCV = 0.f;
            }
            if (j==1){
                if (sequenceRunning){
                    outputs[GATE_OUTPUT].setVoltage(gateCV);
                } else {
                    outputs[GATE_OUTPUT].setVoltage(0.0f);                
                }
            }
            oscPhase[0] = (previousStagesLength[j] + normallizedStageProgress[j] * (stageDuration[j] / SyncInterval[j])) / 8.f;
    
            if (j==0){ //only plot the display level
                //Wave display
                int sampleIndex = static_cast<int>(oscPhase[0] * 1024); 
                if (sampleIndex < 0) sampleIndex = 0;
                else if (sampleIndex > 1023) sampleIndex = 1023;
                waveBuffers[0][sampleIndex] = finalCV[j];
                if (slewRate>0){
                    waveBuffers[1][sampleIndex] = slewedVoltage[j];
                } else { 
                    waveBuffers[1][sampleIndex] = finalCV[j];
                }
                waveBuffers[2][sampleIndex] = 0.2*gateCV - 5.8;
            }
        }            
    }           
};

struct WaveDisplay : TransparentWidget {
    StepWave* module;
    float centerX, centerY;
    float heightScale; 

    void draw(const DrawArgs& args) override {
        // Draw non-illuminating elements if any
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (!module) return;

        if (layer == 1) {
            centerX = box.size.x / 2.0f;
            centerY = box.size.y / 2.0f;
            heightScale = centerY / 5; // Calculate based on current center Y

            drawWaveform(args, module->waveBuffers[0], nvgRGBAf(1, 0.4, 0, 0.8));
            drawWaveform(args, module->waveBuffers[1], nvgRGBAf(0, 0.4, 1, 0.8));
            drawWaveform(args, module->waveBuffers[2], nvgRGBAf(0.5, 0.5, 0.6, 0.8));
            
        }

        TransparentWidget::drawLayer(args, layer);
    }

    void drawWaveform(const DrawArgs& args, const CircularBuffer<float, 1024>& waveBuffer, NVGcolor color) {
        nvgBeginPath(args.vg);
    
        for (size_t i = 0; i < 1024; i++) {
            // Calculate x position based on the index
            float xPos = (float)i / 1023 * box.size.x;
            
            // Scale and center y position based on buffer value
            float yPos = centerY - waveBuffer[i] * heightScale;
    
            if (i == 0)
                nvgMoveTo(args.vg, xPos, yPos);
            else
                nvgLineTo(args.vg, xPos, yPos);
        }
    
        nvgStrokeColor(args.vg, color); // Set the color for the waveform
        nvgStrokeWidth(args.vg, 1.0);
        nvgStroke(args.vg);
    }
};

struct StepWaveWidget : ModuleWidget {
    StepWaveWidget(StepWave* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/StepWave.svg"),
            asset::plugin(pluginInstance, "res/StepWave-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        addInput(createInputCentered<ThemedPJ301MPort>(Vec(25, 75), module, StepWave::CLOCK_INPUT));
        addParam(createParamCentered<TL1105>              (Vec(25, 170), module, StepWave::ON_OFF_BUTTON));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(25, 170), module, StepWave::ON_OFF_LIGHT ));
        addInput(createInputCentered<ThemedPJ301MPort>    (Vec(25, 200), module, StepWave::ON_OFF_INPUT));

        addParam(createParamCentered<TL1105>              (Vec(25,270), module, StepWave::RESET_BUTTON));
        addInput(createInputCentered<ThemedPJ301MPort>    (Vec(25,300), module, StepWave::RESET_INPUT));

        // Constants for positioning
        const Vec channelOffset(23, 160); // Start position for the first channel controls
        const float sliderX = 44.0f;     // Horizontal spacing for sliders
        const float Spacing = 35.0f;     // Vertical spacing between inputs/outputs

        // Positioning variables
        float yPos = channelOffset.y;
        float xPos = channelOffset.x;

        // Loop through each channel
        for (int i = 0; i < 8; i++) {

            xPos = 50 + channelOffset.x + i * sliderX;

            // Volume slider with light
            addParam(createLightParamCentered<VCVLightSlider<YellowLight>>(Vec(xPos, yPos-5), module, StepWave::STEP_1_VAL + i, StepWave::STEP_1_VAL_LIGHT + i));

            if (i<7){
                addParam(createParamCentered<RoundBlackKnob>(Vec(xPos + 22 , yPos - 25), module, StepWave::STEP_1_2_DISPLACE + i));
                addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos + 22, yPos + 15), module, StepWave::STEP_1_2_DISPLACE_IN + i));
            }

            // Step CV input
            yPos += Spacing + 10;
            addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, StepWave::STEP_1_IN_VAL + i));

            // Shape knob
            yPos += Spacing;
            addParam(createParamCentered<RoundBlackKnob>(Vec(xPos, yPos), module, StepWave::STEP_1_SHAPE + i));

            // Beats knob
            yPos += Spacing + 10;
            addParam(createParamCentered<RoundBlackKnob>(Vec(xPos, yPos), module, StepWave::STEP_1_BEATS + i));

            // Stage Gate and Button
            yPos += Spacing;
            addParam(createParamCentered<LEDButton>(Vec(xPos, yPos), module, StepWave::STEP_1_BUTTON + i));
            addChild(createLightCentered<LargeLight<RedLight>>(Vec(xPos, yPos), module, StepWave::STEP_1_GATE_LIGHT + i ));

            yPos += Spacing - 10;
            addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, StepWave::STEP_1_GATE_OUT + i));

            // Reset yPos for next channel
            yPos = channelOffset.y;
        }

        addParam(createParamCentered<RoundBlackKnob>              (Vec(425, 170), module, StepWave::SLEW_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>    (Vec(425, 200), module, StepWave::SLEW_INPUT));

        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(425, 75), module, StepWave::CV_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(425, 300), module, StepWave::GATE_OUTPUT));
        
        // Create and add the WaveDisplay
        WaveDisplay* waveDisplay = createWidget<WaveDisplay>(Vec(50.5,50)); // Positioning
        waveDisplay->box.size = Vec(351, 50); // Size of the display widget
        waveDisplay->module = module;
        addChild(waveDisplay);
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);

        StepWave* StepWaveModule = dynamic_cast<StepWave*>(module);
        assert(StepWaveModule); // Ensure the cast succeeds

        // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());

        // TrackCV menu item
        struct TrackCVMenuItem : MenuItem {
            StepWave* StepWaveModule;
            void onAction(const event::Action& e) override {
                // Toggle the "Track CV Values" mode
                StepWaveModule->trackCV = !StepWaveModule->trackCV;
            }
            void step() override {
                // Update the display to show a checkmark when the mode is active
                rightText = StepWaveModule->trackCV ? "✔" : "";
                MenuItem::step();
            }
        };
       
        TrackCVMenuItem* trackCVItem = new TrackCVMenuItem();
        trackCVItem->text = "Track CV Values";
        trackCVItem->StepWaveModule = StepWaveModule;
        menu->addChild(trackCVItem);
        
        // ShapeBeats menu item
        struct ShapeBeatsMenuItem : MenuItem {
            StepWave* StepWaveModule;
            void onAction(const event::Action& e) override {
                // Toggle the "Shape follows number of Beats" mode
                StepWaveModule->shapeBeats = !StepWaveModule->shapeBeats;
            }
            void step() override {
                // Update the display to show a checkmark when the mode is active
                rightText = StepWaveModule->shapeBeats ? "✔" : "";
                MenuItem::step();
            }
        };
                
        ShapeBeatsMenuItem* shapeBeatsItem = new ShapeBeatsMenuItem();
        shapeBeatsItem->text = "Shape follows number of Beats";
        shapeBeatsItem->StepWaveModule = StepWaveModule;
        menu->addChild(shapeBeatsItem);       
    }        
};

Model* modelStepWave = createModel<StepWave, StepWaveWidget>("StepWave");