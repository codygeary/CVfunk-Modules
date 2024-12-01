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
#include <cmath>

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

#include "Filter6pButter.h"
#define OVERSAMPLING_FACTOR 8 
class OverSamplingShaper {
public:
    OverSamplingShaper() {
        interpolatingFilter.setCutoffFreq(1.f / (OVERSAMPLING_FACTOR * 4));
        decimatingFilter.setCutoffFreq(1.f / (OVERSAMPLING_FACTOR * 4));
    }
    float process(float input) {
        float signal;
        for (int i = 0; i < OVERSAMPLING_FACTOR; ++i) {
            signal = (i == 0) ? input * OVERSAMPLING_FACTOR : 0.f;   
            signal = interpolatingFilter.process(signal);
            signal = processShape(signal);
            signal = decimatingFilter.process(signal);
        }
        return signal;
    }
private:
    virtual float processShape(float) = 0;
    Filter6PButter interpolatingFilter;
    Filter6PButter decimatingFilter;
};

// Define the OverSamplingShaper derived class
class SimpleShaper : public OverSamplingShaper {
private:
    float processShape(float input) override {
        // No additional shaping; just pass through
        return input;
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
        LINK_BUTTON, TRACK_BUTTON,
        NUM_PARAMS
    };
    enum InputIds {
        CLOCK_INPUT,        
        STEP_1_IN_VAL, STEP_2_IN_VAL, STEP_3_IN_VAL, STEP_4_IN_VAL,
        STEP_5_IN_VAL, STEP_6_IN_VAL, STEP_7_IN_VAL, STEP_8_IN_VAL,
        STEP_1_2_DISPLACE_IN, STEP_2_3_DISPLACE_IN, STEP_3_4_DISPLACE_IN, STEP_4_5_DISPLACE_IN,
        STEP_5_6_DISPLACE_IN, STEP_6_7_DISPLACE_IN, STEP_7_8_DISPLACE_IN,  
        SLEW_INPUT,
        ON_OFF_INPUT, RESET_INPUT, LINK_INPUT, TRACK_INPUT,                    
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
        LINK_LIGHT, TRACK_LIGHT,
        NUM_LIGHTS
    };

    // For Clocking 
    dsp::Timer SyncTimer;
    dsp::Timer ClockTimerA; 
    dsp::Timer ClockTimerB; 
    dsp::SchmittTrigger SyncTrigger;
    float SyncInterval[2] = {1.0f/60.f, 1.0f};
    bool firstClockPulse = true;

    // For Reset and ON/OFF
    dsp::SchmittTrigger resetTrigger;
    dsp::SchmittTrigger onOffTrigger;
    dsp::SchmittTrigger onOffButtonTrigger;

    bool sequenceRunning = true;  //default sequence run to on

    // For each stage
    int currentStage[2] = {0,0};
    float stepValues[8] = {0.0f};
    float sampledStepValue[2][8] =  {{0.0f}};
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
    float shapeValues[8] = {0.0f};  
    float displacementValues[7] = {0.0f};

    //For the display
    CircularBuffer<float, 1024> waveBuffers[3];
    float oscPhase[2] = {0.0f}; // Current oscillator phase for each channel
    float sequenceProgress = 0.0f;

    // Initialize Butterworth filter for oversampling
    SimpleShaper shaper;  // Instance of the oversampling and shaping processor
    Filter6PButter butterworthFilter;  // Butterworth filter instance

    // For the output
    dsp::SlewLimiter slewLimiterA; 
    dsp::SlewLimiter slewLimiterB; 
    float lastTargetVoltage[2] = {0.f,0.f};
    bool linkShapeBeats = false;
    bool waitForReset = true;
    bool linkButtonPressed = false;
    bool linkLatched = false;
    bool linkGateActive = false;

    bool trackCV = false;
    bool trackButtonPressed = false;
    bool trackLatched = false;
    bool trackGateActive = false;
    bool isSupersamplingEnabled = false;
    bool stageShapeCV = false;
    bool quantizeCVOut = false;
 
    //for gate output when sequence is not running
	dsp::PulseGenerator stepTrigger; 

    json_t* dataToJson() override {
        json_t* rootJ = json_object();

        // Save the state of linkLatched
        json_object_set_new(rootJ, "linkLatched", json_boolean(linkLatched));

        // Save the state of trackGateActive
        json_object_set_new(rootJ, "trackLatched", json_boolean(trackLatched));

        // Save the state of stageShapeCV
        json_object_set_new(rootJ, "stageShapeCV", json_boolean(stageShapeCV));
    
        // Save the state of sequenceRunning
        json_object_set_new(rootJ, "sequenceRunning", json_boolean(sequenceRunning));

        // Save the state of quantizeCVOut
        json_object_set_new(rootJ, "quantizeCVOut", json_boolean(quantizeCVOut));

        // Save the state of trackCV
        json_object_set_new(rootJ, "trackCV", json_boolean(trackCV));
    
        // Save the state of linkShapeBeats
        json_object_set_new(rootJ, "linkShapeBeats", json_boolean(linkShapeBeats));
        
        // Save the value of SyncInterval[1]
        json_object_set_new(rootJ, "SyncInterval1", json_real(SyncInterval[1]));

        // Save the value of stageDuration[1]
        json_object_set_new(rootJ, "stageDuration1", json_real(stageDuration[1]));

        // Save the value of currentStage[1]
        json_object_set_new(rootJ, "currentStage1", json_real(currentStage[1]));

        return rootJ;
    }
    
    void dataFromJson(json_t* rootJ) override {

        // Load the state of linkLatched
        json_t* linkLatchedJ = json_object_get(rootJ, "linkLatched");
        if (linkLatchedJ) {
            linkLatched = json_is_true(linkLatchedJ);
        }

        // Load the state of trackLatched
        json_t* trackLatchedJ = json_object_get(rootJ, "trackLatched");
        if (trackLatchedJ) {
            trackLatched = json_is_true(trackLatchedJ);
        }

        // Load the state of stageShapeCV
        json_t* stageShapeCVJ = json_object_get(rootJ, "stageShapeCV");
        if (stageShapeCVJ) {
            stageShapeCV = json_is_true(stageShapeCVJ);
        }

        // Load the state of sequenceRunning
        json_t* sequenceRunningJ = json_object_get(rootJ, "sequenceRunning");
        if (sequenceRunningJ) {
            sequenceRunning = json_is_true(sequenceRunningJ);
        }

        // Load the state of quantizeCVOut
        json_t* quantizeCVOutJ = json_object_get(rootJ, "quantizeCVOut");
        if (quantizeCVOutJ) {
            quantizeCVOut = json_is_true(quantizeCVOutJ);
        }
    
        // Load the state of trackCV
        json_t* trackCVJ = json_object_get(rootJ, "trackCV");
        if (trackCVJ) {
            trackCV = json_is_true(trackCVJ);
        }
    
        // Load the state of linkShapeBeats
        json_t* linkShapeBeatsJ = json_object_get(rootJ, "linkShapeBeats");
        if (linkShapeBeatsJ) {
            linkShapeBeats = json_is_true(linkShapeBeatsJ);
        }
        
        // Load the value of SyncInterval[1]
        json_t* SyncInterval1J = json_object_get(rootJ, "SyncInterval1");
        if (SyncInterval1J) {
            SyncInterval[1] = (float)json_real_value(SyncInterval1J);
        }

        // Load the value of stageDuration[1]
        json_t* stageDuration1J = json_object_get(rootJ, "stageDuration1");
        if (stageDuration1J) {
            stageDuration[1] = (float)json_real_value(stageDuration1J);
        }

        // Load the value of currentStage[1]
        json_t* currentStage1J = json_object_get(rootJ, "currentStage1");
        if (currentStage1J) {
            currentStage[1] = (float)json_real_value(currentStage1J);
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

        configParam(STEP_1_BEATS, 0.f, 10.f, 1.0f, "Stage 1 Beats");
        configParam(STEP_2_BEATS, 0.f, 10.f, 1.0f, "Stage 2 Beats");
        configParam(STEP_3_BEATS, 0.f, 10.f, 1.0f, "Stage 3 Beats");
        configParam(STEP_4_BEATS, 0.f, 10.f, 1.0f, "Stage 4 Beats");
        configParam(STEP_5_BEATS, 0.f, 10.f, 1.0f, "Stage 5 Beats");
        configParam(STEP_6_BEATS, 0.f, 10.f, 1.0f, "Stage 6 Beats");
        configParam(STEP_7_BEATS, 0.f, 10.f, 1.0f, "Stage 7 Beats");
        configParam(STEP_8_BEATS, 0.f, 10.f, 1.0f, "Stage 8 Beats");
 
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

        configInput(CLOCK_INPUT, "Clock");
        configInput(STEP_1_IN_VAL, "Stage 1 Value");
        configInput(STEP_2_IN_VAL, "Stage 2 Value");
        configInput(STEP_3_IN_VAL, "Stage 3 Value");
        configInput(STEP_4_IN_VAL, "Stage 4 Value");
        configInput(STEP_5_IN_VAL, "Stage 5 Value");
        configInput(STEP_6_IN_VAL, "Stage 6 Value");
        configInput(STEP_7_IN_VAL, "Stage 7 Value");
        configInput(STEP_8_IN_VAL, "Stage 8 Value");
        configInput(STEP_1_2_DISPLACE_IN, "Rhythmic Displacement 1-2");
        configInput(STEP_2_3_DISPLACE_IN, "Rhythmic Displacement 2-3");
        configInput(STEP_3_4_DISPLACE_IN, "Rhythmic Displacement 3-4");
        configInput(STEP_4_5_DISPLACE_IN, "Rhythmic Displacement 4-5");
        configInput(STEP_5_6_DISPLACE_IN, "Rhythmic Displacement 5-6");
        configInput(STEP_6_7_DISPLACE_IN, "Rhythmic Displacement 6-7");
        configInput(STEP_7_8_DISPLACE_IN, "Rhythmic Displacement 7-8");
        configInput(SLEW_INPUT, "Slew CV");
        configInput(ON_OFF_INPUT, "ON/OFF");
        configInput(RESET_INPUT, "Reset");
        configInput(LINK_INPUT, "Link Beats to Step");
        configInput(TRACK_INPUT, "Track Stage Value CV");
 
        configOutput(CV_OUTPUT, "Sequencer CV");
        configOutput(GATE_OUTPUT, "Sequencer Gate");
        configOutput(STEP_1_GATE_OUT, "Stage 1 Gate");
        configOutput(STEP_2_GATE_OUT, "Stage 2 Gate");
        configOutput(STEP_3_GATE_OUT, "Stage 3 Gate");
        configOutput(STEP_4_GATE_OUT, "Stage 4 Gate");
        configOutput(STEP_5_GATE_OUT, "Stage 5 Gate");
        configOutput(STEP_6_GATE_OUT, "Stage 6 Gate");
        configOutput(STEP_7_GATE_OUT, "Stage 7 Gate");
        configOutput(STEP_8_GATE_OUT, "Stage 8 Gate");

    }

    void process(const ProcessArgs &args) override {

        // Process clock sync input
        if (inputs[CLOCK_INPUT].isConnected()) {
            float SyncInputVoltage = inputs[CLOCK_INPUT].getVoltage();

            if (SyncTrigger.process(SyncInputVoltage)) {
                if (!firstClockPulse) {
                    SyncInterval[1] = SyncTimer.time; // Get the accumulated time since the last reset                         
					SyncTimer.reset(); // Reset the timer for the next trigger interval measurement
                }
                
                if (firstClockPulse && SyncTimer.time > SyncInterval[1]){
					firstClockPulse = false;
                }
            }
        } 

        // Check if the signal is at audio rate by comparing stageDuration[j]
        isSupersamplingEnabled = (SyncInterval[1] < 0.05f);

        // Process timers
        float deltaTimeA = args.sampleTime; //for the display clock
        float deltaTimeB = args.sampleTime; //for the synced clock

        SyncTimer.process(deltaTimeB); //measures the synced clock

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

        // Check for link button input and toggle
        if (params[LINK_BUTTON].getValue() > 0) {
            if (!linkButtonPressed) {
                linkLatched = !linkLatched;
                linkButtonPressed = true;
            }
        } else {
            linkButtonPressed = false;
        }

        // Determine gate states based on latched states and external gate presence
        linkGateActive = inputs[LINK_INPUT].isConnected() ? linkLatched ^ (inputs[LINK_INPUT].getVoltage() > 0.05f) : linkLatched;

        // Update link light based on latched state or external gate activity
        lights[LINK_LIGHT].setBrightness(linkGateActive ? 1.0 : 0.0);
        if (linkGateActive) {linkShapeBeats = true;} else { linkShapeBeats = false;}

        // Check for track button input and toggle
        if (params[TRACK_BUTTON].getValue() > 0) {
            if (!trackButtonPressed) {
                trackLatched = !trackLatched;
                trackButtonPressed = true;
            }
        } else {
            trackButtonPressed = false;
        }

        // Determine gate states based on latched states and external gate presence
        trackGateActive = inputs[TRACK_INPUT].isConnected() ? trackLatched ^ (inputs[TRACK_INPUT].getVoltage() > 0.05f) : trackLatched;

        // Update tracking light based on latched state or external gate activity
        lights[TRACK_LIGHT].setBrightness(trackGateActive ? 1.0 : 0.0);
        if (trackGateActive) {trackCV = true;} else { trackCV = false;}

        if (!sequenceRunning) {
            deltaTimeB = 0.f;
            // ClockTimerB.reset(); //don't reset the individual stage clock when the sequence is paused
            firstClockPulse = true;
        }

        int displacementChannels[7] = {0};   // Number of polyphonic channels for rhythmic displacement CV inputs
        int stageChannels[8] = {0};   // Number of polyphonic channels for stage value CV inputs
        
        // Arrays to store the current input signals and connectivity status
        // initialize all active channels with -1, indicating nothing connected.
        int activeDisplacementChannel[7] = {-1};   // Stores the number of the previous active channel for the rhythmic displacement CV 
        int activeStageChannel[8] = {-1};   // Stores the number of the previous active channel for the stage value CV     

        // Scan all inputs to determine the polyphony
        for (int i = 0; i < 8; i++) {            
            // Update the Rhythmic displacement CV channels
            if (i < 7){    //there are only 7 channels here vs 8
                if (inputs[STEP_1_2_DISPLACE_IN + i].isConnected()) {
                    displacementChannels[i] = inputs[STEP_1_2_DISPLACE_IN + i].getChannels();
                    activeDisplacementChannel[i] = i;
                } else if (i > 0 && activeDisplacementChannel[i-1] != -1) {
                    if (displacementChannels[activeDisplacementChannel[i-1]] >= (i - activeDisplacementChannel[i-1])) {
                        activeDisplacementChannel[i] = activeDisplacementChannel[i-1]; // Carry over the active channel
                    } else {
                        activeDisplacementChannel[i] = -1; // No valid polyphonic channel to carry over
                    }
                } else {
                    activeDisplacementChannel[i] = -1; // Explicitly reset if not connected
                }
            }
            
            if (inputs[STEP_1_IN_VAL + i].isConnected()) {
                stageChannels[i] = inputs[STEP_1_IN_VAL + i].getChannels();
                activeStageChannel[i] = i;
            } else if (i > 0 && activeStageChannel[i-1] != -1) {
                if (stageChannels[activeStageChannel[i-1]] > (i - activeStageChannel[i-1])) {
                    activeStageChannel[i] = activeStageChannel[i-1]; // Carry over the active channel
                } else {
                    activeStageChannel[i] = -1; // No valid polyphonic channel to carry over
                }
            } else {
                activeStageChannel[i] = -1; // Explicitly reset if not connected
            }            
        }
            
        for (int i = 0; i < 7; i++){
            if (activeDisplacementChannel[i]==i) {
                displacementValues[i] = inputs[STEP_1_2_DISPLACE_IN + i].getPolyVoltage(0);
            } else if (activeDisplacementChannel[i] > -1){
                // Now we compute which channel we need to grab
                int diffBetween = i - activeDisplacementChannel[i];
                int currentChannelMax =  displacementChannels[activeDisplacementChannel[i]] ;    
                if (currentChannelMax - diffBetween > 0) {    //If we are before the last poly channel
                    displacementValues[i] = inputs[STEP_1_2_DISPLACE_IN + activeDisplacementChannel[i]].getPolyVoltage(diffBetween); 
                }
            }
        }
  
        if (!stageShapeCV){
            // Override and animate stage level controls if external CV connected
            for (int i = 0; i < 8; i++){
                if (activeStageChannel[i]==i) {
                    stepValues[i] = clamp(inputs[STEP_1_IN_VAL + i].getVoltage(),-5.0f, 5.0f);
                    params[STEP_1_VAL + i].setValue(stepValues[i]);
                } else if (activeStageChannel[i] > -1){
                    //Now we compute which channel to grab
                    int diffBetween = i - activeStageChannel[i];
                    int currentChannelMax =  stageChannels[activeStageChannel[i]] ;    
                    
                    if (currentChannelMax - diffBetween > 0) {    //If we are before the last poly channel
                        stepValues[i] = clamp(inputs[STEP_1_IN_VAL + activeStageChannel[i]].getPolyVoltage(diffBetween), -5.0f, 5.0f); 
                        params[STEP_1_VAL + i].setValue(stepValues[i]);
                    }
                } else {                        
                    stepValues[i] = params[STEP_1_VAL + i].getValue();        
                }
                shapeValues[i] = params[STEP_1_SHAPE + i].getValue();
            }
        } else { // the CV input modulates shape instead
            for (int i = 0; i < 8; i++){
                if (activeStageChannel[i]==i) {
                    shapeValues[i] = clamp(inputs[STEP_1_IN_VAL + i].getVoltage() + params[STEP_1_SHAPE + i].getValue() , 1.0f, 12.0f);
                } else if (activeStageChannel[i] > -1){
                    //Now we compute which channel to grab
                    int diffBetween = i - activeStageChannel[i];
                    int currentChannelMax =  stageChannels[activeStageChannel[i]] ;    
                    
                    if (currentChannelMax - diffBetween > 0) {    //If we are before the last poly channel
                        shapeValues[i] = clamp(params[STEP_1_SHAPE + i].getValue() + inputs[STEP_1_IN_VAL + activeStageChannel[i]].getPolyVoltage(diffBetween), 1.0f, 12.0f);
                    }
                } else {                        
                    shapeValues[i] = params[STEP_1_SHAPE + i].getValue();
                }
                stepValues[i] = params[STEP_1_VAL + i].getValue();  
            }
        } 
          
        for (int j = 0; j < 2; j++){ //Cycle through the two different clock layers
              
            if (currentStage[j] == 0) { 
                previousStagesLength[j] = 0.0f;  // Reset at the beginning of the sequence
            }           
         
            float displacementZero = 0.0f;
            float displacementOne = 0.0f;
            float displacementTwo = 0.0f;
            float displacementLast = 0.0f;
            float stageStart = 0.0f; //track the position of the beginning of the current stage over 0...8

            //Compute rhythmic offsets
            if (currentStage[j] == 0){
                displacementZero = clamp ( params[STEP_1_2_DISPLACE].getValue() + displacementValues[0], -5.f, 5.f);
                stageDuration[j] = ((displacementZero / 10.f) + 1) * SyncInterval[j];
                if (j==1){ stageStart = 0.0f; }
            } else if (currentStage[j] > 0 && currentStage[j] < 7) {
                displacementOne = clamp ( params[STEP_1_2_DISPLACE + currentStage[j]].getValue() + displacementValues[currentStage[j]], -5.f, 5.f);
                displacementTwo = clamp ( params[STEP_1_2_DISPLACE + currentStage[j] - 1].getValue() + displacementValues[currentStage[j] - 1], -5.f, 5.f);        
                stageDuration[j] = ((displacementOne / 10.f ) - (displacementTwo / 10.f ) + 1) * SyncInterval[j];
                if (j==1){ stageStart = currentStage[j] + (displacementTwo / 10.f);}
            } else {
                displacementLast = clamp ( params[STEP_7_8_DISPLACE].getValue() + displacementValues[6], -5.f, 5.f);
                stageDuration[j] = SyncInterval[j] * (1 - displacementLast / 10.f );
                if (j==1){ stageStart = currentStage[j] + (displacementLast / 10.0f);}
            }        

            // Ensure stageDuration[j] does not become too small
            const float minStageDuration = 0.0001f; // Minimum value to prevent division by zero
            stageDuration[j] = fmax(stageDuration[j], minStageDuration);
    
            // Clock the stages and track progress through each stage
            if (j==0){ //display clock
                ClockTimerA.process(deltaTimeA);
                currentTime[0] = ClockTimerA.time;
                normallizedStageProgress[0] = currentTime[0]/stageDuration[0];
            } else {  //sequence clock
                ClockTimerB.process(deltaTimeB);
                currentTime[1] = ClockTimerB.time;
                normallizedStageProgress[1] = currentTime[1]/stageDuration[1];     
                sequenceProgress = stageStart + currentTime[1]/SyncInterval[1];
            }
	
			// Check if sequencer is resetting
			bool resetCondition = (inputs[RESET_INPUT].isConnected() && resetTrigger.process(inputs[RESET_INPUT].getVoltage())) || (params[RESET_BUTTON].getValue() > 0.1f);    
			if (resetCondition) {
				ClockTimerB.reset(); //reset sequencer clock
				currentStage[1] = 0; //set sequencer stage to 0
				currentStage[0] = 0; 
				sequenceProgress = 0.f; //reset progress bar
				sampledStepValue[1][0] = stepValues[0]; //recollect the sample at reset
                currentShape[1] = shapeValues[0];
			} 

		
            if (currentTime[j] >= stageDuration[j]){
                if (j==0){
                    ClockTimerA.reset();  //Reset master channel
                } else {
                    ClockTimerB.reset();  //Reset the sequencer channel
                }
                                
                currentStage[j] += 1; // advance the sequence when the time>duration
                           
                // Wrap the stage back to 0 at the end of the sequence
                if (currentStage[j] > 7) {
                    currentStage[j] = 0;
                    sequenceProgress = 0.f;
                } 

				sampledStepValue[j][currentStage[j]] = stepValues[currentStage[j]]; 
                currentShape[j] = shapeValues[currentStage[j]];
               
                previousStagesLength[j] += stageDuration[j] / SyncInterval[j]; // Accumulate the duration of each stage normalized to SyncInterval[j]
                normallizedStageProgress[j] = 0; //reset the current stage progress meter
                
                numBeats = floor( params[STEP_1_BEATS + currentStage[j]].getValue() );                
            }
      
            if (j==1){//only jump to button for the sequencer layer
                // Jump to step if step button is pushed
                for (int i = 0; i < 8; i++) {

                    if (stepButtonTrigger[i].process(params[STEP_1_BUTTON + i].getValue())) {
                        currentStage[j] = i;
                        sampledStepValue[j][currentStage[j]] = stepValues[currentStage[j]];
						stepTrigger.trigger(0.001f);  // Trigger a 1ms pulse (0.001 seconds)
                        
                        currentShape[j] = shapeValues[currentStage[j]];
                                              
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
                        
                        sequenceProgress = currentStage[j] - (displacementPrevious/10.f + 1);
                    }
                }
            }
  
            if (j==1){ //update stage lights based on only the sequencer clock
                // Stage Lights
                for (int i=0; i<8; i++){
                    if (currentStage[j] == i && sequenceRunning){
                        outputs[STEP_1_GATE_OUT + i ].setVoltage(10.0);
                        if (!isSupersamplingEnabled){
                            lights[STEP_1_GATE_LIGHT + i].setBrightness(1.0);
                            lights[STEP_1_VAL_LIGHT + i].setBrightness(1.0);  
                        }              
                    } else if (currentStage[j] == i){
                        outputs[STEP_1_GATE_OUT + i ].setVoltage(10.0);
                        if (!isSupersamplingEnabled){
                            lights[STEP_1_GATE_LIGHT + i].setBrightness(0.5); //dim lights when sequencer off
                            lights[STEP_1_VAL_LIGHT + i].setBrightness(0.25);
                        }
                    } else {
                        outputs[STEP_1_GATE_OUT + i ].setVoltage(0.0);
                        if (!isSupersamplingEnabled){
                            lights[STEP_1_GATE_LIGHT + i].setBrightness(0.0);
                            lights[STEP_1_VAL_LIGHT + i].setBrightness(0.25); 
                        }           
                    }
                    
                    if (isSupersamplingEnabled){
                        lights[STEP_1_GATE_LIGHT + i].setBrightness(0.5);
                        lights[STEP_1_VAL_LIGHT + i].setBrightness(0.5);                        
                    }
                }
            }    
         
            //CV and Gate computation
            if (linkShapeBeats){
                numBeats = floor( params[STEP_1_BEATS + currentStage[j]].getValue() );   
                if (numBeats < 1){numBeats = 1;} // setting the param to zero will only effect the GATE CV output.  
            } else { numBeats = 1;}   
                    
            frameLength[j] = stageDuration[j]/numBeats;
            splitTime[j] = currentTime[j];
            while( splitTime[j] > stageDuration[j]/numBeats ) { //Compute subdivisions for beats
                splitTime[j] -= stageDuration[j]/numBeats;
            }
            normallizedSplitTime[j] = splitTime[j]/(stageDuration[j]/numBeats);
    
            if (currentShape[j] == 1.f) { // Rectangle Shape
                if (!trackCV) {
                    finalCV[j] = sampledStepValue[j][currentStage[j]];  
                } else {
                    finalCV[j] = stepValues[currentStage[j]]; 
                }
            } 
            else if (currentShape[j] > 1.f && currentShape[j] <= 2.f) { // Morphing to Right-Leaning Sawtooth
                float sawFactor = normallizedSplitTime[j];
                float morphFactor = (currentShape[j] - 1.f); // From 0 to 1 as currentShape goes from 1 to 2
            
                if (!trackCV) {
                    finalCV[j] = sampledStepValue[j][currentStage[j]] * (1 - morphFactor) + sampledStepValue[j][currentStage[j]] * morphFactor * sawFactor;
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
                    finalCV[j] = sampledStepValue[j][currentStage[j]] * triangleValue;
                } else {
                    finalCV[j] = stepValues[currentStage[j]] * triangleValue;
                }
            }
            else if (currentShape[j] > 3.f && currentShape[j] <= 4.f) { // Morphing from Left-Leaning Triangle to Rectangle
                float morphFactor = currentShape[j] - 3.f; // Ranges from 0 to 1 as currentShape goes from 3 to 4
            
                // Triangle to rectangle morphing logic
                if (normallizedSplitTime[j] < 1.f - morphFactor) {
                    // Left part of the shape, which is still the triangle
                    finalCV[j] = sampledStepValue[j][currentStage[j]] * (1.f - normallizedSplitTime[j] / (1.f - morphFactor));
                } else {
                    // Right part morphing to a rectangle
                    finalCV[j] = 0.f;
                }
            }        ///REMINDER implement trackCV
            else if (currentShape[j] > 4.f && currentShape[j] <= 5.f) { // Pulse Width Modulation from Left-Sided PWM to Rectangle
                float morphFactor = currentShape[j] - 4.f; // Ranges from 0 to 1 as currentShape goes from 4 to 5
                
                // Calculate the pulse width position
                float pulseWidth = morphFactor; // Moves from narrow pulse (0) to full width (1)
                
                // Generate the PWM shape based on pulse width
                if (normallizedSplitTime[j] <= pulseWidth) {
                    finalCV[j] = sampledStepValue[j][currentStage[j]]; // High part of the pulse
                } else {
                    finalCV[j] = 0.f; // Low part of the pulse
                }
            }       ///REMINDER implement trackCV
            else if (currentShape[j] > 5.f && currentShape[j] <= 6.f) { // Morphing from Rectangle to Sine Wave
                float morphFactor = currentShape[j] - 5.f; // Ranges from 0 to 1 as currentShape goes from 5 to 6
                
                // Generate a sine wave and interpolate it with the rectangle
                float sineValue = (sinf(2.f * M_PI * normallizedSplitTime[j]));
                if (!trackCV) {
                    finalCV[j] = sampledStepValue[j][currentStage[j]] * ((1 - morphFactor) + sineValue * morphFactor);
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
                    finalCV[j] = sampledStepValue[j][currentStage[j]] * ((1 - morphFactor) * sineValue + morphFactor * squareValue);
                } else {
                    finalCV[j] = stepValues[currentStage[j]] * ((1 - morphFactor) * sineValue + morphFactor * squareValue);
                }
            }
            else if (currentShape[j] > 7.f && currentShape[j] <= 8.f) { // Morphing from Square Wave to Inverted Sawtooth
                float morphFactor = currentShape[j] - 7.f; // Ranges from 0 to 1 as currentShape goes from 7 to 8
                
                // Generate an inverted sawtooth wave and interpolate it with the square wave
                float squareValue = (normallizedSplitTime[j] < 0.5f) ? 1.f : -1.f;
                float sawtoothValue = 1.f - 2 * normallizedSplitTime[j];
                if (!trackCV) {
                    finalCV[j] = sampledStepValue[j][currentStage[j]] * ((1 - morphFactor) * squareValue + morphFactor * sawtoothValue);
                } else {
                    finalCV[j] = stepValues[currentStage[j]] * ((1 - morphFactor) * squareValue + morphFactor * sawtoothValue);
                }
            }
            else if (currentShape[j] > 8.f && currentShape[j] <= 9.f) { // Morphing from Inverted Sawtooth to Triangle
                float morphFactor = currentShape[j] - 8.f; // Ranges from 0 to 1 as currentShape goes from 8 to 9
                
                // Generate a triangle wave and interpolate it with the inverted sawtooth
                float sawtoothValue = 1.f - 2 * normallizedSplitTime[j];
                float triangleValue = (normallizedSplitTime[j] < 0.5f) ? (1-4.f * normallizedSplitTime[j]) : (1-4.f * (1.f - normallizedSplitTime[j]));
                if (!trackCV) {
                    finalCV[j] = sampledStepValue[j][currentStage[j]] * ((1 - morphFactor) * sawtoothValue + morphFactor * triangleValue);
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
                    finalCV[j] = sampledStepValue[j][currentStage[j]] * ((1 - morphFactor) * triangleValue + morphFactor * logRampValue);
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
                    finalCV[j] = sampledStepValue[j][currentStage[j]] * interpolatedValue;
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
                    finalCV[j] = sampledStepValue[j][currentStage[j]] * interpolatedValue;
                } else {
                    finalCV[j] = stepValues[currentStage[j]] * interpolatedValue;
                }
            }
  
			if (!sequenceRunning && j==1) { //if the sequencer if off, then preview the CV directly
				finalCV[1] = stepValues[currentStage[1]]; 
			}
       
            // Compute Slew and output CV voltages
            float slewRate = clamp(params[SLEW_PARAM].getValue() + inputs[SLEW_INPUT].getVoltage() / 10.f, 0.0f, 1.f);

            if (slewRate > 0) {
                // Calculate the absolute voltage difference from the last target
                float voltageDifference = fabs(finalCV[j] - lastTargetVoltage[j]);

                // Adjust slewSpeed based on the voltage difference and trigger interval
                float adjustedTriggerInterval = fmax(stageDuration[j], 1e-8f);
                float slewSpeed = voltageDifference / adjustedTriggerInterval; // Voltage difference per second

                // Apply the SLEW_PARAM knob to scale the slewSpeed
                slewSpeed *= 1.0f / (slewRate + 1e-8f);

                // Set the rise and fall speeds of the slew limiter to the calculated slew speed
                if (j == 0) {
                    slewLimiterA.setRiseFall(slewSpeed, slewSpeed);
                    slewedVoltage[j] = slewLimiterA.process(args.sampleTime, finalCV[j]);
                } else {
                    slewLimiterB.setRiseFall(slewSpeed, slewSpeed);
                    slewedVoltage[j] = slewLimiterB.process(args.sampleTime, finalCV[j]);
                }
            } else {
                // If no slew rate is applied, directly set the slewed voltage to the final CV
                slewedVoltage[j] = finalCV[j];
            }
             
            if (quantizeCVOut){
                slewedVoltage[j] = std::roundf(slewedVoltage[j]*12.f)/12.f;
            }
                             
            //Main Gate Output   
            numBeats = floor( params[STEP_1_BEATS + currentStage[j]].getValue() ); 
            float gateCV = 0.f;
            if (numBeats > 0){                         
                frameLength[j] = stageDuration[j]/numBeats;
                splitTime[j] = currentTime[j];
                while( splitTime[j] > stageDuration[j]/numBeats ) {
                    splitTime[j] -= stageDuration[j]/numBeats;
                }            
                
                if (splitTime[j] < frameLength[j]/2.f){
                    gateCV = 10.0f;
                } else {
                    gateCV = 0.f;
                }
                if (j==1){
                    if (sequenceRunning){
                        outputs[GATE_OUTPUT].setVoltage(gateCV);
                    } else {
						if (stepTrigger.process(args.sampleTime)) {
							outputs[GATE_OUTPUT].setVoltage(10.f);  // Output a 10V trigger
						} else {
							outputs[GATE_OUTPUT].setVoltage(0.0f);   // Otherwise, output 0V
						}                      
                    }
                }
            } else {
                gateCV = 0.f;
                outputs[GATE_OUTPUT].setVoltage(gateCV);                
            }                

            if (j == 1) {
                if (isSupersamplingEnabled) {
                    // Use the oversampling shaper for the signal
                    float outputValue = shaper.process(slewedVoltage[j]);

                    // Output the processed value
                    outputs[CV_OUTPUT].setVoltage(outputValue);                   
                    
                } else {
                    // If supersampling is not enabled, output the slewed voltage directly
                    outputs[CV_OUTPUT].setVoltage(slewedVoltage[j]);
                }

                // Update last target voltage after setting the output
                lastTargetVoltage[j] = slewedVoltage[j];
            } else {
                // Update last target voltage even if no output is set
                lastTargetVoltage[j] = slewedVoltage[j];
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

            if (!module->isSupersamplingEnabled) {
                // Draw the sequence progress bar
                float progressBarX = box.size.x * (module->sequenceProgress / 8.0f); // X position of the progress bar
                float progressBarWidth = 1.0f;  // Width of the progress bar
    
                // Draw a vertical rectangle as the progress bar
                nvgBeginPath(args.vg);
                nvgRect(args.vg, progressBarX, -box.size.y*0.2, progressBarWidth, box.size.y * 1.39); // Full height of the widget
                nvgFillColor(args.vg, nvgRGBAf(0.5f, 0.5f, 0.5f, 0.8f)); // Light grey color
                nvgFill(args.vg); // Fill the progress bar
            }

            drawWaveform(args, module->waveBuffers[0], nvgRGBAf(0.3, 0.3, 0.3, 0.8));
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

	struct DiscreteRoundBlackKnob : RoundBlackKnob {
		void onDragEnd(const DragEndEvent& e) override {
			ParamQuantity* paramQuantity = getParamQuantity();
			
			if (paramQuantity) {
				// Get the raw value from the knob
				float rawValue = paramQuantity->getValue();
				
				// Round the value to the nearest integer
				float discreteValue = std::roundf(rawValue);
				
				// Set the snapped value
				paramQuantity->setValue(discreteValue);
			}
			
			// Call the base class implementation to ensure proper behavior
			RoundBlackKnob::onDragEnd(e);
		}
	};
	


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

        addInput(createInputCentered<ThemedPJ301MPort>(Vec(25, 30), module, StepWave::CLOCK_INPUT));
        addParam(createParamCentered<TL1105>              (Vec(25, 110), module, StepWave::ON_OFF_BUTTON));
        addChild(createLightCentered<MediumLight<YellowLight>>(Vec(25, 110), module, StepWave::ON_OFF_LIGHT ));
        addInput(createInputCentered<ThemedPJ301MPort>    (Vec(25, 85), module, StepWave::ON_OFF_INPUT));

        addParam(createParamCentered<LEDButton>(Vec(48, 157), module, StepWave::TRACK_BUTTON));
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(48, 157), module, StepWave::TRACK_LIGHT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(25, 157), module, StepWave::TRACK_INPUT));


        addParam(createParamCentered<LEDButton>(Vec(48, 265), module, StepWave::LINK_BUTTON));
        addChild(createLightCentered<LargeLight<RedLight>>(Vec(48, 265), module, StepWave::LINK_LIGHT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(25, 265), module, StepWave::LINK_INPUT));


        addParam(createParamCentered<TL1105>              (Vec(25,310), module, StepWave::RESET_BUTTON));
        addInput(createInputCentered<ThemedPJ301MPort>    (Vec(25,335), module, StepWave::RESET_INPUT));

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
            addParam(createParamCentered<DiscreteRoundBlackKnob>(Vec(xPos, yPos), module, StepWave::STEP_1_BEATS + i));


            // Stage Gate and Button
            yPos += Spacing;
            addParam(createParamCentered<LEDButton>(Vec(xPos, yPos - 4), module, StepWave::STEP_1_BUTTON + i));
            addChild(createLightCentered<LargeLight<RedLight>>(Vec(xPos, yPos - 4), module, StepWave::STEP_1_GATE_LIGHT + i ));

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

        // Stage Value CV controls Shape menu item
        struct ShapeMenuItem : MenuItem {
            StepWave* StepWaveModule;
            void onAction(const event::Action& e) override {
                // Toggle the "Stage Value CV controls Shape" mode
                StepWaveModule->stageShapeCV = !StepWaveModule->stageShapeCV;
            }
            void step() override {
                // Update the display to show a checkmark when the mode is active
                rightText = StepWaveModule->stageShapeCV ? "✔" : "";
                MenuItem::step();
            }
        };
 
        ShapeMenuItem* stageShapeItem = new ShapeMenuItem();
        stageShapeItem->text = "Stage Value CV Modulates Shape";
        stageShapeItem->StepWaveModule = StepWaveModule;
        menu->addChild(stageShapeItem);
 
		menu->addChild(new MenuSeparator());
        
		// Create a new menu item for "Quantize CV Out"
		struct QuantizeCVMenuItem : MenuItem {
			StepWave* StepWaveModule;  // Pointer to your module
			void onAction(const event::Action& e) override {
				// Toggle the "Quantize CV Out" mode
				StepWaveModule->quantizeCVOut = !StepWaveModule->quantizeCVOut;
			}
			void step() override {
				// Show a checkmark when "Quantize CV Out" is active
				rightText = StepWaveModule->quantizeCVOut ? "✔" : "";
				MenuItem::step();
			}
		};
		
		// Add "Quantize CV Out" item to the menu
		QuantizeCVMenuItem* quantizeCVItem = new QuantizeCVMenuItem();
		quantizeCVItem->text = "Quantize CV Out";
		quantizeCVItem->StepWaveModule = StepWaveModule;  // Assign the module pointer
		menu->addChild(quantizeCVItem);		
                
    }              
      
};

Model* modelStepWave = createModel<StepWave, StepWaveWidget>("StepWave");