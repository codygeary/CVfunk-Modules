////////////////////////////////////////////////////////////
//
//   Pressed Duck
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   A stereo 6 channel mixer with compression and ducking.
//
////////////////////////////////////////////////////////////

#include "plugin.hpp"

struct SecondOrderHPF {
    float x1 = 0, x2 = 0; // previous two inputs
    float y1 = 0, y2 = 0; // previous two outputs
    float a0, a1, a2;     // filter coefficients for the input
    float b1, b2;         // filter coefficients for the output

    SecondOrderHPF() {}

    // Initialize the filter coefficients
    void setCutoffFrequency(float sampleRate, float cutoffFreq) {
        float w0 = 2 * M_PI * cutoffFreq / sampleRate;
        float cosw0 = cos(w0);
        float sinw0 = sin(w0);
        float alpha = sinw0 / 2 * sqrt(2);  // sqrt(2) results in a Butterworth filter

        float a = 1 + alpha;
        a = fmax(a, 0.00001f); //prevent div by zero
        a0 = (1 + cosw0) / 2 / a;
        a1 = -(1 + cosw0) / a;
        a2 = (1 + cosw0) / 2 / a;
        b1 = -2 * cosw0 / a;
        b2 = (1 - alpha) / a;
    }

    // Process the input sample
    float process(float input) {
        float output = a0 * input + a1 * x1 + a2 * x2 - b1 * y1 - b2 * y2;
        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = output;
        return output;
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

struct PressedDuck : Module {

    enum ParamIds {
        VOLUME1_PARAM, VOLUME2_PARAM, VOLUME3_PARAM, VOLUME4_PARAM, VOLUME5_PARAM, VOLUME6_PARAM,
        PAN1_PARAM, PAN2_PARAM, PAN3_PARAM, PAN4_PARAM, PAN5_PARAM, PAN6_PARAM,
        SIDECHAIN_VOLUME_PARAM, DUCK_PARAM, DUCK_ATT,
        PRESS_PARAM, PRESS_ATT, MASTER_VOL, MASTER_VOL_ATT, FEEDBACK_PARAM, FEEDBACK_ATT,
        MUTE1_PARAM, MUTE2_PARAM, MUTE3_PARAM, MUTE4_PARAM, MUTE5_PARAM, MUTE6_PARAM, MUTESIDE_PARAM,
        NUM_PARAMS
    };
    enum InputIds {
        AUDIO_1L_INPUT, AUDIO_1R_INPUT, AUDIO_2L_INPUT, AUDIO_2R_INPUT,
        AUDIO_3L_INPUT, AUDIO_3R_INPUT, AUDIO_4L_INPUT, AUDIO_4R_INPUT,
        AUDIO_5L_INPUT, AUDIO_5R_INPUT, AUDIO_6L_INPUT, AUDIO_6R_INPUT,
        VCA_CV1_INPUT, VCA_CV2_INPUT, VCA_CV3_INPUT, VCA_CV4_INPUT, VCA_CV5_INPUT, VCA_CV6_INPUT, VCA_SIDECHAIN_INPUT,
        PAN_CV1_INPUT, PAN_CV2_INPUT, PAN_CV3_INPUT, PAN_CV4_INPUT, PAN_CV5_INPUT, PAN_CV6_INPUT,
        SIDECHAIN_INPUT_L, SIDECHAIN_INPUT_R, DUCK_CV, PRESS_CV_INPUT, FEEDBACK_CV, MASTER_VOL_CV,
        MUTE_1_INPUT, MUTE_2_INPUT, MUTE_3_INPUT, MUTE_4_INPUT, MUTE_5_INPUT, MUTE_6_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        AUDIO_OUTPUT_L, AUDIO_OUTPUT_R,
        NUM_OUTPUTS
    };
    enum LightIds {
        VOLUME1_LIGHT, VOLUME2_LIGHT, VOLUME3_LIGHT, VOLUME4_LIGHT, VOLUME5_LIGHT, VOLUME6_LIGHT, BASS_VOLUME_LIGHT,

        MUTE1_LIGHT, MUTE2_LIGHT, MUTE3_LIGHT, MUTE4_LIGHT, MUTE5_LIGHT, MUTE6_LIGHT, MUTESIDE_LIGHT,

        PRESS_LIGHT1L,  PRESS_LIGHT2L,  PRESS_LIGHT3L,  PRESS_LIGHT4L,  PRESS_LIGHT5L,
        PRESS_LIGHT6L,  PRESS_LIGHT7L,  PRESS_LIGHT8L,  PRESS_LIGHT9L,  PRESS_LIGHT10L,
        PRESS_LIGHT11L,  PRESS_LIGHT12L,  PRESS_LIGHT13L,  PRESS_LIGHT14L,  PRESS_LIGHT15L,
        PRESS_LIGHT16L,  PRESS_LIGHT17L,  PRESS_LIGHT18L,  PRESS_LIGHT19L,  PRESS_LIGHT20L,
        PRESS_LIGHT1R,  PRESS_LIGHT2R,  PRESS_LIGHT3R,  PRESS_LIGHT4R,  PRESS_LIGHT5R,
        PRESS_LIGHT6R,  PRESS_LIGHT7R,  PRESS_LIGHT8R,  PRESS_LIGHT9R,  PRESS_LIGHT10R,
        PRESS_LIGHT11R,  PRESS_LIGHT12R,  PRESS_LIGHT13R,  PRESS_LIGHT14R,  PRESS_LIGHT15R,
        PRESS_LIGHT16R,  PRESS_LIGHT17R,  PRESS_LIGHT18R,  PRESS_LIGHT19R,  PRESS_LIGHT20R,

        FEED_LIGHT1L, FEED_LIGHT2L, FEED_LIGHT3L, FEED_LIGHT4L, FEED_LIGHT5L,
        FEED_LIGHT6L, FEED_LIGHT7L, FEED_LIGHT8L, FEED_LIGHT9L, FEED_LIGHT10L,
        FEED_LIGHT11L, FEED_LIGHT12L, FEED_LIGHT13L, FEED_LIGHT14L, FEED_LIGHT15L,
        FEED_LIGHT16L, FEED_LIGHT17L, FEED_LIGHT18L, FEED_LIGHT19L, FEED_LIGHT20L,
        FEED_LIGHT1R, FEED_LIGHT2R, FEED_LIGHT3R, FEED_LIGHT4R, FEED_LIGHT5R,
        FEED_LIGHT6R, FEED_LIGHT7R, FEED_LIGHT8R, FEED_LIGHT9R, FEED_LIGHT10R,
        FEED_LIGHT11R, FEED_LIGHT12R, FEED_LIGHT13R, FEED_LIGHT14R, FEED_LIGHT15R,
        FEED_LIGHT16R, FEED_LIGHT17R, FEED_LIGHT18R, FEED_LIGHT19R, FEED_LIGHT20R,

        VOL_LIGHT1, VOL_LIGHT2, VOL_LIGHT3, VOL_LIGHT4, VOL_LIGHT5,
        VOL_LIGHT6, VOL_LIGHT7, VOL_LIGHT8, VOL_LIGHT9, VOL_LIGHT10,
        VOL_LIGHT11, VOL_LIGHT12, VOL_LIGHT13, VOL_LIGHT14, VOL_LIGHT15,
        VOL_LIGHT16, VOL_LIGHT17, VOL_LIGHT18, VOL_LIGHT19, VOL_LIGHT20,
        VOL_LIGHT1R, VOL_LIGHT2R, VOL_LIGHT3R, VOL_LIGHT4R, VOL_LIGHT5R,
        VOL_LIGHT6R, VOL_LIGHT7R, VOL_LIGHT8R, VOL_LIGHT9R, VOL_LIGHT10R,
        VOL_LIGHT11R, VOL_LIGHT12R, VOL_LIGHT13R, VOL_LIGHT14R, VOL_LIGHT15R,
        VOL_LIGHT16R, VOL_LIGHT17R, VOL_LIGHT18R, VOL_LIGHT19R, VOL_LIGHT20R,
        NUM_LIGHTS
    };

    bool applyFilters = true; // Filter out DC is on by default

    // Initialize Butterworth filter for oversampling
    SimpleShaper shaperL;  // Instance of the oversampling and shaping processor
    SimpleShaper shaperR;  // Instance of the oversampling and shaping processor
    Filter6PButter butterworthFilter;  // Butterworth filter instance
    bool isSupersamplingEnabled = false;  // Enable supersampling is off by default

    //for tracking the mute state of each channel
    bool muteLatch[7] = {false,false,false,false,false,false,false};
    bool muteState[7] = {false,false,false,false,false,false,false};
    bool muteStatePrevious[7] = {false,false,false,false,false,false,false};

    dsp::SchmittTrigger muteButton[7];
    dsp::SchmittTrigger muteButtonInput[7];

    // For mute transition
    float transitionSamples = 100.f; // Number of samples to complete the transition, updated in config
    float fadeLevel[7] = {1.0f};
    int transitionCount[7] = {0};  // Array to track transition progress for each channel

    bool mutedSideDucks = false;

    alignas(std::atomic<bool>) std::atomic<bool> isShifted[6]; // For shift modified detection on the mute buttons

    // Serialization method to save module state
    json_t* dataToJson() override {
        json_t* rootJ = json_object();

        // Save the state of applyFilters as a boolean
        json_object_set_new(rootJ, "applyFilters", json_boolean(applyFilters));

        // Save the state of mutedSideDucks as a boolean
        json_object_set_new(rootJ, "mutedSideDucks", json_boolean(mutedSideDucks));

        // Save the state of isSupersamplingEnabled as a boolean
        json_object_set_new(rootJ, "isSupersamplingEnabled", json_boolean(isSupersamplingEnabled));

        // Save the muteLatch and muteState arrays
        json_t* muteLatchJ = json_array();
        json_t* muteStateJ = json_array();
        json_t* fadeLevelJ = json_array();
        json_t* transitionCountJ = json_array();

        for (int i = 0; i < 7; i++) {
            json_array_append_new(muteLatchJ, json_boolean(muteLatch[i]));
            json_array_append_new(muteStateJ, json_boolean(muteState[i]));
            json_array_append_new(fadeLevelJ, json_real(fadeLevel[i]));
            json_array_append_new(transitionCountJ, json_integer(transitionCount[i]));

        }

        json_object_set_new(rootJ, "muteLatch", muteLatchJ);
        json_object_set_new(rootJ, "muteState", muteStateJ);
        json_object_set_new(rootJ, "fadeLevel", fadeLevelJ);
        json_object_set_new(rootJ, "transitionCount", transitionCountJ);

        return rootJ;
    }

    // Deserialization method to load module state
    void dataFromJson(json_t* rootJ) override {
        // Load the state of applyFilters
        json_t* applyFiltersJ = json_object_get(rootJ, "applyFilters");
        if (applyFiltersJ) {
            applyFilters = json_is_true(applyFiltersJ);
        }

        // Load the state of mutedSideDucks
        json_t* mutedSideDucksJ = json_object_get(rootJ, "mutedSideDucks");
        if (mutedSideDucksJ) {
            mutedSideDucks = json_is_true(mutedSideDucksJ);
        }

        // Load the state of isSupersamplingEnabled
        json_t* isSupersamplingEnabledJ = json_object_get(rootJ, "isSupersamplingEnabled");
        if (isSupersamplingEnabledJ) {
            isSupersamplingEnabled = json_is_true(isSupersamplingEnabledJ);
        }

        // Load muteLatch and muteState arrays
        json_t* muteLatchJ = json_object_get(rootJ, "muteLatch");
        json_t* muteStateJ = json_object_get(rootJ, "muteState");
        json_t* fadeLevelJ = json_object_get(rootJ, "fadeLevel");
        json_t* transitionCountJ = json_object_get(rootJ, "transitionCount");

        if (muteLatchJ) {
            for (size_t i = 0; i < json_array_size(muteLatchJ) && i < 7; i++) {
                json_t* muteLatchValue = json_array_get(muteLatchJ, i);
                if (muteLatchValue) {
                    muteLatch[i] = json_is_true(muteLatchValue);
                }
            }
        }

        if (muteStateJ) {
            for (size_t i = 0; i < json_array_size(muteStateJ) && i < 7; i++) {
                json_t* muteStateValue = json_array_get(muteStateJ, i);
                if (muteStateValue) {
                    muteState[i] = json_is_true(muteStateValue);
                }
            }
        }

        if (fadeLevelJ) {
            for (size_t i = 0; i < json_array_size(fadeLevelJ) && i < 7; i++) {
                json_t* fadeLevelValue = json_array_get(fadeLevelJ, i);
                if (fadeLevelValue) {
                    fadeLevel[i] = json_real_value(fadeLevelValue);  // Use json_real_value for float
                }
            }
        }

        if (transitionCountJ) {
            for (size_t i = 0; i < json_array_size(transitionCountJ) && i < 7; i++) {
                json_t* transitionCountValue = json_array_get(transitionCountJ, i);
                if (transitionCountValue) {
                    transitionCount[i] = json_integer_value(transitionCountValue);  // Use json_integer_value for int
                }
            }
        }
                
    }

    // Variables for envelope followers and lights
    float sidePeakL = 0.0f;
    float sidePeakR = 0.0f;
    float envPeakL[6] = {0.0f};
    float envPeakR[6] = {0.0f};
    float envelopeL[6] = {0.0f};
    float envelopeR[6] = {0.0f};

    int cycleCount = 0;
    float pressTotalL = 1.0f;
    float pressTotalR = 1.0f;
    float distortTotalL = 1.0f;
    float distortTotalR = 1.0f;
    float pressTotal = 1.0f;
    float distortTotal = 1.0f;
    float volTotalL = 1.0f;
    float volTotalR = 1.0f;

    // Arrays to hold last computed values for differentiation
    float lastOutputL = 0.0f;
    float lastOutputR = 0.0f;
    float sideEnvelopeL = 0.0f;
    float sideEnvelopeR = 0.0f;
    float sideEnvelope = 0.0f;
    float inputL[6] = {0.0f};
    float inputR[6] = {0.0f};
    float panL[6] = {0.0f};
    float panR[6] = {0.0f};
    float lastPan[6] = {0.0f};
    bool initialized[6] = {false};
    float filteredEnvelopeL[6] = {0.0f};
    float filteredEnvelopeR[6] = {0.0f};
    float filteredEnvelope[6] = {0.0f};
    float filteredSideEnvelopeL = 0.0f;
    float filteredSideEnvelopeR = 0.0f;

    float alpha = 0.01f;

    // For filters
    float lastInputL = 0.0f;
    float lastInputR = 0.0f;
    float lastHPOutputL = 0.0f;
    float lastHPOutputR = 0.0f;
    float lastLPOutputL = 0.0f;
    float lastLPOutputR = 0.0f;

    // Declare high-pass filter
    SecondOrderHPF hpfL, hpfR;

    PressedDuck() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        // Configure volume and pan parameters for each channel
        configParam(VOLUME1_PARAM, 0.f, 2.f, 1.0f, "Channel 1 Volume");
        configParam(VOLUME2_PARAM, 0.f, 2.f, 1.0f, "Channel 2 Volume");
        configParam(VOLUME3_PARAM, 0.f, 2.f, 1.0f, "Channel 3 Volume");
        configParam(VOLUME4_PARAM, 0.f, 2.f, 1.0f, "Channel 4 Volume");
        configParam(VOLUME5_PARAM, 0.f, 2.f, 1.0f, "Channel 5 Volume");
        configParam(VOLUME6_PARAM, 0.f, 2.f, 1.0f, "Channel 6 Volume");
        configParam(MASTER_VOL, 0.f, 2.f, 1.0f, "Master Volume");
        configParam(FEEDBACK_PARAM, 0.f, 11.f, 0.0f, "Feedback");

        configParam(PAN1_PARAM, -1.f, 1.f, 0.f, "Channel 1 Pan");
        configParam(PAN2_PARAM, -1.f, 1.f, 0.f, "Channel 2 Pan");
        configParam(PAN3_PARAM, -1.f, 1.f, 0.f, "Channel 3 Pan");
        configParam(PAN4_PARAM, -1.f, 1.f, 0.f, "Channel 4 Pan");
        configParam(PAN5_PARAM, -1.f, 1.f, 0.f, "Channel 5 Pan");
        configParam(PAN6_PARAM, -1.f, 1.f, 0.f, "Channel 6 Pan");

        configParam(MUTE1_PARAM, 0.0, 1.0, 0.0, "Mute 1" );
        configParam(MUTE2_PARAM, 0.0, 1.0, 0.0, "Mute 2" );
        configParam(MUTE3_PARAM, 0.0, 1.0, 0.0, "Mute 3" );
        configParam(MUTE4_PARAM, 0.0, 1.0, 0.0, "Mute 4" );
        configParam(MUTE5_PARAM, 0.0, 1.0, 0.0, "Mute 5" );
        configParam(MUTE6_PARAM, 0.0, 1.0, 0.0, "Mute 6" );
        configParam(MUTESIDE_PARAM, 0.0, 1.0, 0.0, "Mute Sidechain" );

        // Configure side and saturation parameters
        configParam(SIDECHAIN_VOLUME_PARAM, 0.f, 2.f, 0.6f, "Sidechain Volume");
        configParam(DUCK_PARAM, 0.f, 1.f, 0.7f, "Duck Amount");
        configParam(DUCK_ATT, -1.f, 1.f, 0.0f, "Duck Att.");
        configParam(FEEDBACK_ATT, -1.f, 1.f, 0.0f, "Feedback Att.");
        configParam(MASTER_VOL_ATT, -1.f, 1.f, 0.0f, "Master Volume Att.");

        configParam(PRESS_PARAM, 0.f, 1.f, 0.f, "Press");
        configParam(PRESS_ATT, -1.f, 1.f, 0.0f, "Press Att.");

        // Configure inputs for each channel
#ifdef METAMODULE    
        configInput(AUDIO_1L_INPUT, "Chan. 1 L");
        configInput(AUDIO_1R_INPUT, "Chan. 1 R");
#else
        configInput(AUDIO_1L_INPUT, "Chan. 1 L / Poly");
        configInput(AUDIO_1R_INPUT, "Chan. 1 R / Poly");
#endif
        configInput(AUDIO_2L_INPUT, "Chan. 2 L");
        configInput(AUDIO_2R_INPUT, "Chan. 2 R");
        configInput(AUDIO_3L_INPUT, "Chan. 3 L");
        configInput(AUDIO_3R_INPUT, "Chan. 3 R");
        configInput(AUDIO_4L_INPUT, "Chan. 4 L");
        configInput(AUDIO_4R_INPUT, "Chan. 4 R");
        configInput(AUDIO_5L_INPUT, "Chan. 5 L");
        configInput(AUDIO_5R_INPUT, "Chan. 5 R");
        configInput(AUDIO_6L_INPUT, "Chan. 6 L");
        configInput(AUDIO_6R_INPUT, "Chan. 6 R");

#ifdef METAMODULE    
        configInput(VCA_CV1_INPUT, "Chan. 1 VCA CV");
#else
        configInput(VCA_CV1_INPUT, "Chan. 1 VCA CV / Poly");
#endif
        configInput(VCA_CV2_INPUT, "Chan. 2 VCA CV");
        configInput(VCA_CV3_INPUT, "Chan. 3 VCA CV");
        configInput(VCA_CV4_INPUT, "Chan. 4 VCA CV");
        configInput(VCA_CV5_INPUT, "Chan. 5 VCA CV");
        configInput(VCA_CV6_INPUT, "Chan. 6 VCA CV");
        configInput(VCA_SIDECHAIN_INPUT, "Sidechain VCA CV");

#ifdef METAMODULE    
        configInput(PAN_CV1_INPUT, "Channel 1 Pan CV");
#else
        configInput(PAN_CV1_INPUT, "Channel 1 Pan CV / Poly");
#endif        
        configInput(PAN_CV2_INPUT, "Channel 2 Pan CV");
        configInput(PAN_CV3_INPUT, "Channel 3 Pan CV");
        configInput(PAN_CV4_INPUT, "Channel 4 Pan CV");
        configInput(PAN_CV5_INPUT, "Channel 5 Pan CV");
        configInput(PAN_CV6_INPUT, "Channel 6 Pan CV");

#ifdef METAMODULE    
        configInput(MUTE_1_INPUT, "Channel 1 Mute CV");
#else
        configInput(MUTE_1_INPUT, "Channel 1 Mute CV / Poly");
#endif              
        configInput(MUTE_2_INPUT, "Channel 2 Mute CV");
        configInput(MUTE_3_INPUT, "Channel 3 Mute CV");
        configInput(MUTE_4_INPUT, "Channel 4 Mute CV");
        configInput(MUTE_5_INPUT, "Channel 5 Mute CV");
        configInput(MUTE_6_INPUT, "Channel 6 Mute CV");

        // Side and saturation CV inputs
        configInput(SIDECHAIN_INPUT_L, "Sidechain L In");
        configInput(SIDECHAIN_INPUT_R, "Sidechain R In");
        configInput(DUCK_CV, "Duck CV");
        configInput(PRESS_CV_INPUT, "Press CV");
        configInput(FEEDBACK_CV, "Feedback CV");
        configInput(MASTER_VOL_CV, "Master Vol. CV");

        // Outputs
        configOutput(AUDIO_OUTPUT_L, "Main Out L");
        configOutput(AUDIO_OUTPUT_R, "Main Out R");

        for (int i = 0; i < 6; ++i) {
            isShifted[i].store(false);
        }

        transitionSamples = 0.01 * APP->engine->getSampleRate(); // 10 ms * sample rate
     }

    void onSampleRateChange() override {
         float sampleRate = APP->engine->getSampleRate();
         transitionSamples = 0.01 * sampleRate; // 10 ms * sample rate
    }

    void onReset(const ResetEvent& e) override {
        // Reset all parameters
        Module::onReset(e);

        for (int i=0; i<7; i++){
			muteLatch[i] = false;
			muteState[i] = false;
			muteStatePrevious[i] = false;
		}       
    }

    void process(const ProcessArgs& args) override {
        float mixL = 0.0f;
        float mixR = 0.0f;
        float sampleRate = args.sampleRate;

        // Setup filters
        hpfL.setCutoffFrequency(args.sampleRate, 30.0f); // Set cutoff frequency
        hpfR.setCutoffFrequency(args.sampleRate, 30.0f);

        // Calculate scale factor based on the current sample rate
        float scaleFactor = sampleRate / 96000.0f; // Reference sample rate (96 kHz)

        // Adjust alpha and decayRate based on sample rate
        alpha = 0.01f / scaleFactor;  // Smoothing factor for envelope
        float decayRate = pow(0.999f, scaleFactor);  // Decay rate adjusted for sample rate

        float compressionAmountL = 0.0f;
        float compressionAmountR = 0.0f;
        float inputCount = 0.0f;

        // Check if the channel has polyphonic input
        int audioChannels[6] = {0}; // Number of polyphonic channels for AUDIO inputs
        int lChannels[6] = {0}; int rChannels[6] = {0};
        int vcaChannels[6] = {0}; // Number of polyphonic channels for VCA CV inputs
        int panChannels[6] = {0};   // Number of polyphonic channels for PAN CV inputs
        int muteChannels[6] = {0};  // Number of polyphonic channels for MUTE inputs

        // Arrays to store the current input signals and connectivity status
        int activeAudio[6] = {-1, -1, -1, -1, -1, -1};        // Stores the number of the previous active channel for the AUDIO inputs
        int activeVcaChannel[6] = {-1, -1, -1, -1, -1, -1}; // Stores the number of the previous active channel for the VCA CV
        int activePanChannel[6] = {-1, -1, -1, -1, -1, -1};   // Stores the number of the previous active channel for the PAN CV
        int activeMuteChannel[6] = {-1, -1, -1, -1, -1, -1};  // Stores the number of the previous active channel for the MUTE
        //initialize all active channels with -1, indicating nothing connected.

        // Scan all inputs to determine the polyphony
        for (int i = 0; i < 6; i++) {

            // Check if L input is connected and get its number of channels
            if (inputs[AUDIO_1L_INPUT + 2 * i].isConnected()) {
                lChannels[i] = inputs[AUDIO_1L_INPUT + 2 * i].getChannels();
            }

            // Check if R input is connected and get its number of channels
            if (inputs[AUDIO_1R_INPUT + 2 * i].isConnected()) {
                rChannels[i] = inputs[AUDIO_1R_INPUT + 2 * i].getChannels();
            }

            // Determine the maximum number of channels between L and R
            audioChannels[i] = std::max(lChannels[i], rChannels[i]);

            // Handle polyphonic AUDIO input distribution
            if (audioChannels[i] > 0) {
                activeAudio[i] = i;
            } else if (i > 0 && activeAudio[i-1] != -1) {
                if (audioChannels[activeAudio[i-1]] > (i - activeAudio[i-1])) {
                    activeAudio[i] = activeAudio[i-1]; // Carry over the active channel
                } else {
                    activeAudio[i] = -1; // No valid polyphonic channel to carry over
                }
            } else {
                activeAudio[i] = -1; // Explicitly reset if not connected
            }

            // Update the VCA CV channels
            if (inputs[VCA_CV1_INPUT + i].isConnected()) {
                vcaChannels[i] = inputs[VCA_CV1_INPUT + i].getChannels();
                activeVcaChannel[i] = i;
            } else if (i > 0 && activeVcaChannel[i-1] != -1) {
                if (vcaChannels[activeVcaChannel[i-1]] > (i - activeVcaChannel[i-1])) {
                    activeVcaChannel[i] = activeVcaChannel[i-1]; // Carry over the active channel
                } else {
                    activeVcaChannel[i] = -1; // No valid polyphonic channel to carry over
                }
            } else {
                activeVcaChannel[i] = -1; // Explicitly reset if not connected
            }

            // Update the PAN CV channels
            if (inputs[PAN_CV1_INPUT + i].isConnected()) {
                panChannels[i] = inputs[PAN_CV1_INPUT + i].getChannels();
                activePanChannel[i] = i;
            } else if (i > 0 && activePanChannel[i-1] != -1) {
                if (panChannels[activePanChannel[i-1]] > (i - activePanChannel[i-1])) {
                    activePanChannel[i] = activePanChannel[i-1]; // Carry over the active channel
                } else {
                    activePanChannel[i] = -1; // No valid polyphonic channel to carry over
                }
            } else {
                activePanChannel[i] = -1; // Explicitly reset if not connected
            }

            // Update the MUTE channels
            if (inputs[MUTE_1_INPUT + i].isConnected()) {
                muteChannels[i] = inputs[MUTE_1_INPUT + i].getChannels();
                activeMuteChannel[i] = i;
            } else if (i > 0 && activeMuteChannel[i-1] != -1) {
                if (muteChannels[activeMuteChannel[i-1]] > (i - activeMuteChannel[i-1])) {
                    activeMuteChannel[i] = activeMuteChannel[i-1];
                } else {
                    activeMuteChannel[i] = -1; // No valid polyphonic channel to carry over
                }
            } else {
                activeMuteChannel[i] = -1; // Explicitly reset if not connected
            }
        }

		// Process each of the six main channels
		for (int i = 0; i < 6; i++) {
		
			bool muteButtonPressed = muteButton[i].process(params[MUTE1_PARAM + i].getValue());
			bool muteInput = muteButtonInput[i].process(inputs[MUTE_1_INPUT + i].getVoltage());
			bool shiftHeld = !isShifted[i].load(); // un-invert
			
			if (shiftHeld && muteButtonPressed) {
				// Shift-click: solo logic
				bool thisChannelSoloing = true;
				for (int j = 0; j < 6; j++) {
					if (j != i && !muteState[j]) {
						thisChannelSoloing = false;
						break;
					}
				}
			
				if (!thisChannelSoloing) {
					// Solo this channel: mute all others
					for (int j = 0; j < 6; j++) {
						muteState[j] = (j != i);
					}
				} else {
					// Already soloing: unmute all
					for (int j = 0; j < 6; j++) {
						muteState[j] = false;
					}
				}
			} else if ((!shiftHeld && muteButtonPressed) || muteInput) {
				// Normal click: just toggle this channel (even if last one)
				if (!muteLatch[i]) {
					muteLatch[i] = true;
					muteState[i] = !muteState[i];  // simple toggle
				}
			} else {
				// Button released: reset latch
				muteLatch[i] = false;
			}

			if (muteStatePrevious[i] != muteState[i]) {
				muteStatePrevious[i] = muteState[i];
				transitionCount[i] = transitionSamples;  // reset transition
			}
				
			// -----------------------------
			// Now check if the channel has an active audio source
			// -----------------------------
			bool hasSource = false;
			int base = activeAudio[i];
			if (base >= 0) {
				int diff = i - base;
				if (diff >= 0 && diff < audioChannels[base]) {
					hasSource = true;
				}
			}
		
			if (!hasSource) {
				// Force silence and reset state, but keep mute info updated
				inputL[i] = inputR[i] = 0.0f;
				filteredEnvelopeL[i] = filteredEnvelopeR[i] = filteredEnvelope[i] = 0.0f;
				fadeLevel[i] = 0.0f;
				transitionCount[i] = 0;
				initialized[i] = false;
				lastPan[i] = 0.f;
				continue; // skip DSP, but mute state is preserved
			}
		
			// -----------------------------
			// Read polyphonic audio input
			// -----------------------------
			bool inputActive = false;
			bool baseHasL = false;
			bool baseHasR = false;
		
			if (base >= 0) {
				int diff = i - base;
				int baseChannels = audioChannels[base];
				if (diff >= 0 && diff < baseChannels) {
					inputActive = true;
					baseHasL = (lChannels[base] > 0);
					baseHasR = (rChannels[base] > 0);
		
					if (baseHasL && baseHasR) {
						inputL[i] = inputs[AUDIO_1L_INPUT + 2 * base].getPolyVoltage(diff);
						inputR[i] = inputs[AUDIO_1R_INPUT + 2 * base].getPolyVoltage(diff);
					} else if (baseHasL) {
						float v = inputs[AUDIO_1L_INPUT + 2 * base].getPolyVoltage(diff);
						inputL[i] = inputR[i] = v;
					} else if (baseHasR) {
						float v = inputs[AUDIO_1R_INPUT + 2 * base].getPolyVoltage(diff);
						inputL[i] = inputR[i] = v;
					} else {
						inputL[i] = inputR[i] = 0.f;
						inputActive = false;
					}
				}
			}
		
			// -----------------------------
			// Handle fade transition
			// -----------------------------
			if (transitionCount[i] > 0) {
				float step = 1.0f / transitionSamples;
				fadeLevel[i] += muteState[i] ? -step : step;
				if ((muteState[i] && fadeLevel[i] <= 0.f) || (!muteState[i] && fadeLevel[i] >= 1.f)) {
					fadeLevel[i] = muteState[i] ? 0.f : 1.f;
					transitionCount[i] = 0;
				} else {
					transitionCount[i]--;
				}
			} else {
				fadeLevel[i] = muteState[i] ? 0.f : 1.f;
			}
		
			// -----------------------------
			// Apply fade and compute inputCount
			// -----------------------------
			inputL[i] *= fadeLevel[i];
			inputR[i] *= fadeLevel[i];
		
			if (inputActive && fadeLevel[i] > 0.f) {
				inputCount += 1.f; // only count channels contributing signal
			}
		
			// -----------------------------
			// Apply VCA CV if connected
			// -----------------------------
			if (activeVcaChannel[i] == i) {
				float v = clamp(inputs[VCA_CV1_INPUT + i].getPolyVoltage(0) / 10.f, 0.f, 2.f);
				inputL[i] *= v;
				inputR[i] *= v;
			} else if (activeVcaChannel[i] > -1) {
				int diff = i - activeVcaChannel[i];
				int maxCh = vcaChannels[activeVcaChannel[i]];
				if (diff >= 0 && diff < maxCh) {
					float v = clamp(inputs[VCA_CV1_INPUT + activeVcaChannel[i]].getPolyVoltage(diff) / 10.f, 0.f, 2.f);
					inputL[i] *= v;
					inputR[i] *= v;
				}
			}

            float vol = params[VOLUME1_PARAM + i].getValue();
            inputL[i] *= vol;
            inputR[i] *= vol;

            // Simple peak detection using the absolute maximum of the current input
            envPeakL[i] = fmax(envPeakL[i] * decayRate, fabs(inputL[i]));
            envPeakR[i] = fmax(envPeakR[i] * decayRate, fabs(inputR[i]));

            if (inputActive){
                filteredEnvelopeL[i] = fmax(filteredEnvelopeL[i],0.1f);
                filteredEnvelopeR[i] = fmax(filteredEnvelopeR[i],0.1f);
                filteredEnvelope[i] = (filteredEnvelopeL[i]+filteredEnvelopeR[i])/2.0f;
            }

            filteredEnvelopeL[i] = alpha * envPeakL[i] + (1 - alpha) * filteredEnvelopeL[i];
            filteredEnvelopeR[i] = alpha * envPeakR[i] + (1 - alpha) * filteredEnvelopeR[i];
            compressionAmountL += filteredEnvelopeL[i];
            compressionAmountR += filteredEnvelopeR[i];

            // Apply panning
            float pan = params[PAN1_PARAM + i].getValue();

            if (activePanChannel[i]==i) {
                pan += inputs[PAN_CV1_INPUT + i].getPolyVoltage(0) / 5.f;
            } else if (activePanChannel[i] > -1){
                // Now we compute which channel we need to grab
                int diffBetween = i - activePanChannel[i];
                int currentChannelMax =  panChannels[activePanChannel[i]] ;
                if (diffBetween >= 0 && diffBetween < currentChannelMax) {    //If we are before the last poly channel
                    pan += inputs[PAN_CV1_INPUT + activePanChannel[i]].getPolyVoltage(diffBetween) / 5.f;
                }
            }

            pan = clamp(pan, -1.f, 1.f);

            // Convert pan range from -1...1 to 0...1 for sinusoidal calculations
            float scaledPan = (pan + 1.f) * 0.5f;

            // Initialize or update panning if the pan value has changed
            if (!initialized[i] || pan != lastPan[i]) {
                panL[i] = polyCos(M_PI_2 * scaledPan);  // π/2 * scaledPan ranges from 0 to π/2
                panR[i] = polySin(M_PI_2 * scaledPan);
                lastPan[i] = pan;
                initialized[i] = true;
            }

            // Mix processed signals into left and right outputs
            inputL[i] *= panL[i];
            inputR[i] *= panR[i];

        } //end process channels

        // Handle muting with fade transition
        if (params[MUTESIDE_PARAM].getValue() > 0.5f) {
            if (!muteLatch[6]) {
                muteLatch[6] = true;
                muteState[6] = !muteState[6];
                transitionCount[6] = transitionSamples;  // Reset the transition count
            }
        } else {
            muteLatch[6] = false;
        }

		// If no audio or side-chain channels are active, exit early to save CPU.
		bool sideConnected = inputs[SIDECHAIN_INPUT_L].isConnected() || inputs[SIDECHAIN_INPUT_R].isConnected();
		if (inputCount <= 0.0f && !sideConnected) {
			// Clean up transient state so UI meters and envelopes don't show spurious values
			for (int k = 0; k < 6; ++k) {
				filteredEnvelopeL[k] = 0.0f;
				filteredEnvelopeR[k] = 0.0f;
				filteredEnvelope[k]  = 0.0f;
				envPeakL[k] = 0.0f;
				envPeakR[k] = 0.0f;
				fadeLevel[k] = 0.0f;
				transitionCount[k] = 0;
				initialized[k] = false;
				lastPan[k] = 0.0f;
				inputL[k] = 0.0f;
				inputR[k] = 0.0f;
			}
		
			// Reset mix/envelope tracking variables used later
			compressionAmountL = 0.0f;
			compressionAmountR = 0.0f;
			pressTotalL = 0.0f;
			pressTotalR = 0.0f;
			volTotalL = 0.0f;
			volTotalR = 0.0f;
			distortTotalL = 0.0f;
			distortTotalR = 0.0f;
		
			// Write zero to outputs and exit before any heavy processing (ADAA/shaper/HPF/etc.)
			outputs[AUDIO_OUTPUT_L].setVoltage(0.0f);
			outputs[AUDIO_OUTPUT_R].setVoltage(0.0f);
			return;
		}

        float sideChain=0.f;
        if (sideConnected) sideChain = 1.f;
        compressionAmountL = compressionAmountL/((inputCount+sideChain)*5.0f); //divide by the expected ceiling
        compressionAmountR = compressionAmountR/((inputCount+sideChain)*5.0f); //process L and R separately

        float pressAmount = params[PRESS_PARAM].getValue();
        if(inputs[PRESS_CV_INPUT].isConnected()){
            pressAmount += inputs[PRESS_CV_INPUT].getVoltage()*params[PRESS_ATT].getValue();
        }
        pressAmount = clamp(pressAmount, 0.0f, 1.0f);

        if (inputCount>0 && compressionAmountL > 0 && compressionAmountR > 0){  //div zero protection
            pressTotalL = ( (1.0f-pressAmount) + (pressAmount / compressionAmountL) ) * 6.0f / inputCount;
            pressTotalR = ( (1.0f-pressAmount) + (pressAmount / compressionAmountR) ) * 6.0f / inputCount;
        } else {
            pressTotalL = 0.0f;
            pressTotalR = 0.0f;
        }

        // MIX the channels scaled by compression
        for (int i=0; i<6; i++){
            if (compressionAmountL > 0.0f && inputCount>0.0f){ //avoid div by zero
                    mixL += inputL[i]*pressTotalL;
            } else { mixL = 0.0f;}
            if (compressionAmountR > 0.0f && inputCount>0.0f){ //avoid div by zero
                    mixR += inputR[i]*pressTotalR;
            } else {mixR = 0.0f;}

        }

        // Side processing and envelope calculation

        // Initially check connection and set initial input values
        bool isSideConnectedL = inputs[SIDECHAIN_INPUT_L].isConnected();
        bool isSideConnectedR = inputs[SIDECHAIN_INPUT_R].isConnected();
        float sideL = isSideConnectedL ? inputs[SIDECHAIN_INPUT_L].getVoltage() : 0.0f;
        float sideR = isSideConnectedR ? inputs[SIDECHAIN_INPUT_R].getVoltage() : 0.0f;

        // Handle mono to stereo routing
        if (!isSideConnectedL && isSideConnectedR) {
            sideL = sideR;
        }
        if (!isSideConnectedR && isSideConnectedL) {
            sideR = sideL;
        }
        processSide(sideL, sideR, decayRate, mixL, mixR);

        float feedbackSetting = params[FEEDBACK_PARAM].getValue();
        if(inputs[FEEDBACK_CV].isConnected()){
            feedbackSetting += inputs[FEEDBACK_CV].getVoltage()*params[FEEDBACK_ATT].getValue();
        }

        feedbackSetting = 11.0f*pow(feedbackSetting/11.0f, 3.0f);
        feedbackSetting = clamp(feedbackSetting, 0.0f, 11.0f);

        float saturationEffect = 1 + feedbackSetting;
        mixL *= saturationEffect;
        mixR *= saturationEffect;

        // Remove DC offsets by high pass filtering
        if(applyFilters){
            mixL = hpfL.process(mixL);
            mixR = hpfR.process(mixR);
        }

        distortTotalL = distortTotalL * decayRate + log1p(fmax(mixL-35.f, 0.0f)) * (35.0f / log1p(35)) * (1.0f - decayRate);
        distortTotalR = distortTotalR * decayRate +log1p(fmax(mixR-35.f, 0.0f)) * (35.0f / log1p(35)) * (1.0f - decayRate);


        // Apply ADAA
        float maxHeadRoom = 46.f; //1.314*35 exceeding this number results in strange wavefolding due to the polytanh bad fit beyond this point
        mixL = clamp(mixL, -maxHeadRoom, maxHeadRoom);
        mixR = clamp(mixR, -maxHeadRoom, maxHeadRoom);
        mixL = applyADAA(mixL/35.f, lastOutputL, sampleRate); //35 is 7x5v
        mixR = applyADAA(mixR/35.f, lastOutputR, sampleRate);
        lastOutputL = mixL;
        lastOutputR = mixR;

        // Set outputs
        float masterVol = params[MASTER_VOL].getValue();
        if (inputs[MASTER_VOL_CV].isConnected()){
            masterVol += inputs[MASTER_VOL_CV].getVoltage()*params[MASTER_VOL_ATT].getValue()/10.f;
        }
        masterVol = clamp(masterVol, 0.0f, 2.0f);

        // Processing the outputs
        float outputL = mixL * 6.9f * masterVol;
        float outputR = mixR * 6.9f * masterVol;


        volTotalL = volTotalL * decayRate + fabs(outputL) * (1.0f - decayRate);
        volTotalR = volTotalR * decayRate + fabs(outputR) * (1.0f - decayRate);


        if (isSupersamplingEnabled) {
            // Use the oversampling shaper for the signal
            outputL = shaperL.process(outputL);
            outputR = shaperR.process(outputR);
        }

        outputs[AUDIO_OUTPUT_L].setVoltage(outputL);
        outputs[AUDIO_OUTPUT_R].setVoltage(outputR);

    }

    float applyADAA(float input, float lastInput, float sampleRate) {
        float delta = input - lastInput;
        if (fabs(delta) > 1e-6) {
            return (antiderivative(input) - antiderivative(lastInput)) / delta;
        } else {
            return polyTanh(input);
        }
    }

	float antiderivative(float x) {
		float x2 = x * x;
		return x2 * (0.5f - x2 * (1.0f/12.0f - x2 * (1.0f/45.0f - 17.0f/2520.0f * x2)));
	}
	
	float polyTanh(float x) {
		float x2 = x * x;
		return x - x * x2 * (1.0f/3.0f - x2 * (2.0f/15.0f - 17.0f/315.0f * x2));
	}
	
	float polySin(float x) {
		float x2 = x * x;
		return x - x * x2 * (1.0f/6.0f - x2 * (1.0f/120.0f - x2 / 5040.0f));
	}
	
	float polyCos(float x) {
		float x2 = x * x;
		return 1.0f - x2 * (0.5f - x2 * (1.0f/24.0f - x2 / 720.0f));
	}

    void processSide(float &sideL, float &sideR, float decayRate, float &mixL, float &mixR) {
        // Apply VCA control if connected
        if (inputs[VCA_SIDECHAIN_INPUT].isConnected()) {
            float vcaVoltage = inputs[VCA_SIDECHAIN_INPUT].getVoltage() / 10.f;
            float vcaLevel = clamp(vcaVoltage, 0.f, 2.f);
            sideL *= vcaLevel;
            sideR *= vcaLevel;
        }

        // Apply volume control from the parameters
        float sideVol = params[SIDECHAIN_VOLUME_PARAM].getValue();
        sideL *= sideVol;
        sideR *= sideVol;

        if (transitionCount[6] > 0) {
            float fadeStep = (muteState[6] ? -1.0f : 1.0f) / transitionSamples;
            fadeLevel[6] += fadeStep;
            if ((muteState[6] && fadeLevel[6] < 0.0f) || (!muteState[6] && fadeLevel[6] > 1.0f)) {
                fadeLevel[6] = muteState[6] ? 0.0f : 1.0f;
                transitionCount[6] = 0;  // End transition
            }
            transitionCount[6]--;
        } else {
            fadeLevel[6] = muteState[6] ? 0.0f : 1.0f;
        }

        if (!mutedSideDucks){    //only fade out the sound if mixing it
            sideL *= fadeLevel[6];
            sideR *= fadeLevel[6];
        }

        // Check sidechain connection
        bool isSideConnectedL = inputs[SIDECHAIN_INPUT_L].isConnected();
        bool isSideConnectedR = inputs[SIDECHAIN_INPUT_R].isConnected();

        if (!isSideConnectedL && !isSideConnectedR) {
            // Reset envelope if sidechain inputs are not connected
            sidePeakL = 0.0f;
            sidePeakR = 0.0f;
            filteredSideEnvelopeL = 0.0f;
            filteredSideEnvelopeR = 0.0f;
            sideEnvelope = 0.0f;
        } else {
            // Calculate the envelope for the side signals
            sidePeakL = fmax(sidePeakL * decayRate, fabs(sideL));
            sidePeakR = fmax(sidePeakR * decayRate, fabs(sideR));
            filteredSideEnvelopeL = alpha * sidePeakL + (1 - alpha) * filteredSideEnvelopeL;
            filteredSideEnvelopeR = alpha * sidePeakR + (1 - alpha) * filteredSideEnvelopeR;

            // Apply the envelope to the side signals
            sideL *= filteredSideEnvelopeL;
            sideR *= filteredSideEnvelopeR;

            // Calculate ducking based on the side envelope
            float duckAmount = params[DUCK_PARAM].getValue();
            if (inputs[DUCK_CV].isConnected()) {
                duckAmount += clamp(inputs[DUCK_CV].getVoltage() / 5.0f, 0.f, 1.f) * params[DUCK_ATT].getValue();
            }
            float duckingFactorL = fmax(0.0f, 1.f - duckAmount * (filteredSideEnvelopeL / 5.0f));
            float duckingFactorR = fmax(0.0f, 1.f - duckAmount * (filteredSideEnvelopeR / 5.0f));
            sideEnvelope = (filteredSideEnvelopeL + filteredSideEnvelopeR) / 2.0f;

            if (!mutedSideDucks){
                // Apply ducking to the main mix and add the processed side signals
                mixL = (mixL * duckingFactorL) + sideL;
                mixR = (mixR * duckingFactorR) + sideR;
            } else {
                if (muteState[6]) {
                    mixL = (mixL * duckingFactorL) ;
                    mixR = (mixR * duckingFactorR) ;

                } else {
                    mixL = (mixL * duckingFactorL) + sideL;
                    mixR = (mixR * duckingFactorR) + sideR;
                }
            }
        }
    }//end process side

};


struct PressedDuckWidget : ModuleWidget {

	// Define ShiftButton template
	template <typename BaseButton>
	struct ShiftButton : BaseButton {
		void onButton(const event::Button& e) override {
			if (e.action != GLFW_PRESS || e.button != GLFW_MOUSE_BUTTON_LEFT) {
				BaseButton::onButton(e);
				return;
			}
	
			auto paramQuantity = this->getParamQuantity();
			if (paramQuantity == nullptr) {
				BaseButton::onButton(e);
				return;
			}
			if (PressedDuck* module = dynamic_cast<PressedDuck*>(paramQuantity->module)) {
				int index = paramQuantity->paramId - PressedDuck::MUTE1_PARAM;
				if (index >= 0 && index < 6) {  // isShifted has size 6
					module->isShifted[index].store((e.mods & GLFW_MOD_SHIFT) == 0);
				}
			}
			e.consume(this);
		}
	};
	struct ShiftLEDButton : ShiftButton<LEDButton> {};

    PressedDuckWidget(PressedDuck* module) {
        setModule(module);

        setPanel(createPanel(
                asset::plugin(pluginInstance, "res/PressedDuck.svg"),
                asset::plugin(pluginInstance, "res/PressedDuck-dark.svg")
            ));

        // Add screws or additional design elements as needed
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // Constants for positioning
        const Vec channelOffset(30, 60); // Start position for the first channel controls
        const float sliderX = 36.0f;     // Horizontal spacing for sliders
        const float Spacing = 27.0f;  // Vertical spacing between inputs/outputs

        // Positioning variables
        float yPos = channelOffset.y;
        float xPos = channelOffset.x;

        // Sidechain audio inputs
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PressedDuck::SIDECHAIN_INPUT_L ));
        yPos += Spacing;
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PressedDuck::SIDECHAIN_INPUT_R ));

        // Sidechain Volume slider with light
        yPos += 40+Spacing;
        addParam(createLightParamCentered<VCVLightSlider<YellowLight>>(Vec(xPos, yPos), module, PressedDuck::SIDECHAIN_VOLUME_PARAM , PressedDuck::BASS_VOLUME_LIGHT));

        // Sidechain VCA CV input
        yPos += 38+Spacing;
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PressedDuck::VCA_SIDECHAIN_INPUT ));

        // Ducking amount knob
        yPos += 1.25*Spacing;
        addParam(createParamCentered<RoundBlackKnob>(Vec(xPos, yPos), module, PressedDuck::DUCK_PARAM));

        // Ducking attenuator
        yPos = channelOffset.y + 5*Spacing + 84;
        addParam(createParamCentered<Trimpot>(Vec(xPos, yPos), module, PressedDuck::DUCK_ATT));

        // Ducking CV input
        yPos += Spacing;
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PressedDuck::DUCK_CV));

        // Sidechain Mute
        yPos = channelOffset.y + 4*Spacing + 170;
        addParam(createParamCentered<LEDButton>           (Vec(xPos, yPos), module, PressedDuck::MUTESIDE_PARAM));
        addChild(createLightCentered<SmallLight<RedLight>>(Vec(xPos, yPos), module, PressedDuck::MUTESIDE_LIGHT));

        yPos = channelOffset.y;
        // Loop through each channel
        for (int i = 0; i < 6; i++) {
            xPos = 1.25*sliderX + channelOffset.x + i * sliderX;

            // Audio inputs
            addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PressedDuck::AUDIO_1L_INPUT + 2 * i));
            yPos += Spacing;
            addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PressedDuck::AUDIO_1R_INPUT + 2 * i));

            // Volume slider with light
            yPos += 40+Spacing;
            addParam(createLightParamCentered<VCVLightSlider<YellowLight>>(Vec(xPos, yPos), module, PressedDuck::VOLUME1_PARAM + i, PressedDuck::VOLUME1_LIGHT+ i));

            // VCA CV input
            yPos += 38+Spacing;
            addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PressedDuck::VCA_CV1_INPUT + i));

            // Pan knob
            yPos += Spacing + 20;
            addParam(createParamCentered<RoundBlackKnob>(Vec(xPos, yPos), module, PressedDuck::PAN1_PARAM + i));

            // Pan CV input
            yPos += Spacing;
            addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PressedDuck::PAN_CV1_INPUT + i));

            // Mute
            yPos += 1.2*Spacing;
            addParam(createParamCentered<ShiftLEDButton>           (Vec(xPos, yPos), module, PressedDuck::MUTE1_PARAM + i));
            addChild(createLightCentered<SmallLight<RedLight>>(Vec(xPos, yPos), module, PressedDuck::MUTE1_LIGHT + i));
            yPos += 0.8*Spacing;
            addInput(createInputCentered<ThemedPJ301MPort> (Vec(xPos, yPos), module, PressedDuck::MUTE_1_INPUT + i));

            // Reset yPos for next channel
            yPos = channelOffset.y;
        }

        // Global controls for saturation and side processing (placing these at the end of channels)
        xPos += 1.75*sliderX; // Shift to the right of the last channel
        yPos = channelOffset.y + 0.5*Spacing;

        // Saturation ceiling knob
        addParam(createParamCentered<RoundHugeBlackKnob>(Vec(xPos, yPos), module, PressedDuck::PRESS_PARAM));

        // Add Ring of Lights
        addLightsAroundKnob(module, xPos, yPos, PressedDuck::PRESS_LIGHT1R, 20, 31.f);
        addLightsAroundKnob(module, xPos, yPos, PressedDuck::PRESS_LIGHT1L, 20, 35.f);

        // Saturation ceiling attenuator
        yPos += 1.5*Spacing ;
        xPos -= .5*sliderX; // Shift to the right of the last channel
        addParam(createParamCentered<Trimpot>(Vec(xPos, yPos), module, PressedDuck::PRESS_ATT));

        // Saturation CV input
        xPos += 1.0*sliderX; // Shift to the right of the last channel
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PressedDuck::PRESS_CV_INPUT));

        xPos -= .5*sliderX; // Shift to the right of the last channel
        yPos += 2.1*Spacing;

        // FEEDBACK
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(xPos, yPos), module, PressedDuck::FEEDBACK_PARAM));

        // Add Ring of Lights
        addLightsAroundKnob(module, xPos, yPos, PressedDuck::FEED_LIGHT1R, 20, 22.5f);
        addLightsAroundKnob(module, xPos, yPos, PressedDuck::FEED_LIGHT1L, 20, 26.5f);

        // FEEDBACK attenuator
        yPos += 1.3*Spacing;
        xPos -= .5*sliderX; // Shift to the right of the last channel
        addParam(createParamCentered<Trimpot>(Vec(xPos, yPos), module, PressedDuck::FEEDBACK_ATT));

        // FEEDBACK CV input
        xPos += 1.0*sliderX; // Shift to the right of the last channel

        addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PressedDuck::FEEDBACK_CV));
        xPos -= .5*sliderX; // Shift to the right of the last channel

        // Master Volume
        yPos = channelOffset.y + 4.3*Spacing + 85;
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(xPos, yPos), module, PressedDuck::MASTER_VOL));

        // Add Ring of Lights for L/R channels
        addLightsAroundKnob(module, xPos, yPos, PressedDuck::VOL_LIGHT1R, 20, 22.5f);
        addLightsAroundKnob(module, xPos, yPos, PressedDuck::VOL_LIGHT1, 20, 26.5f);

        // Master Volume attenuator
        yPos += 1.3*Spacing;
        xPos -= .5*sliderX; // Shift to the right of the last channel
        addParam(createParamCentered<Trimpot>(Vec(xPos, yPos), module, PressedDuck::MASTER_VOL_ATT));

        // Master Volume CV input
        xPos += 1.0*sliderX; // Shift to the right of the last channel

        addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PressedDuck::MASTER_VOL_CV));
        xPos -= .5*sliderX; // Shift to the right of the last channel

        xPos -= .5*sliderX; // Shift to the right of the last channel

        // Outputs
        yPos = channelOffset.y + 4*Spacing + 170;
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PressedDuck::AUDIO_OUTPUT_L));
        xPos += 1*sliderX; // Shift to the right of the last channel
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PressedDuck::AUDIO_OUTPUT_R));

    }

    void addLightsAroundKnob(Module* module, float knobX, float knobY, int firstLightId, int numLights, float radius) {
        const float startAngle = M_PI*0.7f; // Start angle in radians (8 o'clock on the clock face)
        const float endAngle = 2.0f*M_PI+M_PI*0.3f;   // End angle in radians (4 o'clock on the clock face)

        for (int i = 0; i < numLights; i++) {
            float fraction = (float)i / (numLights - 1); // Fraction that goes from 0 to 1
            float angle = startAngle + fraction * (endAngle - startAngle);
            float x = knobX + radius * cos(angle);
            float y = knobY + radius * sin(angle);

            // Create and add the light
            if (i< .5*numLights){
                addChild(createLightCentered<TinyLight<YellowLight>>(Vec(x, y), module, firstLightId + i));
            } else {
                addChild(createLightCentered<TinyLight<RedLight>>(Vec(x, y), module, firstLightId + i));
            }
        }
    }

	void step() override {
		// Cast base Module* to your subclass
		PressedDuck* module = dynamic_cast<PressedDuck*>(this->module);
		if (!module) return;
	
		updateLights(module);
		ModuleWidget::step();
	}
	
	void updateLights(PressedDuck* module) {    
		for (int i = 0; i < 6; i++) {
			module->lights[PressedDuck::VOLUME1_LIGHT + i].setBrightness(module->filteredEnvelope[i]);
			module->lights[PressedDuck::MUTE1_LIGHT + i].setBrightness(module->muteState[i] ? 1.0f : 0.0f);
		}
		module->lights[PressedDuck::BASS_VOLUME_LIGHT].setBrightness(module->sideEnvelope);
		module->lights[PressedDuck::MUTESIDE_LIGHT].setBrightness(module->muteState[6] ? 1.0f : 0.0f);
	
		updateSegmentedLights(module, PressedDuck::PRESS_LIGHT1L, module->pressTotalL, 35.0f, 20);
		updateSegmentedLights(module, PressedDuck::PRESS_LIGHT1R, module->pressTotalR, 35.0f, 20);
		updateSegmentedLights(module, PressedDuck::FEED_LIGHT1L, module->distortTotalL, 100.0f, 20);
		updateSegmentedLights(module, PressedDuck::FEED_LIGHT1R, module->distortTotalR, 100.0f, 20);
		updateSegmentedLights(module, PressedDuck::VOL_LIGHT1, module->volTotalL, 10.0f, 20);
		updateSegmentedLights(module, PressedDuck::VOL_LIGHT1R, module->volTotalR, 10.0f, 20);
	}
	
	void updateSegmentedLights(PressedDuck* module, int startLightId, float totalValue, float maxValue, int numLights) {
		float normalizedValue = totalValue / maxValue;
		int fullLights = static_cast<int>(normalizedValue * numLights);
		float fractionalBrightness = (normalizedValue * numLights) - fullLights;
	
		for (int i = 0; i < numLights; i++) {
			if (i < fullLights) {
				module->lights[startLightId + i].setBrightness(1.0f);
			} else if (i == fullLights) {
				module->lights[startLightId + i].setBrightness(fractionalBrightness);
			} else {
				float dimming = module->lights[startLightId + i].getBrightness();
				module->lights[startLightId + i].setBrightness(dimming * 0.75f);
			}
		}
	}

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);

        PressedDuck* PressedDuckModule = dynamic_cast<PressedDuck*>(module);
        assert(PressedDuckModule); // Ensure the cast succeeds

        // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());

        // VOctCV menu item
        struct FilterMenuItem : MenuItem {
            PressedDuck* PressedDuckModule;
            void onAction(const event::Action& e) override {
                // Toggle the "Apply Filters" mode
                PressedDuckModule->applyFilters = !PressedDuckModule->applyFilters;
            }
            void step() override {
                // Update the display to show a checkmark when the mode is active
                rightText = PressedDuckModule->applyFilters ? "✔" : "";
                MenuItem::step();
            }
        };

        FilterMenuItem* filterItem = new FilterMenuItem();
        filterItem->text = "Apply Filters";
        filterItem->PressedDuckModule = PressedDuckModule;
        menu->addChild(filterItem);

        // MutedSideDucks menu item
        struct MutedSideDucksMenuItem : MenuItem {
            PressedDuck* PressedDuckModule;
            void onAction(const event::Action& e) override {
                // Toggle the "Muted Side Ducks" mode
                PressedDuckModule->mutedSideDucks = !PressedDuckModule->mutedSideDucks;
            }
            void step() override {
                // Update the display to show a checkmark when the mode is active
                rightText = PressedDuckModule->mutedSideDucks ? "✔" : "";
                MenuItem::step();
            }
        };

        // Create the MutedSideDucks menu item and add it to the menu
        MutedSideDucksMenuItem* mutedSideDucksItem = new MutedSideDucksMenuItem();
        mutedSideDucksItem->text = "Muted Sidechain still Ducks";
        mutedSideDucksItem->PressedDuckModule = PressedDuckModule;
        menu->addChild(mutedSideDucksItem);

        // Supersampling menu item
        struct SupersamplingMenuItem : MenuItem {
            PressedDuck* PressedDuckModule;

            void onAction(const event::Action& e) override {
                // Toggle the "Supersampling" mode
                PressedDuckModule->isSupersamplingEnabled = !PressedDuckModule->isSupersamplingEnabled;
            }

            void step() override {
                // Update the display to show a checkmark when the mode is active
                rightText = PressedDuckModule->isSupersamplingEnabled ? "✔" : "";
                MenuItem::step();
            }
        };

        // Create the Supersampling menu item and add it to the menu
        SupersamplingMenuItem* supersamplingItem = new SupersamplingMenuItem();
        supersamplingItem->text = "Enable Supersampling";
        supersamplingItem->PressedDuckModule = PressedDuckModule;
        menu->addChild(supersamplingItem);

    }
};

Model* modelPressedDuck = createModel<PressedDuck, PressedDuckWidget>("PressedDuck");