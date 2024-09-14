////////////////////////////////////////////////////////////
//
//   Preeeeeeeeeeessed Duck
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   A stereo 16 channel mixer with compression and ducking.
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

struct PreeeeeeeeeeessedDuck : Module {

    enum ParamIds {
        VOLUME1_PARAM, VOLUME2_PARAM, VOLUME3_PARAM, VOLUME4_PARAM, VOLUME5_PARAM, VOLUME6_PARAM,   
        VOLUME7_PARAM, VOLUME8_PARAM, VOLUME9_PARAM, VOLUME10_PARAM, VOLUME11_PARAM, VOLUME12_PARAM,   
        VOLUME13_PARAM, VOLUME14_PARAM, VOLUME15_PARAM, VOLUME16_PARAM,   
        PAN1_PARAM, PAN2_PARAM, PAN3_PARAM, PAN4_PARAM, PAN5_PARAM, PAN6_PARAM,      
        PAN7_PARAM, PAN8_PARAM, PAN9_PARAM, PAN10_PARAM, PAN11_PARAM, PAN12_PARAM,      
        PAN13_PARAM, PAN14_PARAM, PAN15_PARAM, PAN16_PARAM,    
        SIDECHAIN_VOLUME_PARAM, DUCK_PARAM, DUCK_ATT,
        PRESS_PARAM, PRESS_ATT, MASTER_VOL, MASTER_VOL_ATT, FEEDBACK_PARAM, FEEDBACK_ATT, 
        MUTE1_PARAM, MUTE2_PARAM, MUTE3_PARAM, MUTE4_PARAM, MUTE5_PARAM, MUTE6_PARAM, 
        MUTE7_PARAM, MUTE8_PARAM, MUTE9_PARAM, MUTE10_PARAM, MUTE11_PARAM, MUTE12_PARAM, 
        MUTE13_PARAM, MUTE14_PARAM, MUTE15_PARAM, MUTE16_PARAM, MUTESIDE_PARAM,
        NUM_PARAMS
    };
    enum InputIds {
        AUDIO_1L_INPUT, AUDIO_1R_INPUT, AUDIO_2L_INPUT, AUDIO_2R_INPUT, 
        AUDIO_3L_INPUT, AUDIO_3R_INPUT, AUDIO_4L_INPUT, AUDIO_4R_INPUT, 
        AUDIO_5L_INPUT, AUDIO_5R_INPUT, AUDIO_6L_INPUT, AUDIO_6R_INPUT,   
        AUDIO_7L_INPUT, AUDIO_7R_INPUT, AUDIO_8L_INPUT, AUDIO_8R_INPUT,   
        AUDIO_9L_INPUT, AUDIO_9R_INPUT, AUDIO_10L_INPUT, AUDIO_10R_INPUT,   
        AUDIO_11L_INPUT, AUDIO_11R_INPUT, AUDIO_12L_INPUT, AUDIO_12R_INPUT,   
        AUDIO_13L_INPUT, AUDIO_13R_INPUT, AUDIO_14L_INPUT, AUDIO_14R_INPUT,   
        AUDIO_15L_INPUT, AUDIO_15R_INPUT, AUDIO_16L_INPUT, AUDIO_16R_INPUT,   
        VCA_CV1_INPUT, VCA_CV2_INPUT, VCA_CV3_INPUT, VCA_CV4_INPUT, VCA_CV5_INPUT, VCA_CV6_INPUT,
        VCA_CV7_INPUT, VCA_CV8_INPUT, VCA_CV9_INPUT, VCA_CV10_INPUT, VCA_CV11_INPUT, VCA_CV12_INPUT,
        VCA_CV13_INPUT, VCA_CV14_INPUT, VCA_CV15_INPUT, VCA_CV16_INPUT,  VCA_SIDECHAIN_INPUT,
        PAN_CV1_INPUT, PAN_CV2_INPUT, PAN_CV3_INPUT, PAN_CV4_INPUT, PAN_CV5_INPUT, PAN_CV6_INPUT,  
        PAN_CV7_INPUT, PAN_CV8_INPUT, PAN_CV9_INPUT, PAN_CV10_INPUT, PAN_CV11_INPUT, PAN_CV12_INPUT,  
        PAN_CV13_INPUT, PAN_CV14_INPUT, PAN_CV15_INPUT, PAN_CV16_INPUT,
        SIDECHAIN_INPUT_L, SIDECHAIN_INPUT_R, DUCK_CV, PRESS_CV_INPUT, FEEDBACK_CV, MASTER_VOL_CV,
        MUTE_1_INPUT, MUTE_2_INPUT, MUTE_3_INPUT, MUTE_4_INPUT, MUTE_5_INPUT, MUTE_6_INPUT,
        MUTE_7_INPUT, MUTE_8_INPUT, MUTE_9_INPUT, MUTE_10_INPUT, MUTE_11_INPUT, MUTE_12_INPUT,
        MUTE_13_INPUT, MUTE_14_INPUT, MUTE_15_INPUT, MUTE_16_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        AUDIO_OUTPUT_L, AUDIO_OUTPUT_R, 
        NUM_OUTPUTS
    };
    enum LightIds {
        VOLUME1_LIGHT, VOLUME2_LIGHT, VOLUME3_LIGHT, VOLUME4_LIGHT, VOLUME5_LIGHT, VOLUME6_LIGHT, 
        VOLUME7_LIGHT, VOLUME8_LIGHT, VOLUME9_LIGHT, VOLUME10_LIGHT, VOLUME11_LIGHT, VOLUME12_LIGHT, 
        VOLUME13_LIGHT, VOLUME14_LIGHT, VOLUME15_LIGHT, VOLUME16_LIGHT,  BASS_VOLUME_LIGHT, 

        MUTE1_LIGHT, MUTE2_LIGHT, MUTE3_LIGHT, MUTE4_LIGHT, MUTE5_LIGHT, MUTE6_LIGHT, 
        MUTE7_LIGHT, MUTE8_LIGHT, MUTE9_LIGHT, MUTE10_LIGHT, MUTE11_LIGHT, MUTE12_LIGHT, 
        MUTE13_LIGHT, MUTE14_LIGHT, MUTE15_LIGHT, MUTE16_LIGHT,  MUTESIDE_LIGHT,

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

    bool applyFilters = true;

    //for tracking the mute state of each channel
    bool muteLatch[17] = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
    bool muteState[17] = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
    bool mutedSideDucks = false;
    
    // Serialization method to save module state
    json_t* dataToJson() override {
        json_t* rootJ = json_object();
    
        // Save the state of applyFilters as a boolean
        json_object_set_new(rootJ, "applyFilters", json_boolean(applyFilters));
    
        // Save the state of mutedSideDucks as a boolean
        json_object_set_new(rootJ, "mutedSideDucks", json_boolean(mutedSideDucks));
    
        // Save the muteLatch and muteState arrays
        json_t* muteLatchJ = json_array();
        json_t* muteStateJ = json_array();
    
        for (int i = 0; i < 17; i++) {
            json_array_append_new(muteLatchJ, json_boolean(muteLatch[i]));
            json_array_append_new(muteStateJ, json_boolean(muteState[i]));
        }
    
        json_object_set_new(rootJ, "muteLatch", muteLatchJ);
        json_object_set_new(rootJ, "muteState", muteStateJ);
    
        return rootJ;
    }
    
    // Deserialization method to load module state
    void dataFromJson(json_t* rootJ) override {
        // Load the state of applyFilters
        json_t* applyFiltersJ = json_object_get(rootJ, "applyFilters");
        if (applyFiltersJ) {
            // Use json_is_true() to check if the JSON value is true; otherwise, set to false
            applyFilters = json_is_true(applyFiltersJ);
        }
    
        // Load the state of mutedSideDucks
        json_t* mutedSideDucksJ = json_object_get(rootJ, "mutedSideDucks");
        if (mutedSideDucksJ) {
            mutedSideDucks = json_is_true(mutedSideDucksJ);
        }
    
        // Load muteLatch and muteState arrays
        json_t* muteLatchJ = json_object_get(rootJ, "muteLatch");
        json_t* muteStateJ = json_object_get(rootJ, "muteState");
    
        if (muteLatchJ) {
            for (size_t i = 0; i < json_array_size(muteLatchJ) && i < 17; i++) {
                json_t* muteLatchValue = json_array_get(muteLatchJ, i);
                if (muteLatchValue) {
                    muteLatch[i] = json_is_true(muteLatchValue);
                }
            }
        }
    
        if (muteStateJ) {
            for (size_t i = 0; i < json_array_size(muteStateJ) && i < 17; i++) {
                json_t* muteStateValue = json_array_get(muteStateJ, i);
                if (muteStateValue) {
                    muteState[i] = json_is_true(muteStateValue);
                }
            }
        }        
    }

    // Variables for envelope followers and lights
    float sidePeakL = 0.0f;
    float sidePeakR = 0.0f;
    float envPeakL[16] = {0.0f};
    float envPeakR[16] = {0.0f};
    float envelopeL[16] = {0.0f}; 
    float envelopeR[16] = {0.0f}; 

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
    float inputL[16] = {0.0f};
    float inputR[16] = {0.0f};
    float panL[16] = {0.0f};
    float panR[16] = {0.0f};
    float lastPan[16] = {0.0f}; 
    bool initialized[16] = {false};  
    float filteredEnvelopeL[16] = {0.0f}; 
    float filteredEnvelopeR[16] = {0.0f}; 
    float filteredEnvelope[16] = {0.0f}; 
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

    // For mute transition
    float transitionTimeMs = 10000.f; // Transition time in milliseconds
    float transitionSamples; // Number of samples to complete the transition
    float fadeLevel[17] = {1.0f}; 
    int transitionCount[17] = {0};  // Array to track transition progress for each channel
    float targetFadeLevel[17] = {0.0f};

    PreeeeeeeeeeessedDuck() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        // Configure volume and pan parameters for each channel
        configParam(VOLUME1_PARAM, 0.f, 2.f, 1.0f, "Channel 1 Volume");
        configParam(VOLUME2_PARAM, 0.f, 2.f, 1.0f, "Channel 2 Volume");
        configParam(VOLUME3_PARAM, 0.f, 2.f, 1.0f, "Channel 3 Volume");
        configParam(VOLUME4_PARAM, 0.f, 2.f, 1.0f, "Channel 4 Volume");
        configParam(VOLUME5_PARAM, 0.f, 2.f, 1.0f, "Channel 5 Volume");
        configParam(VOLUME6_PARAM, 0.f, 2.f, 1.0f, "Channel 6 Volume");
        configParam(VOLUME7_PARAM, 0.f, 2.f, 1.0f, "Channel 7 Volume");
        configParam(VOLUME8_PARAM, 0.f, 2.f, 1.0f, "Channel 8 Volume");
        configParam(VOLUME9_PARAM, 0.f, 2.f, 1.0f, "Channel 9 Volume");
        configParam(VOLUME10_PARAM, 0.f, 2.f, 1.0f, "Channel 10 Volume");
        configParam(VOLUME11_PARAM, 0.f, 2.f, 1.0f, "Channel 11 Volume");
        configParam(VOLUME12_PARAM, 0.f, 2.f, 1.0f, "Channel 12 Volume");
        configParam(VOLUME13_PARAM, 0.f, 2.f, 1.0f, "Channel 13 Volume");
        configParam(VOLUME14_PARAM, 0.f, 2.f, 1.0f, "Channel 14 Volume");
        configParam(VOLUME15_PARAM, 0.f, 2.f, 1.0f, "Channel 15 Volume");
        configParam(VOLUME16_PARAM, 0.f, 2.f, 1.0f, "Channel 16 Volume");
        configParam(MASTER_VOL, 0.f, 2.f, 1.0f, "Master Volume");
        configParam(FEEDBACK_PARAM, 0.f, 11.f, 0.0f, "Feedback");

        configParam(PAN1_PARAM, -1.f, 1.f, 0.f, "Channel 1 Pan");
        configParam(PAN2_PARAM, -1.f, 1.f, 0.f, "Channel 2 Pan");
        configParam(PAN3_PARAM, -1.f, 1.f, 0.f, "Channel 3 Pan");
        configParam(PAN4_PARAM, -1.f, 1.f, 0.f, "Channel 4 Pan");
        configParam(PAN5_PARAM, -1.f, 1.f, 0.f, "Channel 5 Pan");
        configParam(PAN6_PARAM, -1.f, 1.f, 0.f, "Channel 6 Pan");
        configParam(PAN7_PARAM, -1.f, 1.f, 0.f, "Channel 7 Pan");
        configParam(PAN8_PARAM, -1.f, 1.f, 0.f, "Channel 8 Pan");
        configParam(PAN9_PARAM, -1.f, 1.f, 0.f, "Channel 9 Pan");
        configParam(PAN10_PARAM, -1.f, 1.f, 0.f, "Channel 10 Pan");
        configParam(PAN11_PARAM, -1.f, 1.f, 0.f, "Channel 11 Pan");
        configParam(PAN12_PARAM, -1.f, 1.f, 0.f, "Channel 12 Pan");
        configParam(PAN13_PARAM, -1.f, 1.f, 0.f, "Channel 13 Pan");
        configParam(PAN14_PARAM, -1.f, 1.f, 0.f, "Channel 14 Pan");
        configParam(PAN15_PARAM, -1.f, 1.f, 0.f, "Channel 15 Pan");
        configParam(PAN16_PARAM, -1.f, 1.f, 0.f, "Channel 16 Pan");

        configParam(MUTE1_PARAM, 0.0, 1.0, 0.0, "Mute 1" );
        configParam(MUTE2_PARAM, 0.0, 1.0, 0.0, "Mute 2" );
        configParam(MUTE3_PARAM, 0.0, 1.0, 0.0, "Mute 3" );
        configParam(MUTE4_PARAM, 0.0, 1.0, 0.0, "Mute 4" );
        configParam(MUTE5_PARAM, 0.0, 1.0, 0.0, "Mute 5" );
        configParam(MUTE6_PARAM, 0.0, 1.0, 0.0, "Mute 6" );
        configParam(MUTE7_PARAM, 0.0, 1.0, 0.0, "Mute 7" );
        configParam(MUTE8_PARAM, 0.0, 1.0, 0.0, "Mute 8" );
        configParam(MUTE9_PARAM, 0.0, 1.0, 0.0, "Mute 9" );
        configParam(MUTE10_PARAM, 0.0, 1.0, 0.0, "Mute 10" );
        configParam(MUTE11_PARAM, 0.0, 1.0, 0.0, "Mute 11" );
        configParam(MUTE12_PARAM, 0.0, 1.0, 0.0, "Mute 12" );
        configParam(MUTE13_PARAM, 0.0, 1.0, 0.0, "Mute 13" );
        configParam(MUTE14_PARAM, 0.0, 1.0, 0.0, "Mute 14" );
        configParam(MUTE15_PARAM, 0.0, 1.0, 0.0, "Mute 15" );
        configParam(MUTE16_PARAM, 0.0, 1.0, 0.0, "Mute 16" );
        configParam(MUTESIDE_PARAM, 0.0, 1.0, 0.0, "Mute Sidechain" );

        // Configure side and saturation parameters
        configParam(SIDECHAIN_VOLUME_PARAM, 0.f, 2.f, 0.6f, "Sidechain Volume");
        configParam(DUCK_PARAM, 0.f, 1.f, 0.7f, "Duck Amount");
        configParam(DUCK_ATT, -1.f, 1.f, 0.0f, "Duck Attenuverter");
        configParam(FEEDBACK_ATT, -1.f, 1.f, 0.0f, "Feedback Attenuverter");
        configParam(MASTER_VOL_ATT, -1.f, 1.f, 0.0f, "Master Volume Attenuverter");

        configParam(PRESS_PARAM, 0.f, 1.f, 0.f, "Press");
        configParam(PRESS_ATT, -1.f, 1.f, 0.0f, "Press Attenuation");

        // Configure inputs for each channel
        configInput(AUDIO_1L_INPUT, "Channel 1 L / Poly");
        configInput(AUDIO_1R_INPUT, "Channel 1 R / Poly");
        configInput(AUDIO_2L_INPUT, "Channel 2 L");
        configInput(AUDIO_2R_INPUT, "Channel 2 R");
        configInput(AUDIO_3L_INPUT, "Channel 3 L");
        configInput(AUDIO_3R_INPUT, "Channel 3 R");
        configInput(AUDIO_4L_INPUT, "Channel 4 L");
        configInput(AUDIO_4R_INPUT, "Channel 4 R");
        configInput(AUDIO_5L_INPUT, "Channel 5 L");
        configInput(AUDIO_5R_INPUT, "Channel 5 R");
        configInput(AUDIO_6L_INPUT, "Channel 6 L");
        configInput(AUDIO_6R_INPUT, "Channel 6 R");
        configInput(AUDIO_7L_INPUT, "Channel 7 L");
        configInput(AUDIO_7R_INPUT, "Channel 7 R");
        configInput(AUDIO_8L_INPUT, "Channel 8 L");
        configInput(AUDIO_8R_INPUT, "Channel 8 R");
        configInput(AUDIO_9L_INPUT, "Channel 9 L");
        configInput(AUDIO_9R_INPUT, "Channel 9 R");
        configInput(AUDIO_10L_INPUT, "Channel 10 L");
        configInput(AUDIO_10R_INPUT, "Channel 10 R");
        configInput(AUDIO_11L_INPUT, "Channel 11 L");
        configInput(AUDIO_11R_INPUT, "Channel 11 R");
        configInput(AUDIO_12L_INPUT, "Channel 12 L");
        configInput(AUDIO_12R_INPUT, "Channel 12 R");
        configInput(AUDIO_13L_INPUT, "Channel 13 L");
        configInput(AUDIO_13R_INPUT, "Channel 13 R");
        configInput(AUDIO_14L_INPUT, "Channel 14 L");
        configInput(AUDIO_14R_INPUT, "Channel 14 R");
        configInput(AUDIO_15L_INPUT, "Channel 15 L");
        configInput(AUDIO_15R_INPUT, "Channel 15 R");
        configInput(AUDIO_16L_INPUT, "Channel 16 L");
        configInput(AUDIO_16R_INPUT, "Channel 16 R");

        configInput(VCA_CV1_INPUT, "Channel 1 VCA CV / Poly");
        configInput(VCA_CV2_INPUT, "Channel 2 VCA CV");
        configInput(VCA_CV3_INPUT, "Channel 3 VCA CV");
        configInput(VCA_CV4_INPUT, "Channel 4 VCA CV");
        configInput(VCA_CV5_INPUT, "Channel 5 VCA CV");
        configInput(VCA_CV6_INPUT, "Channel 6 VCA CV");
        configInput(VCA_CV7_INPUT, "Channel 7 VCA CV");
        configInput(VCA_CV8_INPUT, "Channel 8 VCA CV");
        configInput(VCA_CV9_INPUT, "Channel 9 VCA CV");
        configInput(VCA_CV10_INPUT, "Channel 10 VCA CV");
        configInput(VCA_CV11_INPUT, "Channel 11 VCA CV");
        configInput(VCA_CV12_INPUT, "Channel 12 VCA CV");
        configInput(VCA_CV13_INPUT, "Channel 13 VCA CV");
        configInput(VCA_CV14_INPUT, "Channel 14 VCA CV");
        configInput(VCA_CV14_INPUT, "Channel 15 VCA CV");
        configInput(VCA_CV16_INPUT, "Channel 16 VCA CV");

        configInput(VCA_SIDECHAIN_INPUT, "Sidechain VCA CV");

        configInput(PAN_CV1_INPUT, "Channel 1 Pan CV / Poly");
        configInput(PAN_CV2_INPUT, "Channel 2 Pan CV");
        configInput(PAN_CV3_INPUT, "Channel 3 Pan CV");
        configInput(PAN_CV4_INPUT, "Channel 4 Pan CV");
        configInput(PAN_CV5_INPUT, "Channel 5 Pan CV");
        configInput(PAN_CV6_INPUT, "Channel 6 Pan CV");
        configInput(PAN_CV7_INPUT, "Channel 7 Pan CV");
        configInput(PAN_CV8_INPUT, "Channel 8 Pan CV");
        configInput(PAN_CV9_INPUT, "Channel 9 Pan CV");
        configInput(PAN_CV10_INPUT, "Channel 10 Pan CV");
        configInput(PAN_CV11_INPUT, "Channel 11 Pan CV");
        configInput(PAN_CV12_INPUT, "Channel 12 Pan CV");
        configInput(PAN_CV13_INPUT, "Channel 13 Pan CV");
        configInput(PAN_CV14_INPUT, "Channel 14 Pan CV");
        configInput(PAN_CV15_INPUT, "Channel 15 Pan CV");
        configInput(PAN_CV16_INPUT, "Channel 16 Pan CV");

        configInput(MUTE_1_INPUT, "Channel 1 Mute CV / Poly");
        configInput(MUTE_2_INPUT, "Channel 2 Mute CV");
        configInput(MUTE_3_INPUT, "Channel 3 Mute CV");
        configInput(MUTE_4_INPUT, "Channel 4 Mute CV");
        configInput(MUTE_5_INPUT, "Channel 5 Mute CV");
        configInput(MUTE_6_INPUT, "Channel 6 Mute CV");
        configInput(MUTE_7_INPUT, "Channel 7 Mute CV");
        configInput(MUTE_8_INPUT, "Channel 8 Mute CV");
        configInput(MUTE_9_INPUT, "Channel 9 Mute CV");
        configInput(MUTE_10_INPUT, "Channel 10 Mute CV");
        configInput(MUTE_11_INPUT, "Channel 11 Mute CV");
        configInput(MUTE_12_INPUT, "Channel 12 Mute CV");
        configInput(MUTE_13_INPUT, "Channel 13 Mute CV");
        configInput(MUTE_14_INPUT, "Channel 14 Mute CV");
        configInput(MUTE_15_INPUT, "Channel 15 Mute CV");
        configInput(MUTE_16_INPUT, "Channel 16 Mute CV");

        // Side and saturation CV inputs
        configInput(SIDECHAIN_INPUT_L, "Sidechain L");
        configInput(SIDECHAIN_INPUT_R, "Sidechain R");
        configInput(DUCK_CV, "Duck CV");
        configInput(PRESS_CV_INPUT, "Press CV");
        configInput(FEEDBACK_CV, "Feedback CV");
        configInput(MASTER_VOL_CV, "Master Volume CV");

        // Outputs
        configOutput(AUDIO_OUTPUT_L, "Main Out L");
        configOutput(AUDIO_OUTPUT_R, "Main Out R");
        
        // Calculate transition samples (default assuming 44100 Hz, will update in process)
        transitionSamples = 0.005 * 44100; // 5 ms * sample rate
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
		int audioChannels[16] = {0}; // Number of polyphonic channels for AUDIO inputs
		int lChannels[16] = {0}; int rChannels[16] = {0};
		bool isConnectedL[16] = {false}; bool isConnectedR[16] = {false};
		int vcaChannels[16] = {0}; // Number of polyphonic channels for VCA CV inputs
		int panChannels[16] = {0};   // Number of polyphonic channels for PAN CV inputs
		int muteChannels[16] = {0};  // Number of polyphonic channels for MUTE inputs
		
		// Arrays to store the current input signals and connectivity status
		int activeAudio[16] = {-1};        // Stores the number of the previous active channel for the AUDIO inputs
		int activeVcaChannel[16] = {-1}; // Stores the number of the previous active channel for the VCA CV 
		int activePanChannel[16] = {-1};   // Stores the number of the previous active channel for the PAN CV 
		int activeMuteChannel[16] = {-1};  // Stores the number of the previous active channel for the MUTE
		//initialize all active channels with -1, indicating nothing connected.

		// Scan all inputs to determine the polyphony
		// Scan all inputs to determine the polyphony
		for (int i = 0; i < 16; i++) {
		
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
				if (audioChannels[activeAudio[i-1]] >= (i - activeAudio[i-1])) {
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
				if (vcaChannels[activeVcaChannel[i-1]] >= (i - activeVcaChannel[i-1])) {
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
				if (panChannels[activePanChannel[i-1]] >= (i - activePanChannel[i-1])) {
					activePanChannel[i] = activePanChannel[i-1]; // Carry over the active channel
				} else {
					activePanChannel[i] = -1; // No valid polyphonic channel to carry over
				}
			} else {
				activePanChannel[i] = -1; // Explicitly reset if not connected
			}
		
			// Update the MUTE channels (your original fix)
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


		// Process each of the sixteen main channels
		for (int i = 0; i < 16; i++) {

            bool inputActive = false;

			// Determine if the current channel's input is connected
			isConnectedL[i] = inputs[AUDIO_1L_INPUT + 2 * i].isConnected();
			isConnectedR[i] = inputs[AUDIO_1R_INPUT + 2 * i].isConnected();

		    //if something is connected to any audio input we use the top value of that input. Normalizing if we only have 1 connection in either L or R.
			if (activeAudio[i] == i) { 
			    inputActive = true;
				// Handle mono to stereo routing
				if (!isConnectedR[i] && isConnectedL[i]) { //Left only
					inputL[i]=inputs[AUDIO_1L_INPUT + 2 * i].getPolyVoltage(0);
					inputR[i]=inputs[AUDIO_1L_INPUT + 2 * i].getPolyVoltage(0); //Normalize L to R
				} 
				if (!isConnectedL[i] && isConnectedR[i]) { //Right only
					inputL[i]=inputs[AUDIO_1R_INPUT + 2 * i].getPolyVoltage(0); //Normalize R to L
					inputR[i]=inputs[AUDIO_1R_INPUT + 2 * i].getPolyVoltage(0);
				}
				if (isConnectedR[i] && isConnectedL[i]) { //Both
					inputL[i]=inputs[AUDIO_1L_INPUT + 2 * i].getPolyVoltage(0);
					inputR[i]=inputs[AUDIO_1R_INPUT + 2 * i].getPolyVoltage(0);
				} 
			} else if (activeAudio[i] > -1){ //If channel is not active, then we look at activeAudio[i] to get the previous active channel number
			    // Now we compute which channel we need to grab
			    int diffBetween = i - activeAudio[i];
			    int currentChannelMax =  audioChannels[activeAudio[i]] ;	
			    if (currentChannelMax - diffBetween > 0){    //If we are before the last poly channel
			        inputActive = true;
					// Handle mono to stereo routing
					if (!isConnectedR[ activeAudio[i] ] && isConnectedL[ activeAudio[i] ]) { //Left only
						inputL[i]=inputs[AUDIO_1L_INPUT + 2 * activeAudio[i]].getPolyVoltage(diffBetween);
						inputR[i]=inputs[AUDIO_1L_INPUT + 2 * activeAudio[i]].getPolyVoltage(diffBetween); //Normalize L to R
					} 
					if (!isConnectedL[ activeAudio[i] ] && isConnectedR[ activeAudio[i] ]) { //Right only
						inputL[i]=inputs[AUDIO_1R_INPUT + 2 * activeAudio[i]].getPolyVoltage(diffBetween); //Normalize R to L
						inputR[i]=inputs[AUDIO_1R_INPUT + 2 * activeAudio[i]].getPolyVoltage(diffBetween);
					}
					if (isConnectedR[ activeAudio[i] ] && isConnectedL[ activeAudio[i] ]) { //Both
						inputL[i]=inputs[AUDIO_1L_INPUT + 2 * activeAudio[i]].getPolyVoltage(diffBetween);
						inputR[i]=inputs[AUDIO_1R_INPUT + 2 * activeAudio[i]].getPolyVoltage(diffBetween);
					} 
				} 
			}

            if (inputActive) {
                inputCount += 1.0f;
            } else {
                // Reset envelopes if inputs are not connected
                filteredEnvelopeL[i] = 0.0f;
                filteredEnvelopeR[i] = 0.0f;
                filteredEnvelope[i] = 0.0f;
            }
	  
	        /////////////
	        //// Deal with polyphonic Mute inputs
	        
			bool inputMute = false;
		
			// Check if mute is triggered by the input or previous poly input
			if (activeMuteChannel[i] == i) { //if there's an input here
				inputMute = inputs[MUTE_1_INPUT + i].getPolyVoltage(0) > 0.5f;
			} else if (activeMuteChannel[i] > -1) { //otherwise check the previous channel
			    // Now we compute which channel we need to grab
			    int diffBetween = i - activeMuteChannel[i];
			    int currentChannelMax =  muteChannels[activeMuteChannel[i]] ;	
			    if (currentChannelMax - diffBetween > 0) {    //If we are before the last poly channel
					inputMute = inputs[ MUTE_1_INPUT + activeMuteChannel[i] ].getPolyVoltage(diffBetween) > 0.5f;
				}
		    } 
		
			// Determine final mute state
			if (activeMuteChannel[i] > -1) {
				// If CV is connected or if it's poly CV, ignore the button
				muteState[i] = inputMute;
				muteLatch[i] = false; // Reset the latch
			} else {
				// If no CV is connected, use the button for muting
				if (params[MUTE1_PARAM + i].getValue() > 0.5f) {
					if (!muteLatch[i]) {
						muteLatch[i] = true;
						muteState[i] = !muteState[i];
						transitionCount[i] = transitionSamples;  // Reset the transition count
					}
				} else {
					muteLatch[i] = false; // Release latch if button 
				}
				
				// Ensure the mute state is handled
				muteState[i] = muteState[i];
			}
        
            if (transitionCount[i] > 0) {
                float fadeStep = (muteState[i] ? -1.0f : 1.0f) / transitionSamples;
                fadeLevel[i] += fadeStep;
                if ((muteState[i] && fadeLevel[i] < 0.0f) || (!muteState[i] && fadeLevel[i] > 1.0f)) {
                    fadeLevel[i] = muteState[i] ? 0.0f : 1.0f;
                    transitionCount[i] = 0;  // End transition
                }
                transitionCount[i]--;
            } else {
                fadeLevel[i] = muteState[i] ? 0.0f : 1.0f;
            }

            inputL[i] *= fadeLevel[i];
            inputR[i] *= fadeLevel[i];

            // Apply VCA control and volume
			if (activeVcaChannel[i] == i) {
				inputL[i] *= clamp(inputs[VCA_CV1_INPUT + i].getPolyVoltage(0) / 10.f, 0.f, 2.f);
				inputR[i] *= clamp(inputs[VCA_CV1_INPUT + i].getPolyVoltage(0) / 10.f, 0.f, 2.f);
			} else if (activeVcaChannel[i] > -1) {
				// Now we compute which channel we need to grab
				int diffBetween = i - activeVcaChannel[i];
				int currentChannelMax =  vcaChannels[activeVcaChannel[i]] ;	
				if (currentChannelMax - diffBetween > 0) {    //If we are before the last poly channel
					inputL[i] *= clamp(inputs[VCA_CV1_INPUT + activeVcaChannel[i]].getPolyVoltage(diffBetween) / 10.f, 0.f, 2.f);
					inputR[i] *= clamp(inputs[VCA_CV1_INPUT + activeVcaChannel[i]].getPolyVoltage(diffBetween) / 10.f, 0.f, 2.f);
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
				if (currentChannelMax - diffBetween > 0) {    //If we are before the last poly channel
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
            
        }

        compressionAmountL = compressionAmountL/((inputCount+1.f)*5.0f); //divide by the expected ceiling 
        compressionAmountR = compressionAmountR/((inputCount+1.f)*5.0f); //process L and R separately 

        float pressAmount = params[PRESS_PARAM].getValue();
        if(inputs[PRESS_CV_INPUT].isConnected()){
            pressAmount += inputs[PRESS_CV_INPUT].getVoltage()*params[PRESS_ATT].getValue();
        }
        pressAmount = clamp(pressAmount, 0.0f, 1.0f);
        
        pressTotalL = (1.0f*(1-pressAmount) + pressAmount/compressionAmountL)*16/inputCount;
        pressTotalR = (1.0f*(1-pressAmount) + pressAmount/compressionAmountR)*16/inputCount;
        
        // MIX the channels scaled by compression
        for (int i=0; i<16; i++){  
            if (compressionAmountL > 0.0f && inputCount>0.0f){ //avoid div by zero
                    mixL += inputL[i]*pressTotalL;
            } 
            if (compressionAmountR > 0.0f && inputCount>0.0f){ //avoid div by zero
                    mixR += inputR[i]*pressTotalR;
            } 

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

        distortTotalL = log1p(fmax(mixL-85.f, 0.0f)) * (85.0f / log1p(85)); 
        distortTotalR = log1p(fmax(mixR-85.f, 0.0f)) * (85.0f / log1p(85)); 

        // Apply ADAA
        float maxHeadRoom = 111.7f; //exceeding this number results in strange wavefolding due to the polytanh bad fit beyond this point
        mixL = clamp(mixL, -maxHeadRoom, maxHeadRoom);
        mixR = clamp(mixR, -maxHeadRoom, maxHeadRoom);
        mixL = applyADAA(mixL/85.f, lastOutputL, sampleRate); //85 is 17x5v
        mixR = applyADAA(mixR/85.f, lastOutputR, sampleRate);
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

        volTotalL = fmax(outputL * decayRate, fabs(outputL)) ;
        volTotalR = fmax(outputR * decayRate, fabs(outputR)) ;

        // Check output connections to implement conditional mono-to-stereo mirroring
        if (outputs[AUDIO_OUTPUT_L].isConnected() && !outputs[AUDIO_OUTPUT_R].isConnected()) {
            // Only left output is connected, copy to right output
            outputs[AUDIO_OUTPUT_L].setVoltage(outputL);
            outputs[AUDIO_OUTPUT_R].setVoltage(outputL);  // Mirror left to right
        } else if (!outputs[AUDIO_OUTPUT_L].isConnected() && outputs[AUDIO_OUTPUT_R].isConnected()) {
            // Only right output is connected, copy to left output
            outputs[AUDIO_OUTPUT_L].setVoltage(outputR);  // Mirror right to left
            outputs[AUDIO_OUTPUT_R].setVoltage(outputR);
        } else {
            // Both outputs are connected, or neither are, operate in true stereo or silence both
            outputs[AUDIO_OUTPUT_L].setVoltage(outputL);
            outputs[AUDIO_OUTPUT_R].setVoltage(outputR);
        }

        // Update lights periodically
        updateLights();
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
        float x4 = x2 * x2;
        float x6 = x4 * x2;
        float x8 = x4 * x4;
        return x2 / 2.0f - x4 / 12.0f + x6 / 45.0f - 17.0f * x8 / 2520.0f;
    }

    float polyTanh(float x) {
        float x2 = x * x;       // x^2
        float x3 = x2 * x;      // x^3
        float x5 = x3 * x2;     // x^5
        float x7 = x5 * x2;     // x^7
        return x - x3 / 3.0f + (2.0f * x5) / 15.0f - (17.0f * x7) / 315.0f;
    }

    float polySin(float x) {
        float x2 = x * x;       // x^2
        float x3 = x * x2;      // x^3
        float x5 = x3 * x2;     // x^5
        float x7 = x5 * x2;     // x^7
        return x - x3 / 6.0f + x5 / 120.0f - x7 / 5040.0f;
    }

    float polyCos(float x) {
        float x2 = x * x;       // x^2
        float x4 = x2 * x2;     // x^4
        float x6 = x4 * x2;     // x^6
        return 1.0f - x2 / 2.0f + x4 / 24.0f - x6 / 720.0f;
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

		// Handle muting with fade transition
		if (params[MUTESIDE_PARAM].getValue() > 0.5f) {
			if (!muteLatch[16]) {
				muteLatch[16] = true;
				muteState[16] = !muteState[16];
				transitionCount[16] = transitionSamples;  // Reset the transition count
			}
		} else {
			muteLatch[16] = false;
		}
	
		if (transitionCount[16] > 0) {
			float fadeStep = (muteState[16] ? -1.0f : 1.0f) / transitionSamples;
			fadeLevel[16] += fadeStep;
			if ((muteState[16] && fadeLevel[16] < 0.0f) || (!muteState[16] && fadeLevel[16] > 1.0f)) {
				fadeLevel[16] = muteState[16] ? 0.0f : 1.0f;
				transitionCount[16] = 0;  // End transition
			}
			transitionCount[16]--;
		} else {
			fadeLevel[16] = muteState[16] ? 0.0f : 1.0f;
		}

		if (!mutedSideDucks){	//only fade out the sound if mixing it
			sideL *= fadeLevel[16];
			sideR *= fadeLevel[16];
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
				if (muteState[16]) {
					mixL = (mixL * duckingFactorL) ;
					mixR = (mixR * duckingFactorR) ;
					
				} else {
					mixL = (mixL * duckingFactorL) + sideL;
					mixR = (mixR * duckingFactorR) + sideR;				
				}
			}
		}
	}//end process side
   
    void updateLights() {
        if (++cycleCount >= 2000) {
            for (int i = 0; i < 16; i++) {
                lights[VOLUME1_LIGHT + i].setBrightness(filteredEnvelope[i]);
                // Update mute lights based on the mute button state
                if (muteState[i]){
                    lights[MUTE1_LIGHT + i].setBrightness(1.0f);
                } else {
                    lights[MUTE1_LIGHT + i].setBrightness(0.0f);
                }
            }
            lights[BASS_VOLUME_LIGHT].setBrightness(sideEnvelope);
            if (muteState[16]){
                lights[MUTESIDE_LIGHT].setBrightness(1.0f);
            } else {
                lights[MUTESIDE_LIGHT].setBrightness(0.0f);
            } 
                       
            // Update PRESS lights with segmented levels
            updateSegmentedLights(PRESS_LIGHT1L, pressTotalL, 35.0f, 20);
            updateSegmentedLights(PRESS_LIGHT1R, pressTotalR, 35.0f, 20);

            // Update FEED lights with segmented levels
            updateSegmentedLights(FEED_LIGHT1L, distortTotalL, 100.0f, 20);
            updateSegmentedLights(FEED_LIGHT1R, distortTotalR, 100.0f, 20);

            // Update VOL lights with segmented levels
            updateSegmentedLights(VOL_LIGHT1, volTotalL, 10.0f, 20);
            // Update VOL lights with segmented levels
            updateSegmentedLights(VOL_LIGHT1R, volTotalR, 10.0f, 20);

            cycleCount = 0;
        }
    } 
    
    void updateSegmentedLights(int startLightId, float totalValue, float maxValue, int numLights) {
        float normalizedValue = totalValue / maxValue;
        int fullLights = static_cast<int>(normalizedValue * numLights);
        float fractionalBrightness = (normalizedValue * numLights) - fullLights;

        for (int i = 0; i < numLights; i++) {
            if (i < fullLights) {
                lights[startLightId + i].setBrightness(1.0f); // Full brightness for fully covered segments
            } else if (i == fullLights) {
                lights[startLightId + i].setBrightness(fractionalBrightness); // Partial brightness for the last partially covered segment
            } else {
                float dimming = lights[startLightId + i].getBrightness(); 
                lights[startLightId + i].setBrightness(dimming * 0.75f); 
            }
        }
    }    
       
};

struct PreeeeeeeeeeessedDuckWidget : ModuleWidget {
    PreeeeeeeeeeessedDuckWidget(PreeeeeeeeeeessedDuck* module) {
        setModule(module);

        setPanel(createPanel(
                asset::plugin(pluginInstance, "res/PreeeeeeeeeeessedDuck.svg"),
                asset::plugin(pluginInstance, "res/PreeeeeeeeeeessedDuck-dark.svg")
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
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::SIDECHAIN_INPUT_L ));
        yPos += Spacing;
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::SIDECHAIN_INPUT_R ));
   
        // Sidechain Volume slider with light
        yPos += 40+Spacing;
        addParam(createLightParamCentered<VCVLightSlider<YellowLight>>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::SIDECHAIN_VOLUME_PARAM , PreeeeeeeeeeessedDuck::BASS_VOLUME_LIGHT));

        // Sidechain VCA CV input
        yPos += 38+Spacing;
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::VCA_SIDECHAIN_INPUT ));

        // Ducking amount knob
        yPos += 1.25*Spacing;
        addParam(createParamCentered<RoundBlackKnob>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::DUCK_PARAM));

        // Ducking attenuator
        yPos = channelOffset.y + 5*Spacing + 84;
        addParam(createParamCentered<Trimpot>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::DUCK_ATT));

        // Ducking CV input
        yPos += Spacing;
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::DUCK_CV));

        // Sidechain Mute
        yPos = channelOffset.y + 4*Spacing + 170;
        addParam(createParamCentered<LEDButton>           (Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::MUTESIDE_PARAM));
        addChild(createLightCentered<SmallLight<RedLight>>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::MUTESIDE_LIGHT));

        yPos = channelOffset.y;
        // Loop through each channel
        for (int i = 0; i < 16; i++) {
            xPos = 1.25*sliderX + channelOffset.x + i * sliderX;

            // Audio inputs
            addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::AUDIO_1L_INPUT + 2 * i));
            yPos += Spacing;
            addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::AUDIO_1R_INPUT + 2 * i));

            // Volume slider with light
            yPos += 40+Spacing;
            addParam(createLightParamCentered<VCVLightSlider<YellowLight>>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::VOLUME1_PARAM + i, PreeeeeeeeeeessedDuck::VOLUME1_LIGHT+ i));

            // VCA CV input
            yPos += 38+Spacing;
            addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::VCA_CV1_INPUT + i));

            // Pan knob
            yPos += Spacing + 20;
            addParam(createParamCentered<RoundBlackKnob>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::PAN1_PARAM + i));

            // Pan CV input
            yPos += Spacing;
            addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::PAN_CV1_INPUT + i));

            // Mute
            yPos += 1.2*Spacing;
            addParam(createParamCentered<LEDButton>           (Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::MUTE1_PARAM + i));
            addChild(createLightCentered<SmallLight<RedLight>>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::MUTE1_LIGHT + i));
            yPos += 0.8*Spacing;
            addInput(createInputCentered<ThemedPJ301MPort> (Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::MUTE_1_INPUT + i));

            // Reset yPos for next channel
            yPos = channelOffset.y;
        }
            
        // Global controls for saturation and side processing (placing these at the end of channels)
        xPos += 1.75*sliderX; // Shift to the right of the last channel
        yPos = channelOffset.y + 0.5*Spacing;

        // Saturation ceiling knob
        addParam(createParamCentered<RoundHugeBlackKnob>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::PRESS_PARAM));

        // Add Ring of Lights
        addLightsAroundKnob(module, xPos, yPos, PreeeeeeeeeeessedDuck::PRESS_LIGHT1R, 20, 31.f);
        addLightsAroundKnob(module, xPos, yPos, PreeeeeeeeeeessedDuck::PRESS_LIGHT1L, 20, 35.f);

        // Saturation ceiling attenuator
        yPos += 1.5*Spacing ;
        xPos -= .5*sliderX; // Shift to the right of the last channel
        addParam(createParamCentered<Trimpot>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::PRESS_ATT));

        // Saturation CV input
        xPos += 1.0*sliderX; // Shift to the right of the last channel
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::PRESS_CV_INPUT));

        xPos -= .5*sliderX; // Shift to the right of the last channel
        yPos += 2.1*Spacing;

        // FEEDBACK
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::FEEDBACK_PARAM));

        // Add Ring of Lights
        addLightsAroundKnob(module, xPos, yPos, PreeeeeeeeeeessedDuck::FEED_LIGHT1R, 20, 22.5f);
        addLightsAroundKnob(module, xPos, yPos, PreeeeeeeeeeessedDuck::FEED_LIGHT1L, 20, 26.5f);

        // FEEDBACK attenuator
        yPos += 1.3*Spacing;
        xPos -= .5*sliderX; // Shift to the right of the last channel
        addParam(createParamCentered<Trimpot>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::FEEDBACK_ATT));

        // FEEDBACK CV input
        xPos += 1.0*sliderX; // Shift to the right of the last channel

        addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::FEEDBACK_CV));
        xPos -= .5*sliderX; // Shift to the right of the last channel

        // Master Volume
        yPos = channelOffset.y + 4.3*Spacing + 85;
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::MASTER_VOL));

        // Add Ring of Lights for L/R channels
        addLightsAroundKnob(module, xPos, yPos, PreeeeeeeeeeessedDuck::VOL_LIGHT1R, 20, 22.5f);
        addLightsAroundKnob(module, xPos, yPos, PreeeeeeeeeeessedDuck::VOL_LIGHT1, 20, 26.5f);

        // Master Volume attenuator
        yPos += 1.3*Spacing;
        xPos -= .5*sliderX; // Shift to the right of the last channel
        addParam(createParamCentered<Trimpot>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::MASTER_VOL_ATT));

        // Master Volume CV input
        xPos += 1.0*sliderX; // Shift to the right of the last channel

        addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::MASTER_VOL_CV));
        xPos -= .5*sliderX; // Shift to the right of the last channel

        xPos -= .5*sliderX; // Shift to the right of the last channel

        // Outputs
        yPos = channelOffset.y + 4*Spacing + 170;
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::AUDIO_OUTPUT_L));
        xPos += 1*sliderX; // Shift to the right of the last channel
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PreeeeeeeeeeessedDuck::AUDIO_OUTPUT_R));

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
    
    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);

        PreeeeeeeeeeessedDuck* PreeeeeeeeeeessedDuckModule = dynamic_cast<PreeeeeeeeeeessedDuck*>(module);
        assert(PreeeeeeeeeeessedDuckModule); // Ensure the cast succeeds

        // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());

        // VOctCV menu item
        struct FilterMenuItem : MenuItem {
            PreeeeeeeeeeessedDuck* PreeeeeeeeeeessedDuckModule;
            void onAction(const event::Action& e) override {
                // Toggle the "Apply Filters" mode
                PreeeeeeeeeeessedDuckModule->applyFilters = !PreeeeeeeeeeessedDuckModule->applyFilters;
            }
            void step() override {
                // Update the display to show a checkmark when the mode is active
                rightText = PreeeeeeeeeeessedDuckModule->applyFilters ? "✔" : "";
                MenuItem::step();
            }
        };
        
        FilterMenuItem* filterItem = new FilterMenuItem();
        filterItem->text = "Apply Filters";
        filterItem->PreeeeeeeeeeessedDuckModule = PreeeeeeeeeeessedDuckModule;
        menu->addChild(filterItem);
        
        // MutedSideDucks menu item
        struct MutedSideDucksMenuItem : MenuItem {
            PreeeeeeeeeeessedDuck* PreeeeeeeeeeessedDuckModule;
            void onAction(const event::Action& e) override {
                // Toggle the "Muted Side Ducks" mode
                PreeeeeeeeeeessedDuckModule->mutedSideDucks = !PreeeeeeeeeeessedDuckModule->mutedSideDucks;
            }
            void step() override {
                // Update the display to show a checkmark when the mode is active
                rightText = PreeeeeeeeeeessedDuckModule->mutedSideDucks ? "✔" : "";
                MenuItem::step();
            }
        };
        
        // Create the MutedSideDucks menu item and add it to the menu
        MutedSideDucksMenuItem* mutedSideDucksItem = new MutedSideDucksMenuItem();
        mutedSideDucksItem->text = "Muted Sidechain still Ducks";
        mutedSideDucksItem->PreeeeeeeeeeessedDuckModule = PreeeeeeeeeeessedDuckModule;
        menu->addChild(mutedSideDucksItem);        
        
    }        
};

Model* modelPreeeeeeeeeeessedDuck = createModel<PreeeeeeeeeeessedDuck, PreeeeeeeeeeessedDuckWidget>("PreeeeeeeeeeessedDuck");
