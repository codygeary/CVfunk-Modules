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
        NUM_INPUTS
    };
    enum OutputIds {
        AUDIO_OUTPUT_L, AUDIO_OUTPUT_R, 
        NUM_OUTPUTS
    };
    enum LightIds {
        VOLUME1_LIGHT, VOLUME2_LIGHT, VOLUME3_LIGHT, VOLUME4_LIGHT, VOLUME5_LIGHT, VOLUME6_LIGHT, BASS_VOLUME_LIGHT, 
        MUTE1_LIGHT, MUTE2_LIGHT, MUTE3_LIGHT, MUTE4_LIGHT, MUTE5_LIGHT, MUTE6_LIGHT, MUTESIDE_LIGHT,
        NUM_LIGHTS
    };

    bool applyFilters = true;

    //for tracking the mute state of each channel
    bool muteLatch[7] = {false,false,false,false,false,false,false};
    bool muteState[7] = {false,false,false,false,false,false,false};

    // Serialization method to save module state
    json_t* dataToJson() override {
        json_t* rootJ = json_object();

        // Save the state of retriggerEnabled as a boolean
        json_object_set_new(rootJ, "applyFilters", json_boolean(applyFilters));

        // Save the muteLatch and muteState arrays
        json_t* muteLatchJ = json_array();
        json_t* muteStateJ = json_array();

        for (int i = 0; i < 7; i++) {
            json_array_append_new(muteLatchJ, json_boolean(muteLatch[i]));
            json_array_append_new(muteStateJ, json_boolean(muteState[i]));
        }

        json_object_set_new(rootJ, "muteLatch", muteLatchJ);
        json_object_set_new(rootJ, "muteState", muteStateJ);

        return rootJ;
    }

    // Deserialization method to load module state
    void dataFromJson(json_t* rootJ) override {
        // Load the state of retriggerEnabled
        json_t* applyFiltersJ = json_object_get(rootJ, "applyFilters");
        if (applyFiltersJ) {
            // Use json_is_true() to check if the JSON value is true; otherwise, set to false
            applyFilters = json_is_true(applyFiltersJ);
        }   
        
        // Load muteLatch and muteState arrays
        json_t* muteLatchJ = json_object_get(rootJ, "muteLatch");
        json_t* muteStateJ = json_object_get(rootJ, "muteState");

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
                   
    }

    float bassPeakL = 0.0f;
    float bassPeakR = 0.0f;
    float envPeakL[6] = {0.0f};
    float envPeakR[6] = {0.0f};
    float peak[6] = {0.0f};
    float envelope[6] = {0.0f}; 
    int cycleCount = 0;

    // Arrays to hold last computed values for differentiation
    float lastOutputL = 0.0f;
    float lastOutputR = 0.0f;
    float bassEnvelope = 0.0f;
    float inputL[6] = {0.0f};
    float inputR[6] = {0.0f};
    float panL[6] = {0.0f};
    float panR[6] = {0.0f};
    float lastPan[6] = {0.0f}; 
    bool initialized[6] = {false};  
    float filteredEnvelope[6] = {0.0f}; 
    float filteredBassEnvelope = 0.0f;
    float alpha = 0.01f;

    //for filters
    float lastInputL = 0.0f;
    float lastInputR = 0.0f;
    float lastHPOutputL = 0.0f;
    float lastHPOutputR = 0.0f;
    float lastLPOutputL = 0.0f;
    float lastLPOutputR = 0.0f;

    // Declare high-pass filter
    SecondOrderHPF hpfL, hpfR;

    //for mute transition
    float transitionTimeMs = 10000.f; // Transition time in milliseconds
    float transitionSamples; // Number of samples to complete the transition
    float fadeLevel[7] = {1.0f}; 
    int transitionCount[7] = {0};  // Array to track transition progress for each channel
    float targetFadeLevel[7] = {0.0f};

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

        // Configure bass and saturation parameters
        configParam(SIDECHAIN_VOLUME_PARAM, 0.f, 2.f, 0.6f, "Sidechain Volume");
        configParam(DUCK_PARAM, 0.f, 1.f, 0.7f, "Duck Amount");
        configParam(DUCK_ATT, -1.f, 1.f, 0.0f, "Duck Attenuation");
        configParam(FEEDBACK_ATT, -1.f, 1.f, 0.0f, "Feedback Attenuation");
        configParam(MASTER_VOL_ATT, -1.f, 1.f, 0.0f, "Master Volume Attenuation");

        configParam(PRESS_PARAM, 0.f, 1.f, 0.f, "Press");
        configParam(PRESS_ATT, -1.f, 1.f, 0.0f, "Press Attenuation");

        // Configure inputs for each channel
        configInput(AUDIO_1L_INPUT, "Channel 1 L");
        configInput(AUDIO_1R_INPUT, "Channel 1 R");
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

        configInput(VCA_CV1_INPUT, "Channel 1 VCA CV ");
        configInput(VCA_CV2_INPUT, "Channel 2 VCA CV");
        configInput(VCA_CV3_INPUT, "Channel 3 VCA CV");
        configInput(VCA_CV4_INPUT, "Channel 4 VCA CV");
        configInput(VCA_CV5_INPUT, "Channel 5 VCA CV");
        configInput(VCA_CV6_INPUT, "Channel 6 VCA CV");
        configInput(VCA_SIDECHAIN_INPUT, "Sidechain VCA CV");

        configInput(PAN_CV1_INPUT, "Channel 1 Pan CV");
        configInput(PAN_CV2_INPUT, "Channel 2 Pan CV");
        configInput(PAN_CV3_INPUT, "Channel 3 Pan CV");
        configInput(PAN_CV4_INPUT, "Channel 4 Pan CV");
        configInput(PAN_CV5_INPUT, "Channel 5 Pan CV");
        configInput(PAN_CV6_INPUT, "Channel 6 Pan CV");

        // Bass and saturation CV inputs
        configInput(SIDECHAIN_INPUT_L, "Sidechain L");
        configInput(SIDECHAIN_INPUT_R, "Sidechain R");
        configInput(DUCK_CV, "Duck CV");
        configInput(PRESS_CV_INPUT, "Press CV");
        configInput(FEEDBACK_CV, "Feedback CV");
        configInput(MASTER_VOL_CV, "Master Volume CV");

        // Outputs
        
        //configOutput(SURVEY, "Survey Test Out"); //hidden output for testing

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

        float compressionAmount = 0.0f;
        float inputCount = 0.0f;

        // Process each of the six main channels
        for (int i = 0; i < 6; i++) {
           
            // Initially check connection and set initial input values
            bool isConnectedL = inputs[AUDIO_1L_INPUT + 2 * i].isConnected();
            bool isConnectedR = inputs[AUDIO_1R_INPUT + 2 * i].isConnected();
            inputL[i] = isConnectedL ? inputs[AUDIO_1L_INPUT + 2 * i].getVoltage() : 0.0f;
            inputR[i] = isConnectedR ? inputs[AUDIO_1R_INPUT + 2 * i].getVoltage() : 0.0f;

            // Handle mono to stereo routing
            if (!isConnectedL && isConnectedR) {
                inputL[i] = inputR[i];
            }
            if (!isConnectedR && isConnectedL) {
                inputR[i] = inputL[i];
            } 

            bool inputActive = false; 
            if ( isConnectedR || isConnectedL ) {
                inputCount += 1.0f;
                inputActive = true;
            }

            // Mute logic
            if (params[MUTE1_PARAM + i].getValue() > 0.5f) {
                if (!muteLatch[i]) {
                    muteLatch[i] = true;
                    muteState[i] = !muteState[i];
                    transitionCount[i] = transitionSamples;  // Reset the transition count
                }
            } else {
                muteLatch[i] = false;
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
            if (inputs[VCA_CV1_INPUT + i].isConnected()) {
                inputL[i] *= clamp(inputs[VCA_CV1_INPUT + i].getVoltage() / 10.f, 0.f, 2.f);
                inputR[i] *= clamp(inputs[VCA_CV1_INPUT + i].getVoltage() / 10.f, 0.f, 2.f);
            }

            float vol = params[VOLUME1_PARAM + i].getValue();
            inputL[i] *= vol;
            inputR[i] *= vol;

            // Simple peak detection using the absolute maximum of the current input
            envPeakL[i] = fmax(envPeakL[i] * decayRate, fabs(inputL[i]));
            envPeakR[i] = fmax(envPeakR[i] * decayRate, fabs(inputR[i]));
            envelope[i] = (envPeakL[i] + envPeakR[i]) / 2.0f;

            if (inputActive){
                filteredEnvelope[i] = fmax(filteredEnvelope[i],0.1f);
            }

            filteredEnvelope[i] = alpha * envelope[i] + (1 - alpha) * filteredEnvelope[i];
            compressionAmount += filteredEnvelope[i];

            // Apply panning
            float pan = params[PAN1_PARAM + i].getValue();
            if (inputs[PAN_CV1_INPUT + i].isConnected()) {
                pan += inputs[PAN_CV1_INPUT + i].getVoltage() / 5.f; // Scale CV influence
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

        compressionAmount = compressionAmount/((inputCount+1.f)*5.0f); //divide by the expected ceiling 

        float pressAmount = params[PRESS_PARAM].getValue();
        if(inputs[PRESS_CV_INPUT].isConnected()){
            pressAmount += inputs[PRESS_CV_INPUT].getVoltage()*params[PRESS_ATT].getValue();
        }
        pressAmount = clamp(pressAmount, 0.0f, 1.0f);

        for (int i=0; i<6; i++){
            if (compressionAmount > 0.0f && inputCount>0.0f){ //avoid div by zero
                    mixL += inputL[i]*(1.0f*(1-pressAmount) + pressAmount/compressionAmount)*6/inputCount;
                    mixR += inputR[i]*(1.0f*(1-pressAmount) + pressAmount/compressionAmount)*6/inputCount;
            } 
        }

        // Bass processing and envelope calculation
        float bassL = inputs[SIDECHAIN_INPUT_L].getVoltage();
        float bassR = inputs[SIDECHAIN_INPUT_R].getVoltage();
        processBass(bassL, bassR, decayRate, mixL, mixR);

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

        // Apply ADAA
        float maxHeadRoom = 46.f; //exceeding this number results in strange wavefolding due to the polytanh bad fit beyond this point
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

    void processBass(float &bassL, float &bassR, float decayRate, float &mixL, float &mixR) {
        // Apply VCA control if connected
        if (inputs[VCA_SIDECHAIN_INPUT].isConnected()) {
            bassL *= clamp(inputs[VCA_SIDECHAIN_INPUT].getVoltage() / 10.f, 0.f, 2.f);
            bassR *= clamp(inputs[VCA_SIDECHAIN_INPUT].getVoltage() / 10.f, 0.f, 2.f);
        }

        // Apply volume control from the parameters
        float bassVol = params[SIDECHAIN_VOLUME_PARAM].getValue();
        bassL *= bassVol;
        bassR *= bassVol;

        if (params[MUTESIDE_PARAM].getValue() > 0.5f) {
                if (!muteLatch[6]) {
                    muteLatch[6] = true;
                    muteState[6] = !muteState[6];
                    transitionCount[6] = transitionSamples;  // Reset the transition count
                }
            } else {
                muteLatch[6] = false;
        }

        // Compute mute fade transition
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

        bassL *= fadeLevel[6];
        bassR *= fadeLevel[6];

        // Calculate the envelope for the bass signals
        bassPeakL = fmax(bassPeakL * decayRate, fabs(bassL));
        bassPeakR = fmax(bassPeakR * decayRate, fabs(bassR));
        bassEnvelope = (bassPeakL + bassPeakR) / 2.0f;
        filteredBassEnvelope = alpha * bassEnvelope + (1 - alpha) * filteredBassEnvelope;

        // Apply the envelope to the bass signals
        bassL *= filteredBassEnvelope;
        bassR *= filteredBassEnvelope;

        // Calculate ducking based on the bass envelope
        float duckAmount = params[DUCK_PARAM].getValue();
        if (inputs[DUCK_CV].isConnected()) {
            duckAmount += clamp(inputs[DUCK_CV].getVoltage() / 5.0f, 0.f, 1.f) * params[DUCK_ATT].getValue();
        }
        float duckingFactor = fmax(0.0f, 1.f - duckAmount * (filteredBassEnvelope / 5.0f));

        // Apply ducking to the main mix and add the processed bass signals
        mixL = (mixL * duckingFactor) + bassL;
        mixR = (mixR * duckingFactor) + bassR;
    }    

    void updateLights() {
        if (++cycleCount >= 2000) {
            for (int i = 0; i < 6; i++) {
                lights[VOLUME1_LIGHT + i].setBrightness(filteredEnvelope[i]);
                // Update mute lights based on the mute button state
                if (muteState[i]){
                    lights[MUTE1_LIGHT + i].setBrightness(1.0f);
                } else {
                    lights[MUTE1_LIGHT + i].setBrightness(0.0f);
                }
            }
            lights[BASS_VOLUME_LIGHT].setBrightness(bassEnvelope);
            if (muteState[6]){
                lights[MUTESIDE_LIGHT].setBrightness(1.0f);
            } else {
                lights[MUTESIDE_LIGHT].setBrightness(0.0f);
            }            
            cycleCount = 0;
        }
    }    
};

struct PressedDuckWidget : ModuleWidget {
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

        yPos += 1.25*Spacing;
        // Ducking amount knob
        addParam(createParamCentered<RoundBlackKnob>(Vec(xPos, yPos), module, PressedDuck::DUCK_PARAM));

        yPos = channelOffset.y + 4*Spacing + 92;

        // Ducking attenuator
        yPos += Spacing -8;
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
            yPos += 1.5*Spacing;
            addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PressedDuck::PAN_CV1_INPUT + i));

            // Mute
            yPos = channelOffset.y + 4*Spacing + 170;
            addParam(createParamCentered<LEDButton>           (Vec(xPos, yPos), module, PressedDuck::MUTE1_PARAM + i));
            addChild(createLightCentered<SmallLight<RedLight>>(Vec(xPos, yPos), module, PressedDuck::MUTE1_LIGHT + i));

            // Reset yPos for next channel
            yPos = channelOffset.y;
        }
            
        // Global controls for saturation and bass processing (placing these at the end of channels)
        xPos += 1.75*sliderX; // Shift to the right of the last channel
        yPos = channelOffset.y;

        yPos += 0.5*Spacing;

        // Saturation ceiling knob
        addParam(createParamCentered<RoundHugeBlackKnob>(Vec(xPos, yPos), module, PressedDuck::PRESS_PARAM));

        // Saturation ceiling attenuator
        yPos += 1.5*Spacing ;
        xPos -= .5*sliderX; // Shift to the right of the last channel
        addParam(createParamCentered<Trimpot>(Vec(xPos, yPos), module, PressedDuck::PRESS_ATT));

        // Saturation CV input
        xPos += 1.0*sliderX; // Shift to the right of the last channel
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PressedDuck::PRESS_CV_INPUT));

        xPos -= .5*sliderX; // Shift to the right of the last channel

        yPos += 2.3*Spacing;

        // FEEDBACK
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(xPos, yPos), module, PressedDuck::FEEDBACK_PARAM));

        // FEEDBACK attenuator
        yPos += 1.2*Spacing;
        xPos -= .5*sliderX; // Shift to the right of the last channel
        addParam(createParamCentered<Trimpot>(Vec(xPos, yPos), module, PressedDuck::FEEDBACK_ATT));

        // FEEDBACK CV input
        xPos += 1.0*sliderX; // Shift to the right of the last channel

        addInput(createInputCentered<ThemedPJ301MPort>(Vec(xPos, yPos), module, PressedDuck::FEEDBACK_CV));
        xPos -= .5*sliderX; // Shift to the right of the last channel

        yPos = channelOffset.y + 4.4*Spacing + 85;

        // Master Volume
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(xPos, yPos), module, PressedDuck::MASTER_VOL));

        // Master Volume attenuator
        yPos += 1.2*Spacing;
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

    }    
    
};

Model* modelPressedDuck = createModel<PressedDuck, PressedDuckWidget>("PressedDuck");