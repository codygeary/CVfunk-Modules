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

struct PressedDuck : Module {
    enum ParamIds {
        VOLUME1_PARAM, VOLUME2_PARAM, VOLUME3_PARAM, VOLUME4_PARAM, VOLUME5_PARAM, VOLUME6_PARAM,   
        PAN1_PARAM, PAN2_PARAM, PAN3_PARAM, PAN4_PARAM, PAN5_PARAM, PAN6_PARAM,      
        BASS_VOLUME_PARAM, DUCK_PARAM, DUCK_ATT,
        PRESS_PARAM, PRESS_ATT, MASTER_VOL, MASTER_VOL_ATT, FEEDBACK_PARAM, FEEDBACK_ATT, 
        NUM_PARAMS
    };
    enum InputIds {
        AUDIO_1L_INPUT, AUDIO_1R_INPUT, AUDIO_2L_INPUT, AUDIO_2R_INPUT, 
        AUDIO_3L_INPUT, AUDIO_3R_INPUT, AUDIO_4L_INPUT, AUDIO_4R_INPUT, 
        AUDIO_5L_INPUT, AUDIO_5R_INPUT, AUDIO_6L_INPUT, AUDIO_6R_INPUT,   
        VCA_CV1_INPUT, VCA_CV2_INPUT, VCA_CV3_INPUT, VCA_CV4_INPUT, VCA_CV5_INPUT, VCA_CV6_INPUT, VCA_BASS_INPUT,
        PAN_CV1_INPUT, PAN_CV2_INPUT, PAN_CV3_INPUT, PAN_CV4_INPUT, PAN_CV5_INPUT, PAN_CV6_INPUT,  
        BASS_AUDIO_INPUT_L, BASS_AUDIO_INPUT_R, DUCK_CV, PRESS_CV_INPUT, FEEDBACK_CV, MASTER_VOL_CV,
        NUM_INPUTS
    };
    enum OutputIds {
        AUDIO_OUTPUT_L, AUDIO_OUTPUT_R, SURVEY,
        NUM_OUTPUTS
    };
    enum LightIds {
        VOLUME1_LIGHT, VOLUME2_LIGHT, VOLUME3_LIGHT, VOLUME4_LIGHT, VOLUME5_LIGHT, VOLUME6_LIGHT, BASS_VOLUME_LIGHT, 
        NUM_LIGHTS
    };

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
    float filteredEnvelope[6] = {0.0f}; 

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
        configParam(BASS_VOLUME_PARAM, 0.f, 2.f, 0.6f, "Bass Volume");
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
        configInput(VCA_BASS_INPUT, "Bass CV");

        configInput(PAN_CV1_INPUT, "Channel 1 Pan CV");
        configInput(PAN_CV2_INPUT, "Channel 2 Pan CV");
        configInput(PAN_CV3_INPUT, "Channel 3 Pan CV");
        configInput(PAN_CV4_INPUT, "Channel 4 Pan CV");
        configInput(PAN_CV5_INPUT, "Channel 5 Pan CV");
        configInput(PAN_CV6_INPUT, "Channel 6 Pan CV");

        // Bass and saturation CV inputs
        configInput(BASS_AUDIO_INPUT_L, "Bass Audio L");
        configInput(BASS_AUDIO_INPUT_R, "Bass Audio R");
        configInput(DUCK_CV, "Duck CV");
        configInput(PRESS_CV_INPUT, "Press CV");
        configInput(FEEDBACK_CV, "Feedback CV");
        configInput(MASTER_VOL_CV, "Master Volume CV");

        // Outputs
        
        //configOutput(SURVEY, "Survey Test Out"); //hidden output for testing

        configOutput(AUDIO_OUTPUT_L, "Main Out L");
        configOutput(AUDIO_OUTPUT_R, "Main Out R");    }

    void process(const ProcessArgs& args) override {
        float mixL = 0.0f;
        float mixR = 0.0f;
        float sampleRate = args.sampleRate;
        float alpha = 0.01f;  // Smoothing factor for envelope

        // State variables for bass envelope tracking
        float decayRate = 0.999f;
        float compressionAmount = 0.0f;

        // Process each of the six main channels
        for (int i = 0; i < 6; i++) {

            inputL[i]=0.0f;
            inputR[i]=0.0f;
            // Check left input connection
            if (inputs[AUDIO_1L_INPUT + 2 * i].isConnected()) {
                inputL[i] = inputs[AUDIO_1L_INPUT + 2 * i].getVoltage();  // Use left input if connected
                inputR[i] = inputL[i];  // Copy left input to right if right is not connected
            }

            // Check right input connection
            if (inputs[AUDIO_1R_INPUT + 2 * i].isConnected()) {
                inputR[i] = inputs[AUDIO_1R_INPUT + 2 * i].getVoltage();  // Use right input if connected
                if (!inputs[AUDIO_1L_INPUT + 2 * i].isConnected()) {
                    inputL[i] = inputR[i];  // Copy right input to left if left is not connected
                }
            }            
            
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
            filteredEnvelope[i] = alpha * envelope[i] + (1 - alpha) * filteredEnvelope[i];

            compressionAmount += filteredEnvelope[i];

            // Apply panning
            float pan = params[PAN1_PARAM + i].getValue();
            if (inputs[PAN_CV1_INPUT + i].isConnected()) {
                pan += inputs[PAN_CV1_INPUT + i].getVoltage() / 5.f; // Scale CV influence
            }
            pan = clamp(pan, -1.f, 1.f);
            float panL = cosf(M_PI_4 * (pan + 1.f));
            float panR = sinf(M_PI_4 * (pan + 1.f));

            // Mix processed signals into left and right outputs
            inputL[i] = inputL[i] * panL;
            inputR[i] = inputR[i] * panR;
        }

        compressionAmount = compressionAmount/30.0f; //divide by the expected ceiling.

        float pressAmount = params[PRESS_PARAM].getValue();
        if(inputs[PRESS_CV_INPUT].isConnected()){
            pressAmount += inputs[PRESS_CV_INPUT].getVoltage()*params[PRESS_ATT].getValue();
        }
        pressAmount = clamp(pressAmount, 0.0f, 1.0f);

        for (int i=0; i<6; i++){
            if (compressionAmount > 0.0f){ //avoid div by zero
                    mixL += inputL[i]*(1.0f*(1-pressAmount) + pressAmount/compressionAmount);
                    mixR += inputR[i]*(1.0f*(1-pressAmount) + pressAmount/compressionAmount);
            } 
        }

        // Bass processing and envelope calculation
        float bassL = inputs[BASS_AUDIO_INPUT_L].getVoltage();
        float bassR = inputs[BASS_AUDIO_INPUT_R].getVoltage();
        processBass(bassL, bassR, decayRate, mixL, mixR);


        float feedbackSetting = params[FEEDBACK_PARAM].getValue();
        if(inputs[FEEDBACK_CV].isConnected()){
            feedbackSetting += inputs[FEEDBACK_CV].getVoltage()*params[FEEDBACK_ATT].getValue();
        }
        feedbackSetting = clamp(feedbackSetting, 0.0f, 11.0f);

        float saturationEffect = 1 + feedbackSetting;
        mixL *= saturationEffect;
        mixR *= saturationEffect;
        
        // Apply ADAA
        mixL = clamp(mixL, -40.f, 40.f);
        mixR = clamp(mixR, -40.f, 40.f);
        mixL = applyADAA(mixL/30.f, lastOutputL, sampleRate);
        mixR = applyADAA(mixR/30.f, lastOutputR, sampleRate);
        lastOutputL = mixL;
        lastOutputR = mixR;

        // Set outputs
		float masterVol = params[MASTER_VOL].getValue();
		if (inputs[MASTER_VOL_CV].isConnected()){
			masterVol += inputs[MASTER_VOL_CV].getVoltage()*params[MASTER_VOL_ATT].getValue()/10.f;
		}
		masterVol = clamp(masterVol, 0.0f, 2.0f);

        // Processing the outputs
        float outputL = mixL * 10.f * masterVol;
        float outputR = mixR * 10.f * masterVol;

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
            return tanh(input);
        }
    }

    float antiderivative(float x) {
        float x2 = x * x;
        float x4 = x2 * x2;
        float x6 = x4 * x2;
        float x8 = x4 * x4;
        return x2 / 2.0f - x4 / 12.0f + x6 / 45.0f - 17.0f * x8 / 2520.0f;
    }
    
    void processBass(float &bassL, float &bassR, float decayRate, float &mixL, float &mixR) {
        // Apply VCA control if connected
        if (inputs[VCA_BASS_INPUT].isConnected()) {
            bassL *= clamp(inputs[VCA_BASS_INPUT].getVoltage() / 10.f, 0.f, 2.f);
            bassR *= clamp(inputs[VCA_BASS_INPUT].getVoltage() / 10.f, 0.f, 2.f);
        }

        // Apply volume control from the parameters
        float bassVol = params[BASS_VOLUME_PARAM].getValue();
        bassL *= bassVol;
        bassR *= bassVol;

        // Calculate the envelope for the bass signals
        bassPeakL = fmax(bassPeakL * decayRate, fabs(bassL));
        bassPeakR = fmax(bassPeakR * decayRate, fabs(bassR));
        bassEnvelope = (bassPeakL + bassPeakR) / 2.0f;

        // Apply the envelope to the bass signals
        bassL *= bassEnvelope;
        bassR *= bassEnvelope;

        // Calculate ducking based on the bass envelope
        float duckAmount = params[DUCK_PARAM].getValue();
        if (inputs[DUCK_CV].isConnected()) {
            duckAmount += clamp(inputs[DUCK_CV].getVoltage() / 5.0f, 0.f, 1.f) * params[DUCK_ATT].getValue();
        }
        float duckingFactor = fmax(0.0f, 1.f - duckAmount * (bassEnvelope / 5.0f));

        // Apply ducking to the main mix and add the processed bass signals
        mixL = (mixL * duckingFactor) + bassL;
        mixR = (mixR * duckingFactor) + bassR;
    }    

    void updateLights() {
        if (++cycleCount >= 1000) {
            for (int i = 0; i < 6; i++) {
                lights[VOLUME1_LIGHT + i].setBrightness(filteredEnvelope[i]);
            }
            lights[BASS_VOLUME_LIGHT].setBrightness(bassEnvelope);
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

         // Audio inputs
        addInput(createInputCentered<PJ301MPort>(Vec(xPos, yPos), module, PressedDuck::BASS_AUDIO_INPUT_L ));
        yPos += Spacing;
        addInput(createInputCentered<PJ301MPort>(Vec(xPos, yPos), module, PressedDuck::BASS_AUDIO_INPUT_R ));
   
        // Volume slider with light
        yPos += 40+Spacing;
        addParam(createLightParamCentered<VCVLightSlider<YellowLight>>(Vec(xPos, yPos), module, PressedDuck::BASS_VOLUME_PARAM , PressedDuck::BASS_VOLUME_LIGHT));

        // VCA CV input
        yPos += 38+Spacing;
        addInput(createInputCentered<PJ301MPort>(Vec(xPos, yPos), module, PressedDuck::VCA_BASS_INPUT ));

        yPos += 1.95*Spacing;
        // Ducking amount knob
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(xPos, yPos), module, PressedDuck::DUCK_PARAM));

        yPos = channelOffset.y + 4*Spacing + 120;

        // Ducking attenuator
        yPos += Spacing -8;
        addParam(createParamCentered<Trimpot>(Vec(xPos, yPos), module, PressedDuck::DUCK_ATT));

        // Ducking CV input
        yPos += Spacing;
        addInput(createInputCentered<PJ301MPort>(Vec(xPos, yPos), module, PressedDuck::DUCK_CV));

        yPos = channelOffset.y;
        // Loop through each channel
        for (int i = 0; i < 6; i++) {
            xPos = 1.25*sliderX + channelOffset.x + i * sliderX;

            // Audio inputs
            addInput(createInputCentered<PJ301MPort>(Vec(xPos, yPos), module, PressedDuck::AUDIO_1L_INPUT + 2 * i));
            yPos += Spacing;
            addInput(createInputCentered<PJ301MPort>(Vec(xPos, yPos), module, PressedDuck::AUDIO_1R_INPUT + 2 * i));

            // Volume slider with light
            yPos += 40+Spacing;
            addParam(createLightParamCentered<VCVLightSlider<YellowLight>>(Vec(xPos, yPos), module, PressedDuck::VOLUME1_PARAM + i, PressedDuck::VOLUME1_LIGHT+ i));

            // VCA CV input
            yPos += 38+Spacing;
            addInput(createInputCentered<PJ301MPort>(Vec(xPos, yPos), module, PressedDuck::VCA_CV1_INPUT + i));

            // Pan knob
            yPos += Spacing + 40;
            addParam(createParamCentered<RoundBlackKnob>(Vec(xPos, yPos), module, PressedDuck::PAN1_PARAM + i));

            // Pan CV input
            yPos += 1.5*Spacing;
            addInput(createInputCentered<PJ301MPort>(Vec(xPos, yPos), module, PressedDuck::PAN_CV1_INPUT + i));

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
        addInput(createInputCentered<PJ301MPort>(Vec(xPos, yPos), module, PressedDuck::PRESS_CV_INPUT));

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

        addInput(createInputCentered<PJ301MPort>(Vec(xPos, yPos), module, PressedDuck::FEEDBACK_CV));
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

        addInput(createInputCentered<PJ301MPort>(Vec(xPos, yPos), module, PressedDuck::MASTER_VOL_CV));
        xPos -= .5*sliderX; // Shift to the right of the last channel

        xPos -= .5*sliderX; // Shift to the right of the last channel

        // Outputs
        yPos = channelOffset.y + 4*Spacing + 170;
        addOutput(createOutputCentered<PJ301MPort>(Vec(xPos, yPos), module, PressedDuck::AUDIO_OUTPUT_L));
        xPos += 1*sliderX; // Shift to the right of the last channel
        addOutput(createOutputCentered<PJ301MPort>(Vec(xPos, yPos), module, PressedDuck::AUDIO_OUTPUT_R));

        xPos -= 2*sliderX; // Shift to the right of the last channel
        // addOutput(createOutputCentered<PJ301MPort>(Vec(xPos, yPos), module, PressedDuck::SURVEY));

    }};

Model* modelPressedDuck = createModel<PressedDuck, PressedDuckWidget>("PressedDuck");
