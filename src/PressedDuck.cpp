////////////////////////////////////////////////////////////
//
//   Dynamic Mixer
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   A stereo mixer with dynamic mixing and ducking
//
////////////////////////////////////////////////////////////

#include "plugin.hpp"

struct PressedDuck : Module {
    enum ParamIds {
        VOLUME1_PARAM, VOLUME2_PARAM, VOLUME3_PARAM, VOLUME4_PARAM, VOLUME5_PARAM, VOLUME6_PARAM,   
        PAN1_PARAM, PAN2_PARAM, PAN3_PARAM, PAN4_PARAM, PAN5_PARAM, PAN6_PARAM,      
        BASS_VOLUME_PARAM, BASS_DUCK_AMOUNT_PARAM, BASS_DUCK_AMOUNT_ATT,
        SATURATION_CEILING_PARAM, SATURATION_CEILING_ATT, MASTER_VOL, MASTER_VOL_ATT, FEEDBACK_PARAM, FEEDBACK_ATT, 
        NUM_PARAMS
    };
    enum InputIds {
        AUDIO_1L_INPUT, AUDIO_1R_INPUT, AUDIO_2L_INPUT, AUDIO_2R_INPUT, 
        AUDIO_3L_INPUT, AUDIO_3R_INPUT, AUDIO_4L_INPUT, AUDIO_4R_INPUT, 
        AUDIO_5L_INPUT, AUDIO_5R_INPUT, AUDIO_6L_INPUT, AUDIO_6R_INPUT,   
        VCA_CV1_INPUT, VCA_CV2_INPUT, VCA_CV3_INPUT, VCA_CV4_INPUT, VCA_CV5_INPUT, VCA_CV6_INPUT, VCA_BASS_INPUT,
        PAN_CV1_INPUT, PAN_CV2_INPUT, PAN_CV3_INPUT, PAN_CV4_INPUT, PAN_CV5_INPUT, PAN_CV6_INPUT,  
        BASS_AUDIO_INPUT_L, BASS_AUDIO_INPUT_R, BASS_DUCK_CV_INPUT, SATURATION_CV_INPUT, FEEDBACK_CV, MASTER_VOL_CV,
        NUM_INPUTS
    };
    enum OutputIds {
        AUDIO_OUTPUT_L, AUDIO_OUTPUT_R, 
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
        configParam(FEEDBACK_PARAM, 0.f, 11.f, 0.0f, "FEEDBACK");


        configParam(PAN1_PARAM, -1.f, 1.f, 0.f, "Channel 1 Pan");
        configParam(PAN2_PARAM, -1.f, 1.f, 0.f, "Channel 2 Pan");
        configParam(PAN3_PARAM, -1.f, 1.f, 0.f, "Channel 3 Pan");
        configParam(PAN4_PARAM, -1.f, 1.f, 0.f, "Channel 4 Pan");
        configParam(PAN5_PARAM, -1.f, 1.f, 0.f, "Channel 5 Pan");
        configParam(PAN6_PARAM, -1.f, 1.f, 0.f, "Channel 6 Pan");

        // Configure bass and saturation parameters
        configParam(BASS_VOLUME_PARAM, 0.f, 2.f, 0.6f, "Bass Volume");
        configParam(BASS_DUCK_AMOUNT_PARAM, 0.f, 1.f, 0.7f, "Bass Duck Amount");
        configParam(BASS_DUCK_AMOUNT_ATT, -1.f, 1.f, 0.0f, "Bass Duck Attenuation");
        configParam(FEEDBACK_ATT, -1.f, 1.f, 0.0f, "Feedback Attenuation");
        configParam(MASTER_VOL_ATT, -1.f, 1.f, 0.0f, "Master Volume Attenuation");

        configParam(SATURATION_CEILING_PARAM, 0.f, 10.f, 5.f, "Saturation");
        configParam(SATURATION_CEILING_ATT, -1.f, 1.f, 0.0f, "Saturation Attenuation");

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
        configInput(BASS_DUCK_CV_INPUT, "Bass Duck CV");
        configInput(SATURATION_CV_INPUT, "Saturation CV");
        configInput(FEEDBACK_CV, "FEEDBACK CV");
        configInput(MASTER_VOL_CV, "Master Volume CV");

        // Outputs
        configOutput(AUDIO_OUTPUT_L, "Main Out L");
        configOutput(AUDIO_OUTPUT_R, "Main Out R");    }

    void process(const ProcessArgs& args) override {
		float mixL = 0.0f;
		float mixR = 0.0f;

		// State variables for bass envelope tracking
		float decayRate = 0.999f; // Controls the rate at which the peak decays

		// Process each of the six main channels
		for (int i = 0; i < 6; i++) {
			float inputL = inputs[AUDIO_1L_INPUT + 2 * i].getVoltage();
			float inputR = inputs[AUDIO_1R_INPUT + 2 * i].getVoltage();

			// Apply VCA control and volume
			if (inputs[VCA_CV1_INPUT + i].isConnected()) {
				inputL *= clamp(inputs[VCA_CV1_INPUT + i].getVoltage() / 10.f, 0.f, 2.f);
				inputR *= clamp(inputs[VCA_CV1_INPUT + i].getVoltage() / 10.f, 0.f, 2.f);
			}

			float vol = params[VOLUME1_PARAM + i].getValue();
			inputL *= vol;
			inputR *= vol;
			
			// Simple peak detection using the absolute maximum of the current input
    		envPeakL[i] = fmax(envPeakL[i] * decayRate, fabs(inputL));
	     	envPeakR[i] = fmax(envPeakR[i] * decayRate, fabs(inputR));
    		envelope[i] = (envPeakL[i] + envPeakR[i]) / 2.0f;
			
			inputL *= envelope[i];
			inputR *= envelope[i];
			
			// Apply panning
			float pan = params[PAN1_PARAM + i].getValue();
			if (inputs[PAN_CV1_INPUT + i].isConnected()) {
				pan += inputs[PAN_CV1_INPUT + i].getVoltage() / 5.f; // Scale CV influence
			}
			pan = clamp(pan, -1.f, 1.f);
			float panL = cosf(M_PI_4 * (pan + 1.f));
			float panR = sinf(M_PI_4 * (pan + 1.f));

			// Mix processed signals into left and right outputs
			mixL += inputL * panL;
			mixR += inputR * panR;
		}

		// Bass processing and envelope calculation
		float bassL = inputs[BASS_AUDIO_INPUT_L].getVoltage();
		float bassR = inputs[BASS_AUDIO_INPUT_R].getVoltage();
		if (inputs[VCA_BASS_INPUT].isConnected()) {
			bassL *= clamp(inputs[VCA_BASS_INPUT].getVoltage() / 10.f, 0.f, 2.f);
			bassR *= clamp(inputs[VCA_BASS_INPUT].getVoltage() / 10.f, 0.f, 2.f);
		}

		float bassVol = params[BASS_VOLUME_PARAM].getValue();
		bassL *= bassVol;
		bassR *= bassVol;

		bassPeakL = fmax(bassPeakL * decayRate, fabs(bassL));
		bassPeakR = fmax(bassPeakR * decayRate, fabs(bassR));
		float bassEnvelope = (bassPeakL + bassPeakR) / 2.0f;

        bassL *= bassEnvelope;
        bassR *= bassEnvelope;

		// Ducking based on bass
		float duckAmount = params[BASS_DUCK_AMOUNT_PARAM].getValue();
		if (inputs[BASS_DUCK_CV_INPUT].isConnected()) {
			duckAmount += clamp(inputs[BASS_DUCK_CV_INPUT].getVoltage() / 10.f, 0.f, 1.f) * params[BASS_DUCK_AMOUNT_ATT].getValue();
		}
		float duckingFactor = fmax( 0.0f, 1.f - duckAmount * (bassEnvelope / 5.0f));

		// Apply ducking to the main mix
		mixL = (mixL * duckingFactor ) + bassL;
		mixR = (mixR * duckingFactor ) + bassR;

		float feedback = params[FEEDBACK_PARAM].getValue();
		if (inputs[FEEDBACK_CV].isConnected() ){
			feedback += inputs[FEEDBACK_CV].getVoltage()*params[FEEDBACK_ATT].getValue();
		}
		feedback = pow(feedback, 0.5f);
		feedback = clamp(feedback,0.0f, 11.0f) + 1;

		mixL *= feedback;
		mixR *= feedback;

		// Apply saturation
		float satKnob = params[SATURATION_CEILING_PARAM].getValue();
		if (inputs[SATURATION_CV_INPUT].isConnected()) {
			satKnob += inputs[SATURATION_CV_INPUT].getVoltage() * params[SATURATION_CEILING_ATT].getValue();
		}
		satKnob = clamp (satKnob, 0.01f, 10.0f); //clamp to reasonable values
		float saturationCeiling = 3.f * (10.5f - satKnob); //reverse the knob and make it to 30, since 6x5 channels =30v headroom		
		saturationCeiling = fmax(0.01f, saturationCeiling); // Avoid division by zero
		float masterVol = params[MASTER_VOL].getValue();
		mixL =  (6.f-satKnob/2.f) * tanh(mixL / saturationCeiling);
		mixR =  (6.f-satKnob/2.f) * tanh(mixR / saturationCeiling);

		// Update lights every 1000 process cycles
		if (++cycleCount >= 1000) {
			for (int i = 0; i < 6; i++) {
				lights[VOLUME1_LIGHT + i].setBrightness(envelope[i]);
			}
			lights[BASS_VOLUME_LIGHT].setBrightness(bassEnvelope);
			cycleCount = 0; // Reset the cycle counter
		}

		// Set the outputs
		outputs[AUDIO_OUTPUT_L].setVoltage(mixL*masterVol);
		outputs[AUDIO_OUTPUT_R].setVoltage(mixR*masterVol);
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
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(xPos, yPos), module, PressedDuck::BASS_DUCK_AMOUNT_PARAM));

        yPos = channelOffset.y + 4*Spacing + 120;

        // Ducking attenuator
        yPos += Spacing -8;
        addParam(createParamCentered<Trimpot>(Vec(xPos, yPos), module, PressedDuck::BASS_DUCK_AMOUNT_ATT));

        // Ducking CV input
        yPos += Spacing;
        addInput(createInputCentered<PJ301MPort>(Vec(xPos, yPos), module, PressedDuck::BASS_DUCK_CV_INPUT));

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
        addParam(createParamCentered<RoundHugeBlackKnob>(Vec(xPos, yPos), module, PressedDuck::SATURATION_CEILING_PARAM));

        // Saturation ceiling attenuator
        yPos += 1.5*Spacing ;
        xPos -= .5*sliderX; // Shift to the right of the last channel
        addParam(createParamCentered<Trimpot>(Vec(xPos, yPos), module, PressedDuck::SATURATION_CEILING_ATT));

        // Saturation CV input
        xPos += 1.0*sliderX; // Shift to the right of the last channel
        addInput(createInputCentered<PJ301MPort>(Vec(xPos, yPos), module, PressedDuck::SATURATION_CV_INPUT));

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
    }};

Model* modelPressedDuck = createModel<PressedDuck, PressedDuckWidget>("PressedDuck");
