#include "rack.hpp"
#include "plugin.hpp"
#include "dsp/digital.hpp"
#include "dsp/fft.hpp"
#include <vector>
#include <complex>
#include <string>
#include <cmath>
#include <array>

using namespace rack;

std::array<std::array<float, 12>, 6> Scales = {{
    {{65.41, 69.3, 73.42, 77.78, 82.41, 87.31, 92.5, 98, 103.83, 110, 116.54, 123.47}},
    {{130.81, 138.59, 146.83, 155.56, 164.81, 174.61, 185, 196, 207.65, 220, 233.08, 246.94}},
    {{261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392, 415.3, 440, 466.16, 493.88}},
    {{523.25, 554.37, 587.33, 622.25, 659.26, 698.46, 739.99, 783.99, 830.61, 880, 932.33, 987.77}},
    {{1046.5, 1108.73, 1174.66, 1244.51, 1318.51, 1396.91, 1479.98, 1567.98, 1661.22, 1760, 1864.66, 1975.53}},
    {{2093, 2217.46, 2349.32, 2489.02, 2637.02, 2793.83, 2959.96, 3135.96, 3322.44, 3520, 3729.31, 3951.07}}
}};

std::array<std::array<std::string, 12>, 6> Names = {{
    {"C2", "C#2/Db2", "D2", "D#2/Eb2", "E2", "F2", "F#2/Gb2", "G2", "G#2/Ab2", "A2", "A#2/Bb2", "B2"},
    {"C3", "C#3/Db3", "D3", "D#3/Eb3", "E3", "F3", "F#3/Gb3", "G3", "G#3/Ab3", "A3", "A#3/Bb3", "B3"},
    {"C4", "C#4/Db4", "D4", "D#4/Eb4", "E4", "F4", "F#4/Gb4", "G4", "G#4/Ab4", "A4", "A#4/Bb4", "B4"},
    {"C5", "C#5/Db5", "D5", "D#5/Eb5", "E5", "F5", "F#5/Gb5", "G5", "G#5/Ab5", "A5", "A#5/Bb5", "B5"},
    {"C6", "C#6/Db6", "D6", "D#6/Eb6", "E6", "F6", "F#6/Gb6", "G6", "G#6/Ab6", "A6", "A#6/Bb6", "B6"},
    {"C7", "C#7/Db7", "D7", "D#7/Eb7", "E7", "F7", "F#7/Gb7", "G7", "G#7/Ab7", "A7", "A#7/Bb7", "B7"}
}};

struct FlowerPatch : Module {
    enum ParamIds {
        HUE_PARAM,
        HUE_ATT_PARAM,
        FILL_PARAM,
        FILL_ATT_PARAM,
        FLOWER_PARAM,
        FLOWER_ATT_PARAM,
        FFT_PARAM,
        FFT_ATT_PARAM,
        NUM_PARAMS
    };
    enum InputIds {
        LEFT_AUDIO_INPUT,
        RIGHT_AUDIO_INPUT,
        HUE_INPUT,
        FILL_INPUT,
        FLOWER_INPUT,
        FFT_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        NUM_OUTPUTS
    };
    enum LightIds {
        NUM_LIGHTS
    };

    // Audio buffer for visualization
    static constexpr size_t BUFFER_SIZE = 4096;
    float audioBuffer[BUFFER_SIZE] = {};
    int bufferIndex = 0;
    int phaseOffset = 0; 
    float sampleRate = 44100.f;
 
    bool isBufferFilling = false;
    int zeroCrossIndex = -1;
    float maxVal = 0.f;  // For normalizing the waveform display

    // FFT related
    dsp::RealFFT fft;  // Using RealFFT from the Rack DSP library
    float fftOutput[BUFFER_SIZE];  // Output buffer for FFT results
    float intensityValues[72] = {};
 
    float flowerColorVar1[72]={0.0f};
    float flowerColorVar2[72]={0.0f};
    float hue=0.f;
    float FFTknob = 0.0f;

    FlowerPatch() : Module(), fft(BUFFER_SIZE) {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
        configInput(LEFT_AUDIO_INPUT, "Left Audio Input");
        configInput(RIGHT_AUDIO_INPUT, "Right Audio Input");

        configParam(HUE_PARAM, -5.0, 5.0, 0.0, "Hue");
        configParam(HUE_ATT_PARAM, -1.0, 1.0, 0.0, "Hue Attenuvertor");
        configInput(HUE_INPUT, "Hue");

        configParam(FILL_PARAM, -5.0, 5.0, 0.0, "Fill");
        configParam(FILL_ATT_PARAM, -1.0, 1.0, 0.0, "Fill Attenuvertor");
        configInput(FILL_INPUT, "Fill");

        configParam(FLOWER_PARAM, -5.0, 5.0, 0.0, "Flower");
        configParam(FLOWER_ATT_PARAM, -1.0, 1.0, 0.0, "Flower Attenuvertor");
        configInput(FLOWER_INPUT, "Flower");

        configParam(FFT_PARAM, -5.0, 5.0, 1.0, "FFT Intensity");
        configParam(FFT_ATT_PARAM, -1.0, 1.0, 0.0, "FFT Attenuvertor");
        configInput(FFT_INPUT, "FFT");
     }
    
    void onSampleRateChange() override {
         sampleRate = APP->engine->getSampleRate();
    }

    void process(const ProcessArgs& args) override {
        sampleRate = args.sampleRate;
        float left = 0.0f, right = 0.0f;
        bool leftConnected = inputs[LEFT_AUDIO_INPUT].isConnected();
        bool rightConnected = inputs[RIGHT_AUDIO_INPUT].isConnected();

        if (leftConnected) {
            left = inputs[LEFT_AUDIO_INPUT].getVoltage();
        }
        if (rightConnected) {
            right = inputs[RIGHT_AUDIO_INPUT].getVoltage();
        }

        // Handle mono or stereo inputs
        if (leftConnected && rightConnected) {
            // Average both channels if both are connected
            audioBuffer[bufferIndex] = (left + right) * 0.05f; // 0.1 * 0.5 to keep the same scaling as before
        } else if (leftConnected) {
            audioBuffer[bufferIndex] = left * 0.1f;
        } else if (rightConnected) {
            audioBuffer[bufferIndex] = right * 0.1f;
        } else {
            audioBuffer[bufferIndex] = 0.0f;
        }

        FFTknob = params[FFT_PARAM].getValue()*0.2f; //scale knob to +-1

		if(inputs[FFT_INPUT].isConnected()){
			FFTknob = clamp(FFTknob + 0.1f * params[FFT_ATT_PARAM].getValue() * inputs[FFT_INPUT].getVoltage(), -1.0f, 1.1f);
		}


        
        float flowerVal = params[FLOWER_PARAM].getValue(); 
		if(inputs[FLOWER_INPUT].isConnected()){
			flowerVal = clamp(flowerVal +  params[FLOWER_ATT_PARAM].getValue() * inputs[FLOWER_INPUT].getVoltage(), -5.0f, 5.0f);
		}
     
               
        audioBuffer[bufferIndex] = ( audioBuffer[bufferIndex]-0.11f*flowerVal )/2.f;
        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

        // Trigger FFT processing
        if (bufferIndex == 0){
            fft.rfft(audioBuffer, fftOutput);
            computeIntensityValues();
        }             
    }

void computeIntensityValues() {
    float maxIntensity = 0.0f;
    float freqResolution = sampleRate / BUFFER_SIZE;

    for (int i = 0; i < 72; i++) {
        float targetFreq = Scales[i / 12][i % 12];

	// Calculate bin with a tiny nudge to targetFreq
	int calculatedBin = static_cast<int>((targetFreq * 0.99f) / freqResolution);

	// Ensure bin is never negative and compare properly with size_t
	size_t bin = calculatedBin > 0 ? static_cast<size_t>(calculatedBin) : 0;

        if ( bin > 0 && bin < BUFFER_SIZE / 2) {
            float real = fftOutput[2 * bin + 2];  // Correct index in FFT output
            float imag = fftOutput[2 * bin + 3];
            intensityValues[i] = std::sqrt(real * real + imag * imag);

            if (intensityValues[i] > maxIntensity) {
                maxIntensity = intensityValues[i];
            }
        } else {
            intensityValues[i] = 0.0f;  // Handling out of range frequencies
        }
    }

    // Normalize and apply power law scaling
    for (int i = 0; i < 72; i++) {
        intensityValues[i] /= fmax(maxIntensity, 0.001f);
        intensityValues[i] = pow(intensityValues[i], 3.0f);
    }
}
    
    float getBufferedSample(size_t index) {
        index = (bufferIndex + index) % BUFFER_SIZE;
        return audioBuffer[index];
    }
    
	void updatePhaseOffset() {
		int maxIndex = 0;
        maxVal = 0;
		// Find max peak in the first half of the buffer
		for (int i = 0; i < 4096; i++) { 
			if ((audioBuffer[i]) > maxVal) {
				maxVal = (audioBuffer[i]);
				maxIndex = i;
			}
		}

		// Initialize zeroCrossIndex to -1 indicating not found
		int zeroCrossIndex = -1;

		// Try to find zero-crossing before the maximum positive peak
		for (int i = maxIndex; i > 0; i--) {
			if (audioBuffer[i] >= 0 && audioBuffer[i - 1] < 0) {
				zeroCrossIndex = i;
				break;
			}
		}

		// If zero-crossing is not found before the peak, search after the peak
		if (zeroCrossIndex == -1) {
			for (int i = maxIndex; i < 4096 - 1; i++) {
				if (audioBuffer[i] >= 0 && audioBuffer[i + 1] < 0) {
					zeroCrossIndex = i + 1;
					break;
				}
			}
		}

		// If still not found, set to the start of the buffer
		if (zeroCrossIndex == -1) {
			zeroCrossIndex = 0;
		}

		// Set the phaseOffset to the found zero-crossing index
		phaseOffset = zeroCrossIndex;
	}   
};

NVGcolor colorFromMagnitude(FlowerPatch* module, float magnitude) {
    magnitude = clamp(magnitude, 0.0f, 1.0f); // Ensure magnitude is between 0 and 1
 
    float hue1 = (module->params[FlowerPatch::HUE_PARAM].getValue() + 5.0f) / 10.0f; // Map -5 to 5 to 0 to 1
    if(module->inputs[FlowerPatch::HUE_INPUT].isConnected()){
        hue1 = clamp(hue1 + 0.1f * module->params[FlowerPatch::HUE_ATT_PARAM].getValue() * module->inputs[FlowerPatch::HUE_INPUT].getVoltage(), -0.1f, 1.1f);
	}

    float fillKnob = (module->params[FlowerPatch::FILL_PARAM].getValue() + 4.9f)/ 9.9f; // Map -5 to 5 to -.01 to 1 - then scale non-linearly    
    if(module->inputs[FlowerPatch::FILL_INPUT].isConnected()){
        fillKnob = clamp(fillKnob + 0.1f * module->params[FlowerPatch::FILL_ATT_PARAM].getValue() * module->inputs[FlowerPatch::FILL_INPUT].getVoltage(), -0.1f, 1.1f);
	}
	
	fillKnob = pow(fillKnob , 0.001f); //extreme non-linear scaling for the fill, so it has at least a little knob range

    float hue2 = hue1 + 0.15f; // Define second hue point

    // Set configurable transition points
    float lowPoint = 1.0f-fillKnob; // Lower transition point for hue changes
    float highPoint = 1.0f-fillKnob/2; // Upper transition point for hue changes

    if (magnitude < lowPoint) {
        // Transition from White (0% saturation, 100% lightness) to hue1
        float blend = magnitude / lowPoint;
        return nvgHSLA(hue1, blend, 1.0f - 0.5f * blend, 255);
    } else if (magnitude < highPoint) {
        // Transition from hue1 to hue2
        float blend = (magnitude - lowPoint) / (highPoint - lowPoint);
        return nvgHSLA(hue1 + (hue2 - hue1) * blend, 1.0f, 0.5f + 0.5f * blend, 255);
    } else {
        // Keep at hue2 for any magnitude above highPoint
        return nvgHSLA(hue2, 1.0f, 0.75f, 255);
    }
}

struct FlowerDisplay : TransparentWidget {
    FlowerPatch* module;

    int updateCounter = 0;  // Counter to track update intervals
    int updateRate = 1;  // Number of frames to wait between updates

    void draw(const DrawArgs& args) override {
        if (!module) return;
                
        const float padding = 20.0f;
        const float totalWidth = box.size.x - 2 * padding;
        const float totalHeight = box.size.y - 2 * padding;

        const float spaceX = totalWidth / 12;
        const float spaceY = totalHeight / 6;
        const float twoPi = 2 * M_PI ;
 
        // Update the phase offset at a controlled rate to minimize CPU load
        if (++updateCounter >= updateRate) {
            updateCounter = 0;
            module->updatePhaseOffset();
        }
		int phaseOffset = module->phaseOffset; // Use the updated phase offset

        for (size_t scale = 0; scale < 6; scale++) {
            for (size_t note = 0; note < 12; note++) {
                float centerX = padding + spaceX * note + spaceX / 2;
                float centerY = padding + spaceY * scale + spaceY / 2;
                float maxRadius = std::min(spaceX, spaceY) * 0.6f;
                float freq = Scales[scale][note];
			    int flowerIndex = (scale*12)+note;
			    
			    float drawSkip = 0; //for skipping the drawing of some dots to save CPU

                // Draw waveform based on phase
				size_t lastSample = static_cast<size_t>(2 * (module->sampleRate / freq));
                for (size_t i = 0; i < lastSample; i++) {
                    float sample = module->getBufferedSample( (i+phaseOffset)%lastSample );
                    float angle = twoPi * (i / (module->sampleRate/freq) );
                    float radius = maxRadius * (0.5f + 0.5f * sample*(0.5f/(fmax(module->maxVal,0.15f))) );

					float FFTintensity = 0.f;
					float FFTknob = module->FFTknob;
					if (FFTknob >0){
						FFTintensity = (1.f-module->FFTknob) + module->FFTknob * module->intensityValues[flowerIndex];    
					} else {
						FFTintensity = (1.f+module->FFTknob) - module->FFTknob * (1-module->intensityValues[flowerIndex]);    
					}

                    float posX = centerX + 1.1f * radius * fast_sin(angle+M_PI_2) * FFTintensity;
                    float posY = centerY + 1.1f * radius * fast_sin(angle) * FFTintensity;
                                       
                    bool drawDots = false;
					NVGcolor color = colorFromMagnitude(module, module->intensityValues[flowerIndex]);
					drawSkip++;
                    if(lastSample >= 1920){
                        if(drawSkip > 3){
                            drawSkip = 0; 
                            drawDots = true;
                        }
                    } else if (lastSample >= 640){
                        if(drawSkip > 2){
                            drawSkip = 0;
                            drawDots = true;
                        }
                    }  else {
                       drawSkip = 0;
                       drawDots = true;
                    }

                    if (drawDots){
                        nvgBeginPath(args.vg);
                        nvgCircle(args.vg, posX, posY, .2f*(scale/2+1)); // Small dots for clarity
                        nvgFillColor(args.vg, color);  // Use calculated color for each dot
                        nvgFill(args.vg);
                    }
                }                 
            }
        }
    }

	double fast_sin(double x) {  //intriguing fast_sin approximation I found on the web in a random place
		int k;
		double y;
		double z;

		z = x;
		z *= 0.3183098861837907;
		z += 6755399441055744.0;
		k = *((int *) &z);
		z = k;
		z *= 3.1415926535897932;
		x -= z;
		y = x;
		y *= x;
		z = 0.0073524681968701;
		z *= y;
		z -= 0.1652891139701474;
		z *= y;
		z += 0.9996919862959676;
		x *= z;
		k &= 1;
		k += k;
		z = k;
		z *= x;
		x -= z;
		
		return x;
	}    
	
};

struct FlowerPatchWidget : ModuleWidget {
    FlowerPatchWidget(FlowerPatch* module) {
        setModule(module);

        setPanel(createPanel(
                asset::plugin(pluginInstance, "res/FlowerPatch.svg"),
                asset::plugin(pluginInstance, "res/FlowerPatch-dark.svg")
            ));

        // Add screws 
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        float spacing = 2*5.08;
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(spacing, 112.373)), module, FlowerPatch::LEFT_AUDIO_INPUT));
        spacing += 2*5.08;
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(spacing, 112.373)), module, FlowerPatch::RIGHT_AUDIO_INPUT));
        spacing += 3*5.08;

        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(spacing, 112.373)), module, FlowerPatch::HUE_PARAM));
        spacing += 1.75*5.08;
        addParam(createParamCentered<Trimpot>(mm2px(Vec(spacing, 112.373)), module, FlowerPatch::HUE_ATT_PARAM));
        spacing += 1.5*5.08;
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(spacing, 112.373)), module, FlowerPatch::HUE_INPUT));
        spacing += 2.6*5.08;

        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(spacing, 112.373)), module, FlowerPatch::FILL_PARAM));
        spacing += 1.75*5.08;
        addParam(createParamCentered<Trimpot>(mm2px(Vec(spacing, 112.373)), module, FlowerPatch::FILL_ATT_PARAM));
        spacing += 1.5*5.08;
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(spacing, 112.373)), module, FlowerPatch::FILL_INPUT));
        spacing += 2.6*5.08;

        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(spacing, 112.373)), module, FlowerPatch::FLOWER_PARAM));
        spacing += 1.75*5.08;
        addParam(createParamCentered<Trimpot>(mm2px(Vec(spacing, 112.373)), module, FlowerPatch::FLOWER_ATT_PARAM));
        spacing += 1.5*5.08;
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(spacing, 112.373)), module, FlowerPatch::FLOWER_INPUT));
        spacing += 2.6*5.08;

        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(spacing, 112.373)), module, FlowerPatch::FFT_PARAM));
        spacing += 1.75*5.08;
        addParam(createParamCentered<Trimpot>(mm2px(Vec(spacing, 112.373)), module, FlowerPatch::FFT_ATT_PARAM));
        spacing += 1.5*5.08;
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(spacing, 112.373)), module, FlowerPatch::FFT_INPUT));

        FlowerDisplay* display = new FlowerDisplay();
        display->box.pos = Vec(5, 25);
        display->box.size = Vec(box.size.x, 300);
        display->module = module;
        addChild(display);
    }
};

Model* modelFlowerPatch = createModel<FlowerPatch, FlowerPatchWidget>("FlowerPatch");