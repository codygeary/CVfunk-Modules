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
    {{65.41f, 69.3f, 73.42f, 77.78f, 82.41f, 87.31f, 92.5f, 98.0f, 103.83f, 110.0f, 116.54f, 123.47f}},
    {{130.81f, 138.59f, 146.83f, 155.56f, 164.81f, 174.61f, 185.0f, 196.0f, 207.65f, 220.0f, 233.08f, 246.94f}},
    {{261.63f, 277.18f, 293.66f, 311.13f, 329.63f, 349.23f, 369.99f, 392.0f, 415.3f, 440.0f, 466.16f, 493.88f}},
    {{523.25f, 554.37f, 587.33f, 622.25f, 659.26f, 698.46f, 739.99f, 783.99f, 830.61f, 880.0f, 932.33f, 987.77f}},
    {{1046.5f, 1108.73f, 1174.66f, 1244.51f, 1318.51f, 1396.91f, 1479.98f, 1567.98f, 1661.22f, 1760.0f, 1864.66f, 1975.53f}},
    {{2093.0f, 2217.46f, 2349.32f, 2489.02f, 2637.02f, 2793.83f, 2959.96f, 3135.96f, 3322.44f, 3520.0f, 3729.31f, 3951.07f}}
}};

std::array<std::array<std::string, 12>, 6> Names = {{
    {"C2", "C#2/Db2", "D2", "D#2/Eb2", "E2", "F2", "F#2/Gb2", "G2", "G#2/Ab2", "A2", "A#2/Bb2", "B2"},
    {"C3", "C#3/Db3", "D3", "D#3/Eb3", "E3", "F3", "F#3/Gb3", "G3", "G#3/Ab3", "A3", "A#3/Bb3", "B3"},
    {"C4", "C#4/Db4", "D4", "D#4/Eb4", "E4", "F4", "F#4/Gb4", "G4", "G#4/Ab4", "A4", "A#4/Bb4", "B4"},
    {"C5", "C#5/Db5", "D5", "D#5/Eb5", "E5", "F5", "F#5/Gb5", "G5", "G#5/Ab5", "A5", "A#5/Bb5", "B5"},
    {"C6", "C#6/Db6", "D6", "D#6/Eb6", "E6", "F6", "F#6/Gb6", "G6", "G#6/Ab6", "A6", "A#6/Bb6", "B6"},
    {"C7", "C#7/Db7", "D7", "D#7/Eb7", "E7", "F7", "F#7/Gb7", "G7", "G#7/Ab7", "A7", "A#7/Bb7", "B7"}
}};

#include <cstddef> // for std::size_t
bool isAligned(void* ptr, std::size_t alignment) {
    return reinterpret_cast<uintptr_t>(ptr) % alignment == 0;
}

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

    static constexpr size_t BUFFER_SIZE = 4096;

    // Wave buffer for visualization
    float waveBuffer[BUFFER_SIZE] = {0.0f}; //to store the waveform shape 
    
    // FFT related buffers
    dsp::RealFFT fft;  // Using RealFFT from the Rack DSP library
    float* fftOutput; // This will point to the aligned buffer
    float* audioBuffer;  //dynamic for FFT analysis

    int bufferIndex = 0;
    int waveIndex = 0;
    int phaseOffset = 0; 
    float sampleRate = 44100.f; //will update in process
 
    bool isBufferFilling = false;
    int zeroCrossIndex = -1;
    float maxVal = 0.f;  // For normalizing the waveform display

    float intensityValues[72] = {};
    float flowerColorVar1[72]={0.0f};
    float flowerColorVar2[72]={0.0f};
    float hue=0.f;
    float FFTknob = 0.0f;
    float flowerVal = 0.0f;
    bool inputConnected = false;

    FlowerPatch() : Module(), fft(BUFFER_SIZE) {

        //special aligned memory allocation for FFT using pretty-fast-FFT-aligned-malloc
        audioBuffer = static_cast<float*>(pffft_aligned_malloc(BUFFER_SIZE * sizeof(float)));
        fftOutput   = static_cast<float*>(pffft_aligned_malloc(BUFFER_SIZE * sizeof(float)));
        
        if (!audioBuffer || !fftOutput || !isAligned(audioBuffer, 16) || !isAligned(fftOutput, 16)) {
            throw std::runtime_error("Memory allocation failed or is not aligned");
        }

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
 
    ~FlowerPatch() {
        // Free the aligned memory
        pffft_aligned_free(audioBuffer);
        pffft_aligned_free(fftOutput);
        fftOutput = nullptr; //set back to nullptr as good practice
        audioBuffer = nullptr;
    }
    
    void onSampleRateChange() override {
         sampleRate = APP->engine->getSampleRate();
    }

    void process(const ProcessArgs& args) override {
        sampleRate = args.sampleRate;
        float left = 0.0f, right = 0.0f;
        bool leftConnected = inputs[LEFT_AUDIO_INPUT].isConnected();
        bool rightConnected = inputs[RIGHT_AUDIO_INPUT].isConnected();

        if (leftConnected || rightConnected) {
            inputConnected = true;  // Ensure we track connection status
            left = leftConnected ? inputs[LEFT_AUDIO_INPUT].getVoltage() : 0.0f;
            right = rightConnected ? inputs[RIGHT_AUDIO_INPUT].getVoltage() : 0.0f;

            // Handle mono or stereo inputs
            float mixedSignal = (left + right) * (leftConnected && rightConnected ? 0.05f : 0.1f);

            flowerVal = params[FLOWER_PARAM].getValue(); 
            if(inputs[FLOWER_INPUT].isConnected()){
                flowerVal = clamp(flowerVal +  params[FLOWER_ATT_PARAM].getValue() * inputs[FLOWER_INPUT].getVoltage(), -5.0f, 5.0f);
            }
            
            bufferIndex++;
            bufferIndex %= BUFFER_SIZE;  // Ensure we loop around correctly

            audioBuffer[bufferIndex] = mixedSignal ;            

            waveBuffer[bufferIndex] = ( mixedSignal - 0.11f * flowerVal) / 2.f;

            // Update FFT knob based on parameter and possible external modulation
            FFTknob = params[FFT_PARAM].getValue() * 0.2f; // Scale knob to +-1
            if (inputs[FFT_INPUT].isConnected()) {
                FFTknob = clamp(FFTknob + 0.1f * params[FFT_ATT_PARAM].getValue() * inputs[FFT_INPUT].getVoltage(), -1.0f, 1.1f);
            }

            // Check alignment before triggering FFT processing
            if (bufferIndex == 0 && inputConnected && isAligned(audioBuffer, 16) && isAligned(fftOutput, 16)) {
                fft.rfft(audioBuffer, fftOutput);
                computeIntensityValues();
            }
        }
    }

    void computeIntensityValues() {
        float maxIntensity = 0.0f;
        float freqResolution = sampleRate / BUFFER_SIZE;

        for (size_t i = 0; i < 72; i++) {
            float targetFreq = Scales[i / 12][i % 12];

            // Calculate bin with a tiny nudge to targetFreq and ensure it's non-negative
            int calculatedBin = static_cast<int>((targetFreq * 0.99f) / freqResolution);
            size_t bin = calculatedBin > 0 ? static_cast<size_t>(calculatedBin) : 0;

            // Protect against out-of-bounds access
            if (bin > 0 && bin < (BUFFER_SIZE / 2)) {  // Adjusted boundary check
                // Ensure we are not exceeding the buffer size
                size_t indexReal = 2 * bin;  // Index for real part
                size_t indexImag = 2 * bin + 1;  // Index for imaginary part

                if (indexReal + 1 < BUFFER_SIZE) { // Check if imaginary part is also within bounds
                    float real = fftOutput[indexReal];
                    float imag = fftOutput[indexImag];
                    intensityValues[i] = std::sqrt(real * real + imag * imag);

                    if (intensityValues[i] > maxIntensity) {
                        maxIntensity = intensityValues[i];
                    }
                } else {
                    intensityValues[i] = 0.0f;  // Out of range frequencies set to zero
                }
            } else {
                intensityValues[i] = 0.0f;  // Handle out of range frequencies
            }
        }

        // Normalize and apply power law scaling
        for (size_t i = 0; i < 72; i++) {
            intensityValues[i] /= std::max(maxIntensity, 0.001f);  // Avoid division by zero
            intensityValues[i] = std::pow(intensityValues[i], 3.0f);
        }
    }
    
    void updatePhaseOffset() {
        int maxIndex = 0;
        maxVal = 0;
        zeroCrossIndex = -1;  //-1 indicates not found

        // Find max peak in the buffer
        for (int i = 1; i < 4096/2; i++) { 
            if ((waveBuffer[i]) > maxVal) {
                maxVal = (waveBuffer[i]);
            }
            if (audioBuffer[i] >= 0 && audioBuffer[i - 1] < 0 && zeroCrossIndex == -1) {
                zeroCrossIndex = i;
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
        hue1 = hue1 + 0.1f * module->params[FlowerPatch::HUE_ATT_PARAM].getValue() * module->inputs[FlowerPatch::HUE_INPUT].getVoltage();
    }

    float fillKnob = (module->params[FlowerPatch::FILL_PARAM].getValue())/ 5.0f; // Map -5 to 5 to 0 to 1    
    if(module->inputs[FlowerPatch::FILL_INPUT].isConnected()){
        fillKnob = clamp(fillKnob + 0.1f * module->params[FlowerPatch::FILL_ATT_PARAM].getValue() * module->inputs[FlowerPatch::FILL_INPUT].getVoltage(), -1.0f, 1.0f);
    }
    
    hue1 = fmod(hue1, 1.0f); //wrap the hue
    
    float hue2 = fmod(hue1 + 0.15f,1.0f); // Define second hue point
    
    if (fillKnob<0.0f){
        hue1 = fmod(hue1 - fillKnob*0.15f, 1.0f);
    }
     
    // Set configurable transition points
    float adjFillKnob = pow (fabs(fillKnob), 0.001f); //adjusted Fill to compensate for the steep spikes of FFT output
    float lowPoint = clamp(1.0f-adjFillKnob, 0.000000001f, 1.0f); // Avoid div by zero later - Low point for transition
    float highPoint = clamp(1.0f-adjFillKnob/2.0f, 0.00000001f, 1.0f); // Upper transition point for hue changes

    if (fillKnob < -.99f){
        return nvgHSLA(hue2, 1.0f, 0.75f, 255);   
    }
    
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

    void draw(const DrawArgs& args) override {
        // Handle any non-illuminating drawing here if necessary
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (!module || !module->inputConnected) return;

        if (layer == 1) {  // Only draw on the self-illuminating layer
            const float padding = 20.0f;
            const float totalWidth = box.size.x - 2.0f * padding;
            const float totalHeight = box.size.y - 2.0f * padding;
            const float spaceX = totalWidth / 12.0f;
            const float spaceY = totalHeight / 6.0f;
            const float twoPi = 2.0f * M_PI;

            module->updatePhaseOffset();

			for (size_t scale = 0; scale < 6; scale++) {
				for (size_t note = 0; note < 12; note++) {
					float centerX = padding + spaceX * note + spaceX / 2.0f;
					float centerY = padding + spaceY * scale + spaceY / 2.0f;
					float maxRadius = std::min(spaceX, spaceY) * 0.6f;
					float freq = Scales[scale][note];
					int lastSample = static_cast<int>(2 * (module->sampleRate / freq));
					int flowerIndex = scale * 12 + note;

					nvgBeginPath(args.vg); // Begin the path for the line strip
					bool isFirstSegment = true;  // Flag to skip drawing the line at the wrapping point

					for (int i = 0; i < lastSample; i++) {
						int bufferIndex = (i + module->phaseOffset) % lastSample;
						float sample = module->waveBuffer[bufferIndex];
						float angle = twoPi * (i / (module->sampleRate / freq));
						float radius = maxRadius * (0.5f + 0.5f * sample * (0.5f / fmax(module->maxVal, 0.15f)));

						float FFTintensity = (module->FFTknob > 0) ? 
							(1.f - module->FFTknob) + module->FFTknob * clamp(module->intensityValues[flowerIndex], 0.f, 1.f) :
							(1.f + module->FFTknob) - module->FFTknob * (1 - clamp(module->intensityValues[flowerIndex], 0.f, 1.f));
	
						radius *= FFTintensity;
						radius = std::min(radius, maxRadius);  // Ensuring radius does not exceed maxRadius

						float posX = centerX + radius * cos(angle); // Using cosine for X
						float posY = centerY + radius * sin(angle); // Using sine for Y

						if (isFirstSegment || bufferIndex != 0) {  // Skip drawing if it's the wrap-around segment
							if (i == 0) nvgMoveTo(args.vg, posX, posY);
							else nvgLineTo(args.vg, posX, posY);
						} else {
							nvgMoveTo(args.vg, posX, posY);  // Move to position without drawing a line
						}

						isFirstSegment = false;  // Update flag after the first iteration
					}

					NVGcolor color = colorFromMagnitude(module, module->intensityValues[flowerIndex]);
					nvgStrokeColor(args.vg, color);
					float size = 0.10f * (scale + 3.0f);  // Calculating the size of the square
					nvgStrokeWidth(args.vg, size);
					nvgStroke(args.vg); // Draw the line strip
				}
			}
        }

        // Call the superclass's method to handle other layers or default behaviors
        Widget::drawLayer(args, layer);
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