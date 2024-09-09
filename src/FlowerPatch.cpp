////////////////////////////////////////////////////////////
//
//   Flower Patch
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   12-tone scale waveform visualizer with FFT 
//
////////////////////////////////////////////////////////////


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

// std::array<std::array<std::string, 12>, 6> Names = {{
//     {"C2", "C#2/Db2", "D2", "D#2/Eb2", "E2", "F2", "F#2/Gb2", "G2", "G#2/Ab2", "A2", "A#2/Bb2", "B2"},
//     {"C3", "C#3/Db3", "D3", "D#3/Eb3", "E3", "F3", "F#3/Gb3", "G3", "G#3/Ab3", "A3", "A#3/Bb3", "B3"},
//     {"C4", "C#4/Db4", "D4", "D#4/Eb4", "E4", "F4", "F#4/Gb4", "G4", "G#4/Ab4", "A4", "A#4/Bb4", "B4"},
//     {"C5", "C#5/Db5", "D5", "D#5/Eb5", "E5", "F5", "F#5/Gb5", "G5", "G#5/Ab5", "A5", "A#5/Bb5", "B5"},
//     {"C6", "C#6/Db6", "D6", "D#6/Eb6", "E6", "F6", "F#6/Gb6", "G6", "G#6/Ab6", "A6", "A#6/Bb6", "B6"},
//     {"C7", "C#7/Db7", "D7", "D#7/Eb7", "E7", "F7", "F#7/Gb7", "G7", "G#7/Ab7", "A7", "A#7/Bb7", "B7"}
// }};

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
        AUDIO_INPUT,
        HUE_INPUT,
        FILL_INPUT,
        FLOWER_INPUT,
        FFT_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        FREQUENCY_OUTPUT,
        AMPLITUDE_OUTPUT,
        NUM_OUTPUTS
    };
    enum LightIds {
        NUM_LIGHTS
    };
    enum VisualizerMode {
        FLOWER_MODE,
        WATERFALL_MODE,
        SCOPE_MODE
    };

    static constexpr size_t BUFFER_SIZE = 4096;

    static constexpr int MAX_HISTORY_FRAMES = 100; // Number of frames to store

    // Wave buffer for visualization
    float waveBuffer[BUFFER_SIZE] = {0.0f}; // To store the waveform shape

    // Frame history buffer
    float frameHistory[MAX_HISTORY_FRAMES][BUFFER_SIZE] = {{0}}; // Store history for waveform frames
    int currentFrame = 0; // Index for the current frame

   
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
    bool audioConnected = false;

    VisualizerMode visualizerMode = FLOWER_MODE; // Default to Flower mode

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
    
        // Save the state of visualizerMode
        json_object_set_new(rootJ, "visualizerMode", json_integer(visualizerMode));
        
        return rootJ;
    }
    
    void dataFromJson(json_t* rootJ) override {
        // Load the state of visualizerMode
        json_t* visualizerModeJ = json_object_get(rootJ, "visualizerMode");
        if (visualizerModeJ) {
            visualizerMode = static_cast<VisualizerMode>(json_integer_value(visualizerModeJ));
        }                
    }

    // Update method to handle frame history
    void addFrameToHistory(const float* currentFrameData) {
        currentFrame = (currentFrame + 1) % MAX_HISTORY_FRAMES;
        std::copy(currentFrameData, currentFrameData + 72, frameHistory[currentFrame]);
    }

    FlowerPatch() : Module(), fft(BUFFER_SIZE) {

        //special aligned memory allocation for FFT using pretty-fast-FFT-aligned-malloc
        audioBuffer = static_cast<float*>(pffft_aligned_malloc(BUFFER_SIZE * sizeof(float)));
        fftOutput   = static_cast<float*>(pffft_aligned_malloc(BUFFER_SIZE * sizeof(float)));
        
        if (!audioBuffer || !fftOutput || !isAligned(audioBuffer, 16) || !isAligned(fftOutput, 16)) {
            throw std::runtime_error("Memory allocation failed or is not aligned");
        }

        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
        configInput(AUDIO_INPUT, "Audio Input");

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

        configOutput(FREQUENCY_OUTPUT, "Frequency Peaks (poly)");
        configOutput(AMPLITUDE_OUTPUT, "Amplitudes (poly)");
       
     }
 
    ~FlowerPatch() {
        // Free the aligned memory
        pffft_aligned_free(audioBuffer);
        pffft_aligned_free(fftOutput);
        fftOutput = nullptr; //set back to nullptr as good practice
        audioBuffer = nullptr;
    }

    void addFrameToHistory() {
        // Copy current waveBuffer to the history
        std::copy(waveBuffer, waveBuffer + BUFFER_SIZE, frameHistory[currentFrame]);
        currentFrame = (currentFrame + 1) % MAX_HISTORY_FRAMES;
    }
    
    void onSampleRateChange() override {
         sampleRate = APP->engine->getSampleRate();
    }

    void process(const ProcessArgs& args) override {
        sampleRate = args.sampleRate;

        if (inputs[AUDIO_INPUT].isConnected()) {
            audioConnected = true;
            float audioSignal = inputs[AUDIO_INPUT].getVoltage() * 0.1f;

            flowerVal = params[FLOWER_PARAM].getValue(); 
            if(inputs[FLOWER_INPUT].isConnected()){
                flowerVal = clamp(flowerVal +  params[FLOWER_ATT_PARAM].getValue() * inputs[FLOWER_INPUT].getVoltage(), -5.0f, 5.0f);
            }
            
            bufferIndex++;
            bufferIndex %= BUFFER_SIZE;  // Ensure we loop around correctly

            audioBuffer[bufferIndex] = audioSignal ;            

            waveBuffer[bufferIndex] = ( audioSignal - 0.11f * flowerVal) / 2.f;

            // Update FFT knob based on parameter and possible external modulation
            FFTknob = params[FFT_PARAM].getValue() * 0.2f; // Scale knob to +-1
            if (inputs[FFT_INPUT].isConnected()) {
                FFTknob = clamp(FFTknob + 0.1f * params[FFT_ATT_PARAM].getValue() * inputs[FFT_INPUT].getVoltage(), -1.0f, 1.1f);
            }

            // Check alignment before triggering FFT processing
            if (bufferIndex == 0 && isAligned(audioBuffer, 16) && isAligned(fftOutput, 16)) {
                applyWindow(audioBuffer, BUFFER_SIZE);  // Apply windowing function
                fft.rfft(audioBuffer, fftOutput);
                computeIntensityValues(); // calculate the intensity based on 12 tone scale bins
                findTopPeaks();  // identify and output top peaks as v/oct
            }
        }
    }


    void computeIntensityValues() {
        float maxIntensity = 0.0f;
        float freqResolution = sampleRate / BUFFER_SIZE;
    
        for (size_t i = 0; i < 72; i++) {
            float targetFreq = Scales[i / 12][i % 12];
            size_t bin = static_cast<size_t>(std::round(targetFreq / freqResolution));
    
            // Protect against out-of-bounds access
            if (bin < (BUFFER_SIZE / 2)) {
                size_t indexReal = 2 * bin;
                size_t indexImag = 2 * bin + 1;
    
                if (indexReal + 1 < BUFFER_SIZE) {
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

    void findTopPeaks() {
        const size_t numPeaks = 6;
        float topIntensities[numPeaks] = {0};
        size_t topIndices[numPeaks] = {0};
    
        // Initialize arrays to store top peaks
        std::fill(std::begin(topIntensities), std::end(topIntensities), -1.0f);  // Start with -1 to ensure any intensity is higher
    
        // Find the top peaks
        for (size_t i = 0; i < 72; i++) {
            float intensity = intensityValues[i];
    
            // Check if current intensity is higher than the lowest of the top peaks
            for (size_t j = 0; j < numPeaks; j++) {
                if (intensity > topIntensities[j]) {
                    // Shift lower peaks down
                    for (size_t k = numPeaks - 1; k > j; k--) {
                        topIntensities[k] = topIntensities[k - 1];
                        topIndices[k] = topIndices[k - 1];
                    }
    
                    // Insert new peak
                    topIntensities[j] = intensity;
                    topIndices[j] = i;
                    break;
                }
            }
        }
    
        // Set the number of channels for the outputs
        outputs[FREQUENCY_OUTPUT].setChannels(numPeaks);
        outputs[AMPLITUDE_OUTPUT].setChannels(numPeaks);
    
        // Output the top N bins as v/oct and their amplitudes
        for (size_t i = 0; i < numPeaks; i++) {
            // Convert bin index to frequency
            size_t binIndex = topIndices[i];
            float frequency = Scales[binIndex / 12][binIndex % 12];
            
            // Convert frequency to v/oct using inline formula
    const float referenceFrequency = 440.0f; // Reference frequency (A4)
    const float referenceVoltage = 0.750f;   // Voltage corresponding to the reference frequency

    // Calculate the voltage in v/oct
    float vOct = referenceVoltage + std::log2(frequency / referenceFrequency);
            // Output v/oct and scaled amplitude for each peak
            outputs[FREQUENCY_OUTPUT].setVoltage(vOct, i);  // Set voltage for each channel
            outputs[AMPLITUDE_OUTPUT].setVoltage(topIntensities[i] * 10.0f, i);  // Set voltage for each channel
        }
    }

    void applyWindow(float* buffer, size_t size) {
        for (size_t i = 0; i < size; ++i) {
            buffer[i] *= 0.5f * (1.0f - std::cos(2.0f * M_PI * i / (size - 1))); // Hamming window
        }
    }
    
    void updatePhaseOffset() {
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
        if (!module || !module->audioConnected) return;

        if (layer == 1) {  // Only draw on the self-illuminating layer
            const float padding = 20.0f;
            const float totalWidth = box.size.x - 2.0f * padding;
            const float totalHeight = box.size.y - 2.0f * padding;
            const float spaceX = totalWidth / 12.0f;
            const float spaceY = totalHeight / 6.0f;
            const float twoPi = 2.0f * M_PI;

            module->updatePhaseOffset();

            switch (module->visualizerMode) {
                case FlowerPatch::FLOWER_MODE: {
                    // Flower mode visualization
                    for (size_t scale = 0; scale < 6; scale++) {
                        for (size_t note = 0; note < 12; note++) {
                            float centerX = padding + spaceX * note + spaceX / 2.0f;
                            float centerY = padding + spaceY * scale + spaceY / 2.0f;
                            float maxRadius = std::min(spaceX, spaceY) * 0.6f;
                            float freq = Scales[scale][note];
                            int lastSample = static_cast<int>(2 * (module->sampleRate / freq));
                            int flowerIndex = scale * 12 + note;

                            nvgBeginPath(args.vg);
                            bool isFirstSegment = true;

                            for (int i = 0; i < lastSample; i++) {
                                int bufferIndex = (i + module->phaseOffset) % lastSample;
                                float sample = module->waveBuffer[bufferIndex];
                                float angle = twoPi * (i / (module->sampleRate / freq));
                                float radius = maxRadius * (0.5f + 0.5f * sample * (0.5f / fmax(module->maxVal, 0.15f)));

                                float FFTintensity = (module->FFTknob > 0) ? 
                                    (1.f - module->FFTknob) + module->FFTknob * clamp(module->intensityValues[flowerIndex], 0.f, 1.f) :
                                    (1.f + module->FFTknob) - module->FFTknob * (1 - clamp(module->intensityValues[flowerIndex], 0.f, 1.f));
 
                                radius *= FFTintensity;
                                radius = std::min(radius, maxRadius);

                                float posX = centerX + radius * cos(angle);
                                float posY = centerY + radius * sin(angle);

                                if (isFirstSegment || bufferIndex != 0) {
                                    if (i == 0) nvgMoveTo(args.vg, posX, posY);
                                    else nvgLineTo(args.vg, posX, posY);
                                } else {
                                    nvgMoveTo(args.vg, posX, posY);
                                }

                                isFirstSegment = false;
                            }

                            NVGcolor color = colorFromMagnitude(module, module->intensityValues[flowerIndex]);
                            nvgStrokeColor(args.vg, color);
                            float size = 0.10f * (scale + 3.0f);
                            nvgStrokeWidth(args.vg, size);
                            nvgStroke(args.vg);
                        }
                    }
                    break;
                }

                case FlowerPatch::WATERFALL_MODE: {
                    const float barWidth = (totalWidth != 0.0f) ? (totalWidth / 72.0f) : 1.0f;
                    const float fadeFactor = 0.95f; // Fading coefficient
                    const int numBars = 72; // Number of bars across the width
                    const float maxDrift = totalHeight * 0.6f; // Increased vertical drift for old spectra
                
                    // Knob settings
                    float fillKnob = module->params[FlowerPatch::FILL_PARAM].getValue() / 5.0f + 1.2f; // Fill opacity
                    float powerKnob = module->params[FlowerPatch::FFT_PARAM].getValue() / 6.0f + 1.0f; // Scales the max height
                    float flowerKnob = module->params[FlowerPatch::FLOWER_PARAM].getValue() / 5.0f + 1.0f; // Controls X-scale expansion
                
                    // Check for knob inputs
                    if (module->inputs[FlowerPatch::FILL_INPUT].isConnected()) {
                        fillKnob = clamp(fillKnob + 0.1f * module->params[FlowerPatch::FILL_ATT_PARAM].getValue() * module->inputs[FlowerPatch::FILL_INPUT].getVoltage(), -1.0f, 1.0f);
                    }
                    if (module->inputs[FlowerPatch::FFT_INPUT].isConnected()) {
                        powerKnob = clamp(powerKnob + 0.1f * module->params[FlowerPatch::FFT_ATT_PARAM].getValue() * module->inputs[FlowerPatch::FFT_INPUT].getVoltage(), -1.0f, 1.0f);
                    }
                    if (module->inputs[FlowerPatch::FLOWER_INPUT].isConnected()) {
                        flowerKnob = clamp(flowerKnob + 0.1f * module->params[FlowerPatch::FLOWER_ATT_PARAM].getValue() * module->inputs[FlowerPatch::FLOWER_INPUT].getVoltage(), -1.0f, 1.0f);
                    }
                
                    // Clip the drawing to the bounds of the widget
                    nvgScissor(args.vg, padding, padding, totalWidth, totalHeight);
                
                    // Draw previous frames with fading effect
                    for (int i = 0; i < FlowerPatch::MAX_HISTORY_FRAMES; i++) {
                        int frameIndex = (module->currentFrame - i + FlowerPatch::MAX_HISTORY_FRAMES) % FlowerPatch::MAX_HISTORY_FRAMES;
                        float opacity = powf(fadeFactor, i) * fillKnob; // Decrease opacity for older frames and apply fill knob
                
                        if (opacity < 0.05f) continue; // Skip frames that are too faded
                
                        // Calculate vertical drift and horizontal scale based on age
                        float drift = (FlowerPatch::MAX_HISTORY_FRAMES != 0) ? (i / static_cast<float>(FlowerPatch::MAX_HISTORY_FRAMES)) * maxDrift : 0.0f;
                        float scale = (FlowerPatch::MAX_HISTORY_FRAMES != 0) ? 1.0f + (i / static_cast<float>(FlowerPatch::MAX_HISTORY_FRAMES)) * (flowerKnob - 1.0f) : 1.0f;
           
                        // Begin drawing path for the faded frame
                        nvgBeginPath(args.vg);
                
                        // Draw the spectrum for the frame
                        for (size_t bar = 0; bar < numBars; bar++) {
                            size_t note = bar % 12;
                            size_t scaleIndex = bar / 12;
                
                            float centerX = padding + bar * barWidth * scale - drift*(flowerKnob) + drift;
                            float centerY = padding + totalHeight - drift;
                            float intensity = module->frameHistory[frameIndex][scaleIndex * 12 + note];
                            if (std::isnan(intensity) || std::isinf(intensity)) { // avoid rendering errors
                                intensity = 1.0f; // Set a fallback value
                            }
                            float barHeight = intensity * totalHeight * 0.4f * powerKnob; // Apply scaling to height
                
                            // Ensure we don't draw outside the visible bounds
                            barHeight = std::min(barHeight, totalHeight);
                
                            if (bar == 0) {
                                nvgMoveTo(args.vg, centerX, centerY - barHeight);
                            } else {
                                nvgLineTo(args.vg, centerX, centerY - barHeight);
                            }
                        }
                
                        NVGcolor color = colorFromMagnitude(module, 1.0f);
                        color.a *= opacity; // Apply fade effect and fill knob
                        nvgStrokeColor(args.vg, color);
                        nvgStrokeWidth(args.vg, 1.5f); // Adjust stroke width as needed
                        nvgStroke(args.vg);
                    }
                
                    // Draw the current frame on top
                    module->addFrameToHistory(module->intensityValues);
                
                    nvgBeginPath(args.vg);
                
                    // Draw the spectrum for the current frame
                    for (size_t bar = 0; bar < numBars; bar++) {
                        size_t note = bar % 12;
                        size_t scaleIndex = bar / 12;
                
                        float centerX = padding + bar * barWidth; // Apply flower knob to X-scale
                        float centerY = padding + totalHeight;
                        float intensity = module->intensityValues[scaleIndex * 12 + note];
                        float barHeight = intensity * totalHeight * 0.5f * powerKnob; // Apply scaling to height
                
                        // Ensure we don't draw outside the visible bounds
                        barHeight = std::min(barHeight, totalHeight);
                
                        if (bar == 0) {
                            nvgMoveTo(args.vg, centerX, centerY - barHeight);
                        } else {
                            nvgLineTo(args.vg, centerX, centerY - barHeight);
                        }
                    }
                
                    NVGcolor color = colorFromMagnitude(module, 1.0f);
                    color.a *= fillKnob; // Apply fill knob to opacity
                    nvgStrokeColor(args.vg, color);
                    nvgStrokeWidth(args.vg, 2.0f); // Adjust stroke width as needed
                    nvgStroke(args.vg);
                
                    // Reset the scissor
                    nvgResetScissor(args.vg);
                
                    break;
                }

                case FlowerPatch::SCOPE_MODE: {
                    const float fadeFactor = 0.92f;  // Adjust for fading effect
                    const int maxHistoryFrames = FlowerPatch::MAX_HISTORY_FRAMES;
                    const float maxDrift = totalHeight * 0.6f;
                    
                    float hueKnob = module->params[FlowerPatch::HUE_PARAM].getValue();
                    if (module->inputs[FlowerPatch::HUE_INPUT].isConnected()) {
                        hueKnob += module->params[FlowerPatch::HUE_ATT_PARAM].getValue() * module->inputs[FlowerPatch::HUE_INPUT].getVoltage();
                    }
                    hueKnob = clamp(hueKnob, -5.0f, 5.0f);
                    hueKnob = (hueKnob + 5.0f) / 10.0f;  // Scale from range [-5, 5] to [0, 1]
                
                    // Use Power knob to select which flower to draw
                    float powerKnob = module->params[FlowerPatch::FFT_PARAM].getValue();
                    if (module->inputs[FlowerPatch::FFT_INPUT].isConnected()) {
                        powerKnob += module->params[FlowerPatch::FFT_ATT_PARAM].getValue() * module->inputs[FlowerPatch::FFT_INPUT].getVoltage();
                    }
                    powerKnob = clamp(powerKnob, -1.0f, 1.0f);  // Ensure it's within valid range
                    int selectedFlower = clamp(static_cast<int>(roundf((powerKnob + 5.0f) * 71.0f / 10.0f)), 0, 71);
                
                    float centerX = padding + totalWidth / 2.0f;
                    float centerY = padding + totalHeight / 2.0f;
                    float maxRadius = std::min(totalWidth, totalHeight) * 0.4f;
 
                    float fillKnob = module->params[FlowerPatch::FILL_PARAM].getValue();
                    
                    // Adjust with input CV and attenuverter
                    if (module->inputs[FlowerPatch::FILL_INPUT].isConnected()) {
                        fillKnob += module->params[FlowerPatch::FILL_ATT_PARAM].getValue() * module->inputs[FlowerPatch::FILL_INPUT].getVoltage();
                    }
                    
                    // Scale and clamp the fillKnob value
                    fillKnob = clamp(fillKnob / 5.0f + 1.0f, 0.0, 2.5f);
           
                    nvgScissor(args.vg, padding, padding, totalWidth, totalHeight);
                
                    // Draw previous frames with fading
                    for (int i = 0; i < maxHistoryFrames; i++) { //also avoids div/zero if maxHistoryFrames is zero
                        int frameIndex = (module->currentFrame - i + maxHistoryFrames) % maxHistoryFrames;
                        float opacity = powf(fadeFactor, i) * fillKnob;
                
                        if (opacity < 0.02f) continue;
                
                        float drift = (i / static_cast<float>(maxHistoryFrames)) * maxDrift;
                        float scale = 1.0f + (i / static_cast<float>(maxHistoryFrames));
             
                        nvgBeginPath(args.vg);
                        bool isFirstSegment = true;
                
                        float freq = Scales[selectedFlower / 12][selectedFlower % 12];
                        int lastSample = static_cast<int>(4 * (module->sampleRate / freq)); // Draw 2x as many samples
                
                        const auto& frameData = module->frameHistory[frameIndex];
                
                        for (int j = 0; j < lastSample; j++) {
                            int bufferIndex = j % (lastSample / 2); // Handle wrapping around
                            float sample = frameData[bufferIndex];
               
                            // Avoid NaN or inf values
                            if (std::isnan(sample) || std::isinf(sample)) {
                                sample = 0.0f; // Fallback to zero
                            }
                
                            float angle = 2.0f * twoPi * (float(j) / lastSample); // 720° rotation
                            float radius = maxRadius * (0.5f + 0.5f * sample * (0.5f / fmax(module->maxVal, 0.15f)));
                            radius *= scale;
                
                            float posX = centerX + (radius + drift) * cos(angle);
                            float posY = centerY + (radius + drift) * sin(angle);
                
                            if (isFirstSegment) {
                                nvgMoveTo(args.vg, posX, posY);
                                isFirstSegment = false;
                            } else {
                                nvgLineTo(args.vg, posX, posY);
                            }
                        }
                
                        // Apply color with fading hue
                        float hueShift = i / float(maxHistoryFrames) * 0.2f;  // Adjust hue shift to match the range in other modes
                        NVGcolor color = nvgHSL(hueKnob + hueShift, 0.5f, 0.5f);  // Use hueKnob to set color
                        color.a *= opacity;
                        nvgStrokeColor(args.vg, color);
                        nvgStrokeWidth(args.vg, 1.5f);
                        nvgStroke(args.vg);
                    }
                
                    // Draw the current frame on top (without fading)
                    nvgBeginPath(args.vg);
                    bool isFirstSegment = true;
                    float freq = Scales[selectedFlower / 12][selectedFlower % 12];
                    int lastSample = std::max(static_cast<int>(4 * (module->sampleRate / freq)), 1); // Ensure valid sample count, Use 2X so it goes around twice
      
                    for (int i = 0; i < lastSample; i++) {
                        int bufferIndex = (i + module->phaseOffset) % (lastSample / 2); // Handle wrapping around
                        float sample = module->waveBuffer[bufferIndex];
               
                        // Avoid NaN or inf values
                        if (std::isnan(sample) || std::isinf(sample)) {
                            sample = 0.0f; // Fallback to zero
                        }
                
                        float angle = 2.0f * twoPi * (float(i) / lastSample); // 720° rotation
                        float radius = maxRadius * (0.5f + 0.5f * sample * (0.5f / fmax(module->maxVal, 0.15f)));
                
                        float posX = centerX + radius * cos(angle);
                        float posY = centerY + radius * sin(angle);
                
                        if (isFirstSegment) {
                            nvgMoveTo(args.vg, posX, posY);
                            isFirstSegment = false;
                        } else {
                            nvgLineTo(args.vg, posX, posY);
                        }
                    }
                
                    // Apply color to the current frame
                    NVGcolor color = nvgHSL(hueKnob, 0.5f, 0.5f);  // Use hueKnob for the current frame color
                    nvgStrokeColor(args.vg, color);
                    nvgStrokeWidth(args.vg, 2.0f);
                    nvgStroke(args.vg);
                
                    // Store current frame in history buffer
                    module->addFrameToHistory();
                
                    nvgResetScissor(args.vg);
                    break;
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
        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        float spacing = 2*5.08;
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(spacing, 112.373)), module, FlowerPatch::AUDIO_INPUT));
        spacing += 2.6*5.08;

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

        spacing += 1.75*5.08;
        addOutput(createOutput<ThemedPJ301MPort>(mm2px(Vec(spacing, 112.373-4)), module, FlowerPatch::FREQUENCY_OUTPUT));
        addOutput(createOutput<ThemedPJ301MPort>(mm2px(Vec(spacing, 112.373+4)), module, FlowerPatch::AMPLITUDE_OUTPUT));



        FlowerDisplay* display = new FlowerDisplay();
        display->box.pos = Vec(5, 25);
        display->box.size = Vec(box.size.x, 300);
        display->module = module;
        addChild(display);
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
    
        FlowerPatch* flowerPatchModule = dynamic_cast<FlowerPatch*>(module);
        assert(flowerPatchModule); // Ensure the cast succeeds
    
        // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());
    
        // Define the menu item for Flower mode
        struct FlowerModeMenuItem : MenuItem {
            FlowerPatch* flowerPatchModule;
            void onAction(const event::Action& e) override {
                flowerPatchModule->visualizerMode = FlowerPatch::FLOWER_MODE;
            }
            void step() override {
                rightText = (flowerPatchModule->visualizerMode == FlowerPatch::FLOWER_MODE) ? "✔" : "";
                MenuItem::step();
            }
        };
    
        // Define the menu item for Waterfall mode
        struct WaterfallModeMenuItem : MenuItem {
            FlowerPatch* flowerPatchModule;
            void onAction(const event::Action& e) override {
                flowerPatchModule->visualizerMode = FlowerPatch::WATERFALL_MODE;
            }
            void step() override {
                rightText = (flowerPatchModule->visualizerMode == FlowerPatch::WATERFALL_MODE) ? "✔" : "";
                MenuItem::step();
            }
        };
    
        // Define the menu item for Scope mode
        struct ScopeModeMenuItem : MenuItem {
            FlowerPatch* flowerPatchModule;
            void onAction(const event::Action& e) override {
                flowerPatchModule->visualizerMode = FlowerPatch::SCOPE_MODE;
            }
            void step() override {
                rightText = (flowerPatchModule->visualizerMode == FlowerPatch::SCOPE_MODE) ? "✔" : "";
                MenuItem::step();
            }
        };
    
        // Add the menu items to the context menu
        FlowerModeMenuItem* flowerModeItem = new FlowerModeMenuItem();
        flowerModeItem->text = "Flower Mode";
        flowerModeItem->flowerPatchModule = flowerPatchModule;
        menu->addChild(flowerModeItem);
    
        WaterfallModeMenuItem* waterfallModeItem = new WaterfallModeMenuItem();
        waterfallModeItem->text = "Waterfall Mode";
        waterfallModeItem->flowerPatchModule = flowerPatchModule;
        menu->addChild(waterfallModeItem);
    
        ScopeModeMenuItem* scopeModeItem = new ScopeModeMenuItem();
        scopeModeItem->text = "Scope Mode";
        scopeModeItem->flowerPatchModule = flowerPatchModule;
        menu->addChild(scopeModeItem);
    }    
    
};

Model* modelFlowerPatch = createModel<FlowerPatch, FlowerPatchWidget>("FlowerPatch");