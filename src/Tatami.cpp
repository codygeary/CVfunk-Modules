////////////////////////////////////////////////////////////
//
//   Tatami
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   A wave-shaper and wave-folder
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

// Utility function to constrain input to the range [-pi, pi]
float wrapToPi(float x) {
    const float twoPi = 2.0f * M_PI;
    x = fmod(x + M_PI, twoPi); // Wrap x to [0, 2*pi)
    if (x < 0.0f) x += twoPi; // Ensure non-negative result
    return x - M_PI;          // Shift to [-pi, pi]
}


// Sine approximation with cyclic input
float polySin(float x) {
    x = wrapToPi(x);
    float x2 = x * x;       // x^2
    float x3 = x * x2;      // x^3
    float x5 = x3 * x2;     // x^5
    float x7 = x5 * x2;     // x^7
    float x9 = x7 * x2;     // x^9
    return x - x3 / 6.0f + x5 / 120.0f - x7 / 5040.0f + x9 / 362880.0f;
}


// Cosine approximation with cyclic input
float polyCos(float x) {
    x = wrapToPi(x);
    float x2 = x * x;       // x^2
    float x4 = x2 * x2;     // x^4
    float x6 = x4 * x2;     // x^6
    float x8 = x6 * x2;     // x^8
    return 1.0f - x2 / 2.0f + x4 / 24.0f - x6 / 720.0f + x8 / 40320.0f;
}


struct SecondOrderHPF {
    float x1 = 0, x2 = 0; // previous two inputs
    float y1 = 0, y2 = 0; // previous two outputs
    float a0, a1, a2;     // filter coefficients for the input
    float b1, b2;         // filter coefficients for the output

    SecondOrderHPF() {}

    // Initialize the filter coefficients
    void setCutoffFrequency(float sampleRate, float cutoffFreq) {
        float w0 = 2 * M_PI * cutoffFreq / sampleRate;
        float cosw0 = polyCos(w0);
        float sinw0 = polySin(w0);
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

struct Tatami : Module {
    enum ParamId {
        SHAPE_ATT_PARAM,
        SHAPE_PARAM,
        COMPRESS_ATT_PARAM,
        COMPRESS_PARAM,
        SYMMETRY_ATT_PARAM,
        SYMMETRY_PARAM,
        DENSITY_PARAM1,
        DENSITY_ATT_PARAM,
        DENSITY_PARAM2,
        PARAMS_LEN
    };
    enum InputId {
        AUDIO_L_INPUT,
        SHAPE_INPUT,
        AUDIO_R_INPUT,
        COMPRESS_INPUT,
        SYMMETRY_INPUT,
        DENSITY_INPUT1,
        DENSITY_INPUT2,
        INPUTS_LEN
    };
    enum OutputId {
        AUDIO_L_OUTPUT,
        AUDIO_R_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        LIGHTS_LEN
    };

    float sampleRate = APP->engine->getSampleRate();
    float scaleFactor = sampleRate / 96000.0f; // Reference sample rate (96 kHz)
    float decayRate = pow(0.999f, scaleFactor);  // Decay rate adjusted for sample rate
    float increment_factor = 44100.f/(1024.f * sampleRate);

    float alpha = 0.01f;
    float inputL[16] = {0.0f};
    float inputR[16] = {0.0f};
    float envPeakL[16] = {0.0f};
    float envPeakR[16] = {0.0f};
    float filteredEnvelopeL[16] = {0.0f};
    float filteredEnvelopeR[16] = {0.0f};
    float lastOutputL = 0.0f;
    float lastOutputR = 0.0f;
    float outputL[16] = {0.0f};
    float outputR[16] = {0.0f};
    bool initialize = true;
    bool applyFilters = true;
    
    // Declare high-pass filter
    SecondOrderHPF hpfL[16], hpfR[16];

    //For the display
    CircularBuffer<float, 1024> waveBuffers[3];
    float oscPhase = 0.0f;
    float prevOscVal = 0.0f;
    float funcPhase = 0.0f;
    float zeroCrossingPhase = 0.0f;

    CircularBuffer<float, 1024> tempBuffer;
    int tempBufferIndex = 0;
    float tempBufferPhase = 0.0f;

    // Initialize Butterworth filter for oversampling
    SimpleShaper shaperL[16];  // Instance of the oversampling and shaping processor
    SimpleShaper shaperR[16];  // Instance of the oversampling and shaping processor
    Filter6PButter butterworthFilter;  // Butterworth filter instance
    bool isSupersamplingEnabled = false;  // Enable supersampling is off by default

    // Save state to JSON
    json_t* toJson() override {
        json_t* rootJ = Module::toJson();
        json_object_set_new(rootJ, "applyFilters", json_boolean(applyFilters));
        json_object_set_new(rootJ, "isSupersamplingEnabled", json_boolean(isSupersamplingEnabled));

        return rootJ;
    }

    // Load state from JSON
    void fromJson(json_t* rootJ) override {
        Module::fromJson(rootJ);
        json_t* applyFiltersJ = json_object_get(rootJ, "applyFilters");
        if (applyFiltersJ) {
            applyFilters = json_is_true(applyFiltersJ);
        }

        json_t* isSupersamplingEnabledJ = json_object_get(rootJ, "isSupersamplingEnabled");
        if (isSupersamplingEnabledJ) {
            isSupersamplingEnabled = json_is_true(isSupersamplingEnabledJ);
        }
    }

    Tatami() : Module() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(SHAPE_ATT_PARAM, 0.f, 1.f, 0.f, "Shape Att.");
        configParam(SHAPE_PARAM, 0.f, 3.f, 0.f, "Wave Shape");
        configParam(COMPRESS_ATT_PARAM, 0.f, 1.f, 0.f, "Compress Att.");
        configParam(COMPRESS_PARAM, 0.f, 10.f, 0.f, "Pre Folding Compression");
        configParam(SYMMETRY_ATT_PARAM, 0.f, 1.f, 0.f, "Symmetry Att.");
        configParam(SYMMETRY_PARAM, -5.f, 5.f, 0.f, "Symmetry - Input Bias");
        configParam(DENSITY_PARAM1, 1.f, 20.f, 1.f, "Folding Density Left");
        configParam(DENSITY_PARAM2, 1.f, 20.f, 1.f, "Folding Density Right");
        configParam(DENSITY_ATT_PARAM, 0.f, 1.f, 0.f, "Density Att.");

        configInput(AUDIO_L_INPUT, "L Audio In");
        configInput(AUDIO_R_INPUT, "R Audio In");

        configInput(SHAPE_INPUT, "Shape CV");
        configInput(COMPRESS_INPUT, "Compress CV");
        configInput(SYMMETRY_INPUT, "Symmetry CV");
        configInput(DENSITY_INPUT1, "Density Left CV");
        configInput(DENSITY_INPUT2, "Density Right CV");

        configOutput(AUDIO_L_OUTPUT, "L Audio Out");
        configOutput(AUDIO_R_OUTPUT, "R Audio Out");

    }

    void onSampleRateChange() override {
        // Calculate scale factor based on the current sample rate
        scaleFactor = sampleRate / 96000.0f; // Reference sample rate (96 kHz)
        // Adjust alpha and decayRate based on sample rate
        alpha = 0.01f / scaleFactor;  // Smoothing factor for envelope
        decayRate = pow(0.999f, scaleFactor);  // Decay rate adjusted for sample rate
        increment_factor = 44100.f/(1024.f * sampleRate);
    }
    
    void process(const ProcessArgs& args) override {

        // Setup filters
        if (initialize){
            for (int i=0; i<16; i++){
                hpfL[i].setCutoffFrequency(sampleRate, 10.0f); // Set cutoff frequency
                hpfR[i].setCutoffFrequency(sampleRate, 10.0f);
            }
            initialize = false;
        }

        int audioLChannels = inputs[AUDIO_L_INPUT].getChannels();
        int audioRChannels = inputs[AUDIO_R_INPUT].getChannels();

        int numChannels = std::max(audioLChannels, audioRChannels);
        numChannels = std::max(numChannels, 1);
        outputs[AUDIO_L_OUTPUT].setChannels(numChannels);
        outputs[AUDIO_R_OUTPUT].setChannels(numChannels);

        // Check if each input is monophonic
        bool isShapeMonophonic = inputs[SHAPE_INPUT].isConnected() && (inputs[SHAPE_INPUT].getChannels() == 1);
        bool isCompressMonophonic = inputs[COMPRESS_INPUT].isConnected() && (inputs[COMPRESS_INPUT].getChannels() == 1);
        bool isSymmetryMonophonic = inputs[SYMMETRY_INPUT].isConnected() && (inputs[SYMMETRY_INPUT].getChannels() == 1);
        bool isDensity1Monophonic = inputs[DENSITY_INPUT1].isConnected() && (inputs[DENSITY_INPUT1].getChannels() == 1);
        bool isDensity2Monophonic = inputs[DENSITY_INPUT2].isConnected() && (inputs[DENSITY_INPUT2].getChannels() == 1);

        // Get monophonic input values
        float shapeMonoValue = isShapeMonophonic ? inputs[SHAPE_INPUT].getVoltage(0) : 0.0f;
        float compressMonoValue = isCompressMonophonic ? inputs[COMPRESS_INPUT].getVoltage(0) : 0.0f;
        float symmetryMonoValue = isSymmetryMonophonic ? inputs[SYMMETRY_INPUT].getVoltage(0) : 0.0f;
        float densityMonoValue1 = isDensity1Monophonic ? inputs[DENSITY_INPUT1].getVoltage(0) : 0.0f;
        float densityMonoValue2 = isDensity2Monophonic ? inputs[DENSITY_INPUT2].getVoltage(0) : 0.0f;

        if (inputs[DENSITY_INPUT1].isConnected() && !inputs[DENSITY_INPUT2].isConnected()) {
            // Normalize Density Input 2 to Density Input 1
            densityMonoValue2 = densityMonoValue1;
        } else if (!inputs[DENSITY_INPUT1].isConnected() && inputs[DENSITY_INPUT2].isConnected()) {
            // Normalize Density Input 1 to Density Input 2
            densityMonoValue1 = densityMonoValue2;
        }

        float shape_top = 0.0f;
        float zero_tracking = 0.0f;

        for (int c = 0; c < numChannels; c++) {
            // Process Shape input
            float shape = params[SHAPE_PARAM].getValue();
            if (inputs[SHAPE_INPUT].isConnected()) {
                shape += (isShapeMonophonic ? shapeMonoValue : inputs[SHAPE_INPUT].getVoltage(c)) * params[SHAPE_ATT_PARAM].getValue();
            }

            // Wrap the shape value to the range [0, 3)
            shape = fmod(shape, 3.0f);
            if (shape < 0.0f) {
                shape += 3.0f; // Wrapping from negative to positive
            }
            shape = clamp (shape, 0.0f, 3.0f);
            if (c==0){shape_top = shape;}

            // Process Compress input
            float compress = params[COMPRESS_PARAM].getValue()*0.1f;
            if (inputs[COMPRESS_INPUT].isConnected()) {
                compress += (isCompressMonophonic ? compressMonoValue : inputs[COMPRESS_INPUT].getVoltage(c)) * 0.1f * params[COMPRESS_ATT_PARAM].getValue();
            }
            compress = clamp(compress, 0.0f, 1.0f);

            // Process Symmetry input
            float symmetry = params[SYMMETRY_PARAM].getValue();
            if (inputs[SYMMETRY_INPUT].isConnected()) {
                symmetry += (isSymmetryMonophonic ? symmetryMonoValue : inputs[SYMMETRY_INPUT].getVoltage(c)) * params[SYMMETRY_ATT_PARAM].getValue();
            }
            symmetry = clamp(symmetry, -5.0f, 5.0f);

            // Process Density inputs (normalized if one is disconnected)
            float densityLeft = params[DENSITY_PARAM1].getValue();
            float densityRight = params[DENSITY_PARAM2].getValue();

            if (inputs[DENSITY_INPUT1].isConnected() || inputs[DENSITY_INPUT2].isConnected()) {
                float densityInput1 = (isDensity1Monophonic ? densityMonoValue1 : inputs[DENSITY_INPUT1].getVoltage(c));
                float densityInput2 = (isDensity2Monophonic ? densityMonoValue2 : inputs[DENSITY_INPUT2].getVoltage(c));

                if (inputs[DENSITY_INPUT1].isConnected() && !inputs[DENSITY_INPUT2].isConnected()) {
                    densityInput2 = densityInput1; // Normalize Input 2 to Input 1
                } else if (!inputs[DENSITY_INPUT1].isConnected() && inputs[DENSITY_INPUT2].isConnected()) {
                    densityInput1 = densityInput2; // Normalize Input 1 to Input 2
                }

                densityLeft += densityInput1 * params[DENSITY_ATT_PARAM].getValue();
                densityRight += densityInput2 * params[DENSITY_ATT_PARAM].getValue();
            }
            densityLeft = clamp (densityLeft, 1.0f, 30.0f);
            densityRight = clamp (densityRight, 1.0f, 30.0f);
            
            if (inputs[AUDIO_L_INPUT].isConnected() && inputs[AUDIO_R_INPUT].isConnected()) {
                // Both L and R are connected, use them directly
                inputL[c] = inputs[AUDIO_L_INPUT].getVoltage(c);
                inputR[c] = inputs[AUDIO_R_INPUT].getVoltage(c);
            }
            else if (inputs[AUDIO_L_INPUT].isConnected() && !inputs[AUDIO_R_INPUT].isConnected()) {
                // Only L is connected, map L to both L and R
                inputL[c] = inputs[AUDIO_L_INPUT].getVoltage(c);
                inputR[c] = inputs[AUDIO_L_INPUT].getVoltage(c);
            }
            else if (!inputs[AUDIO_L_INPUT].isConnected() && inputs[AUDIO_R_INPUT].isConnected()) {
                // Only R is connected, map R to both L and R
                inputL[c] = inputs[AUDIO_R_INPUT].getVoltage(c);
                inputR[c] = inputs[AUDIO_R_INPUT].getVoltage(c);
            }
            else {
                // Neither L nor R is connected, default to 0V for both
                inputL[c] = 0.0f;
                inputR[c] = 0.0f;
            }
            
            inputL[c] = clamp (inputL[c], -10.f, 10.f);
            inputR[c] = clamp (inputR[c], -10.f, 10.f);
            
            if (c==0){zero_tracking = inputL[0];} //for centering the scope signal

            if (compress > 0.01f){ //check if compression is enabled
                // Simple peak detection using the absolute maximum of the current input
                envPeakL[c] = fmax(envPeakL[c] * decayRate, fabs(inputL[c]));
                envPeakR[c] = fmax(envPeakR[c] * decayRate, fabs(inputR[c]));

                filteredEnvelopeL[c] = fmax(filteredEnvelopeL[c],0.1f);
                filteredEnvelopeR[c] = fmax(filteredEnvelopeR[c],0.1f);

                filteredEnvelopeL[c] = alpha * envPeakL[c] + (1 - alpha) * filteredEnvelopeL[c];
                filteredEnvelopeR[c] = alpha * envPeakR[c] + (1 - alpha) * filteredEnvelopeR[c];

                // Compress audio inputs:
                inputL[c] = (inputL[c]/filteredEnvelopeL[c])*compress*5.0f + inputL[c]*(1-compress);
                inputR[c] = (inputR[c]/filteredEnvelopeR[c])*compress*5.0f + inputR[c]*(1-compress);
            }

            // Apply Symmetry
            inputL[c] += symmetry;
            inputR[c] += symmetry;

            // Apply Density
            inputL[c] *= densityLeft;
            inputR[c] *= densityRight;

            inputL[c] = clamp (inputL[c], -200.f, 200.f);
            inputR[c] = clamp (inputR[c], -200.f, 200.f);

            // Apply ADAA wavefolding
            outputL[c] = applyADAAWaveFolding(inputL[c]*0.2f, lastOutputL, sampleRate, shape);
            outputR[c] = applyADAAWaveFolding(inputR[c]*0.2f, lastOutputR, sampleRate, shape);

            lastOutputL = outputL[c];
            lastOutputR = outputR[c];

            outputL[c] *= 5.0f;
            outputR[c] *= 5.0f;

            outputL[c] -= symmetry;
            outputR[c] -= symmetry;

            // Undo compression after wavefolding:
            if (compress > 0.01f){ //double check for div zero
                outputL[c] = (outputL[c] - outputL[c] * (1 - compress)) / (compress * 5.0f / filteredEnvelopeL[c]);
                outputR[c] = (outputR[c] - outputR[c] * (1 - compress)) / (compress * 5.0f / filteredEnvelopeR[c]);
            }

            if (applyFilters){
                outputL[c] = hpfL[c].process(outputL[c]);
                outputR[c] = hpfR[c].process(outputR[c]);
            }

            if (isSupersamplingEnabled) {
                outputL[c] = shaperL[c].process(outputL[c]);
                outputR[c] = shaperR[c].process(outputR[c]);
            }

            outputL[c] = clamp(outputL[c], -10.0f, 10.0f);
            outputR[c] = clamp(outputR[c], -10.0f, 10.0f);

            outputs[AUDIO_L_OUTPUT].setVoltage(outputL[c], c);
            outputs[AUDIO_R_OUTPUT].setVoltage(outputR[c], c);

        }//end channels


        //Wave display
        oscPhase += increment_factor;
        if (oscPhase >= 2.0f){ oscPhase =0.0f;}

        // Store the current sample into the circular buffer
        tempBufferPhase += increment_factor;
        if (tempBufferPhase >= 0.5f){ tempBufferPhase =0.0f;}
        
        tempBuffer[tempBufferIndex] = outputL[0];
        tempBufferIndex = static_cast<int>(2*tempBufferPhase * 512); // Wrap index

        if (oscPhase >=1.0f) {
            if (zero_tracking >=0 && prevOscVal <=0){
                oscPhase = 0.5f; // reset oscilloscope display at rising crossing point
                int waveBufferIndex = 0;
                // Copy the last 512 samples from the circular buffer to the first half of waveBuffers[0]
                for (int i = 0; i < 512; i++) {
                    int circularIndex = (tempBufferIndex + i) % 512; // Read from circular buffer
                    waveBuffers[0][waveBufferIndex++] = tempBuffer[circularIndex] * 0.5f;
                }
            }
        }
        prevOscVal = zero_tracking;
        if (oscPhase < 1.0f){
            int sampleIndex = static_cast<int>(oscPhase * 1024);
            if (sampleIndex < 0) sampleIndex = 0;
            else if (sampleIndex > 1023) sampleIndex = 1023;
            waveBuffers[0][sampleIndex] = outputL[0]*0.5f;
        }

        //Function display
        funcPhase += increment_factor;
        if (funcPhase >= 1.0f){ funcPhase =0.0f;}
        //Draw the wavefolding function
        float functionVal = 0.0f;
        float functionX = funcPhase*20.0f - 10.0f;
        if (shape_top == 0.0f) {
            // If shape is 0, return logistic function
            functionVal = scaledLogistic(functionX);
        }
        else if (shape_top <= 1.0f) {
            // Morph between logistic and sin(x)
            functionVal =  (scaledLogistic(functionX) * (1.f-shape_top)) + polySin(functionX)*shape_top;
        }
        else if (shape_top <= 2.0f) {
            // Morph between sin(x) and sin(x^n)
            float powerVal = 0.5f * (shape_top - 1.0f) + 1.0f;
            float powerInput = sgn(functionX)*pow(fabs(functionX), powerVal);
            float morphShape = shape_top-1.f;

            functionVal =  (polySin(functionX) * (1.f-morphShape)) + (polySin(powerInput) * morphShape);
        }
        else {
            // Morph between sin(x^2) and logistic
            float powerInput = sgn(functionX)*pow(fabs(functionX), 1.5f);
            float morphShape = shape_top-2.f;

            functionVal =  (polySin(powerInput) * (1.f-morphShape)) + (scaledLogistic(functionX) * morphShape);
        }
        int funcsampleIndex = static_cast<int>(funcPhase * 1024);
        if (funcsampleIndex < 0) funcsampleIndex = 0;
        else if (funcsampleIndex > 1023) funcsampleIndex = 1023;
        waveBuffers[1][funcsampleIndex] = functionVal*5.0f;

    }

    float applyADAAWaveFolding(float input, float lastInput, float sampleRate, float shape) {
        float delta = input - lastInput;

        if (fabs(delta) > 1e-6) {
            if (shape == 0.0f) {
                // Logistic function
                return (logisticAntiderivative(input) - logisticAntiderivative(lastInput)) / delta;
            }
            else if (shape <= 1.0f) {
                // Morph between logistic and sin(x) for 0 < shape <= 1
                return ( (logisticAntiderivative(input)*(1.f-shape) - polyCos(input)*shape) -
                         (logisticAntiderivative(lastInput)*(1.f-shape) - polyCos(lastInput)*shape) ) / delta;
            }
            else if (shape <= 2.0f) {
                // Morph between sin(x) and sin(x^n) for 1 < shape <= 2
                float powerVal = 0.5f * (shape - 1.0f) + 1.0f;
                float powerInput = sgn(input)*pow(fabs(input), powerVal);
                float powerLastInput = sgn(lastInput)*pow(fabs(lastInput), powerVal);
                float morphShape = shape-1.f;

                return (  (-polyCos(input) * (1.f-morphShape)  - polyCos(powerInput) * morphShape ) -
                          (-polyCos(lastInput) * (1.f-morphShape) - polyCos(powerLastInput) * morphShape ) ) / delta;
            }
            else {
                // Morph between sin(x^n) and logistic for 2 < shape <= 3
                float powerInput = sgn(input)*pow(fabs(input), 1.5f);
                float powerLastInput = sgn(lastInput)*pow(fabs(lastInput), 1.5f);
                float morphShape = shape-2.f;

                return ( (-polyCos(powerInput) * (1.f-morphShape) + logisticAntiderivative(input) * morphShape) -
                         (-polyCos(powerLastInput) * (1.f-morphShape) + logisticAntiderivative(lastInput) * morphShape) ) / delta;
            }
        }
        else {
            if (shape == 0.0f) {
                // If shape is 0, return logistic function
                return scaledLogistic(input);
            }
            else if (shape <= 1.0f) {
                // Morph between logistic and sin(x)
                return (scaledLogistic(input) * (1.f-shape)) + polySin(input)*shape;
            }
            else if (shape <= 2.0f) {
                // Morph between sin(x) and sin(x^n)
                float powerVal = 0.5f * (shape - 1.0f) + 1.0f;
                float powerInput = sgn(input)*pow(fabs(input), powerVal);
                float morphShape = shape-1.f;

                return (polySin(input) * (1.f-morphShape)) + (polySin(powerInput) * morphShape);
            }
            else {
                // Morph between sin(x^2) and logistic
                float powerInput = sgn(input)*pow(fabs(input), 1.5f);
                float morphShape = shape-2.f;

                return (polySin(powerInput) * (1.f-morphShape)) + (scaledLogistic(input) * morphShape);
            }
        }
    }

    float scaledLogistic(float x, float k = 2.0f) {
        return 2.0f / (1.0f + exp(-k * x)) - 1.0f;
    }

    float logisticAntiderivative(float x, float k = 2.0f) {
        return (2.0f / k) * log(1.0f + exp(k * x)) - x;
    }

};

struct TatamiWidget : ModuleWidget {

    struct WaveDisplay : TransparentWidget {
        Tatami* module;
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

                drawWaveform(args, module->waveBuffers[1], nvgRGBAf(0.3, 0.3, 0.3, 0.8));
                drawWaveform(args, module->waveBuffers[0], nvgRGBAf(0, 0.7, 1, 0.9));
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

    TatamiWidget(Tatami* module) {
        setModule(module);

        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Tatami.svg"),
            asset::plugin(pluginInstance, "res/Tatami-dark.svg")
        ));

        // Screws
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(19.242, 69.353)), module, Tatami::SHAPE_INPUT));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(27.918, 69.353)), module, Tatami::SHAPE_ATT_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(56.926, 69.353)), module, Tatami::SHAPE_PARAM));

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(19.242, 84.386)), module, Tatami::COMPRESS_INPUT));     
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(27.918, 84.386)), module, Tatami::COMPRESS_ATT_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(56.926, 84.386)), module, Tatami::COMPRESS_PARAM));

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(19.242,  99.62)), module, Tatami::SYMMETRY_INPUT));     
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(27.918,  99.62)), module, Tatami::SYMMETRY_ATT_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(56.926, 99.62)), module, Tatami::SYMMETRY_PARAM));
     
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(56.926, 114.25)), module, Tatami::DENSITY_PARAM1));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(27.918, 114.25)), module, Tatami::DENSITY_ATT_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(68.752, 114.25)), module, Tatami::DENSITY_PARAM2));

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(6.815, 114.252)), module, Tatami::DENSITY_INPUT1));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(19.242, 114.252)), module, Tatami::DENSITY_INPUT2));

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(6.815, 57.326)), module, Tatami::AUDIO_L_INPUT));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(6.815, 70.756)), module, Tatami::AUDIO_R_INPUT));

        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(69.152, 57.326)), module, Tatami::AUDIO_L_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(69.152, 70.756)), module, Tatami::AUDIO_R_OUTPUT));

        // Create and add the WaveDisplay
        WaveDisplay* waveDisplay = createWidget<WaveDisplay>(mm2px(Vec(7.981, 12.961))); // Positioning
        waveDisplay->box.size = mm2px(Vec(29.939*2, 32.608)); // Size of the display widget
        waveDisplay->module = module;
        addChild(waveDisplay);
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);

        Tatami* TatamiModule = dynamic_cast<Tatami*>(module);
        assert(TatamiModule); // Ensure the cast succeeds

        // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());

        // HPF menu item
        struct FilterMenuItem : MenuItem {
            Tatami* TatamiModule;
            void onAction(const event::Action& e) override {
                // Toggle the "Apply DC Blocking Filter" mode
                TatamiModule->applyFilters = !TatamiModule->applyFilters;
            }
            void step() override {
                // Update the display to show a checkmark when the mode is active
                rightText = TatamiModule->applyFilters ? "✔" : "";
                MenuItem::step();
            }
        };

        FilterMenuItem* filterItem = new FilterMenuItem();
        filterItem->text = "Apply DC Blocking Filter";
        filterItem->TatamiModule = TatamiModule;
        menu->addChild(filterItem);

        // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());

        // 8x Supersampling menu item
        struct SupersamplingMenuItem : MenuItem {
            Tatami* TatamiModule;

            void onAction(const event::Action& e) override {
                // Toggle the "8x Supersampling" mode
                TatamiModule->isSupersamplingEnabled = !TatamiModule->isSupersamplingEnabled;
            }

            void step() override {
                // Update the display to show a checkmark when the mode is active
                rightText = TatamiModule->isSupersamplingEnabled ? "✔" : "";
                MenuItem::step();
            }
        };

        // Create and configure the menu item
        SupersamplingMenuItem* supersamplingItem = new SupersamplingMenuItem();
        supersamplingItem->text = "Enable 8x Supersampling";
        supersamplingItem->TatamiModule = TatamiModule;

        // Add the new item to the menu
        menu->addChild(supersamplingItem);

    }
};

Model* modelTatami = createModel<Tatami, TatamiWidget>("Tatami");