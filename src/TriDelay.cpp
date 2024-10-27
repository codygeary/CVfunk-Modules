////////////////////////////////////////////////////////////
//
//   Tri Delay
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   Three-tap delay effect module
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

struct TriDelay : Module {
    enum ParamIds {
        GLOBAL_DELAY, GLOBAL_DELAY_ATT, TAP_1_DELAY, TAP_2_DELAY, TAP_3_DELAY,
        GLOBAL_PAN, GLOBAL_PAN_ATT, TAP_1_PAN, TAP_2_PAN, TAP_3_PAN,
        GLOBAL_FEEDBACK, GLOBAL_FEEDBACK_ATT, TAP_1_FEEDBACK, TAP_2_FEEDBACK, TAP_3_FEEDBACK, 
        GLOBAL_WETDRY, GLOBAL_WETDRY_ATT,  
        CLEAR_BUFFER_BUTTON, HOLD_BUTTON,   
        NUM_PARAMS
    };
    enum InputIds {
        AUDIO_INPUT_L,
        AUDIO_INPUT_R,
        GLOBAL_DELAY_IN, GLOBAL_PAN_IN, GLOBAL_BP_IN, GLOBAL_BP_WIDTH_IN, GLOBAL_FEEDBACK_IN,
        GLOBAL_WETDRY_IN,
        CLEAR_BUFFER_IN, HOLD_IN,
        NUM_INPUTS
    };
    enum OutputIds {
        AUDIO_OUTPUT_L,
        AUDIO_OUTPUT_R,       
        NUM_OUTPUTS
    };
    enum LightIds {
        NUM_LIGHTS
    };
 
    float sampleRate = APP->engine->getSampleRate();
    int bufferSize = static_cast<size_t>(3.6 * sampleRate);  // max buffer size for 3.6 seconds delay

    float tapDelay[3] = {0.f, 0.f, 0.f};
    float tapPan[3] = {0.f, 0.f, 0.f};
    float lastOutputL[3] = {0.f, 0.f, 0.f};
    float lastOutputR[3] = {0.f, 0.f, 0.f};

    // Delay buffer for stereo (left and right) as dynamic arrays
    std::vector<float> buffer[2];  // Two vectors for left and right audio channels
    int bufferIndex = 0;           // Buffer write position
    
    // For clearing the buffer and holding the buffer
    int clearIndex = 0;   // Tracks which part of the buffer we're clearing
    bool bufferClearing = false;  // Flag to indicate if the buffer is being cleared
    bool holdBuffer = false;  // Flag to hold the buffer for looping
    int clearBatchSize = 64;  // How many samples to clear per process call (tune as needed)

    // Initialize Butterworth filter for oversampling
    SimpleShaper shaperL;  // Instance of the oversampling and shaping processor
    SimpleShaper shaperR;  // Instance of the oversampling and shaping processor
    Filter6PButter butterworthFilter;  // Butterworth filter instance

    // Stereo output accumulator
    float stereoBuffer[2] = {0.f, 0.f};

    // Tap feedback values
    float tapFeedback[3] = {0.f, 0.f, 0.f};

    // Envelope stuff
    float alpha = 0.1f;
    float oscPhase = 0.0f;
  
    float envPeakL = 0.f;
    float envPeakR = 0.f;
    float filteredEnvelopeL = 0.0f;
    float filteredEnvelopeR = 0.0f;
    float filteredEnvelope = 0.0f;

    float envPeakWetL = 0.f;
    float envPeakWetR = 0.f;
    float filteredEnvelopeWetL = 0.0f;
    float filteredEnvelopeWetR = 0.0f;
    float filteredEnvelopeWet = 0.0f;
    float delayLength = 3600.0f;
    
    // Save state to JSON
    json_t* toJson() override {
        json_t* rootJ = Module::toJson();
        json_object_set_new(rootJ, "delayLength", json_real(delayLength));
        return rootJ;
    }

    // Load state from JSON
    void fromJson(json_t* rootJ) override {
        Module::fromJson(rootJ);
        json_t* delayLengthJ = json_object_get(rootJ, "delayLength");
        if (delayLengthJ)
            delayLength = json_real_value(delayLengthJ);
    }  
    
    //For the display
    CircularBuffer<float, 1024> waveBuffers[2];
   
    TriDelay() : Module() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        // Initialize buffer size at startup
        buffer[0].resize(bufferSize, 0.f);
        buffer[1].resize(bufferSize, 0.f);

        // Global Delay
        configParam(GLOBAL_DELAY, 0.f, 1.f, 0.138888f, "Global Delay Time", " msec");
        configParam(GLOBAL_DELAY_ATT, -1.f, 1.f, 0.f, "Global Delay Attenuverter");

        // Global WetDry
        configParam(GLOBAL_WETDRY, 0.f, 100.0f, 50.f, "Wet/Dry", "% Wet");
        configParam(GLOBAL_WETDRY_ATT, -10.f, 10.f, 0.f, "Wet/Dry Attenuverter");

    
        // Per-tap Delay Offsets
        configParam(TAP_1_DELAY, -1.f, 1.f, 0.f, "Tap 1 Delay Offset", " msec");
        configParam(TAP_2_DELAY, -1.f, 1.f, 0.f, "Tap 2 Delay Offset", " msec");
        configParam(TAP_3_DELAY, -1.f, 1.f, 0.f, "Tap 3 Delay Offset", " msec");
    
        // Global Panning
        configParam(GLOBAL_PAN, -1.f, 1.f, 0.f, "Global Pan", " L/R");
        configParam(GLOBAL_PAN_ATT, -1.f, 1.f, 0.f, "Global Pan Attenuverter");
    
        // Per-tap Panning Offsets
        configParam(TAP_1_PAN, -1.f, 1.f, -.5f, "Tap 1 Pan Offset", " L/R");
        configParam(TAP_2_PAN, -1.f, 1.f, 0.f, "Tap 2 Pan Offset", " L/R");
        configParam(TAP_3_PAN, -1.f, 1.f, .5f, "Tap 3 Pan Offset", " L/R");
        
        // Global Feedback
        configParam(GLOBAL_FEEDBACK, 0.f, 100.f, 35.f, "Global Feedback", "%");
        configParam(GLOBAL_FEEDBACK_ATT, -10.f, 10.f, 0.f, "Global Feedback Attenuverter");
    
        // Per-tap Feedback Offsets
        configParam(TAP_1_FEEDBACK, -100.f, 100.f, 0.f, "Tap 1 Feedback Offset", "%");
        configParam(TAP_2_FEEDBACK, -100.f, 100.f, 0.f, "Tap 2 Feedback Offset", "%");
        configParam(TAP_3_FEEDBACK, -100.f, 100.f, 0.f, "Tap 3 Feedback Offset", "%");

        // Buttons
        configParam(CLEAR_BUFFER_BUTTON, 0.0, 1.0, 0.0, "Clear Buffer" );
        configParam(HOLD_BUTTON, 0.0, 1.0, 0.0, "Hold" );
    
        // Inputs: Global CVs for delay, pan, BP, width, feedback
        configInput(GLOBAL_DELAY_IN, "Global Delay CV");
        configInput(GLOBAL_PAN_IN, "Global Pan CV");
        configInput(GLOBAL_FEEDBACK_IN, "Global Feedback CV");
        configInput(GLOBAL_WETDRY_IN, "Wet/Dry CV");
        configInput(CLEAR_BUFFER_IN, "Clear Buffer");
        configInput(HOLD_IN, "Hold");

    }

    // Handle sample rate changes
    void onSampleRateChange() override {
        sampleRate = APP->engine->getSampleRate();
        int newBufferSize = static_cast<size_t>(3.6 * sampleRate); // Resize buffer
        resizeBuffer(newBufferSize);

    }

    // Resize the delay buffer for both channels, preserving old content
    void resizeBuffer(int newBufferSize) {
        // Copy old buffer content to a temporary buffer
        std::vector<float> tempBufferL = buffer[0];
        std::vector<float> tempBufferR = buffer[1];

        // Resize both buffers to the new buffer size
        buffer[0].resize(newBufferSize, 0.f);
        buffer[1].resize(newBufferSize, 0.f);

        // Copy the data from the old buffer into the resized buffer, preserving wrap-around
        for (int i = 0; i < std::min(bufferSize, newBufferSize); ++i) {
            buffer[0][i] = tempBufferL[(bufferIndex + i) % bufferSize];
            buffer[1][i] = tempBufferR[(bufferIndex + i) % bufferSize];
        }

        // Update buffer size to the new value
        bufferSize = newBufferSize;
        // Ensure the bufferIndex doesn't exceed the new bufferSize
        bufferIndex = bufferIndex % bufferSize;
    }
    
    void process(const ProcessArgs& args) override {
        // Determine if the inputs are connected
        bool isConnectedL = inputs[AUDIO_INPUT_L].isConnected();
        bool isConnectedR = inputs[AUDIO_INPUT_R].isConnected();

        if (!isConnectedR && !isConnectedL) { // If nothing is connected do nothing
            return;
        } 
      
        // Initialize input signals
        float inputL = 0.f;
        float inputR = 0.f;
    
        if (!isConnectedR && isConnectedL) { // Left only, normalize L to R
            inputL = inputs[AUDIO_INPUT_L].getVoltage();
            inputR = inputL;
        } 
        else if (!isConnectedL && isConnectedR) { // Right only, normalize R to L
            inputR = inputs[AUDIO_INPUT_R].getVoltage();
            inputL = inputR;
        } 
        else if (isConnectedL && isConnectedR) { // Both channels connected
            inputL = inputs[AUDIO_INPUT_L].getVoltage();
            inputR = inputs[AUDIO_INPUT_R].getVoltage();
        }
    
        paramQuantities[GLOBAL_DELAY]->displayMultiplier = static_cast<int> (delayLength);
        paramQuantities[TAP_1_DELAY]->displayMultiplier = static_cast<int> (delayLength);
        paramQuantities[TAP_2_DELAY]->displayMultiplier = static_cast<int> (delayLength);
        paramQuantities[TAP_3_DELAY]->displayMultiplier = static_cast<int> (delayLength);
//         paramQuantities[GLOBAL_DELAY_ATT]->displayMultiplier = static_cast<int> (delayLength/36);
       
    
        // Global delay (with CV and attenuverter)
        float globalDelay = params[GLOBAL_DELAY].getValue() * 0.001f * delayLength; // Convert to seconds
        globalDelay += inputs[GLOBAL_DELAY_IN].isConnected() ? params[GLOBAL_DELAY_ATT].getValue() * inputs[GLOBAL_DELAY_IN].getVoltage() * 0.001f * (delayLength/36) : 0.f;
    
        // Global panning (with CV and attenuverter)
        float globalPan = params[GLOBAL_PAN].getValue();
        globalPan += inputs[GLOBAL_PAN_IN].isConnected() ? params[GLOBAL_PAN_ATT].getValue() * inputs[GLOBAL_PAN_IN].getVoltage() : 0.f;
    
        // Global feedback (with CV and attenuverter)
        float globalFeedback = params[GLOBAL_FEEDBACK].getValue() * 0.01f;
        globalFeedback += inputs[GLOBAL_FEEDBACK_IN].isConnected() ? params[GLOBAL_FEEDBACK_ATT].getValue() * inputs[GLOBAL_FEEDBACK_IN].getVoltage() * 0.01f : 0.f;
    
        // Wet/Dry Mix (with CV and attenuverter)
        float wetDry = params[GLOBAL_WETDRY].getValue()*0.01f;
        wetDry += inputs[GLOBAL_WETDRY_IN].isConnected() ? params[GLOBAL_WETDRY_ATT].getValue() * inputs[GLOBAL_WETDRY_IN].getVoltage() * 0.01f : 0.f;
        wetDry = clamp(wetDry, 0.f, 1.f); // Ensure wet/dry is within the range [0, 1]
    
        // Compute tap-specific delay times, panning, feedback, and filter settings with offsets
        for (int i = 0; i < 3; i++) {
            tapDelay[i] = clamp(globalDelay + params[TAP_1_DELAY + i].getValue() * 0.001f * delayLength, 0.0001f, (delayLength/1000.f) );
            tapPan[i] = clamp(globalPan + params[TAP_1_PAN + i].getValue(), -1.f, 1.f);
            tapFeedback[i] = clamp(globalFeedback + params[TAP_1_FEEDBACK + i].getValue() * 0.01f, 0.f, 0.99f);
        }

        // Monitor Buttons
        float clearButton = params[CLEAR_BUFFER_BUTTON].getValue(); 
        clearButton += inputs[CLEAR_BUFFER_IN].isConnected() ? inputs[CLEAR_BUFFER_IN].getVoltage() : 0.f;
        float holdButton = params[HOLD_BUTTON].getValue(); 
        holdButton += inputs[HOLD_IN].isConnected() ? inputs[HOLD_IN].getVoltage() : 0.f;

        // Start buffer clearing process if the clear button is pressed
        if (clearButton > 0 && !bufferClearing) {
            bufferClearing = true;  // Set the flag to start clearing
            clearIndex = 0;         // Start clearing from the beginning
        }
        
        // Hold buffer logic for looping
        holdBuffer = (holdButton > 0);
        
        // Incrementally clear the buffer if clearing is in progress
        if (bufferClearing) {
            inputL = 0.f;
            inputR = 0.f;  
            buffer[0][bufferIndex]=0.f;
            buffer[1][bufferIndex]=0.f;
            clearBufferIncrementally();
        }    
        
        // Reset stereo buffer (prepare to accumulate outputs)
        stereoBuffer[0] = 0.f;
        stereoBuffer[1] = 0.f;

        if (holdBuffer){
            inputL = buffer[0][bufferIndex];
            inputR = buffer[1][bufferIndex];
        }
    
        // Process each tap (L and R independently for stereo delay)
        for (int i = 0; i < 3; i++) {
            processTap(i, tapDelay[i], tapFeedback[i], tapPan[i], inputL, inputR);
        }
    
        // Mix Wet/Dry signal for each channel
        float outputL = (1.0f - wetDry) * inputL + wetDry * stereoBuffer[0];  // Dry/Wet mix for left channel
        float outputR = (1.0f - wetDry) * inputR + wetDry * stereoBuffer[1];  // Dry/Wet mix for right channel

        // Use the oversampling shaper for the signal
        float outputValueL = shaperL.process(outputL);
        float outputValueR = shaperR.process(outputR);

        //// For Envelope tracing
        // Calculate scale factor based on the current sample rate
        float scaleFactor = sampleRate / args.sampleRate; 
        // Adjust alpha and decayRate based on sample rate
        alpha = 0.01f / scaleFactor;  // Smoothing factor for envelope
        float decayRate = pow(0.999f, scaleFactor);  // Decay rate adjusted for sample rate
  
        // Simple peak detection using the absolute maximum of the current input
        envPeakL = fmax(envPeakL * decayRate, fabs(inputL) );
        envPeakR = fmax(envPeakR * decayRate, fabs(inputR) );
        envPeakWetL = fmax(envPeakWetL * decayRate, fabs(stereoBuffer[0]) );
        envPeakWetR = fmax(envPeakWetR * decayRate, fabs(stereoBuffer[1]) );

        filteredEnvelopeL = alpha * envPeakL + (1 - alpha) * filteredEnvelopeL;
        filteredEnvelopeR = alpha * envPeakR + (1 - alpha) * filteredEnvelopeR; 
        filteredEnvelopeWetL = alpha * envPeakWetL + (1 - alpha) * filteredEnvelopeWetL;
        filteredEnvelopeWetR = alpha * envPeakWetR + (1 - alpha) * filteredEnvelopeWetR; 

        //Wave display
        float progress = bufferIndex/((delayLength/1000.f)*sampleRate);
        oscPhase = clamp(progress, 0.f, 1.f);
        int sampleIndex = static_cast<int>(oscPhase * 1024); 
        sampleIndex = (sampleIndex) % 1024;
        waveBuffers[0][sampleIndex] = clamp( (filteredEnvelopeL + filteredEnvelopeR) * 0.40f, -10.f, 10.f) + 0.4f;
        waveBuffers[1][sampleIndex] = clamp( (filteredEnvelopeWetL + filteredEnvelopeWetR) * -0.20f, -10.f, 10.f) - 0.4f;
     
        // Output the mixed signal
        outputs[AUDIO_OUTPUT_L].setVoltage(outputValueL);
        outputs[AUDIO_OUTPUT_R].setVoltage(outputValueR);
    
        // Increment buffer index (circular buffer wrap-around)
        bufferIndex = (bufferIndex + 1) % bufferSize;
        
//         bufferSize = static_cast<size_t>(delayLength/1000 * sampleRate);
    }

    void processTap(int tapIndex, float delayTime, float feedback, float pan, float inputL, float inputR) {
        float delaySamples = delayTime * sampleRate;
        int wholeDelaySamples = static_cast<int>(delaySamples);  // Integer part of delay
        float fractionalDelay = delaySamples - wholeDelaySamples; // Fractional part of delay
    
        // Use Lagrange interpolation for fractional delays
        int readIndex0 = (bufferIndex - wholeDelaySamples - 1 + bufferSize) % bufferSize;
        int readIndex1 = (bufferIndex - wholeDelaySamples + bufferSize) % bufferSize;
        int readIndex2 = (bufferIndex - wholeDelaySamples + 1 + bufferSize) % bufferSize;
        int readIndex3 = (bufferIndex - wholeDelaySamples + 2 + bufferSize) % bufferSize;

        float delayedSampleL = lagrangeInterpolate(buffer[0][readIndex0], buffer[0][readIndex1], buffer[0][readIndex2], buffer[0][readIndex3], fractionalDelay);
        float delayedSampleR = lagrangeInterpolate(buffer[1][readIndex0], buffer[1][readIndex1], buffer[1][readIndex2], buffer[1][readIndex3], fractionalDelay);

        float scaledPan = (pan + 1.f) * 0.5f;  
        // Apply equal-power panning
        float panLeft = polyCos(M_PI_2 * scaledPan);  // π/2 * scaledPan ranges from 0 to π/2
        float panRight = polySin(M_PI_2 * scaledPan);

        // Apply ADAA
        float maxHeadRoom = 1.31f * 10.f; //exceeding this number results in strange wavefolding due to the polytanh bad fit beyond this point
        delayedSampleL = clamp(delayedSampleL, -maxHeadRoom, maxHeadRoom);
        delayedSampleR = clamp(delayedSampleR, -maxHeadRoom, maxHeadRoom);
        delayedSampleL = applyADAA(delayedSampleL/10.f, lastOutputL[tapIndex], sampleRate); //max is 2x5V
        delayedSampleR = applyADAA(delayedSampleR/10.f, lastOutputR[tapIndex], sampleRate);
        lastOutputL[tapIndex] = delayedSampleL;
        lastOutputR[tapIndex] = delayedSampleR; 
        delayedSampleL *= 10.f; 
        delayedSampleR *= 10.f; 

        // Apply feedback: mix delayed sample with current input and pan the feedback
        buffer[0][bufferIndex] = clamp(inputL + feedback * (delayedSampleL * panLeft + delayedSampleR * (1.0f - panRight)), -10.f, 10.f);
        buffer[1][bufferIndex] = clamp(inputR + feedback * (delayedSampleL * (1.0f - panLeft) + delayedSampleR * panRight), -10.f, 10.f);
    
        // Accumulate the delayed signals into the stereo output buffer
        stereoBuffer[0] += delayedSampleL;
        stereoBuffer[1] += delayedSampleR;

    }
 
    void clearBufferIncrementally() {
        if (clearIndex < bufferSize) {
            // Clear a small batch of samples in both left and right channels
            for (int i = 0; i < clearBatchSize && clearIndex < bufferSize; ++i) {
                buffer[0][clearIndex] = 0.0f;  // Set values to zero
                buffer[1][clearIndex] = 0.0f;        
                ++clearIndex;  // Move to the next sample
            }
        } else {
            // If the buffer is fully cleared, stop clearing
            bufferClearing = false;
            clearIndex = 0;  // Reset clear index for next time
        }
    }

    // Lagrange interpolation for 4 points
    float lagrangeInterpolate(float y0, float y1, float y2, float y3, float fraction) {
        float L0 = ((fraction - 1.0f) * (fraction - 2.0f) * (fraction - 3.0f)) / -6.0f;
        float L1 = ((fraction) * (fraction - 2.0f) * (fraction - 3.0f)) / 2.0f;
        float L2 = ((fraction) * (fraction - 1.0f) * (fraction - 3.0f)) / -2.0f;
        float L3 = ((fraction) * (fraction - 1.0f) * (fraction - 2.0f)) / 6.0f;
    
        return L0 * y0 + L1 * y1 + L2 * y2 + L3 * y3;
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
};

struct EnvDisplay : TransparentWidget {
    TriDelay* module;
    float centerX, centerY;
    float heightScale; 

    void draw(const DrawArgs& args) override {
        // Draw non-illuminating elements if any
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (!module) return;

        centerX = box.size.x / 2.0f;
        centerY = box.size.y / 2.0f;
        heightScale = centerY / 5; // Calculate based on current center Y

        drawWaveform(args, module->waveBuffers[0], nvgRGBAf(1, 0.4, 0, 0.8));
        drawWaveform(args, module->waveBuffers[1], nvgRGBAf(0, 0.4, 1, 0.8));

        /// Draw Tap Indicators
        for (int i = 0; i < 3; i++) {
            nvgBeginPath(args.vg);
        
            // Set position of the indicator based on delay time
            nvgCircle(args.vg, box.size.x * (module->tapDelay[i] / 3.6f) * ( 3600.f / module->delayLength ), centerY, module->tapFeedback[i] * 8.0f);
        
            // Get the pan value (-1 to 1)
            float pan = module->tapPan[i];
        
            // Interpolate between Orange (1, 0.4, 0) and Blue (0, 0.4, 1)
            float r = (1.0f - (pan + 1.0f) * 0.5f); // From 1 to 0
            float g = 0.4f;                         // Constant green channel (same for both colors)
            float b = ((pan + 1.0f) * 0.5f);        // From 0 to 1
        
            // Set the color for the tap indicator
            nvgFillColor(args.vg, nvgRGBAf(r, g, b, 1.0f)); // Full opacity
        
            // Fill the circle with the computed color
            nvgFill(args.vg);
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

struct TriDelayWidget : ModuleWidget {
    TriDelayWidget(TriDelay* module) {
        setModule(module);
        
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/TriDelay.svg"),
            asset::plugin(pluginInstance, "res/TriDelay-dark.svg")
        ));
    
        // Screws
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    
        // Layout constants
        const Vec knobStartPos = Vec(35, 155);  // Starting position for knobs and inputs
        const float knobSpacingX = 30.f;       // Horizontal spacing between columns of controls
        const float knobSpacingY = 48.f;       // Vertical spacing between rows of controls
    
        // Global Delay Controls (row 1)
        addParam(createParamCentered<RoundBlackKnob>(knobStartPos, module, TriDelay::GLOBAL_DELAY));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(1*knobSpacingX, 0)), module, TriDelay::GLOBAL_DELAY_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(2 * knobSpacingX, 0)), module, TriDelay::GLOBAL_DELAY_IN));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(3.5*knobSpacingX, 0)), module, TriDelay::TAP_1_DELAY));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(4.5*knobSpacingX, 0)), module, TriDelay::TAP_2_DELAY));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(5.5*knobSpacingX, 0)), module, TriDelay::TAP_3_DELAY));
   
        // Global Pan Controls (row 2)
        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec(0, knobSpacingY)), module, TriDelay::GLOBAL_PAN));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(1*knobSpacingX, knobSpacingY)), module, TriDelay::GLOBAL_PAN_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(2 * knobSpacingX, knobSpacingY)), module, TriDelay::GLOBAL_PAN_IN));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(3.5*knobSpacingX, knobSpacingY)), module, TriDelay::TAP_1_PAN));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(4.5*knobSpacingX, knobSpacingY)), module, TriDelay::TAP_2_PAN));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(5.5*knobSpacingX, knobSpacingY)), module, TriDelay::TAP_3_PAN));
    
        // Feedback Controls (row 3)
        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec(0, 2 * knobSpacingY)), module, TriDelay::GLOBAL_FEEDBACK));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(1*knobSpacingX, 2 * knobSpacingY)), module, TriDelay::GLOBAL_FEEDBACK_ATT));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(2 * knobSpacingX, 2 * knobSpacingY)), module, TriDelay::GLOBAL_FEEDBACK_IN));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(3.5*knobSpacingX, 2*knobSpacingY)), module, TriDelay::TAP_1_FEEDBACK));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(4.5*knobSpacingX, 2*knobSpacingY)), module, TriDelay::TAP_2_FEEDBACK));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(5.5*knobSpacingX, 2*knobSpacingY)), module, TriDelay::TAP_3_FEEDBACK));

        // Wet Dry Controls (row 4)
        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec(0, 3 * knobSpacingY)), module, TriDelay::GLOBAL_WETDRY));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(1*knobSpacingX, 3 * knobSpacingY)), module, TriDelay::GLOBAL_WETDRY_ATT));
        addInput(createInputCentered<PJ301MPort>(knobStartPos.plus(Vec(2 * knobSpacingX, 3 * knobSpacingY)), module, TriDelay::GLOBAL_WETDRY_IN));

        addParam(createParamCentered<TL1105>          (knobStartPos.plus(Vec(3.5 * knobSpacingX, 3 * knobSpacingY)), module, TriDelay::CLEAR_BUFFER_BUTTON));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(4.2 * knobSpacingX, 3 * knobSpacingY)), module, TriDelay::CLEAR_BUFFER_IN));
        addParam(createParamCentered<TL1105>          (knobStartPos.plus(Vec(4.9 * knobSpacingX, 3 * knobSpacingY)), module, TriDelay::HOLD_BUTTON));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(5.6 * knobSpacingX, 3 * knobSpacingY)), module, TriDelay::HOLD_IN));
    
        // Audio IO (bottom row)
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(30, 345), module, TriDelay::AUDIO_INPUT_L));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(70, 345), module, TriDelay::AUDIO_INPUT_R));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(160, 345), module, TriDelay::AUDIO_OUTPUT_L));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(200, 345), module, TriDelay::AUDIO_OUTPUT_R));        
        
        // Create and add the Envelopes Display
        EnvDisplay* envDisplay = createWidget<EnvDisplay>(Vec(15,50)); // Positioning
        envDisplay->box.size = Vec(195, 40); // Size of the display widget
        envDisplay->module = module;
        addChild(envDisplay);

    }  
    
    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
    
        TriDelay* module = dynamic_cast<TriDelay*>(this->module);
        assert(module);
    
        menu->addChild(new MenuSeparator());
        menu->addChild(createMenuLabel("Delay Time"));
    
        struct DelayLengthItem : MenuItem {
            TriDelay* module;
            float length;
            void onAction(const event::Action& e) override {
                module->delayLength = length;
                module->bufferSize = static_cast<size_t>(length/1000 * module->sampleRate);

            }
            void step() override {
                rightText = (module->delayLength == length) ? "✔" : "";
                MenuItem::step();
            }
        };
    
        // Define delay options as pairs of label and length
        std::pair<const char*, float> delayOptions[] = {
            { "36 ms", 36.0f },
            { "360 ms", 360.0f },
            { "3600 ms", 3600.0f }
        };
    
        for (const auto& option : delayOptions) {
            DelayLengthItem* item = createMenuItem<DelayLengthItem>(option.first);
            item->module = module;
            item->length = option.second;
            menu->addChild(item);
        }
    }
      
};

Model* modelTriDelay = createModel<TriDelay, TriDelayWidget>("TriDelay");