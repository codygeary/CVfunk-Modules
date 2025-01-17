////////////////////////////////////////////////////////////
//
//   Ouros
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   Stereo oscillator with phase-feedback 
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
using simd::float_4;

const float twoPi = 2.0f * M_PI;
simd::float_4 twoPiSIMD = simd::float_4(twoPi);

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

struct Ouros : Module {

    static inline float linearInterpolation(float a, float b, float fraction) {
        return a + fraction * (b - a);
    }

    enum ParamIds {
        RATE_KNOB,
        NODE_KNOB,
        ROTATE_KNOB,
        SPREAD_KNOB,
        FEEDBACK_KNOB,
        MULTIPLY_KNOB,
        RATE_ATT_KNOB,
        NODE_ATT_KNOB,
        ROTATE_ATT_KNOB,
        SPREAD_ATT_KNOB,
        FEEDBACK_ATT_KNOB,
        FM_ATT_KNOB,
        POSITION_KNOB,
        POSITION_ATT_KNOB,        
        MULTIPLY_ATT_KNOB,
        RESET_BUTTON,
        PRESET,
        NUM_PARAMS
    };
    enum InputIds {
        HARD_SYNC_INPUT,
        RATE_INPUT,
        NODE_INPUT,
        ROTATE_INPUT,
        SPREAD_INPUT,
        FEEDBACK_INPUT,
        FM_INPUT,
        POSITION_INPUT,
        MULTIPLY_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        L_OUTPUT,
        R_OUTPUT,
        NUM_OUTPUTS
    };
    enum LightIds {
        
        NUM_LIGHTS
    };

    // Global variables for each channel
    dsp::PulseGenerator resetPulse[16];
    dsp::SchmittTrigger SyncTrigger[16];
    CircularBuffer<float, 512> waveBuffers[4];
    
    float oscPhase[16][4] = {{0.0f}};
    float prevPhaseResetInput[16] = {0.0f};
    float lastTargetVoltages[16][4] = {{0.f, 0.f, 0.f, 0.f}};
    float place[16][4] = {{0.f, 0.f, 0.f, 0.f}};
    bool risingState[16] = {false};
    bool latch[16] = {true};
    float oscOutput[16][4] = {{0.f, 0.f, 0.f, 0.f}};
    float nextChunk[16][4] = {{0.f, 0.f, 0.f, 0.f}};
    float lastConnectedInputVoltage[16] = {0.0f};
    float SyncInterval[16] = {2};
    float lastoscPhase[16][4] = {{0.0f}};
    float eatValue[16] = {0.0f};

    // Serialization method to save module state
    json_t* dataToJson() override {
        json_t* rootJ = json_object();
    
        // Create a JSON array to store the eatValue array
        json_t* eatValueArrayJ = json_array();
        for (int i = 0; i < 16; i++) {
            json_array_append_new(eatValueArrayJ, json_real(eatValue[i]));
        }
        json_object_set_new(rootJ, "eatValue", eatValueArrayJ);
    
        // Repeat for other arrays if needed, e.g., oscPhase, lastTargetVoltages, etc.
    
        return rootJ;
    }
    
    // Deserialization method to load module state
    void dataFromJson(json_t* rootJ) override {
        // Load the eatValue array
        json_t* eatValueArrayJ = json_object_get(rootJ, "eatValue");
        if (eatValueArrayJ) {
            for (int i = 0; i < 16; i++) {
                json_t* valueJ = json_array_get(eatValueArrayJ, i);
                if (valueJ) {
                    eatValue[i] = json_real_value(valueJ);
                }
            }
        }
    
        // Repeat for other arrays if needed, e.g., oscPhase, lastTargetVoltages, etc.
    
        // Trigger resets the phase
        for (int i = 0; i < 16; i++) {
            resetPulse[i].trigger(1e-4);
        }
    }
        
    void onReset(const ResetEvent& e) override {
        // Reset all parameters
        Module::onReset(e);
  
          params[RATE_KNOB].setValue(0.0f);
          params[NODE_KNOB].setValue(0.0f);
          params[POSITION_KNOB].setValue(0.0f);
          params[ROTATE_KNOB].setValue(0.0f);
          params[SPREAD_KNOB].setValue(0.0f);
          params[FEEDBACK_KNOB].setValue(0.0f);
          params[MULTIPLY_KNOB].setValue(1.0f);
          params[NODE_ATT_KNOB].setValue(0.0f);
          params[ROTATE_ATT_KNOB].setValue(0.0f);
          params[SPREAD_ATT_KNOB].setValue(0.0f);
          params[FEEDBACK_ATT_KNOB].setValue(0.0f);
          params[POSITION_ATT_KNOB].setValue(0.0f);
          params[MULTIPLY_ATT_KNOB].setValue(0.0f);

        // Trigger resets the phase 
        for (int i=0; i<16; i++){
            resetPulse[i].trigger(1e-4);  
        }     
    }

     Ouros() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        // Initialize knob parameters with a reasonable range and default values
        configParam(RATE_KNOB, -4.0f, 4.0f, 0.0f, "V/Oct offset"); // 
        configParam(NODE_KNOB, 0.0f, 5.0f, 0.0f, "Node Distribution"); // 0: Hexagonal, 1: Unison, 2: Bimodal, 3: Trimodal, 4: Unison, 5:Hexagonal
        configParam(POSITION_KNOB, -360.0f, 360.0f, 0.0f, "Feedback Position"); // 

        configParam(ROTATE_KNOB, -360.0f, 360.0f, 0.0f, "Phase Rotation"); // 
        configParam(SPREAD_KNOB, -360.0f, 360.0f, 0.0f, "Stereo Phase Separation"); // 
        configParam(FEEDBACK_KNOB, -1.0f, 1.0f, 0.0f, "Feedback Amount"); // 
        configParam(MULTIPLY_KNOB, 1.0f, 10.0f, 1.0f, "Multiply Feedback Osc"); // 

        configParam(NODE_ATT_KNOB, -1.0f, 1.0f, 0.0f, "Node Attenuverter"); // 
        configParam(ROTATE_ATT_KNOB, -1.0f, 1.0f, 0.0f, "Rotate Attenuverter"); // 
        configParam(SPREAD_ATT_KNOB, -1.0f, 1.0f, 0.0f, "Spread Attenuverter"); // 
        configParam(FEEDBACK_ATT_KNOB, -1.0f, 1.0f, 0.0f, "Feedback Attenuverter"); // 
        configParam(POSITION_ATT_KNOB, -1.0f, 1.0f, 0.0f, "Feedback Position Attenuverter"); // 
        configParam(MULTIPLY_ATT_KNOB, -1.0f, 1.0f, 0.0f, "Multiply Attenuverter"); // 

        configParam(FM_ATT_KNOB, -1.0f, 1.0f, 0.0f, "FM Attenuverter"); // 
        configInput(HARD_SYNC_INPUT, "Sync");
        configParam(RESET_BUTTON, 0.0, 1.0, 0.0, "Reset" );

        configInput(ROTATE_INPUT, "Rotate");
        configInput(SPREAD_INPUT, "Phase Spread");
        configInput(FEEDBACK_INPUT, "Feedback");
        configInput(FM_INPUT, "FM");

        configInput(RATE_INPUT, "V/Oct");
        configInput(NODE_INPUT, "Node Distribution");
        configInput(POSITION_INPUT, "Feedback Position");
        configInput(MULTIPLY_INPUT, "Multiply");

        configOutput(L_OUTPUT, "Orange Oscillator (L)" );
        configOutput(R_OUTPUT, "Blue Oscillator (R)" );

     }

    void process(const ProcessArgs &args) override {    
    
        int numChannels = std::max(inputs[RATE_INPUT].getChannels(), 1);
        outputs[L_OUTPUT].setChannels(numChannels);
        outputs[R_OUTPUT].setChannels(numChannels);
        
        // Check if each input is monophonic
        bool isFMMonophonic = inputs[FM_INPUT].isConnected() && (inputs[FM_INPUT].getChannels() == 1);
        bool isMultiplyMonophonic = inputs[MULTIPLY_INPUT].isConnected() && (inputs[MULTIPLY_INPUT].getChannels() == 1);
        bool isRateMonophonic = inputs[RATE_INPUT].isConnected() && (inputs[RATE_INPUT].getChannels() == 1);
        bool isRotateMonophonic = inputs[ROTATE_INPUT].isConnected() && (inputs[ROTATE_INPUT].getChannels() == 1);
        bool isSpreadMonophonic = inputs[SPREAD_INPUT].isConnected() && (inputs[SPREAD_INPUT].getChannels() == 1);
        bool isEatMonophonic = inputs[POSITION_INPUT].isConnected() && (inputs[POSITION_INPUT].getChannels() == 1);
        bool isFeedbackMonophonic = inputs[FEEDBACK_INPUT].isConnected() && (inputs[FEEDBACK_INPUT].getChannels() == 1);
        bool isNodeMonophonic = inputs[NODE_INPUT].isConnected() && (inputs[NODE_INPUT].getChannels() == 1);
    
        // Get the monophonic input values
        float fmMonoValue = isFMMonophonic ? inputs[FM_INPUT].getVoltage(0) : 0.0f;
        float multiplyMonoValue = isMultiplyMonophonic ? inputs[MULTIPLY_INPUT].getVoltage(0) : 0.0f;
        float rateMonoValue = isRateMonophonic ? inputs[RATE_INPUT].getVoltage(0) : 0.0f;
        float rotateMonoValue = isRotateMonophonic ? inputs[ROTATE_INPUT].getVoltage(0) : 0.0f;
        float spreadMonoValue = isSpreadMonophonic ? inputs[SPREAD_INPUT].getVoltage(0) : 0.0f;
        float eatMonoValue = isEatMonophonic ? inputs[POSITION_INPUT].getVoltage(0) : 0.0f;
        float feedbackMonoValue = isFeedbackMonophonic ? inputs[FEEDBACK_INPUT].getVoltage(0) : 0.0f;
        float nodeMonoValue = isNodeMonophonic ? inputs[NODE_INPUT].getVoltage(0) : 0.0f;
    
        for (int c = 0; c < numChannels; c++) {
            
            float deltaTime = args.sampleTime; 
        
            // Process FM input
            float fm = 0.0f;
            if (inputs[FM_INPUT].isConnected()) {
                fm += isFMMonophonic ? fmMonoValue : inputs[FM_INPUT].getVoltage(c);
                fm *= 0.2f * params[FM_ATT_KNOB].getValue();
            }
            fm = clamp(fm, -3.0f, 3.0f);
    
            // Process Multiply input
            float multiply = params[MULTIPLY_KNOB].getValue();
            if (inputs[MULTIPLY_INPUT].isConnected()) {
                float multiplyIn = isMultiplyMonophonic ? multiplyMonoValue : inputs[MULTIPLY_INPUT].getVoltage(c);
                multiplyIn *= params[MULTIPLY_ATT_KNOB].getValue(); 
                if (multiplyIn < 0.0f) {
                    if ((multiplyIn + multiply) < 1.0) { 
                        multiply = 1 - 0.1f * (multiplyIn + multiply);
                    } else {
                        multiply += multiplyIn;
                    }
                } else {
                    multiply += multiplyIn;
                }
            }    
            multiply = clamp(multiply, 0.000001f, 10.0f);
    
            // Apply non-linear adjustment to Multiply
            float baseMultiple = int(multiply);
            float remainder = multiply - baseMultiple;
            multiply = (remainder < 0.5f) ? baseMultiple + pow(remainder, 5.f)
                                          : (baseMultiple + 1) - pow(1.0f - remainder, 5.f);
    
            // Process Rate input
            float rate = params[RATE_KNOB].getValue();
            if (inputs[RATE_INPUT].isConnected()) {
                rate += isRateMonophonic ? rateMonoValue : inputs[RATE_INPUT].getVoltage(c);
            }    
            rate += fm;
            rate = clamp(rate, -4.0f, 4.0f); 
            rate = 261.625565 * pow(2.0, rate);
    
            float multi_rate = rate * multiply;
    
            // Process Rotate input
            float rotate = params[ROTATE_KNOB].getValue();
            if (inputs[ROTATE_INPUT].isConnected()) {
                rotate += (isRotateMonophonic ? rotateMonoValue : inputs[ROTATE_INPUT].getVoltage(c)) * 36.0f * params[ROTATE_ATT_KNOB].getValue(); 
            }    
    
            // Process Spread input
            float spread = params[SPREAD_KNOB].getValue();
            if (inputs[SPREAD_INPUT].isConnected()) {
                spread += (isSpreadMonophonic ? spreadMonoValue : inputs[SPREAD_INPUT].getVoltage(c)) * 36.0f * params[SPREAD_ATT_KNOB].getValue(); 
            }    
    
            // Process Eat/Position input
            float eat = params[POSITION_KNOB].getValue();
            if (inputs[POSITION_INPUT].isConnected()) {
                eat += (isEatMonophonic ? eatMonoValue : inputs[POSITION_INPUT].getVoltage(c)) * 36.0f * params[POSITION_ATT_KNOB].getValue(); 
            }    
    
            // Process Feedback input
            float feedback = params[FEEDBACK_KNOB].getValue();
            if (inputs[FEEDBACK_INPUT].isConnected()) {
                feedback += (isFeedbackMonophonic ? feedbackMonoValue : inputs[FEEDBACK_INPUT].getVoltage(c)) * 0.1f * params[FEEDBACK_ATT_KNOB].getValue(); 
            }    
            feedback = clamp(feedback, -1.0f, 1.0f);
    
            // Process Node input
            float NodePosition = params[NODE_KNOB].getValue();
            if (inputs[NODE_INPUT].isConnected()) {
                NodePosition += (isNodeMonophonic ? nodeMonoValue : inputs[NODE_INPUT].getVoltage(c)) * params[NODE_ATT_KNOB].getValue(); 
            }
            
            NodePosition += feedback * oscOutput[c][3];
            NodePosition = fmod(NodePosition, 5.0f);
            NodePosition = clamp(NodePosition, 0.0f, 5.0f);
    
            // Phase Reset input logic
            float PhaseResetInput = 0.0f;
            bool manualResetPressed = params[RESET_BUTTON].getValue() > 0.0f;
    
            if (inputs[HARD_SYNC_INPUT].isConnected() || manualResetPressed) {
                PhaseResetInput = inputs[HARD_SYNC_INPUT].getVoltage(c) + params[RESET_BUTTON].getValue(); 
                lastConnectedInputVoltage[c] = PhaseResetInput;
            } else {
                lastConnectedInputVoltage[c] = PhaseResetInput;
            }
    
            if (PhaseResetInput < 0.0001f) { latch[c] = true; }
            PhaseResetInput = clamp(PhaseResetInput, 0.0f, 10.0f);
    
            // Rising/falling state logic
            if (risingState[c]) {
                if (PhaseResetInput < prevPhaseResetInput[c]) {
                    risingState[c] = false;
                }
            } else {
                if (PhaseResetInput > prevPhaseResetInput[c]) {
                    risingState[c] = true;
                }
            }
    
            // Handle reset pulse
            if (resetPulse[c].process(args.sampleTime)) {
                latch[c] = true;
                risingState[c] = true;
            }

            // Initialize SIMD variables
            simd::float_4 phases, places;
            simd::float_4 rateVec(rate), multi_rateVec(multi_rate);
            simd::float_4 twoPiVec(2 * M_PI), fiveVec(5.0f);
            simd::float_4 zeroVec(0.0f), oneVec(1.0f);

            for (int i = 0; i < 4; i++) {
    
                /////////////////////
                // NODE positioning logic
                //
            
                float nodeOne = (rotate+spread/2)/360;
                float nodeTwo = (rotate-spread/2)/360;
                float nodeThree = eat/360;
                float currentNode = 0.0;
                if (i==0){currentNode = nodeOne;}
                if (i==1){currentNode = nodeTwo;}
                if (i==3){currentNode = nodeThree;}
    
                float basePhase = currentNode;  
                float targetPhase = basePhase; 
            
                if (NodePosition < 1.0f) {
                    // Unison
                    targetPhase = linearInterpolation(basePhase, 0.5f, NodePosition);
                } else if (NodePosition < 2.0f) {
                    // Bimodal distribution
                    float bimodalPhase = fmod(currentNode, 2.0f) / 2.0f;
                    float dynamicFactor = -1.0f * (NodePosition - 1.0f) * ((currentNode + 1.0f) / 2.0f);
                    targetPhase = linearInterpolation(0.5f, bimodalPhase * dynamicFactor, NodePosition - 1.0f);
                } else if (NodePosition < 3.0f) {
                    // Trimodal distribution
                    float bimodalPhase = fmod(currentNode, 2.0f) / 2.0f;
                    float dynamicFactor = -1.0f * (NodePosition - 1.0f) * ((currentNode + 1.0f) / 2.0f);
                    float trimodalPhase = fmod(currentNode, 3.0f) / 3.0f;
    
                    float blendFactor = NodePosition - 2.0f; // Gradually changes from 0 to 1 as NodePosition goes from 2.0 to 3.0
                    float adjustedTrimodalPhase = linearInterpolation(bimodalPhase * dynamicFactor, trimodalPhase, blendFactor * 1.0f);
                    targetPhase = adjustedTrimodalPhase;
                } else if (NodePosition < 4.0f) {
                    float trimodalPhase = fmod(currentNode, 3.0f) / 3.0f;
    
                    // Smoothly map back to Unison
                    float blendFactor = NodePosition - 3.0f; // Gradually changes from 0 to 1 as NodePosition goes from 3.0 to 4.0
                    targetPhase = linearInterpolation(trimodalPhase, 0.5f, blendFactor);
                } else {
                    // Map smoothly to the basePhase for 4-5
                    float blendFactor = NodePosition - 4.0f; // Gradually changes from 0 to 1 as NodePosition goes from 4.0 to 5.0
                    targetPhase = linearInterpolation(0.5f, basePhase, blendFactor);
                }   
                
                targetPhase += place[c][i];
            
                if (i == 2) {
                    targetPhase = place[c][i];
                }
    
                targetPhase = fmod(targetPhase, 1.0f);
    
                float phaseDiff = targetPhase - oscPhase[c][i];
                phaseDiff -= roundf(phaseDiff);  // Ensures phaseDiff is in the range -0.5 to 0.5
    
                //Phase returns to the correct spot, rate determined by PhaseGate
                oscPhase[c][i] += phaseDiff*( 0.05f )  ;
    
                if (i==3){
                        // Update the LFO phase based on the rate
                        oscPhase[c][i] += multi_rate * deltaTime ;        
                        place[c][i] += multi_rate * deltaTime;
                    
                        if (oscPhase[c][2]==0){
                            oscPhase[c][3]=0;
                            place[c][3]=0;
                        }
                } else {
                        // Update the LFO phase based on the rate
                        oscPhase[c][i] += rate * deltaTime ;           
                        place[c][i] += rate * deltaTime;
                }
    
                oscPhase[c][i] -= static_cast<int>(oscPhase[c][i]);
    
                if (place[c][i] >= 1.0f) place[c][i] -= 1.0f; // Wrap 
    
                // Reset LFO phase to 0 at the peak of the envelope
                if ((risingState[c] && latch[c]) ) {
                    oscPhase[c][0] = 0.0f;
                    place[c][0] = 0.0f;
                    oscPhase[c][1] = 0.0f;
                    place[c][1] = 0.0f;
                    oscPhase[c][2] = 0.0f;
                    place[c][2] = 0.0f;
                    latch[c]= false;
                    risingState[c]= false;
                    place[c][3] = 0.0f;
                    oscPhase[c][3] = 0.0f;
                } 
 
                 // Store phase values in SIMD vectors
                phases[i] = oscPhase[c][i];
                places[i] = place[c][i];
                              
                prevPhaseResetInput[c]= PhaseResetInput;
            }

            // Compute oscillator shape using SIMD
            simd::float_4 phaseVector = phases * twoPiVec;
            simd::float_4 sinValues = simd::sin(phaseVector);
            simd::float_4 outputValues = clamp(fiveVec * sinValues, -5.0f, 5.0f);
    
            // Store results and output the voltages
            for (int i = 0; i < 4; i++) {
                oscOutput[c][i] = outputValues[i];
                if (i < 2) {
                    outputs[L_OUTPUT + i].setVoltage(oscOutput[c][i], c);
                }
            }

            lastoscPhase[c][2] = oscPhase[c][2];                
            // Handling for wrapping around 0
            for (int i = 0; i < 4; i++) {
                if (oscPhase[c][i] < lastoscPhase[c][i]) { // This means the phase has wrapped
                    lastoscPhase[c][i] = oscPhase[c][i]; // Update the last phase
                }
            }
            
        }
        
        int sampleIndex = static_cast<int>(oscPhase[0][2] * 512); 
        sampleIndex = clamp(sampleIndex, 0, 511);
        waveBuffers[0][sampleIndex] = outputs[L_OUTPUT].getVoltage(0);
        waveBuffers[1][sampleIndex] = outputs[R_OUTPUT].getVoltage(0);
                
    }//void process
     
};

struct PolarXYDisplay : TransparentWidget {
    Ouros* module;
    float centerX, centerY;
    float radiusScale; 
    static constexpr float twoPi = 2.0f * M_PI; // Precomputed constant for 2π

    void drawLayer(const DrawArgs& args, int layer) override {
        if (!module) return;

        if (layer == 1) {
            centerX = box.size.x / 2.0f;
            centerY = box.size.y / 2.0f;
            radiusScale = centerY * 0.8f; // Adjust as needed

            // Clear the area before drawing the waveform
            nvgBeginPath(args.vg);
            nvgRect(args.vg, 0, 0, box.size.x, box.size.y);
            nvgFillColor(args.vg, nvgRGBA(0, 0, 0, 0)); // Transparent background
            nvgFill(args.vg);

            // Draw waveforms
                drawWaveform(args, module->waveBuffers[0], nvgRGBAf(1, 0.4, 0, 0.8));
                drawWaveform(args, module->waveBuffers[1], nvgRGBAf(0, 0.4, 1, 0.8));
        }

        TransparentWidget::drawLayer(args, layer);
    }

    void drawWaveform(const DrawArgs& args, const CircularBuffer<float, 512>& waveBuffer, NVGcolor color) {

        nvgBeginPath(args.vg);
        bool firstPoint = true;

        const int displaySamples = 512; // Adjust for performance
        for (int i = 0; i < displaySamples; i++) {
            size_t bufferIndex = i * (waveBuffer.size() - 1) / (displaySamples - 1);

            float theta = ((float)i / (displaySamples - 1)) * twoPi; // From 0 to 2π

            float amplitude = waveBuffer[bufferIndex] / 5.0f; // Normalize to -1 to +1
            float radius = centerY + amplitude * radiusScale;

            Vec pos = polarToCartesian(theta, radius);

            if (firstPoint) {
                nvgMoveTo(args.vg, pos.x, pos.y);
                firstPoint = false;
            } else {
                nvgLineTo(args.vg, pos.x, pos.y);
            }
        }

        nvgStrokeColor(args.vg, color); // Set the color for the waveform
        nvgStrokeWidth(args.vg, 1.0);
        nvgStroke(args.vg);
    }

    Vec polarToCartesian(float theta, float radius) const {
        float x = centerX + radius * cos(theta);
        float y = centerY + radius * sin(theta);
        return Vec(x, y);
    }
};



struct OurosWidget : ModuleWidget {
    OurosWidget(Ouros* module) {
        setModule(module);

        setPanel(createPanel(
                asset::plugin(pluginInstance, "res/Ouros.svg"),
                asset::plugin(pluginInstance, "res/Ouros-dark.svg")
            ));

        // Add screws or additional design elements as needed
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // Row of knobs at the bottom, with attenuators and CV inputs
        const Vec knobStartPos = Vec(30, 165);
        const float knobSpacing = 50.5f;

        addParam(createParamCentered<TL1105>        (knobStartPos.plus(Vec( 0*knobSpacing, -25  )), module, Ouros::RESET_BUTTON));
        addInput(createInputCentered<ThemedPJ301MPort>    (knobStartPos.plus(Vec( 0*knobSpacing, 0  )), module, Ouros::HARD_SYNC_INPUT));
 
        addParam(createParamCentered<Trimpot>       (knobStartPos.plus(Vec( 0*knobSpacing, 40 )), module, Ouros::FM_ATT_KNOB));
        addInput(createInputCentered<ThemedPJ301MPort>    (knobStartPos.plus(Vec( 0*knobSpacing, 65 )), module, Ouros::FM_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec( 1*knobSpacing, 0  )), module, Ouros::ROTATE_KNOB));
        addParam(createParamCentered<Trimpot>       (knobStartPos.plus(Vec( 1*knobSpacing, 30 )), module, Ouros::ROTATE_ATT_KNOB));
        addInput(createInputCentered<ThemedPJ301MPort>    (knobStartPos.plus(Vec( 1*knobSpacing, 55 )), module, Ouros::ROTATE_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec( 2*knobSpacing, 0  )), module, Ouros::SPREAD_KNOB));
        addParam(createParamCentered<Trimpot>       (knobStartPos.plus(Vec( 2*knobSpacing, 30 )), module, Ouros::SPREAD_ATT_KNOB));
        addInput(createInputCentered<ThemedPJ301MPort>    (knobStartPos.plus(Vec( 2*knobSpacing, 55 )), module, Ouros::SPREAD_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec( 3*knobSpacing, 0  )), module, Ouros::MULTIPLY_KNOB));
        addParam(createParamCentered<Trimpot>       (knobStartPos.plus(Vec( 3*knobSpacing, 30 )), module, Ouros::MULTIPLY_ATT_KNOB));
        addInput(createInputCentered<ThemedPJ301MPort>    (knobStartPos.plus(Vec( 3*knobSpacing, 55 )), module, Ouros::MULTIPLY_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec( 0*knobSpacing, 125 )), module, Ouros::RATE_KNOB));
        addInput(createInputCentered<ThemedPJ301MPort>    (knobStartPos.plus(Vec( 0*knobSpacing, 165 )), module, Ouros::RATE_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec( 1*knobSpacing, 110 )), module, Ouros::FEEDBACK_KNOB));
        addParam(createParamCentered<Trimpot>       (knobStartPos.plus(Vec( 1*knobSpacing, 140 )), module, Ouros::FEEDBACK_ATT_KNOB));
        addInput(createInputCentered<ThemedPJ301MPort>    (knobStartPos.plus(Vec( 1*knobSpacing, 165 )), module, Ouros::FEEDBACK_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec( 2*knobSpacing, 110 )), module, Ouros::POSITION_KNOB));
        addParam(createParamCentered<Trimpot>       (knobStartPos.plus(Vec( 2*knobSpacing, 140 )), module, Ouros::POSITION_ATT_KNOB));
        addInput(createInputCentered<ThemedPJ301MPort>    (knobStartPos.plus(Vec( 2*knobSpacing, 165 )), module, Ouros::POSITION_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec( 3*knobSpacing, 110 )), module, Ouros::NODE_KNOB));
        addParam(createParamCentered<Trimpot>       (knobStartPos.plus(Vec( 3*knobSpacing, 140 )), module, Ouros::NODE_ATT_KNOB));
        addInput(createInputCentered<ThemedPJ301MPort>    (knobStartPos.plus(Vec( 3*knobSpacing, 165 )), module, Ouros::NODE_INPUT));

        addOutput(createOutputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(3*knobSpacing, -102)), module, Ouros::L_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(3*knobSpacing, -72)), module, Ouros::R_OUTPUT));

        // Create and add the PolarXYDisplay
        PolarXYDisplay* polarDisplay = createWidget<PolarXYDisplay>(Vec(56.5,55.5)); // Positioning
        polarDisplay->box.size = Vec(50, 50); // Size of the display widget
        polarDisplay->module = module;
        addChild(polarDisplay);

    }    
};

Model* modelOuros = createModel<Ouros, OurosWidget>("Ouros");