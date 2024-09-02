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
    CircularBuffer<float, 1024> waveBuffers[4];
    
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
        configParam(RATE_KNOB, -3.0f, 3.0f, 0.0f, "V/Oct offset"); // 
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

        for (int c = 0; c < numChannels; c++) {
        
            float deltaTime = args.sampleTime; 
    
            //PROCESS INPUTS
    
            // Calculate target phase based on Node knob
            float fm = 0.0f;
            if (inputs[FM_INPUT].isConnected()) {
                fm += inputs[FM_INPUT].getVoltage(c)*0.2f*params[FM_ATT_KNOB].getValue(); 
            }
            fm = clamp(fm, -3.0f, 3.0f);
    
            float multiply = params[MULTIPLY_KNOB].getValue();
            if (inputs[MULTIPLY_INPUT].isConnected()) {
                float multiplyIn = inputs[MULTIPLY_INPUT].getVoltage(c) * params[MULTIPLY_ATT_KNOB].getValue(); 
                if (multiplyIn < 0.0f){
                    if ( (multiplyIn + multiply) < 1.0 ){ 
                        multiply = 1-0.1f*(multiplyIn + multiply);
                    } else {
                        multiply +=multiplyIn;
                    }
                } else {
                    multiply += multiplyIn;
                }
            }    
            multiply = clamp(multiply, 0.000001f, 10.0f);
    
            // Extract the integer part and the fractional part of multiply
            float baseMultiple = int(multiply);
            float remainder = multiply - baseMultiple;
    
            // Apply the non-linear adjustment based on the remainder
            if (remainder < 0.5f) {
                // If the remainder is less than 0.5, enhance its contribution non-linearly
                multiply = baseMultiple + pow(remainder, 5.f);
            } else {
                // If the remainder is 0.5 or greater, non-linearly approach the next integer
                multiply = (baseMultiple + 1) - pow(1.0f - remainder, 5.f);
            }
    
            float rate = params[RATE_KNOB].getValue();
            if (inputs[RATE_INPUT].isConnected()) {
                rate += inputs[RATE_INPUT].getVoltage(c); 
            }    
            rate += fm; //add the FM to the computed rate
            rate = clamp(rate, -4.0f, 4.0f); 
        
            rate = 261.625565 * pow(2.0, rate);
    
            float multi_rate = rate*multiply;
    
            float rotate = params[ROTATE_KNOB].getValue();
            if (inputs[ROTATE_INPUT].isConnected()) {
                rotate += inputs[ROTATE_INPUT].getVoltage(c) * 36.0f * params[ROTATE_ATT_KNOB].getValue(); 
            }    
    
            float spread = params[SPREAD_KNOB].getValue();
            if (inputs[SPREAD_INPUT].isConnected()) {
                spread += inputs[SPREAD_INPUT].getVoltage(c) * 36.0f * params[SPREAD_ATT_KNOB].getValue(); 
            }    
    
            float eat = params[POSITION_KNOB].getValue();
            if (inputs[POSITION_INPUT].isConnected()) {
                eat += inputs[POSITION_INPUT].getVoltage(c) * 36.0f * params[POSITION_ATT_KNOB].getValue(); 
            }    
    
            float feedback = params[FEEDBACK_KNOB].getValue();
            if (inputs[FEEDBACK_INPUT].isConnected()) {
                feedback += inputs[FEEDBACK_INPUT].getVoltage(c) * 0.1f * params[FEEDBACK_ATT_KNOB].getValue(); 
            }    
            feedback = clamp(feedback, -1.0f, 1.0f);
    
            // Calculate target phase based on Node knob
            float NodePosition = params[NODE_KNOB].getValue();
            if (inputs[NODE_INPUT].isConnected()) {
                NodePosition += inputs[NODE_INPUT].getVoltage(c)*params[NODE_ATT_KNOB].getValue(); 
            }
        
            NodePosition += feedback*oscOutput[c][3];
            NodePosition = fmod(NodePosition, 5.0f);
            NodePosition = clamp(NodePosition, 0.0f, 5.0f); 
    
            // Gate/trigger to Phase Reset input
            float PhaseResetInput=0.0f;
     
            bool manualResetPressed = params[RESET_BUTTON].getValue() > 0.0f;
    
            // If the current input is connected, use it and update lastConnectedInputVoltage
            if (inputs[HARD_SYNC_INPUT].isConnected() || manualResetPressed) {
                PhaseResetInput = inputs[HARD_SYNC_INPUT].getVoltage(c) + params[RESET_BUTTON].getValue(); 
                lastConnectedInputVoltage[c] = PhaseResetInput;
            } else {
                lastConnectedInputVoltage[c] = PhaseResetInput;
            }
        
            if (PhaseResetInput < 0.0001f){latch[c]= true; }
            PhaseResetInput = clamp(PhaseResetInput, 0.0f, 10.0f);
    
            // Check if the envelope is rising or falling with hysteresis
            if (risingState[c]) {
                // If it was rising, look for a significant drop before considering it falling
                if (PhaseResetInput < prevPhaseResetInput[c]) {
                    risingState[c] = false; // Now it's falling
                }
            } else {
                // If it was falling, look for a significant rise before considering it rising
                if (PhaseResetInput > prevPhaseResetInput[c]) {
                    risingState[c] = true; // Now it's rising
                }
            }
    
            //if we get a reset pulse then manually set latch and risingState
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
        
        int sampleIndex = static_cast<int>(oscPhase[0][2] * 1024); 
        sampleIndex = clamp(sampleIndex, 0, 1023);
        waveBuffers[0][sampleIndex] = outputs[L_OUTPUT].getVoltage(0);
        waveBuffers[1][sampleIndex] = outputs[R_OUTPUT].getVoltage(0);
                
    }//void process
     
};

struct PolarXYDisplay : TransparentWidget {
    Ouros* module;
    float centerX, centerY;
    float xScale = 2 * M_PI / 1023; 
    float radiusScale; 

    static constexpr float twoPi = 2.0f * M_PI; // Precomputed constant for 2*pi

    void draw(const DrawArgs& args) override {
        // We do not need to clear the drawing area here if it is handled in drawLayer()
        TransparentWidget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (!module) return;

        if (layer == 1) {
            centerX = box.size.x / 2.0f;
            centerY = box.size.y / 2.0f;
            radiusScale = centerY / 5; // Adjust scale factor for radius

            // Clear the area before drawing the waveform
            nvgBeginPath(args.vg);
            nvgRect(args.vg, 0, 0, box.size.x, box.size.y);
            nvgFillColor(args.vg, nvgRGBA(0, 0, 0, 0)); // Transparent background
            nvgFill(args.vg);

            // Draw waveforms
            if (!waveBufferEmpty(module->waveBuffers[0])) {
                drawWaveform(args, module->waveBuffers[0], nvgRGBAf(1, 0.4, 0, 0.8));
            }

            if (!waveBufferEmpty(module->waveBuffers[1])) {
                drawWaveform(args, module->waveBuffers[1], nvgRGBAf(0, 0.4, 1, 0.8));
            }
        }

        TransparentWidget::drawLayer(args, layer);
    }

    bool waveBufferEmpty(const CircularBuffer<float, 1024>& waveBuffer) const {
        // Check if the buffer has meaningful data; in this case, we'll assume
        // that a buffer is "empty" if all values are zero (or some other criteria).
        for (size_t i = 0; i < waveBuffer.size(); i++) {
            if (waveBuffer[i] != 0.0f) {
                return false;
            }
        }
        return true;
    }

    void drawWaveform(const DrawArgs& args, const CircularBuffer<float, 1024>& waveBuffer, NVGcolor color) {
        nvgBeginPath(args.vg);
        bool firstPoint = true;

        for (size_t i = 0; i < waveBuffer.size(); i++) {
            float theta = i * xScale; // Compute angle based on index
            float radius = waveBuffer[i] * radiusScale + centerY; // Adjust radius based on sample value
            Vec pos = polarToCartesian(theta, radius);

            if (firstPoint) {
                nvgMoveTo(args.vg, pos.x, pos.y);
                firstPoint = false;
            } else {
                nvgLineTo(args.vg, pos.x, pos.y);
            }
        }

        // Properly close the path to avoid any unintended lines
        nvgClosePath(args.vg);
        nvgStrokeColor(args.vg, color); // Set the color for the waveform
        nvgStrokeWidth(args.vg, 1.0);
        nvgStroke(args.vg);
    }

    Vec polarToCartesian(float theta, float radius) const {
        // Normalize theta to be between -pi and pi
        theta = fmod(theta + M_PI, twoPi);
        if (theta < 0) theta += twoPi;
        theta -= M_PI;

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