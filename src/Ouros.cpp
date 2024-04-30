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

static const float twoPi = 2.0f * M_PI;

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

    // Initialize global variables
    dsp::PulseGenerator resetPulse;
    dsp::SchmittTrigger SyncTrigger;
    CircularBuffer<float, 1024> waveBuffers[4];

    float oscPhase[4] = {0.0f}; // Current oscillator phase for each channel
    float prevPhaseResetInput = 0.0f; // Previous envelope input, for peak detection
    float calculateTargetPhase(int channel, float NodePosition, float deltaTime, float place);
    void adjustoscPhase(int channel, float targetPhase, float envInfluence, float deltaTime);
    float lastTargetVoltages[4] = {0.f, 0.f, 0.f, 0.f}; // Initialize with default voltages, assuming start at 0V  
    float place[4] = {0.f, 0.f, 0.f, 0.f};
    bool risingState = false; // Initialize all channels as falling initially
    bool latch = true; // Initialize all latches
    float oscOutput[4] = {0.f, 0.f, 0.f, 0.f};
    float nextChunk[4] = {0.f, 0.f, 0.f, 0.f}; //measure next voltage step to subdivide
    int LEDprocessCounter = 0; // Counter to track process cycles
    int SINprocessCounter = 0; // Counter to track process cycles
    float lastConnectedInputVoltage = 0.0f;
    float SyncInterval = 2; //default to 2hz
    float lastoscPhase[4] = {}; // Track the last phase for each LFO channel to detect wraps
    float eatValue = 0.0f;

    // Serialization method to save module state
    json_t* dataToJson() override {
        json_t* rootJ = json_object();

        // Save the state of retriggerEnabled as a boolean
        json_object_set_new(rootJ, "eatValue", json_boolean(eatValue));

        return rootJ;
    }

    // Deserialization method to load module state
    void dataFromJson(json_t* rootJ) override {
        // Load the state of retriggerEnabled
        json_t* eatValueJ = json_object_get(rootJ, "eatValue");
        if (eatValueJ) {
            // Use json_is_true() to check if the JSON value is true; otherwise, set to false
            eatValue = json_is_true(eatValueJ);
        }
         // Trigger resets the phase 
         resetPulse.trigger(1e-4); 
              
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
        resetPulse.trigger(1e-4);       
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
        float deltaTime = args.sampleTime; 

        //PROCESS INPUTS

        // Calculate target phase based on Node knob
        float fm = 0.0f;
        if (inputs[FM_INPUT].isConnected()) {
            fm += inputs[FM_INPUT].getVoltage()*0.2f*params[FM_ATT_KNOB].getValue(); 
        }
        fm = clamp(fm, -3.0f, 3.0f);

        float multiply = params[MULTIPLY_KNOB].getValue();
        if (inputs[MULTIPLY_INPUT].isConnected()) {
            float multiplyIn = inputs[MULTIPLY_INPUT].getVoltage() * params[MULTIPLY_ATT_KNOB].getValue(); 
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
            rate += inputs[RATE_INPUT].getVoltage(); 
        }    
        rate += fm; //add the FM to the computed rate
        rate = clamp(rate, -3.0f, 3.0f); 
    
        rate = 261.625565 * pow(2.0, rate);

        float multi_rate = rate*multiply;

        float rotate = params[ROTATE_KNOB].getValue();
        if (inputs[ROTATE_INPUT].isConnected()) {
            rotate += inputs[ROTATE_INPUT].getVoltage() * 36.0f * params[ROTATE_ATT_KNOB].getValue(); 
        }    

        float spread = params[SPREAD_KNOB].getValue();
        if (inputs[SPREAD_INPUT].isConnected()) {
            spread += inputs[SPREAD_INPUT].getVoltage() * 36.0f * params[SPREAD_ATT_KNOB].getValue(); 
        }    

        float eat = params[POSITION_KNOB].getValue();
        if (inputs[POSITION_INPUT].isConnected()) {
            eat += inputs[POSITION_INPUT].getVoltage() * 36.0f * params[POSITION_ATT_KNOB].getValue(); 
        }    

        float feedback = params[FEEDBACK_KNOB].getValue();
        if (inputs[FEEDBACK_INPUT].isConnected()) {
            feedback += inputs[FEEDBACK_INPUT].getVoltage() * 0.1f * params[FEEDBACK_ATT_KNOB].getValue(); 
        }    
        feedback = clamp(feedback, -1.0f, 1.0f);

        // Calculate target phase based on Node knob
        float NodePosition = params[NODE_KNOB].getValue();
        if (inputs[NODE_INPUT].isConnected()) {
            NodePosition += inputs[NODE_INPUT].getVoltage()*params[NODE_ATT_KNOB].getValue(); 
        }
    
        NodePosition += feedback*oscOutput[3];
        NodePosition = fmod(NodePosition, 5.0f);
        NodePosition = clamp(NodePosition, 0.0f, 5.0f); 

        // Gate/trigger to Phase Reset input
        float PhaseResetInput=0.0f;
 
        bool manualResetPressed = params[RESET_BUTTON].getValue() > 0.0f;

        // If the current input is connected, use it and update lastConnectedInputVoltage
        if (inputs[HARD_SYNC_INPUT].isConnected() || manualResetPressed) {
            PhaseResetInput = inputs[HARD_SYNC_INPUT].getVoltage() + params[RESET_BUTTON].getValue(); 
            lastConnectedInputVoltage = PhaseResetInput;
        } else {
            lastConnectedInputVoltage = PhaseResetInput;
        }
    
        if (PhaseResetInput < 0.0001f){latch= true; }
        PhaseResetInput = clamp(PhaseResetInput, 0.0f, 10.0f);

        // Check if the envelope is rising or falling with hysteresis
        if (risingState) {
            // If it was rising, look for a significant drop before considering it falling
            if (PhaseResetInput < prevPhaseResetInput) {
                risingState = false; // Now it's falling
            }
        } else {
            // If it was falling, look for a significant rise before considering it rising
            if (PhaseResetInput > prevPhaseResetInput) {
                risingState = true; // Now it's rising
            }
        }

        //if we get a reset pulse then manually set latch and risingState
        if (resetPulse.process(args.sampleTime)) {
            latch = true;
            risingState = true;
        }

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
            
            targetPhase += place[i];
        
			if (i == 2) {
				targetPhase = place[i];
			}

			targetPhase = fmod(targetPhase, 1.0f);
			if (targetPhase < 0.0f) targetPhase += 1.0f;

			float phaseDiff = targetPhase - oscPhase[i];
			phaseDiff -= roundf(phaseDiff);  // Ensures phaseDiff is in the range -0.5 to 0.5

            //Phase returns to the correct spot, rate determined by PhaseGate
            oscPhase[i] += phaseDiff*( 0.05f )  ;

            if (i==3){
                    // Update the LFO phase based on the rate
                    oscPhase[i] += multi_rate * deltaTime ;        
                    place[i] += multi_rate * deltaTime;
                
                    if (oscPhase[2]==0){
                        oscPhase[3]=0;
                        place[3]=0;
                    }
            } else {
                    // Update the LFO phase based on the rate
                    oscPhase[i] += rate * deltaTime ;           
                    place[i] += rate * deltaTime;
            }

            oscPhase[i] -= (int)oscPhase[i];

            if (place[i] >= 1.0f) place[i] -= 1.0f; // Wrap 

            // Reset LFO phase to 0 at the peak of the envelope
            if ((risingState && latch) ) {
                oscPhase[0] = 0.0f;
                place[0] = 0.0f;
                oscPhase[1] = 0.0f;
                place[1] = 0.0f;
                oscPhase[2] = 0.0f;
                place[2] = 0.0f;
                latch= false;
                risingState= false;
                place[3] = 0.0f;
                oscPhase[3] = 0.0f;
            } 

            ////////////
            //COMPUTE the Oscillator Shape
            oscOutput[i] = clamp(5.0f * sinf(twoPi * oscPhase[i]), -5.0f, 5.0f);

            if (i<2){
                //Output Voltage
                outputs[L_OUTPUT + i].setVoltage(oscOutput[i]);
            }    
                          
            prevPhaseResetInput = PhaseResetInput;
        }
    
        int sampleIndex = static_cast<int>(oscPhase[2] * 1024); 
        if (sampleIndex < 0) sampleIndex = 0;
        else if (sampleIndex > 1023) sampleIndex = 1023;
        waveBuffers[0][sampleIndex] = outputs[L_OUTPUT].getVoltage();
        waveBuffers[1][sampleIndex] = outputs[R_OUTPUT].getVoltage();
        lastoscPhase[2] = oscPhase[2];
    
        // Handling for wrapping around 0
        for (int i = 0; i < 4; i++) {
            if (oscPhase[i] < lastoscPhase[i]) { // This means the phase has wrapped
                lastoscPhase[i] = oscPhase[i]; // Update the last phase
            }
        }
       
    }//void process
     
};

struct PolarXYDisplay : TransparentWidget {
    Ouros* module;
    float centerX, centerY;

    Vec previousL = Vec(0.0f, 0.0f);
    Vec previousR = Vec(0.0f, 0.0f);
     
    void draw(const DrawArgs& args) override {
        if (!module) return;

        // Coordinates for the circle's center
        centerX = box.size.x / 2.0f;
        centerY = box.size.y / 2.0f;

        float xScale = 2 * M_PI / 1023; // Scale to fit the circular path
        float radiusScale = centerY / 5; 

        // Draw waveform from waveBuffers[0]
        nvgBeginPath(args.vg);
        for (size_t i = 0; i < 1024; i++) {
            float theta = i * xScale; // Angle based on index
            float radius = module->waveBuffers[0][i] * radiusScale + centerY; // Adjust radius based on sample value
            Vec pos = polarToCartesian(theta, radius);

            if (i == 0) nvgMoveTo(args.vg, pos.x, pos.y);
            else nvgLineTo(args.vg, pos.x, pos.y);
        }

        nvgStrokeColor(args.vg, nvgRGBAf(1, .4, 0, 0.8)); // Drawing color for waveform 1
        nvgStrokeWidth(args.vg, 1.0);
        nvgStroke(args.vg);

        // Draw waveform from waveBuffers[1]
        nvgBeginPath(args.vg);
        for (size_t i = 0; i < 1024; i++) {
            float theta = i * xScale; // Angle based on index
            float radius = module->waveBuffers[1][i] * radiusScale + centerY; // Adjust radius based on sample value
            Vec pos = polarToCartesian(theta, radius);

            if (i == 0) nvgMoveTo(args.vg, pos.x, pos.y);
            else nvgLineTo(args.vg, pos.x, pos.y);
        }

        nvgStrokeColor(args.vg, nvgRGBAf(0, .4, 1, 0.8)); // Drawing color for waveform 2
        nvgStrokeWidth(args.vg, 1.0);
        nvgStroke(args.vg);
    }

    Vec polarToCartesian(float theta, float radius) {
	
		// Wrap theta between -pi and pi
		theta = fmod(theta + M_PI, twoPi);
		if (theta < 0) theta += twoPi;
		theta -= M_PI;
        
        float x = centerX + radius * cos(theta);
        float y = centerY + radius * sin(theta);
        return Vec(x, y);
    }

    void drawLine(const DrawArgs& args, Vec fromPos, Vec toPos, NVGcolor color) {
        nvgStrokeColor(args.vg, color);
        nvgBeginPath(args.vg);
        nvgMoveTo(args.vg, fromPos.x, fromPos.y);
        nvgLineTo(args.vg, toPos.x, toPos.y);
        nvgStrokeWidth(args.vg, 1.5);
        nvgStroke(args.vg);
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