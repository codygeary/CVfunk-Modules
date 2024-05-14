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
    CircularBuffer() : buffer{T{}}, index{0} {}

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

constexpr float twoPi = 2.0f * M_PI;

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
    float lastTargetVoltages[4] = {0.f, 0.f, 0.f, 0.f}; // Initialize with default voltages, assuming start at 0V  
    float place[4] = {0.f, 0.f, 0.f, 0.f};
    bool risingState = false; // Initialize all channels as falling initially
    bool latch = true; // Initialize all latches
    float oscOutput[4] = {0.f, 0.f, 0.f, 0.f};
    float nextChunk[4] = {0.f, 0.f, 0.f, 0.f}; // Measure next voltage step to subdivide
    int LEDprocessCounter = 0; // Counter to track process cycles
    int SINprocessCounter = 0; // Counter to track process cycles
    float lastConnectedInputVoltage = 0.0f;
    float SyncInterval = 2; // Default to 2Hz
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

        // Precompute values that remain constant during the loop
        float fm = inputs[FM_INPUT].isConnected() ? clamp(inputs[FM_INPUT].getVoltage() * 0.2f * params[FM_ATT_KNOB].getValue(), -3.0f, 3.0f) : 0.0f;

        float multiply = params[MULTIPLY_KNOB].getValue();
        if (inputs[MULTIPLY_INPUT].isConnected()) {
            float multiplyIn = inputs[MULTIPLY_INPUT].getVoltage() * params[MULTIPLY_ATT_KNOB].getValue(); 
            multiply += (multiplyIn < 0.0f && (multiplyIn + multiply) < 1.0f) ? -0.1f * (multiplyIn + multiply) : multiplyIn;
        }    
        multiply = clamp(multiply, 0.000001f, 10.0f);

        float baseMultiple = static_cast<int>(multiply);
        float remainder = multiply - baseMultiple;
        multiply = (remainder < 0.5f) ? baseMultiple + pow(remainder, 5.f) : (baseMultiple + 1) - pow(1.0f - remainder, 5.f);

        float rate = clamp(params[RATE_KNOB].getValue() + (inputs[RATE_INPUT].isConnected() ? inputs[RATE_INPUT].getVoltage() : 0) + fm, -4.0f, 4.0f); 
        rate = 261.625565 * pow(2.0, rate);
        float multi_rate = rate * multiply;

        float rotate = params[ROTATE_KNOB].getValue() + (inputs[ROTATE_INPUT].isConnected() ? inputs[ROTATE_INPUT].getVoltage() * 36.0f * params[ROTATE_ATT_KNOB].getValue() : 0);    
        float spread = params[SPREAD_KNOB].getValue() + (inputs[SPREAD_INPUT].isConnected() ? inputs[SPREAD_INPUT].getVoltage() * 36.0f * params[SPREAD_ATT_KNOB].getValue() : 0);    
        float eat = params[POSITION_KNOB].getValue() + (inputs[POSITION_INPUT].isConnected() ? inputs[POSITION_INPUT].getVoltage() * 36.0f * params[POSITION_ATT_KNOB].getValue() : 0);    
        float feedback = clamp(params[FEEDBACK_KNOB].getValue() + (inputs[FEEDBACK_INPUT].isConnected() ? inputs[FEEDBACK_INPUT].getVoltage() * 0.1f * params[FEEDBACK_ATT_KNOB].getValue() : 0), -1.0f, 1.0f);

        float NodePosition = clamp(fmod(params[NODE_KNOB].getValue() + (inputs[NODE_INPUT].isConnected() ? inputs[NODE_INPUT].getVoltage() * params[NODE_ATT_KNOB].getValue() : 0) + feedback * oscOutput[3], 5.0f), 0.0f, 5.0f); 

        float PhaseResetInput = clamp((inputs[HARD_SYNC_INPUT].isConnected() ? inputs[HARD_SYNC_INPUT].getVoltage() : 0.0f) + params[RESET_BUTTON].getValue(), 0.0f, 10.0f);
        if (PhaseResetInput < 0.0001f) latch = true;
        risingState = (risingState) ? (PhaseResetInput < prevPhaseResetInput ? false : true) : (PhaseResetInput > prevPhaseResetInput ? true : false);

        if (resetPulse.process(args.sampleTime)) {
            latch = true;
            risingState = true;
        }

        for (int i = 0; i < 4; i++) {
            float nodeOne = (rotate + spread / 2) / 360;
            float nodeTwo = (rotate - spread / 2) / 360;
            float nodeThree = eat / 360;
            float currentNode = (i == 0) ? nodeOne : (i == 1) ? nodeTwo : nodeThree;

            float basePhase = currentNode;  
            float targetPhase = basePhase; 

            if (NodePosition < 1.0f) {
                targetPhase = linearInterpolation(basePhase, 0.5f, NodePosition);
            } else if (NodePosition < 2.0f) {
                float bimodalPhase = fmod(currentNode, 2.0f) / 2.0f;
                float dynamicFactor = -1.0f * (NodePosition - 1.0f) * ((currentNode + 1.0f) / 2.0f);
                targetPhase = linearInterpolation(0.5f, bimodalPhase * dynamicFactor, NodePosition - 1.0f);
            } else if (NodePosition < 3.0f) {
                float bimodalPhase = fmod(currentNode, 2.0f) / 2.0f;
                float dynamicFactor = -1.0f * (NodePosition - 1.0f) * ((currentNode + 1.0f) / 2.0f);
                float trimodalPhase = fmod(currentNode, 3.0f) / 3.0f;

                float blendFactor = NodePosition - 2.0f; 
                float adjustedTrimodalPhase = linearInterpolation(bimodalPhase * dynamicFactor, trimodalPhase, blendFactor * 1.0f);
                targetPhase = adjustedTrimodalPhase;
            } else if (NodePosition < 4.0f) {
                float trimodalPhase = fmod(currentNode, 3.0f) / 3.0f;

                float blendFactor = NodePosition - 3.0f;
                targetPhase = linearInterpolation(trimodalPhase, 0.5f, blendFactor);
            } else {
                float blendFactor = NodePosition - 4.0f;
                targetPhase = linearInterpolation(0.5f, basePhase, blendFactor);
            }   

            targetPhase += place[i];

            if (i == 2) {
                targetPhase = place[i];
            }

            targetPhase = fmod(targetPhase, 1.0f);
            float phaseDiff = targetPhase - oscPhase[i];
            phaseDiff -= roundf(phaseDiff);  // Ensures phaseDiff is in the range -0.5 to 0.5

            oscPhase[i] += phaseDiff * 0.05f;

            if (i == 3) {
                oscPhase[i] += multi_rate * deltaTime;        
                place[i] += multi_rate * deltaTime;

                if (oscPhase[2] == 0) {
                    oscPhase[3] = 0;
                    place[3] = 0;
                }
            } else {
                oscPhase[i] += rate * deltaTime;           
                place[i] += rate * deltaTime;
            }

            oscPhase[i] -= static_cast<int>(oscPhase[i]);
            if (place[i] >= 1.0f) place[i] -= 1.0f;

            if (risingState && latch) {
                for (int j = 0; j < 4; j++) {
                    oscPhase[j] = 0.0f;
                    place[j] = 0.0f;
                }
                latch = false;
                risingState = false;
            }

            oscOutput[i] = clamp(5.0f * sinf(twoPi * oscPhase[i]), -5.0f, 5.0f);

            if (i < 2) {
                outputs[L_OUTPUT + i].setVoltage(oscOutput[i]);
            }

            prevPhaseResetInput = PhaseResetInput;
        }

        int sampleIndex = clamp(static_cast<int>(oscPhase[2] * 1024), 0, 1023);
        waveBuffers[0][sampleIndex] = outputs[L_OUTPUT].getVoltage();
        waveBuffers[1][sampleIndex] = outputs[R_OUTPUT].getVoltage();
        lastoscPhase[2] = oscPhase[2];

        for (int i = 0; i < 4; i++) {
            if (oscPhase[i] < lastoscPhase[i]) {
                lastoscPhase[i] = oscPhase[i];
            }
        }
    }

};

struct PolarXYDisplay : TransparentWidget {
    Ouros* module;
    float centerX, centerY;
    float xScale = 2 * M_PI / 1023; 
    float radiusScale; 

    static constexpr float twoPi = 2.0f * M_PI; // Precomputed constant for 2*pi

    void draw(const DrawArgs& args) override {
        // Draw non-illuminating elements if any
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (!module) return;

        if (layer == 1) {
            centerX = box.size.x / 2.0f;
            centerY = box.size.y / 2.0f;
            radiusScale = centerY / 5; // Calculate based on current center Y

            drawWaveform(args, module->waveBuffers[0], nvgRGBAf(1, 0.4, 0, 0.8));
            drawWaveform(args, module->waveBuffers[1], nvgRGBAf(0, 0.4, 1, 0.8));
        }

        TransparentWidget::drawLayer(args, layer);
    }

    void drawWaveform(const DrawArgs& args, const CircularBuffer<float, 1024>& waveBuffer, NVGcolor color) {
        nvgBeginPath(args.vg);
        for (size_t i = 0; i < 1024; i++) {
            float theta = i * xScale; // Compute angle based on index
            float radius = waveBuffer[i] * radiusScale + centerY; // Adjust radius based on sample value
            Vec pos = polarToCartesian(theta, radius);

            if (i == 0) nvgMoveTo(args.vg, pos.x, pos.y);
            else nvgLineTo(args.vg, pos.x, pos.y);
        }
        nvgStrokeColor(args.vg, color); // Set the color for the waveform
        nvgStrokeWidth(args.vg, 1.0);
        nvgStroke(args.vg);
    }

    Vec polarToCartesian(float theta, float radius) {
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

        addParam(createParamCentered<TL1105>(knobStartPos.plus(Vec(0 * knobSpacing, -25)), module, Ouros::RESET_BUTTON));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(0 * knobSpacing, 0)), module, Ouros::HARD_SYNC_INPUT));
 
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(0 * knobSpacing, 40)), module, Ouros::FM_ATT_KNOB));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(0 * knobSpacing, 65)), module, Ouros::FM_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec(1 * knobSpacing, 0)), module, Ouros::ROTATE_KNOB));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(1 * knobSpacing, 30)), module, Ouros::ROTATE_ATT_KNOB));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(1 * knobSpacing, 55)), module, Ouros::ROTATE_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec(2 * knobSpacing, 0)), module, Ouros::SPREAD_KNOB));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(2 * knobSpacing, 30)), module, Ouros::SPREAD_ATT_KNOB));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(2 * knobSpacing, 55)), module, Ouros::SPREAD_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec(3 * knobSpacing, 0)), module, Ouros::MULTIPLY_KNOB));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(3 * knobSpacing, 30)), module, Ouros::MULTIPLY_ATT_KNOB));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(3 * knobSpacing, 55)), module, Ouros::MULTIPLY_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec(0 * knobSpacing, 125)), module, Ouros::RATE_KNOB));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(0 * knobSpacing, 165)), module, Ouros::RATE_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec(1 * knobSpacing, 110)), module, Ouros::FEEDBACK_KNOB));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(1 * knobSpacing, 140)), module, Ouros::FEEDBACK_ATT_KNOB));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(1 * knobSpacing, 165)), module, Ouros::FEEDBACK_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec(2 * knobSpacing, 110)), module, Ouros::POSITION_KNOB));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(2 * knobSpacing, 140)), module, Ouros::POSITION_ATT_KNOB));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(2 * knobSpacing, 165)), module, Ouros::POSITION_INPUT));

        addParam(createParamCentered<RoundBlackKnob>(knobStartPos.plus(Vec(3 * knobSpacing, 110)), module, Ouros::NODE_KNOB));
        addParam(createParamCentered<Trimpot>(knobStartPos.plus(Vec(3 * knobSpacing, 140)), module, Ouros::NODE_ATT_KNOB));
        addInput(createInputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(3 * knobSpacing, 165)), module, Ouros::NODE_INPUT));

        addOutput(createOutputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(3 * knobSpacing, -102)), module, Ouros::L_OUTPUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(knobStartPos.plus(Vec(3 * knobSpacing, -72)), module, Ouros::R_OUTPUT));

        // Create and add the PolarXYDisplay
        PolarXYDisplay* polarDisplay = createWidget<PolarXYDisplay>(Vec(56.5, 55.5)); // Positioning
        polarDisplay->box.size = Vec(50, 50); // Size of the display widget
        polarDisplay->module = module;
        addChild(polarDisplay);
    }    
};

Model* modelOuros = createModel<Ouros, OurosWidget>("Ouros");
