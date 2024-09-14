////////////////////////////////////////////////////////////
//
//   Signals
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   Displays 6 signals, with pass through
//
////////////////////////////////////////////////////////////

#include "plugin.hpp"
#include "rack.hpp"
#include <array>
#include <vector>

using namespace rack;

float MAX_TIME = 10.0f; // Max window time in seconds
int MAX_BUFFER_SIZE; // Buffer size will be set in the constructor based on the sample rate

struct Signals : Module {
    enum ParamId {
        RANGE_PARAM,
        TRIGGER_ON_PARAM,
        RANGE_BUTTON_PARAM,
        NUM_PARAMS
    };
    enum InputId {
        ENV1_INPUT, ENV2_INPUT, ENV3_INPUT, ENV4_INPUT, ENV5_INPUT, ENV6_INPUT,
        NUM_INPUTS
    };
    enum OutputId {
        ENV1_OUTPUT, ENV2_OUTPUT, ENV3_OUTPUT, ENV4_OUTPUT, ENV5_OUTPUT, ENV6_OUTPUT,
        NUM_OUTPUTS
    };
    enum LightId {
        TRIGGER_ON_LIGHT,
        LONG_LIGHT,
        NUM_LIGHTS
    };

    float currentTimeSetting = 1.0f;
    std::array<std::vector<float>, 6> envelopeBuffers;
    std::array<int, 6> writeIndices = {}; 
    float lastInputs[6] = {};
    std::array<float, 6> lastTriggerTime = {}; 
    bool retriggerEnabled = false; 
    bool retriggerToggleProcessed = false;
    double displayUpdateTime = 0.1; 
    double timeSinceLastUpdate = 0.0;
    float scopeInput[6] = {0.0f};
    int scopeChannels[6] = {0};  // Number of polyphonic channels for Scope inputs
    int activeScopeChannel[6] = {-1};  // Stores the number of the previous active channel for the Scope


    FramebufferWidget* fbWidget = nullptr;

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
        json_object_set_new(rootJ, "retriggerEnabled", json_boolean(retriggerEnabled));
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        json_t* retriggerEnabledJ = json_object_get(rootJ, "retriggerEnabled");
        if (retriggerEnabledJ) {
            retriggerEnabled = json_is_true(retriggerEnabledJ);
        }
    }

    Signals() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
        configParam(RANGE_PARAM, 0.1f, 0.9999f, 0.5f, "Range");
        configParam(TRIGGER_ON_PARAM, 0.f, 1.f, 1.f, "Retriggering");
        configParam(RANGE_BUTTON_PARAM, 0.f, 1.f, 0.f, "Range Mode");

        lastTriggerTime.fill(0.0f);
        MAX_BUFFER_SIZE = int(static_cast<int>(APP->engine->getSampleRate() * MAX_TIME));

        for (auto &buffer : envelopeBuffers) {
            buffer.resize(MAX_BUFFER_SIZE, 0.0f);
        }
    }

    void process(const ProcessArgs& args) override {
        float range = pow(params[RANGE_PARAM].getValue(),3.0f);

        if (params[RANGE_BUTTON_PARAM].getValue() > 0.5) {
            currentTimeSetting = MAX_TIME;
            lights[LONG_LIGHT].setBrightness(1.0f);
        } else {
            currentTimeSetting = 1.0f;
            lights[LONG_LIGHT].setBrightness(0.0f);
        }
        range = clamp(range, 0.000001f, .9999f);

        int currentBufferSize = int((MAX_BUFFER_SIZE / MAX_TIME) * currentTimeSetting * range);

		// Reset the arrays for storing polyphony
		for (int count = 0; count<6; count++){
	        scopeChannels[count] = 0;  // Number of polyphonic channels for Scope inputs
		    activeScopeChannel[count] = -1;  // Stores the number of the previous active channel for the Scope
		}
		//initialize all active channels with -1, indicating nothing connected.

		// Scan all inputs to determine the polyphony
		for (int i = 0; i < 6; i++) {		
			// Update the Scope channels
			if (inputs[ENV1_INPUT + i].isConnected()) {
				scopeChannels[i] = inputs[ENV1_INPUT + i].getChannels();
				activeScopeChannel[i] = i;
			} else if (i > 0){
				activeScopeChannel[i] = activeScopeChannel[i-1]; // Carry over the active channel		
			}
		}

        for (int i = 0; i < 6; ++i) { //for the 6 wave inputs
 
 
 			if (activeScopeChannel[i] == i) {
				scopeInput[i] = clamp(inputs[ENV1_INPUT + i].getPolyVoltage(0) , -10.f, 10.f);
				
                lastTriggerTime[i] += args.sampleTime;

                if (retriggerEnabled && scopeInput[i] > 0.0f 
                    && lastInputs[i] <= 0.0f 
                    && lastTriggerTime[i] >= ( range  * currentTimeSetting ) ) {
                        writeIndices[i] = 0;
                        lastTriggerTime[i] = 0.0f;
                } else {
                    envelopeBuffers[i][writeIndices[i]] = scopeInput[i];
                    writeIndices[i] = (writeIndices[i] + 1) % currentBufferSize;
                }

                lastInputs[i] = scopeInput[i];
				
			} else if (activeScopeChannel[i] > -1) {
				// Now we compute which channel we need to grab
				int diffBetween = i - activeScopeChannel[i];
				int currentChannelMax =  scopeChannels[activeScopeChannel[i]] ;	
				if (currentChannelMax - diffBetween > 0) {    //If we are before the last poly channel
					scopeInput[i] = clamp(inputs[ENV1_INPUT + activeScopeChannel[i]].getPolyVoltage(diffBetween), -10.f, 10.f);
					
                    lastTriggerTime[i] += args.sampleTime;
    
                    if (retriggerEnabled && scopeInput[i] > 0.0f 
                        && lastInputs[i] <= 0.0f 
                        && lastTriggerTime[i] >= ( range  * currentTimeSetting ) ) {
                            writeIndices[i] = 0;
                            lastTriggerTime[i] = 0.0f;
                    } else {
                        envelopeBuffers[i][writeIndices[i]] = scopeInput[i];
                        writeIndices[i] = (writeIndices[i] + 1) % currentBufferSize;
                    }
    
                    lastInputs[i] = scopeInput[i];
									
				}
			} else if (lastInputs[i] != 0.0f) {
                std::fill(envelopeBuffers[i].begin(), envelopeBuffers[i].end(), 0.0f);
                writeIndices[i] = 0;
                lastInputs[i] = 0.0f;
                lastTriggerTime[i] = 0.0f;
            }
 
 
        }

        if (params[TRIGGER_ON_PARAM].getValue() > 0.5f && !retriggerToggleProcessed) {
            retriggerEnabled = !retriggerEnabled;
            retriggerToggleProcessed = true;
            params[TRIGGER_ON_PARAM].setValue(0.0f);

            if (!retriggerEnabled) {
                for (int i = 0; i < NUM_INPUTS; ++i) {
                    std::fill(envelopeBuffers[i].begin(), envelopeBuffers[i].end(), 0.0f);
                    writeIndices[i] = 0;
                    lastTriggerTime[i] = 0.0f;
                }
            }
        } else if (params[TRIGGER_ON_PARAM].getValue() <= 0.5f) {
            retriggerToggleProcessed = false;
        }

        lights[TRIGGER_ON_LIGHT].setBrightness(retriggerEnabled ? 1.0f : 0.0f);

        for (int i = 0; i < NUM_INPUTS; ++i) {
            if (inputs[i].isConnected()) {
                outputs[i].setVoltage(inputs[i].getVoltage());
            } else {
                outputs[i].setVoltage(0.0f);
            }
        }

        timeSinceLastUpdate += args.sampleTime;
        if (timeSinceLastUpdate >= displayUpdateTime) {
            timeSinceLastUpdate = 0.0;
            if (fbWidget) {
                fbWidget->dirty = true;
            }
        }
    }
};

struct WaveformDisplay : TransparentWidget {
    Signals* module;
    int channelId;
    NVGcolor waveformColor;

    WaveformDisplay(NVGcolor color) : waveformColor(color) {}

    void drawWaveform(const DrawArgs& args) {
        if (!module) return;

        const auto& buffer = module->envelopeBuffers[channelId];
        float range = pow(module->params[Signals::RANGE_PARAM].getValue(),3.0f) / (MAX_TIME / module->currentTimeSetting);

        int displaySamples = 1024;
        std::vector<Vec> points;

        float firstSampleY = box.size.y;
        if ((module->activeScopeChannel[channelId] > -1) && !buffer.empty()) {
            firstSampleY = box.size.y * (1.0f - (buffer.front() / 15.0f));
        }

        points.push_back(Vec(0, box.size.y));
        points.push_back(Vec(0, firstSampleY));

        for (int i = 0; i < displaySamples; ++i) {
            int bufferIndex = int(i * ((buffer.size() - 1) * range + 1) / (displaySamples - 1));
            float x = (static_cast<float>(i) / (displaySamples - 1)) * box.size.x;
            float y = box.size.y;
            if ((module->activeScopeChannel[channelId] > -1 )) {
                y = box.size.y * (1.0f - (buffer[bufferIndex] / 15.0f));
            }
            points.push_back(Vec(x, y));
        }

        //draw the wave
        nvgBeginPath(args.vg);
        nvgStrokeWidth(args.vg, 2.0f);
        nvgStrokeColor(args.vg, waveformColor);        
        nvgMoveTo(args.vg, points[0].x, points[0].y);
        for (size_t i = 0; i < points.size()-1; ++i) {
            nvgLineTo(args.vg, points[i].x, points[i].y);
        }
        nvgStroke(args.vg);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer == 1) {
            drawWaveform(args);
        }
        TransparentWidget::drawLayer(args, layer);
    }

    void draw(const DrawArgs& args) override {
        // Only drawing in the self-illuminating layer
    }
};

struct SignalsWidget : ModuleWidget {
    FramebufferWidget* fbWidget;

    SignalsWidget(Signals* module) {
        setModule(module);
              
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Signals.svg"),
            asset::plugin(pluginInstance, "res/Signals-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // Range knob
        addParam(createParam<RoundBlackKnob>(mm2px(Vec(5, 12)), module, Signals::RANGE_PARAM));

        addParam(createParam<CKSS>(mm2px(Vec(17, 14)), module, Signals::RANGE_BUTTON_PARAM));
        addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(23, 16)), module, Signals::LONG_LIGHT));

        addParam(createParamCentered<TL1105>(mm2px(Vec(50, 19)), module, Signals::TRIGGER_ON_PARAM));
        addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(55, 19)), module, Signals::TRIGGER_ON_LIGHT));

        fbWidget = new FramebufferWidget();
        addChild(fbWidget);

        if (module) {
            module->fbWidget = fbWidget;
        }

        NVGcolor colors[6] = {
            nvgRGB(0xa0, 0xa0, 0xa0), // Even Lighter Grey
            nvgRGB(0x90, 0x90, 0x90), // Lighter Grey
            nvgRGB(0x80, 0x80, 0x80), // Grey
            nvgRGB(0x70, 0x70, 0x9b), // Grey-Blue
            nvgRGB(0x60, 0x60, 0x8b), // Darker Grey-Blue
            nvgRGB(0x50, 0x50, 0x7b)  // Even Darker Grey-Blue
        };

        float initialYPos = 75; 
        float spacing = 45; // Increase spacing
        for (int i = 0; i < 6; ++i) {
            float yPos = initialYPos + i * spacing; // Adjusted positioning and spacing

            addInput(createInput<ThemedPJ301MPort>(Vec(5, yPos + 20), module, i));
            addOutput(createOutput<ThemedPJ301MPort>(Vec(148, yPos + 20), module, i));

            WaveformDisplay* display = new WaveformDisplay(colors[i]);
            display->box.pos = Vec(39, yPos);
            display->box.size = Vec(104, 40); 
            display->module = module;
            display->channelId = i;
            fbWidget->addChild(display);
        }
    }
};

Model* modelSignals = createModel<Signals, SignalsWidget>("Signals");
