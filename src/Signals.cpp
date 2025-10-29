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
    std::array<std::vector<float>, 6> displayBuffers; // persistent display copy
    std::array<int, 6> writeIndices = {}; 
    float lastInputs[6] = {};
    std::array<float, 6> lastTriggerTime = {}; 
    bool retriggerEnabled = false; 
    bool retriggerToggleProcessed = false;
    double displayUpdateTime = 0.1; 
    double timeSinceLastUpdate = 0.0;
    float scopeInput[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    int scopeChannels[6] = {0, 0, 0, 0, 0, 0};  // Number of polyphonic channels for Scope inputs
    int activeScopeChannel[6] = {-1, -1, -1, -1, -1, -1};  // Stores the number of the previous active channel for the Scope
    int previousActiveScopeChannel[6] = {-1, -1, -1, -1, -1, -1};  // Track previous state to detect changes

    //non-glitchy display refreshing
    bool waitingForTrigger[6] = {true, true, true, true, true, true};
    bool displayReady[6] = {false, false, false, false, false, false};
    int samplesSinceTrigger[6] = {0, 0, 0, 0, 0, 0};

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
        configSwitch(RANGE_BUTTON_PARAM, 0.f, 1.f, 0.f, "Mode", {"Default", "Slow"});

        lastTriggerTime.fill(0.0f);
        MAX_BUFFER_SIZE = int(static_cast<int>(APP->engine->getSampleRate() * MAX_TIME));

        for (auto &buffer : envelopeBuffers) {
            buffer.resize(MAX_BUFFER_SIZE, 0.0f);
        }
        for (auto &buffer : displayBuffers) {
            buffer.resize(MAX_BUFFER_SIZE, 0.0f);
        }        
    }

    void onSampleRateChange() override {
        MAX_BUFFER_SIZE = int(static_cast<int>(APP->engine->getSampleRate() * MAX_TIME));
    
        for (auto &buffer : envelopeBuffers) {
            buffer.resize(MAX_BUFFER_SIZE, 0.0f);
        }
        for (auto &buffer : displayBuffers) {
            buffer.resize(MAX_BUFFER_SIZE, 0.0f);
        }
    }
    
    void process(const ProcessArgs& args) override {
        float range = pow(params[RANGE_PARAM].getValue(),3.0f);
    
        if (params[RANGE_BUTTON_PARAM].getValue() > 0.5f) {
            currentTimeSetting = MAX_TIME;
            lights[LONG_LIGHT].setBrightness(1.0f);
        } else {
            currentTimeSetting = 1.0f;
            lights[LONG_LIGHT].setBrightness(0.0f);
        }
        range = clamp(range, 0.000001f, .9999f);
    
        int currentBufferSize = std::max(1, int((MAX_BUFFER_SIZE / MAX_TIME) * currentTimeSetting * range));
    
        // --- Scan inputs ---
        for (int i = 0; i < 6; i++) {
            scopeChannels[i] = 0;
            activeScopeChannel[i] = -1;
    
            if (inputs[ENV1_INPUT + i].isConnected()) {
                scopeChannels[i] = inputs[ENV1_INPUT + i].getChannels();
                activeScopeChannel[i] = i;
            } else if (i > 0 && activeScopeChannel[i-1] != -1) {
                if (scopeChannels[activeScopeChannel[i-1]] > (i - activeScopeChannel[i-1])) {
                    activeScopeChannel[i] = activeScopeChannel[i-1];
                }
            }
        }
    
        for (int i = 0; i < 6; ++i) {
            // --- Check if channel state changed ---
            bool channelStateChanged = (activeScopeChannel[i] != previousActiveScopeChannel[i]);
            
            // --- Inactive channel: zero buffers ONLY when state changes ---
            if (activeScopeChannel[i] == -1) {
                if (channelStateChanged) {
                    std::fill(envelopeBuffers[i].begin(), envelopeBuffers[i].end(), 0.f);
                    std::fill(displayBuffers[i].begin(), displayBuffers[i].end(), 0.f);
                    writeIndices[i] = 0;
                    lastInputs[i] = 0.f;
                    lastTriggerTime[i] = 0.f;
                    displayReady[i] = false;
                    waitingForTrigger[i] = true;
                    samplesSinceTrigger[i] = 0;
                }
                outputs[ENV1_OUTPUT + i].setVoltage(0.0f);
                previousActiveScopeChannel[i] = activeScopeChannel[i];
                continue;
            }
        
            // --- Fetch input voltage ---
            int diffBetween = i - activeScopeChannel[i];
            int currentChannelMax = scopeChannels[activeScopeChannel[i]];
            if (currentChannelMax - diffBetween <= 0) {
                // This channel index doesn't exist in the poly cable
                if (channelStateChanged) {
                    std::fill(envelopeBuffers[i].begin(), envelopeBuffers[i].end(), 0.f);
                    std::fill(displayBuffers[i].begin(), displayBuffers[i].end(), 0.f);
                    writeIndices[i] = 0;
                    lastInputs[i] = 0.f;
                }
                outputs[ENV1_OUTPUT + i].setVoltage(0.0f);
                previousActiveScopeChannel[i] = -2;  // Special state: poly cable exists but this channel is out of range
                continue;
            }
        
            // Reset state if channel just became active
            if (channelStateChanged) {
                writeIndices[i] = 0;
                lastInputs[i] = 0.f;
                lastTriggerTime[i] = 0.f;
                waitingForTrigger[i] = true;
                displayReady[i] = false;
                samplesSinceTrigger[i] = 0;
            }
            
            scopeInput[i] = clamp(inputs[ENV1_INPUT + activeScopeChannel[i]].getPolyVoltage(diffBetween), -10.f, 10.f);
            lastTriggerTime[i] += args.sampleTime;
        
            if (retriggerEnabled) {
                // --- Retrigger / capture ---
                if (waitingForTrigger[i]) {
                    if (scopeInput[i] > 0.f && lastInputs[i] <= 0.f) {
                        waitingForTrigger[i] = false;
                        displayReady[i] = false;
                        writeIndices[i] = 0;
                        samplesSinceTrigger[i] = 0;
                    }
                } else {
                    if (writeIndices[i] < currentBufferSize) {
                        envelopeBuffers[i][writeIndices[i]] = scopeInput[i];
                    }
                    writeIndices[i] = (writeIndices[i] + 1) % currentBufferSize;
                    samplesSinceTrigger[i]++;
        
                    if (samplesSinceTrigger[i] >= currentBufferSize) {
                        displayReady[i] = true;
                        waitingForTrigger[i] = true;
                        samplesSinceTrigger[i] = 0;
        
                        // Copy current capture into display buffer
                        std::copy(
                            envelopeBuffers[i].begin(),
                            envelopeBuffers[i].begin() + currentBufferSize,
                            displayBuffers[i].begin()
                        );
                    }
                }
            } else {
                // --- Free-running continuous update ---
                // Write to current position BEFORE incrementing
                if (writeIndices[i] < currentBufferSize) {
                    envelopeBuffers[i][writeIndices[i]] = scopeInput[i];
                    displayBuffers[i][writeIndices[i]] = scopeInput[i];
                }
                writeIndices[i] = (writeIndices[i] + 1) % currentBufferSize;
        
                displayReady[i] = true;
                waitingForTrigger[i] = false;
            }
        
            lastInputs[i] = scopeInput[i];
        
            // --- Output ---
            outputs[ENV1_OUTPUT + i].setVoltage(scopeInput[i]);
            
            // Update previous state
            previousActiveScopeChannel[i] = activeScopeChannel[i];
        }

    
        // --- Retrigger toggle ---
        if (params[TRIGGER_ON_PARAM].getValue() > 0.5f && !retriggerToggleProcessed) {
            retriggerEnabled = !retriggerEnabled;
            retriggerToggleProcessed = true;
            params[TRIGGER_ON_PARAM].setValue(0.0f);
    
            if (!retriggerEnabled) {
                for (int i = 0; i < NUM_INPUTS; ++i) {
                    writeIndices[i] = 0;
                    lastTriggerTime[i] = 0.f;
                }
            }
        } else if (params[TRIGGER_ON_PARAM].getValue() <= 0.5f) {
            retriggerToggleProcessed = false;
        }
    
        lights[TRIGGER_ON_LIGHT].setBrightness(retriggerEnabled ? 1.0f : 0.0f);
    
        // --- Refresh display ---
        timeSinceLastUpdate += args.sampleTime;
        if (timeSinceLastUpdate >= displayUpdateTime) {
            timeSinceLastUpdate = 0.0;
            if (fbWidget) fbWidget->dirty = true;
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

        // Always show last valid waveform if available
        const auto& buffer = module->displayBuffers[channelId];
        if (buffer.empty()) return;

        float range = pow(module->params[Signals::RANGE_PARAM].getValue(), 3.0f) / (MAX_TIME / module->currentTimeSetting);
    
        int displaySamples = 1024;
        std::vector<Vec> points;
    
        float firstSampleY = box.size.y;
        if ((module->activeScopeChannel[channelId] > -1) && !buffer.empty()) {
            firstSampleY = box.size.y * (1.0f - (buffer.front() / 15.0f));
        }
    
        points.push_back(Vec(0, box.size.y));
        points.push_back(Vec(0, firstSampleY));
    
        for (int i = 0; i < displaySamples; ++i) {
            // Ensure bufferIndex does not exceed buffer.size() - 1
            int bufferIndex = int(i * ((buffer.size() - 1) * range + 1) / (displaySamples - 1));
            bufferIndex = clamp(bufferIndex, 0, static_cast<int>(buffer.size()) - 1);
    
            float x = (static_cast<float>(i) / (displaySamples - 1)) * box.size.x;
            float y = box.size.y;
            if ((module->activeScopeChannel[channelId] > -1 )) {
                y = box.size.y * (1.0f - (buffer[bufferIndex] / 15.0f));
            }
            points.push_back(Vec(x, y));
        }
    
        // Draw the waveform
        nvgBeginPath(args.vg);
        nvgStrokeWidth(args.vg, 2.0f);
        nvgStrokeColor(args.vg, waveformColor);        
        nvgMoveTo(args.vg, points[0].x, points[0].y);
        for (size_t i = 1; i < points.size(); ++i) { // Start from 1 to avoid duplicating the first point
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