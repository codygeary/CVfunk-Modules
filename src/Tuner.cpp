////////////////////////////////////////////////////////////
//
//   Tuner
//
//   written by Cody Geary
//   Copyright 2025, MIT License
//
//   A tuner and zero-crossing dynamic scope
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
#include <cmath>
#include <string>
#include "digital_display.hpp"

using namespace rack;
template<typename T, size_t Size>
class CircularBuffer {
private:
    T buffer[Size];
public:
    T& operator[](size_t i) { return buffer[i % Size]; }
    const T& operator[](size_t i) const { return buffer[i % Size]; }
    static constexpr size_t size() { return Size; }
};

// Put at top of file where other includes are
#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>

struct FrequencyTracker {
    static constexpr int BUFFER_SIZE = 6000;
    
    // Double buffering
    float writeBuffer[BUFFER_SIZE] = {0.f};
    float processBuffer[BUFFER_SIZE] = {0.f};
    int writeIndex = 0;
    bool bufferReady = false;
    
    // Autocorrelation state
    std::vector<float> ac;
    std::vector<float> win;
    std::vector<float> s;
    int acState = 0;  // 0=idle, 1=preprocessing, 2=computing AC, 3=finding peak
    int acLag = 1;
    int acHalf = 0;
    float acMean = 0.f;
    
    // AC accumulation for current lag
    float acNum = 0.f;
    float acDenom1 = 0.f;
    float acDenom2 = 0.f;
    int acSampleIndex = 0;
    
    float sampleRate = 44100.f;
    float lastFreq = 261.63;
    float smoothedFreq = 261.63;
    const float SMOOTH_FACTOR = 0.999f;  // Direct multiplication factor (cheaper than exp)
    
    // How many lags to compute per process() call
    static constexpr int LAGS_PER_CYCLE = 16;
    // Process AC computation every N cycles (1 = every cycle, 2 = every other, etc.)
    int processDivider = 8;
    int processCycleCounter = 0;
    
    // Fast approximations
    static float polyCos(float x) {
        // Wrap to [-pi, pi]
        x = x - 6.28318530718f * std::floor(x * 0.159154943092f + 0.5f);
        float x2 = x * x;
        float x4 = x2 * x2;
        float x6 = x4 * x2;
        float x8 = x6 * x2;
        return 1.0f - x2 * 0.5f + x4 * 0.04166667f - x6 * 0.00138889f + x8 * 0.00002480f;
    }

    FrequencyTracker() {
        ac.resize(BUFFER_SIZE / 2);
        win.resize(BUFFER_SIZE / 2);
        s.resize(BUFFER_SIZE);
        
        // Pre-compute Hann window using fast polyCos
        int half = BUFFER_SIZE / 2;
        for (int i = 0; i < half; ++i)
            win[i] = 0.5f * (1.f - polyCos(2.f * M_PI * i / (half - 1)));
    }

    void setSampleRate(float sr) { sampleRate = sr; }

    void setProcessDivider(int divider) {
        processDivider = std::max(1, divider);  // safety clamp
    }

    float process(float in) {
        // Fill write buffer
        writeBuffer[writeIndex++] = in;
        
        // When buffer is full, swap to process buffer
        if (writeIndex >= BUFFER_SIZE) {
            writeIndex = 0;
            if (acState == 0) {  // Only swap if not currently processing
                std::memcpy(processBuffer, writeBuffer, BUFFER_SIZE * sizeof(float));
                bufferReady = true;
                acState = 1;  // Start preprocessing
            }
        }
        
        // Do incremental autocorrelation work (with cycle skipping)
        if (bufferReady) {
            if (++processCycleCounter >= processDivider) {
                processCycleCounter = 0;
                processAutocorrelationChunk();
            }
        }
        
        // Simple exponential smoothing (cheaper than exp-based)
        smoothedFreq = SMOOTH_FACTOR * smoothedFreq + (1.f - SMOOTH_FACTOR) * lastFreq;
        return smoothedFreq;
    }
    
    void processAutocorrelationChunk() {
        int N = BUFFER_SIZE;
        acHalf = N / 2;
        
        switch (acState) {
            case 1: {  // Preprocessing: DC removal
                acMean = std::accumulate(processBuffer, processBuffer + N, 0.f) / N;
                for (int i = 0; i < N; ++i) 
                    s[i] = processBuffer[i] - acMean;
                
                // Clear AC buffer
                std::fill(ac.begin(), ac.end(), 0.f);
                
                acLag = 1;
                acState = 2;  // Move to AC computation
                break;
            }
            
            case 2: {  // Compute autocorrelation for one lag per cycle
                if (acLag < acHalf) {
                    // Compute AC for current lag - optimized inner loop
                    float num = 0.f, denom1 = 0.f, denom2 = 0.f;
                    const float* w = win.data();
                    const float* sig = s.data();
                    
                    // Unroll hint and minimize memory access
                    for (int i = 0; i < acHalf; ++i) {
                        float w_val = w[i];
                        float a = sig[i] * w_val;
                        float b = sig[i + acLag] * w_val;
                        num += a * b;
                        denom1 += a * a;
                        denom2 += b * b;
                    }
                    
                    // Use standard sqrt for accuracy (critical for pitch detection)
                    float denom = std::sqrt(denom1 * denom2) + 1e-12f;
                    ac[acLag] = num / denom;
                    
                    // Clamp to [-1,1]
                    ac[acLag] = std::min(1.f, std::max(-1.f, ac[acLag]));
                    
                    acLag++;
                }
                
                if (acLag >= acHalf) {
                    acState = 3;  // Move to peak finding
                }
                break;
            }
            
            case 3: {  // Find peak and compute frequency
                lastFreq = computeFrequencyFromAC();
                bufferReady = false;
                acState = 0;  // Return to idle
                break;
            }
        }
    }

    float computeFrequencyFromAC() {
        int half = acHalf;
        
        // Find global max with reduced branching
        float globalMaxVal = -2.f;
        int globalMaxLag = 1;
        for (int lag = 1; lag < half; ++lag) {
            float val = ac[lag];
            if (val > globalMaxVal) {
                globalMaxVal = val;
                globalMaxLag = lag;
            }
        }
        
        // Adaptive threshold - simplified
        float adaptiveThreshold = 0.55f + 0.1f * (globalMaxVal - 0.8f);
        adaptiveThreshold = std::min(0.9f, std::max(0.45f, adaptiveThreshold));
        
        // Find first strong local peak
        int candidateLag = 0;
        for (int lag = 2; lag < half - 1; ++lag) {
            bool isLocalMax = (ac[lag] > ac[lag - 1]) && (ac[lag] >= ac[lag + 1]);
            bool isStrong = ac[lag] >= adaptiveThreshold;
            if (isLocalMax & isStrong) {
                candidateLag = lag;
                break;
            }
        }
        
        int bestLag = (candidateLag >= 2) ? candidateLag : std::max(2, globalMaxLag);
    
        // --- SIMPLE SIGNAL QUALITY CHECK ---
        if (ac[bestLag] < 0.1f) {
            // Really bad: probably noise
            return -1.f;
        } else if (ac[bestLag] < 0.25f) {
            // Slightly weak: keep previous to avoid jitter
            return lastFreq;
        }
    
        // Parabolic refinement
        if (bestLag > 1 && bestLag < half - 1) {
            float y0 = ac[bestLag - 1];
            float y1 = ac[bestLag];
            float y2 = ac[bestLag + 1];
            float denom = y0 - 2.f * y1 + y2;
            float shift = (fabsf(denom) > 1e-12f) ? (0.5f * (y0 - y2) / denom) : 0.f;
            float refinedLag = (float)bestLag + shift;
            refinedLag = (refinedLag <= 1.f) ? (float)bestLag : refinedLag;
            float freq = sampleRate / refinedLag;
            
            return (std::isfinite(freq) && freq > 0.f) ? freq : lastFreq;
        }
        
        float freq = sampleRate / (float)bestLag;
        
        if (!std::isfinite(freq) || freq < 20.f || freq > 20000.f)
            return -1.f; // Out-of-range        
    
        return freq;
    }

};

struct Tuner : Module {

    enum ParamId {
        OFFSET_PARAM, OFFSET2_PARAM, 
        WIDTH_PARAM, WIDTH2_PARAM,
        GAIN_PARAM, GAIN2_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        AUDIO_INPUT, AUDIO2_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        FREQ_OUTPUT, FREQ2_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        LIGHTS_LEN
    };

    FrequencyTracker freqTracker[2];
    CircularBuffer<float, 1024> waveBuffer[2];

    float sampleRate = 48000.f;
    float increment_factor[2] = {0.f, 0.f};

    float currentHz[2] = {0.f, 0.f};
    float currentVOct[2] = {0.f, 0.f};
    std::string currentNote[2] = {"---", "---"};
    std::string centsDeviation[2] = {"---", "---"};

    int counter[2] = {0, 0}; //CPU reduction counter
    int prevSampleIndex[2] = {0, 0};
    
    // --- Oscilloscope Engine ---
    float prevIn[2] = {0.f, 0.f};
    bool capturing[2] = {false, false};
    float captureProgress[2] = {0.f, 0.f}; // [0..1) over display window

    float offset[2] = {0.f, 0.f};
    float gain[2] = {1.0f, 1.0f};

    int updateSpeed = 8;

    bool displayMode = false;
    
    json_t* dataToJson() override {
        json_t* rootJ = json_object();
    
        // Save updateSpeed
        json_object_set_new(rootJ, "updateSpeed", json_integer(updateSpeed));
    
        // Save displayMode
        json_object_set_new(rootJ, "displayMode", json_boolean(displayMode));
    
        return rootJ;
    }
    
    void dataFromJson(json_t* rootJ) override {
        // Load displayMode
        json_t* displayModeJ = json_object_get(rootJ, "displayMode");
        if (displayModeJ) {
            displayMode = json_boolean_value(displayModeJ);
        }
    
        // Load updateSpeed with clamping & tracker update
        json_t* speedJ = json_object_get(rootJ, "updateSpeed");
        if (speedJ) {
            updateSpeed = json_integer_value(speedJ);
            updateSpeed = clamp(updateSpeed, 1, 16);
            for (int i = 0; i < 2; i++)
                freqTracker[i].setProcessDivider(updateSpeed);
        }
    }



    Tuner() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configInput(AUDIO_INPUT, "Audio 1");
        configOutput(FREQ_OUTPUT, "Frequency (V/oct)");
        configParam(GAIN_PARAM, 0.f, 5.f, 1.f, "Wave Gain");
        configParam(OFFSET_PARAM, -5.f, 5.f, 0.f, "Wave Offset");
        configParam(WIDTH_PARAM, 1.f, 6.f, 1.f, "Width in Wavelengths")->snapEnabled=true;
        paramQuantities[WIDTH_PARAM]->displayMultiplier = 2.0f;

        configInput(AUDIO2_INPUT, "Audio 2");
        configOutput(FREQ2_OUTPUT, "Frequency (V/oct)");
        configParam(GAIN2_PARAM, 0.f, 5.f, 1.f, "Wave Gain");
        configParam(OFFSET2_PARAM, -5.f, 5.f, 0.f, "Wave Offset");
        configParam(WIDTH2_PARAM, 1.f, 6.f, 1.f, "Width in Wavelengths")->snapEnabled=true;
        paramQuantities[WIDTH2_PARAM]->displayMultiplier = 2.0f;
        
    }

    void onSampleRateChange() override {
        sampleRate = APP->engine->getSampleRate();
        freqTracker[0].setSampleRate(sampleRate);
        freqTracker[1].setSampleRate(sampleRate);
    }

    void process(const ProcessArgs& args) override {
    
        for (int layer = 0; layer<2; layer++){    
            float in = 0.f;
     
            // --- Read params infrequently
            if (++counter[layer]>= 100) { 
                offset[layer] = params[OFFSET_PARAM + layer].getValue();
                gain[layer] = params[GAIN_PARAM + layer].getValue();
            }
                 
            if (inputs[AUDIO_INPUT + layer].isConnected())
                in = clamp(inputs[AUDIO_INPUT + layer].getVoltage(0)*gain[layer] + offset[layer], -10.f, 10.f);
    
            // --- Track frequency ---
            currentHz[layer] = freqTracker[layer].process(in);
            if (currentHz[layer] < 20.f || currentHz[layer] > 20000.f ) currentHz[layer] = -1.f;
    
            // --- Compute V/oct output only every N samples ---
            if (counter[layer] >= 100) { // update every ~2ms @ 48kHz
                counter[layer] = 0;
    
                if (currentHz[layer] > 20.f) {
                    currentVOct[layer] = log2f(currentHz[layer] / 440.f) + 4.f - 3.25f;
                    outputs[FREQ_OUTPUT + layer].setVoltage(currentVOct[layer]);
                } else {
                    outputs[FREQ_OUTPUT + layer].setVoltage(0.f);
                    currentVOct[layer] = -999.f;
                }
            }
            
            // --- Dynamic increment factor based on frequency ---
            float N_CYCLES = params[WIDTH_PARAM + layer].getValue()*2.0f; // display N full cycles
            float inc = 1.0f / 1024.0f;
            if (currentVOct[layer] > -100.f && std::isfinite(currentVOct[layer])) {
                float f = 440.0f * std::exp2f(currentVOct[layer] - 4.f + 3.25f); // reverse correction
                if (f > 1e-3f)
                    inc = f / (N_CYCLES * sampleRate);
            }
            increment_factor[layer] = clamp(inc, 1e-6f, 0.5f);
    
            
            float scaledIn = in * 0.5f;
        
            // --- Start new capture on zero-crossing ---
            if (!capturing[layer] && in >= 0.f && prevIn[layer] <= 0.f) {
                capturing[layer] = true;
                captureProgress[layer] = 0.f;
            }
        
            prevIn[layer] = in;
        
            // --- Capture samples when active ---
            if (capturing[layer]) {
                // advance fractional progress through display window
                captureProgress[layer] += increment_factor[layer];
                if (captureProgress[layer] >= 1.f) {
                    // finished one full display window
                    capturing[layer] = false;
                    captureProgress[layer] = 0.f;
                }
        
                int sampleIndex = static_cast<int>(captureProgress[layer] * 1024.f);
                sampleIndex = clamp(sampleIndex, 0, 1023);
        
                // write current sample
                waveBuffer[layer][sampleIndex] = scaledIn;
        
                // --- interpolate gaps ---
                if (sampleIndex != prevSampleIndex[layer]) {
                    int prev = prevSampleIndex[layer];
                    int next = sampleIndex;
                    float prevVal = waveBuffer[layer][prev];
        
                    if (prev < next) {
                        int gap = next - prev;
                        for (int i = 1; i < gap; ++i) {
                            float t = (float)i / gap;
                            waveBuffer[layer][prev + i] = prevVal + t * (scaledIn - prevVal);
                        }
                    } else if (prev > next) {
                        // wrap-around interpolation
                        int gapToEnd = 1023 - prev;
                        for (int i = 1; i <= gapToEnd; ++i) {
                            float t = (float)i / (gapToEnd + 1);
                            waveBuffer[layer][prev + i] = prevVal + t * (scaledIn - prevVal);
                        }
                        for (int i = 0; i <= next; ++i) {
                            float t = (float)(i + 1) / (next + 1);
                            waveBuffer[layer][i] = prevVal + t * (scaledIn - prevVal);
                        }
                    }
                    prevSampleIndex[layer] = sampleIndex;
                }
            }            
        }
    }    
};

struct TunerWidget : ModuleWidget {
    struct WaveDisplay : TransparentWidget {
        Tuner* module = nullptr;
        unsigned buf_idx = 0;
    
        void drawLayer(const DrawArgs& args, int layer) override {
            if (!module || layer != 1) return;

            Tuner* tuner = dynamic_cast<Tuner*>(module);
            if (!tuner)
                return;

            if (tuner->displayMode) {
                nvgBeginPath(args.vg);
                nvgRect(args.vg, 0.f, box.size.y/2.f - 20.f, box.size.x, 40.f); 
                nvgFillColor(args.vg, nvgRGB(0x21, 0x21, 0x21));  // #212121
                nvgFill(args.vg);
                nvgClosePath(args.vg);

                // Stop before drawing scope
                return;
            }
    
            float centerY = box.size.y / 2.f;
            float scale = centerY / 5.f;
    
            // --- Draw the waveform ---
            nvgBeginPath(args.vg);
            for (size_t i = 0; i < 1024; i++) {
                float x = (float)i / 1023.f * box.size.x;
                float y = centerY - module->waveBuffer[buf_idx][i] * scale;
                // FIX: Check the correct channel based on buf_idx
                if (module->currentHz[buf_idx] < 0.0f) y = centerY;
                if (i == 0)
                    nvgMoveTo(args.vg, x, y);
                else
                    nvgLineTo(args.vg, x, y);
            }
    
            nvgStrokeColor(args.vg, nvgRGBAf(0.0, 0.7, 1.0, 0.9));
            nvgStrokeWidth(args.vg, 1.2);
            nvgStroke(args.vg);
    
        }
    };
    DigitalDisplay* noteDisp = nullptr;
    DigitalDisplay* centsDisp = nullptr;
    DigitalDisplay* freqDisp = nullptr;
    WaveDisplay* waveDisp = nullptr;

    DigitalDisplay* noteDisp2 = nullptr;
    DigitalDisplay* centsDisp2 = nullptr;
    DigitalDisplay* freqDisp2 = nullptr;
    WaveDisplay* waveDisp2 = nullptr;

    TunerWidget(Tuner* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Tuner.svg"),
            asset::plugin(pluginInstance, "res/Tuner-dark.svg")
        ));

        // Screws
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // WaveDisplay 
        waveDisp = new WaveDisplay();
        waveDisp->module = module;
        waveDisp->buf_idx = 0;
        waveDisp->box.pos = mm2px(Vec(8, 13));
        waveDisp->box.size = mm2px(Vec(29.939*2, 32.608));
        addChild(waveDisp);

        // Digital displays
        noteDisp = createDigitalDisplay(Vec(box.size.x / 2 - 25-20, 40), "C4");
        addChild(noteDisp);

        centsDisp = createDigitalDisplay(Vec(box.size.x / 2 - 25+20, 40), "0.0%");
        addChild(centsDisp);

        freqDisp = createDigitalDisplay(Vec(box.size.x / 2 - 25, 120), "261.6 Hz");
        addChild(freqDisp);

        // I/O
        addInput(createInputCentered<PJ301MPort>(Vec( (box.size.x / 6)*1.0f, 170), module, Tuner::AUDIO_INPUT));
        addParam(createParamCentered<RoundBlackKnob>(Vec((box.size.x / 6)*2.0f, 160), module, Tuner::OFFSET_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(Vec((box.size.x / 6)*3.0f, 160), module, Tuner::GAIN_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(Vec((box.size.x / 6)*4.0f, 160), module, Tuner::WIDTH_PARAM));
        addOutput(createOutputCentered<PJ301MPort>(Vec((box.size.x / 6)*5.0f, 170), module, Tuner::FREQ_OUTPUT));

        float dispOffset =165.f;
        
        // WaveDisplay 
        waveDisp2 = new WaveDisplay();
        waveDisp2->module = module;
        waveDisp2->buf_idx = 1;
        waveDisp2->box.pos = mm2px(Vec(8, 13 + 25.4f/75.f*dispOffset));
        waveDisp2->box.size = mm2px(Vec(29.939*2, 32.608));
        addChild(waveDisp2);

        // Digital displays
        noteDisp2 = createDigitalDisplay(Vec(box.size.x / 2 - 25-20, 40+ dispOffset), "C4");
        addChild(noteDisp2);

        centsDisp2 = createDigitalDisplay(Vec(box.size.x / 2 - 25+20, 40+ dispOffset), "0.0%");
        addChild(centsDisp2);

        freqDisp2 = createDigitalDisplay(Vec(box.size.x / 2 - 25, 120+ dispOffset), "261.6 Hz");
        addChild(freqDisp2);

        // I/O
        addInput(createInputCentered<PJ301MPort>(Vec( (box.size.x / 6)*1.0f, 170+ dispOffset), module, Tuner::AUDIO_INPUT + 1));
        addParam(createParamCentered<RoundBlackKnob>(Vec((box.size.x / 6)*2.0f, 160+ dispOffset), module, Tuner::OFFSET_PARAM + 1));
        addParam(createParamCentered<RoundBlackKnob>(Vec((box.size.x / 6)*3.0f, 160+ dispOffset), module, Tuner::GAIN_PARAM + 1));
        addParam(createParamCentered<RoundBlackKnob>(Vec((box.size.x / 6)*4.0f, 160+ dispOffset), module, Tuner::WIDTH_PARAM + 1));
        addOutput(createOutputCentered<PJ301MPort>(Vec((box.size.x / 6)*5.0f, 170+ dispOffset), module, Tuner::FREQ_OUTPUT + 1));

    }

    DigitalDisplay* createDigitalDisplay(Vec position, std::string initialValue) {
        DigitalDisplay* display = new DigitalDisplay();
        display->box.pos = position;
        display->box.size = Vec(50, 18);
        display->text = initialValue;
        display->fgColor = nvgRGB(208, 140, 89);
        display->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        display->setFontSize(18.f);
        return display;
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);

        Tuner* tunerModule = dynamic_cast<Tuner*>(this->module);
        if (!tunerModule) return;

        menu->addChild(new MenuSeparator());

        // Title
        menu->addChild(createMenuLabel("Autocorrelation update speed"));

        // Menu item base struct
        struct UpdateSpeedItem : MenuItem {
            Tuner* module;
            int speed;
            void onAction(const event::Action& e) override {
                module->updateSpeed = speed;
                for (int i = 0; i < 2; i++)
                    module->freqTracker[i].setProcessDivider(speed);
            }
            void step() override {
                rightText = (module->updateSpeed == speed) ? "✔" : "";
                MenuItem::step();
            }
        };
        
        // Extended options (more granular control)
        const std::vector<std::pair<const char*, int>> options = {
            {"Ultra light (slow updates, lowest CPU)", 16},
            {"Light (CPU friendly)", 8},
            {"Medium (balanced)", 4},
            {"Fast (high precision)", 2},
            {"Ultra fast (maximum precision, heavy CPU)", 1},
        };
        
        for (auto& opt : options) {
            auto* item = new UpdateSpeedItem();
            item->text = opt.first;
            item->speed = opt.second;
            item->module = tunerModule;
            menu->addChild(item);
        }

        menu->addChild(new MenuSeparator());

        struct DisplayModeItem : MenuItem {
            Tuner* tunerModule;
        
            void onAction(const event::Action& e) override {
                // Toggle the displayMode variable in the module
                tunerModule->displayMode = !tunerModule->displayMode;
            }
        
            void step() override {
                rightText = tunerModule->displayMode ? "✔" : ""; // Show checkmark if true
                MenuItem::step();
            }
        };
        
        // Create the Display Mode menu item
        DisplayModeItem* displayModeItem = new DisplayModeItem();
        displayModeItem->text = "Large Hz display (disable waveform)"; // Set menu item text
        displayModeItem->tunerModule = tunerModule;                     // Pass the module pointer
        menu->addChild(displayModeItem);                                 // Add to context menu

        
    }

    void step() override {
        Tuner* module = dynamic_cast<Tuner*>(this->module);
        if (!module) return;
    
        // Compute note/cents for display only
        if (module->currentHz[0] > 0.0001f) {
            static const char* names[12] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
            float midi = 69.f + 12.f * log2f(module->currentHz[0] / 440.f);
            int noteNum = std::lround(midi);
            int noteIdx = (noteNum + 1200) % 12;
            int octave = (noteNum / 12) - 1;
            module->currentNote[0] = string::f("%s%d", names[noteIdx], octave);
            float cents = (midi - noteNum) * 100.f;
            module->centsDeviation[0] = string::f("%+0.1f", cents);
        } else {
            module->currentNote[0] = "(0)";
            module->centsDeviation[0] = "(0)";
        }
    
        noteDisp->text = module->currentNote[0];
        centsDisp->text = module->centsDeviation[0];
    
        // Precision based on display mode
        if (module->displayMode)
            freqDisp->text = (module->currentHz[0] > 0.f) ? string::f("%.2f Hz", module->currentHz[0]) : "-=-";
        else
            freqDisp->text = (module->currentHz[0] > 0.f) ? string::f("%.1f Hz", module->currentHz[0]) : "-=-";
    
        // Compute note/cents for display only (second channel)
        if (module->currentHz[1] > 0.0001f) {
            static const char* names[12] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
            float midi = 69.f + 12.f * log2f(module->currentHz[1] / 440.f);
            int noteNum = std::lround(midi);
            int noteIdx = (noteNum + 1200) % 12;
            int octave = (noteNum / 12) - 1;
            module->currentNote[1] = string::f("%s%d", names[noteIdx], octave);
            float cents = (midi - noteNum) * 100.f;
            module->centsDeviation[1] = string::f("%+0.1f", cents);
        } else {
            module->currentNote[1] = "(o)";
            module->centsDeviation[1] = "(o)";
        }
    
        noteDisp2->text = module->currentNote[1];
        centsDisp2->text = module->centsDeviation[1];
    
        if (module->displayMode)
            freqDisp2->text = (module->currentHz[1] > 0.f) ? string::f("%.2f Hz", module->currentHz[1]) : "-=-";
        else
            freqDisp2->text = (module->currentHz[1] > 0.f) ? string::f("%.1f Hz", module->currentHz[1]) : "-=-";
    
        // --- Apply display mode visual change (fixed) ---
        const float baseFreqY1 = 120.f;
        const float baseFreqY2 = 120.f + 165.f;
    
        if (module->displayMode) {
            freqDisp->setFontSize(36.f);
            freqDisp->box.pos.y = baseFreqY1 - 40.f;
    
            freqDisp2->setFontSize(36.f);
            freqDisp2->box.pos.y = baseFreqY2 - 40.f;
        } else {
            freqDisp->setFontSize(18.f);
            freqDisp->box.pos.y = baseFreqY1;
            freqDisp2->setFontSize(18.f);
            freqDisp2->box.pos.y = baseFreqY2;
        }
        ModuleWidget::step(); 
    }
};

Model* modelTuner = createModel<Tuner, TunerWidget>("Tuner");