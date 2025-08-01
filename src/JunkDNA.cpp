#include "rack.hpp"
#include "plugin.hpp"
#include "digital_display.hpp"
using namespace rack;
#include <random>
#include <map>
#include <vector>


// Accepted IUPAC chars (uppercase only)
static const std::string VALID_IUPAC = "ACGUTRYWSKMBDHVN";

#define GENE_CAPACITY 2056

struct JunkDNA : Module {
    enum ParamIds {
        FWD_BUTTON, REV_BUTTON, RESET_BUTTON,
        NUM_PARAMS
    };
    enum InputIds {
        FWD_IN, REV_IN, RESET_IN,
        NUM_INPUTS
    };
    enum OutputIds {
        A_OUT, T_OUT, G_OUT, C_OUT,
        R_OUT, Y_OUT, S_OUT, W_OUT,
        D_OUT, H_OUT, V_OUT, B_OUT, N_OUT,
        DNA_OUT,
        NUM_OUTS
    };
    enum LightIds {
        A_LIGHT, T_LIGHT, G_LIGHT, C_LIGHT,
        R_LIGHT, Y_LIGHT, S_LIGHT, W_LIGHT,
        D_LIGHT, H_LIGHT, V_LIGHT, B_LIGHT, N_LIGHT,
        NUM_LIGHTS
    };

    std::string sequenceText = "N";
    std::string prevSequenceText = "N";
    dsp::SchmittTrigger fwdTrigger, revTrigger, resetTrigger, fwdButtonTrigger, revButtonTrigger, resetButtonTrigger;
    dsp::PulseGenerator outputPulse;
    bool pulseOutput = true;

    DigitalDisplay* displayRibbon[31] = {nullptr}; //odd number allows for a central display

    int gene[GENE_CAPACITY] = {}; // Stores the gene, max size 2056nts

    int geneSize = GENE_CAPACITY; //Length of the circular genome
    int sequenceIndex = 0; // Current index being read

    uint32_t seed = 42; //the answer :-) !
    std::mt19937 rng;

    bool initializing = true;

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
    
        json_object_set_new(rootJ, "sequenceText", json_string(sequenceText.c_str()));
        json_object_set_new(rootJ, "sequenceIndex", json_integer(sequenceIndex));
        json_object_set_new(rootJ, "geneSize", json_integer(geneSize));
        json_object_set_new(rootJ, "pulseOutput", json_boolean(pulseOutput));
    
        json_t* geneJ = json_array();
        for (int i = 0; i < GENE_CAPACITY; i++) {
            json_array_append_new(geneJ, json_integer(gene[i]));
        }
        json_object_set_new(rootJ, "gene", geneJ);
    
        return rootJ;
    }
    
    void dataFromJson(json_t* rootJ) override {
        json_t* seqJ = json_object_get(rootJ, "sequenceText");
        if (seqJ) {
            sequenceText = json_string_value(seqJ);
            prevSequenceText = sequenceText;
        }
    
        json_t* idxJ = json_object_get(rootJ, "sequenceIndex");
        if (idxJ)
            sequenceIndex = json_integer_value(idxJ);
    
        json_t* sizeJ = json_object_get(rootJ, "geneSize");
        if (sizeJ)
            geneSize = json_integer_value(sizeJ);

        json_t* gateOutputJ = json_object_get(rootJ, "pulseOutput");
        if (gateOutputJ) {
            pulseOutput = json_boolean_value(gateOutputJ);
        }

        json_t* geneArrJ = json_object_get(rootJ, "gene");
        if (geneArrJ && json_is_array(geneArrJ)) {
            size_t count = std::min(json_array_size(geneArrJ), (size_t)GENE_CAPACITY);
                for (size_t i = 0; i < count; ++i){
                gene[i] = json_integer_value(json_array_get(geneArrJ, i));
                
            }
            for (int i = count; i < GENE_CAPACITY; i++) {
                gene[i] = 0;
            }
        } else {
            regenerateGene();  // fallback if gene missing
        }
    }

    JunkDNA() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTS, NUM_LIGHTS);
        rng.seed(seed);
        configInput(FWD_IN, "Forward");
        configInput(REV_IN, "Reverse");
        configInput(RESET_IN, "Reset");
        configOutput(A_OUT, "A");
        configOutput(T_OUT, "T");
        configOutput(C_OUT, "C");
        configOutput(G_OUT, "G");
        configOutput(R_OUT, "R (puRine: A or G)");
        configOutput(Y_OUT, "Y (pYramidine: C or T)");
        configOutput(S_OUT, "S (Strong: C or G)");
        configOutput(W_OUT, "W (Weak: A or T)");
        configOutput(D_OUT, "D (Not C)");
        configOutput(H_OUT, "H (Not G)");
        configOutput(V_OUT, "V (Not T)");
        configOutput(B_OUT, "B (Not A)");
        configOutput(N_OUT, "N (aNy) - Outputs trigger each step");
        configOutput(DNA_OUT, "DNA Signal: 0=A, 1=T, 2=C 3=G");

        configParam(FWD_BUTTON, 0.f, 1.f, 0.f, "Forward");
        configParam(REV_BUTTON, 0.f, 1.f, 0.f, "Reverse");
        configParam(RESET_BUTTON, 0.f, 1.f, 0.f, "Reset");

    }

    void regenerateGene() {
        std::string pattern = sequenceText;
        int patternLength = pattern.length();
        if (patternLength == 0)
            return;
    
        // Define IUPAC expansion map
        static const std::map<char, std::vector<int>> iupacToBases = {
            {'A', {0}},           // A
            {'T', {1}},           // T
            {'U', {1}},           // U -> T
            {'C', {2}},           // C
            {'G', {3}},           // G
            {'R', {0, 3}},        // A or G
            {'Y', {1, 2}},        // T or C
            {'S', {2, 3}},        // C or G
            {'W', {0, 1}},        // A or T
            {'K', {1, 3}},        // T or G
            {'M', {0, 2}},        // A or C
            {'B', {1, 2, 3}},     // Not A
            {'D', {0, 1, 3}},     // Not C
            {'H', {0, 1, 2}},     // Not G
            {'V', {0, 2, 3}},     // Not T
            {'N', {0, 1, 2, 3}}   // Any base
        };
    
        // Fit gene to exact multiple of pattern
        int repeatCount = geneSize / patternLength;
        geneSize = repeatCount * patternLength;
        
        for (int i = 0; i < geneSize; ++i) {
            char base = toupper(pattern[i % patternLength]);
            if (base == 'U') base = 'T';  // normalize
    
            auto it = iupacToBases.find(base);
            std::vector<int> choices = (it != iupacToBases.end()) ? it->second : std::vector<int>{3}; // default to G
    
            std::uniform_int_distribution<int> dist(0, choices.size() - 1);
            gene[i] = choices[dist(rng)];
        }
    }

    void process(const ProcessArgs& args) override {

        // Reset Input and Button
        if (inputs[RESET_IN].isConnected()) {
            if ( resetTrigger.process( inputs[RESET_IN].getVoltage() -0.1f) ) {
                sequenceIndex = 0;
                outputPulse.trigger(0.001f);                
            }
        }
        if (resetButtonTrigger.process(params[RESET_BUTTON].getValue())) {
            sequenceIndex = 0;
            outputPulse.trigger(0.001f);            
        }

        if (fwdButtonTrigger.process( params[FWD_BUTTON].getValue() )
            || (inputs[FWD_IN].isConnected() && fwdTrigger.process( inputs[FWD_IN].getVoltage() -0.1f) ) ) {
            sequenceIndex++;
            outputPulse.trigger(0.001f);          
        }
        if (revButtonTrigger.process( params[REV_BUTTON].getValue() )
            || (inputs[REV_IN].isConnected() && revTrigger.process( inputs[REV_IN].getVoltage() -0.1f) ) ) {
            sequenceIndex--;
            outputPulse.trigger(0.001f);
        }

        if (sequenceIndex>geneSize){sequenceIndex = 0;} //Handle boundary conditions of sequenceIndex
        if (sequenceIndex<0){sequenceIndex = geneSize;}

        if (initializing) {
            initializing = false;
            regenerateGene();
        } else if (prevSequenceText != sequenceText) {
            prevSequenceText = sequenceText;
            regenerateGene();
        }
        
        std::string pattern = sequenceText;
        
        int currentNT = gene[sequenceIndex];

        // Set all outputs/lights to 0 first
        for (int i = 0; i < NUM_OUTS; i++)
            outputs[i].setVoltage(0.f);
        for (int i = 0; i < NUM_LIGHTS; i++)
            lights[i].value = 0.f;

        //Check if the step advancing pulse is active    
        bool pulseActive = outputPulse.process(args.sampleTime);

        //Deal with special case of N output that is always pulsed.
        if (pulseActive){
            outputs[N_OUT].setVoltage(10.f);
            lights[N_LIGHT].value = 1.f;
        }
        
        if (pulseOutput) {pulseActive = true;} //pulseOutput overrides the pulse-length
    
        float high = pulseActive ? 10.f : 0.f;
     
        // Now set specific outputs/lights for each NT
        switch (currentNT) {
            case 0: // A
                if (pulseActive) {
                    outputs[A_OUT].setVoltage(high);
                    outputs[R_OUT].setVoltage(high);
                    outputs[W_OUT].setVoltage(high);
                    outputs[H_OUT].setVoltage(high);
                    outputs[D_OUT].setVoltage(high);
                    outputs[V_OUT].setVoltage(high);
                }
                lights[A_LIGHT].value = 1.f;
                lights[R_LIGHT].value = 1.f;
                lights[W_LIGHT].value = 1.f;
                lights[H_LIGHT].value = 1.f;
                lights[D_LIGHT].value = 1.f;
                lights[V_LIGHT].value = 1.f;
                outputs[DNA_OUT].setVoltage(0.f);
                break;
    
            case 1: // T/U
                if (pulseActive) {
                    outputs[T_OUT].setVoltage(high);
                    outputs[Y_OUT].setVoltage(high);
                    outputs[W_OUT].setVoltage(high);
                    outputs[H_OUT].setVoltage(high);
                    outputs[D_OUT].setVoltage(high);
                    outputs[B_OUT].setVoltage(high);
                }
                lights[T_LIGHT].value = 1.f;
                lights[Y_LIGHT].value = 1.f;
                lights[W_LIGHT].value = 1.f;
                lights[H_LIGHT].value = 1.f;
                lights[D_LIGHT].value = 1.f;
                lights[B_LIGHT].value = 1.f;
                outputs[DNA_OUT].setVoltage(1.f);
                break;

            case 2: // C
                if (pulseActive) {
                    outputs[C_OUT].setVoltage(high);
                    outputs[Y_OUT].setVoltage(high);
                    outputs[S_OUT].setVoltage(high);
                    outputs[H_OUT].setVoltage(high);
                    outputs[V_OUT].setVoltage(high);
                    outputs[B_OUT].setVoltage(high);
                }
                lights[C_LIGHT].value = 1.f;
                lights[Y_LIGHT].value = 1.f;
                lights[S_LIGHT].value = 1.f;
                lights[H_LIGHT].value = 1.f;
                lights[V_LIGHT].value = 1.f;
                lights[B_LIGHT].value = 1.f;
                outputs[DNA_OUT].setVoltage(2.f);
                break;
    
            case 3: // G
                if (pulseActive) {
                    outputs[G_OUT].setVoltage(high);
                    outputs[R_OUT].setVoltage(high);
                    outputs[S_OUT].setVoltage(high);
                    outputs[D_OUT].setVoltage(high);
                    outputs[V_OUT].setVoltage(high);
                    outputs[B_OUT].setVoltage(high);
                }
                lights[G_LIGHT].value = 1.f;
                lights[R_LIGHT].value = 1.f;
                lights[S_LIGHT].value = 1.f;
                lights[D_LIGHT].value = 1.f;
                lights[V_LIGHT].value = 1.f;
                lights[B_LIGHT].value = 1.f;
                outputs[DNA_OUT].setVoltage(3.f);                
                break;
        }
            
        updateDisplays();        
    }

    void updateDisplays() {
        int displayCount = 23; //IMPORTANT, if text display changes size this has to be updated
        int halfWindow = displayCount / 2;

        for (int i = 0; i < displayCount; i++) {
            int offset = i - halfWindow;
            int geneIndex = (sequenceIndex + offset + geneSize - 1) % geneSize;

            char baseChar = 'N';
            switch (gene[geneIndex]) {
                case 0: baseChar = 'A'; break;
                case 1: baseChar = 'T'; break;
                case 2: baseChar = 'C'; break;
                case 3: baseChar = 'G'; break;
            }

            if (displayRibbon[i]) displayRibbon[i]->text = std::string(1, baseChar);

        }
    }
};

struct SequenceTextField : rack::ui::TextField {
    JunkDNA* module = nullptr;
    bool settingText = false;  // guard against recursive text changes

    SequenceTextField(JunkDNA* mod) : module(mod) {
        multiline = false;
        placeholder = "Enter sequence";
    }
    
    static std::string sanitizeSequence(const std::string& input) {
        std::string out;
        for (char c : input) {
            c = toupper(c);
            if (c == 'U') //Auto-switch RNA to DNA
                c = 'T';

            if (VALID_IUPAC.find(c) != std::string::npos) {
                out += c;
            } else {
                // Replace invalid char with 'N'
                out += 'N';
            }
        }
        return out;
    }

    void updateText(const std::string& newText) {
        if (settingText)
            return; // prevent recursion
        settingText = true;

        text = newText;
        cursor = (int)text.size();  // put cursor at the end safely
        if (module)
            module->sequenceText = newText;

        settingText = false;
    }

    void processSanitize() {
        if (!module)
            return;

        std::string sanitized = sanitizeSequence(text);
        if (sanitized != text) {
            updateText(sanitized);
        } else {
            // text unchanged, still update module
            module->sequenceText = sanitized;
        }
    }

    void onSelectKey(const SelectKeyEvent& e) override {
        rack::ui::TextField::onSelectKey(e);
        processSanitize();
    }

    void onButton(const ButtonEvent& e) override {
        rack::ui::TextField::onButton(e);
        processSanitize();
    }

};

DigitalDisplay* createDigitalDisplay(Vec position, std::string initialValue) {
    auto* display = new DigitalDisplay();
    display->box.pos = position;
    display->box.size = Vec(28.32, 17.76);
    display->text = initialValue;
    display->fgColor = nvgRGB(208, 140, 89);  // Gold text
    display->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
    display->setFontSize(14.0f);
    return display;
}

struct JunkDNAWidget : ModuleWidget {
    SequenceTextField* input = nullptr;

    JunkDNAWidget(JunkDNA* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/JunkDNA.svg"),
            asset::plugin(pluginInstance, "res/JunkDNA-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // --- Sequence text entry widget ---
        input = new SequenceTextField(module);
        input->box.pos = Vec(15, 30);
        input->box.size = mm2px(Vec(65.f, 12.f));
        if (module)
            input->text = module->sequenceText;
        addChild(input);
        
        // --- Display ribbon (redesigned for visibility in library) ---
        float baseYmm = 38.f;
        float centerXmm = 38.f;
        float yPx = mm2px(Vec(0, baseYmm)).y;
        float centerXPx = mm2px(Vec(centerXmm, 0)).x;
        
        // Estimated panel width in px (adjust to your panel if different)
        float panelWidthPx = mm2px(Vec(76.2f, 0)).x;
        float edgePaddingPx = 1.f;  // min space from left/right edge
        
        // Symmetrical font sizes
        std::vector<float> fontSizes;
        for (int i = 1; i <= 12; ++i) fontSizes.push_back(2*(float)i);
        fontSizes.push_back(30.f);  // boosted center
        for (int i = 12; i >= 1; --i) fontSizes.push_back(2*(float)i);

        int mid = static_cast<int>(fontSizes.size() / 2);        
        float leftXPx = centerXPx;
        float rightXPx = centerXPx;
        
        // Left side (reversed)

        for (int i = mid - 1; i >= 0; i--) {
            if (i == mid - 1) leftXPx -= 6.f;  // extra spacing before center
        
            float size = fontSizes[i];
            float charWidth = size * 0.6f;
            float offsetX = charWidth + 1.0f;
        
            leftXPx -= offsetX;
            if (leftXPx - charWidth / 2 < edgePaddingPx)
                continue;  // skip if it would overflow left
        
            DigitalDisplay* d = new DigitalDisplay();
            d->box.size = Vec(charWidth, size * 1.3f);
            d->box.pos = Vec(leftXPx - charWidth / 2, yPx);
            const char bases[] = {'G', 'C', 'A', 'T'};
            d->text = std::string(1, bases[std::rand() % 4]);
            d->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
            d->setFontSize(size);
            d->fgColor = nvgRGB(250 - 6 * i, 250 - 6 * i, 250 - 6 * i);
            addChild(d);
        
            if (module) module->displayRibbon[i] = d;
        }
        
        // Center display
        {
            float size = fontSizes[mid];
            float charWidth = size * 0.6f;
        
            DigitalDisplay* d = new DigitalDisplay();
            d->box.size = Vec(charWidth, size * 1.3f);
            d->box.pos = Vec(centerXPx - charWidth / 2, yPx);
            d->text = "C";
            d->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
            d->setFontSize(size);
            d->fgColor = nvgRGB(255, 180, 100);
            addChild(d);
        
            if (module) module->displayRibbon[mid] = d;
        }
        
        // Right side
        for (size_t i = mid + 1; i < fontSizes.size(); i++) {
            float size = fontSizes[i];
            float charWidth = size * 0.6f;
            float offsetX = charWidth + 1.0f;
        
            if (i == static_cast<size_t>(mid + 1)) rightXPx += 6.f;  // extra spacing after center
        
            rightXPx += offsetX;
            if (rightXPx + charWidth / 2 > panelWidthPx - edgePaddingPx)
                continue;  // skip if it would overflow right
        
            DigitalDisplay* d = new DigitalDisplay();
            d->box.size = Vec(charWidth, size * 1.3f);
            d->box.pos = Vec(rightXPx - charWidth / 2, yPx);
            const char bases[] = {'G', 'C', 'A', 'T'};
            d->text = std::string(1, bases[std::rand() % 4]);
            d->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
            d->setFontSize(size);
            d->fgColor = nvgRGB(250 - 10 * i, 250 - 10 * i, 250 - 10 * i);
        
            addChild(d);
            if (module) module->displayRibbon[i] = d;
        }

        addParam(createParamCentered<TL1105>(mm2px(Vec(13, 30.5)), module, JunkDNA::REV_BUTTON));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(6, 30.5)), module, JunkDNA::REV_IN));

        addParam(createParamCentered<TL1105>(mm2px(Vec(13, 50)), module, JunkDNA::RESET_BUTTON));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(6, 50)), module, JunkDNA::RESET_IN));

        addParam(createParamCentered<TL1105>(mm2px(Vec(62, 30.5)), module, JunkDNA::FWD_BUTTON));
        addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(70, 30.5)), module, JunkDNA::FWD_IN));

        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(70, 50)), module, JunkDNA::DNA_OUT));

        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(16.109, 64.81  )), module, JunkDNA::D_OUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(60.123, 64.857 )), module, JunkDNA::H_OUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(38.305, 73.407 )), module, JunkDNA::S_OUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(26.381, 75.482 )), module, JunkDNA::C_OUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(49.738, 75.482 )), module, JunkDNA::G_OUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(24.067, 87.143 )), module, JunkDNA::Y_OUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(38.112, 87.252 )), module, JunkDNA::N_OUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(52.094, 87.143 )), module, JunkDNA::R_OUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(26.381, 99.145 )), module, JunkDNA::T_OUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(49.738, 99.145 )), module, JunkDNA::A_OUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(38.305, 100.846)), module, JunkDNA::W_OUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(16.256, 109.829)), module, JunkDNA::V_OUT));
        addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(60.008, 109.574)), module, JunkDNA::B_OUT));

        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(16.109, 64.81  )), module, JunkDNA::D_LIGHT));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(60.123, 64.857 )), module, JunkDNA::H_LIGHT));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(38.305, 73.407 )), module, JunkDNA::S_LIGHT));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(26.381, 75.482 )), module, JunkDNA::C_LIGHT));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(49.738, 75.482 )), module, JunkDNA::G_LIGHT));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(24.067, 87.143 )), module, JunkDNA::Y_LIGHT));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(38.112, 87.252 )), module, JunkDNA::N_LIGHT));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(52.094, 87.143 )), module, JunkDNA::R_LIGHT));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(26.381, 99.145 )), module, JunkDNA::T_LIGHT));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(49.738, 99.145 )), module, JunkDNA::A_LIGHT));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(38.305, 100.846)), module, JunkDNA::W_LIGHT));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(16.256, 109.829)), module, JunkDNA::V_LIGHT));
        addChild(createLightCentered<LargeLight<WhiteLight>>(mm2px(Vec(60.008, 109.574)), module, JunkDNA::B_LIGHT));

    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
 
        JunkDNA* junkDNAModule = dynamic_cast<JunkDNA*>(module);
        assert(junkDNAModule); // Ensure the cast succeeds

         // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());
     
        struct GateOutputMenuItem : MenuItem {
            JunkDNA* junkDNAModule;
            void onAction(const event::Action& e) override {
                junkDNAModule->pulseOutput = !junkDNAModule->pulseOutput;
            }
            void step() override {
                rightText = junkDNAModule->pulseOutput ? "" : "✔";
                MenuItem::step();
            }
        };
    
        GateOutputMenuItem* gateOutputItem = new GateOutputMenuItem();
        gateOutputItem->text = "Output Pulses instead of Gates";
        gateOutputItem->junkDNAModule = junkDNAModule;
        menu->addChild(gateOutputItem);

    
        menu->addChild(new MenuSeparator());
        menu->addChild(createMenuLabel("IUPAC nucleotide codes"));
    
        static const std::vector<std::pair<std::string, std::string>> codes = {
            {"A", "Adenine"},
            {"T", "Thymine (or Uracil)"},
            {"C", "Cytosine"},
            {"G", "Guanine"},
            {"R", "A or G"},
            {"Y", "C or T"},
            {"S", "G or C"},
            {"W", "A or T"},
            {"K", "G or T"},
            {"M", "A or C"},
            {"B", "not A (C/G/T)"},
            {"D", "not C (A/G/T)"},
            {"H", "not G (A/C/T)"},
            {"V", "not T (A/C/G)"},
            {"N", "any base"}
        };
    
        for (const auto& pair : codes) {
            std::string label = pair.first + " — " + pair.second;
            menu->addChild(createMenuItem(label));
        }
    }


};

Model* modelJunkDNA = createModel<JunkDNA, JunkDNAWidget>("JunkDNA");
