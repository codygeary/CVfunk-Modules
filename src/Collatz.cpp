////////////////////////////////////////////////////////////
//
//   Collatz
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   Turns Collatz sequences into polyrhythms
//
////////////////////////////////////////////////////////////


#include "rack.hpp"
#include "plugin.hpp"
#include "digital_display.hpp"

using namespace rack;

// Define our Module derived from Rack's Module class
struct Collatz : Module {
    enum ParamIds {
        START_NUMBER,
        START_NUMBER_ATT,
        RESET_BUTTON_PARAM,
        BEAT_MODULUS,
        BEAT_MODULUS_ATT,
        START_BUTTON_PARAM,
        NUM_PARAMS
    };
    enum InputIds {
        CLOCK_INPUT,
        START_NUMBER_CV,
        BEAT_MODULUS_CV,
        RESET_INPUT,
        START_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        GATE_OUTPUT,
        ACCENT_OUTPUT,
        COMPLETION_OUTPUT,
        NUM_OUTPUTS
    };
    enum LightIds {
        RUNNING_LIGHT,
        COMPLETION_LIGHT,
        GATE_LIGHT,
        ACCENT_LIGHT,
        RUN_LIGHT,
        NUM_LIGHTS
    };

    dsp::SchmittTrigger clockTrigger, resetTrigger, triggerTrigger, sampleTrigger;
    int currentNumber = 0;
    bool sequenceRunning = false;
    int rhythmStepIndex = 0;
    int modNumber = 0;
    DigitalDisplay* digitalDisplay = nullptr;
    DigitalDisplay* modNumberDisplay = nullptr; 
    float lastClockTriggerTime = -1.0f;
    float clockRate = 1.0f; 
    float lastClockTime = 1.0f;
    float internalClockTime = 1.0f;
    bool firstPulseReceived = false;
    bool sequenceTriggered = false;
    int steps = 0;
    int accents = 0;
    int beatMod = 0;

    float accumulatedTime = 0.0f;
    float accumulatedTimeB = 0.0f;
    
    float gatePulse=0.0f;
    float accentPulse=0.0f;

    Collatz() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        configParam(START_NUMBER, 2.f, 1000.f, 5.f, "Starting Number");
        configParam(BEAT_MODULUS, 1.f, 100.f, 24.f, "Beat Modulus");
        configParam(START_NUMBER_ATT, -1.f, 1.f, 0.f, "Starting Number Attenuation");
        configParam(BEAT_MODULUS_ATT, -1.f, 1.f, 0.f, "Beat Modulus Attenuation");

        configParam(RESET_BUTTON_PARAM, 0.f, 1.f, 0.f, "Reset");
        configParam(START_BUTTON_PARAM, 0.f, 1.f, 0.f, "Start");

        // Configuring inputs
        configInput(CLOCK_INPUT, "Clock");
        configInput(RESET_INPUT, "Reset");
        configInput(START_INPUT, "Start");
        configInput(START_NUMBER_CV, "Starting Number CV");
        configInput(BEAT_MODULUS_CV, "Beat Modulus CV");

        // Configuring outputsBEAT_MODULUS
        configOutput(GATE_OUTPUT, "Gate Output");
        configOutput(ACCENT_OUTPUT, "Accent Output");
        configOutput(COMPLETION_OUTPUT, "Sequence Completion");
        configLight(COMPLETION_LIGHT, "Completion Indicator");
    }

    // Additional method signatures
    void process(const ProcessArgs& args) override;
    void advanceSequence();
};

void Collatz::process(const ProcessArgs& args) {

    // Calculate the potential starting number every cycle
    float knobValue = params[START_NUMBER].getValue();
    float cvValue = inputs[START_NUMBER_CV].isConnected() ?
                    inputs[START_NUMBER_CV].getVoltage() : 0.0f;
    cvValue *= params[START_NUMBER_ATT].getValue();
    int startingNumber = std::abs(static_cast<int>((knobValue + 100*cvValue)));

    // Calculate the potential starting number every cycle
    float beatModIN = params[BEAT_MODULUS].getValue();
    float beatModAtt = params[BEAT_MODULUS_ATT].getValue();
    float beatModCV = inputs[BEAT_MODULUS_CV].isConnected() ?
                    inputs[BEAT_MODULUS_CV].getVoltage() : 0.0f;
    beatMod = std::abs(static_cast<int>(beatModIN+beatModAtt*10*beatModCV));
   
    if (currentNumber == 0){
        modNumber = startingNumber % beatMod;
        steps = modNumber;
        if (modNumber<1) {accents = 0;} //avoid divide by zero 
        else { accents = floor((currentNumber/modNumber) % beatMod);}
    }

    // Display update logic
    if (digitalDisplay) {
        if (sequenceRunning) {
            digitalDisplay->text = std::to_string(currentNumber) + " mod " + std::to_string(beatMod);
            modNumber = currentNumber % beatMod;
            steps = modNumber ;
            if (modNumber<1) {accents = 0;} //avoid divide by zero 
            else { accents = floor((currentNumber/modNumber) % beatMod);}

            if (modNumberDisplay) {
                std::string displayText = std::to_string(steps) + " : " + std::to_string(accents);
                modNumberDisplay->text = displayText; 
            }
            outputs[COMPLETION_OUTPUT].setVoltage(0.0f);
            lights[COMPLETION_LIGHT].setBrightness(0);
        } else {
            digitalDisplay->text = std::to_string(startingNumber) + " mod " + std::to_string(beatMod);
            modNumber = startingNumber % beatMod;
            steps = modNumber;
             if (modNumber<1) {//avoid divide by zero 
                accents = 0;
            } else {
                accents = floor((startingNumber/modNumber) % beatMod);
            }
                     
            if (modNumberDisplay) {
                std::string displayText = std::to_string(steps) + " : " + std::to_string(accents) ;
                modNumberDisplay->text = displayText; 
            }
            
            outputs[COMPLETION_OUTPUT].setVoltage(5.0f);
            lights[COMPLETION_LIGHT].setBrightness(1);
        }
    } 

    // Handle reset logic
    if (resetTrigger.process(params[RESET_BUTTON_PARAM].getValue()) || 
        resetTrigger.process(inputs[RESET_INPUT].getVoltage()-0.01f)) {
        sequenceRunning = false;
        rhythmStepIndex = 0;
        currentNumber = 0;
        lights[RUN_LIGHT].setBrightness(0);      
        outputs[GATE_OUTPUT].setVoltage(0.0f);
        outputs[ACCENT_OUTPUT].setVoltage(0.0f);
        lights[GATE_LIGHT].setBrightness(0);
        lights[ACCENT_LIGHT].setBrightness(0);
    }

    // Handle trigger logic
    if ( (sampleTrigger.process(inputs[START_INPUT].getVoltage()) || params[START_BUTTON_PARAM].getValue() > 0) && !sequenceRunning && !sequenceTriggered) {
        sequenceTriggered = true;  // Mark that a sequence start is pending
         lights[RUN_LIGHT].setBrightness(1);      
    }
        
    // Clock handling logic
    bool externalClockConnected = inputs[CLOCK_INPUT].isConnected();
    if (externalClockConnected && clockTrigger.process(inputs[CLOCK_INPUT].getVoltage()-0.01f)) {
        if (sequenceTriggered) {
            // Reset necessary variables for starting the sequence
            currentNumber = startingNumber;
            sequenceRunning = true;
            sequenceTriggered = false; // Reset trigger flag after starting the sequence
            rhythmStepIndex = 0; // Reset rhythm index if needed
        } else if (sequenceRunning ) {
            advanceSequence();
        }

        // Update lastClockTime for rate calculation
        if (firstPulseReceived) {clockRate = 1.0f / lastClockTime;}
        lastClockTime = 0.0f;
        firstPulseReceived = true;
    } 

    // Accumulate time since the last clock pulse for rate calculation
    if (firstPulseReceived && externalClockConnected) {
        lastClockTime += args.sampleTime;
    } 
     
    // rhythm and output logic
    if (sequenceRunning) {
        steps = modNumber ;
        if (modNumber<1) {accents = 0;} //avoid divide by zero 
        else { accents = floor((currentNumber/modNumber) % beatMod);}

        accumulatedTime += args.sampleTime;
        accumulatedTimeB += args.sampleTime;
        if (steps>=1){ //avoid divide by zero 
                float stepDuration = 1.0f / clockRate / steps; 
                if (accumulatedTime < stepDuration/2) {
                    gatePulse=5;
                } else {gatePulse=0;}
                if (accumulatedTime >= stepDuration) {
                    accumulatedTime -= stepDuration;
                }
        } else {
                float stepDuration = 1.0f / clockRate / 1.0f; //avoid div by zero 
                if (accumulatedTime < stepDuration/2) {
                    gatePulse=5;
                } else {gatePulse=0;}
                if (accumulatedTime >= stepDuration) {
                    accumulatedTime -= stepDuration;
                }
        }

        if (accents>=1){ //avoid divide by zero 
                float accentDuration = 1.0f / clockRate / accents;
                if (accumulatedTimeB < accentDuration/2) {
                    accentPulse=5;
                } else {accentPulse=0;}
                if (accumulatedTimeB >= accentDuration) {
                    accumulatedTimeB -= accentDuration;
                } 
        } else {
                float accentDuration = 1.0f / clockRate / 1.0f;
                if (accumulatedTimeB < accentDuration/2) {
                    accentPulse=5;
                } else {accentPulse=0;}
                if (accumulatedTimeB >= accentDuration) {
                    accumulatedTimeB -= accentDuration;
                } 
        }

        if (externalClockConnected){
        // Set gate and accent outputs
            //for step or accents =0 suppress outputs
            if (steps>=1){outputs[GATE_OUTPUT].setVoltage(gatePulse);
            }else {outputs[GATE_OUTPUT].setVoltage(0.0f);}
            if (accents>=1){outputs[ACCENT_OUTPUT].setVoltage(accentPulse);
            }else {outputs[ACCENT_OUTPUT].setVoltage(0.0f);}
            if (steps>=1){lights[GATE_LIGHT].setBrightness(gatePulse/5);
            }else {lights[GATE_LIGHT].setBrightness(0);}
            if (accents>=1){lights[ACCENT_LIGHT].setBrightness(accentPulse/5);
            }else {lights[ACCENT_LIGHT].setBrightness(0);}
        } else {
            outputs[GATE_OUTPUT].setVoltage(0.0f);
            outputs[ACCENT_OUTPUT].setVoltage(0.0f);
            lights[GATE_LIGHT].setBrightness(0);
            lights[ACCENT_LIGHT].setBrightness(0);
            firstPulseReceived = false;
        }   
    }// if (sequenceRunning)
}//void Collatz::process

void Collatz::advanceSequence() {
    if (currentNumber <= 0) {
        sequenceRunning = false;
        return;
    }

    if (currentNumber == 1) {
        sequenceRunning = false;              
        modNumber=0;
        steps = 1;
        firstPulseReceived = true; 
        rhythmStepIndex = 0;
         lights[RUN_LIGHT].setBrightness(0);      
        
        accumulatedTime =0.0f;
        accumulatedTimeB =0.0f;

        lights[GATE_LIGHT].setBrightness(0);
        lights[ACCENT_LIGHT].setBrightness(0);
        outputs[GATE_OUTPUT].setVoltage(0.0f);
        outputs[ACCENT_OUTPUT].setVoltage(0.0f);
    
        return;
    }

    // Collatz sequence logic
    if (currentNumber % 2 == 0) {
        currentNumber /= 2;
        
    } else {
        currentNumber = 3 * currentNumber + 1;
    }

    modNumber = currentNumber % beatMod;
    steps = modNumber ;
    if (modNumber<1) {accents = 0;} //avoid divide by zero 
    else { accents = floor((currentNumber/modNumber) % beatMod);} 
}

struct CollatzWidget : ModuleWidget {
    CollatzWidget(Collatz* module) {
        setModule(module);

        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Collatz.svg"),
            asset::plugin(pluginInstance, "res/Collatz-dark.svg")
        ));

        // Add screws or additional design elements as needed
        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        box.size = Vec(8 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT); // 8HP wide module

        // Knobs
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10, 28.738+7.5)), module, Collatz::START_NUMBER));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(30, 28.738+7.5)), module, Collatz::BEAT_MODULUS));

        addParam(createParamCentered<Trimpot>(mm2px(Vec(10, 41.795+7)), module, Collatz::START_NUMBER_ATT));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(30, 41.795+7)), module, Collatz::BEAT_MODULUS_ATT));

        // Inputs
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10, 60.194)), module, Collatz::START_NUMBER_CV));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(30, 60.194)), module, Collatz::BEAT_MODULUS_CV));

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10,  93.125)), module, Collatz::CLOCK_INPUT));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(30,  93.125)), module, Collatz::START_INPUT));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20,  93.125)), module, Collatz::RESET_INPUT));

        // Buttons
        addParam(createParamCentered<LEDButton>(mm2px(Vec(30, 82)), module, Collatz::START_BUTTON_PARAM));
        addParam(createParamCentered<LEDButton>(mm2px(Vec(20, 82)), module, Collatz::RESET_BUTTON_PARAM));
    
        // Outputs 
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10, 112.3)), module, Collatz::GATE_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(20, 112.3)), module, Collatz::ACCENT_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(30, 112.3)), module, Collatz::COMPLETION_OUTPUT));
        addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(30,  105.867)), module, Collatz::COMPLETION_LIGHT));
        addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(20,   105.867)), module, Collatz::ACCENT_LIGHT));
        addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(10,   105.867)), module, Collatz::GATE_LIGHT));

        addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(30, 75)), module, Collatz::RUN_LIGHT));

         // Configure and add the first digital display
        DigitalDisplay* digitalDisplay = new DigitalDisplay();
        digitalDisplay->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        digitalDisplay->box.pos = Vec(10, 34); // Position on the module
        digitalDisplay->box.size = Vec(100, 18); // Size of the display
        digitalDisplay->text = "Collatz"; // Initial text
        digitalDisplay->fgColor = nvgRGB(208, 140, 89); // White color text
        digitalDisplay->textPos = Vec(0, 15); // Text position
        digitalDisplay->setFontSize(16.0f); // Set the font size as desired
        addChild(digitalDisplay);

        if (module) {
            module->digitalDisplay = digitalDisplay; // Link the module to the display
        }

        // Configure and add the second digital display for modNumber
        DigitalDisplay* modNumberDisplay = new DigitalDisplay();
        modNumberDisplay->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        modNumberDisplay->box.pos = Vec(10, 50); // Position below the first display
        modNumberDisplay->box.size = Vec(100, 18); // Size of the display
        modNumberDisplay->text = "Beats : Accents"; // Initial text or placeholder
        modNumberDisplay->fgColor = nvgRGB(208, 140, 89); // White color text
        modNumberDisplay->textPos = Vec(0, 15); // Text position
        modNumberDisplay->setFontSize(12.0f); // Set the font size as desired

        addChild(modNumberDisplay);

        if (module) {
            module->modNumberDisplay = modNumberDisplay;
        }
    }
};

Model* modelCollatz = createModel<Collatz, CollatzWidget>("Collatz");