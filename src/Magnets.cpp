////////////////////////////////////////////////////////////
//
//   Magnets
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   Simulates Ising glass and outputs 24 4-pole smoothed LFOs
//
////////////////////////////////////////////////////////////

#include "rack.hpp"
#include "plugin.hpp"
#include <random>
#include <algorithm> // For std::shuffle
#include <vector>    // For std::vector
using namespace rack;

// Assuming each light group represents a 5x5 section of the larger grid, and there's 25 such sections
#define GRID_WIDTH 25
#define GRID_HEIGHT 25
#define NUM_SECTIONS 25 // 25  5x5 sections

struct Magnets : Module {
    enum ParamIds {
        TEMP_PARAM,
        POLARIZATION_PARAM,
        INTERACTION_PARAM,
        UPDATE_INTERVAL_PARAM,
        TEMP_ATTENUATOR,
        POLARIZATION_ATTENUATOR,
        INTERACTION_ATTENUATOR,
        RESET_BUTTON,
        NUM_PARAMS
    };
    enum InputIds {
        HEAD_INPUT,
        RESET_INPUT,
        TEMP_INPUT,
        POLARIZATION_INPUT,
        INTERACTION_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        OUTPUTS_START, // Use this as the start index for dynamically generated outputs
        OUTPUTS_END = OUTPUTS_START + NUM_SECTIONS - 1, // End index for outputs
        NUM_OUTPUTS = NUM_SECTIONS
    };
    enum LightIds {
        LIGHTS_START, // Start index for red dynamically generated lights
        LIGHTS_END = LIGHTS_START + GRID_WIDTH * GRID_HEIGHT - 1, 
        NUM_LIGHTS = GRID_WIDTH * GRID_HEIGHT // Total number of lights 
    };

    // Initialize Random
    std::mt19937 eng{std::random_device{}()};
    std::uniform_real_distribution<float> distr{0.0f, 1.0f};

    // Initialize variables for trigger detection
    dsp::SchmittTrigger Reset;
    dsp::SchmittTrigger ResetBut;


    float resetCount=0;
    float head = 0.0f;
    float spinStates[625]={0.0f};
    float lastOutputStates[NUM_SECTIONS] = {}; // Last update cycle states
    float currentOutputStates[NUM_SECTIONS] = {}; // States for the current update cycle
    float outputInterpolationPhase = 0.f; // [0, 1] phase through the current update interval
    float phase = 0.f;

    float filteredOutputs[NUM_SECTIONS][4] = {{0.0f}}; // Four stages for each output section

    bool VoltRange = false;

    // Serialization method to save module state
    json_t* dataToJson() override {
        json_t* rootJ = json_object();

        // Save the state of VoltRange as a boolean
        json_object_set_new(rootJ, "VoltRange", json_boolean(VoltRange));

        return rootJ;
    }

    // Deserialization method to load module state
    void dataFromJson(json_t* rootJ) override {
        // Load the state of VoltRange
        json_t* VoltRangeJ = json_object_get(rootJ, "VoltRange");
        if (VoltRangeJ) { // Adds braces for consistency and future-proofing
            VoltRange = json_is_true(VoltRangeJ);
        }
    }
    
    Magnets() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        // Initialize parameters 
        configParam(TEMP_PARAM, 0.f, 1.f, 0.5f, "Temperature");
        configParam(POLARIZATION_PARAM, -1.f, 1.f, 0.f, "Polarization");
        configParam(INTERACTION_PARAM, 0.f, 1.f, 0.5f, "Interaction Strength");
        configParam(UPDATE_INTERVAL_PARAM, 0.15f, 2.f, 0.5f, "Update Interval", " ms");
		configParam(RESET_BUTTON, 0.0, 1.0, 0.0, "Reset" );

        // Initialize attenuators 
        configParam(TEMP_ATTENUATOR, -1.f, 1.f, 0.0f, "Temperature Attenuator");
        configParam(POLARIZATION_ATTENUATOR, -1.f, 1.f, 0.0f, "Polarization Attenuator");
        configParam(INTERACTION_ATTENUATOR, -1.f, 1.f, 0.0f, "Interaction Strength Attenuator");

        // Initialize inputs with labels
        configInput(HEAD_INPUT, "Tape Head");
        configInput(RESET_INPUT, "Reset");
        configInput(TEMP_INPUT, "Temperature CV");
        configInput(POLARIZATION_INPUT, "Polarization CV");
        configInput(INTERACTION_INPUT, "Interaction CV");

        // Initialize outputs with labels
        for (int i = 0; i < NUM_SECTIONS; ++i) {
            configOutput(OUTPUTS_START + i, "Zone " + std::to_string(i+1));
        }
        
        // Initialize all lights to off
        for (int i = 0; i < GRID_WIDTH * GRID_HEIGHT; ++i) {
            lights[LIGHTS_START + i].setBrightness(0.f);
        }

        for (int i = 0; i < 625; ++i) {
            spinStates[i] = distr(eng) > 0.5 ? 1.0f : -1.0f;
        }     
    }//Magnets()

    void process(const ProcessArgs &args) override {    
        // Read parameters and apply attenuations from CV inputs
        float temperature = params[TEMP_PARAM].getValue();
        float polarization = params[POLARIZATION_PARAM].getValue();
        float interactionStrength = params[INTERACTION_PARAM].getValue();
        float updateInterval = params[UPDATE_INTERVAL_PARAM].getValue();
        head = inputs[HEAD_INPUT].getVoltage() * 0.2f;
        head = clamp(head,-1.0f,1.0f);

        temperature += inputs[TEMP_INPUT].getVoltage() * 0.1 * params[TEMP_ATTENUATOR].getValue();
        polarization += inputs[POLARIZATION_INPUT].getVoltage() * 0.1 * params[POLARIZATION_ATTENUATOR].getValue();
        polarization += head;
        interactionStrength += inputs[INTERACTION_INPUT].getVoltage() * 0.1 * params[INTERACTION_ATTENUATOR].getValue();

        // Clamp the parameters to their expected ranges
        temperature = clamp(temperature, 0.f, 1.f);
        polarization = clamp(polarization, -1.f, 1.f);
        polarization = 0.5f * polarization + 0.5f;
        interactionStrength = clamp(interactionStrength, 0.f, 1.f);

        if ( Reset.process( inputs[RESET_INPUT].getVoltage() ) || ResetBut.process(params[RESET_BUTTON].getValue()) ) {
            resetSpinStates(polarization);
        }

        // Accumulate phase for Magnets model updates
        phase += args.sampleTime * 1000.0f; // Convert ms to seconds

        if (phase >= updateInterval) {
            phase -= updateInterval;

            std::uniform_int_distribution<int> distr_spin(0, GRID_WIDTH * GRID_HEIGHT - 1); // For selecting random spin

            // Reset the INPUT grid every update cycle
            resetInputGrid();

            // Example Magnets model update (single spin flip attempt per interval)
            int index = distr_spin(eng); // Randomly choose a spin
            int x = index % GRID_WIDTH;
            int y = index / GRID_WIDTH;

            // Calculate coordinates of nearest neighbors with periodic boundary conditions
            int left = ((x - 1 + GRID_WIDTH) % GRID_WIDTH) + y * GRID_WIDTH;
            int right = ((x + 1) % GRID_WIDTH) + y * GRID_WIDTH;
            int up = x + ((y - 1 + GRID_HEIGHT) % GRID_HEIGHT) * GRID_WIDTH;
            int down = x + ((y + 1) % GRID_HEIGHT) * GRID_WIDTH;

            // Calculate energy change if this spin were flipped
            float deltaE = 2 * interactionStrength * spinStates[index] * (spinStates[left] + spinStates[right] + spinStates[up] + spinStates[down]);

            //////////////
            // Metropolis criterion with a  polarization effect
            if (deltaE <= 0 || ( distr(eng) < exp(-deltaE / (temperature * 2.0f) ) ) ) {
                spinStates[index] *= -1; // Flip the spin

                //Let polarization bias spin states with probability proportional to the degree of polarization
                //This lets the array act like it's under a recorder head
                if( distr(eng)< (1 * abs(polarization - 0.5) ) ){  
                    if (polarization > 0.5){
                        spinStates[index]=1.0f;
                    } else if (polarization < 0.5) {
                        spinStates[index]=-1.0f;
                    }
                }
            }

//             // After updating, set the light states
//             for (int i = 0; i < GRID_WIDTH * GRID_HEIGHT; ++i) {
//                 int lightIndex = LIGHTS_START + i;
//                 bool spinUp = spinStates[i] > 0;
//                 lights[lightIndex].setBrightness(spinUp ? 1.f : 0.f);
//             }         
                        
            phase = 0.f; // Reset phase for the next interval
            outputInterpolationPhase = 0.f; // Reset interpolation phase

            // Reset current states for new calculation
            for (int i = 0; i < NUM_SECTIONS; ++i) {
                lastOutputStates[i] = currentOutputStates[i];
                currentOutputStates[i] = 0.f; // Will be recalculated
            }

            // Iterate over all spin states to update corresponding section output states
            for (int y = 0; y < GRID_HEIGHT; ++y) {
                for (int x = 0; x < GRID_WIDTH; ++x) {
                    // Calculate the linear index correctly using zero-based indexing
                    int index = y * GRID_WIDTH + x;

                    // Calculate the section index. Adjust for sections beyond the central exclusion
                    int sectionX = x / 5;
                    int sectionY = y / 5;
                    int section = sectionY * 5 + sectionX;

                    // Safeguard to prevent out-of-range access
                    if (section < 0 || section >= NUM_SECTIONS) continue;

                    // Update the section's current output state based on spin state
                    currentOutputStates[section] += (spinStates[index] > 0) ? 1.f : -1.f;
                }
            }

            // Normalize the current output states based on the number of spins per section
            for (int i = 0; i < NUM_SECTIONS; ++i) {
                currentOutputStates[i] /= 25.f; // Assuming each section (except central) has 25 spins
            }
 
        }
                
         // Calculate dynamic cutoff frequency for low-pass filter based on updateInterval
        float minInterval = 0.1f, maxInterval = 2.0f;
        float minFc = 5.0f, maxFc = 20.0f;
        float normalizedInterval = (updateInterval - minInterval) / (maxInterval - minInterval);
        float dynamicFc = minFc + (maxFc - minFc) * (1.0f - normalizedInterval); // Linear interpolation between minFc and maxFc
        float sampleRate = args.sampleRate;
        float dt = 1.0f / sampleRate;
        float rc = 1.0f / (2.0f * M_PI * dynamicFc);
        float alpha = dt / (rc + dt);

        // Update interpolation phase and apply low-pass filter to output values
        float interpolationStep = args.sampleTime * 1000.0f / updateInterval;
        outputInterpolationPhase += interpolationStep;
        outputInterpolationPhase = clamp(outputInterpolationPhase, 0.f, 1.f);

        for (int i = 0; i < NUM_SECTIONS; ++i) {
            if (i == 12) continue; // Skip central section if necessary

            float interpolatedValue = crossfade(lastOutputStates[i], currentOutputStates[i], outputInterpolationPhase) * 10.f;

            // Apply four stages of low-pass filtering
            for (int stage = 0; stage < 4; ++stage) {
                filteredOutputs[i][stage] = alpha * interpolatedValue + (1.0f - alpha) * filteredOutputs[i][stage];
                interpolatedValue = filteredOutputs[i][stage]; // Use output of current stage as input to next
            }

            if (VoltRange){
                outputs[OUTPUTS_START + i].setVoltage(interpolatedValue / 2.0f ); // Scale output of the last stage
            } else {
                outputs[OUTPUTS_START + i].setVoltage(interpolatedValue); // Use the output of the last stage
            }
 
        }
        // Correct the central lights of each 5x5 section except the actual central grid
        for (int sectionY = 0; sectionY < 5; sectionY++) {
            for (int sectionX = 0; sectionX < 5; sectionX++) {
                // Skip the central grid
                if (sectionX == 2 && sectionY == 2) continue;

                int centralLightIndex = (sectionY * 5 + 2) * GRID_WIDTH + (sectionX * 5 + 2);
                lights[LIGHTS_START + centralLightIndex].setBrightness(currentOutputStates[sectionY*5 + sectionX]); 
            }
        }
    }//void process
    
    void resetSpinStates(float polarization) {
        int indexes[600]; // array for indexes
        int indexCount = 0; // Keep track of the actual number of indexes used

        // Fill with indexes, excluding the central grid
        for (int i = 0; i < 625; ++i) {
            if (!((i % 25 >= 10 && i % 25 < 15) && (i / 25 >= 10 && i / 25 < 15))) {
                indexes[indexCount++] = i;
            }
        }

        // Now shuffle the used portion of the indexes array manually
        for (int i = 0; i < indexCount - 1; ++i) {
            std::uniform_int_distribution<int> distribution(i, indexCount - 1);
            int j = distribution(eng);
            // Swap indexes[i] and indexes[j]
            int temp = indexes[i];
            indexes[i] = indexes[j];
            indexes[j] = temp;
        }

        // Now reset spinStates based on the shuffled indexes
        for (int idx = 0; idx < indexCount; ++idx) {
            spinStates[indexes[idx]] = (distr(eng) < polarization) ? 1.0f : -1.0f;
        }
    }//void resetSpinStates
    
    void resetInputGrid() {
        // Calculate target polarization from HEAD_INPUT voltage
        float targetPolarization = (inputs[HEAD_INPUT].getVoltage() / 5.0f); 
        targetPolarization = clamp(targetPolarization, -1.0f, 1.0f);
    
        // Define the bounds of the central 5x5 grid within the 25x25 layout
        int startX = 10, endX = 15; // Horizontal bounds for the central grid
        int startY = 10, endY = 15; // Vertical bounds for the central grid

        // Calculate current average polarization of the central grid
        float currentAverage = 0.0f;
        for (int y = startY; y < endY; ++y) {
            for (int x = startX; x < endX; ++x) {
                int idx = y * GRID_WIDTH + x;
                currentAverage += spinStates[idx];
            }
        }
        currentAverage /= 25.0f; // There are 25 points in the 5x5 grid

        // Pick a random position within the central grid
        std::uniform_int_distribution<int> distX(startX, endX - 1);
        std::uniform_int_distribution<int> distY(startY, endY - 1);
        int randomX = distX(eng);
        int randomY = distY(eng);
        int randomIdx = randomY * GRID_WIDTH + randomX;

        // Set the state of the randomly chosen position
        if (currentAverage < targetPolarization) {
            spinStates[randomIdx] = 1.0f; // Set to +1 to increase average towards target
        } else {
            spinStates[randomIdx] = -1.0f; // Set to -1 to decrease average towards target
        }
    }//void resetInputGrid()
};//module

struct MagnetsWidget : ModuleWidget {
    MagnetsWidget(Magnets* module) {
        setModule(module);

        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Magnets.svg"),
            asset::plugin(pluginInstance, "res/Magnets-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // Layout parameters
        const Vec gridStartPos = Vec(122.0f, 45.0f); // Starting position for the grid on the panel
        const Vec gridSpacing = Vec(63.0f, 63.0); // Spacing between sections
        const Vec lightSpacing = Vec(12.6f, 12.6f); // Spacing for lights within each section
        const float smallShift = 3.0f; // Small shift amount for lights adjacent to the output

        // Create the grid sections with lights
        for (int sectionY = 0; sectionY < 5; sectionY++) {
            for (int sectionX = 0; sectionX < 5; sectionX++) {
                // Calculate the starting position for this section
                Vec sectionStartPos = gridStartPos.plus(gridSpacing.mult(Vec(sectionX, sectionY)));
                // Loop to create the lights for each section
                for (int y = 0; y < 5; y++) {
                    for (int x = 0; x < 5; x++) {
                        Vec lightPos = sectionStartPos.plus(lightSpacing.mult(Vec(x, y)));
                        float ShiftAmount = smallShift; 
                        if (sectionX ==2 && sectionY==2){ //Don't shift lights in the middle section
                            ShiftAmount=0.0f;
                        } else{
                            ShiftAmount= smallShift;
                        }
                        // Shift lights adjacent to the central output
                        if ((x == 1 || x == 3) && y == 2) lightPos.x += (x - 2) * ShiftAmount;
                        if ((y == 1 || y == 3) && x == 2) lightPos.y += (y - 2) * ShiftAmount;
 
                        // Corrected index calculation
                        int index = (sectionY * 5 * GRID_WIDTH) + (sectionX * 5) + (y * GRID_WIDTH) + x;

                        // Add a YellowLight for the center light of each grid unit, WhiteLight for others
                        if ( (x == 2 && y == 2) && !( sectionX==2 && sectionY==2 ) ) {
                            addChild(createLightCentered<MediumLight<YellowLight>>(lightPos, module, Magnets::LIGHTS_START + index));
                        } else {
                            addChild(createLightCentered<TinyLight<WhiteLight>>(lightPos, module, Magnets::LIGHTS_START + index));
                        }
                    }
                }
                // Add output ports for all sections except the central one
                if (!(sectionX == 2 && sectionY == 2)) {
                    Vec outputPos = sectionStartPos.plus(lightSpacing.mult(Vec(2, 2)));
                    addOutput(createOutputCentered<ThemedPJ301MPort>(outputPos, module, Magnets::OUTPUTS_START + sectionX + sectionY * 5));
                } else {
                    // Logic for the central section (without an output port) if needed
                }
            }
        }  
              
         // Control positions
        const Vec column1Pos = Vec(30.0f, 60.0f); // Position for column 1
        const Vec column2Pos = Vec(80.0f, 60.0f); // Position for column 2
        const float verticalSpacing = 32.5f; // Vertical spacing between controls
        // Add controls and inputs for column 1
        addParam(createParamCentered<RoundBlackKnob>(Vec(column1Pos.x, column1Pos.y), module, Magnets::TEMP_PARAM));
        addParam(createParamCentered<Trimpot>(Vec(column1Pos.x, column1Pos.y + verticalSpacing + 3), module, Magnets::TEMP_ATTENUATOR));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(column1Pos.x, column1Pos.y + 2 * verticalSpacing), module, Magnets::TEMP_INPUT));
        addParam(createParamCentered<RoundBlackKnob>(Vec(column1Pos.x, column1Pos.y + 4 * verticalSpacing), module, Magnets::UPDATE_INTERVAL_PARAM));
        // Head input at the blightIndexottom of column 1
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(column1Pos.x, column1Pos.y + 8 * verticalSpacing), module, Magnets::HEAD_INPUT));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(column2Pos.x, column1Pos.y + 8 * verticalSpacing), module, Magnets::RESET_INPUT));
        //Add button for Reset
        addParam(createParamCentered<TL1105>(Vec(column2Pos.x, column1Pos.y + 7.25 * verticalSpacing ), module, Magnets::RESET_BUTTON));

        // Add controls and inputs for column 2
        addParam(createParamCentered<RoundBlackKnob>(Vec(column2Pos.x, column2Pos.y), module, Magnets::POLARIZATION_PARAM));
        addParam(createParamCentered<Trimpot>(Vec(column2Pos.x, column2Pos.y + verticalSpacing + 3), module, Magnets::POLARIZATION_ATTENUATOR));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(column2Pos.x, column2Pos.y + 2 * verticalSpacing), module, Magnets::POLARIZATION_INPUT));
        addParam(createParamCentered<RoundBlackKnob>(Vec(column2Pos.x, column2Pos.y + 4 * verticalSpacing), module, Magnets::INTERACTION_PARAM));
        addParam(createParamCentered<Trimpot>(Vec(column2Pos.x, column2Pos.y + 5 * verticalSpacing + 3), module, Magnets::INTERACTION_ATTENUATOR));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(column2Pos.x, column2Pos.y + 6 * verticalSpacing), module, Magnets::INTERACTION_INPUT));       
    }
    
    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);

        Magnets* MagnetsModule = dynamic_cast<Magnets*>(module);
        assert(MagnetsModule); // Ensure the cast succeeds

        // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());

        // Retriggering enabled/disabled menu item
        struct VoltRangeMenuItem : MenuItem {
            Magnets* MagnetsModule;
            void onAction(const event::Action& e) override {
                MagnetsModule->VoltRange = !MagnetsModule->VoltRange;
            }
            void step() override {
                rightText = MagnetsModule->VoltRange ? "✔" : "";
                MenuItem::step();
            }
        };

        VoltRangeMenuItem* item = new VoltRangeMenuItem();
        item->text = "Voltage Range ±5V";
        item->MagnetsModule = MagnetsModule; // Ensure we're setting the module
        menu->addChild(item);
    }

    void draw(const DrawArgs& args) override {
        ModuleWidget::draw(args);
        Magnets* module = dynamic_cast<Magnets*>(this->module);
        if (!module) return;

        // After updating, set the light states
        for (int i = 0; i < GRID_WIDTH * GRID_HEIGHT; ++i) {
            int lightIndex = module->LIGHTS_START + i;
            bool spinUp = module->spinStates[i] > 0;
            module->lights[lightIndex].setBrightness(spinUp ? 1.f : 0.f);
        }         
    } 
      
};

Model* modelMagnets = createModel<Magnets, MagnetsWidget>("Magnets");