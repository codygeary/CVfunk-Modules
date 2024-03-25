////////////////////////////////////////////////////////////
//
//   Strings
//
//   written by Cody Geary
//   Copyright 2024, MIT License
//
//   Outputs guitar chords
//
////////////////////////////////////////////////////////////


#include "rack.hpp"
#include "plugin.hpp"
#include <string>
#include <array>
#include "dsp/digital.hpp"
#include "digital_display.hpp"  
#include "chord_display.hpp"
using namespace rack;

const int MAX_CHORDS_PER_ROW = 7;
const int MAX_ROWS = 4;

// Base frequencies for each guitar string
const float baseFrequencies[6] =  { -1.666f, // E2
                                    -1.25f, // A2
                                    -0.833f, // D3
                                    -0.417f,  // G3
                                    -0.084f, // B3
                                     0.3333f};// E4



// Helper function to convert a fingering (e.g., "X21202") to semitone shifts
std::array<int, 6> fingeringToSemitoneShifts(const std::string& fingering) {
    std::array<int, 6> semitoneShifts = {0, 0, 0, 0, 0, 0}; // Initialize shifts
    for (size_t i = 0; i < 6; ++i) {
        if (fingering[i] == 'X' || i >= fingering.length()) {
            semitoneShifts[i] = -1; // Muted string, set to -1 to indicate mute
        } else {
            // Convert character to integer and subtract base fret offset
            semitoneShifts[i] = fingering[i] - '0';
        }
    }
    return semitoneShifts;
}

struct Strings : Module {

    enum ParamIds {
        CHORD_SELECTOR_PARAM,
        ROW_SELECTOR_PARAM,
        BARRE_CHORD_BUTTON,
        ALT_CHORD_BUTTON,
        CAPO_PARAM,
        CHORD_BUTTON_1,
        NUM_PARAMS = CHORD_BUTTON_1 + MAX_CHORDS_PER_ROW * MAX_ROWS
    };
    enum InputIds {
        CHORD_SELECTOR_CV,
        ROW_SELECTOR_CV,
        CAPO_CV,
        BARRE_CHORD_GATE,
        ALT_CHORD_GATE,
        ENVELOPE_IN_1,
        ENVELOPE_IN_2,
        ENVELOPE_IN_3,
        ENVELOPE_IN_4,
        ENVELOPE_IN_5,
        ENVELOPE_IN_6,
        WHAMMY_BAR_CV,
        NUM_INPUTS
    };
    enum OutputIds {
        STRING_CV_OUT_1,
        STRING_CV_OUT_2,
        STRING_CV_OUT_3,
        STRING_CV_OUT_4,
        STRING_CV_OUT_5,
        STRING_CV_OUT_6,
        MUTE_OUT_1,
        MUTE_OUT_2,
        MUTE_OUT_3,
        MUTE_OUT_4,
        MUTE_OUT_5,
        MUTE_OUT_6,
        ROOT_NOTE_CV_OUT,
        TRIGGER_OUT,
        NUM_OUTPUTS
    };
    enum LightIds {
        BARRE_CHORD_LIGHT,
        ALT_CHORD_LIGHT,
        CHORD_SELECTION_LIGHT_1 = ALT_CHORD_LIGHT + 6, // Followed by 6 active string lights
        NUM_LIGHTS = CHORD_SELECTION_LIGHT_1 + (MAX_CHORDS_PER_ROW * MAX_ROWS) // 7 chords per row * 4 rows = 28 chord selection lights
    };
    
        // Direct initialization of Chords Arrays
        // Define chord names for row 1
        const std::array<std::array<std::string, 4>, 7> Row1_Names = {{
            {"B7", "B"     , "Bsus4", "Badd9"},
            {"E7", "E"     , "Esus4", "Eadd9"},
            {"A7", "A7-Bar", "Asus4", "Aadd9"},
            {"D7", "D7-Bar", "Dsus4", "Dadd9"},
            {"G7", "G7-Bar", "Gsus4", "Gadd9"},
            {"C7", "C7-Bar", "Csus4", "Cadd9"},
            {"F7", "F7-Bar", "Fsus4", "Fadd9"}
        }};
        // Define chord fingerings for row 1
        const std::array<std::array<std::string, 4>, 7> Row1_Chords = {{
             {"X21202", "X24442", "X24452", "744647"},
             {"020100", "022100", "022200", "024100"},
             {"X02020", "575685", "X00230", "X02420"},
             {"XX0212", "X5453X", "XX0233", "X54252"},
             {"320001", "353433", "330013", "300003"},
             {"X32310", "X35353", "X33013", "X32033"},
             {"101211", "131211", "113311", "103013"}
        }};
        // Define root notes for row 1
        const std::array<float, 7> Row1_Roots = {0.917f, 0.3333f, 0.75f, 0.1667f, 0.5833f, 1.0f, 0.4167f};

        const std::array<std::array<std::string, 4>, 7> Row2_Names = {{
            {"A" ,"A-Bar" ,"Amaj7" ,"Aaug"}, 
            {"D" ,"D-Bar" ,"Dmaj7" ,"Caug"}, 
            {"G" ,"G-Bar" ,"Gmaj7" ,"Gaug"}, 
            {"C" ,"C-Bar" ,"Cmaj7" ,"Caug"}, 
            {"F" ,"F-Bar" ,"Fmaj7" ,"Faug"}, 
            {"Bb","Bb-Bar","Bbmaj7","Bbaug"},
            {"Eb","Eb-Bar","Ebmaj7","Ebaug"}
        }};
        const std::array<std::array<std::string, 4>, 7> Row2_Chords = {{
            {"X02220", "577655", "X02120", "X03221"}, 
            {"XX0232", "X57775", "XX0222", "XX0131"}, 
            {"320003", "355433", "320002", "XX5323"}, 
            {"X32010", "X35553", "X32000", "X3454X"}, 
            {"133211", "133211", "102210", "12310X"}, 
            {"X10331", "X13331", "X10231", "X12320"}, 
            {"XX1343", "X68886", "XX1333", "XX1242"} 
        }};
        const std::array<float, 7> Row2_Roots = { 0.75f, 0.167f, 0.583f, 1.0f, 0.417f, 0.833f ,0.25f };

        const std::array<std::array<std::string, 4>, 7> Row3_Names = {{
            {"Em" ,"Em-Bar" ,"Em7"    ,"Em6"},    
            {"Am" ,"Am-Bar" ,"Am7"    ,"Am6"},    
            {"Dm" ,"Dm-Bar" ,"Dm7"    ,"Dm6"},    
            {"Gm" ,"Gm-Bar" ,"Gm7"    ,"Gm6"},    
            {"Cm" ,"Cm-Bar" ,"Cm7"    ,"Cm6"},    
            {"Bb7","Bb7-Bar","Bbm7","Bb7sus2"},  
            {"Ab" ,"Ab-Bar" ,"Ab7","Ab6sus2"}  
        }};
        const std::array<std::array<std::string, 4>, 7> Row3_Chords = {{
            {"022000", "X79987", "020000", "042000"},
            {"X02210", "577555", "X02010", "X04210"},
            {"XX0231", "X57765", "XX0211", "X53435"},
            {"310033", "355333", "313333", "312030"},
            {"X31013", "X35543", "X31313", "X31213"},
            {"X10131", "X13134", "X13124", "X13314"},
            {"431114", "466544", "431112", "411142"}
        }};
        const std::array<float, 7> Row3_Roots = {0.333f, 0.75f, 0.1667f, 0.583f, 1.0f, 0.833f, 0.667f };

        const std::array<std::array<std::string, 4>, 7> Row4_Names = {{
            {"Asus2" ,"A6" ,"A7sus4" ,"Am9"}, 
            {"Dsus2" ,"D6" ,"D7sus4" ,"Dm9"}, 
            {"Gsus2" ,"G6" ,"G7sus4" ,"Gm9"}, 
            {"Csus2" ,"C6" ,"C7sus4" ,"Cm9"}, 
            {"Fsus2" ,"F6" ,"F7sus4" ,"Fm9"}, 
            {"Bbsus2","Bb6","Bb7sus4","Bm9"}, 
            {"Ebsus2","Eb6","Eb7sus4","Em9"} 
        }};
        const std::array<std::array<std::string, 4>, 7> Row4_Chords = {{
            {"X02200", "X02222", "X02030", "X02413"},
            {"XX0230", "XX0202", "XX0213", "X53555"},
            {"300033", "320030", "330031", "300331"},
            {"X30013", "X32253", "X35363", "X31333"},
            {"133011", "100211", "131341", "133044"},
            {"X13311", "X13031", "X13141", "XX8698"},
            {"XX1341", "X65586", "XX1324", "X64666"}
        }};
        const std::array<float, 7> Row4_Roots = {0.75f, 0.1667f, 0.583f, 1.0f, 0.417f, 0.833f, 0.25f};

    //Load Digital Display
    DigitalDisplay* digitalDisplay = nullptr;
    DigitalDisplay* fingeringDisplay = nullptr;

    DigitalDisplay* Row1Display = nullptr;
    DigitalDisplay* Row2Display = nullptr;
    DigitalDisplay* Row3Display = nullptr;
    DigitalDisplay* Row4Display = nullptr;

    //Chord Diagram Display
    ChordDiagram* chordDiagram = nullptr;

    dsp::PulseGenerator triggerPulse;

    int chordIndex = 0;
    int rowIndex = 0;

    int lastChordIndex = 0;
    int lastRowIndex = 0;
    bool lastInputWasButton = false;

    float lastKnobChordPosition = 0.0;
    float lastKnobRowPosition = 0.0;

    int currentChordIndex = 0;
    int currentRowIndex = 0;

    bool barreButtonPressed = false;
    bool altButtonPressed = false;
    bool barreLatched = false;
    bool altLatched = false;
    int capo_offset = 0;
    bool barreGateActive = false;
    bool altGateActive = false;

    // Serialization method to save module state
    json_t* dataToJson() override {
        json_t* rootJ = json_object();

        json_object_set_new(rootJ, "barreLatched",    json_boolean(barreLatched));
        json_object_set_new(rootJ, "altLatched",      json_boolean(altLatched));
         return rootJ;
    }

    // Deserialization method to load module state
    void dataFromJson(json_t* rootJ) override {
        json_t* barreLatchedJ = json_object_get(rootJ, "barreLatched");
        if (barreLatchedJ) {
            barreLatched = json_is_true(barreLatchedJ);
        }

        json_t* altLatchedJ = json_object_get(rootJ, "altLatched");
        if (altLatchedJ) {
            altLatched = json_is_true(altLatchedJ);
        }

    }

    Strings() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);            
        // Initialize parameters for chord buttons
        for (int row = 0; row < MAX_ROWS; ++row) {
            for (int chord = 0; chord < MAX_CHORDS_PER_ROW; ++chord) {
                int index = row * MAX_CHORDS_PER_ROW + chord;
                configParam(CHORD_BUTTON_1 + index, 0.0, 1.0, 0.0, "Chord " + std::to_string(index + 1));
            }
        }

        //Initialize Knobs
        configParam(CHORD_SELECTOR_PARAM, 1.f, 7.f, 1.f, "Chord Selection");
        configParam(ROW_SELECTOR_PARAM, 1.f, 4.f, 1.f, "Chord Bank");
        configParam(CAPO_PARAM, -12.f, 12.f, 0.f, "Capo Position");

        // Initialize inputs
        configInput(CHORD_SELECTOR_CV, "Chord Selector CV");
        configInput(ROW_SELECTOR_CV, "Row Selector CV");
        configInput(CAPO_CV, "Capo CV");

        configInput(BARRE_CHORD_GATE, "Chord Bank I");
        configInput(ALT_CHORD_GATE, "Chord Bank II");
        
        for (int i = ENVELOPE_IN_1; i <= ENVELOPE_IN_6; i++) {
            configInput(i, "Envelope Input " + std::to_string(i - ENVELOPE_IN_1 + 1));
        }
        configInput(WHAMMY_BAR_CV, "Whammy Bar CV");

        // Initialize outputs
        for (int i = STRING_CV_OUT_1; i <= STRING_CV_OUT_6; i++) {
            configOutput(i, "String CV Output " + std::to_string(i - STRING_CV_OUT_1 + 1));
        }
        configOutput(ROOT_NOTE_CV_OUT, "Root Note CV");      
        configOutput(TRIGGER_OUT, "Chord Change Trigger");      
 
        barreLatched = false;
        altLatched = false; 
        barreGateActive = false;
        altGateActive = false;
       
    }

    void process(const ProcessArgs& args) override {
        // Calculate the whammy bar effect if connected
        float whammyBarEffect = inputs[WHAMMY_BAR_CV].isConnected() ? std::abs(inputs[WHAMMY_BAR_CV].getVoltage() * (0.2f / 12.0f)) : 0.f;

        float CapoAmount = inputs[CAPO_CV].isConnected() ? (floor(inputs[CAPO_CV].getVoltage() + params[CAPO_PARAM].getValue()) * (2.f/12.f)) : floor(params[CAPO_PARAM].getValue()) * (2.f/12.f); 
        
        float PitchBend[6] = {0.0f};

        // Determine the current chord and row selections
        int knobChordPosition = static_cast<int>(
            floor(
                clamp(
                    params[CHORD_SELECTOR_PARAM].getValue() + 
                    (inputs[CHORD_SELECTOR_CV].isConnected() ? inputs[CHORD_SELECTOR_CV].getVoltage() : 0) - 1,
                    0.0,6.0
                )
            )
        );
        int knobRowPosition = static_cast<int>(
            floor(
                clamp(
                    params[ROW_SELECTOR_PARAM].getValue() + 
                    (inputs[ROW_SELECTOR_CV].isConnected() ? inputs[ROW_SELECTOR_CV].getVoltage() : 0) - 1,
                    0.0,3.0
                )
            )
        );

        static int latchedChordIndex = -1; // Initialize to invalid index to ensure first update
        static int latchedRowIndex = -1; // Initialize to invalid index to ensure first update

        // Check for button presses
        bool buttonPressed = false;
        for (int i = 0; i < NUM_PARAMS - CHORD_BUTTON_1; i++) {
            if (params[CHORD_BUTTON_1 + i].getValue() > 0) {
                latchedRowIndex = i / MAX_CHORDS_PER_ROW;
                latchedChordIndex = i % MAX_CHORDS_PER_ROW;
                buttonPressed = true;
                lastInputWasButton = true;
                break; // Exit the loop after handling the first pressed button
            }
        }

        // Latch button-pressed indices if a button was just pressed
        if (buttonPressed) {
            currentChordIndex = latchedChordIndex;
            currentRowIndex = latchedRowIndex;
            triggerPulse.trigger(0.001f); // 1ms pulse
        } else {
            // Update based on knobs if no button is currently latched
            if (knobChordPosition != lastKnobChordPosition || knobRowPosition != lastKnobRowPosition) {
                currentChordIndex = static_cast<int>(floor(clamp(knobChordPosition, 0, 6)));
                currentRowIndex = static_cast<int>(floor(clamp(knobRowPosition, 0, 3)));
                lastKnobChordPosition = knobChordPosition;
                lastKnobRowPosition = knobRowPosition;
            }
        }

        // Reset all chord selection lights
        for (int k = 0; k < NUM_PARAMS - CHORD_BUTTON_1; k++) {
            lights[CHORD_SELECTION_LIGHT_1 + k].setBrightness(0.0);
        }

        // Activate the light for the current chord selection
        // Calculate the index for the light corresponding to the current selection
        int currentSelectionIndex = currentRowIndex * MAX_CHORDS_PER_ROW + currentChordIndex;
        lights[CHORD_SELECTION_LIGHT_1 + currentSelectionIndex].setBrightness(1.0);

        // Selecting the arrays based on currentRowIndex
        const auto& currentNames = (currentRowIndex == 0) ? Row1_Names : (currentRowIndex == 1) ? Row2_Names : (currentRowIndex == 2) ? Row3_Names : Row4_Names;
        const auto& currentChords = (currentRowIndex == 0) ? Row1_Chords : (currentRowIndex == 1) ? Row2_Chords : (currentRowIndex == 2) ? Row3_Chords : Row4_Chords;
        const auto& currentRoots = (currentRowIndex == 0) ? Row1_Roots : (currentRowIndex == 1) ? Row2_Roots : (currentRowIndex == 2) ? Row3_Roots : Row4_Roots;

        // Set the root note voltage
        outputs[ROOT_NOTE_CV_OUT].setVoltage(currentRoots[currentChordIndex] + CapoAmount - 1.0f);

        // Handle BARRE_CHORD_BUTTON latching independently
        if (params[BARRE_CHORD_BUTTON].getValue() > 0) {
            if (!barreButtonPressed) {
                barreLatched = !barreLatched;
                barreButtonPressed = true;
            }
        } else {
            barreButtonPressed = false;
        }

        // Handle ALT_CHORD_BUTTON latching independently
        if (params[ALT_CHORD_BUTTON].getValue() > 0) {
            if (!altButtonPressed) {
                altLatched = !altLatched;
                altButtonPressed = true;
            }
        } else {
            altButtonPressed = false;
        }

        // Determine gate states based on latched states and external gate presence
        barreGateActive = inputs[BARRE_CHORD_GATE].isConnected() ? !barreLatched ^ (inputs[BARRE_CHORD_GATE].getVoltage() > 0.05f) : barreLatched;
        altGateActive = inputs[ALT_CHORD_GATE].isConnected() ? !altLatched ^ (inputs[ALT_CHORD_GATE].getVoltage() > 0.05f) : altLatched;

        // Update lights based on latched state or external gate activity
        lights[BARRE_CHORD_LIGHT].setBrightness(barreGateActive ? 1.0 : 0.0);
        lights[ALT_CHORD_LIGHT].setBrightness(altGateActive ? 1.0 : 0.0);

        // Use the modified gate states to choose the fingering version
        int fingeringVersion = getFingeringVersion(barreGateActive, altGateActive);

        // Iterate over strings to set voltages
        for (int stringIdx = 0; stringIdx < 6; ++stringIdx) {
            // Convert fingering string to semitone shifts
            auto semitoneShifts = fingeringToSemitoneShifts(currentChords[currentChordIndex][fingeringVersion]);

            PitchBend[stringIdx]=(0.1f/12.f) * (inputs[ENVELOPE_IN_1 + stringIdx].isConnected() ? inputs[ENVELOPE_IN_1 + stringIdx].getVoltage() : 0);
            PitchBend[stringIdx]= abs(PitchBend[stringIdx]);

            // Process each string
            if (semitoneShifts[stringIdx] >= 0) {
                // Calculate pitch voltage
                float pitchVoltage = baseFrequencies[stringIdx] + (semitoneShifts[stringIdx] * (1.0f / 12.0f)) + whammyBarEffect + CapoAmount + PitchBend[stringIdx];
                outputs[STRING_CV_OUT_1 + stringIdx].setVoltage(pitchVoltage);
                outputs[MUTE_OUT_1 + stringIdx].setVoltage(0.0);
            } else {
                // Mute this string             
                outputs[STRING_CV_OUT_1 + stringIdx].setVoltage(currentRoots[currentChordIndex] + CapoAmount - 1.0f);  //set muted string to the root note, just incase you don't mute it
                outputs[MUTE_OUT_1 + stringIdx].setVoltage(5.0);
            }
        }
        
        // Update display logic
        if (digitalDisplay && fingeringDisplay) {
            // Static variables to remember the last displayed chord, row indices, and fingering
            static int lastDisplayedChordIndex = -1;
            static int lastDisplayedRowIndex = -1;
            static int lastFingering = -1;


            // Define a small tolerance value for comparison
            const float capoTolerance = 0.01f; // Adjust tolerance as needed

            // Keep track of the last CapoAmount for comparison
            static float lastCapoAmount = -1.0f; // Initialize with an unlikely value

            // Calculate CapoAmount considering both CV input and knob position
            float CapoAmount = inputs[CAPO_CV].isConnected() ? 
                (floor(inputs[CAPO_CV].getVoltage() + params[CAPO_PARAM].getValue()) * (1.f/12.f)) : 
                (floor(params[CAPO_PARAM].getValue()) * (1.f/12.f));

            // Determine if CapoAmount has "effectively" changed using the tolerance
            bool capoAmountChanged = std::abs(CapoAmount - lastCapoAmount) > capoTolerance;

            // Proceed with checking if an update is needed
            if (currentChordIndex != lastDisplayedChordIndex || currentRowIndex != lastDisplayedRowIndex || 
                fingeringVersion != lastFingering || capoAmountChanged) {
                // Update the last displayed indices to the current selection
                lastDisplayedChordIndex = currentChordIndex;
                lastDisplayedRowIndex = currentRowIndex;
                lastFingering = fingeringVersion;
    
                // Only update lastCapoAmount if it has effectively changed
                if (capoAmountChanged) {
                    lastCapoAmount = CapoAmount;
                }
                // Retrieve the current chord name based on the selection for the digital display
                std::string currentChordName = currentNames[currentChordIndex][fingeringVersion];
                // Update the digital display text with the current chord name
                digitalDisplay->text = currentChordName;

                // Retrieve the current fingering pattern for the fingering display
                std::string currentFingeringPattern = currentChords[currentChordIndex][fingeringVersion];
                // Update the fingering display text with the current fingering pattern

                int capoAmountInt = static_cast<int>(floor(CapoAmount*12)); // Round to the nearest whole number if necessary
                std::string capoAmountStr = std::to_string(capoAmountInt); // Convert the integer to a string
                
                if (CapoAmount ==0){
                    fingeringDisplay->text = currentFingeringPattern;
                } else {
                    // Update the fingering display text with the current fingering pattern and capo setting
                    if (CapoAmount>-0.01){
                        fingeringDisplay->text = currentFingeringPattern + " +" + capoAmountStr;    
                    } else {
                        fingeringDisplay->text = currentFingeringPattern + "  " + capoAmountStr;    
                    }
                }
                
                if (chordDiagram) {
                    auto semitoneShifts = fingeringToSemitoneShifts(currentChords[currentChordIndex][fingeringVersion]);
                    chordDiagram->setFingering(semitoneShifts);
                }  
    
                if (Row1Display) {        
                    //{"B7", "B"     , "Bsus4", "Badd9"},
                    auto row1text = "Row1";
                    if (fingeringVersion == 0){row1text = "7";}
                    else if (fingeringVersion == 1){row1text = "7 Bar";}
                    else if (fingeringVersion == 2){row1text = "sus4";}
                    else if (fingeringVersion == 3){row1text = "add9";}
                    Row1Display->text = row1text;
                }

                if (Row2Display) {        
                    //{"A" ,"A-Bar" ,"Amaj7" ,"Aaug"}
                    auto row2text = "Row2";
                    if (fingeringVersion == 0){row2text = "Maj";}
                    else if (fingeringVersion == 1){row2text = "M Bar";}
                    else if (fingeringVersion == 2){row2text = "Maj7";}
                    else if (fingeringVersion == 3){row2text = "aug";}
                    Row2Display->text = row2text;
                }

                if (Row3Display) {        
                    //{"Em" ,"Em-Bar" ,"Em7","Em6  "},
                    auto row3text = "Row3";
                    if (fingeringVersion == 0){row3text = "min";}
                    else if (fingeringVersion == 1){row3text = "m Bar";}
                    else if (fingeringVersion == 2){row3text = "m7";}
                    else if (fingeringVersion == 3){row3text = "m6";}
                    Row3Display->text = row3text;
                }

                if (Row4Display) {        
                    //{"Asus2" ,"A6" ,"A7sus4" ,"Am9"},
                    auto row4text = "Row4";
                    if (fingeringVersion == 0){row4text = "sus2";}
                    else if (fingeringVersion == 1){row4text = "6";}
                    else if (fingeringVersion == 2){row4text = "7sus4";}
                    else if (fingeringVersion == 3){row4text = "m9";}
                    Row4Display->text = row4text;
                } 
                
                triggerPulse.trigger(0.001f); // 1ms pulse
            }
        }
        
        if (triggerPulse.process(args.sampleTime)) {
            outputs[TRIGGER_OUT].setVoltage(10.f); // High voltage of the pulse
        } else {
            outputs[TRIGGER_OUT].setVoltage(0.f); // Low voltage, pulse off
        }
        
    }
    
    int getFingeringVersion(float barreVoltage, float altVoltage) {
        // Default to regular fingering
        int version = 0;
   
        // Process inputs only if they are connected
        if (barreVoltage >= 1.0f) {
            version = 1; // Barre version
        }
        if (altVoltage >= 1.0f) {
            version += 2; // Alternative version
        }

        return version;
    }    
};

struct StringsWidget : ModuleWidget {
    StringsWidget(Strings* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Strings.svg"),
            asset::plugin(pluginInstance, "res/Strings-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(4*RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 5 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(4*RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 5 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // Knobs
        addParam(createParamCentered<RoundBlackKnob>(Vec(30,  30), module, Strings::CHORD_SELECTOR_PARAM));
        addInput(createInputCentered<PJ301MPort>(Vec(30,   65), module, Strings::CHORD_SELECTOR_CV));

        addParam(createParamCentered<RoundBlackKnob>(Vec(30, 40+80), module, Strings::ROW_SELECTOR_PARAM));
        addInput(createInputCentered<PJ301MPort>(Vec(30,  75+80), module, Strings::ROW_SELECTOR_CV));

        addParam(createParamCentered<RoundBlackKnob>(Vec(270, 30), module, Strings::CAPO_PARAM));
        addInput(createInputCentered<PJ301MPort>(Vec(270,  65), module, Strings::CAPO_CV));

        // Gate Inputs

        addInput(createInputCentered<PJ301MPort>(Vec(30, 220), module, Strings::BARRE_CHORD_GATE));
        addInput(createInputCentered<PJ301MPort>(Vec(60, 220), module, Strings::ALT_CHORD_GATE));

        addParam(createParamCentered<LEDButton>           (Vec(30, 195), module, Strings::BARRE_CHORD_BUTTON));
        addChild(createLightCentered<SmallLight<RedLight>>(Vec(30, 195), module, Strings::BARRE_CHORD_LIGHT));
        addParam(createParamCentered<LEDButton>           (Vec(60, 195), module, Strings::ALT_CHORD_BUTTON));
        addChild(createLightCentered<SmallLight<RedLight>>(Vec(60, 195), module, Strings::ALT_CHORD_LIGHT));

        float left = 35; //sets the left edge
        float jack = 35; //sets spacing between jacks

        // Inputs
        addInput(createInputCentered<PJ301MPort>(Vec(left,        270), module, Strings::ENVELOPE_IN_1));
        addInput(createInputCentered<PJ301MPort>(Vec(1*jack+left, 270), module, Strings::ENVELOPE_IN_2));
        addInput(createInputCentered<PJ301MPort>(Vec(2*jack+left, 270), module, Strings::ENVELOPE_IN_3));
        addInput(createInputCentered<PJ301MPort>(Vec(3*jack+left, 270), module, Strings::ENVELOPE_IN_4));
        addInput(createInputCentered<PJ301MPort>(Vec(4*jack+left, 270), module, Strings::ENVELOPE_IN_5));
        addInput(createInputCentered<PJ301MPort>(Vec(5*jack+left, 270), module, Strings::ENVELOPE_IN_6));
        addInput(createInputCentered<PJ301MPort>(Vec(6.5*jack+left, 270), module, Strings::WHAMMY_BAR_CV));

        // Outputs              
        addOutput(createOutputCentered<PJ301MPort>(Vec(left,        310), module, Strings::MUTE_OUT_1));
        addOutput(createOutputCentered<PJ301MPort>(Vec(1*jack+left, 310), module, Strings::MUTE_OUT_2));
        addOutput(createOutputCentered<PJ301MPort>(Vec(2*jack+left, 310), module, Strings::MUTE_OUT_3));
        addOutput(createOutputCentered<PJ301MPort>(Vec(3*jack+left, 310), module, Strings::MUTE_OUT_4));
        addOutput(createOutputCentered<PJ301MPort>(Vec(4*jack+left, 310), module, Strings::MUTE_OUT_5));
        addOutput(createOutputCentered<PJ301MPort>(Vec(5*jack+left, 310), module, Strings::MUTE_OUT_6));
        addOutput(createOutputCentered<PJ301MPort>(Vec(6.5*jack+left, 310), module, Strings::TRIGGER_OUT));


        addOutput(createOutputCentered<PJ301MPort>(Vec(left,        340), module, Strings::STRING_CV_OUT_1));
        addOutput(createOutputCentered<PJ301MPort>(Vec(1*jack+left, 340), module, Strings::STRING_CV_OUT_2));
        addOutput(createOutputCentered<PJ301MPort>(Vec(2*jack+left, 340), module, Strings::STRING_CV_OUT_3));
        addOutput(createOutputCentered<PJ301MPort>(Vec(3*jack+left, 340), module, Strings::STRING_CV_OUT_4));
        addOutput(createOutputCentered<PJ301MPort>(Vec(4*jack+left, 340), module, Strings::STRING_CV_OUT_5));
        addOutput(createOutputCentered<PJ301MPort>(Vec(5*jack+left, 340), module, Strings::STRING_CV_OUT_6));
        addOutput(createOutputCentered<PJ301MPort>(Vec(6.5*jack+left, 340), module, Strings::ROOT_NOTE_CV_OUT));

        // Chord selection buttons and indicator lights
        Vec buttonStartPos = Vec(70, 110);
        float xSpacing = 25.0;
        float ySpacing = 36.0;
        float xShift = 12.0;
    
        for (int i = 0; i < MAX_CHORDS_PER_ROW * MAX_ROWS; i++) {
            Vec pos = buttonStartPos.plus(Vec(xSpacing * (i % MAX_CHORDS_PER_ROW) + xShift * (i / MAX_CHORDS_PER_ROW), (i / MAX_CHORDS_PER_ROW) * ySpacing));
            addParam(createParamCentered<LEDButton>(pos, module, Strings::CHORD_BUTTON_1 + i));
            addChild(createLightCentered<SmallLight<RedLight>>(pos, module, Strings::CHORD_SELECTION_LIGHT_1 + i));
        }
             
        float disp_x = 95;
        //Display        
        DigitalDisplay* digitalDisplay = new DigitalDisplay();
        digitalDisplay->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        digitalDisplay->box.pos = Vec(disp_x, 34); // Position on the module
        digitalDisplay->box.size = Vec(50, 18); // Size of the display
        digitalDisplay->text = "Ready"; // Initial text
        digitalDisplay->fgColor = nvgRGB(208, 140, 89); // Gold color text
        digitalDisplay->textPos = Vec(disp_x, 35); // Text position
        digitalDisplay->setFontSize(16.0f); // Set the font size as desired
        addChild(digitalDisplay);

        if (module) {
            module->digitalDisplay = digitalDisplay; // Link the module to the display
        }    
        
        // Configure and add the second digital display for modNumber
        DigitalDisplay* fingeringDisplay = new DigitalDisplay();
        fingeringDisplay->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        fingeringDisplay->box.pos = Vec(disp_x, 54); // Position below the first display
        fingeringDisplay->box.size = Vec(50, 18); // Size of the display
        fingeringDisplay->text = "Chord"; // Initial text or placeholder
        fingeringDisplay->fgColor = nvgRGB(208, 140, 89); // Gold color text
        fingeringDisplay->textPos = Vec(disp_x, 55); // Text position
        fingeringDisplay->setFontSize(10.0f); // Set the font size as desired

        addChild(fingeringDisplay);

        if (module) {
            module->fingeringDisplay = fingeringDisplay;
        }

        //Spacings for the ROW display elements
        disp_x =233;
        float disp_y = 100;

        DigitalDisplay* Row1Display = new DigitalDisplay();
        Row1Display->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        Row1Display->box.pos = Vec(disp_x, disp_y); // Position below the first display
        Row1Display->box.size = Vec(50, 18); // Size of the display
        Row1Display->text = "Row1"; // Initial text or placeholder
        Row1Display->setTextAlign(NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
        Row1Display->fgColor = nvgRGB(120, 120, 120); // White color text
        Row1Display->textPos = Vec(disp_x, disp_y); // Text position
        Row1Display->setFontSize(10.0f); // Set the font size as desired

        addChild(Row1Display);

        if (module) {
            module->Row1Display = Row1Display;
        }

        DigitalDisplay* Row2Display = new DigitalDisplay();
        Row2Display->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        Row2Display->box.pos = Vec(disp_x+xShift, disp_y+ySpacing); // Position below the first display
        Row2Display->box.size = Vec(50, 18); // Size of the display
        Row2Display->text = "Row2"; // Initial text or placeholder
        Row2Display->setTextAlign(NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
        Row2Display->fgColor = nvgRGB(120, 120, 120); // White color text
        Row2Display->textPos = Vec(disp_x+xShift, disp_y+ySpacing); // Text position
        Row2Display->setFontSize(10.0f); // Set the font size as desired

        addChild(Row2Display);

        if (module) {
            module->Row2Display = Row2Display;
        }
        
        DigitalDisplay* Row3Display = new DigitalDisplay();
        Row3Display->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        Row3Display->box.pos = Vec(disp_x+2*xShift, disp_y+2*ySpacing); // Position below the first display
        Row3Display->box.size = Vec(50, 18); // Size of the display
        Row3Display->text = "Row3"; // Initial text or placeholder
        Row3Display->setTextAlign(NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
        Row3Display->fgColor = nvgRGB(120, 120, 120); // White color text
        Row3Display->textPos = Vec(disp_x+2*xShift, disp_y+2*ySpacing); // Text position
        Row3Display->setFontSize(10.0f); // Set the font size as desired

        addChild(Row3Display);

        if (module) {
            module->Row3Display = Row3Display;
        }

        DigitalDisplay* Row4Display = new DigitalDisplay();
        Row4Display->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        Row4Display->box.pos = Vec(disp_x+3*xShift, disp_y+3*ySpacing); // Position below the first display
        Row4Display->box.size = Vec(50, 18); // Size of the display
        Row4Display->text = "Row4"; // Initial text or placeholder
        Row4Display->setTextAlign(NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
        Row4Display->fgColor = nvgRGB(120, 120, 120); // White color text
        Row4Display->textPos = Vec(disp_x+3*xShift, disp_y+3*ySpacing); // Text position
        Row4Display->setFontSize(10.0f); // Set the font size as desired

        addChild(Row4Display);

        if (module) {
            module->Row4Display = Row4Display;
        }
   
        // Chord diagram display
        ChordDiagram* chordDiagram = new ChordDiagram();
        chordDiagram->box.pos = Vec(158, 30); // Position on the module
        chordDiagram->box.size = Vec(50, 50); // Size of the chord diagram area
        addChild(chordDiagram);
       
        if (module) {
            module->chordDiagram = chordDiagram;
        }                          
    }
};

Model* modelStrings = createModel<Strings, StringsWidget>("Strings");