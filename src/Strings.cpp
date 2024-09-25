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

struct DiscreteRoundBlackKnob : RoundBlackKnob { 
    void onDragEnd(const DragEndEvent& e) override {
        ParamQuantity* paramQuantity = getParamQuantity();
        
        if (paramQuantity) {
            // Get the raw value from the knob
            float rawValue = paramQuantity->getValue();
            
            // Round the value to the nearest integer
            float discreteValue = round(rawValue);
            
            // Set the snapped value
            paramQuantity->setValue(discreteValue);
        }
        
        // Call the base class implementation to ensure proper behavior
        RoundBlackKnob::onDragEnd(e);
    }
};

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
        const std::array<float, 7> Row1_Roots = {0.917f, 0.3333f, 0.75f, 0.1667f, 0.5833f, 0.0f, 0.4167f};

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
        const std::array<float, 7> Row2_Roots = { 0.75f, 0.167f, 0.583f, 0.0f, 0.417f, 0.833f ,0.25f };

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
        const std::array<float, 7> Row3_Roots = {0.333f, 0.75f, 0.1667f, 0.583f, 0.0f, 0.833f, 0.667f };

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
        const std::array<float, 7> Row4_Roots = {0.75f, 0.1667f, 0.583f, 0.0f, 0.417f, 0.833f, 0.25f};

        // Direct initialization of Chords Arrays
        // Define chord names for row 1
        const std::array<std::array<std::string, 4>, 7> Row1_NamesB = {{
            {"B7", "B"     , "B2", "B6"},
            {"E7", "E"     , "E2", "E6"},
            {"A7", "A7-Bar", "A2", "A6"},
            {"D7", "D7-Bar", "D2", "D6"},
            {"G7", "G7-Bar", "G2", "G6"},
            {"C7", "C7-Bar", "C2", "C6"},
            {"F7", "F7-Bar", "F2", "F6"}
        }};
        // Define chord fingerings for row 1
        const std::array<std::array<std::string, 4>, 7> Row1_ChordsB = {{
             {"X21202", "X24442", "744647", "X21102"},
             {"020100", "022100", "024100", "042100"},
             {"X02020", "575685", "X02420", "X04220"},
             {"XX0212", "X5453X", "X54252", "XX0202"},
             {"320001", "353433", "300003", "320000"},
             {"X32310", "X35353", "X30010", "X32253"},
             {"101211", "131211", "103013", "100211"}
        }};
        // Define root notes for row 1
        const std::array<float, 7> Row1_RootsB = {0.917f, 0.3333f, 0.75f, 0.1667f, 0.5833f, 0.0f, 0.4167f};

        const std::array<std::array<std::string, 4>, 7> Row2_NamesB = {{
            {"A" ,"A-Bar" ,"Amaj7" ,"A7+5"}, 
            {"D" ,"D-Bar" ,"Dmaj7" ,"D7+5"}, 
            {"G" ,"G-Bar" ,"Gmaj7" ,"G7+5"}, 
            {"C" ,"C-Bar" ,"Cmaj7" ,"C7+5"}, 
            {"F" ,"F-Bar" ,"Fmaj7" ,"F7+5"}, 
            {"Bb","Bb-Bar","Bbmaj7","Bb7+5"},
            {"Eb","Eb-Bar","Ebmaj7","Eb7+5"}
        }};
        const std::array<std::array<std::string, 4>, 7> Row2_ChordsB = {{
            {"X02220", "577655", "X02120", "X03021"}, 
            {"XX0232", "X57775", "XX0222", "XX0312"}, 
            {"320003", "355433", "320002", "321001"}, 
            {"X32010", "X35553", "X32000", "X36354"}, 
            {"133211", "133211", "102210", "101221"}, 
            {"X10331", "X13331", "X10231", "X10132"}, 
            {"XX1343", "X68886", "XX1333", "XX1423"} 
        }};
        const std::array<float, 7> Row2_RootsB = { 0.75f, 0.167f, 0.583f, 0.0f, 0.417f, 0.833f ,0.25f };

        const std::array<std::array<std::string, 4>, 7> Row3_NamesB = {{
            {"Em" ,"Em-Bar" ,"Em7"    ,"Em6"},    
            {"Am" ,"Am-Bar" ,"Am7"    ,"Am6"},    
            {"Dm" ,"Dm-Bar" ,"Dm7"    ,"Dm6"},    
            {"Gm" ,"Gm-Bar" ,"Gm7"    ,"Gm6"},    
            {"Cm" ,"Cm-Bar" ,"Cm7"    ,"Cm6"},    
            {"Bb7","Bb7-Bar","Bbm7"   ,"Bb7sus2"},  
            {"Ab" ,"Ab-Bar" ,"Ab7"    ,"Abm6"}  
        }};
        const std::array<std::array<std::string, 4>, 7> Row3_ChordsB = {{
            {"022000", "X79987", "020000", "042000"},
            {"X02210", "577555", "X02010", "X04210"},
            {"XX0231", "X57765", "XX0211", "X53435"},
            {"310033", "355333", "313333", "312030"},
            {"X31013", "X35543", "X31313", "X31213"},
            {"X10131", "X13134", "X13124", "X13314"},
            {"431114", "466544", "431112", "421141"}
        }};
        const std::array<float, 7> Row3_RootsB = {0.333f, 0.75f, 0.1667f, 0.583f, 0.0f, 0.833f, 0.667f };

        const std::array<std::array<std::string, 4>, 7> Row4_NamesB = {{
            {"Adim" ,"Adim7" ,"A9" ,"Aaug"}, 
            {"Ddim" ,"Ddim7" ,"D9" ,"Daug"}, 
            {"Gdim" ,"Gdim7" ,"G9" ,"Gaug"}, 
            {"Cdim" ,"Cdim7" ,"C9" ,"Caug"}, 
            {"Fdim" ,"Fdim7" ,"F9" ,"Faug"}, 
            {"Bbdim","Bbdim7","Bb9","Bbaug"}, 
            {"Ebdim","Ebdim7","Eb9","Ebaug"} 
        }};
        const std::array<std::array<std::string, 4>, 7> Row4_ChordsB = {{
            {"X0121X", "2312XX", "X02423", "X03221"},
            {"XX0131", "XX0101", "X52532", "XX0332"},
            {"XX5323", "X1202X", "300001", "321003"},
            {"X3454X", "X3424X", "X30310", "X32110"},
            {"12310X", "1201XX", "101013", "XX3221"},
            {"X12320", "X1202X", "X10314", "X10332"},
            {"XX1242", "XX1212", "X63643", "XX1403"}
        }};
        const std::array<float, 7> Row4_RootsB = {0.75f, 0.1667f, 0.583f, 0.0f, 0.417f, 0.833f, 0.25f};

    //Load Digital Displays
    DigitalDisplay* digitalDisplay = nullptr;
    DigitalDisplay* fingeringDisplay = nullptr;
    DigitalDisplay* Row1Display = nullptr;
    DigitalDisplay* Row2Display = nullptr;
    DigitalDisplay* Row3Display = nullptr;
    DigitalDisplay* Row4Display = nullptr;
    DigitalDisplay* CVModeDisplay = nullptr;
    ChordDiagram* chordDiagram = nullptr;

    dsp::PulseGenerator triggerPulse;

    int chordIndex = 0;
    int rowIndex = 0;
    int lastChordIndex = 0;
    int lastRowIndex = 0;
    int currentChordIndex = 0;
    int currentRowIndex = 0;
    int capo_offset = 0;
    float lastKnobChordPosition = 0.0;
    float lastKnobRowPosition = 0.0;

    bool barreButtonPressed = false;
    bool altButtonPressed = false;
    bool barreLatched = false;
    bool altLatched = false;
    bool barreGateActive = false;
    bool altGateActive = false;
    bool ChordBank = false;
    bool VOctCV = false;
    bool InvertMutes = false;

    int latchedChordIndex = -1; // Initialize to invalid index to ensure first update
    int latchedRowIndex = -1; // Initialize to invalid index to ensure first update

    int process_count = 0;
    int display_count = 0;
    int process_skip = 10;
    int display_skip = 1000;

    // Serialization method to save module state
    json_t* dataToJson() override {
        json_t* rootJ = json_object();

        json_object_set_new(rootJ, "InvertMutes", json_boolean(InvertMutes));
        json_object_set_new(rootJ, "VOctCV", json_boolean(VOctCV));
        json_object_set_new(rootJ, "ChordBank", json_boolean(ChordBank));
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
        
        // Load the state of VoltRange
        json_t* ChordBankJ = json_object_get(rootJ, "ChordBank");
        if (ChordBankJ) {
            ChordBank = json_is_true(ChordBankJ);
        }

        // Load the state of VOctCV
        json_t* VOctCVJ = json_object_get(rootJ, "VOctCV");
        if (VOctCVJ) {
            VOctCV = json_is_true(VOctCVJ);
        }  

        // Load the state of InvertMutes
        json_t* InvertMutesJ = json_object_get(rootJ, "InvertMutes");
        if (InvertMutesJ) {
            VOctCV = json_is_true(InvertMutesJ);
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
        configParam(CHORD_SELECTOR_PARAM, 1.f, 7.5f, 1.f, "Chord Selection");
        configParam(ROW_SELECTOR_PARAM, 1.f, 4.5f, 1.f, "Chord Bank");
        configParam(CAPO_PARAM, -12.f, 12.f, 0.f, "Capo Position");

        // Initialize inputs
        configInput(CHORD_SELECTOR_CV, "Chord Selector CV");
        configInput(ROW_SELECTOR_CV, "Row Selector CV");
        configInput(CAPO_CV, "Capo CV");

        configInput(BARRE_CHORD_GATE, "Chord Bank I");
        configInput(ALT_CHORD_GATE, "Chord Bank II");
        
        for (int i = ENVELOPE_IN_1; i <= ENVELOPE_IN_6; i++) {
            configInput(i, "Pitch Bend " + std::to_string(i - ENVELOPE_IN_1 + 1));
        }
        configInput(WHAMMY_BAR_CV, "Whammy Bar");

        // Initialize outputs
        configOutput(STRING_CV_OUT_1, "String 1 V/oct / Poly");
        configOutput(STRING_CV_OUT_2, "String 2 V/oct");
        configOutput(STRING_CV_OUT_3, "String 3 V/oct");
        configOutput(STRING_CV_OUT_4, "String 4 V/oct");
        configOutput(STRING_CV_OUT_5, "String 5 V/oct");
        configOutput(STRING_CV_OUT_6, "String 6 V/oct");
        
        configOutput(ROOT_NOTE_CV_OUT, "Root Note V/oct");      
        configOutput(TRIGGER_OUT, "Chord Change Trigger");      

        configParam(BARRE_CHORD_BUTTON, 0.0, 1.0, 0.0, "Chord Bank I Button" );
        configParam(ALT_CHORD_BUTTON, 0.0, 1.0, 0.0, "Chord Bank II Button" );

        configOutput(MUTE_OUT_1, "Mute 1 / Poly");
        configOutput(MUTE_OUT_2, "Mute 2");
        configOutput(MUTE_OUT_3, "Mute 3");
        configOutput(MUTE_OUT_4, "Mute 4");
        configOutput(MUTE_OUT_5, "Mute 5");
        configOutput(MUTE_OUT_6, "Mute 6");

        barreLatched = false;
        altLatched = false; 
        barreGateActive = false;
        altGateActive = false;
       
    }

    void process(const ProcessArgs& args) override {

        // Lambda function for selecting the appropriate chord bank
        auto selectChordBankStrings = [&](
            bool bank,
            const std::array<std::array<std::string, 4>, 7>& bank0Row0, const std::array<std::array<std::string, 4>, 7>& bank0Row1,
            const std::array<std::array<std::string, 4>, 7>& bank0Row2, const std::array<std::array<std::string, 4>, 7>& bank0Row3,
            const std::array<std::array<std::string, 4>, 7>& bank1Row0, const std::array<std::array<std::string, 4>, 7>& bank1Row1,
            const std::array<std::array<std::string, 4>, 7>& bank1Row2, const std::array<std::array<std::string, 4>, 7>& bank1Row3
        ) -> const std::array<std::array<std::string, 4>, 7>& {

            if (bank) { // If ChordBank is true, use Bank B
                switch (currentRowIndex) {
                    case 0: return bank1Row0; case 1: return bank1Row1; case 2: return bank1Row2; default: return bank1Row3;
                }
            } else { // Else, use Bank A
                switch (currentRowIndex) {
                    case 0: return bank0Row0; case 1: return bank0Row1; case 2: return bank0Row2; default: return bank0Row3;
                }
            }
        };
        process_count++;
        display_count++;
        if (process_count>process_skip){
            process_count=0;

            // Calculate the whammy bar effect if connected
            float whammyBarEffect = inputs[WHAMMY_BAR_CV].isConnected() ? std::abs(inputs[WHAMMY_BAR_CV].getVoltage() * (0.2f / 12.0f)) : 0.f;
            float CapoAmount = inputs[CAPO_CV].isConnected() ? (floor(inputs[CAPO_CV].getVoltage() + params[CAPO_PARAM].getValue()) * (1.f/12.f)) : floor(params[CAPO_PARAM].getValue()) * (1.f/12.f); 
            float PitchBend[6] = {0.0f};

            // Determine the current chord and row selections
          
            int knobRowPosition = static_cast<int>(
                floor(
                    clamp(
                        params[ROW_SELECTOR_PARAM].getValue() + 
                        (inputs[ROW_SELECTOR_CV].isConnected() ? inputs[ROW_SELECTOR_CV].getVoltage() : 0) - 1,
                        0.0,3.0
                    )
                )
            );

            int knobChordPosition = static_cast<int>(
                floor(
                    clamp(
                        params[CHORD_SELECTOR_PARAM].getValue() + 
                        (inputs[CHORD_SELECTOR_CV].isConnected() ? inputs[CHORD_SELECTOR_CV].getVoltage() : 0) - 1,
                        0.0,6.0
                    )
                )
            );    

            int semitoneDifference = 0;
            float octavesDifference = 0.0f;

            // Define the noteToChordPosition arrays 
            //            //                                             C      D       E   F      G      A       B
            static const std::array<int, 12> noteToChordPositionRow1 = {{5, -1, 3, -1,  1,  6, -1, 4, -1, 2, -1,  0}};
            static const std::array<int, 12> noteToChordPositionRow2 = {{3, -1, 1,  6, -1,  4, -1, 2, -1, 0,  5, -1}};
            static const std::array<int, 12> noteToChordPositionRow3 = {{4, -1, 2, -1,  0, -1, -1, 3,  6, 1,  5, -1}};
            static const std::array<int, 12> noteToChordPositionRow4 = {{3, -1, 1,  6, -1,  4, -1, 2, -1, 0,  5, -1}};

            // V/Oct CV processing
            if(VOctCV) {
                //param chord_selector has no effect in V/Oct mode
                float chordInputVal = (inputs[CHORD_SELECTOR_CV].isConnected() ? inputs[CHORD_SELECTOR_CV].getVoltage() : 0);

                if (chordInputVal >= 2.0f) {
                    octavesDifference = 2.0f;  // 2 octaves up
                } else if (chordInputVal >= 1.0f) {
                    octavesDifference = 1.0f;  // 1 octave up
                } else if (chordInputVal >= 0.0f) {
                    octavesDifference = 0.0f;  // No octave shift
                } else if (chordInputVal >= -1.0f) {
                    octavesDifference = -1.0f; // 1 octave down
                } else if (chordInputVal >= -2.0f) {
                    octavesDifference = -2.0f; // 2 octaves down
                } else if (chordInputVal < -2.0f) {
                    octavesDifference = -2.0f; // max is 2 octaves down
                }       
                
                int noteIndex = static_cast<int>(round(chordInputVal * 12)); // Total semitones from C
                int noteRelativeToC = (noteIndex % 12 + 12) % 12; // Ensuring a positive result

                // Dynamically select the appropriate noteToChordPosition array based on knobRowPosition
                const std::array<int, 12>* noteToChordPosition;
                switch (knobRowPosition) {
                    case 0: noteToChordPosition = &noteToChordPositionRow1; break;
                    case 1: noteToChordPosition = &noteToChordPositionRow2; break;
                    case 2: noteToChordPosition = &noteToChordPositionRow3; break;
                    case 3: noteToChordPosition = &noteToChordPositionRow4; break;
                    default: noteToChordPosition = &noteToChordPositionRow1; break; // Default case to avoid uninitialized usage
                }

                // Compute the chord position based on the selected noteToChordPosition array
                int computedChordPosition = (*noteToChordPosition)[noteRelativeToC];
                if (computedChordPosition == -1) {
                    // Search for the next lower note that maps to a chord
                    for (int i = noteRelativeToC - 1; i >= 0; --i) {
                        if ((*noteToChordPosition)[i] != -1) {
                            computedChordPosition = (*noteToChordPosition)[i];
                            semitoneDifference = noteRelativeToC - i;
                            break;
                        }
                    }
                    // If no lower note is found in the octave, wrap around and look from the top
                    if (computedChordPosition == -1) {
                        for (int i = 11; i > noteRelativeToC; --i) {
                            if ((*noteToChordPosition)[i] != -1) {
                                computedChordPosition = (*noteToChordPosition)[i];
                                semitoneDifference = noteRelativeToC + (12 - i); // Adjusting for wrap-around
                                break;
                            }
                        }
                    }
                } else {
                    semitoneDifference = 0; // No difference if the note directly maps to a chord position
                }
                knobChordPosition = computedChordPosition;
            }

            // Clamp and adjust CapoAmount based on semitoneDifference and octavesDifference
            semitoneDifference = clamp(semitoneDifference, 0, 10);
            CapoAmount += semitoneDifference / 12.f + octavesDifference;

            // Check for button presses
            bool buttonPressed = false;
            for (int i = 0; i < NUM_PARAMS - CHORD_BUTTON_1; i++) {
                if (params[CHORD_BUTTON_1 + i].getValue() > 0) {
                    latchedRowIndex = i / MAX_CHORDS_PER_ROW;
                    latchedChordIndex = i % MAX_CHORDS_PER_ROW;
                    buttonPressed = true;
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

            // Select the arrays based on currentRowIndex
            const auto& currentNames = selectChordBankStrings(ChordBank, Row1_Names, Row2_Names, Row3_Names, Row4_Names, Row1_NamesB, Row2_NamesB, Row3_NamesB, Row4_NamesB);
            const auto& currentChords = selectChordBankStrings(ChordBank, Row1_Chords, Row2_Chords, Row3_Chords, Row4_Chords, Row1_ChordsB, Row2_ChordsB, Row3_ChordsB, Row4_ChordsB);
            const auto& currentRoots = ChordBank ? 
            (currentRowIndex == 0) ? Row1_RootsB : (currentRowIndex == 1) ? Row2_RootsB : (currentRowIndex == 2) ? Row3_RootsB : Row4_RootsB :
            (currentRowIndex == 0) ? Row1_Roots : (currentRowIndex == 1) ? Row2_Roots : (currentRowIndex == 2) ? Row3_Roots : Row4_Roots;

            // Set the root note voltage
            outputs[ROOT_NOTE_CV_OUT].setVoltage(currentRoots[currentChordIndex] + CapoAmount);

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
            barreGateActive = inputs[BARRE_CHORD_GATE].isConnected() ? barreLatched ^ (inputs[BARRE_CHORD_GATE].getVoltage() > 0.05f) : barreLatched;
            altGateActive = inputs[ALT_CHORD_GATE].isConnected() ? altLatched ^ (inputs[ALT_CHORD_GATE].getVoltage() > 0.05f) : altLatched;

            // Update lights based on latched state or external gate activity
            lights[BARRE_CHORD_LIGHT].setBrightness(barreGateActive ? 1.0 : 0.0);
            lights[ALT_CHORD_LIGHT].setBrightness(altGateActive ? 1.0 : 0.0);

            // Use the modified gate states to choose the fingering version
            int fingeringVersion = getFingeringVersion(barreGateActive, altGateActive);

            // Determine the connection state of the outputs
            bool firstOutputConnected = outputs[STRING_CV_OUT_1].isConnected();
            bool multipleOutputsConnected = false;
            bool firstMuteOutputConnected = outputs[MUTE_OUT_1].isConnected();
            bool multipleMuteOutputsConnected = false;
            
            for (int i = 1; i < 6; ++i) {
                if (outputs[STRING_CV_OUT_1 + i].isConnected()) {
                    multipleOutputsConnected = true;
                    break;
                }
            }
            
            for (int i = 1; i < 6; ++i) {
                if (outputs[MUTE_OUT_1 + i].isConnected()) {
                    multipleMuteOutputsConnected = true;
                    break;
                }
            }
            
            // Adjust the polyphony of the first outputs based on the current connection state
            if (firstOutputConnected && multipleOutputsConnected) {
                // If the first output was polyphonic and additional outputs are now connected, reset it to mono
                outputs[STRING_CV_OUT_1].setChannels(1);
            }
            if (firstMuteOutputConnected && multipleMuteOutputsConnected) {
                // If the first mute output was polyphonic and additional mute outputs are now connected, reset it to mono
                outputs[MUTE_OUT_1].setChannels(1);
            }
            
            // Iterate over strings to set voltages
            for (int stringIdx = 0; stringIdx < 6; ++stringIdx) {
                // Convert fingering string to semitone shifts
                auto semitoneShifts = fingeringToSemitoneShifts(currentChords[currentChordIndex][fingeringVersion]);
            
                PitchBend[stringIdx] = (0.1f / 12.f) * (inputs[ENVELOPE_IN_1 + stringIdx].isConnected() ? inputs[ENVELOPE_IN_1 + stringIdx].getVoltage() : 0);
                PitchBend[stringIdx] = abs(PitchBend[stringIdx]);
            
                // Calculate pitch voltage
                float pitchVoltage;
                float muteVoltage;
                if (semitoneShifts[stringIdx] >= 0) {
                    pitchVoltage = baseFrequencies[stringIdx] + (semitoneShifts[stringIdx] * (1.0f / 12.0f)) + whammyBarEffect + CapoAmount + PitchBend[stringIdx];
                    if (!InvertMutes){muteVoltage = 0.0f;} else {muteVoltage = 10.0f;} // Not muted
                } else {
                    // Mute this string
                    pitchVoltage = currentRoots[currentChordIndex] + CapoAmount - 1.0f;  // set muted string to the root note, just in case you don't mute it
                    if (!InvertMutes){muteVoltage = 10.0f;} else {muteVoltage = 0.f;} // Muted
                }
            
                // Handle STRING_CV_OUT output routing
                if (firstOutputConnected && !multipleOutputsConnected) {
                    // Only the first output is connected, send polyphonic output
                    outputs[STRING_CV_OUT_1].setChannels(6);
                    outputs[STRING_CV_OUT_1].setVoltage(pitchVoltage, stringIdx);
                } else {
                    // Multiple outputs are connected, send monophonic output per string
                    outputs[STRING_CV_OUT_1 + stringIdx].setVoltage(pitchVoltage);
                }
            
                // Handle MUTE_OUT output routing
                if (firstMuteOutputConnected && !multipleMuteOutputsConnected) {
                    // Only the first mute output is connected, send polyphonic mute output
                    outputs[MUTE_OUT_1].setChannels(6);
                    outputs[MUTE_OUT_1].setVoltage(muteVoltage, stringIdx);
                } else {
                    // Multiple mute outputs are connected, send monophonic mute output per string
                    outputs[MUTE_OUT_1 + stringIdx].setVoltage(muteVoltage);
                }
            }
            
            // If only the first STRING_CV_OUT is connected, clear the other outputs to avoid unintended voltages
            if (firstOutputConnected && !multipleOutputsConnected) {
                for (int i = 1; i < 6; ++i) {
                    outputs[STRING_CV_OUT_1 + i].setVoltage(0.0f); // Ensure other outputs are silent
                    outputs[STRING_CV_OUT_1 + i].setChannels(1); // Set these channels to monophonic
                }
            }
            
            // If only the first MUTE_OUT is connected, clear the other mute outputs to avoid unintended voltages
            if (firstMuteOutputConnected && !multipleMuteOutputsConnected) {
                for (int i = 1; i < 6; ++i) {
                    outputs[MUTE_OUT_1 + i].setVoltage(0.0f); // Ensure other mute outputs are silent
                    outputs[MUTE_OUT_1 + i].setChannels(1); // Set these channels to monophonic
                }
            }

            if(display_count>display_skip){
                display_count=0;        

                if (Row1Display && Row2Display && Row3Display && Row4Display) { 
                    if (settings::preferDarkPanels){
                        Row1Display->fgColor = nvgRGB(250, 250, 250); // White color text
                        Row2Display->fgColor = nvgRGB(250, 250, 250); // White color text
                        Row3Display->fgColor = nvgRGB(250, 250, 250); // White color text
                        Row4Display->fgColor = nvgRGB(250, 250, 250); // White color text
                        CVModeDisplay->fgColor = nvgRGB(250, 250, 250); // White color text
                    } else {
                        Row1Display->fgColor = nvgRGB(10, 10, 10); // Dark color text
                        Row2Display->fgColor = nvgRGB(10, 10, 10); // Dark color text
                        Row3Display->fgColor = nvgRGB(10, 10, 10); // Dark color text
                        Row4Display->fgColor = nvgRGB(10, 10, 10); // Dark color text
                        CVModeDisplay->fgColor = nvgRGB(10, 10, 10); // Dark color text
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

                        // Compute the note name of the capoed root
                        float pitchVoltage = currentRoots[currentChordIndex] + CapoAmount;
                        double fractionalPart = fmod(pitchVoltage, 1.0);
                        int semitone = round(fractionalPart * 12);
                        semitone = (semitone % 12 + 12) % 12;
                        const char* noteNames[12] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
                        const char* noteName = noteNames[semitone];

                        if (CapoAmount ==0){
                            fingeringDisplay->text = currentFingeringPattern;
                        } else {
                            // Update the fingering display text with the current fingering pattern and capo setting
                            if (CapoAmount > -0.01){
                                fingeringDisplay->text = currentFingeringPattern + " +" + capoAmountStr + " " + noteName;    
                            } else {
                                fingeringDisplay->text = currentFingeringPattern + "  " + capoAmountStr + " " + noteName;    
                            }
                        }
                
                        if (chordDiagram) {
                            auto semitoneShifts = fingeringToSemitoneShifts(currentChords[currentChordIndex][fingeringVersion]);
                            chordDiagram->setFingering(semitoneShifts);
                        }  
 
                        if (!ChordBank){   
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
                            if (CVModeDisplay) {        
                                //Mark the knob with the mode setting, 
                                auto CVdisplaytext = "V/oct";
                                if (VOctCV){CVdisplaytext = "(V/Oct)";}
                                else {CVdisplaytext = "        ";}
                                CVModeDisplay->text = CVdisplaytext;
                            } 
 
                        } else {

                            if (Row1Display) {        
                                //{"B7", "B"     , "B2", "B6"},
                                auto row1text = "Row1";
                                if (fingeringVersion == 0){row1text = "7";}
                                else if (fingeringVersion == 1){row1text = "7 Bar";}
                                else if (fingeringVersion == 2){row1text = "2";}
                                else if (fingeringVersion == 3){row1text = "6";}
                                Row1Display->text = row1text;
                            }

                            if (Row2Display) {        
                                //    {"A" ,"A-Bar" ,"Amaj7" ,"A7+5"}, 
                                auto row2text = "Row2";
                                if (fingeringVersion == 0){row2text = "Maj";}
                                else if (fingeringVersion == 1){row2text = "M Bar";}
                                else if (fingeringVersion == 2){row2text = "Maj7";}
                                else if (fingeringVersion == 3){row2text = "7+5";}
                                Row2Display->text = row2text;
                            }

                            if (Row3Display) {        
                                //{{"Em" ,"Em-Bar" ,"Em7"    ,"Em6"},
                                auto row3text = "Row3";
                                if (fingeringVersion == 0){row3text = "min";}
                                else if (fingeringVersion == 1){row3text = "m Bar";}
                                else if (fingeringVersion == 2){row3text = "m7";}
                                else if (fingeringVersion == 3){row3text = "m6";}
                                Row3Display->text = row3text;
                            }

                            if (Row4Display) {        
                                //"Adim" ,"Adim7" ,"A9" ,"Aaug"}, 
                                auto row4text = "Row4";
                                if (fingeringVersion == 0){row4text = "dim";}
                                else if (fingeringVersion == 1){row4text = "dim7";}
                                else if (fingeringVersion == 2){row4text = "9";}
                                else if (fingeringVersion == 3){row4text = "aug";}
                                Row4Display->text = row4text;
                            } 
                    
                            if (CVModeDisplay) {        
                                //Mark the knob with the mode setting, 
                                auto CVdisplaytext = "V/oct";
                                if (VOctCV){CVdisplaytext = "(V/Oct)";}
                                else {CVdisplaytext = "        ";}
                                CVModeDisplay->text = CVdisplaytext;
                            } 
                        }
                 
                        triggerPulse.trigger(0.001f); // 1ms pulse
                    }
                }
            }
            
            if (triggerPulse.process(args.sampleTime)) {
                outputs[TRIGGER_OUT].setVoltage(10.f); // High voltage of the pulse
            } else {
                outputs[TRIGGER_OUT].setVoltage(0.f); // Low voltage, pulse off
            }
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
        addParam(createParamCentered<DiscreteRoundBlackKnob>(Vec(30,  30), module, Strings::CHORD_SELECTOR_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(30,   65), module, Strings::CHORD_SELECTOR_CV));

        addParam(createParamCentered<DiscreteRoundBlackKnob>(Vec(30, 40+80), module, Strings::ROW_SELECTOR_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(30,  75+80), module, Strings::ROW_SELECTOR_CV));

        addParam(createParamCentered<DiscreteRoundBlackKnob>(Vec(270, 30), module, Strings::CAPO_PARAM));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(270,  65), module, Strings::CAPO_CV));

        // CV Mode Indicator
        DigitalDisplay* CVModeDisplay = new DigitalDisplay();
        CVModeDisplay->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        CVModeDisplay->box.pos = Vec(45,   75); // Position below the first display
        CVModeDisplay->box.size = Vec(30, 18); // Size of the display
        CVModeDisplay->text = " "; // Initial text or placeholder
        CVModeDisplay->setTextAlign(NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
        if (settings::preferDarkPanels){
            CVModeDisplay->fgColor = nvgRGB(250, 250, 250); // White color text
        } else {
            CVModeDisplay->fgColor = nvgRGB(10, 10, 10); // Dark color text
        }
        CVModeDisplay->textPos = Vec(47,   78); // Text position
        CVModeDisplay->setFontSize(10.0f); // Set the font size as desired

        addChild(CVModeDisplay);

        if (module) {
            module->CVModeDisplay = CVModeDisplay;
        }

        // Gate Inputs

        addInput(createInputCentered<ThemedPJ301MPort>(Vec(30, 220), module, Strings::BARRE_CHORD_GATE));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(60, 220), module, Strings::ALT_CHORD_GATE));

        addParam(createParamCentered<LEDButton>           (Vec(30, 195), module, Strings::BARRE_CHORD_BUTTON));
        addChild(createLightCentered<SmallLight<RedLight>>(Vec(30, 195), module, Strings::BARRE_CHORD_LIGHT));
        addParam(createParamCentered<LEDButton>           (Vec(60, 195), module, Strings::ALT_CHORD_BUTTON));
        addChild(createLightCentered<SmallLight<RedLight>>(Vec(60, 195), module, Strings::ALT_CHORD_LIGHT));

        float left = 35; //sets the left edge
        float jack = 35; //sets spacing between jacks

        // Inputs
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(left,        270), module, Strings::ENVELOPE_IN_1));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(1*jack+left, 270), module, Strings::ENVELOPE_IN_2));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(2*jack+left, 270), module, Strings::ENVELOPE_IN_3));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(3*jack+left, 270), module, Strings::ENVELOPE_IN_4));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(4*jack+left, 270), module, Strings::ENVELOPE_IN_5));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(5*jack+left, 270), module, Strings::ENVELOPE_IN_6));
        addInput(createInputCentered<ThemedPJ301MPort>(Vec(6.5*jack+left, 270), module, Strings::WHAMMY_BAR_CV));

        // Outputs              
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(left,        310), module, Strings::MUTE_OUT_1));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(1*jack+left, 310), module, Strings::MUTE_OUT_2));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(2*jack+left, 310), module, Strings::MUTE_OUT_3));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(3*jack+left, 310), module, Strings::MUTE_OUT_4));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(4*jack+left, 310), module, Strings::MUTE_OUT_5));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(5*jack+left, 310), module, Strings::MUTE_OUT_6));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(6.5*jack+left, 310), module, Strings::TRIGGER_OUT));

        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(left,        340), module, Strings::STRING_CV_OUT_1));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(1*jack+left, 340), module, Strings::STRING_CV_OUT_2));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(2*jack+left, 340), module, Strings::STRING_CV_OUT_3));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(3*jack+left, 340), module, Strings::STRING_CV_OUT_4));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(4*jack+left, 340), module, Strings::STRING_CV_OUT_5));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(5*jack+left, 340), module, Strings::STRING_CV_OUT_6));
        addOutput(createOutputCentered<ThemedPJ301MPort>(Vec(6.5*jack+left, 340), module, Strings::ROOT_NOTE_CV_OUT));

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
        if (settings::preferDarkPanels){
            Row1Display->fgColor = nvgRGB(250, 250, 250); // White color text
        } else {
            Row1Display->fgColor = nvgRGB(10, 10, 10); // Dark color text
        }
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
        if (settings::preferDarkPanels){
            Row2Display->fgColor = nvgRGB(250, 250, 250); // White color text
        } else {
            Row2Display->fgColor = nvgRGB(10, 10, 10); // Dark color text
        }
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
        if (settings::preferDarkPanels){
            Row3Display->fgColor = nvgRGB(250, 250, 250); // White color text
        } else {
            Row3Display->fgColor = nvgRGB(10, 10, 10); // Dark color text
        }
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
        if (settings::preferDarkPanels){
            Row4Display->fgColor = nvgRGB(250, 250, 250); // White color text
        } else {
            Row4Display->fgColor = nvgRGB(10, 10, 10); // Dark color text
        }
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
    
    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);

        Strings* StringsModule = dynamic_cast<Strings*>(module);
        assert(StringsModule); // Ensure the cast succeeds

        // Separator for visual grouping in the context menu
        menu->addChild(new MenuSeparator());

        // ChordBank menu item
        struct ChordBankMenuItem : MenuItem {
            Strings* StringsModule;
            void onAction(const event::Action& e) override {
                StringsModule->ChordBank = !StringsModule->ChordBank;
            }
            void step() override {
                rightText = StringsModule->ChordBank ? "✔" : "";
                MenuItem::step();
            }
        };

        ChordBankMenuItem* chordBankItem = new ChordBankMenuItem();
        chordBankItem->text = "Classical Chord Set";
        chordBankItem->StringsModule = StringsModule;
        menu->addChild(chordBankItem);

        // VOctCV menu item
        struct VOctCVMenuItem : MenuItem {
            Strings* StringsModule;
            void onAction(const event::Action& e) override {
                // Toggle the "CHORD reads V/oct" mode
                StringsModule->VOctCV = !StringsModule->VOctCV;
            }
            void step() override {
                // Update the display to show a checkmark when the mode is active
                rightText = StringsModule->VOctCV ? "✔" : "";
                MenuItem::step();
            }
        };

        VOctCVMenuItem* vOctCVItem = new VOctCVMenuItem();
        vOctCVItem->text = "CHORD input in V/oct";
        vOctCVItem->StringsModule = StringsModule; // Ensure we're setting the module
        menu->addChild(vOctCVItem);

        // InvertMute
        struct InvertMutesMenuItem : MenuItem {
            Strings* StringsModule;
            void onAction(const event::Action& e) override {
                // Toggle the "Invert Mutes" mode
                StringsModule->InvertMutes = !StringsModule->InvertMutes;
            }
            void step() override {
                // Update the display to show a checkmark when the mode is active
                rightText = StringsModule->InvertMutes ? "✔" : "";
                MenuItem::step();
            }
        };

        InvertMutesMenuItem* invertMutesItem = new InvertMutesMenuItem();
        invertMutesItem->text = "Invert Mute Gate Outputs";
        invertMutesItem->StringsModule = StringsModule; // Ensure we're setting the module
        menu->addChild(invertMutesItem);
    }

};

Model* modelStrings = createModel<Strings, StringsWidget>("Strings");