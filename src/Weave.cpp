////////////////////////////////////////////////////////////
//
//   Weave
//
//   written by Cody Geary
//   Copyright 2025, MIT License
//
//   Six-channel chord generator with unique output permutation options
//
////////////////////////////////////////////////////////////


#include "rack.hpp"
#include "plugin.hpp"
#include "digital_display.hpp"
#include <array>
#include <string>
#include <vector>
#include <cmath>

// Base frequencies for each guitar string
const float baseFrequencies[6] =  { -1.6667f, // E2
                                    -1.25f, // A2
                                    -0.8333f, // D3
                                    -0.4167f,  // G3
                                    -0.0833f, // B3
                                     0.3333f};// E4

const int WEAVE_PATTERNS=21; //total weave patterns

//WEAVE PATTERNS
const std::array<std::array<int, 6>, WEAVE_PATTERNS> Weave_Chart = {{

    {0,1,2,3,4,5}, //default, no weave - 0
    {5,0,1,2,3,4}, //6-state: rotate
    {1,2,3,4,5,0}, //rotate rev

    {5,4,3,2,1,0}, //2-state - full flip
    {1,0,3,2,5,4}, //          pair flips
    {2,4,0,5,1,3},
    {3,4,5,0,1,2},
    {4,5,3,2,0,1},

    {2,0,1,4,5,3}, //3-state
    {2,0,1,5,3,4},

    {4,5,0,1,2,3}, //4-state
    {3,0,4,1,5,2},
    {3,2,5,4,0,1},

    {1,2,5,0,3,4},
    {3,0,1,4,5,2},
    {1,2,4,5,0,3},
    {1,3,5,0,2,4},
    {3,4,5,2,1,0},
    {3,2,5,4,1,0},
    {4,5,3,0,1,2},
    {3,4,1,2,5,0}
}};

struct Weave : Module {
    enum ParamId {
        WEAVE_KNOB_PARAM, WEAVE_ATT_PARAM,
        CHORD_KNOB_PARAM,
        OCTAVE_DOWN_BUTTON, OCTAVE_UP_BUTTON,
        TRIG_BUTTON, RESET_BUTTON,
        SHIFT_KNOB_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        TRIG_INPUT,
        RESET_INPUT,
        WEAVE_INPUT,
        NOTE_INPUT,
        CHORD_INPUT,
        SHIFT_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        TRIG_OUTPUT,
        POLY_OUTPUT,
        OUTPUT_1, OUTPUT_2, OUTPUT_3, OUTPUT_4, OUTPUT_5, OUTPUT_6, OUTPUT_ROOT,
        OUTPUTS_LEN
    };
    enum LightId {
        CHORD_1_LIGHT, CHORD_2_LIGHT, CHORD_3_LIGHT, CHORD_4_LIGHT, CHORD_5_LIGHT,
        CHORD_6_LIGHT, CHORD_7_LIGHT, CHORD_8_LIGHT, CHORD_9_LIGHT, CHORD_10_LIGHT,
        CHORD_11_LIGHT, CHORD_12_LIGHT, CHORD_13_LIGHT, CHORD_14_LIGHT, CHORD_15_LIGHT,
        CHORD_16_LIGHT, CHORD_17_LIGHT, OCTAVE_DOWN_LIGHT, OCTAVE_UP_LIGHT,
        LIGHTS_LEN
    };

    //For the trigger inputs
    dsp::SchmittTrigger resetInput, resetButton, trigInput, trigButton, octUpTrigger, octDownTrigger;

    // For the chord and keyboard
    bool playingNotes[12] = {true, false, false, false, false, false, false, false, false, false, false, false};
    bool noteClicked = false;
    int noteValue = 0; // Default to 0 to C
    int prevNoteValue = 0;
    int chordIndex = 0; // Default to 0 to indicate Octaves
    int prevChordIndex = -1;
    int octaveState = 0;
    float currentNotes[6] = {-3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
    float finalNotes[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    float previousFinalNotes[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    float previousStageNotes[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    bool upButtonPressed = false; // Check if octave up or down buttons are pressed
    bool downButtonPressed = false;
    dsp::PulseGenerator notePulseGen; // Trigger generator for note previewing

    // For the note offsets
    int noteOffset[6] = {0,0,0,0,0,0};
    int currentPermute[6] = {0,1,2,3,4,5};
    float extOffset = 0.0f; // For the transpose input
    int processSkipper = 0;
    int processSkips = 100;
    bool prevNoteConnected = false;
    int weaveSetting = 0;

    //Context Option
    bool quantizeShift = false;
    bool inputTracksOctaves = false;
    float inputOctaveOffset = 0.f;

    const std::array<std::array<std::string, 16>, 12> Chord_Chart = {{
        // Maj       min        7        Maj7       min7      6        min6       9         Maj9     min9      add9      sus2      sus4       pow       aug        dim
        // Row for C chords
        {"X32010", "X31013", "X35356", "X32000", "X31313", "X32253", "X31213", "X30310", "X30000", "X30343", "X32033", "X30013", "X33011", "835588", "X32110", "X3454X"},
        // Row for C# chords
        {"X43121", "X42120", "X43101", "X43114", "X42100", "X43364", "X42324", "X41101", "X41111", "X41100", "X43141", "X41124", "X44122", "946699", "X43225", "X42020"},
        // Row for D chords
        {"XX0232", "XX0231", "XX0212", "XX0222", "XX0211", "XX0202", "XX0201", "X52532", "X52222", "X53555", "X54252", "XX0230", "5X0233", "X5023X", "XX0332", "XX0131"},
        // Row for D# chords
        {"XX1343", "XX1342", "XX1323", "XX1333", "XX1322", "XX1313", "XX1312", "XX1021", "XX1031", "X64666", "X65363", "XX1341", "XX1344", "XX1346", "XX1003", "XX1242"},
        // Row for E chords
        {"022100", "022000", "020100", "021100", "020000", "042100", "042000", "020102", "021102", "020002", "024100", "024400", "002200", "022450", "032110", "0120XX"},
        // Row for F chords
        {"133211", "133114", "101211", "102210", "133141", "100211", "130114", "101013", "102010", "131044", "103013", "133011", "133311", "133561", "1X3221", "12310X"},
        // Row for F# chords
        {"244322", "244225", "XX4320", "XX4321", "202220", "XX4646", "201222", "212122", "213121", "202120", "214122", "XX4124", "244422", "244672", "2X4332", "XX4212"},
        // Row for G chords
        {"320003", "310033", "320001", "320002", "310031", "320000", "310030", "300001", "300002", "300331", "300003", "300033", "330013", "355033", "321003", "3453XX"},
        // Row for G# chords
        {"431114", "466447", "431112", "431113", "424444", "431141", "421141", "411312", "411313", "XX6476", "411114", "XX6346", "466644", "466144", "XX6554", "420104"},
        // Row for A chords
        {"X02220", "X02210", "X02020", "X02120", "X02010", "X04220", "X04210", "X02423", "X02424", "X02413", "X02420", "X02200", "X00230", "5022X5", "X03221", "XX7545"},
        // Row for A# chords
        {"X10331", "X13321", "X10131", "X10231", "X13124", "X10031", "X13023", "X10314", "X10211", "XX8698", "633536", "X13311", "X13341", "613366", "X10332", "X12320"},
        // Row for B chords
        {"X24442", "X24432", "X21202", "X21302", "X20202", "X21102", "X20132", "X21222", "744646", "X20222", "744647", "X24422", "X24452", "799402", "X21003", "X23431"}
    }};
    // Root offset for alternative bass notes for different chord types (Offset 1)
    const std::array<float, 16> Root_Offset1 = {
        0.0f,    // Maj - Root is fine
        0.0f,    // Min - Root is fine
        0.58333f,    // 7 - 5th works well as an alternative
        0.3333f, // Maj7 - 3rd often works well
        0.0f, // Min7 - 3rd can work well
        0.3333f, // 6 - 3rd can work well
        0.58333f, // Min6 - 5th can work well
        0.58333f,    // 9 - 5th can provide better stability
        0.3333f, // Maj9 - 3rd or 5th can work well
        0.0f, // Min9 - Root
        0.0f,    // Add9 -
        0.0f,    // Sus2 -
        0.0f,    // Sus4
        1.0f,    // 5 -root
        0.0f, // Aug
        0.0f  // Dim - b3rd or b5th works better
    };
    // Root offset for alternative bass notes for different chord types (Offset 2)
    const std::array<float, 16> Root_Offset2 = {
        0.58333f,    // Maj - 5th can add more harmonic fullness
        0.58333f,    // Min - 5th can add more harmonic fullness
        0.0f,    // 7 - Root can also work, depending on context
        0.58333f,    // Maj7 - 5th can work
        0.58333f,    // Min7 - 5th can work
        0.58333f,    // 6 - 5th can work
        0.41666f,    // Min6 - 3rd can work
        0.3333f, // 9 - 3rd can also provide a unique flavor
        0.58333f,    // Maj9 - 5th provides stability
        0.58333f,    // Min9 - 5th provides stability
        0.41666f, // Add9 - 5TH
        0.58333f, // Sus2 - 5TH
        0.41666f, // Sus4 - 4TH
        0.58333f,    // 5 - 5th
        1.0f,    // Aug -high root
        0.0f     // Dim -high root
    };

    json_t* dataToJson() override {
        json_t* rootJ = json_object();

        // Existing state
        json_object_set_new(rootJ, "octaveState", json_integer(octaveState));
        json_t* permuteJ = json_array();
        for (int i = 0; i < 6; i++)
            json_array_append_new(permuteJ, json_integer(currentPermute[i]));
        json_object_set_new(rootJ, "currentPermute", permuteJ);

        // Existing flags
        json_object_set_new(rootJ, "quantizeShift", json_boolean(quantizeShift));

        // --- NEW: input octave tracking flag ---
        json_object_set_new(rootJ, "inputTracksOctaves", json_boolean(inputTracksOctaves));

        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        // Existing loads
        json_t* octaveStateJ = json_object_get(rootJ, "octaveState");
        if (octaveStateJ)
            octaveState = json_integer_value(octaveStateJ);

        json_t* permuteJ = json_object_get(rootJ, "currentPermute");
        if (permuteJ) {
            for (int i = 0; i < 6; i++) {
                json_t* valJ = json_array_get(permuteJ, i);
                if (valJ)
                    currentPermute[i] = json_integer_value(valJ);
            }
        }

        json_t* quantizeShiftJ = json_object_get(rootJ, "quantizeShift");
        if (quantizeShiftJ)
            quantizeShift = json_boolean_value(quantizeShiftJ);

        // --- NEW: input octave tracking flag ---
        json_t* inputTracksOctavesJ = json_object_get(rootJ, "inputTracksOctaves");
        if (inputTracksOctavesJ)
            inputTracksOctaves = json_boolean_value(inputTracksOctavesJ);
    }

    Weave() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(WEAVE_KNOB_PARAM, 0.f, WEAVE_PATTERNS-1, 0.f, "Weave");
        configParam(WEAVE_ATT_PARAM, -1.f, 1.0, 1.f, "Weave Att.");
        configParam(CHORD_KNOB_PARAM, 0.f, 1.41666f, 0.f, "Chord");
        configInput(TRIG_INPUT, "Weave Trig.");
        configInput(RESET_INPUT, "Reset Trig.");

#ifdef METAMODULE
        configInput(WEAVE_INPUT, "Weave CV");
        configInput(NOTE_INPUT, "Root Note");        
        configInput(CHORD_INPUT, "Chord CV");
        configInput(SHIFT_INPUT, "Shift");
        configOutput(POLY_OUTPUT, "Poly: Note 1");
#else
        configInput(WEAVE_INPUT, "Weave CV 1V/pattern");
        configInput(NOTE_INPUT, "Root Note V/oct");        
        configInput(CHORD_INPUT, "Chord (1 semitone per value)");
        configInput(SHIFT_INPUT, "Transpose Shift");
        configOutput(POLY_OUTPUT, "Poly V/Oct");
#endif

        configOutput(TRIG_OUTPUT, "Note Trigger");
        configParam(TRIG_BUTTON, 0.f, 1.f, 0.f, "Trigger Weave");
        configParam(RESET_BUTTON, 0.f, 1.f, 0.f, "Reset Weave");
        configParam(SHIFT_KNOB_PARAM, -1.f, 1.f, 0.f, "Transpose");
        configParam(OCTAVE_DOWN_BUTTON, 0.f, 1.f, 0.f, "Octave Down");
        configParam(OCTAVE_UP_BUTTON, 0.f, 1.f, 0.f, "Octave Up");

        configOutput(OUTPUT_1, "Note 1");
        configOutput(OUTPUT_2, "Note 2");
        configOutput(OUTPUT_3, "Note 3");
        configOutput(OUTPUT_4, "Note 4");
        configOutput(OUTPUT_5, "Note 5");
        configOutput(OUTPUT_6, "Note 6");
        configOutput(OUTPUT_ROOT, "Root Note");
    }

    void process(const ProcessArgs& args) override {

        bool noteOrChordPressed = false;
        bool inputNotPoly = true;
        bool noteInputConnected = false;
        double deltaTime = args.sampleTime;

        processSkipper++;
        if (processSkipper>=processSkips){
            // Track connection states of each output
            bool noteConnected = outputs[POLY_OUTPUT].isConnected();

            // Re-initialize polyphonic channels only if connections change
            if (noteConnected && !prevNoteConnected) {
                outputs[POLY_OUTPUT].setChannels(6);
            }

            // Update previous connection states
            prevNoteConnected = noteConnected;

            // Check if NOTE_INPUT is connected
            if (inputs[NOTE_INPUT].isConnected()) {
                noteInputConnected = true;
            }

            processSkipper = 0;
        }

        if (noteInputConnected ){
            int inputChannels = inputs[NOTE_INPUT].getChannels();

            if (inputChannels <= 0) { // Surely an unnecessary safety check
                inputChannels = 1; // Safety fallback
            }
            inputChannels = std::min(inputChannels, 16); // VCV Rack limit

            if (inputChannels == 1) {
                // Monophonic V/OCT input - treat as a root note + chord
                inputNotPoly = true;

                // Read the voltage from NOTE_INPUT and quantize it to determine which note to activate

                float noteVoltage = inputs[NOTE_INPUT].getVoltage();
                int quantizedNote = static_cast<int>(std::roundf(noteVoltage * 12.0f));
                int octaveOffset = 0;

                if (inputTracksOctaves) {
                    // Preserve the octave information
                    octaveOffset = static_cast<int>(std::floor(noteVoltage));
                    // Make sure we wrap the note index properly to 0–11 range
                    quantizedNote = ((quantizedNote % 12) + 12) % 12;
                    inputOctaveOffset = static_cast<float>(octaveOffset);
                } else {
                    // Classic behavior – wrap to single octave
                    while (quantizedNote < 0)
                        quantizedNote += 12;
                    while (quantizedNote > 11)
                        quantizedNote -= 12;
                    inputOctaveOffset = 0.f;
                }

                // Update the playingNotes array
                for (int i = 0; i < 12; i++) {
                    playingNotes[i] = (i == quantizedNote);
                }

                noteValue = quantizedNote; // Store the quantized note
            } else {
                // Polyphonic V/OCT input - spread notes out over 6 channels
                inputNotPoly = false;

                // Clear the playingNotes array
                for (int i = 0; i < 12; i++) {
                    playingNotes[i] = false;
                }

                // Number of notes to distribute
                const int totalNotes = 6;

                // Calculate how many notes each channel should control
                int notesPerChannel = totalNotes / inputChannels;
                int extraNotes = totalNotes % inputChannels; // For uneven divisions

                int noteIndex = 0; // Start index for assigning notes

                for (int nt = 0; nt<12; nt++){
                    playingNotes[nt] = false; //clear out the playing notes
                }

                // Loop over each channel
                for (int ch = 0; ch < inputChannels; ch++) {
                    // Determine the number of notes for this channel
                    int channelNotes = notesPerChannel + (ch < extraNotes ? 1 : 0);

                    // Read the voltage from the current channel
                    float noteVoltage = inputs[NOTE_INPUT].getVoltage(ch);
                    int quantizedNote = static_cast<int>(std::roundf(noteVoltage * 12.0f));
                    while (quantizedNote < 0)
                        quantizedNote += 12;
                    while (quantizedNote > 11)
                        quantizedNote -= 12;

                    for (int nt = 0; nt<12; nt++){
                        if (nt == quantizedNote){
                             playingNotes[nt] = true;
                        }
                    }

                    // Assign the quantized note to the assigned notes for this channel
                    for (int n = 0; n < channelNotes; n++) {
                        if (noteIndex < totalNotes) {
                            currentNotes[noteIndex] = std::roundf(noteVoltage * 12.0f)/12.0f; // Store the quantized note value
                            noteIndex++;
                        }
                    }
                }
            }
        } else {
            inputNotPoly = true;
            // Otherwise, use the current active note from the playingNotes array
            for (int i = 0; i < 12; i++) {
                if (playingNotes[i]) {
                    noteValue = i; // Set noteValue to the active note in playingNotes
                    break;
                }
            }
        }
        if (noteClicked) {
            noteOrChordPressed = true;
            noteClicked = false; //reset noteClick detector from keyboard widget
        }

        // CHORD INPUT HANDLING SECTION

        // Check if CHORD_INPUT is connected
        if (inputNotPoly){
            if (inputs[CHORD_INPUT].isConnected()) {
                // Read the voltage from CHORD_INPUT and quantize it to determine which chord to activate
                float chordVoltage = inputs[CHORD_INPUT].getVoltage();
                chordIndex = static_cast<int>(std::roundf(chordVoltage * 12.0f)); // Quantize to determine the active chord (0-15)

                while (chordIndex < 0)
                    chordIndex += 16;
                while (chordIndex > 15)
                    chordIndex -= 16;

                if (chordIndex != prevChordIndex){
                   noteOrChordPressed = true;
                   prevChordIndex = chordIndex;
               }
            } else {
                // Otherwise, check which chord is selected
                chordIndex = static_cast<int>(params[CHORD_KNOB_PARAM].getValue()*12.f-1.f);
                if (chordIndex != prevChordIndex){
                   noteOrChordPressed = true;
                   prevChordIndex = chordIndex;
                }
            }
        } else {
            chordIndex = -1;
        }

        if (octUpTrigger.process(params[OCTAVE_UP_BUTTON].getValue())) {
            if (!upButtonPressed && octaveState < 1) {
                octaveState++; // Move up an octave
            }
            noteOrChordPressed = true;
            upButtonPressed = true;
        } else {
            upButtonPressed = false;
        }

        if (octDownTrigger.process(params[OCTAVE_DOWN_BUTTON].getValue())) {
            if (!downButtonPressed && octaveState > -1) {
                octaveState--; // Move down an octave
            }
            noteOrChordPressed = true;
            downButtonPressed = true;
        } else {
            downButtonPressed = false;
        }

        // GUITAR FINGERING TO SEMITONE SHIFT CALCULATION
        if ( noteOrChordPressed || noteInputConnected){ //only set the value at the time the note is clicked
            if (chordIndex >= 0 && noteValue >= 0) {
                // Retrieve the chord name and fingering
                const std::string& fingering = Chord_Chart[noteValue][chordIndex];

                // Convert the fingering to semitone shifts
                std::array<int, 6> semitoneShifts = fingeringToSemitoneShifts(fingering);

                // Calculate final voltages for each string
                for (size_t i = 0; i < 6; i++) {
                    if (semitoneShifts[i] == -1) {
                        // String is muted, determine root offset based on chord type
                        float rootOffset = Root_Offset1[chordIndex];
                        if (i == 1) { // If more than one string is muted, use Root_Offset2
                            rootOffset = Root_Offset2[chordIndex];
                        }
                        float finalVoltage = (noteValue / 12.0f) - 2 + (octaveState * 1.0f); // Add octave offset based on octave state
                        currentNotes[5-i] = finalVoltage + rootOffset; //reverse the string order, add appropriate rootOffset for muted strings
                    } else {
                        // Calculate the voltage for the string
                        float finalVoltage = baseFrequencies[i] + (semitoneShifts[i] / 12.0f) + (octaveState * 1.0f); // Add octave offset based on octave state
                        currentNotes[5-i] = finalVoltage;  //reverse the string order
                    }
                }
            }

            if (chordIndex == -1 && noteValue >= 0) {
                currentNotes[0]=(noteValue / 12.0f) - 3.f;
                currentNotes[1]=(noteValue / 12.0f) - 2.f;
                currentNotes[2]=(noteValue / 12.0f) - 1.f;
                currentNotes[3]=(noteValue / 12.0f);
                currentNotes[4]=(noteValue / 12.0f) + 1.f;
                currentNotes[5]=(noteValue / 12.0f) + 2.f;
            }

        }

        for (int i = 0; i < 6; i++){
            finalNotes[i] = currentNotes[i] + noteOffset[i]*(1.0f/12.0f);
        }

        //Weave Section
        if (inputs[WEAVE_INPUT].isConnected()){
            float rawWeave = inputs[WEAVE_INPUT].getVoltage() + params[WEAVE_KNOB_PARAM].getValue();
            weaveSetting = static_cast<int>(roundf(rawWeave));
            weaveSetting = ((weaveSetting % WEAVE_PATTERNS) + WEAVE_PATTERNS) % WEAVE_PATTERNS;
        } else {
            weaveSetting = static_cast<int>(params[WEAVE_KNOB_PARAM].getValue());
            weaveSetting = std::max(0, std::min(weaveSetting, WEAVE_PATTERNS - 1));
        }

        bool applyWeave = false;
        if (inputs[TRIG_INPUT].isConnected()){
            if (trigInput.process(inputs[TRIG_INPUT].getVoltage())){
                applyWeave = true;
            }
        }
        if (trigButton.process(params[TRIG_BUTTON].getValue())){
            applyWeave = true;
        }

        if (applyWeave){
            notePulseGen.trigger(0.001f);
            for (int i=0; i<6; i++){
                if (currentPermute[i] >= 0 && currentPermute[i] < 6 &&
                    weaveSetting >= 0 && weaveSetting < WEAVE_PATTERNS) { //additional bounds checks
                    currentPermute[i] = Weave_Chart[weaveSetting][currentPermute[i]];
                } else {
                    currentPermute[i] = i; // Reset if somehow corrupted
                }
            }
        }

        // Handle button press for Reset
        if (inputs[RESET_INPUT].isConnected()) {
            if (resetInput.process(inputs[RESET_INPUT].getVoltage() ) ){
                for (int i=0; i<6; i++){
                    currentPermute[i] = i;
                }
            }
        }
        if (resetButton.process(params[RESET_BUTTON].getValue())){
            for (int i=0; i<6; i++){
                currentPermute[i] = i;
            }
        }

        //Handle note presses in preview mode while sequencer is paused
        if (noteOrChordPressed){
            notePulseGen.trigger(0.001f);   // Trigger a 1ms pulse (0.001 seconds)
        }

        if (noteValue != prevNoteValue){
            notePulseGen.trigger(0.001f);
            prevNoteValue = noteValue;
        }

        if (notePulseGen.process(deltaTime)) {
            outputs[TRIG_OUTPUT].setVoltage(10.f);  // Output a 10V trigger
        } else {
            outputs[TRIG_OUTPUT].setVoltage(0.0f);   // Otherwise, output 0V
        }

        // Outputs
        extOffset = params[SHIFT_KNOB_PARAM].getValue();
        if (inputs[SHIFT_INPUT].isConnected()) { extOffset += inputs[SHIFT_INPUT].getVoltage(); }

        extOffset += inputOctaveOffset;

        // --- Quantize shift to semitones if enabled ---
        if (quantizeShift) {  extOffset = std::roundf(extOffset * 12.0f) / 12.0f; }

        for (int c = 0; c < 6; c++) {
            float outputNote = clamp(finalNotes[c] + extOffset, -10.f, 10.f);
            outputs[POLY_OUTPUT].setVoltage(outputNote, currentPermute[c]);

            // Also send the same notes to the individual mono outputs
            if (outputs[OUTPUT_1].isConnected()) outputs[OUTPUT_1 + currentPermute[0]].setVoltage(finalNotes[0] + extOffset);
            if (outputs[OUTPUT_2].isConnected()) outputs[OUTPUT_1 + currentPermute[1]].setVoltage(finalNotes[1] + extOffset);
            if (outputs[OUTPUT_3].isConnected()) outputs[OUTPUT_1 + currentPermute[2]].setVoltage(finalNotes[2] + extOffset);
            if (outputs[OUTPUT_4].isConnected()) outputs[OUTPUT_1 + currentPermute[3]].setVoltage(finalNotes[3] + extOffset);
            if (outputs[OUTPUT_5].isConnected()) outputs[OUTPUT_1 + currentPermute[4]].setVoltage(finalNotes[4] + extOffset);
            if (outputs[OUTPUT_6].isConnected()) outputs[OUTPUT_1 + currentPermute[5]].setVoltage(finalNotes[5] + extOffset);
        }

        // --- Root Output Logic ---
        // Find the lowest note voltage among the 6 notes
        float lowestNote = 10.f;
        for (int i = 0; i < 6; i++) {
            if (finalNotes[i] < lowestNote)
                lowestNote = finalNotes[i];
        }

        // Compute the octave based on the lowest note (integer octave range)
        int lowestOctave = static_cast<int>(std::floor(lowestNote));

        // Root is noteValue (0–11) scaled to volts + that octave
        float rootVoltage = (noteValue / 12.0f) + lowestOctave + extOffset;

        // Clamp for safety
        outputs[OUTPUT_ROOT].setVoltage(clamp(rootVoltage, -10.f, 10.f));

    }//end process

    // Helper function to convert a fingering (e.g., "X21202") to semitone shifts
    std::array<int, 6> fingeringToSemitoneShifts(const std::string& fingering) {
        std::array<int, 6> semitoneShifts = {0, 0, 0, 0, 0, 0}; // Initialize shifts
        for (size_t i = 0; i < 6; i++) {
            if (fingering[i] == 'X' || i >= fingering.length()) {
                semitoneShifts[i] = -1; // Muted string, set to -1 to indicate mute
            } else {
                // Convert character to integer and subtract base fret offset
                semitoneShifts[i] = fingering[i] - '0';
            }
        }
        return semitoneShifts;
    }
};

struct WeaveWidget : ModuleWidget {
    DigitalDisplay* noteDisplays[6] = {nullptr};
    DigitalDisplay* chordDisplay = nullptr;

    //Draw the Keyboard
    struct KeyboardKey : OpaqueWidget {
        int note = 0;
        Weave* module = nullptr;

        void drawLayer(const DrawArgs& args, int layer) override {
            if (layer != 1) { return; }
            if (!module) { return; }

            Rect r = box.zeroPos();
            const float margin = mm2px(1.0f);
            Rect rMargin = r.grow(Vec(margin, margin));

            nvgBeginPath(args.vg);
            nvgRect(args.vg, RECT_ARGS(rMargin));
            nvgFillColor(args.vg, nvgRGB(12, 12, 12));
            nvgFill(args.vg);

            nvgBeginPath(args.vg);
            nvgRect(args.vg, RECT_ARGS(r));
            if (module->playingNotes[note]) {
                nvgFillColor(args.vg, nvgRGB(208, 140, 89));
            } else {
                // Determine if it's a white or black key
                bool isWhiteKey = (note == 0 || note == 2 || note == 4 || note == 5 || note == 7 || note == 9 || note == 11);
                if (isWhiteKey) {
                    nvgFillColor(args.vg, nvgRGB(160, 160, 160)); // Lighter color for inactive white keys
                } else {
                    nvgFillColor(args.vg, nvgRGB(24, 24, 24)); // Dark color for inactive black keys
                    // Draw an inner outline to give the 3D effect of the key
                    nvgStrokeWidth(args.vg, 1.5);
                    nvgStrokeColor(args.vg, nvgRGB(50, 50, 50)); // Darker outline color
                    nvgStroke(args.vg);
                }
            }
            nvgFill(args.vg);
        }
        void onDragStart(const event::DragStart& e) override {
            if (e.button == GLFW_MOUSE_BUTTON_LEFT && module) {
                // Unlatch all notes and latch only the clicked note
                for (int i = 0; i < 12; i++) {
                    module->playingNotes[i] = false;
                }
                module->playingNotes[note] = true;
                module->noteClicked = true;
            }
            OpaqueWidget::onDragStart(e);
        }
        void onButton(const event::Button& e) override {
            if (e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT && module) {
                // Unlatch all notes and latch only the clicked note
                for (int i = 0; i < 12; i++) {
                    module->playingNotes[i] = false;
                }
                module->playingNotes[note] = true;
                module->noteClicked = true;
                e.consume(this);
            }
            OpaqueWidget::onButton(e);
        }
    };

    struct KeyboardDisplay : LedDisplay {
        Weave* module = nullptr;

        void setModule(Weave* mod) {
            module = mod;
            if (!module) {return; }
            float disp_offset_a = 3.7f-5.f;
            float disp_offset_b = 2.5f;

            std::vector<Vec> noteAbsPositions = {
                mm2px(Vec(8.259f + disp_offset_a, 86.558f+ disp_offset_b)), // white
                mm2px(Vec(11.286f+ disp_offset_a, 85.049f+ disp_offset_b)), // black
                mm2px(Vec(15.168f+ disp_offset_a, 86.558f+ disp_offset_b)), // white
                mm2px(Vec(19.95f + disp_offset_a, 85.049f+ disp_offset_b)), // black
                mm2px(Vec(22.138f+ disp_offset_a, 86.558f+ disp_offset_b)), // white
                mm2px(Vec(29.048f+ disp_offset_a, 86.558f+ disp_offset_b)), // white
                mm2px(Vec(32.174f+ disp_offset_a, 85.049f+ disp_offset_b)), // black
                mm2px(Vec(36.056f+ disp_offset_a, 86.558f+ disp_offset_b)), // white
                mm2px(Vec(39.931f+ disp_offset_a, 85.049f+ disp_offset_b)), // black
                mm2px(Vec(42.966f+ disp_offset_a, 86.558f+ disp_offset_b)), // white
                mm2px(Vec(47.667f+ disp_offset_a, 85.049f+ disp_offset_b)), // black
                mm2px(Vec(49.855f+ disp_offset_a, 86.558f+ disp_offset_b)), // white
            };

            Vec whiteNoteSize = mm2px(Vec(6.689f, 13.393f));
            Vec blackNoteSize = mm2px(Vec(4.588f, 9.499f));

            // White notes
            static const std::vector<int> whiteNotes = {0, 2, 4, 5, 7, 9, 11};
            for (int note : whiteNotes) {
                KeyboardKey* keyboardKey = new KeyboardKey();
                keyboardKey->box.pos = noteAbsPositions[note] - box.pos;
                keyboardKey->box.size = whiteNoteSize;
                keyboardKey->module = module;
                keyboardKey->note = note;
                addChild(keyboardKey);
            }

            // Black notes
            static const std::vector<int> blackNotes = {1, 3, 6, 8, 10};
            for (int note : blackNotes) {
                KeyboardKey* keyboardKey = new KeyboardKey();
                keyboardKey->box.pos = noteAbsPositions[note] - box.pos;
                keyboardKey->box.size = blackNoteSize;
                keyboardKey->module = module;
                keyboardKey->note = note;
                addChild(keyboardKey);
            }
        }
    };

    struct WeaveDisplay : TransparentWidget {
        Weave* module = nullptr;
        int index;        // Index from 0 to 5 for each envelope

        void drawLayer(const DrawArgs& args, int layer) override {

            if (layer == 1) { // Self-illuminating layer

                float columns = 7.0f;
                float buffer = 2.0f;
                float columnWidth = (box.size.x-2.0f*buffer)/columns;
                float rowHeight = (box.size.y-buffer)/6.0f;

                NVGcolor color = nvgRGB(208, 140, 89); //gold

                //Draw Circles
                nvgBeginPath(args.vg);
                for (int i=0; i<columns; i++){
                    for (int j=0; j<6; j++){
                        nvgCircle(args.vg, columnWidth*i + buffer, rowHeight*j + 0.5*buffer, 2.0f);
                    }
                }
                nvgFillColor(args.vg, color);
                nvgFill(args.vg);

                int tempPermute[6]={5,0,1,2,3,4};

                if (module){
                    for (int i=0; i<6; i++) tempPermute[i] = module->currentPermute[i];
                }

                for (int i=0; i<(columns-1); i++){

                    for (int j=0; j<6; j++){
                        nvgBeginPath(args.vg);
                        nvgMoveTo(args.vg, columnWidth*i + buffer, tempPermute[j]*rowHeight + 0.5*buffer);

                        int curWeaveSetting = 1; //default to a fav pattern
                        if (module ) curWeaveSetting = module->weaveSetting;

                        int lineDest = Weave_Chart[curWeaveSetting][ tempPermute[j] ];

                        nvgLineTo(args.vg, columnWidth*(i+1) + buffer, lineDest*rowHeight + 0.5*buffer);

                        nvgStrokeColor(args.vg, nvgRGB(208, 140, 89));
                        nvgStrokeWidth(args.vg, 0.4f*( j + 1) );
                        nvgStroke(args.vg);

                        tempPermute[j] = lineDest;

                    }
                }
            }
        }
    };

    WeaveWidget(Weave* module) {
        setModule(module);
        setPanel(createPanel(
            asset::plugin(pluginInstance, "res/Weave.svg"),
            asset::plugin(pluginInstance, "res/Weave-dark.svg")
        ));

        addChild(createWidget<ThemedScrew>(Vec(0, 0)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ThemedScrew>(Vec(0, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ThemedScrew>(Vec(box.size.x - 1 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));


        float left = -4.f;
        float leftW = -9.f;
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(45.0+leftW, 42.0f)), module, Weave::WEAVE_KNOB_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(55.0+leftW, 42.00)), module, Weave::WEAVE_ATT_PARAM));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(65.0+leftW-1.f, 42.00)), module, Weave::WEAVE_INPUT));
        addParam(createParamCentered<RoundHugeBlackKnob>(mm2px(Vec(23.299+left, 62.14)), module, Weave::CHORD_KNOB_PARAM));
        addLightsAroundKnob(module, mm2px(23.299+left), mm2px(62.14), Weave::CHORD_1_LIGHT, 17, 32.f);

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.872, 13.656)), module, Weave::TRIG_INPUT));
        addParam(createParamCentered<TL1105>(mm2px(Vec(8.872, 6.656)), module, Weave::TRIG_BUTTON));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.872, 32.024)), module, Weave::RESET_INPUT));
        addParam(createParamCentered<TL1105>(mm2px(Vec(8.872, 25.024)), module, Weave::RESET_BUTTON));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(9.193+7, 112.123)), module, Weave::NOTE_INPUT));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(23.561+7, 112.123)), module, Weave::CHORD_INPUT));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(37.95+7, 112.123)), module, Weave::SHIFT_INPUT));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(56-7, 73)), module, Weave::SHIFT_KNOB_PARAM));

        float right = 4.5f;
        float spacing = 11.0f;
        for (int out=0; out<7; out++){
            if (out==6) spacing +=.5f;
            addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(62.642+right, 16.0f+out*spacing)), module, Weave::OUTPUT_1+out));
        }

        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(62.642+right, 16.0f+7*12-1)), module, Weave::TRIG_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(62.642+right, 16.0f+8*12)), module, Weave::POLY_OUTPUT));


        //Octave Buttons
        addParam(createParamCentered<TL1105>(mm2px(Vec(9.64f+left+4.f, 85.4f  )), module, Weave::OCTAVE_DOWN_BUTTON));
        addParam(createParamCentered<TL1105>(mm2px(Vec(61.292f+left-4.f, 85.4f)), module, Weave::OCTAVE_UP_BUTTON));
        addChild(createLightCentered<LargeLight<RedLight>>(mm2px(Vec(9.64f+left+4.f, 85.f)), module, Weave::OCTAVE_DOWN_LIGHT));
        addChild(createLightCentered<LargeLight<RedLight>>(mm2px(Vec(61.292f+left-4.f, 85.f)), module, Weave::OCTAVE_UP_LIGHT));

        ////////////
        // ADD DISPLAY WIDGETS
        // Note Displays Initialization
        std::vector<std::string> baseText = {"C1", "C2", "C3", "C4", "C5", "C6"};
        for (int i = 0; i < 6; i++) {
            noteDisplays[i] = createDigitalDisplay(mm2px(Vec(15.06f-4.f, 11.084f + (float)i * 3.363f)), baseText[i], 10.f);
            addChild(noteDisplays[i]);
        }

        // Chord Display
        chordDisplay = createDigitalDisplay(mm2px(Vec(47.667f-6.f,55.419f)), "Oct", 14.f);
        addChild(chordDisplay);

        // Create and add the Weave Display
        WeaveDisplay* weaveDisplay = createWidget<WeaveDisplay>(mm2px(Vec(28.f-4.f, 13.5))); // Positioning
        weaveDisplay->box.size = Vec(115.f, 63.f); // Size of the display widget
        weaveDisplay->module = module;
        addChild(weaveDisplay);

        if (module) {
            // Quantizer display
            KeyboardDisplay* keyboardDisplay = createWidget<KeyboardDisplay>(mm2px(Vec(10.7f-5.f, 87.5f)));
            keyboardDisplay->box.size = mm2px(Vec(50.501f, 16.168f));
            keyboardDisplay->setModule(module);
            addChild(keyboardDisplay);
        }
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);

        Weave* module = dynamic_cast<Weave*>(this->module);
        if (!module)
            return;

        menu->addChild(new MenuSeparator());

        // --- Existing Quantize Shift Toggle ---
        struct QuantizeShiftItem : MenuItem {
            Weave* module;
            void onAction(const event::Action& e) override {
                module->quantizeShift = !module->quantizeShift;
            }
            void step() override {
                rightText = module->quantizeShift ? "✔" : "";
                MenuItem::step();
            }
        };
        auto* quantItem = new QuantizeShiftItem();
        quantItem->text = "Quantize Shift to semitones";
        quantItem->module = module;
        menu->addChild(quantItem);

        // --- NEW: Input Octave Tracking Toggle ---
        struct InputOctaveTrackingItem : MenuItem {
            Weave* module;
            void onAction(const event::Action& e) override {
                module->inputTracksOctaves = !module->inputTracksOctaves;
            }
            void step() override {
                rightText = module->inputTracksOctaves ? "✔" : "";
                MenuItem::step();
            }
        };
        auto* octaveItem = new InputOctaveTrackingItem();
        octaveItem->text = "Allow input to track multiple octaves";
        octaveItem->module = module;
        menu->addChild(octaveItem);
    }



    void addLightsAroundKnob(Module* module, float knobX, float knobY, int firstLightId, int numLights, float radius) {
        const float startAngle = M_PI*0.7f; // Start angle in radians (8 o'clock on the clock face)
        const float endAngle = 2.0f*M_PI+M_PI*0.3f;   // End angle in radians (4 o'clock on the clock face)

        for (int i = 0; i < numLights; i++) {
            float fraction = (float)i / (numLights - 1); // Fraction that goes from 0 to 1
            float angle = startAngle + fraction * (endAngle - startAngle);
            float x = knobX + radius * cos(angle);
            float y = knobY + radius * sin(angle);
            addChild(createLightCentered<SmallLight<RedLight>>(Vec(x, y), module, firstLightId + i));
        }
    }

    void step() override {
        Weave* module = dynamic_cast<Weave*>(this->module);
        if (!module) return;

        int rootNoteVal = 0;
        std::string rootNoteNames[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
        if (chordDisplay) {
            for (int i=0; i<17; i++){ //blank all chord lights
                module->lights[Weave::CHORD_1_LIGHT + i].setBrightness(0.0f);
            }
            if (module->chordIndex >= 0 && module->noteValue >= 0) {
                // Display the current root note and chord type
                std::string chordTypeNames[16] = {"Maj", "Min", "7", "Maj7", "Min7", "6", "Min6", "9", "Maj9", "Min9", "Add9", "Sus2", "Sus4", "5", "Aug", "Dim"};
                rootNoteVal = static_cast<int>(roundf(module->noteValue + 12*module->extOffset));
                rootNoteVal = (rootNoteVal % 12 + 12) % 12;
                std::string rootNote = rootNoteNames[rootNoteVal % 12];
                std::string chordType = chordTypeNames[module->chordIndex % 16];
                chordDisplay->text = rootNote + " " + chordType;
                module->lights[Weave::CHORD_2_LIGHT + module->chordIndex].setBrightness(1.0f);
            } else {
                // Default display if no chord or note is active
                chordDisplay->text = "Oct";
                module->lights[Weave::CHORD_1_LIGHT].setBrightness(1.0f);
            }
        }

        // Update note displays with permutation applied
        for (int i = 0; i < 6; i++) {
            if (noteDisplays[i]) {
                float pitchVoltage = module->finalNotes[i] + module->extOffset;

                // Compute note name and octave
                int octave = static_cast<int>(pitchVoltage + 4);
                double fractionalPart = fmod(pitchVoltage, 1.0);
                int semitone = std::roundf(fractionalPart * 12);
                semitone = (semitone % 12 + 12) % 12;

                // Note names
                const char* noteNames[12] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
                const char* noteName = noteNames[semitone];

                // Format full note display
                char fullNote[7];
                snprintf(fullNote, sizeof(fullNote), "%s%d", noteName, octave);

                // Safe array access with bounds checking
                int displayIndex = module->currentPermute[i];
                if (displayIndex >= 0 && displayIndex < 6 && noteDisplays[displayIndex]) {
                    noteDisplays[displayIndex]->text = fullNote;
                }
            }
        }

        // Set lights based on octave state
        if (module->octaveState == 1) {
            module->lights[Weave::OCTAVE_UP_LIGHT].setBrightness(1.0f);
            module->lights[Weave::OCTAVE_DOWN_LIGHT].setBrightness(0.0f);
        } else if (module->octaveState == -1) {
            module->lights[Weave::OCTAVE_UP_LIGHT].setBrightness(0.0f);
            module->lights[Weave::OCTAVE_DOWN_LIGHT].setBrightness(1.0f);
        } else {
            module->lights[Weave::OCTAVE_UP_LIGHT].setBrightness(0.0f);
            module->lights[Weave::OCTAVE_DOWN_LIGHT].setBrightness(0.0f);
        }
        ModuleWidget::step(); 
    }
    DigitalDisplay* createDigitalDisplay(Vec position, std::string initialValue, float fontSize) {
        DigitalDisplay* display = new DigitalDisplay();
        display->box.pos = position;
        display->box.size = Vec(50, 18);
        display->text = initialValue;
        display->fgColor = nvgRGB(208, 140, 89); // Gold color text
        display->fontPath = asset::plugin(pluginInstance, "res/fonts/DejaVuSansMono.ttf");
        display->setFontSize(fontSize);
        return display;
    }
};
Model* modelWeave = createModel<Weave, WeaveWidget>("Weave");