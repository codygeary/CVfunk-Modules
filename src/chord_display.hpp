#pragma once
#include <rack.hpp>

using namespace rack;

struct ChordDiagram : TransparentWidget {
    std::array<int, 6> fingering;
    const float xOffset = 10.0; // X offset from the left side of the widget
    const float yOffset = 10.0; // Y offset from the top side of the widget
    const float xSpacing = 6.0; // Spacing between strings
    const float ySpacing = 6.0; // Spacing between fret positions
    const float radius = 2.0; // Radius of the dots

    ChordDiagram() {
        // Initialize fingering to open strings by default
        fingering.fill(0);
    }

    void setFingering(const std::array<int, 6>& newFingering) {
        fingering = newFingering;
    }

void drawLayer(const DrawArgs& args, int layer) override {
    if (layer == 1) {
        bool shiftNeeded = std::any_of(fingering.begin(), fingering.end(), [](int f) { return f > 4; });

        int minFret = std::numeric_limits<int>::max();
        // Find minimum fretted note greater than 0 if shift is needed
        if (shiftNeeded) {
            for (int f : fingering) {
                if (f > 0 && f < minFret) {
                    minFret = f;
                }
            }
        }

        int shiftValue = 0; // Initialize shift value (fret number to display)
        for (size_t i = 0; i < 6; ++i) {
            int fretPosition = fingering[i];

            // Draw "X" for muted strings
            if (fingering[i] == -1) {
                Vec pos = Vec(xOffset + i * xSpacing, yOffset);
                nvgBeginPath(args.vg);
                nvgMoveTo(args.vg, pos.x - radius, pos.y - radius);
                nvgLineTo(args.vg, pos.x + radius, pos.y + radius);
                nvgMoveTo(args.vg, pos.x + radius, pos.y - radius);
                nvgLineTo(args.vg, pos.x - radius, pos.y + radius);
                nvgStrokeColor(args.vg, nvgRGB(255, 255, 255)); // White X
                nvgStrokeWidth(args.vg, 2.0); // Bold X
                nvgStroke(args.vg);
            } else if (fingering[i] >= 0) { // Process non-muted strings
                // If shift is needed and fingering is not an open string, adjust by minFret - 1
                if (shiftNeeded && fingering[i] > 0) {
                    fretPosition = fingering[i] - (minFret - 1);
                    shiftValue = minFret; // Set the shift value to display
                }

                Vec pos = Vec(xOffset + i * xSpacing, yOffset + fretPosition * ySpacing);
                nvgBeginPath(args.vg);
                nvgCircle(args.vg, pos.x, pos.y, radius);
                nvgFillColor(args.vg, nvgRGB(255, 255, 255)); // White dots for fretted notes
                nvgFill(args.vg);
            }
        }

        // Display the fret number shift if needed
        if (shiftValue > 0) {
            Vec textPos = Vec(xOffset + 6 * xSpacing , yOffset + ySpacing); // Position to the right of the first fret
            nvgFontSize(args.vg, 10); // Smaller font for the fret number
            nvgFontFaceId(args.vg, APP->window->uiFont->handle);
            nvgTextAlign(args.vg, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
            nvgFillColor(args.vg, nvgRGB(255, 255, 255)); // White text
            nvgText(args.vg, textPos.x, textPos.y, std::to_string(shiftValue).c_str(), NULL);
        }
    }
}
};