#pragma once
#include <rack.hpp>

using namespace rack;

struct DigitalDisplay : Widget {
    std::string fontPath;
    std::string bgText;
    std::string text;
    float fontSize;
    NVGcolor bgColor = nvgRGB(0x46,0x46, 0x46);
    NVGcolor fgColor = SCHEME_YELLOW;
    Vec textPos;
    int textAlign = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE; // Default text alignment

    // Setter for text alignment
    void setTextAlign(int align) {
        textAlign = align;
    }

    void setFontSize(float size) {
        fontSize = size;
    }

    void prepareFont(const DrawArgs& args) {
        // Get font
        std::shared_ptr<Font> font = APP->window->loadFont(fontPath);
        if (!font)
            return;
        nvgFontFaceId(args.vg, font->handle);
        nvgFontSize(args.vg, fontSize);
        nvgTextLetterSpacing(args.vg, 0.0);
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE); // Centers horizontally and vertically
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) return;

        prepareFont(args);

        // Use these default values if specific values are not provided
        NVGcolor currentColor = fgColor;
        int currentAlign = textAlign;
    
        // Calculate center if alignment is not overridden
        Vec pos = Vec(box.size.x / 2.0, box.size.y / 2.0);
        if (currentAlign == (NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE)) {
            pos.x = 0; // Start text from the left if left aligned
        }

        nvgFillColor(args.vg, currentColor);
        nvgTextAlign(args.vg, currentAlign);
        nvgText(args.vg, pos.x, pos.y, text.c_str(), NULL);
    }

};