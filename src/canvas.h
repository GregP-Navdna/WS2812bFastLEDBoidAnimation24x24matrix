#ifndef CANVAS_H
#define CANVAS_H

#include <Arduino.h>
#include <FastLED.h>
#include "config.h"
#include "simd_utils.h"

// ---------------------------------------------------------------------------
// Canvas: the shared pixel-placement layer.
//
// Every effect draws through a Canvas instead of touching the raw LED buffer or
// re-implementing coordinate mapping. This keeps the serpentine layout, bounds
// checking and anti-aliasing in ONE place, so new effects get correct,
// consistent pixel handling for free.
//
// Coordinate system: physical display pixels, x in [0,width), y in [0,height),
// origin at top-left.
// ---------------------------------------------------------------------------

// Free function form of the XY mapping, suitable for APIs that take a plain
// function pointer (e.g. MatrixEffects). Mirrors Canvas::xy().
static inline uint16_t matrixXY(uint8_t x, uint8_t y) {
    if (kMatrixFlipV) y = (ROWS - 1) - y;
    if (kMatrixFlipH) x = (COLS - 1) - x;

    uint16_t i;
    if (kMatrixSerpentineLayout && (y & 0x01)) {
        // Odd rows run backwards
        uint8_t reverseX = (ROWS - 1) - x;
        i = (y * ROWS) + reverseX;
    } else {
        i = (y * ROWS) + x;
    }
    return i;
}

class Canvas {
public:
    const uint8_t width;
    const uint8_t height;

    Canvas(CRGB* buffer, uint8_t width, uint8_t height)
        : width(width), height(height), leds(buffer), ledCount(width * height) {}

    // --- Buffer access -----------------------------------------------------
    CRGB* raw() { return leds; }
    int numLeds() const { return ledCount; }

    // Map 2D coordinates to a strip index (serpentine aware).
    uint16_t xy(uint8_t x, uint8_t y) const { return matrixXY(x, y); }

    // --- Whole-buffer operations ------------------------------------------
    void clear() { fill_solid(leds, ledCount, CRGB::Black); }
    void fill(CRGB color) { fill_solid(leds, ledCount, color); }

    // Fade the whole buffer toward a color using the SIMD-optimized routine.
    void fade(CRGB color, uint8_t amount) {
        simd_fade_to_color(leds, ledCount, color, amount);
    }

    // Push the buffer to the LEDs.
    void show() { FastLED.show(); }

    // --- Single-pixel operations ------------------------------------------
    CRGB getPixel(uint8_t x, uint8_t y) const { return leds[xy(x, y)]; }

    // Hard set a pixel (bounds-checked).
    void setPixel(int x, int y, CRGB color) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        leds[xy((uint8_t)x, (uint8_t)y)] = color;
    }

    // Additive blend a pixel (bounds-checked, saturating).
    void blendPixel(int x, int y, CRGB color) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        leds[xy((uint8_t)x, (uint8_t)y)] += color;
    }

    // Sub-pixel positioned draw using Wu's anti-aliasing algorithm.
    // Spreads the color across up to four neighboring pixels (additive).
    void drawPixelF(float fx, float fy, CRGB color) {
        if (fx < 0 || fx >= width || fy < 0 || fy >= height) return;

        uint8_t xx = (fx - (int)fx) * 255, yy = (fy - (int)fy) * 255;
        uint8_t ix = 255 - xx, iy = 255 - yy;

        #define CANVAS_WU_WEIGHT(a, b) ((uint8_t)(((a) * (b) + (a) + (b)) >> 8))
        uint8_t wu[4] = {CANVAS_WU_WEIGHT(ix, iy), CANVAS_WU_WEIGHT(xx, iy),
                         CANVAS_WU_WEIGHT(ix, yy), CANVAS_WU_WEIGHT(xx, yy)};
        #undef CANVAS_WU_WEIGHT

        for (uint8_t i = 0; i < 4; i++) {
            int16_t xn = fx + (i & 1), yn = fy + ((i >> 1) & 1);
            if (xn >= 0 && xn < width && yn >= 0 && yn < height) {
                CRGB clr = getPixel(xn, yn);
                clr.r = qadd8(clr.r, (color.r * wu[i]) >> 8);
                clr.g = qadd8(clr.g, (color.g * wu[i]) >> 8);
                clr.b = qadd8(clr.b, (color.b * wu[i]) >> 8);
                setPixel(xn, yn, clr);
            }
        }
    }

private:
    CRGB* leds;
    int ledCount;
};

#endif // CANVAS_H
