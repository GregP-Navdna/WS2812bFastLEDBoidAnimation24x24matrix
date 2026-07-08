#ifndef MATRIX_RAIN_EFFECT_H
#define MATRIX_RAIN_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// MatrixRainEffect: "digital rain". Each column has a falling head with a
// bright white tip and a fading green trail (the canvas fade creates the tail).
// ---------------------------------------------------------------------------
class MatrixRainEffect : public Effect {
public:
    const char* name() const override { return "Matrix Rain"; }
    uint32_t suggestedDurationMs() const override { return 18000; }

    void enter(EffectContext& ctx) override {
        ctx.canvas.clear();
        for (uint8_t x = 0; x < COLS; x++) {
            dropY[x] = -(float)random8(0, ROWS);
            speed[x] = random(20, 85) / 100.0f;
        }
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;
        const uint8_t W = c.width, H = c.height;

        c.fade(CRGB::Black, 70); // leaves the fading trails

        for (uint8_t x = 0; x < W; x++) {
            dropY[x] += speed[x];
            int y = (int)dropY[x];

            c.setPixel(x, y, CRGB(190, 255, 190));   // bright head
            c.setPixel(x, y - 1, CRGB(0, 200, 40));   // green body
            c.setPixel(x, y - 2, CRGB(0, 110, 20));

            if (dropY[x] > H + random8(0, 10)) {
                dropY[x] = -(float)random8(0, H);
                speed[x] = random(20, 85) / 100.0f;
            }
        }
    }

private:
    float dropY[COLS];
    float speed[COLS];
};

#endif // MATRIX_RAIN_EFFECT_H
