#ifndef KALEIDOSCOPE_EFFECT_H
#define KALEIDOSCOPE_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../palettes.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// KaleidoscopeEffect: a moving noise pattern is generated in one quadrant and
// mirrored 4-fold across the matrix, producing a symmetric, ever-shifting
// kaleidoscope.
// ---------------------------------------------------------------------------
class KaleidoscopeEffect : public Effect {
public:
    const char* name() const override { return "Kaleidoscope"; }
    uint32_t suggestedDurationMs() const override { return 18000; }

    void enter(EffectContext& ctx) override {
        SetNewPalette(random(0, 24));
        t = 0;
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;
        const uint8_t W = c.width, H = c.height;
        const uint8_t hw = (W + 1) / 2;
        const uint8_t hh = (H + 1) / 2;

        for (uint8_t y = 0; y < hh; y++) {
            for (uint8_t x = 0; x < hw; x++) {
                uint8_t idx = inoise8(x * 50 + t, y * 50 - t, t >> 1) + (x * y);
                uint8_t bri = qadd8(inoise8(x * 40, y * 40, t), 40);
                CRGB col = ColorFromPalette(*currentPalette_p, idx, bri, LINEARBLEND);

                // 4-fold mirror.
                c.setPixel(x, y, col);
                c.setPixel(W - 1 - x, y, col);
                c.setPixel(x, H - 1 - y, col);
                c.setPixel(W - 1 - x, H - 1 - y, col);
            }
        }
        t += 6;
    }

private:
    uint16_t t = 0;
};

#endif // KALEIDOSCOPE_EFFECT_H
