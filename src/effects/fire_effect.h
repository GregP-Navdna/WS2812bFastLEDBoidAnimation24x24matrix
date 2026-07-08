#ifndef FIRE_EFFECT_H
#define FIRE_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// FireEffect: a Fire2012-style flame simulation. Heat is seeded at the bottom
// row, cools as it drifts upward, and is rendered with FastLED's HeatColor.
// ---------------------------------------------------------------------------
class FireEffect : public Effect {
public:
    const char* name() const override { return "Fire"; }
    uint32_t suggestedDurationMs() const override { return 20000; }

    void enter(EffectContext& ctx) override {
        memset(heat, 0, sizeof(heat));
        ctx.canvas.clear();
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;
        const uint8_t W = c.width, H = c.height;

        for (uint8_t x = 0; x < W; x++) {
            uint8_t* col = &heat[x * H];

            // 1. Cool every cell a little.
            for (uint8_t h = 0; h < H; h++) {
                col[h] = qsub8(col[h], random8(0, ((COOLING * 10) / H) + 2));
            }

            // 2. Heat drifts up and diffuses (h indexed from the bottom).
            for (uint8_t h = H - 1; h >= 2; h--) {
                col[h] = (col[h - 1] + col[h - 2] + col[h - 2]) / 3;
            }

            // 3. Randomly ignite new sparks near the base.
            if (random8() < SPARKING) {
                uint8_t hh = random8(min((uint8_t)3, H));
                col[hh] = qadd8(col[hh], random8(160, 255));
            }

            // 4. Render (bottom of the matrix is the hot base).
            for (uint8_t h = 0; h < H; h++) {
                c.setPixel(x, H - 1 - h, HeatColor(col[h]));
            }
        }
    }

private:
    static const uint8_t COOLING = 55;
    static const uint8_t SPARKING = 120;
    uint8_t heat[NUM_LEDS];
};

#endif // FIRE_EFFECT_H
