#ifndef NOISE_FIELD_EFFECT_H
#define NOISE_FIELD_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../palettes.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// NoiseFieldEffect: smoothly flowing Perlin-noise clouds. A drifting sample
// origin makes the field translate while it also evolves over time, giving
// organic, ever-shifting color fog mapped to the active palette.
// ---------------------------------------------------------------------------
class NoiseFieldEffect : public Effect {
public:
    const char* name() const override { return "Noise Field"; }
    uint32_t suggestedDurationMs() const override { return 18000; }

    void enter(EffectContext& ctx) override {
        SetNewPalette(random(0, 24));
        z = random16();
        ox = 0;
        oy = 0;
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;
        for (uint8_t y = 0; y < c.height; y++) {
            for (uint8_t x = 0; x < c.width; x++) {
                uint8_t idx = inoise8(x * 24 + ox, y * 24 + oy, z);
                uint8_t bri = scale8(inoise8(x * 24 + ox + 5000, y * 24 + oy, z), 200) + 55;
                c.setPixel(x, y, ColorFromPalette(*currentPalette_p, idx, bri, LINEARBLEND));
            }
        }
        z += 12;
        ox += 6;   // horizontal drift
        oy += 3;   // vertical drift
    }

private:
    uint16_t z = 0;
    uint16_t ox = 0, oy = 0;
};

#endif // NOISE_FIELD_EFFECT_H
