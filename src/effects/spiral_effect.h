#ifndef SPIRAL_EFFECT_H
#define SPIRAL_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../palettes.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// SpiralEffect: a rotating multi-arm galaxy. Each pixel is colored by a polar
// function of its angle and radius, animated over time to swirl outward.
// ---------------------------------------------------------------------------
class SpiralEffect : public Effect {
public:
    const char* name() const override { return "Spiral"; }
    uint32_t suggestedDurationMs() const override { return 18000; }

    void enter(EffectContext& ctx) override {
        SetNewPalette(random(0, 24));
        t = 0;
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;
        const float cx = (c.width - 1) / 2.0f;
        const float cy = (c.height - 1) / 2.0f;

        for (uint8_t y = 0; y < c.height; y++) {
            for (uint8_t x = 0; x < c.width; x++) {
                float dx = x - cx, dy = y - cy;
                float r = sqrtf(dx * dx + dy * dy);
                float a = atan2f(dy, dx);
                float v = sinf(a * ARMS + r * 0.55f - t);   // -1..1
                uint8_t idx = (uint8_t)((v * 0.5f + 0.5f) * 255) + (uint8_t)(r * 6);
                c.setPixel(x, y, ColorFromPalette(*currentPalette_p, idx, 255, LINEARBLEND));
            }
        }
        t += 0.16f;
    }

private:
    static constexpr float ARMS = 3.0f;
    float t = 0;
};

#endif // SPIRAL_EFFECT_H
