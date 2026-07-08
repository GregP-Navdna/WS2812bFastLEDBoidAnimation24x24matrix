#ifndef PLASMA_EFFECT_H
#define PLASMA_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../canvas.h"
#include "../palettes.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// PlasmaEffect: a minimal reference effect.
//
// It exists to demonstrate how little code a new effect needs: implement
// name() + update(), draw through ctx.canvas, and (optionally) report a
// suggestedDurationMs() so the EffectManager can rotate to the next effect.
//
// To enable it, register an instance in main.cpp:
//     PlasmaEffect plasmaEffect;
//     manager.add(&plasmaEffect);
// ---------------------------------------------------------------------------
class PlasmaEffect : public Effect {
public:
    const char* name() const override { return "Plasma"; }

    void enter(EffectContext& ctx) override {
        ctx.canvas.clear();
        t = 0;
    }

    void update(EffectContext& ctx, uint32_t dtMs) override {
        Canvas& c = ctx.canvas;
        t += 2; // animation phase advance

        for (uint8_t y = 0; y < c.height; y++) {
            for (uint8_t x = 0; x < c.width; x++) {
                // Classic additive sine plasma.
                uint8_t v = sin8(x * 12 + t)
                          + sin8(y * 16 - t)
                          + sin8((x + y) * 8 + t / 2);
                CRGB color = ColorFromPalette(*currentPalette_p, v, 255, LINEARBLEND);
                c.setPixel(x, y, color);
            }
        }
    }

    // Auto-rotate away after 20s when multiple effects are registered.
    uint32_t suggestedDurationMs() const override { return 20000; }

private:
    uint8_t t = 0;
};

#endif // PLASMA_EFFECT_H
