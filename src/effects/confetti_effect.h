#ifndef CONFETTI_EFFECT_H
#define CONFETTI_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../palettes.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// ConfettiEffect: random colored speckles pop into existence and fade out, with
// the spawn hue slowly drifting so the palette of confetti evolves over time.
// ---------------------------------------------------------------------------
class ConfettiEffect : public Effect {
public:
    const char* name() const override { return "Confetti"; }
    uint32_t suggestedDurationMs() const override { return 15000; }

    void enter(EffectContext& ctx) override {
        SetNewPalette(random(0, 24));
        ctx.canvas.clear();
        hue = 0;
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;
        c.fade(CRGB::Black, 28); // slow fade-out of existing speckles

        uint8_t pops = random8(2, 5);
        for (uint8_t i = 0; i < pops; i++) {
            uint8_t x = random8(0, c.width);
            uint8_t y = random8(0, c.height);
            CRGB col = ColorFromPalette(*currentPalette_p, hue + random8(0, 48), 255, LINEARBLEND);
            c.blendPixel(x, y, col);
        }
        hue++;
    }

private:
    uint8_t hue = 0;
};

#endif // CONFETTI_EFFECT_H
