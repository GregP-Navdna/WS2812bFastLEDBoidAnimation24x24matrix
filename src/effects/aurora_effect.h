#ifndef AURORA_EFFECT_H
#define AURORA_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// AuroraEffect: northern-lights curtains. Two layers of Perlin noise drive
// color and brightness from a custom green/purple palette, scrolling upward to
// mimic shimmering aurora bands.
// ---------------------------------------------------------------------------
class AuroraEffect : public Effect {
public:
    const char* name() const override { return "Aurora"; }
    uint32_t suggestedDurationMs() const override { return 20000; }

    void enter(EffectContext& ctx) override {
        pal = CRGBPalette16(
            CRGB::Black,        CRGB(0, 90, 40),    CRGB(0, 180, 70),   CRGB::Black,
            CRGB(0, 140, 90),   CRGB(0, 255, 120),  CRGB(0, 200, 160),  CRGB::Black,
            CRGB(60, 0, 140),   CRGB(140, 0, 200),  CRGB(90, 30, 180),  CRGB::Black,
            CRGB(0, 160, 90),   CRGB(0, 255, 140),  CRGB(120, 0, 200),  CRGB::Black);
        t = 0;
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;
        for (uint8_t y = 0; y < c.height; y++) {
            for (uint8_t x = 0; x < c.width; x++) {
                uint8_t bri = inoise8(x * 38, y * 42 - t);
                uint8_t idx = inoise8(x * 30 + 1000, y * 22 - (t >> 1));
                // Fade the bottom rows so curtains hang from the top.
                uint8_t vfade = scale8(bri, 120 + (y * 135 / c.height));
                c.setPixel(x, y, ColorFromPalette(pal, idx, vfade, LINEARBLEND));
            }
        }
        t += 9;
    }

private:
    CRGBPalette16 pal;
    uint16_t t = 0;
};

#endif // AURORA_EFFECT_H
