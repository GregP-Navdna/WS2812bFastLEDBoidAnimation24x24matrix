#ifndef DNA_HELIX_EFFECT_H
#define DNA_HELIX_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// DnaHelixEffect: two intertwined sine strands scroll vertically with "rungs"
// connecting them, like a rotating DNA double helix. Strand brightness tracks
// depth (front strand brighter) for a 3D feel.
// ---------------------------------------------------------------------------
class DnaHelixEffect : public Effect {
public:
    const char* name() const override { return "DNA Helix"; }
    uint32_t suggestedDurationMs() const override { return 18000; }

    void enter(EffectContext& ctx) override {
        ctx.canvas.clear();
        t = 0;
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;
        c.clear();
        const float cx = (c.width - 1) / 2.0f;
        const float amp = c.width * 0.38f;

        for (uint8_t y = 0; y < c.height; y++) {
            float phase = y * 0.5f + t;
            float s = sinf(phase);
            float x1 = cx + amp * s;
            float x2 = cx - amp * s;
            uint8_t hue = y * 8 + (uint8_t)(t * 12);

            // Depth: cos gives which strand is "in front".
            float depth = cosf(phase);
            uint8_t b1 = (uint8_t)(150 + depth * 105);
            uint8_t b2 = (uint8_t)(150 - depth * 105);

            // Rungs every few rows.
            if ((y % 3) == 0) {
                int xa = (int)x1, xb = (int)x2;
                if (xa > xb) { int tmp = xa; xa = xb; xb = tmp; }
                for (int xr = xa; xr <= xb; xr++) c.setPixel(xr, y, CHSV(hue, 180, 60));
            }

            c.drawPixelF(x1, y, CHSV(hue, 255, b1));
            c.drawPixelF(x2, y, CHSV(hue + 128, 255, b2));
        }
        t += 0.12f;
    }

private:
    float t = 0;
};

#endif // DNA_HELIX_EFFECT_H
