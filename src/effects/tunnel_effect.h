#ifndef TUNNEL_EFFECT_H
#define TUNNEL_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../palettes.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// TunnelEffect: a classic demoscene "infinite tunnel". Each pixel's polar
// angle and inverse distance form texture coordinates that scroll over time,
// creating the illusion of flying down a ringed, rotating tunnel.
// ---------------------------------------------------------------------------
class TunnelEffect : public Effect {
public:
    const char* name() const override { return "Tunnel"; }
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
                float dist = sqrtf(dx * dx + dy * dy) + 0.5f;
                float angle = atan2f(dy, dx);

                uint8_t depth = (uint8_t)(60.0f / dist);          // rings
                uint8_t ang = (uint8_t)(angle * 40.7f);           // twist
                uint8_t idx = depth + ang + t;
                uint8_t bri = (uint8_t)constrain((int)(dist * 24), 30, 255);

                c.setPixel(x, y, ColorFromPalette(*currentPalette_p, idx, bri, LINEARBLEND));
            }
        }
        t += 3;
    }

private:
    uint8_t t = 0;
};

#endif // TUNNEL_EFFECT_H
