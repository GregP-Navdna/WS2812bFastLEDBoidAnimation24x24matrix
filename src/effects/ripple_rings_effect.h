#ifndef RIPPLE_RINGS_EFFECT_H
#define RIPPLE_RINGS_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../palettes.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// RippleRingsEffect: expanding concentric rings spawn at random points and
// grow outward, fading as they widen. Multiple ripples overlap for a lively,
// water-like surface.
// ---------------------------------------------------------------------------
class RippleRingsEffect : public Effect {
public:
    const char* name() const override { return "Ripple Rings"; }
    uint32_t suggestedDurationMs() const override { return 18000; }

    void enter(EffectContext& ctx) override {
        SetNewPalette(random(0, 24));
        ctx.canvas.clear();
        for (uint8_t i = 0; i < MAXR; i++) r[i].active = false;
        lastSpawn = millis();
        spawnInterval = 400;
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;
        const float maxR = c.width * 0.95f;

        c.fade(CRGB::Black, 45);

        if (millis() - lastSpawn > spawnInterval) {
            spawn(c);
            lastSpawn = millis();
            spawnInterval = random(250, 900);
        }

        for (uint8_t i = 0; i < MAXR; i++) {
            Ripple& q = r[i];
            if (!q.active) continue;
            q.radius += 0.55f;
            if (q.radius > maxR) { q.active = false; continue; }

            uint8_t bri = (uint8_t)(255 * (1.0f - q.radius / maxR));
            for (int a = 0; a < 360; a += 6) {
                float rad = a * DEG_TO_RAD;
                float px = q.cx + cosf(rad) * q.radius;
                float py = q.cy + sinf(rad) * q.radius;
                CRGB col = ColorFromPalette(*currentPalette_p,
                                            q.hue + (uint8_t)(q.radius * 4), bri, LINEARBLEND);
                c.drawPixelF(px, py, col);
            }
        }
    }

private:
    static const uint8_t MAXR = 10;
    struct Ripple { float cx, cy, radius; uint8_t hue; bool active; };
    Ripple r[MAXR];
    uint32_t lastSpawn = 0;
    uint32_t spawnInterval = 400;

    void spawn(Canvas& c) {
        for (uint8_t i = 0; i < MAXR; i++) {
            if (!r[i].active) {
                r[i].cx = random8(2, c.width - 2);
                r[i].cy = random8(2, c.height - 2);
                r[i].radius = 0;
                r[i].hue = random8();
                r[i].active = true;
                return;
            }
        }
    }
};

#endif // RIPPLE_RINGS_EFFECT_H
