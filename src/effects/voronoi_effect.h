#ifndef VORONOI_EFFECT_H
#define VORONOI_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../palettes.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// VoronoiEffect: drifting Voronoi cells. Each pixel takes the color of its
// nearest moving site; pixels near a cell boundary darken, producing crisp,
// shifting "stained glass" regions.
// ---------------------------------------------------------------------------
class VoronoiEffect : public Effect {
public:
    const char* name() const override { return "Voronoi"; }
    uint32_t suggestedDurationMs() const override { return 18000; }

    void enter(EffectContext& ctx) override {
        SetNewPalette(random(0, 24));
        for (uint8_t i = 0; i < N; i++) {
            s[i].x = random8(0, COLS);
            s[i].y = random8(0, ROWS);
            s[i].vx = random(-40, 40) / 100.0f;
            s[i].vy = random(-40, 40) / 100.0f;
            s[i].hue = (uint8_t)(i * (256 / N));
        }
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;

        // Move sites; bounce off the edges.
        for (uint8_t i = 0; i < N; i++) {
            s[i].x += s[i].vx;
            s[i].y += s[i].vy;
            if (s[i].x < 0 || s[i].x > c.width - 1)  s[i].vx = -s[i].vx;
            if (s[i].y < 0 || s[i].y > c.height - 1) s[i].vy = -s[i].vy;
        }

        for (uint8_t y = 0; y < c.height; y++) {
            for (uint8_t x = 0; x < c.width; x++) {
                float d1 = 1e9f, d2 = 1e9f;
                uint8_t n1 = 0;
                for (uint8_t i = 0; i < N; i++) {
                    float dx = x - s[i].x, dy = y - s[i].y;
                    float d = dx * dx + dy * dy;
                    if (d < d1) { d2 = d1; d1 = d; n1 = i; }
                    else if (d < d2) { d2 = d; }
                }
                float edge = sqrtf(d2) - sqrtf(d1);
                uint8_t bri = edge < 1.4f ? (uint8_t)(edge / 1.4f * 255) : 255;
                c.setPixel(x, y, ColorFromPalette(*currentPalette_p, s[n1].hue, bri, LINEARBLEND));
            }
        }
    }

private:
    static const uint8_t N = 6;
    struct Site { float x, y, vx, vy; uint8_t hue; };
    Site s[N];
};

#endif // VORONOI_EFFECT_H
