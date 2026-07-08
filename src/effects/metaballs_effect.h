#ifndef METABALLS_EFFECT_H
#define METABALLS_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../palettes.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// MetaballsEffect: gooey "lava lamp" blobs. Each pixel sums an inverse-square
// field from several drifting balls; the summed field drives palette color and
// brightness, so blobs merge and split organically.
// ---------------------------------------------------------------------------
class MetaballsEffect : public Effect {
public:
    const char* name() const override { return "Metaballs"; }
    uint32_t suggestedDurationMs() const override { return 20000; }

    void enter(EffectContext& ctx) override {
        SetNewPalette(random(0, 24));
        for (uint8_t i = 0; i < N; i++) {
            b[i].x = random8(0, COLS);
            b[i].y = random8(0, ROWS);
            b[i].vx = random(-30, 30) / 100.0f;
            b[i].vy = random(-30, 30) / 100.0f;
            b[i].r = random(30, 55) / 10.0f;
        }
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;

        for (uint8_t i = 0; i < N; i++) {
            b[i].x += b[i].vx;
            b[i].y += b[i].vy;
            if (b[i].x < 0 || b[i].x > c.width - 1)  b[i].vx = -b[i].vx;
            if (b[i].y < 0 || b[i].y > c.height - 1) b[i].vy = -b[i].vy;
        }

        for (uint8_t y = 0; y < c.height; y++) {
            for (uint8_t x = 0; x < c.width; x++) {
                float sum = 0;
                for (uint8_t i = 0; i < N; i++) {
                    float dx = x - b[i].x, dy = y - b[i].y;
                    sum += (b[i].r * b[i].r) / (dx * dx + dy * dy + 1.0f);
                }
                uint8_t bri = (uint8_t)constrain((int)(sum * 130), 0, 255);
                uint8_t idx = (uint8_t)(sum * 36) + hue;
                c.setPixel(x, y, ColorFromPalette(*currentPalette_p, idx, bri, LINEARBLEND));
            }
        }
        hue++;
    }

private:
    static const uint8_t N = 5;
    struct Ball { float x, y, vx, vy, r; };
    Ball b[N];
    uint8_t hue = 0;
};

#endif // METABALLS_EFFECT_H
