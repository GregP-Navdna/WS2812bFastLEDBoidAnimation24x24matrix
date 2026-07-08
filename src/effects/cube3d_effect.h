#ifndef CUBE3D_EFFECT_H
#define CUBE3D_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// Cube3dEffect: a rotating 3D wireframe cube. Vertices are rotated around the
// X and Y axes, perspective-projected to the matrix, and the 12 edges are
// drawn as anti-aliased lines with a hue that cycles over time.
// ---------------------------------------------------------------------------
class Cube3dEffect : public Effect {
public:
    const char* name() const override { return "3D Cube"; }
    uint32_t suggestedDurationMs() const override { return 18000; }

    void enter(EffectContext& ctx) override {
        ctx.canvas.clear();
        ax = ay = 0;
        hue = 0;
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;
        c.fade(CRGB::Black, 90); // subtle motion trails

        float sxv[8], syv[8];
        float ca = cosf(ax), sa = sinf(ax);
        float cb = cosf(ay), sb = sinf(ay);
        const float cx = (c.width - 1) / 2.0f;
        const float cy = (c.height - 1) / 2.0f;

        for (uint8_t i = 0; i < 8; i++) {
            float x = V[i][0], y = V[i][1], z = V[i][2];
            // Rotate around X then Y.
            float y1 = y * ca - z * sa;
            float z1 = y * sa + z * ca;
            float x2 = x * cb + z1 * sb;
            float z2 = -x * sb + z1 * cb;
            float persp = 2.6f / (z2 + 3.2f);
            sxv[i] = cx + x2 * persp * (c.width * 0.42f);
            syv[i] = cy + y1 * persp * (c.height * 0.42f);
        }

        CRGB col = CHSV(hue, 220, 255);
        for (uint8_t e = 0; e < 12; e++) {
            line(c, sxv[E[e][0]], syv[E[e][0]], sxv[E[e][1]], syv[E[e][1]], col);
        }

        ax += 0.045f;
        ay += 0.062f;
        hue++;
    }

private:
    float ax = 0, ay = 0;
    uint8_t hue = 0;

    static constexpr float V[8][3] = {
        {-1,-1,-1}, {1,-1,-1}, {1,1,-1}, {-1,1,-1},
        {-1,-1, 1}, {1,-1, 1}, {1,1, 1}, {-1,1, 1}};
    static constexpr uint8_t E[12][2] = {
        {0,1},{1,2},{2,3},{3,0}, {4,5},{5,6},{6,7},{7,4}, {0,4},{1,5},{2,6},{3,7}};

    void line(Canvas& c, float x0, float y0, float x1, float y1, CRGB col) {
        float dx = x1 - x0, dy = y1 - y0;
        int steps = (int)(max(fabsf(dx), fabsf(dy)) * 2) + 1;
        for (int i = 0; i <= steps; i++) {
            float t = (float)i / steps;
            c.drawPixelF(x0 + dx * t, y0 + dy * t, col);
        }
    }
};

// Out-of-class definitions for the static constexpr arrays (C++14 linkage).
constexpr float Cube3dEffect::V[8][3];
constexpr uint8_t Cube3dEffect::E[12][2];

#endif // CUBE3D_EFFECT_H
