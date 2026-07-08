#ifndef STARFIELD_EFFECT_H
#define STARFIELD_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../palettes.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// StarfieldEffect: a 3D "warp speed" starfield. Stars fly toward the viewer
// (z -> 0), accelerating outward from the center, then respawn at the far
// plane. Canvas fade adds motion-blur streaks.
// ---------------------------------------------------------------------------
class StarfieldEffect : public Effect {
public:
    const char* name() const override { return "Starfield"; }
    uint32_t suggestedDurationMs() const override { return 18000; }

    void enter(EffectContext& ctx) override {
        SetNewPalette(random(0, 24));
        ctx.canvas.clear();
        for (uint8_t i = 0; i < N; i++) reset(stars[i]);
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;
        const float cx = (c.width - 1) / 2.0f;
        const float cy = (c.height - 1) / 2.0f;

        c.fade(CRGB::Black, 90);

        for (uint8_t i = 0; i < N; i++) {
            Star& s = stars[i];
            s.z -= 0.012f;
            if (s.z < 0.02f) { reset(s); continue; }

            float sx = (s.x / s.z) * (c.width * 0.5f) + cx;
            float sy = (s.y / s.z) * (c.height * 0.5f) + cy;
            if (sx < 0 || sx >= c.width || sy < 0 || sy >= c.height) { reset(s); continue; }

            uint8_t bri = (uint8_t)((1.0f - s.z) * 255);
            c.drawPixelF(sx, sy, ColorFromPalette(*currentPalette_p, s.hue, bri, LINEARBLEND));
        }
    }

private:
    static const uint8_t N = 80;
    struct Star { float x, y, z; uint8_t hue; };
    Star stars[N];

    void reset(Star& s) {
        s.x = (random(-1000, 1000)) / 1000.0f;
        s.y = (random(-1000, 1000)) / 1000.0f;
        s.z = 1.0f;
        s.hue = random8();
    }
};

#endif // STARFIELD_EFFECT_H
