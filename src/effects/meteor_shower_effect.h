#ifndef METEOR_SHOWER_EFFECT_H
#define METEOR_SHOWER_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// MeteorShowerEffect: glowing meteors streak diagonally across the matrix,
// leaving fading tails. Each meteor respawns from the top edge when it exits.
// ---------------------------------------------------------------------------
class MeteorShowerEffect : public Effect {
public:
    const char* name() const override { return "Meteor Shower"; }
    uint32_t suggestedDurationMs() const override { return 18000; }

    void enter(EffectContext& ctx) override {
        ctx.canvas.clear();
        for (uint8_t i = 0; i < N; i++) respawn(m[i], true);
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;
        c.fade(CRGB::Black, 48); // tails

        for (uint8_t i = 0; i < N; i++) {
            Meteor& q = m[i];
            q.x += q.vx;
            q.y += q.vy;

            // Bright head + a small leading glow.
            c.drawPixelF(q.x, q.y, CHSV(q.hue, 200, 255));
            c.drawPixelF(q.x - q.vx * 0.5f, q.y - q.vy * 0.5f, CHSV(q.hue, 220, 120));

            if (q.x < -2 || q.x > c.width + 2 || q.y > c.height + 2) {
                respawn(q, false);
            }
        }
    }

private:
    static const uint8_t N = 7;
    struct Meteor { float x, y, vx, vy; uint8_t hue; };
    Meteor m[N];

    void respawn(Meteor& q, bool scatter) {
        q.x = random8(0, COLS);
        q.y = scatter ? random8(0, ROWS) : -2.0f;
        q.vx = random(-30, 60) / 100.0f;
        q.vy = random(40, 95) / 100.0f;
        q.hue = random8();
    }
};

#endif // METEOR_SHOWER_EFFECT_H
