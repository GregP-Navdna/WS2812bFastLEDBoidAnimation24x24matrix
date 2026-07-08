#ifndef BOUNCING_BALLS_EFFECT_H
#define BOUNCING_BALLS_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// BouncingBallsEffect: gravity-driven balls bounce off the walls and floor with
// energy loss, leaving glowing trails. Balls that lose too much energy get a
// fresh upward kick so the scene never goes static.
// ---------------------------------------------------------------------------
class BouncingBallsEffect : public Effect {
public:
    const char* name() const override { return "Bouncing Balls"; }
    uint32_t suggestedDurationMs() const override { return 18000; }

    void enter(EffectContext& ctx) override {
        ctx.canvas.clear();
        for (uint8_t i = 0; i < N; i++) kick(ball[i], i);
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;
        c.fade(CRGB::Black, 60); // trails

        for (uint8_t i = 0; i < N; i++) {
            Ball& q = ball[i];
            q.vy += 0.05f; // gravity
            q.x += q.vx;
            q.y += q.vy;

            if (q.x < 0)            { q.x = 0;            q.vx = -q.vx * 0.9f; }
            if (q.x > c.width - 1)  { q.x = c.width - 1;  q.vx = -q.vx * 0.9f; }
            if (q.y > c.height - 1) { q.y = c.height - 1; q.vy = -q.vy * 0.82f; }
            if (q.y < 0)            { q.y = 0;            q.vy = -q.vy * 0.9f; }

            // Re-energize a nearly-settled ball.
            if (fabsf(q.vy) < 0.25f && q.y > c.height - 2) kick(q, i);

            c.drawPixelF(q.x, q.y, CHSV(q.hue, 230, 255));
        }
    }

private:
    static const uint8_t N = 6;
    struct Ball { float x, y, vx, vy; uint8_t hue; };
    Ball ball[N];

    void kick(Ball& q, uint8_t i) {
        q.x = random8(0, COLS);
        q.y = random8(0, ROWS / 2);
        q.vx = random(-50, 50) / 100.0f;
        q.vy = -(random(40, 110) / 100.0f);
        q.hue = i * (256 / N) + random8(0, 24);
    }
};

#endif // BOUNCING_BALLS_EFFECT_H
