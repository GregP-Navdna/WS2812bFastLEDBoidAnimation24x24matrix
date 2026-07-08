#ifndef FIREWORKS_EFFECT_H
#define FIREWORKS_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../palettes.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// FireworksEffect: rockets launch from the bottom, arc upward under gravity,
// then burst into a shower of colored sparks. Canvas fade gives glowing trails.
// ---------------------------------------------------------------------------
class FireworksEffect : public Effect {
public:
    const char* name() const override { return "Fireworks"; }
    uint32_t suggestedDurationMs() const override { return 22000; }

    void enter(EffectContext& ctx) override {
        ctx.canvas.clear();
        for (uint8_t i = 0; i < MAXP; i++) p[i].active = false;
        lastLaunch = millis();
        launchInterval = 600;
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;
        c.fade(CRGB::Black, 55);

        if (millis() - lastLaunch > launchInterval) {
            launch(c);
            lastLaunch = millis();
            launchInterval = random(500, 1600);
        }

        for (uint8_t i = 0; i < MAXP; i++) {
            Particle& q = p[i];
            if (!q.active) continue;

            q.vy += 0.035f; // gravity (down = +y)
            q.x += q.vx;
            q.y += q.vy;

            if (q.rocket) {
                if (q.vy >= -0.05f) { explode(q); continue; }
            } else {
                if (q.life == 0) { q.active = false; continue; }
                q.life--;
            }

            if (q.x < -1 || q.x > c.width + 1 || q.y > c.height + 1) {
                q.active = false;
                continue;
            }

            uint8_t bri = q.rocket ? 255 : (uint8_t)map(q.life, 0, 40, 0, 255);
            CRGB col = q.rocket ? CRGB(255, 240, 180) : (CRGB)CHSV(q.hue, 230, bri);
            c.drawPixelF(q.x, q.y, col);
        }
    }

private:
    static const uint8_t MAXP = 110;
    struct Particle { float x, y, vx, vy; uint8_t life, hue; bool rocket, active; };
    Particle p[MAXP];
    uint32_t lastLaunch = 0;
    uint32_t launchInterval = 600;

    int freeSlot() {
        for (uint8_t i = 0; i < MAXP; i++) if (!p[i].active) return i;
        return -1;
    }

    void launch(Canvas& c) {
        int s = freeSlot();
        if (s < 0) return;
        Particle& q = p[s];
        q.x = random8(4, c.width - 4);
        q.y = c.height - 1;
        q.vx = random(-25, 25) / 100.0f;
        q.vy = -(random(70, 100) / 100.0f);
        q.rocket = true;
        q.active = true;
        q.hue = random8();
        q.life = 120;
    }

    void explode(Particle& rocket) {
        uint8_t baseHue = rocket.hue;
        float ex = rocket.x, ey = rocket.y;
        rocket.active = false;

        uint8_t sparks = random8(18, 30);
        for (uint8_t n = 0; n < sparks; n++) {
            int s = freeSlot();
            if (s < 0) break;
            float ang = (TWO_PI * n) / sparks + (random8() / 255.0f);
            float spd = random(15, 70) / 100.0f;
            Particle& q = p[s];
            q.x = ex; q.y = ey;
            q.vx = cosf(ang) * spd;
            q.vy = sinf(ang) * spd;
            q.rocket = false;
            q.active = true;
            q.hue = baseHue + random8(0, 32);
            q.life = random8(22, 42);
        }
    }
};

#endif // FIREWORKS_EFFECT_H
