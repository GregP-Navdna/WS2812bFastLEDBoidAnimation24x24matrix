#ifndef WAVE_INTERFERENCE_EFFECT_H
#define WAVE_INTERFERENCE_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../palettes.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// WaveInterferenceEffect: a "ripple tank". Three moving point sources emit
// concentric waves; each pixel sums the waves, producing shifting interference
// fringes mapped to the active palette.
// ---------------------------------------------------------------------------
class WaveInterferenceEffect : public Effect {
public:
    const char* name() const override { return "Wave Interference"; }
    uint32_t suggestedDurationMs() const override { return 18000; }

    void enter(EffectContext& ctx) override {
        SetNewPalette(random(0, 24));
        t = 0;
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;
        const float cx = (c.width - 1) / 2.0f;
        const float cy = (c.height - 1) / 2.0f;
        const float radius = c.width * 0.32f;

        float sx[SRC], sy[SRC];
        for (uint8_t i = 0; i < SRC; i++) {
            float ph = t * (0.5f + i * 0.27f) + i * TWO_PI / SRC;
            sx[i] = cx + cosf(ph) * radius;
            sy[i] = cy + sinf(ph * 1.3f) * radius;
        }

        for (uint8_t y = 0; y < c.height; y++) {
            for (uint8_t x = 0; x < c.width; x++) {
                float sum = 0;
                for (uint8_t i = 0; i < SRC; i++) {
                    float dx = x - sx[i], dy = y - sy[i];
                    float d = sqrtf(dx * dx + dy * dy);
                    sum += sinf(d * 0.85f - t * 3.0f);
                }
                uint8_t idx = (uint8_t)((sum / SRC * 0.5f + 0.5f) * 255);
                c.setPixel(x, y, ColorFromPalette(*currentPalette_p, idx, 255, LINEARBLEND));
            }
        }
        t += 0.06f;
    }

private:
    static const uint8_t SRC = 3;
    float t = 0;
};

#endif // WAVE_INTERFERENCE_EFFECT_H
