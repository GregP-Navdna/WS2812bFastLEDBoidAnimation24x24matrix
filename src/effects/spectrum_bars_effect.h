#ifndef SPECTRUM_BARS_EFFECT_H
#define SPECTRUM_BARS_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// SpectrumBarsEffect: a simulated audio equalizer. Each column is a frequency
// bar whose height is driven by per-band oscillators plus noise, colored green
// -> yellow -> red by height, with falling white peak markers.
// ---------------------------------------------------------------------------
class SpectrumBarsEffect : public Effect {
public:
    const char* name() const override { return "Spectrum Bars"; }
    uint32_t suggestedDurationMs() const override { return 18000; }

    void enter(EffectContext& ctx) override {
        ctx.canvas.clear();
        for (uint8_t x = 0; x < COLS; x++) {
            bpm[x] = random8(8, 28);
            phase[x] = random8();
            peak[x] = 0;
        }
        t = 0;
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;
        const uint8_t H = c.height;
        c.clear();

        for (uint8_t x = 0; x < c.width; x++) {
            // Combine an oscillator with drifting noise for a lively bar.
            uint8_t osc = beatsin8(bpm[x], 0, 255, 0, phase[x]);
            uint8_t n = inoise8(x * 90, t);
            uint8_t level = scale8(osc, 180) + scale8(n, 90);
            uint8_t h = 1 + (uint8_t)(((uint16_t)level * (H - 1)) / 255);

            for (uint8_t y = 0; y < h; y++) {
                uint8_t ratio = (uint8_t)(((uint16_t)y * 255) / (H - 1));
                uint8_t hue = 96 - scale8(ratio, 96); // green(96) -> red(0)
                c.setPixel(x, H - 1 - y, CHSV(hue, 255, 255));
            }

            // Peak marker with gravity.
            if (h >= peak[x]) peak[x] = h;
            else if (peak[x] > 0 && (t & 0x03) == 0) peak[x]--;
            if (peak[x] > 0) c.setPixel(x, H - peak[x], CRGB(255, 255, 255));
        }
        t += 3;
    }

private:
    uint8_t bpm[COLS];
    uint8_t phase[COLS];
    uint8_t peak[COLS];
    uint16_t t = 0;
};

#endif // SPECTRUM_BARS_EFFECT_H
