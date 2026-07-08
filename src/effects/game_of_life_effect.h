#ifndef GAME_OF_LIFE_EFFECT_H
#define GAME_OF_LIFE_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../canvas.h"
#include "../palettes.h"
#include "../effect.h"

// ---------------------------------------------------------------------------
// GameOfLifeEffect: Conway's Game of Life on a toroidal grid. Live cells glow
// in the active palette; the canvas fade leaves ghostly trails as cells die.
// The board reseeds when it dies out or stagnates.
// ---------------------------------------------------------------------------
class GameOfLifeEffect : public Effect {
public:
    const char* name() const override { return "Game of Life"; }
    uint32_t suggestedDurationMs() const override { return 20000; }

    void enter(EffectContext& ctx) override {
        SetNewPalette(random(0, 24));
        ctx.canvas.clear();
        seed();
        lastStep = millis();
        lastSeed = millis();
    }

    void update(EffectContext& ctx, uint32_t) override {
        Canvas& c = ctx.canvas;
        c.fade(CRGB::Black, 60); // trails

        if (millis() - lastStep > 110) {
            step();
            hue += 2;
            lastStep = millis();
        }

        for (uint8_t y = 0; y < c.height; y++) {
            for (uint8_t x = 0; x < c.width; x++) {
                if (cells[y * COLS + x]) {
                    c.setPixel(x, y, ColorFromPalette(*currentPalette_p, hue + x + y, 255, LINEARBLEND));
                }
            }
        }

        if (population == 0 || millis() - lastSeed > 15000) {
            seed();
            lastSeed = millis();
        }
    }

private:
    uint8_t cells[NUM_LEDS];
    uint8_t nxt[NUM_LEDS];
    uint16_t population = 0;
    uint8_t hue = 0;
    uint32_t lastStep = 0;
    uint32_t lastSeed = 0;

    void seed() {
        population = 0;
        for (uint16_t i = 0; i < NUM_LEDS; i++) {
            cells[i] = (random8() < 90) ? 1 : 0; // ~35% alive
            population += cells[i];
        }
    }

    uint8_t neighbors(int x, int y) {
        uint8_t n = 0;
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;
                int nx = (x + dx + COLS) % COLS;
                int ny = (y + dy + ROWS) % ROWS;
                n += cells[ny * COLS + nx];
            }
        }
        return n;
    }

    void step() {
        population = 0;
        for (int y = 0; y < ROWS; y++) {
            for (int x = 0; x < COLS; x++) {
                uint8_t n = neighbors(x, y);
                uint8_t alive = cells[y * COLS + x];
                uint8_t next = (alive && (n == 2 || n == 3)) || (!alive && n == 3) ? 1 : 0;
                nxt[y * COLS + x] = next;
                population += next;
            }
        }
        memcpy(cells, nxt, sizeof(cells));
    }
};

#endif // GAME_OF_LIFE_EFFECT_H
