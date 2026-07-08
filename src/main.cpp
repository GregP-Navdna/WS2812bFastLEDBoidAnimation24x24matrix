// ---------------------------------------------------------------------------
// WS2812B 24x24 LED matrix animation framework.
//
// main.cpp is intentionally thin: it wires up the shared services (LED buffer,
// Canvas, overlay FX) and the EffectManager, then registers effects and runs
// the loop. All animation logic lives in self-contained effects under
// src/effects/. See src/effect.h for the 3-step recipe to add a new effect.
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <FastLED.h>

#include "config.h"
#include "simd_utils.h"
#include "canvas.h"
#include "palettes.h"
#include "matrix_effects.h"
#include "effect.h"
#include "effect_manager.h"
#include "effects/boids_effect.h"
#include "effects/plasma_effect.h"
#include "effects/fire_effect.h"
#include "effects/matrix_rain_effect.h"
#include "effects/fireworks_effect.h"
#include "effects/meteor_shower_effect.h"
#include "effects/starfield_effect.h"
#include "effects/spiral_effect.h"
#include "effects/wave_interference_effect.h"
#include "effects/voronoi_effect.h"
#include "effects/ripple_rings_effect.h"
#include "effects/game_of_life_effect.h"
#include "effects/metaballs_effect.h"
#include "effects/aurora_effect.h"
#include "effects/noise_field_effect.h"
#include "effects/tunnel_effect.h"
#include "effects/kaleidoscope_effect.h"
#include "effects/cube3d_effect.h"
#include "effects/dna_helix_effect.h"
#include "effects/bouncing_balls_effect.h"
#include "effects/confetti_effect.h"
#include "effects/spectrum_bars_effect.h"

// --- Shared globals ---------------------------------------------------------
// LED frame buffer.
CRGB leds[NUM_LEDS];

// Color-fade randomization factors. rran is also consumed by
// simd_fade_to_color() (declared extern in simd_utils.h), so it must live here.
int rran = random(1.5F, 4.0F);
int gran = random(1.5F, 4.0F);
int bran = random(1.5F, 4.0F);

// --- Shared services --------------------------------------------------------
Canvas canvas(leds, ROWS, COLS);              // pixel-placement layer
MatrixEffects overlay(ROWS, COLS, matrixXY);  // overlay FX layer
EffectContext context(canvas, overlay);       // services handed to each effect
EffectManager manager(context);

// --- Effects ----------------------------------------------------------------
// Each effect reports a suggestedDurationMs(), so the EffectManager rotates
// through them automatically in registration order.
BoidsEffect boidsEffect;
PlasmaEffect plasmaEffect;
FireEffect fireEffect;
MatrixRainEffect matrixRainEffect;
FireworksEffect fireworksEffect;
MeteorShowerEffect meteorShowerEffect;
StarfieldEffect starfieldEffect;
SpiralEffect spiralEffect;
WaveInterferenceEffect waveInterferenceEffect;
VoronoiEffect voronoiEffect;
RippleRingsEffect rippleRingsEffect;
GameOfLifeEffect gameOfLifeEffect;
MetaballsEffect metaballsEffect;
AuroraEffect auroraEffect;
NoiseFieldEffect noiseFieldEffect;
TunnelEffect tunnelEffect;
KaleidoscopeEffect kaleidoscopeEffect;
Cube3dEffect cube3dEffect;
DnaHelixEffect dnaHelixEffect;
BouncingBallsEffect bouncingBallsEffect;
ConfettiEffect confettiEffect;
SpectrumBarsEffect spectrumBarsEffect;

void setup() {
    #if DEBUG_SERIAL
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n[SETUP] Starting WS2812B LED Matrix Framework");
    #endif

    init_simd();
    randomSeed(analogRead(0));

    delay(3000);
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);

    // Register effects (order defines rotation order; index 0 boots first).
    manager.add(&boidsEffect);
    manager.add(&plasmaEffect);
    manager.add(&fireEffect);
    manager.add(&matrixRainEffect);
    manager.add(&fireworksEffect);
    manager.add(&meteorShowerEffect);
    manager.add(&starfieldEffect);
    manager.add(&spiralEffect);
    manager.add(&waveInterferenceEffect);
    manager.add(&voronoiEffect);
    manager.add(&rippleRingsEffect);
    manager.add(&gameOfLifeEffect);
    manager.add(&metaballsEffect);
    manager.add(&auroraEffect);
    manager.add(&noiseFieldEffect);
    manager.add(&tunnelEffect);
    manager.add(&kaleidoscopeEffect);
    manager.add(&cube3dEffect);
    manager.add(&dnaHelixEffect);
    manager.add(&bouncingBallsEffect);
    manager.add(&confettiEffect);
    manager.add(&spectrumBarsEffect);
    manager.begin(0);
}

void loop() {
    manager.update();
}
