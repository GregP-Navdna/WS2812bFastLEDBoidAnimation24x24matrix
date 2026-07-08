#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <FastLED.h>

// ---------------------------------------------------------------------------
// Central hardware / matrix configuration.
//
// Anything that describes the *physical* display or build options lives here so
// that effects and shared helpers (Canvas, EffectManager, ...) can depend on a
// single source of truth. Effect-specific tuning belongs in the effect itself
// (e.g. virtual-canvas / particle counts live in effects/boids_effect.h).
// ---------------------------------------------------------------------------

// --- LED strip wiring ---
#define LED_PIN 42          // ESP32-S3 data pin
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS 255      // global FastLED brightness (0-255)

// --- Matrix geometry ---
static const uint8_t ROWS = 24;
static const uint8_t COLS = 24;
static const bool kMatrixSerpentineLayout = true;

// Orientation flips (applied in matrixXY, so ALL effects flip consistently).
// Set kMatrixFlipV = true if the display is mounted upside-down (y=0 should be
// the physical top). Set kMatrixFlipH = true to mirror left/right.
static const bool kMatrixFlipV = true;
static const bool kMatrixFlipH = false;

#define NUM_LEDS (ROWS * COLS)

// --- Diagnostics ---
// Set to false to strip serial logging from the build.
#define DEBUG_SERIAL true

#endif // CONFIG_H
