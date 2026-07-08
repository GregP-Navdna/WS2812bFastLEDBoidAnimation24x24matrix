#ifndef EFFECT_H
#define EFFECT_H

#include <Arduino.h>
#include "canvas.h"
#include "matrix_effects.h"

// ---------------------------------------------------------------------------
// Effect framework
//
// An "Effect" is a self-contained, full-screen animation (a scene). All effects
// share the same lifecycle and the same drawing helpers, so adding a new one is
// mechanical:
//
//   1. Create effects/my_effect.h with `class MyEffect : public Effect`.
//   2. Implement name() + update(); optionally enter()/exit()/suggestedDurationMs().
//   3. Register an instance with the EffectManager in main.cpp.
//
// Effects never touch the raw LED buffer or FastLED directly: they draw through
// ctx.canvas (pixel placement, fades, anti-aliasing) and may trigger shared
// overlay FX through ctx.overlay (ripples, starfield, screen shake, ...).
// The EffectManager owns presentation (calls canvas.show() once per frame).
// ---------------------------------------------------------------------------

// Shared services handed to every effect each frame.
struct EffectContext {
    Canvas& canvas;        // pixel-placement layer
    MatrixEffects& overlay; // shared overlay FX layer

    EffectContext(Canvas& c, MatrixEffects& o) : canvas(c), overlay(o) {}
};

class Effect {
public:
    virtual ~Effect() {}

    // Human-readable name (used for logging / selection).
    virtual const char* name() const = 0;

    // Called once when this effect becomes active. Use it to (re)initialize
    // state and allocate any per-effect resources.
    virtual void enter(EffectContext& ctx) {}

    // Called every frame while active. dtMs is the time since the previous
    // update in milliseconds. Draw the frame into ctx.canvas; do NOT call
    // canvas.show() for the normal path (the manager presents the frame).
    virtual void update(EffectContext& ctx, uint32_t dtMs) = 0;

    // Called once when this effect is deactivated. Release transient resources.
    virtual void exit(EffectContext& ctx) {}

    // Optional auto-rotation hint. Return 0 to run indefinitely (the manager
    // will not switch away automatically); return a positive value in
    // milliseconds to request rotation to the next effect after that duration.
    virtual uint32_t suggestedDurationMs() const { return 0; }
};

#endif // EFFECT_H
