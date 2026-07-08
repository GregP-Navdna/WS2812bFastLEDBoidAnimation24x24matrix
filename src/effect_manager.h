#ifndef EFFECT_MANAGER_H
#define EFFECT_MANAGER_H

#include <Arduino.h>
#include "effect.h"

// ---------------------------------------------------------------------------
// EffectManager
//
// Owns the registry of effects and drives the active one each frame. It also
// owns presentation: after the active effect renders, the manager calls
// canvas.show() exactly once per frame.
//
// Auto-rotation is opt-in: if the active effect reports a non-zero
// suggestedDurationMs(), the manager advances to the next registered effect
// after that time. With a single registered effect, behavior is "run forever".
// ---------------------------------------------------------------------------

class EffectManager {
public:
    static const uint8_t MAX_EFFECTS = 32;

    EffectManager(EffectContext& ctx) : ctx(ctx) {}

    // Register an effect. Returns the index assigned, or -1 if full.
    int add(Effect* effect) {
        if (count >= MAX_EFFECTS || effect == nullptr) return -1;
        effects[count] = effect;
        return count++;
    }

    // Activate the first registered effect and start the clock.
    void begin(int startIndex = 0) {
        if (count == 0) return;
        activeIndex = constrain(startIndex, 0, count - 1);
        lastUpdateMs = millis();
        effectStartMs = lastUpdateMs;
        effects[activeIndex]->enter(ctx);
        logActive();
    }

    // Drive one frame: update the active effect, present, and handle rotation.
    void update() {
        if (count == 0) return;

        uint32_t now = millis();
        uint32_t dt = now - lastUpdateMs;
        lastUpdateMs = now;

        effects[activeIndex]->update(ctx, dt);
        ctx.canvas.show();

        uint32_t duration = effects[activeIndex]->suggestedDurationMs();
        if (duration > 0 && (now - effectStartMs) >= duration) {
            next();
        }
    }

    // Switch to a specific effect (runs exit() on the old, enter() on the new).
    void setActive(int index) {
        if (count == 0) return;
        index = constrain(index, 0, count - 1);
        if (index == activeIndex) return;

        effects[activeIndex]->exit(ctx);
        activeIndex = index;
        effectStartMs = millis();
        effects[activeIndex]->enter(ctx);
        logActive();
    }

    // Advance to the next effect (wraps around).
    void next() { setActive((activeIndex + 1) % count); }

    int activeEffectIndex() const { return activeIndex; }
    uint8_t effectCount() const { return count; }
    Effect* activeEffect() { return count ? effects[activeIndex] : nullptr; }

private:
    void logActive() {
        #if DEBUG_SERIAL
        Serial.print("[EFFECT] Active: ");
        Serial.println(effects[activeIndex]->name());
        #endif
    }

    EffectContext& ctx;
    Effect* effects[MAX_EFFECTS] = {nullptr};
    uint8_t count = 0;
    int activeIndex = 0;
    uint32_t lastUpdateMs = 0;
    uint32_t effectStartMs = 0;
};

#endif // EFFECT_MANAGER_H
