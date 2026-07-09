#ifndef BOIDS_EFFECT_H
#define BOIDS_EFFECT_H

#include <Arduino.h>
#include <FastLED.h>
#include "../config.h"
#include "../vec2.h"
#include "../spatial_grid.h" // must precede boid.h
#include "../boid.h"
#include "../attractor.h"
#include "../canvas.h"
#include "../palettes.h"
#include "../effect.h"
#include "../feedback.h"

// rran/gran/bran are shared globals (rran is also consumed by simd_fade_to_color
// in simd_utils.h). They are defined once in main.cpp.
extern int rran;
extern int gran;
extern int bran;

// ---------------------------------------------------------------------------
// BoidsEffect: flocking particles driven by a rotating cast of gravitational
// attractors, with periodic "move to center", slow-down/pause, explosion and
// palette events. This is the original main.cpp animation, encapsulated as an
// Effect so it lives alongside other scenes in the framework.
// ---------------------------------------------------------------------------
class BoidsEffect : public Effect {
public:
    // Milkdrop-style video feedback on the 48x48 virtual canvas. Public so the
    // serial/debug interface in main.cpp can switch presets at runtime.
    VideoFeedback feedback;

    const char* name() const override { return "Boids"; }

    // Run for 30s before the manager rotates to the next effect.
    uint32_t suggestedDurationMs() const override { return 30000; }

    void enter(EffectContext& ctx) override {
        if (!spatialGrid) {
            spatialGrid = new SpatialGrid(GRID_CELLS_X, GRID_CELLS_Y, VIRTUAL_ROWS, VIRTUAL_COLS);
        }

        start();

        // Timing initialization (previously in setup()).
        lastSlowDownTime = millis();
        nextSlowDownInterval = random(10000, 40000);
        lastAttractorChangeTime = millis();
        attractorChangeDuration = random(10000, 20000);
        lastRippleTime = millis();
        rippleInterval = random(3000, 8000);

        feedback.clear();
        lastFeedbackChangeTime = millis();
        feedbackChangeDuration = random(20000, 40000);
    }

    void update(EffectContext& ctx, uint32_t dtMs) override {
        int randomnum = random(0, 100);
        movetocenterrandom = random(0, 200);
        if (randomnum == 5) stopbool = true;

        if (!spatialGrid) {
            spatialGrid = new SpatialGrid(GRID_CELLS_X, GRID_CELLS_Y, VIRTUAL_ROWS, VIRTUAL_COLS);
        }

        // Random slow-down state machine.
        if (!isSlowingDown && !isPaused && (millis() - lastSlowDownTime >= nextSlowDownInterval)) {
            randomSlowDownAndSpeed(ctx);
        }
        if (isSlowingDown || isPaused) {
            randomSlowDownAndSpeed(ctx);
        }

        // Occasionally trigger a random overlay effect.
        EVERY_N_SECONDS(15) {
            if (random8() < 180) {
                ctx.overlay.triggerRandomEffect();
            }
        }

        // Dedicated timer for random ripples.
        if (millis() - lastRippleTime > rippleInterval) {
            ctx.overlay.startRipple();
            lastRippleTime = millis();
            rippleInterval = random(100, 1000);
        }

        // Screen shake offsets (if active).
        int8_t shakeOffsetX = 0;
        int8_t shakeOffsetY = 0;
        ctx.overlay.getShakeOffsets(shakeOffsetX, shakeOffsetY);

        // Repopulate the spatial grid for this frame.
        spatialGrid->clear();
        for (int i = 0; i < count; i++) {
            Boid* boid = &boids[i];
            spatialGrid->insert(boid, boid->location.x, boid->location.y);
        }

        updateAttractors(ctx);

        // Feedback preset rotation (same timer mechanism as attractor patterns).
        if (millis() - lastFeedbackChangeTime > feedbackChangeDuration) {
            feedback.nextPreset();
            lastFeedbackChangeTime = millis();
            feedbackChangeDuration = random(20000, 40000);
        }

        const bool fbActive = feedback.enabled();

        if (fbActive) {
            // Step 1: resample prev virtual canvas -> current with transform +
            // decay. The decay replaces the full-canvas fade below.
            feedback.beginFrame();
        } else {
            // Original path (must stay pixel-identical when feedback is OFF):
            // apply overlay FX, then fade.
            ctx.overlay.update(ctx.canvas.raw());
            ctx.canvas.fade(CRGB::Black, 45);
        }

        // Occasionally move to center (self-contained sub-animation).
        if (movetocenterrandom == 100) {
            movetoCenter(ctx);
            movetocenterrandom = 0;
            // movetoCenter ran its own feedback frames (with buffer swaps), so
            // resample again before this frame's boid render.
            if (fbActive) feedback.beginFrame();
        }

        if (stopbool) stopbool = false;

        // Animation parameter evolution.
        EVERY_N_MILLISECONDS(10) {
            attractor5.incrementMass();
            if (fadebyvalue < 10 || fadebyvalue > 120) fadebydir = -fadebydir;
            fadebyvalue += 1 * fadebydir;
        }

        degreestep += 3;
        if (degreestep > 10 || degreestep < 1) degreestepdir = -degreestepdir;
        degree += degreestep * degreedir;
        if (degree > 45 || degree < 1) degreedir = -degreedir;
        attractor5.location.rotateAroundPoint(center.x, center.y, degree);

        EVERY_N_MILLISECONDS(500) {
            if (count <= 2 || count >= 254) countdir = -countdir;
            count += countstep * countdir;
            if (maxspeed <= 0.4 || maxspeed >= 2.5) maxspeeddir = -maxspeeddir;
            maxspeed += maxspeedstep * maxspeeddir;
        }

        EVERY_N_MILLISECONDS(10000) {
            uint8_t palcount = random(0, 24);
            SetNewPalette(palcount);
            setNewRandomColorFacd();
            ctx.overlay.startRipple();
            ctx.overlay.startRipple();
        }

        // Update + render boids.
        for (int i = 0; i < count; i++) {
            Boid* boid = &boids[i];

            // Apply forces from all active attractors.
            for (int j = 0; j < MAX_ATTRACTORS; j++) {
                if (attractorActive[j]) {
                    PVector force = attractorArray[j]->attract(*boid);
                    boid->applyForce(force);
                }
            }

            // Apply explosion repulsor force if active.
            if (explosionActive) {
                PVector explosionForce = explosionRepulsor.attract(*boid);
                boid->applyForce(explosionForce);
            }

            boid->wrapAroundBorders(VIRTUAL_ROWS, VIRTUAL_COLS);
            boid->brightness = map(boid->velocity.x + boid->velocity.y, 0.1, 4.5, 25, 255);
            boid->mass = (255 - count) / 6;
            boid->update(*spatialGrid);

            // Apply screen shake offset when drawing.
            float drawX = boid->location.x + shakeOffsetX;
            float drawY = boid->location.y + shakeOffsetY;

            // Hue based on velocity direction (if enabled).
            uint8_t renderHue;
            if (velocityBasedHue) {
                float angle = atan2(boid->velocity.y, boid->velocity.x);
                renderHue = (uint8_t)((angle + PI) * 40.7436f);
            } else {
                renderHue = boid->hue * 15;
            }

            drawVirtualF(ctx, drawX, drawY,
                         ColorFromPalette(*currentPalette_p, renderHue, boid->brightness, NOBLEND));

            boid->neighbordist = neidist;
            boid->desiredseparation = boidsep;

            if (stopbool) boid->velocity = PVector(0, 0);
        }

        if (fbActive) {
            // Steps 5-6: extract the 24x24 viewport (through the serpentine
            // Canvas::xy mapping), composite the overlay FX on top, then
            // ping-pong the virtual buffers.
            feedback.extractViewport(ctx.canvas, virtualViewX, virtualViewY);
            ctx.overlay.update(ctx.canvas.raw());
            feedback.endFrame();
        }

        #if DEBUG_SERIAL
        EVERY_N_SECONDS(5) {
            if (feedback.enabled()) {
                Serial.print("[FEEDBACK] ");
                Serial.print(feedback.presetName());
                Serial.print(" pass: ");
                Serial.print(feedback.lastPassMicros());
                Serial.print(" us, FPS: ");
                Serial.println(FastLED.getFPS());
            }
        }
        #endif
        // NOTE: presentation (FastLED.show) is handled by the EffectManager.
    }

private:
    // --- Scene geometry ---------------------------------------------------
    static const uint8_t VIRTUAL_ROWS = 48;
    static const uint8_t VIRTUAL_COLS = 48;
    static const uint8_t VIEWPORT_ROWS = 24;
    static const uint8_t VIEWPORT_COLS = 24;
    static const int GRID_CELLS_X = 8;
    static const int GRID_CELLS_Y = 8;
    static const int NUM_PARTICLES = 255;
    static const int MAX_ATTRACTORS = 17;
    static const int NUM_ATTRACTOR_PATTERNS = 11;

    // Viewport position within the virtual canvas.
    uint8_t virtualViewX = 24;
    uint8_t virtualViewY = 24;

    // --- Particles + spatial partitioning ---------------------------------
    Boid boids[NUM_PARTICLES];
    uint8_t count = 254;
    SpatialGrid* spatialGrid = nullptr;

    // --- Attractors -------------------------------------------------------
    Attractor attractor5;   // Main central attractor
    Attractor attractor1;   // Secondary attractor for moveToCenter
    Attractor attractorTopLeft;
    Attractor attractorTopRight;
    Attractor attractorBottomLeft;
    Attractor attractorBottomRight;
    Attractor repulsorCenter;
    Attractor attractorSquare1;
    Attractor attractorSquare2;
    Attractor attractorSquare3;
    Attractor attractorSquare4;
    Attractor attractorFigure8_1;
    Attractor attractorFigure8_2;
    Attractor attractorSpiral1;
    Attractor attractorSpiral2;
    Attractor attractorSpiral3;
    Attractor attractorVortex;
    Attractor explosionRepulsor;

    Attractor* attractorArray[MAX_ATTRACTORS];
    bool attractorActive[MAX_ATTRACTORS] = {true, false, false, false, false, false,
                                            false, false, false, false, false, false,
                                            false, false, false, false, false};
    int currentAttractorPattern = 0;

    // --- Explosion state --------------------------------------------------
    bool explosionActive = false;
    unsigned long explosionStartTime = 0;
    unsigned long lastExplosionTime = 0;

    // --- Coloring / tuning ------------------------------------------------
    bool velocityBasedHue = false;
    uint8_t neidist = random(3, 5);
    uint8_t boidsep = random(3, 5);
    uint8_t fadebyvalue = random(10, 120);
    int fadebydir = 1;
    float maxspeed = 2.5;
    float maxspeedstep = 0.1;
    int maxspeeddir = 1;
    uint8_t countstep = 5;
    int countdir = 1;

    // --- Rotation animation -----------------------------------------------
    double degree = 0;
    int degreestep = 1;
    int degreedir = 1;
    int degreestepdir = 1;
    PVector center = PVector(36, 36);

    // --- State machines / timers ------------------------------------------
    bool stopbool = false;
    int movetocenterrandom = 0;
    bool isSlowingDown = false;
    bool isPaused = false;
    unsigned long pauseStartTime = 0;
    unsigned long pauseDuration = 0;
    unsigned long lastSlowDownTime = 0;
    unsigned long nextSlowDownInterval = 0;
    unsigned long lastAttractorChangeTime = 0;
    unsigned long attractorChangeDuration = 15000;
    unsigned long lastRippleTime = 0;
    unsigned long rippleInterval = 0;
    unsigned long lastFeedbackChangeTime = 0;
    unsigned long feedbackChangeDuration = 30000;

    // --- Drawing helper ---------------------------------------------------
    // Draw a virtual-canvas coordinate with Wu anti-aliasing. When feedback is
    // active, boids blend additively onto the recirculating virtual canvas
    // (world space, so trails survive viewport movement); otherwise they map
    // straight into the physical viewport as before.
    void drawVirtualF(EffectContext& ctx, float virtualX, float virtualY, CRGB color) {
        if (feedback.enabled()) {
            feedback.drawPixelF(virtualX, virtualY, color);
        } else {
            ctx.canvas.drawPixelF(virtualX - virtualViewX, virtualY - virtualViewY, color);
        }
    }

    // --- Scene setup ------------------------------------------------------
    void start() {
        for (int i = 0; i < count; i++) {
            boids[i] = Boid(random(COLS), 0);
        }

        attractor1.setlocation((virtualViewX + 24 / 2), (virtualViewY + 24 / 2));
        attractor1.setMass(100);
        attractor1.setG(4);

        attractor5.setlocation((virtualViewX + 36 / 2), (virtualViewY + 36 / 2));
        attractor5.setMass(1);
        attractor5.setG(0.1);

        initAttractorPatterns();
        setAttractorPattern(0);
        lastAttractorChangeTime = millis();
    }

    void initAttractorPatterns() {
        attractorArray[0] = &attractor5;
        attractorArray[1] = &attractor1;
        attractorArray[2] = &attractorTopLeft;
        attractorArray[3] = &attractorTopRight;
        attractorArray[4] = &attractorBottomLeft;
        attractorArray[5] = &attractorBottomRight;
        attractorArray[6] = &repulsorCenter;
        attractorArray[7] = &attractorSquare1;
        attractorArray[8] = &attractorSquare2;
        attractorArray[9] = &attractorSquare3;
        attractorArray[10] = &attractorSquare4;
        attractorArray[11] = &attractorFigure8_1;
        attractorArray[12] = &attractorFigure8_2;
        attractorArray[13] = &attractorSpiral1;
        attractorArray[14] = &attractorSpiral2;
        attractorArray[15] = &attractorSpiral3;
        attractorArray[16] = &attractorVortex;

        float centerX = virtualViewX + VIRTUAL_COLS / 2;
        float centerY = virtualViewY + VIRTUAL_ROWS / 2;

        attractorTopLeft.setlocation(virtualViewX + 8, virtualViewY + 8);
        attractorTopLeft.setMass(20);
        attractorTopLeft.setG(1.5);
        attractorTopLeft.setRadius(10.0, 80.0);
        attractorTopLeft.color = CRGB::Green;

        attractorTopRight.setlocation(virtualViewX + VIRTUAL_COLS - 8, virtualViewY + 8);
        attractorTopRight.setMass(20);
        attractorTopRight.setG(1.5);
        attractorTopRight.setRadius(10.0, 80.0);
        attractorTopRight.color = CRGB::Yellow;

        attractorBottomLeft.setlocation(virtualViewX + 8, virtualViewY + VIRTUAL_ROWS - 8);
        attractorBottomLeft.setMass(20);
        attractorBottomLeft.setG(1.5);
        attractorBottomLeft.setRadius(10.0, 80.0);
        attractorBottomLeft.color = CRGB::Purple;

        attractorBottomRight.setlocation(virtualViewX + VIRTUAL_COLS - 8, virtualViewY + VIRTUAL_ROWS - 8);
        attractorBottomRight.setMass(20);
        attractorBottomRight.setG(1.5);
        attractorBottomRight.setRadius(10.0, 80.0);
        attractorBottomRight.color = CRGB::Aqua;

        repulsorCenter.setlocation(centerX, centerY);
        repulsorCenter.setMass(50);
        repulsorCenter.setG(3.0);
        repulsorCenter.setRepulsor(true);
        repulsorCenter.setRadius(5.0, 120.0);

        attractorSquare1.setlocation(centerX, centerY);
        attractorSquare1.setMass(25);
        attractorSquare1.setG(1.8);
        attractorSquare1.setRadius(10.0, 90.0);
        attractorSquare1.color = CRGB::Cyan;

        attractorSquare2.setlocation(centerX, centerY);
        attractorSquare2.setMass(25);
        attractorSquare2.setG(1.8);
        attractorSquare2.setRadius(10.0, 90.0);
        attractorSquare2.color = CRGB::Magenta;

        attractorSquare3.setlocation(centerX, centerY);
        attractorSquare3.setMass(25);
        attractorSquare3.setG(1.8);
        attractorSquare3.setRadius(10.0, 90.0);
        attractorSquare3.color = CRGB::Orange;

        attractorSquare4.setlocation(centerX, centerY);
        attractorSquare4.setMass(25);
        attractorSquare4.setG(1.8);
        attractorSquare4.setRadius(10.0, 90.0);
        attractorSquare4.color = CRGB::Lime;

        attractorFigure8_1.setlocation(centerX, centerY);
        attractorFigure8_1.setMass(30);
        attractorFigure8_1.setG(2.0);
        attractorFigure8_1.setRadius(10.0, 100.0);
        attractorFigure8_1.color = CRGB::HotPink;

        attractorFigure8_2.setlocation(centerX, centerY);
        attractorFigure8_2.setMass(30);
        attractorFigure8_2.setG(2.0);
        attractorFigure8_2.setRadius(10.0, 100.0);
        attractorFigure8_2.color = CRGB::DeepSkyBlue;

        attractorSpiral1.setlocation(centerX, centerY);
        attractorSpiral1.setMass(20);
        attractorSpiral1.setG(1.5);
        attractorSpiral1.setRadius(10.0, 85.0);
        attractorSpiral1.color = CRGB::Gold;

        attractorSpiral2.setlocation(centerX, centerY);
        attractorSpiral2.setMass(20);
        attractorSpiral2.setG(1.5);
        attractorSpiral2.setRadius(10.0, 85.0);
        attractorSpiral2.color = CRGB::Violet;

        attractorSpiral3.setlocation(centerX, centerY);
        attractorSpiral3.setMass(20);
        attractorSpiral3.setG(1.5);
        attractorSpiral3.setRadius(10.0, 85.0);
        attractorSpiral3.color = CRGB::Turquoise;

        attractorVortex.setlocation(centerX, centerY);
        attractorVortex.setMass(40);
        attractorVortex.setG(2.5);
        attractorVortex.setRadius(5.0, 110.0);
        attractorVortex.setVortex(true, 0.8);
        attractorVortex.color = CRGB::Purple;

        explosionRepulsor.setlocation(centerX, centerY);
        explosionRepulsor.setMass(60);
        explosionRepulsor.setG(4.0);
        explosionRepulsor.setRepulsor(true);
        explosionRepulsor.setRadius(5.0, 100.0);
        explosionRepulsor.color = CRGB::Red;
    }

    void setAttractorPattern(int patternIndex) {
        #if DEBUG_SERIAL
        Serial.print("[PATTERN] Switching to pattern: ");
        Serial.println(patternIndex);
        #endif

        if (patternIndex < 0 || patternIndex >= NUM_ATTRACTOR_PATTERNS) {
            #if DEBUG_SERIAL
            Serial.print("[ERROR] Invalid pattern index: ");
            Serial.println(patternIndex);
            #endif
            return;
        }

        for (int i = 0; i < MAX_ATTRACTORS; i++) {
            attractorActive[i] = false;
        }
        attractorActive[0] = true; // Always keep the main attractor active.

        switch (patternIndex) {
            case 0: // Default - just the main attractor
                break;
            case 1: // Four corners
                attractorActive[2] = true;
                attractorActive[3] = true;
                attractorActive[4] = true;
                attractorActive[5] = true;
                break;
            case 2: // Central repulsor with corner attractors
                attractorActive[2] = true;
                attractorActive[3] = true;
                attractorActive[4] = true;
                attractorActive[5] = true;
                attractorActive[6] = true;
                break;
            case 3: // Diagonal TL-BR
                attractorActive[2] = true;
                attractorActive[5] = true;
                break;
            case 4: // Diagonal TR-BL
                attractorActive[3] = true;
                attractorActive[4] = true;
                break;
            case 5: // Central repulsor only
                attractorActive[6] = true;
                break;
            case 6: // Rotating square
                attractorActive[7] = true;
                attractorActive[8] = true;
                attractorActive[9] = true;
                attractorActive[10] = true;
                break;
            case 7: // Figure-8
                attractorActive[11] = true;
                attractorActive[12] = true;
                break;
            case 8: // Pulsing center
                break;
            case 9: // Spiral arms
                attractorActive[13] = true;
                attractorActive[14] = true;
                attractorActive[15] = true;
                break;
            case 10: // Central vortex
                attractorActive[16] = true;
                break;
        }

        currentAttractorPattern = patternIndex;
    }

    void updateAttractors(EffectContext& ctx) {
        float centerX = virtualViewX + VIRTUAL_COLS / 2;
        float centerY = virtualViewY + VIRTUAL_ROWS / 2;

        if (attractorActive[0]) {
            attractorArray[0]->orbit(centerX, centerY, 10, 0.2, 0);
            attractorArray[0]->incrementMass();
        }

        if (attractorActive[2]) {
            float tlx = virtualViewX + VIRTUAL_COLS * 0.25;
            float tly = virtualViewY + VIRTUAL_ROWS * 0.25;
            attractorArray[2]->orbit(tlx, tly, 5, 0.5, 0);
        }
        if (attractorActive[3]) {
            float trx = virtualViewX + VIRTUAL_COLS * 0.75;
            float try_ = virtualViewY + VIRTUAL_ROWS * 0.25;
            attractorArray[3]->orbit(trx, try_, 5, 0.5, PI / 2);
        }
        if (attractorActive[4]) {
            float blx = virtualViewX + VIRTUAL_COLS * 0.25;
            float bly = virtualViewY + VIRTUAL_ROWS * 0.75;
            attractorArray[4]->orbit(blx, bly, 5, 0.5, PI);
        }
        if (attractorActive[5]) {
            float brx = virtualViewX + VIRTUAL_COLS * 0.75;
            float bry = virtualViewY + VIRTUAL_ROWS * 0.75;
            attractorArray[5]->orbit(brx, bry, 5, 0.5, PI * 1.5);
        }

        if (attractorActive[6]) {
            float pulseFactor = (sin(millis() / 1000.0) + 1) / 2.0;
            attractorArray[6]->setG(1.0 + pulseFactor * 3.0);
        }

        // Rotating square pattern (7-10).
        if (attractorActive[7] || attractorActive[8] || attractorActive[9] || attractorActive[10]) {
            float squareRadius = 15;
            float squareSpeed = 0.4;
            float time = millis() / 1000.0;

            if (attractorActive[7]) {
                float angle = time * squareSpeed;
                attractorArray[7]->setlocation(centerX + squareRadius * cos(angle),
                                               centerY + squareRadius * sin(angle));
            }
            if (attractorActive[8]) {
                float angle = time * squareSpeed + PI / 2;
                attractorArray[8]->setlocation(centerX + squareRadius * cos(angle),
                                               centerY + squareRadius * sin(angle));
            }
            if (attractorActive[9]) {
                float angle = time * squareSpeed + PI;
                attractorArray[9]->setlocation(centerX + squareRadius * cos(angle),
                                               centerY + squareRadius * sin(angle));
            }
            if (attractorActive[10]) {
                float angle = time * squareSpeed + PI * 1.5;
                attractorArray[10]->setlocation(centerX + squareRadius * cos(angle),
                                                centerY + squareRadius * sin(angle));
            }
        }

        // Figure-8 pattern (11-12).
        if (attractorActive[11] || attractorActive[12]) {
            float time = millis() / 1000.0;
            float speed = 0.5;
            float width = 18;
            float height = 10;

            if (attractorActive[11]) {
                float t = time * speed;
                float sinT = sin(t);
                float cosT = cos(t);
                float denominator = 1.0f + sinT * sinT;
                if (denominator > 0.1f) {
                    float scale = width / denominator;
                    float x = centerX + scale * cosT;
                    float y = centerY + height * sinT * cosT;
                    attractorArray[11]->setlocation(x, y);
                }
            }
            if (attractorActive[12]) {
                float t = time * speed + PI;
                float sinT = sin(t);
                float cosT = cos(t);
                float denominator = 1.0f + sinT * sinT;
                if (denominator > 0.1f) {
                    float scale = width / denominator;
                    float x = centerX + scale * cosT;
                    float y = centerY + height * sinT * cosT;
                    attractorArray[12]->setlocation(x, y);
                }
            }
        }

        // Pulsing center pattern (attractor 0 enhanced).
        if (currentAttractorPattern == 8 && attractorActive[0]) {
            float pulseFactor = (sin(millis() / 500.0) + 1) / 2.0;
            attractorArray[0]->setMass(10 + pulseFactor * 60);
            attractorArray[0]->setG(0.5 + pulseFactor * 3.5);
        }

        // Spiral pattern (13-15).
        if (attractorActive[13] || attractorActive[14] || attractorActive[15]) {
            float spiralSpeed = 0.6;
            float spiralRadius = 16;
            if (attractorActive[13]) attractorArray[13]->orbit(centerX, centerY, spiralRadius, spiralSpeed, 0);
            if (attractorActive[14]) attractorArray[14]->orbit(centerX, centerY, spiralRadius, spiralSpeed, (2 * PI) / 3);
            if (attractorActive[15]) attractorArray[15]->orbit(centerX, centerY, spiralRadius, spiralSpeed, (4 * PI) / 3);
        }

        // Vortex attractor (16).
        if (attractorActive[16]) {
            attractorArray[16]->orbit(centerX, centerY, 5, 0.15, 0);
            float pulseFactor = (sin(millis() / 800.0) + 1) / 2.0;
            attractorArray[16]->setG(1.5 + pulseFactor * 2.0);
        }

        // Explosion handling.
        if (explosionActive) {
            if (millis() - explosionStartTime > 500) {
                explosionActive = false;
                #if DEBUG_SERIAL
                Serial.println("[EXPLOSION] Ended");
                #endif
            }
        } else {
            if (millis() - lastExplosionTime > 30000 && random8() < 13) {
                #if DEBUG_SERIAL
                Serial.println("[EXPLOSION] Starting!");
                #endif
                explosionActive = true;
                explosionStartTime = millis();
                lastExplosionTime = millis();
                float explodeX = centerX + random(0, 20) - 10;
                float explodeY = centerY + random(0, 20) - 10;
                explodeX = constrain(explodeX, virtualViewX + 2, virtualViewX + VIRTUAL_COLS - 2);
                explodeY = constrain(explodeY, virtualViewY + 2, virtualViewY + VIRTUAL_ROWS - 2);
                explosionRepulsor.setlocation(explodeX, explodeY);

                ctx.overlay.startScreenShake(8, 2);
                ctx.overlay.startRipple();
                delay(50);
                ctx.overlay.startRipple();
            }
        }

        // Pattern switching.
        if (millis() - lastAttractorChangeTime > attractorChangeDuration) {
            #if DEBUG_SERIAL
            Serial.println("[PATTERN] Time to switch patterns");
            #endif
            int newPattern = random(0, NUM_ATTRACTOR_PATTERNS);
            int attempts = 0;
            while (newPattern == currentAttractorPattern && attempts < 20) {
                newPattern = random(0, NUM_ATTRACTOR_PATTERNS);
                attempts++;
            }
            setAttractorPattern(newPattern);

            if (random8() < 77) {
                velocityBasedHue = !velocityBasedHue;
            }

            ctx.overlay.startRipple();
            ctx.overlay.startRipple();
            if (random8() < 180) {
                ctx.overlay.startScreenShake(5, 1);
            }

            lastAttractorChangeTime = millis();
            attractorChangeDuration = random(10000, 30000);
        }

        // Draw active attractors (debug visualization).
        for (int i = 0; i < MAX_ATTRACTORS; i++) {
            if (attractorActive[i]) {
                attractorArray[i]->draw(ctx.canvas, virtualViewX, virtualViewY);
            }
        }
    }

    void movetoCenter(EffectContext& ctx) {
        int countto = random(5, 25);

        if (!spatialGrid) {
            spatialGrid = new SpatialGrid(GRID_CELLS_X, GRID_CELLS_Y, VIRTUAL_ROWS, VIRTUAL_COLS);
        }

        ctx.overlay.startColorWash(1, 8, 25); // Vertical wash

        spatialGrid->clear();
        for (int i = 0; i < count; i++) {
            Boid* boid = &boids[i];
            spatialGrid->insert(boid, boid->location.x, boid->location.y);
        }

        for (int j = 0; j < countto; j++) {
            spatialGrid->clear();
            for (int i = 0; i < count; i++) {
                Boid* boid = &boids[i];
                spatialGrid->insert(boid, boid->location.x, boid->location.y);
            }

            if (feedback.enabled()) feedback.beginFrame();

            for (int i = 0; i < count; i++) {
                Boid* boid = &boids[i];
                PVector force1 = attractor1.attract(*boid);
                boid->applyForce(force1);

                boid->update(*spatialGrid);
                boid->wrapAroundBorders(VIRTUAL_ROWS, VIRTUAL_COLS);

                drawVirtualF(ctx, boid->location.x, boid->location.y,
                             ColorFromPalette(*currentPalette_p, boid->hue * 15, 255, NOBLEND));
                boid->neighbordist = neidist;
                boid->desiredseparation = boidsep;

                if (stopbool) boid->velocity = PVector(0, 0);
            }

            if (feedback.enabled()) {
                feedback.extractViewport(ctx.canvas, virtualViewX, virtualViewY);
                ctx.overlay.update(ctx.canvas.raw());
                feedback.endFrame();
            } else {
                ctx.overlay.update(ctx.canvas.raw());
                ctx.canvas.fade(CRGB::Black, 45);
            }
            ctx.canvas.show();
        }

        ctx.overlay.stopColorWash();
        ctx.overlay.startRipple();
        ctx.overlay.startRipple();
    }

    void setNewRandomColorFacd() {
        rran = random(1.5F, 4.0F);
        gran = random(1.5F, 4.0F);
        bran = random(1.5F, 4.0F);
    }

    void randomSlowDownAndSpeed(EffectContext& ctx) {
        static float originalSpeeds[NUM_PARTICLES];

        // Phase 1: start slowing down - save original speeds.
        if (!isSlowingDown && !isPaused) {
            isSlowingDown = true;
            for (int i = 0; i < count; i++) {
                originalSpeeds[i] = boids[i].maxspeed;
            }
            ctx.overlay.startScreenShake(8, 1);
        }

        // Phase 2: gradually reduce speed.
        if (isSlowingDown) {
            bool allStopped = true;
            for (int i = 0; i < count; i++) {
                if (boids[i].maxspeed > 0.2) {
                    boids[i].maxspeed -= 0.1;
                    allStopped = false;
                } else {
                    boids[i].maxspeed = 0;
                }
            }

            if (allStopped) {
                isSlowingDown = false;
                isPaused = true;
                pauseStartTime = millis();
                pauseDuration = random(3000, 15000);
                ctx.overlay.startStarfield();
            }
        }

        // Phase 3: after pause, set random speeds.
        if (isPaused && (millis() - pauseStartTime >= pauseDuration)) {
            isPaused = false;
            for (int i = 0; i < count; i++) {
                boids[i].maxspeed = random(1.1F, 2.0F);
            }
            lastSlowDownTime = millis();
            nextSlowDownInterval = random(10000, 40000);
            ctx.overlay.stopStarfield();
            ctx.overlay.startRipple();
            ctx.overlay.startRipple();
        }
    }
};

#endif // BOIDS_EFFECT_H
