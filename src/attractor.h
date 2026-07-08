#ifndef ATTRACTOR_H
#define ATTRACTOR_H

#include <Arduino.h>
#include <FastLED.h>
#include "vec2.h"
#include "spatial_grid.h" // must precede boid.h
#include "boid.h"
#include "canvas.h"

// ---------------------------------------------------------------------------
// Attractor: a gravity (or repulsion) point that pulls/pushes boids.
//
// Extracted from main.cpp. Defaults are self-contained; callers configure each
// instance explicitly via setMass()/setG()/setlocation()/setRadius()/...
// ---------------------------------------------------------------------------
class Attractor {
public:
    float mass;                       // Mass, tied to size
    int massdir = 1;
    float massstep = 0.1;
    float GStep = 0.1;
    int gdir = 1;
    uint8_t massdelaywait = random(5, 20);
    uint8_t delaytimer = 0;
    uint8_t massdelay = random(20, 800);
    float G;                          // Gravitational constant
    PVector location;                 // Location
    bool isRepulsor = false;          // If true, repels instead of attracts
    float maxInfluenceRadius = 100.0; // Maximum distance of influence
    float minDistance = 15.0;         // Minimum distance to prevent extreme forces
    CRGB color = CRGB::Blue;          // Visual color for debugging
    bool showVisually = false;        // Whether to render this attractor
    bool hasVortex = false;           // Whether to apply rotational force
    float vortexStrength = 0.5;       // Strength of the vortex effect

    Attractor() {
        location = PVector(0, 0);
        mass = 40;
        G = 2.90;
    }

    void setlocation(float x, float y) { location = PVector(x, y); }
    void setMass(float m) { mass = m; }
    void setG(float g) { G = g; }

    void setRepulsor(bool repel) {
        isRepulsor = repel;
        color = repel ? CRGB::Red : CRGB::Blue;
    }

    void setRadius(float minDist, float maxDist) {
        minDistance = minDist;
        maxInfluenceRadius = maxDist;
    }

    void setVortex(bool enabled, float strength = 0.5) {
        hasVortex = enabled;
        vortexStrength = strength;
    }

    void incrementMass() {
        if (delaytimer > massdelaywait) {
            mass += (mass / 8);
            G += 0.5 * gdir;
            if (G > 4 || G < 0.50) gdir = -gdir;
        }
        if (mass > massdelay) {
            delaytimer = 0;
            mass = 1;
            massdelaywait = random(2, 50);
            massdelay = random(300, 400);
        }
        if (mass == 1) delaytimer += 1;
    }

    void incrementG() {
        if (G > 15) G = 0;
        G *= GStep * gdir;
    }

    // Orbit around a point.
    void orbit(float centerX, float centerY, float radius, float speed, float phase) {
        float angle = (millis() / 1000.0) * speed + phase;
        float x = centerX + radius * cos(angle);
        float y = centerY + radius * sin(angle);
        location = PVector(x, y);
    }

    // Draw the attractor/repulsor (debug visualization).
    // viewX/viewY translate virtual-canvas coordinates into the physical
    // display space that the Canvas operates in.
    void draw(Canvas& canvas, float viewX, float viewY) {
        if (!showVisually) return;

        float radius = constrain(mass / 10.0, 1.0, 3.0);
        CRGB drawColor = color;
        drawColor.nscale8(map(G, 0, 10, 50, 255));

        for (float r = 0; r < radius; r += 0.5) {
            canvas.drawPixelF(location.x - viewX, location.y - viewY, drawColor);
        }
    }

    PVector attract(Boid m) {
        PVector force = location - m.location; // Direction of force
        float d = force.mag();                 // Distance between objects

        // Outside influence radius -> no force.
        if (d > maxInfluenceRadius) return PVector(0, 0);

        d = constrain(d, minDistance, maxInfluenceRadius);
        force.normalize();

        float strength = (G * mass * m.mass) / (d * d);

        if (isRepulsor) {
            force = force * -1;
        }

        force *= strength;

        // Optional vortex / rotational component.
        if (hasVortex && strength > 0.001) {
            PVector tangent = force.ortho();
            float tangentMag = tangent.mag();
            if (tangentMag > 0.001) {
                tangent.normalize();
                tangent *= strength * vortexStrength;
                force += tangent;
            }
        }

        return force;
    }
};

#endif // ATTRACTOR_H
