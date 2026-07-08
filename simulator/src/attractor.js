/**
 * Attractor class - Ported from main.cpp Attractor class
 * Gravitational attractor/repulsor for boid particles
 */
import { Vec2 } from './vec2.js';

export class Attractor {
  constructor() {
    this.location = new Vec2(0, 0);
    this.mass = 40;
    this.G = 2.9;
    this.massdir = 1;
    this.massstep = 0.1;
    this.GStep = 0.1;
    this.gdir = 1;
    this.massdelaywait = Math.floor(Math.random() * 15) + 5;
    this.delaytimer = 0;
    this.massdelay = Math.floor(Math.random() * 380) + 20;
    this.isRepulsor = false;
    this.maxInfluenceRadius = 100.0;
    this.minDistance = 15.0;
    this.color = { r: 0, g: 0, b: 255 }; // Blue
    this.showVisually = false;
    this.hasVortex = false;
    this.vortexStrength = 0.5;
  }

  setLocation(x, y) {
    this.location = new Vec2(x, y);
  }

  setMass(m) {
    this.mass = m;
  }

  setG(g) {
    this.G = g;
  }

  setRepulsor(repel) {
    this.isRepulsor = repel;
    if (repel) {
      this.color = { r: 255, g: 0, b: 0 };
    } else {
      this.color = { r: 0, g: 0, b: 255 };
    }
  }

  setRadius(minDist, maxDist) {
    this.minDistance = minDist;
    this.maxInfluenceRadius = maxDist;
  }

  setVortex(enabled, strength = 0.5) {
    this.hasVortex = enabled;
    this.vortexStrength = strength;
  }

  incrementMass() {
    if (this.delaytimer > this.massdelaywait) {
      this.mass += (this.mass / 8);
      this.G += 0.5 * this.gdir;
      if (this.G > 4 || this.G < 0.50) this.gdir = -this.gdir;
    }
    if (this.mass > this.massdelay) {
      this.delaytimer = 0;
      this.mass = 1;
      this.massdelaywait = Math.floor(Math.random() * 48) + 2;
      this.massdelay = Math.floor(Math.random() * 100) + 300;
    }
    if (this.mass === 1) this.delaytimer += 1;
  }

  orbit(centerX, centerY, radius, speed, phase) {
    const now = performance.now() / 1000.0;
    const angle = now * speed + phase;
    const x = centerX + radius * Math.cos(angle);
    const y = centerY + radius * Math.sin(angle);
    this.location = new Vec2(x, y);
  }

  attract(boid) {
    const force = Vec2.sub(this.location, boid.location);
    let d = force.mag();

    // If outside influence radius, return zero force
    if (d > this.maxInfluenceRadius) return new Vec2(0, 0);

    // Constrain distance
    d = Math.max(this.minDistance, Math.min(d, this.maxInfluenceRadius));
    force.normalize();

    // Calculate gravitational force magnitude
    let strength = (this.G * this.mass * boid.mass) / (d * d);

    // If repulsor, reverse direction
    if (this.isRepulsor) {
      force.mult(-1);
    }

    force.mult(strength);

    // Add vortex/rotational component if enabled
    if (this.hasVortex && strength > 0.001) {
      const tangent = force.ortho();
      const tangentMag = tangent.mag();
      if (tangentMag > 0.001) {
        tangent.normalize();
        tangent.mult(strength * this.vortexStrength);
        force.add(tangent);
      }
    }

    return force;
  }
}
