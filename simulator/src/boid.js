/**
 * Boid class - Ported from boid.h
 * Particle with flocking behavior (separation, alignment, cohesion)
 */
import { Vec2 } from './vec2.js';

export class Boid {
  constructor(x = 0, y = 0) {
    this.acceleration = new Vec2(0, 0);
    this.velocity = new Vec2(Boid.randomf(), Boid.randomf());
    this.location = new Vec2(x, y);
    this.maxspeed = 1.2;
    this.maxforce = 0.28;
    this.brightness = 255;
    this.mass = 1.0 + Math.random() * 2.0;
    this.enabled = true;
    this.hue = Math.floor(Math.random() * 145) + 5;
    this.desiredseparation = 1.0;
    this.neighbordist = 2.0;
    this.colorIndex = 0;
  }

  static randomf() {
    return (Math.random() - 0.5);
  }

  applyForce(force) {
    this.acceleration.add(force);
  }

  seek(target) {
    const desired = Vec2.sub(target, this.location);
    desired.normalize();
    desired.mult(this.maxspeed);
    const steer = Vec2.sub(desired, this.velocity);
    steer.limit(this.maxforce);
    return steer;
  }

  // Fused flocking: single pass over neighbors for separate + align + cohesion
  // This is the key optimization over the C++ version which does 3 passes
  flock(neighbors) {
    const sepSteer = new Vec2(0, 0);
    const aliSum = new Vec2(0, 0);
    const cohSum = new Vec2(0, 0);
    let sepCount = 0;
    let aliCount = 0;
    let cohCount = 0;

    for (let i = 0; i < neighbors.length; i++) {
      const other = neighbors[i];
      if (other === this || !other.enabled) continue;

      const dx = other.location.x - this.location.x;
      const dy = other.location.y - this.location.y;
      const d = Math.sqrt(dx * dx + dy * dy);

      if (d <= 0) continue;

      // Separation: steer away from close neighbors
      if (d < this.desiredseparation) {
        const diffX = (this.location.x - other.location.x) / d / d;
        const diffY = (this.location.y - other.location.y) / d / d;
        sepSteer.x += diffX;
        sepSteer.y += diffY;
        sepCount++;
      }

      // Alignment + Cohesion: use neighbor distance
      if (d < this.neighbordist) {
        aliSum.add(other.velocity);
        aliCount++;
        cohSum.add(other.location);
        cohCount++;
      }
    }

    // Finalize separation
    if (sepCount > 0) {
      sepSteer.div(sepCount);
      if (sepSteer.mag() > 0) {
        sepSteer.normalize();
        sepSteer.mult(this.maxspeed);
        sepSteer.sub(this.velocity);
        sepSteer.limit(this.maxforce);
      }
    }

    // Finalize alignment
    let aliSteer = new Vec2(0, 0);
    if (aliCount > 0) {
      aliSum.div(aliCount);
      aliSum.normalize();
      aliSum.mult(this.maxspeed);
      aliSteer = Vec2.sub(aliSum, this.velocity);
      aliSteer.limit(this.maxforce);
    }

    // Finalize cohesion
    let cohSteer = new Vec2(0, 0);
    if (cohCount > 0) {
      cohSum.div(cohCount);
      cohSteer = this.seek(cohSum);
    }

    // Apply weighted forces (matches C++ weights)
    sepSteer.mult(3.5);
    aliSteer.mult(1.0);
    cohSteer.mult(1.0);

    this.applyForce(sepSteer);
    this.applyForce(aliSteer);
    this.applyForce(cohSteer);
  }

  update(grid) {
    // Get neighbors from spatial grid
    const neighbors = grid.getNeighbors(
      this.location.x, this.location.y, this.neighbordist
    );

    // Single-pass flocking
    this.flock(neighbors);

    // Update velocity
    this.velocity.add(this.acceleration);
    this.velocity.limit(this.maxspeed);
    this.location.add(this.velocity);
    // Reset acceleration
    this.acceleration.mult(0);

    // Update hue for color variation
    this.hue = (this.hue + Math.floor(Math.random() * 9) + 1) % 255;
  }

  wrapAroundBorders(width, height) {
    if (this.location.x < 0) this.location.x = width - 1;
    if (this.location.y < 0) this.location.y = height - 1;
    if (this.location.x >= width) this.location.x = 0;
    if (this.location.y >= height) this.location.y = 0;
  }

  avoidBorders(width, height, margin) {
    let desired = this.velocity.copy();

    if (this.location.x < margin) {
      desired = new Vec2(this.maxspeed, this.velocity.y);
    } else if (this.location.x > width - margin) {
      desired = new Vec2(-this.maxspeed, this.velocity.y);
    }

    if (this.location.y < margin) {
      desired = new Vec2(this.velocity.x, this.maxspeed);
    } else if (this.location.y > height - margin) {
      desired = new Vec2(this.velocity.x, -this.maxspeed);
    }

    if (desired.x !== this.velocity.x || desired.y !== this.velocity.y) {
      const steer = Vec2.sub(desired, this.velocity);
      steer.limit(this.maxforce);
      this.applyForce(steer);
    }
  }
}
