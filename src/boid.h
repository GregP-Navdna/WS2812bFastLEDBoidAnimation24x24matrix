#ifndef BOID_H
#define BOID_H

#include <Arduino.h>
#include "vec2.h"
#include "simd_utils.h"

// Boid class to make the particles interact with each other
class Boid {
  public:
    PVector location;
    PVector velocity;
    PVector acceleration;
    float maxforce; // Maximum steering force
    float maxspeed;
    uint8_t brightness;
    int hue;
    float desiredseparation = 3; // this this is the variable that keeps the boids at a certain distance from eachother. the higher the number the more space between. keep this lower than the neighbor distance otherwise the boids will probably not move.
    float neighbordist = 8;      // this will determine the length that a boid will interact with another boid. the lower the number the more isolated the boid will be. the higher the number the boid will interact with another boid at a further distance.
    byte colorIndex = 0;
    float mass;
    boolean enabled = true;

    Boid() {}

    Boid(float x, float y) {
      acceleration = PVector(0, 0);
      velocity = PVector(randomf(), randomf());
      location = PVector(x, y);
      maxspeed = 1.8;
      maxforce = 0.28;
      brightness = 255;
      mass = random(1.0, 3.0);
      hue = random(10, 255);
    }

    static float randomf() {
      return mapfloat(random(0, 255), 0, 255, -.5, .5);
    }

    static float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    void run(Boid boids[], uint8_t boidCount) {
      flock(boids, boidCount);
    }
    
    // Method to update location
    void update(Boid boids[], uint8_t boidCount) {
      flock(boids, boidCount);
      velocity += acceleration;
      velocity.limit(maxspeed);
      location += velocity;
      acceleration *= 0;
    }

    void applyForce(PVector force) {
      // We could add mass here if we want A = F / M
      acceleration += force;
    }

    void repelForce(PVector obstacle, float radius) {
      // Force that drives boid away from obstacle.
      PVector futPos = location + velocity; // Calculate future position for more effective behavior.
      PVector dist = obstacle - futPos;
      float d = dist.mag();

      if (d <= radius) {
        PVector repelVec = location - obstacle;
        repelVec.normalize();
        if (d != 0) { // Avoid division by zero
          float scale = 1.0 / d; // Closer means stronger repulsion
          repelVec *= (scale * maxspeed);
          repelVec -= velocity;
          repelVec.limit(maxforce * 2); // Stronger than regular forces
        }
        applyForce(repelVec);
      }
    }

    void flock(Boid boids[], uint8_t boidCount) {
      PVector sep = separate(boids, boidCount);
      PVector ali = align(boids, boidCount);
      PVector coh = cohesion(boids, boidCount);

      // Arbitrarily weight these forces
      sep *= 3.5;
      ali *= 1.0;
      coh *= 1.0;

      // Add the force vectors to acceleration
      applyForce(sep);
      applyForce(ali);
      applyForce(coh);
    }

    // Separation
    // Method checks for nearby boids and steers away
    PVector separate(Boid boids[], uint8_t boidCount) {
      PVector steer(0, 0);
      int count = 0;

      // For every boid in the system, check if it's too close
      for (int i = 0; i < boidCount; i++) {
        if (!boids[i].enabled)
          continue;

        float d = location.dist(boids[i].location);

        // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
        if ((d > 0) && (d < desiredseparation)) {
          // Calculate vector pointing away from neighbor
          PVector diff = location - boids[i].location;
          diff.normalize();
          diff /= d;        // Weight by distance
          steer += diff;
          count++;            // Keep track of how many
        }
      }
      // Average -- divide by how many
      if (count > 0) {
        steer /= (float)count;
      }

      // As long as the vector is greater than 0
      if (steer.magSq() > 0) {
        // Implement Reynolds: Steering = Desired - Velocity
        steer.normalize();
        steer *= maxspeed;
        steer -= velocity;
        steer.limit(maxforce);
      }
      return steer;
    }

    // Alignment
    // For every nearby boid in the system, calculate the average velocity
    PVector align(Boid boids[], uint8_t boidCount) {
      PVector sum(0, 0);
      int count = 0;
      for (int i = 0; i < boidCount; i++) {
        if (!boids[i].enabled)
          continue;

        float d = location.dist(boids[i].location);
        if ((d > 0) && (d < neighbordist)) {
          sum += boids[i].velocity;
          count++;
        }
      }
      if (count > 0) {
        sum /= (float)count;
        sum.normalize();
        sum *= maxspeed;
        PVector steer = sum - velocity;
        steer.limit(maxforce);
        return steer;
      } else {
        return PVector(0, 0);
      }
    }

    // Cohesion
    // For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
    PVector cohesion(Boid boids[], uint8_t boidCount) {
      PVector sum(0, 0);   // Start with empty vector to accumulate all locations
      int count = 0;
      for (int i = 0; i < boidCount; i++) {
        if (!boids[i].enabled)
          continue;

        float d = location.dist(boids[i].location);
        if ((d > 0) && (d < neighbordist)) {
          sum += boids[i].location; // Add location
          count++;
        }
      }
      if (count > 0) {
        sum /= count;
        return seek(sum);  // Steer towards the location
      } else {
        return PVector(0, 0);
      }
    }

    // A method that calculates and applies a steering force towards a target
    // STEER = DESIRED MINUS VELOCITY
    PVector seek(PVector target) {
      PVector desired = target - location;  // A vector pointing from the location to the target

      // Normalize desired and scale to maximum speed
      if (desired.magSq() > 0) {
        desired.normalize();
        desired *= maxspeed;
        
        // Steering = Desired minus Velocity
        PVector steer = desired - velocity;
        steer.limit(maxforce);  // Limit to maximum steering force
        return steer;
      } else {
        return PVector(0, 0);
      }
    }

    // Wraparound
    void borders(uint8_t virtualWidth, uint8_t virtualHeight) {
      if (location.x < 0) location.x = virtualWidth;
      if (location.y < 0) location.y = virtualHeight;
      if (location.x > virtualWidth) location.x = 0;
      if (location.y > virtualHeight) location.y = 0;
    }
    
    // Add wrapAroundBorders method that was in the original code
    void wrapAroundBorders(uint8_t VIRTUAL_ROWS, uint8_t VIRTUAL_COLS) {
      if (location.x < 0) location.x += VIRTUAL_ROWS;
      if (location.y < 0) location.y += VIRTUAL_COLS;
      if (location.x >= VIRTUAL_ROWS) location.x -= VIRTUAL_ROWS;
      if (location.y >= VIRTUAL_COLS) location.y -= VIRTUAL_COLS;
    }
    
    // Add avoidBorders method that was in the original code
    void avoidBorders(uint8_t ROWS, uint8_t COLS) {
      PVector desired = velocity;

      if (location.x < 8)
        desired = PVector(maxspeed, velocity.y);
      if (location.x >= ROWS - 8)
        desired = PVector(-maxspeed, velocity.y);
      if (location.y < 8)
        desired = PVector(velocity.x, maxspeed);
      if (location.y >= COLS - 8)
        desired = PVector(velocity.x, -maxspeed);

      if (desired != velocity) {
        PVector steer = desired - velocity;
        steer.limit(maxforce);
        applyForce(steer);
      }

      if (location.x < 0)
        location.x = 0;
      if (location.y < 0)
        location.y = 0;
      if (location.x >= ROWS)
        location.x = ROWS - 1;
      if (location.y >= COLS)
        location.y = COLS - 1;
    }
};

#endif // BOID_H
