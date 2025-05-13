#ifndef BOID_H
#define BOID_H

#include <Arduino.h>
#include "vec2.h"
#include "simd_utils.h"
#include "algorithm"

// Forward declaration
class SpatialGrid;

// Boid class to make the particles interact with each other
class Boid {
public:
    PVector location;
    PVector velocity;
    PVector acceleration;
    float maxforce;
    float maxspeed;
    uint8_t brightness;
    int hue;
    float desiredseparation = 1.0F; // this this is the variable that keeps the boids at a certain distance from eachother. the higher the number the more space between. keep this lower than the neighbor distance otherwise the boids will probably not move.
    float neighbordist = 2.0F;      // this will determine the length that a boid will interact with another boid. the lower the number the more isolated the boid will be. the higher the number the boid will interact with another boid at a further distance.
    byte colorIndex = 0;
    float mass;
    boolean enabled = true;
    
    // Utility functions for random initialization
    static float randomf() {
      return mapfloat(random(0, 255), 0, 255, -.5, .5);
    }

    static float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    
    Boid() {}

    Boid(float x, float y) {
      acceleration = PVector(0.0F, 0.0F);
      velocity = PVector(randomf(), randomf());
      location = PVector(x, y);
      maxspeed = 1.2;
      maxforce = 0.28;
      brightness = 255;
      mass = random(1.0, 3.0);
      enabled = true;
      hue = random(5,150);
    }
    
    void applyForce(PVector force) {
      acceleration += force;
    }
    
    // New simpler method to get nearest neighbors the traditional way
    // This is here to maintain compatibility with code that doesn't use spatial grid
    PVector separate(Boid* boids, int boidsCount) {
      PVector steer = PVector(0, 0);
      int count = 0;
      // For every boid in the system, check if it's too close
      for (int i = 0; i < boidsCount; i++) {
        Boid* other = &boids[i];
        if (other != this && other->enabled) {
          float d = location.dist(other->location);
          // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
          if ((d > 0) && (d < desiredseparation)) {
            // Calculate vector pointing away from neighbor
            PVector diff = location - other->location;
            diff.normalize();
            diff /= d;        // Weight by distance
            steer += diff;
            count++;            // Keep track of how many
          }
        }
      }
      // Average -- divide by how many
      if (count > 0) {
        steer /= (float)count;
      }

      // As long as the vector is greater than 0
      if (steer.mag() > 0) {
        // Implement Reynolds: Steering = Desired - Velocity
        steer.normalize();
        steer *= maxspeed;
        steer -= velocity;
        steer.limit(maxforce);
      }
      return steer;
    }
    
    PVector seek(PVector target) {
      PVector desired = target - location;  // A vector pointing from the location to the target
      // Scale to maximum speed
      desired.normalize();
      desired *= maxspeed;

      // Steering = Desired minus Velocity
      PVector steer = desired - velocity;
      steer.limit(maxforce);  // Limit to maximum steering force
      return steer;
    }
    
    // Alignment
    // For every nearby boid in the system, calculate the average velocity
    PVector align(Boid* boids, int boidsCount) {
      PVector sum = PVector(0, 0);
      int count = 0;
      for (int i = 0; i < boidsCount; i++) {
        Boid* other = &boids[i];
        if (other != this && other->enabled) {
          float d = location.dist(other->location);
          if ((d > 0) && (d < neighbordist)) {
            sum += other->velocity;
            count++;
          }
        }
      }
      if (count > 0) {
        sum /= (float)count;
        // First two lines of code below could be condensed with new PVector setMag() method
        // Not using this method until Processing.js catches up
        sum.normalize();
        sum *= maxspeed;
        PVector steer = sum - velocity;
        steer.limit(maxforce);
        return steer;
      } 
      else {
        return PVector(0, 0);
      }
    }

    // Cohesion
    // For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
    PVector cohesion(Boid* boids, int boidsCount) {
      PVector sum = PVector(0, 0);   // Start with empty vector to accumulate all locations
      int count = 0;
      for (int i = 0; i < boidsCount; i++) {
        Boid* other = &boids[i];
        if (other != this && other->enabled) {
          float d = location.dist(other->location);
          if ((d > 0) && (d < neighbordist)) {
            sum += other->location; // Add location
            count++;
          }
        }
      }
      if (count > 0) {
        sum /= count;
        return seek(sum);  // Steer towards the location
      } 
      else {
        return PVector(0, 0);
      }
    }

    // Keep the original update method for compatibility
    void update(Boid* boids, int boidsCount) {
      PVector sep = separate(boids, boidsCount);
      PVector ali = align(boids, boidsCount);
      PVector coh = cohesion(boids, boidsCount);
      
      // Arbitrarily weight these forces
      sep *= 3.5;
      ali *= 1.0;
      coh *= 1.0;
      
      // Add the force vectors to acceleration
      applyForce(sep);
      applyForce(ali);
      applyForce(coh);
      
      // Update velocity
      velocity += acceleration;
      // Limit speed
      velocity.limit(maxspeed);
      location += velocity;
      // Reset acceleration to 0 each cycle
      acceleration *= 0;
      
      // Update hue value for color variation
      hue = (hue + random8(1, 10)) % 255;
    }

    // Method to wrap boid position around borders
    void wrapAroundBorders(float width, float height) {
      if (location.x < 0) location.x = width - 1;
      if (location.y < 0) location.y = height - 1;
      if (location.x >= width) location.x = 0;
      if (location.y >= height) location.y = 0;
    }
    
    // Method to avoid borders by turning away
    void avoidBorders(float width, float height, float margin) {
      PVector desired = velocity;

      if (location.x < margin) {
        desired = PVector(maxspeed, velocity.y);
      } 
      else if (location.x > width - margin) {
        desired = PVector(-maxspeed, velocity.y);
      }

      if (location.y < margin) {
        desired = PVector(velocity.x, maxspeed);
      } 
      else if (location.y > height - margin) {
        desired = PVector(velocity.x, -maxspeed);
      }

      if (desired != velocity) {
        PVector steer = desired - velocity;
        steer.limit(maxforce);
        applyForce(steer);
      }
    }
    
    // Keep position within borders
    void constrainToBorders(float width, float height) {
      if (location.x < 0)
        location.x = 0;
      if (location.y < 0)
        location.y = 0;
      if (location.x >= width)
        location.x = width - 1;
      if (location.y >= height)
        location.y = height - 1;
    }

    // Optimized version of separate that uses spatial partitioning
    PVector separate(std::vector<void*>& neighbors) {
        PVector steer = PVector(0.0F, 0.0F);
        int count = 0;
        
        // For every boid in the system, check if it's too close
        for (void* obj : neighbors) {
            Boid* other = static_cast<Boid*>(obj);
            if (other != this && other->enabled) {
                float d = location.dist(other->location);
                // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
                if ((d > 0) && (d < desiredseparation)) {
                    // Calculate vector pointing away from neighbor
                    PVector diff = location - other->location;
                    diff.normalize();
                    diff /= d;        // Weight by distance
                    steer += diff;
                    count++;            // Keep track of how many
                }
            }
        }
        // Average -- divide by how many
        if (count > 0) {
            steer /= (float)count;
        }

        // As long as the vector is greater than 0
        if (steer.mag() > 0) {
            // Implement Reynolds: Steering = Desired - Velocity
            steer.normalize();
            steer *= maxspeed;
            steer -= velocity;
            steer.limit(maxforce);
        }
        return steer;
    }

    // Optimized version of align that uses spatial partitioning
    PVector align(std::vector<void*>& neighbors) {
        PVector sum = PVector(0.0F, 0.0F);
        int count = 0;
        
        for (void* obj : neighbors) {
            Boid* other = static_cast<Boid*>(obj);
            if (other != this && other->enabled) {
                float d = location.dist(other->location);
                if ((d > 0) && (d < neighbordist)) {
                    sum += other->velocity;
                    count++;
                }
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
            return PVector(0.0F, 0.0F);
        }
    }

    // Optimized version of cohesion that uses spatial partitioning
    PVector cohesion(std::vector<void*>& neighbors) {
        PVector sum = PVector(0.0F, 0.0F);
        int count = 0;
        
        for (void* obj : neighbors) {
            Boid* other = static_cast<Boid*>(obj);
            if (other != this && other->enabled) {
                float d = location.dist(other->location);
                if ((d > 0) && (d < neighbordist)) {
                    sum += other->location; // Add location
                    count++;
                }
            }
        }
        
        if (count > 0) {
            sum /= (float)count;
            return seek(sum);  // Steer towards the position
        } else {
            return PVector(0.0F, 0.0F);
        }
    }
    
    // Update with spatial grid optimization
    void update(SpatialGrid& grid) {
        // Get only neighboring boids from the spatial grid
        std::vector<void*> neighbors = grid.getNeighbors(location.x, location.y, neighbordist);
        
        // Calculate steering forces
        PVector sep = separate(neighbors);
        PVector ali = align(neighbors);
        PVector coh = cohesion(neighbors);
        
        // Arbitrarily weight these forces
        sep *= 3.5;
        ali *= 1.0;
        coh *= 1.0;
        
        // Add the force vectors to acceleration
        applyForce(sep);
        applyForce(ali);
        applyForce(coh);
        
        // Update velocity
        velocity += acceleration;
        // Limit speed
        velocity.limit(maxspeed);
        location += velocity;
        // Reset acceleration to 0 each cycle
        acceleration *= 0;
        
        // Update hue value for color variation
        hue = (hue + random8(1, 10)) % 255;
    }
};

#endif // BOID_H
