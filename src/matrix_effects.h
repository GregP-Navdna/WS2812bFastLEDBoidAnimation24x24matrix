#ifndef MATRIX_EFFECTS_H
#define MATRIX_EFFECTS_H

#include <Arduino.h>
#include <FastLED.h>
#include <math.h>

class MatrixEffects {
private:
    // Screen dimensions
    const uint8_t rows;
    const uint8_t cols;
    // Function to map 2D coordinates to LED strip index
    uint16_t (*xyFunc)(uint8_t, uint8_t);
    
    // Ripple effect parameters
    struct Ripple {
        int8_t x;
        int8_t y;
        uint8_t radius;
        uint8_t maxRadius;
        uint8_t color;
        uint8_t intensity;
        bool active;
    };
    static const uint8_t MAX_RIPPLES = 3;
    Ripple ripples[MAX_RIPPLES];

    // Color wash parameters
    uint8_t washHue = 0;
    uint8_t washDirection = 0; // 0: horizontal, 1: vertical, 2: diagonal
    uint8_t washSpeed = 5;
    uint8_t washDensity = 20;
    bool washActive = false;

    // Starfield parameters
    static const uint8_t MAX_STARS = 15;
    struct Star {
        int8_t x;
        int8_t y;
        uint8_t brightness;
        uint8_t maxBrightness;
        int8_t fadeStep;
    };
    Star stars[MAX_STARS];
    bool starfieldActive = false;

    // Screen shake parameters
    bool shakeActive = false;
    uint8_t shakeDuration = 0;
    uint8_t shakeIntensity = 0;
    int8_t shakeOffsetX = 0;
    int8_t shakeOffsetY = 0;
    unsigned long lastShakeUpdate = 0;
    
    // Perlin noise parameters
    bool perlinActive = false;
    float perlinScale = 0.05;
    float perlinTimeScale = 0.02;
    float perlinTime = 0.1;
    uint8_t perlinHue = 0;
    uint8_t perlinHueSpeed = 2;
    uint8_t perlinBrightness = 128;

public:
    MatrixEffects(uint8_t rows, uint8_t cols, uint16_t (*xyFunc)(uint8_t, uint8_t))
        : rows(rows), cols(cols), xyFunc(xyFunc) {
        // Initialize ripples
        for (uint8_t i = 0; i < MAX_RIPPLES; i++) {
            ripples[i].active = false;
        }
        
        // Initialize stars
        for (uint8_t i = 0; i < MAX_STARS; i++) {
            stars[i].brightness = 0;
            stars[i].fadeStep = 0;
        }
    }

    // Start a new ripple at a random location
    void startRipple() {
        // Find an inactive ripple slot
        for (uint8_t i = 0; i < MAX_RIPPLES; i++) {
            if (!ripples[i].active) {
                ripples[i].x = random8(cols);
                ripples[i].y = random8(rows);
                ripples[i].radius = 0;
                ripples[i].maxRadius = random8(4, min(rows, cols) / 2);
                ripples[i].color = random8();
                ripples[i].intensity = random8(150, 255);
                ripples[i].active = true;
                break;
            }
        }
    }

    // Update and draw all active ripples
    void updateRipples(CRGB* leds) {
        for (uint8_t i = 0; i < MAX_RIPPLES; i++) {
            if (ripples[i].active) {
                // Draw the ripple
                drawRipple(leds, ripples[i]);
                
                // Expand the ripple
                ripples[i].radius++;
                
                // Decrease intensity as the ripple expands
                ripples[i].intensity = max(0, ripples[i].intensity - 15);
                
                // Deactivate if the ripple has reached its maximum radius or faded out
                if (ripples[i].radius >= ripples[i].maxRadius || ripples[i].intensity == 0) {
                    ripples[i].active = false;
                }
            }
        }
    }

    // Draw a single ripple
    void drawRipple(CRGB* leds, const Ripple& ripple) {
        // Draw a circle with the current radius
        for (int16_t y = 0; y < rows; y++) {
            for (int16_t x = 0; x < cols; x++) {
                // Calculate distance from ripple center
                float distance = sqrt(sq(x - ripple.x) + sq(y - ripple.y));
                
                // Check if this pixel is on the ripple edge (with some thickness)
                if (abs(distance - ripple.radius) < 1.0) {
                    // Calculate fade based on distance from exact radius
                    float fade = 1.0 - abs(distance - ripple.radius);
                    
                    // Get the pixel index
                    uint16_t pixelIndex = xyFunc(x, y);
                    
                    // Add the ripple color to the existing pixel
                    CRGB rippleColor = CHSV(ripple.color, 240, ripple.intensity * fade);
                    leds[pixelIndex] += rippleColor;
                }
            }
        }
    }

    // Start a color wash effect
    void startColorWash(uint8_t direction = 255, uint8_t speed = 255, uint8_t density = 255) {
        washActive = true;
        washHue = random8();
        
        // Use provided parameters or set random ones
        if (direction == 255) {
            washDirection = random8(3); // 0: horizontal, 1: vertical, 2: diagonal
        } else {
            washDirection = constrain(direction, 0, 2);
        }
        
        if (speed == 255) {
            washSpeed = random8(3, 10);
        } else {
            washSpeed = speed;
        }
        
        if (density == 255) {
            washDensity = random8(15, 30);
        } else {
            washDensity = density;
        }
    }

    // Stop color wash effect
    void stopColorWash() {
        washActive = false;
    }

    // Update and draw color wash effect
    void updateColorWash(CRGB* leds) {
        if (!washActive) return;
        
        // Slowly change the base hue
        EVERY_N_MILLISECONDS(50) {
            washHue++;
        }
        
        // Apply the wash pattern based on direction
        for (uint8_t y = 0; y < rows; y++) {
            for (uint8_t x = 0; x < cols; x++) {
                uint8_t hueOffset;
                
                switch (washDirection) {
                    case 0: // Horizontal
                        hueOffset = x * washDensity;
                        break;
                    case 1: // Vertical
                        hueOffset = y * washDensity;
                        break;
                    case 2: // Diagonal
                        hueOffset = (x + y) * washDensity / 2;
                        break;
                }
                
                // Get the pixel index
                uint16_t pixelIndex = xyFunc(x, y);
                
                // Apply a subtle color wash
                leds[pixelIndex] += CHSV(washHue + hueOffset, 200, 40);
            }
        }
    }

    // Initialize starfield
    void startStarfield() {
        starfieldActive = true;
        // Initialize stars
        for (uint8_t i = 0; i < MAX_STARS; i++) {
            if (random8() < 200) { // 80% chance to activate a star initially
                stars[i].x = random8(cols);
                stars[i].y = random8(rows);
                stars[i].brightness = random8(20, 80);
                stars[i].maxBrightness = random8(100, 255);
                // Positive fadeStep means the star is getting brighter
                stars[i].fadeStep = random8(1, 3);
            }
        }
    }

    // Stop starfield
    void stopStarfield() {
        starfieldActive = false;
    }

    // Update and draw starfield
    void updateStarfield(CRGB* leds) {
        if (!starfieldActive) return;
        
        // Update and draw each star
        for (uint8_t i = 0; i < MAX_STARS; i++) {
            // Skip stars with zero brightness and fade step
            if (stars[i].brightness == 0 && stars[i].fadeStep == 0) {
                // 5% chance to activate a new star
                if (random8() < 12) {
                    stars[i].x = random8(cols);
                    stars[i].y = random8(rows);
                    stars[i].brightness = 10;
                    stars[i].maxBrightness = random8(100, 255);
                    stars[i].fadeStep = random8(1, 3);
                }
                continue;
            }
            
            // Update star brightness
            stars[i].brightness = constrain(stars[i].brightness + stars[i].fadeStep, 0, stars[i].maxBrightness);
            
            // If star has reached max brightness, start fading out
            if (stars[i].brightness >= stars[i].maxBrightness) {
                // Change to negative fade step (fading out)
                stars[i].fadeStep = -random8(1, 2);
            }
            
            // If star has faded out completely, reset it
            if (stars[i].brightness <= 0) {
                stars[i].brightness = 0;
                stars[i].fadeStep = 0;
            }
            
            // Draw the star
            uint16_t pixelIndex = xyFunc(stars[i].x, stars[i].y);
            // Stars are white/blue/yellow
            uint8_t starHue = random8(3);
            switch (starHue) {
                case 0:
                    leds[pixelIndex] += CRGB(stars[i].brightness, stars[i].brightness, stars[i].brightness); // White
                    break;
                case 1:
                    leds[pixelIndex] += CRGB(stars[i].brightness/2, stars[i].brightness/2, stars[i].brightness); // Blueish
                    break;
                case 2:
                    leds[pixelIndex] += CRGB(stars[i].brightness, stars[i].brightness, stars[i].brightness/2); // Yellowish
                    break;
            }
        }
    }

    // Start screen shake effect
    void startScreenShake(uint8_t duration = 10, uint8_t intensity = 2) {
        shakeActive = true;
        shakeDuration = duration;
        shakeIntensity = intensity;
        lastShakeUpdate = millis();
    }

    // Update screen shake effect
    bool updateScreenShake() {
        if (!shakeActive) return false;
        
        // Check if shake effect should end
        if (shakeDuration == 0) {
            shakeActive = false;
            shakeOffsetX = 0;
            shakeOffsetY = 0;
            return false;
        }
        
        // Update shake position every 50ms
        if (millis() - lastShakeUpdate > 50) {
            // Decrease duration
            shakeDuration--;
            
            // Calculate new random offsets
            shakeOffsetX = random(-shakeIntensity, shakeIntensity + 1);
            shakeOffsetY = random(-shakeIntensity, shakeIntensity + 1);
            
            lastShakeUpdate = millis();
        }
        
        return true;
    }
    
    // Perlin noise functions
    // Improved Perlin noise implementation for performance and aesthetics
    float fade(float t) { return t * t * t * (t * (t * 6 - 15) + 10); }
    
    float lerp(float a, float b, float t) { return a + t * (b - a); }
    
    float grad(int hash, float x, float y, float z) {
        int h = hash & 15;                      // Convert lower 4 bits of hash code
        float u = h < 8 ? x : y;                // into 12 gradient directions
        float v = h < 4 ? y : h == 12 || h == 14 ? x : z;
        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }
    
    // Fast permutation table - pre-computed to avoid modulo operations
    const uint8_t permutation[256] = {
        151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225,
        140, 36, 103, 30, 69, 142, 8, 99, 37, 240, 21, 10, 23, 190, 6, 148,
        247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32,
        57, 177, 33, 88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175,
        74, 165, 71, 134, 139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122,
        60, 211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54,
        65, 25, 63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169,
        200, 196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3, 64,
        52, 217, 226, 250, 124, 123, 5, 202, 38, 147, 118, 126, 255, 82, 85, 212,
        207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42, 223, 183, 170, 213,
        119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9,
        129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232, 178, 185, 112, 104,
        218, 246, 97, 228, 251, 34, 242, 193, 238, 210, 144, 12, 191, 179, 162, 241,
        81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157,
        184, 84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93,
        222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180
    };
    
    float perlinNoise(float x, float y, float z) {
        // Find unit grid cell containing point
        int X = (int)floor(x) & 255;
        int Y = (int)floor(y) & 255;
        int Z = (int)floor(z) & 255;
        
        // Find relative x, y, z of point in cell
        x -= floor(x);
        y -= floor(y);
        z -= floor(z);
        
        // Compute fade curves for each of x, y, z
        float u = fade(x);
        float v = fade(y);
        float w = fade(z);
        
        // Hash coordinates of the 8 cube corners
        int A = permutation[X] + Y;
        int AA = permutation[A] + Z;
        int AB = permutation[A + 1] + Z;
        int B = permutation[X + 1] + Y;
        int BA = permutation[B] + Z;
        int BB = permutation[B + 1] + Z;
        
        // Add blended results from 8 corners of cube
        float result = lerp(lerp(lerp(grad(permutation[AA], x, y, z),
                                      grad(permutation[BA], x-1, y, z), u),
                                 lerp(grad(permutation[AB], x, y-1, z),
                                      grad(permutation[BB], x-1, y-1, z), u), v),
                            lerp(lerp(grad(permutation[AA+1], x, y, z-1),
                                      grad(permutation[BA+1], x-1, y, z-1), u),
                                 lerp(grad(permutation[AB+1], x, y-1, z-1),
                                      grad(permutation[BB+1], x-1, y-1, z-1), u), v), w);
        return (result + 1.0) / 2.0; // Transform from -1..1 to 0..1
    }

    // Get current shake offsets
    void getShakeOffsets(int8_t& offsetX, int8_t& offsetY) {
        offsetX = shakeOffsetX;
        offsetY = shakeOffsetY;
    }

    // Start a Perlin noise effect
    void startPerlinNoise(float scale = 0.15, uint8_t brightness = 128) {
        perlinActive = true;
        perlinScale = scale;
        perlinTime = 0.0;
        perlinHue = random8();
        perlinBrightness = brightness;
        perlinHueSpeed = random8(1, 5);
        perlinTimeScale = random8(1, 5) / 100.0;
    }
    
    // Stop Perlin noise effect
    void stopPerlinNoise() {
        perlinActive = false;
    }
    
    // Update and draw Perlin noise effect
    void updatePerlinNoise(CRGB* leds) {
        if (!perlinActive) return;
        
        // Increment time value for animation
        perlinTime += perlinTimeScale;
        
        // Slowly change the base hue
        EVERY_N_MILLISECONDS(50) {
            perlinHue += perlinHueSpeed;
        }
        
        // Apply the Perlin noise pattern
        for (uint8_t y = 0; y < rows; y++) {
            for (uint8_t x = 0; x < cols; x++) {
                // Calculate noise value at this coordinate
                float noiseValue = perlinNoise(
                    x * perlinScale, 
                    y * perlinScale, 
                    perlinTime
                );
                
                // Map noise value to brightness (avoiding complete blackness)
                uint8_t brightness = map(noiseValue * 255, 0, 255, 20, perlinBrightness);
                
                // Create a color based on position and noise value
                uint8_t hue = perlinHue + map(noiseValue * 255, 0, 255, 0, 64);
                
                // Get the pixel index
                uint16_t pixelIndex = xyFunc(x, y);
                
                // Apply color based on noise
                leds[pixelIndex] += CHSV(hue, 240, brightness);
            }
        }
    }
    
    // Main update function - applies all active effects
    void update(CRGB* leds) {
        // Update all active effects
        updateRipples(leds);
        updateColorWash(leds);
        updateStarfield(leds);
        updatePerlinNoise(leds);
        updateScreenShake();
    }

    // Randomly trigger a matrix effect
    void triggerRandomEffect() {
        uint8_t effect = random8(5);
        
        switch (effect) {
            case 0:
                // Start a ripple effect
                startRipple();
                break;
                
            case 1:
                // Toggle color wash
                if (washActive) {
                    stopColorWash();
                } else {
                    startColorWash();
                }
                break;
                
            case 2:
                // Toggle starfield
                if (starfieldActive) {
                    stopStarfield();
                } else {
                    startStarfield();
                }
                break;
                
            case 3:
                // Screen shake
                startScreenShake();
                break;
                
            case 4:
                // Toggle Perlin noise
                if (perlinActive) {
                    stopPerlinNoise();
                } else {
                    startPerlinNoise();
                }
                break;
        }
    }
};

#endif // MATRIX_EFFECTS_H
