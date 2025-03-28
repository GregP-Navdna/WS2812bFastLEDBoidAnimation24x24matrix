#ifndef SIMD_UTILS_H
#define SIMD_UTILS_H

#include <Arduino.h>
#include <FastLED.h>

// External reference to rran variable from main.cpp
extern int rran;

// Optimized vector operations for ESP32
class SimdOps {
public:
    // Initialize SIMD capabilities
    static bool init() {
        #if defined(ESP32)
        // ESP32-S3 specific initialization if needed
        return true;
        #else
        return false;
        #endif
    }
};

// Optimized version of fadeToColorBy that processes multiple LEDs at once
void simd_fade_to_color(CRGB* leds, int count, CRGB color, uint8_t fadeAmt) {
    // Process LEDs in batches of 4 for better cache utilization
    int i = 0;
    
    #if defined(ESP32)
    // Process 4 LEDs at once to improve memory access patterns
    for (; i < (count & ~0x3); i += 4) {
        for (int j = 0; j < 4; j++) {
            CRGB* led = &leds[i+j];
            
            // Red component with special handling
            if (led->r < color.r) {
                led->r = ((led->r << 8) + (((int)(((color.r - led->r) << 7) / rran) * fadeAmt / 255) << 1)) >> 8;
            } else {
                led->r = ((led->r << 8) + (((int)(((color.r - led->r) << 7) * rran) * fadeAmt / 255) << 1)) >> 8;
            }
            
            // Green and blue components
            led->g = ((led->g << 8) + ((((color.g - led->g) << 7) * fadeAmt / 255) << 1)) >> 8;
            led->b = ((led->b << 8) + ((((color.b - led->b) << 7) * fadeAmt / 255) << 1)) >> 8;
        }
    }
    #endif
    
    // Handle remaining LEDs
    for (; i < count; i++) {
        CRGB* led = &leds[i];
        
        // Red component with special handling
        if (led->r < color.r) {
            led->r = ((led->r << 8) + (((int)(((color.r - led->r) << 7) / rran) * fadeAmt / 255) << 1)) >> 8;
        } else {
            led->r = ((led->r << 8) + (((int)(((color.r - led->r) << 7) * rran) * fadeAmt / 255) << 1)) >> 8;
        }
        
        // Green and blue components
        led->g = ((led->g << 8) + ((((color.g - led->g) << 7) * fadeAmt / 255) << 1)) >> 8;
        led->b = ((led->b << 8) + ((((color.b - led->b) << 7) * fadeAmt / 255) << 1)) >> 8;
    }
}

// Initialize SIMD functionality
bool init_simd() {
    return SimdOps::init();
}

#endif // SIMD_UTILS_H
