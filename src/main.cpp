#include <Arduino.h>
#include <FastLED.h>
#include <vector>
#include "vec2.h"
#include "spatial_grid.h" // Include spatial grid before boid.h
#include "boid.h"
#include "lookup_tables.h"
#include "simd_utils.h"
#include "matrix_effects.h"
#include "palettes.h"

// Constants for the spatial grid
const int GRID_CELLS_X = 8; // Number of grid cells horizontally
const int GRID_CELLS_Y = 8; // Number of grid cells vertically

float stepcount = 0.1;
uint8_t fadebyvalue = random(10,120 );
uint8_t neidist = random(3,5);
uint8_t boidsep = random(3,5);
int neidistdir = 1;
int boidsepdir = 1;
int fadebydir = 1;
uint8_t viewStep = 1;
uint8_t viewDirX = 1;
uint8_t viewDirY = 1;
uint8_t paletteIndex = 0;
uint8_t palcount = 0;
bool velocityBasedHue = false; // Toggle for velocity-direction based coloring
uint8_t countstep = 5;
int countdir = 1;
float maxspeedstep = 0.1;
int maxspeeddir = 1;
float maxspeed = 2.5; // Maximum speed
float currgravity = 2.90;
float currmass = 40;
#define NUM_PARTICLES 255
uint8_t speed = 1;
uint8_t count = 254;
uint8_t boidseperation = 2;
uint8_t neighbordistance = 6;
#define LED_PIN 42     // this is for the esp32-s3
#define BRIGHTNESS 255 // it's blindingly bright at 255 or looks silly if you don't have enough power injected
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
const uint8_t ROWS = 24;
const uint8_t COLS = 24;

CRGB leds[ROWS * COLS];
const bool kMatrixSerpentineLayout = true;
#define NUM_LEDS (ROWS * COLS)
bool firstPass = true;
float separa = 1.0;
float alignm = 1.0;
float cohesi = 1.0;
const uint8_t VIRTUAL_ROWS = 48;
const uint8_t VIRTUAL_COLS = 48;
const uint8_t VIEWPORT_ROWS = 24;
const uint8_t VIEWPORT_COLS = 24;
// Initialize the viewport position
uint8_t virtualViewX = 24;
uint8_t virtualViewY = 24;
int rran = random(1.5F,4.0F);
int gran = random(1.5F, 4.0F);
int bran = random(1.5F, 4.0F);

typedef vec2<float> PVector;
typedef vec2<double> vec2d;

// Create a spatial grid for efficient boid neighbor lookup
SpatialGrid* spatialGrid = nullptr;

// Create a matrix effects handler
MatrixEffects* matrixEffects = nullptr;

// Other variable declarations
int movetocenterrandom = 0;
bool stopbool = false;

// Variables for the random slow down effect
unsigned long lastSlowDownTime = 0;
unsigned long nextSlowDownInterval = 0;
bool isSlowingDown = false;
bool isPaused = false;
unsigned long pauseStartTime = 0;
unsigned long pauseDuration = 0;

// Variables for attractor/repulsor management
unsigned long lastAttractorChangeTime = 0;
unsigned long attractorChangeDuration = 15000; // Change attractor behavior every 15 seconds
bool useRepulsors = false;

// Variables for additional ripple effects
unsigned long lastRippleTime = 0;
unsigned long rippleInterval = 0; // Random interval

// Define the viewport size (same as your physical display)
//boid class to make the particles interact with eaother
#pragma region
// Boid class has been moved to boid.h
//end of pallet definitions and swapping
#pragma region 
///start of pixel mapping functions
uint16_t XY(uint8_t x, uint8_t y)
{
  uint16_t i;
  if (kMatrixSerpentineLayout == false)
  {
    i = (y * ROWS) + x;
  }
  if (kMatrixSerpentineLayout == true)
  {
    if (y & 0x01)
    {
      // Odd rows run backwards
      uint8_t reverseX = (ROWS - 1) - x;
      i = (y * ROWS) + reverseX;
    }
    else
    {
      // Even rows run forwards
      i = (y * ROWS) + x;
    }
  }
  return i;
}

uint32_t getPixColor(uint32_t thisSegm)
{
  uint32_t thisPixel = thisSegm;
  if (thisPixel > NUM_LEDS - 1)
    return 0;
  return (((uint32_t)leds[thisPixel].r << 16) | ((uint32_t)leds[thisPixel].g << 8) | (uint32_t)leds[thisPixel].b);
}
uint32_t getPixColorXY(uint8_t x, uint8_t y)
{
  return getPixColor(XY(x, y));
}

void drawPixelXY(int8_t x, int8_t y, CRGB color)
{
 
 
  if (x < 0 || x > (ROWS - 1) || y < 0 || y > (COLS - 1))
    return;
  uint32_t thisPixel = XY((uint8_t)x, (uint8_t)y);
  leds[thisPixel] = color;
}
void drawPixelXYF(float virtualX, float virtualY, CRGB color) {
  // Map virtual coordinates to the current view/window of the virtual space
    float mappedX = virtualX - virtualViewX; // Example mapping, adjust virtualViewX/Y based on your logic
    float mappedY = virtualY - virtualViewY;

    // Check if the mapped coordinates are within the physical display bounds
    //if(mappedX < 0 || mappedX >= ROWS || mappedY < 0 || mappedY >= COLS) return; // Skip if outside physical display
  if(mappedX >= 0 && mappedX < VIEWPORT_COLS && mappedY >= 0 && mappedY < VIEWPORT_ROWS) {
    // Perform Wu's algorithm on the mapped coordinates
    #define WU_WEIGHT(a, b) ((uint8_t)(((a) * (b) + (a) + (b)) >> 8))
    uint8_t xx = (mappedX - (int)mappedX) * 255, yy = (mappedY - (int)mappedY) * 255, ix = 255 - xx, iy = 255 - yy;
    uint8_t wu[4] = {WU_WEIGHT(ix, iy), WU_WEIGHT(xx, iy),
                    WU_WEIGHT(ix, yy), WU_WEIGHT(xx, yy)};

    for (uint8_t i = 0; i < 4; i++) {
      int16_t xn = mappedX + (i & 1), yn = mappedY + ((i >> 1) & 1);

      // Ensure xn, yn within physical display bounds before blending
      if(xn >= 0 && xn < ROWS && yn >= 0 && yn < COLS) {
        CRGB clr = getPixColorXY(xn, yn);
        clr.r = qadd8(clr.r, (color.r * wu[i]) >> 8);
        clr.g = qadd8(clr.g, (color.g * wu[i]) >> 8);
        clr.b = qadd8(clr.b, (color.b * wu[i]) >> 8);
        drawPixelXY(xn, yn, clr); // Make sure drawPixelXY checks bounds if not already
      }
    }
  }
}
//attractor class to make the particles gravitate towards a certain point
class Attractor {
  public:
    float mass; // Mass, tied to size
    int massdir = 1;
    float massstep = 0.1;
    float GStep = 0.1;
    int gdir = 1;
    uint8_t massdelaywait = random(5,20);
    uint8_t delaytimer = 0;
    uint8_t massdelay = random (20,800);
    float G; // Gravitational Constant
    PVector location; // Location
    bool isRepulsor = false; // If true, this will repel instead of attract
    float maxInfluenceRadius = 100.0; // Maximum distance this attractor influences
    float minDistance = 15.0; // Minimum distance to prevent extreme forces
    CRGB color = CRGB::Blue; // Visual color for debugging
    bool showVisually = false; // Whether to show this attractor visually
    bool hasVortex = false; // Whether to apply rotational force
    float vortexStrength = 0.5; // Strength of the vortex effect
    
    Attractor() {
      location = PVector((virtualViewX + 12 / 2), (virtualViewY + 12 / 2));
      mass = currmass;
      G = currgravity;
    } 
    
    void setlocation(float x, float y) {
      location = PVector(x, y);
    }
    
    void setMass(float m) {
      mass = m;
    }
    
    void setG(float g) {
      G = g;
    }
    
    void setRepulsor(bool repel) {
      isRepulsor = repel;
      // Repulsors typically look red
      if (repel) {
        color = CRGB::Red;
      } else {
        color = CRGB::Blue;
      }
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
        mass += (mass / 8); // + random(1,2)* massdir;
        G += 0.5 * gdir;
        if (G > 4 || G < 0.50) gdir = -gdir; //G = mass;
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
    
    // Orbit around a point
    void orbit(float centerX, float centerY, float radius, float speed, float phase) {
      float angle = (millis() / 1000.0) * speed + phase;
      float x = centerX + radius * cos(angle);
      float y = centerY + radius * sin(angle);
      location = PVector(x, y);
    }
    
    // Draw the attractor/repulsor for debugging
    void draw() {
      if (!showVisually) return;
      
      // Calculate the radius size based on mass
      float radius = constrain(mass / 10.0, 1.0, 3.0);
      
      // Set color brightness based on G value
      CRGB drawColor = color;
      drawColor.nscale8(map(G, 0, 10, 50, 255));
      
      // Draw the attractor/repulsor
      for (float r = 0; r < radius; r += 0.5) {
        drawPixelXYF(location.x, location.y, drawColor);
      }
    }
    
    PVector attract(Boid m) {
      PVector force = location - m.location; // Calculate direction of force
      float d = force.mag(); // Distance between objects
      
      // If outside influence radius, return zero force
      if (d > maxInfluenceRadius) return PVector(0, 0);
      
      d = constrain(d, minDistance, maxInfluenceRadius); // Limiting the distance to eliminate "extreme" results
      force.normalize(); // Normalize vector (distance doesn't matter here, we just want this vector for direction)
      
      float strength = (G * mass * m.mass) / (d * d); // Calculate gravitational force magnitude
      
      // If this is a repulsor, reverse the direction
      if (isRepulsor) {
        force = force * -1;
      }
      
      force *= strength; // Get force vector --> magnitude * direction
      
      // Add vortex/rotational component if enabled
      if (hasVortex && strength > 0.001) { // Safety check
        PVector tangent = force.ortho(); // Perpendicular vector for rotation
        float tangentMag = tangent.mag();
        if (tangentMag > 0.001) { // Prevent normalization of zero vector
          tangent.normalize();
          tangent *= strength * vortexStrength;
          force += tangent;
        }
      }
      
      return force;
    }
  };
//Fades CRGB array towards the background color by amount.  
//fadeAmt > 102 breaks fade but has artistic value(?)

void fadeToColorBy(CRGB* leds, int count, CRGB color, uint8_t fadeAmt) {
    // Use the SIMD-optimized version of this function
    simd_fade_to_color(leds, count, color, fadeAmt);
}
//attractor class to make the particles gravitate towards a certain point

Attractor attractor5;   // Main central attractor
Attractor attractor1;   // Secondary attractor for moveToCenter

// New attractors and repulsors
Attractor attractorTopLeft;     // Attractor in top-left corner
Attractor attractorTopRight;    // Attractor in top-right corner
Attractor attractorBottomLeft;  // Attractor in bottom-left corner
Attractor attractorBottomRight; // Attractor in bottom-right corner
Attractor repulsorCenter;       // Repulsor in center (pushes boids away)

// New attractors for advanced patterns
Attractor attractorSquare1;     // Rotating square pattern
Attractor attractorSquare2;
Attractor attractorSquare3;
Attractor attractorSquare4;
Attractor attractorFigure8_1;   // Figure-8 pattern
Attractor attractorFigure8_2;
Attractor attractorSpiral1;     // Spiral pattern
Attractor attractorSpiral2;
Attractor attractorSpiral3;
Attractor attractorVortex;      // Central vortex attractor
Attractor explosionRepulsor;    // Temporary explosion effect

// Array of attractors for easier management
const int MAX_ATTRACTORS = 17;
Attractor* attractorArray[MAX_ATTRACTORS];
bool attractorActive[MAX_ATTRACTORS] = {true, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};

// Current attractor pattern index
int currentAttractorPattern = 0;
const int NUM_ATTRACTOR_PATTERNS = 11;

// Explosion state
bool explosionActive = false;
unsigned long explosionStartTime = 0;
unsigned long lastExplosionTime = 0;

// Debug flag
#define DEBUG_SERIAL true

Boid boids[NUM_PARTICLES];
int degreestep = 1;
double degree;
int degreedir = 1;
int degreestepdir = 1;
//main functions 
// Initialize attractor patterns
void initAttractorPatterns() {
  // Store all attractors in the array for easier management
  attractorArray[0] = &attractor5;         // Main central attractor
  attractorArray[1] = &attractor1;         // Secondary central attractor
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
  
  // Initialize corner attractors
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
  
  // Initialize repulsor
  repulsorCenter.setlocation(centerX, centerY);
  repulsorCenter.setMass(50);
  repulsorCenter.setG(3.0);
  repulsorCenter.setRepulsor(true);
  repulsorCenter.setRadius(5.0, 120.0);
  
  // Initialize rotating square attractors
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
  
  // Initialize figure-8 attractors
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
  
  // Initialize spiral attractors
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
  
  // Initialize vortex attractor
  attractorVortex.setlocation(centerX, centerY);
  attractorVortex.setMass(40);
  attractorVortex.setG(2.5);
  attractorVortex.setRadius(5.0, 110.0);
  attractorVortex.setVortex(true, 0.8);
  attractorVortex.color = CRGB::Purple;
  
  // Initialize explosion repulsor (inactive by default)
  explosionRepulsor.setlocation(centerX, centerY);
  explosionRepulsor.setMass(60);  // Reduced from 100
  explosionRepulsor.setG(4.0);     // Reduced from 8.0
  explosionRepulsor.setRepulsor(true);
  explosionRepulsor.setRadius(5.0, 100.0);  // Reduced max radius from 150 to 100
  explosionRepulsor.color = CRGB::Red;
}

// Switch to a new attractor pattern
void setAttractorPattern(int patternIndex) {
  #if DEBUG_SERIAL
  Serial.print("[PATTERN] Switching to pattern: ");
  Serial.println(patternIndex);
  #endif
  
  // Bounds check
  if (patternIndex < 0 || patternIndex >= NUM_ATTRACTOR_PATTERNS) {
    #if DEBUG_SERIAL
    Serial.print("[ERROR] Invalid pattern index: ");
    Serial.println(patternIndex);
    #endif
    return;
  }
  
  // Reset all attractors to inactive
  for (int i = 0; i < MAX_ATTRACTORS; i++) {
    attractorActive[i] = false;
  }
  
  // Always keep the main attractor active
  attractorActive[0] = true;
  
  // Set specific patterns
  switch (patternIndex) {
    case 0: // Default - just the main attractor
      // Nothing to add, main attractor is already active
      break;
      
    case 1: // Corners - activate all four corner attractors
      attractorActive[2] = true; // Top Left
      attractorActive[3] = true; // Top Right
      attractorActive[4] = true; // Bottom Left
      attractorActive[5] = true; // Bottom Right
      break;
      
    case 2: // Central repulsor with corner attractors
      attractorActive[2] = true; // Top Left
      attractorActive[3] = true; // Top Right
      attractorActive[4] = true; // Bottom Left
      attractorActive[5] = true; // Bottom Right
      attractorActive[6] = true; // Central repulsor
      break;
      
    case 3: // Diagonal attractors
      attractorActive[2] = true; // Top Left
      attractorActive[5] = true; // Bottom Right
      break;
      
    case 4: // Other diagonal attractors
      attractorActive[3] = true; // Top Right
      attractorActive[4] = true; // Bottom Left
      break;
      
    case 5: // Central repulsor only
      attractorActive[6] = true; // Central repulsor
      break;
      
    case 6: // Rotating square
      attractorActive[7] = true;  // Square 1
      attractorActive[8] = true;  // Square 2
      attractorActive[9] = true;  // Square 3
      attractorActive[10] = true; // Square 4
      break;
      
    case 7: // Figure-8
      attractorActive[11] = true; // Figure-8 attractor 1
      attractorActive[12] = true; // Figure-8 attractor 2
      break;
      
    case 8: // Pulsing center (just main attractor with enhanced pulsing)
      // Main attractor already active
      break;
      
    case 9: // Spiral arms
      attractorActive[13] = true; // Spiral 1
      attractorActive[14] = true; // Spiral 2
      attractorActive[15] = true; // Spiral 3
      break;
      
    case 10: // Central vortex
      attractorActive[16] = true; // Vortex attractor
      break;
  }
  
  currentAttractorPattern = patternIndex;
}

// Update attractor positions for dynamic movement
void updateAttractors() {
  // Main attractor orbits the center
  float centerX = virtualViewX + VIRTUAL_COLS / 2;
  float centerY = virtualViewY + VIRTUAL_ROWS / 2;
  
  // Update main attractor (typically always active)
  if (attractorActive[0]) {
    attractorArray[0]->orbit(centerX, centerY, 10, 0.2, 0);
    attractorArray[0]->incrementMass();
  }
  
  // Apply orbital motion to corner attractors if active
  if (attractorActive[2]) { // Top Left
    float tlx = virtualViewX + VIRTUAL_COLS * 0.25;
    float tly = virtualViewY + VIRTUAL_ROWS * 0.25;
    attractorArray[2]->orbit(tlx, tly, 5, 0.5, 0);
  }
  
  if (attractorActive[3]) { // Top Right
    float trx = virtualViewX + VIRTUAL_COLS * 0.75;
    float try_ = virtualViewY + VIRTUAL_ROWS * 0.25;
    attractorArray[3]->orbit(trx, try_, 5, 0.5, PI/2);
  }
  
  if (attractorActive[4]) { // Bottom Left
    float blx = virtualViewX + VIRTUAL_COLS * 0.25;
    float bly = virtualViewY + VIRTUAL_ROWS * 0.75;
    attractorArray[4]->orbit(blx, bly, 5, 0.5, PI);
  }
  
  if (attractorActive[5]) { // Bottom Right
    float brx = virtualViewX + VIRTUAL_COLS * 0.75;
    float bry = virtualViewY + VIRTUAL_ROWS * 0.75;
    attractorArray[5]->orbit(brx, bry, 5, 0.5, PI*1.5);
  }
  
  // Pulse the repulsor strength if active
  if (attractorActive[6]) {
    float pulseFactor = (sin(millis() / 1000.0) + 1) / 2.0; // 0 to 1
    attractorArray[6]->setG(1.0 + pulseFactor * 3.0); // 1.0 to 4.0
  }
  
  // Update rotating square pattern (attractors 7-10)
  if (attractorActive[7] || attractorActive[8] || attractorActive[9] || attractorActive[10]) {
    float squareRadius = 15;
    float squareSpeed = 0.4;
    float time = millis() / 1000.0;
    
    if (attractorActive[7]) {
      float angle = time * squareSpeed;
      attractorArray[7]->setlocation(
        centerX + squareRadius * cos(angle),
        centerY + squareRadius * sin(angle)
      );
    }
    if (attractorActive[8]) {
      float angle = time * squareSpeed + PI / 2;
      attractorArray[8]->setlocation(
        centerX + squareRadius * cos(angle),
        centerY + squareRadius * sin(angle)
      );
    }
    if (attractorActive[9]) {
      float angle = time * squareSpeed + PI;
      attractorArray[9]->setlocation(
        centerX + squareRadius * cos(angle),
        centerY + squareRadius * sin(angle)
      );
    }
    if (attractorActive[10]) {
      float angle = time * squareSpeed + PI * 1.5;
      attractorArray[10]->setlocation(
        centerX + squareRadius * cos(angle),
        centerY + squareRadius * sin(angle)
      );
    }
  }
  
  // Update figure-8 pattern (attractors 11-12)
  if (attractorActive[11] || attractorActive[12]) {
    float time = millis() / 1000.0;
    float speed = 0.5;
    float width = 18;
    float height = 10;
    
    if (attractorActive[11]) {
      // Lemniscate (figure-8) parametric equations
      float t = time * speed;
      float sinT = sin(t);
      float cosT = cos(t);
      float denominator = 1.0f + sinT * sinT;
      if (denominator > 0.1f) { // Safety check
        float scale = width / denominator;
        float x = centerX + scale * cosT;
        float y = centerY + height * sinT * cosT;
        attractorArray[11]->setlocation(x, y);
      }
    }
    
    if (attractorActive[12]) {
      // Second attractor on opposite phase
      float t = time * speed + PI;
      float sinT = sin(t);
      float cosT = cos(t);
      float denominator = 1.0f + sinT * sinT;
      if (denominator > 0.1f) { // Safety check
        float scale = width / denominator;
        float x = centerX + scale * cosT;
        float y = centerY + height * sinT * cosT;
        attractorArray[12]->setlocation(x, y);
      }
    }
  }
  
  // Update pulsing center pattern (attractor 0 with enhanced pulsing)
  if (currentAttractorPattern == 8 && attractorActive[0]) {
    // Enhanced pulsing for this pattern
    float pulseFactor = (sin(millis() / 500.0) + 1) / 2.0; // Faster pulse
    attractorArray[0]->setMass(10 + pulseFactor * 60); // 10 to 70
    attractorArray[0]->setG(0.5 + pulseFactor * 3.5); // 0.5 to 4.0
  }
  
  // Update spiral pattern (attractors 13-15)
  if (attractorActive[13] || attractorActive[14] || attractorActive[15]) {
    float time = millis() / 1000.0;
    float spiralSpeed = 0.6;
    float spiralRadius = 16;
    
    if (attractorActive[13]) {
      attractorArray[13]->orbit(centerX, centerY, spiralRadius, spiralSpeed, 0);
    }
    if (attractorActive[14]) {
      attractorArray[14]->orbit(centerX, centerY, spiralRadius, spiralSpeed, (2 * PI) / 3);
    }
    if (attractorActive[15]) {
      attractorArray[15]->orbit(centerX, centerY, spiralRadius, spiralSpeed, (4 * PI) / 3);
    }
  }
  
  // Update vortex attractor (attractor 16)
  if (attractorActive[16]) {
    // Vortex slowly orbits and pulses
    attractorArray[16]->orbit(centerX, centerY, 5, 0.15, 0);
    float pulseFactor = (sin(millis() / 800.0) + 1) / 2.0;
    attractorArray[16]->setG(1.5 + pulseFactor * 2.0); // 1.5 to 3.5
  }
  
  // Handle explosion effect
  if (explosionActive) {
    // Explosion lasts 500ms
    if (millis() - explosionStartTime > 500) {
      explosionActive = false;
      #if DEBUG_SERIAL
      Serial.println("[EXPLOSION] Ended");
      #endif
    }
  } else {
    // Randomly trigger explosions (5% chance every 30-90 seconds)
    if (millis() - lastExplosionTime > 30000 && random8() < 13) {
      #if DEBUG_SERIAL
      Serial.println("[EXPLOSION] Starting!");
      #endif
      explosionActive = true;
      explosionStartTime = millis();
      lastExplosionTime = millis();
      float explodeX = centerX + random(0, 20) - 10;
      float explodeY = centerY + random(0, 20) - 10;
      
      // Constrain to virtual canvas bounds
      explodeX = constrain(explodeX, virtualViewX + 2, virtualViewX + VIRTUAL_COLS - 2);
      explodeY = constrain(explodeY, virtualViewY + 2, virtualViewY + VIRTUAL_ROWS - 2);
      
      #if DEBUG_SERIAL
      Serial.print("[EXPLOSION] Location: ");
      Serial.print(explodeX);
      Serial.print(", ");
      Serial.println(explodeY);
      Serial.print("[EXPLOSION] centerX: ");
      Serial.print(centerX);
      Serial.print(", centerY: ");
      Serial.println(centerY);
      #endif
      explosionRepulsor.setlocation(explodeX, explodeY);
      
      // Trigger visual effects with reduced intensity
      if (matrixEffects) {
        matrixEffects->startScreenShake(8, 2);  // Reduced from (10, 3) to (8, 2)
        matrixEffects->startRipple();
        delay(50);  // Small delay between ripples
        matrixEffects->startRipple();
      }
    }
  }
  
  // Check if it's time to change attractor pattern
  if (millis() - lastAttractorChangeTime > attractorChangeDuration) {
    #if DEBUG_SERIAL
    Serial.println("[PATTERN] Time to switch patterns");
    #endif
    // Change to a new random pattern
    int newPattern = random(0, NUM_ATTRACTOR_PATTERNS);
    int attempts = 0;
    while (newPattern == currentAttractorPattern && attempts < 20) {
      newPattern = random(0, NUM_ATTRACTOR_PATTERNS);
      attempts++;
    }
    #if DEBUG_SERIAL
    Serial.print("[PATTERN] Selected new pattern after ");
    Serial.print(attempts);
    Serial.print(" attempts: ");
    Serial.println(newPattern);
    #endif
    setAttractorPattern(newPattern);
    
    // Randomly toggle velocity-based hue (30% chance)
    if (random8() < 77) {
      velocityBasedHue = !velocityBasedHue;
    }
    
    // Trigger special effects when changing patterns
    if (matrixEffects) {
      matrixEffects->startRipple();
      //delay(120);
      matrixEffects->startRipple(); // Second ripple
      if (random8() < 180) { // 70% chance (was 50%)
        matrixEffects->startScreenShake(5, 1);
      }
    }
    
    // Reset the timer
    lastAttractorChangeTime = millis();
    // Set a random duration for this pattern (10-30 seconds)
    attractorChangeDuration = random(10000, 30000);
  }
  
  // Draw active attractors (for debugging)
  for (int i = 0; i < MAX_ATTRACTORS; i++) {
    if (attractorActive[i]) {
      attractorArray[i]->draw();
    }
  }
}

void start()
{
  for (int i = 0; i < count; i++)
  {
    boids[i] = Boid(random(COLS), 0);
  }
  
  // Initialize the main attractors
  attractor1.setlocation((virtualViewX+24/ 2), (virtualViewY+24/ 2));
  attractor1.setMass(100);
  attractor1.setG(4);
  
  attractor5.setlocation((virtualViewX+36/ 2), (virtualViewY+36/ 2));
  attractor5.setMass(1);
  attractor5.setG(0.1);
  
  // Initialize all attractors
  initAttractorPatterns();
  
  // Start with the default pattern
  setAttractorPattern(0);
  lastAttractorChangeTime = millis();
}
int massdir = 1;
float massstep = 0.5;

PVector center = PVector(36,36);

void movetoCenter()
{
  int countto = random(5, 25);
    
    // Safety check for spatialGrid initialization
    if (!spatialGrid) {
        spatialGrid = new SpatialGrid(GRID_CELLS_X, GRID_CELLS_Y, VIRTUAL_ROWS, VIRTUAL_COLS);
    }
    
    // Trigger a color wash effect when moving to center
    if (matrixEffects) {
        matrixEffects->startColorWash(1, 8, 25); // Vertical wash
    }
    
    // Clear the spatial grid before new movement calculations
    spatialGrid->clear();
    
    // First, insert all boids into the spatial grid
    for (int i = 0; i < count; i++) {
        Boid* boid = &boids[i];
        spatialGrid->insert(boid, boid->location.x, boid->location.y);
    }
    
    for (int j = 0; j < countto; j++) {
        // Re-clear and populate the grid for each iteration
        spatialGrid->clear();
        for (int i = 0; i < count; i++) {
            Boid* boid = &boids[i];
            spatialGrid->insert(boid, boid->location.x, boid->location.y);
        }
        
        for (int i = 0; i < count; i++) {
            Boid* boid = &boids[i];
            PVector force1 = attractor1.attract(*boid);
            boid->applyForce(force1);
            
            // Use the optimized update method with spatial grid
            boid->update(*spatialGrid);
            boid->wrapAroundBorders(VIRTUAL_ROWS, VIRTUAL_COLS);
            
            drawPixelXYF(boid->location.x, boid->location.y, ColorFromPalette(*currentPalette_p, boid->hue * 15, 255, NOBLEND));
            boid->neighbordist = neidist;
            boid->desiredseparation = boidsep;
            
            if (stopbool == true) {
                boid->velocity = PVector(0, 0);
            }
        }
        
        // Apply matrix effects during center movement
        if (matrixEffects) {
            matrixEffects->update(leds);
        }
        
        fadeToColorBy(leds, NUM_LEDS, CRGB::Black, 45);
        LEDS.show();
    }
    
    // After finishing center movement, stop color wash and add multiple ripples
    if (matrixEffects) {
        matrixEffects->stopColorWash();
        matrixEffects->startRipple();
        //delay(100); // Small delay between ripples
        matrixEffects->startRipple(); // Second ripple
    }
}

void setNewRandomColorFacd()
{
  rran = random(1.5F,4.0F);
gran = random(1.5F, 4.0F);
bran = random(1.5F, 4.0F);
}
void randomSlowDownAndSpeed() {
    static float originalSpeeds[NUM_PARTICLES];
    
    // Phase 1: Start slowing down - save original speeds
    if (!isSlowingDown && !isPaused) {
        isSlowingDown = true;
        
        // Store original speeds for each boid
        for (int i = 0; i < count; i++) {
            originalSpeeds[i] = boids[i].maxspeed;
        }
        
        // Trigger a screen shake effect when starting to slow down
        if (matrixEffects) {
            matrixEffects->startScreenShake(8, 1); // Subtle screen shake
        }
    }
    
    // Phase 2: Gradually reduce speed
    if (isSlowingDown) {
        bool allStopped = true;
        
        // Reduce all boid speeds by 0.1
        for (int i = 0; i < count; i++) {
            if (boids[i].maxspeed > 0.2) {
                boids[i].maxspeed -= 0.1;
                allStopped = false;
            } else {
                boids[i].maxspeed = 0;
            }
        }
        
        // If all boids have stopped, start the pause
        if (allStopped) {
            isSlowingDown = false;
            isPaused = true;
            pauseStartTime = millis();
            pauseDuration = random(3000, 15000); // 3-15 seconds in milliseconds
            
            // Start starfield effect during pause
            if (matrixEffects) {
                matrixEffects->startStarfield();
            }
        }
    }
    
    // Phase 3: After pause, set random speeds
    if (isPaused && (millis() - pauseStartTime >= pauseDuration)) {
        isPaused = false;
        
        // Set random speeds for each boid between 1.1 and 2.0
        for (int i = 0; i < count; i++) {
            boids[i].maxspeed = random(1.1F, 2.0F);
        }
        
        // Schedule next slow down event
        lastSlowDownTime = millis();
        nextSlowDownInterval = random(10000, 40000); // 10-40 seconds in milliseconds
        
        // End starfield effect and trigger multiple ripples
        if (matrixEffects) {
            matrixEffects->stopStarfield();
            matrixEffects->startRipple();
            //delay(150); // Small delay between ripples
            matrixEffects->startRipple(); // Second ripple
        }
    }
}
void draw() {
  //  Animation timing variables
    int randomnum = random(0,100);
    movetocenterrandom = random(0,200);
    if (randomnum == 5) stopbool = true;
    
    if (firstPass) {
        start();
        firstPass = false;
    }

    // Safety check for spatialGrid initialization
    if (!spatialGrid) {
        spatialGrid = new SpatialGrid(GRID_CELLS_X, GRID_CELLS_Y, VIRTUAL_ROWS, VIRTUAL_COLS);
    }
    
    // Check if it's time for the random slow down effect
    if (!isSlowingDown && !isPaused && (millis() - lastSlowDownTime >= nextSlowDownInterval)) {
        randomSlowDownAndSpeed();
    }
    
    // Continue slow down effect if it's in progress
    if (isSlowingDown || isPaused) {
        randomSlowDownAndSpeed();
    }
    
    // Occasionally trigger a random matrix effect
    EVERY_N_SECONDS(15) { // Increased frequency (was 30)
        if (matrixEffects && random8() < 180) { // 70% chance (was 50%)
            matrixEffects->triggerRandomEffect();
        }
    }
    
    // Dedicated timer for random ripples
    if (matrixEffects && (millis() - lastRippleTime > rippleInterval)) {
        matrixEffects->startRipple();
        lastRippleTime = millis();
        rippleInterval = random(100, 1000); // 3-8 seconds between random ripples
    }
    
    // Get screen shake offsets if active
    int8_t shakeOffsetX = 0;
    int8_t shakeOffsetY = 0;
    if (matrixEffects) {
        matrixEffects->getShakeOffsets(shakeOffsetX, shakeOffsetY);
    }
    
    // Clear the spatial grid at the beginning of each frame
    spatialGrid->clear();
    
    // First, insert all boids into the spatial grid
    for (int i = 0; i < count; i++) {
        Boid* boid = &boids[i];
        spatialGrid->insert(boid, boid->location.x, boid->location.y);
    }
    
    // Update all attractor positions and behaviors
    updateAttractors();
    
    // Now update all boids using the spatial grid for neighbor lookup

    
    // Apply matrix effects before the fade
    if (matrixEffects) {
        matrixEffects->update(leds);
    }
    
    // Apply fade effect
    fadeToColorBy(leds, NUM_LEDS, CRGB::Black, 45);
    
    // Occasionally move to center
    if (movetocenterrandom == 100) {
        movetoCenter();
        movetocenterrandom = 0;
    }
    
    if (stopbool == true) {
        stopbool = false;
    }
    
    // Update animation parameters
    EVERY_N_MILLISECONDS(10) {
        attractor5.incrementMass();
        
        if (fadebyvalue < 10 || fadebyvalue > 120)
            fadebydir = -fadebydir;
        fadebyvalue += 1 * fadebydir;
    }
    
    degreestep += 3;
    if (degreestep > 10 || degreestep < 1) degreestepdir = -degreestepdir;
    
    degree += degreestep * degreedir;
    if (degree > 45 || degree < 1) degreedir = -degreedir;
    
    attractor5.location.rotateAroundPoint(center.x, center.y, degree);
    
    EVERY_N_MILLISECONDS(500) {
        if (count <= 2 || count >= 254)
            countdir = -countdir;
        count += countstep * countdir;
        
        if (maxspeed <= 0.4 || maxspeed >= 2.5)
            maxspeeddir = -maxspeeddir;
        maxspeed += maxspeedstep * maxspeeddir;
    }
    
    EVERY_N_MILLISECONDS(10000) {
        palcount = random(0, 24);
        SetNewPalette(palcount);
        setNewRandomColorFacd();
        
        // Always trigger ripple effects when changing palette
        if (matrixEffects) {
            matrixEffects->startRipple();
            //delay(150);
            matrixEffects->startRipple();
        }
    }
    for (int i = 0; i < count; i++) {
      Boid* boid = &boids[i];
      
      // Apply forces from all active attractors
      for (int j = 0; j < MAX_ATTRACTORS; j++) {
          if (attractorActive[j]) {
              PVector force = attractorArray[j]->attract(*boid);
              boid->applyForce(force);
          }
      }
      
      // Apply explosion repulsor force if active
      if (explosionActive) {
          PVector explosionForce = explosionRepulsor.attract(*boid);
          boid->applyForce(explosionForce);
      }
      
      // Use the optimized update method with spatial grid
      
      boid->wrapAroundBorders(VIRTUAL_ROWS, VIRTUAL_COLS);
      boid->brightness = map(boid->velocity.x + boid->velocity.y, 0.1, 4.5, 25, 255);
      boid->mass = (255-count)/6;
      boid->update(*spatialGrid);
     // Apply screen shake offset when drawing
     float drawX = boid->location.x + shakeOffsetX;
     float drawY = boid->location.y + shakeOffsetY;
     
     // Calculate hue based on velocity direction if enabled
     uint8_t renderHue;
     if (velocityBasedHue) {
         // Map velocity angle to hue (0-255)
         float angle = atan2(boid->velocity.y, boid->velocity.x);
         renderHue = (uint8_t)((angle + PI) * 40.7436f); // Map -PI to PI -> 0 to 255
     } else {
         renderHue = boid->hue * 15;
     }
     
     drawPixelXYF(drawX, drawY, ColorFromPalette(*currentPalette_p, renderHue, boid->brightness, NOBLEND));//
     // drawPixelXYF(drawX, drawY, ColorFromPalette(*currentPalette_p, 0, 0, NOBLEND));
      boid->neighbordist = neidist;
     boid->desiredseparation = boidsep;
      
      if (stopbool == true) {
          boid->velocity = PVector(0, 0);
      }
  }
    // Show updated LEDs
    LEDS.show();
    
    // Increment step count
  //  stepcount += 0.1;
}

void setup() {
   #if DEBUG_SERIAL
   Serial.begin(115200);
   delay(1000);
   Serial.println("\n\n[SETUP] Starting WS2812B Boid Animation");
   Serial.print("[SETUP] NUM_ATTRACTOR_PATTERNS: ");
   Serial.println(NUM_ATTRACTOR_PATTERNS);
   Serial.print("[SETUP] MAX_ATTRACTORS: ");
   Serial.println(MAX_ATTRACTORS);
   #endif
    // Initialize SIMD functionality
    init_simd();
    randomSeed(analogRead(0));
    
    // Initialize the spatial grid
    spatialGrid = new SpatialGrid(GRID_CELLS_X, GRID_CELLS_Y, VIRTUAL_ROWS, VIRTUAL_COLS);
    
    // Initialize the matrix effects
    matrixEffects = new MatrixEffects(ROWS, COLS, XY);
    
    // Initialize the slow down timer
    lastSlowDownTime = millis();
    nextSlowDownInterval = random(10000, 40000); // 10-40 seconds
    
    // Initialize attractor change timing
    lastAttractorChangeTime = millis();
    attractorChangeDuration = random(10000, 20000); // 10-20 seconds initial duration
    
    // Initialize ripple timer
    lastRippleTime = millis();
    rippleInterval = random(3000, 8000); // 3-8 seconds between random ripples
    
    delay(3000);
    LEDS.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    LEDS.setBrightness(BRIGHTNESS);
    
  //  Serial.println("WS2812B Boids with Spatial Partitioning, Matrix Effects, and Dynamic Attractors");
}

void loop() {
    draw();
}