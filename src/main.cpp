#include <Arduino.h>
#include <FastLED.h>
#include <Math.h>
#include <vector>
#include "vec2.h"

float stepcount = 0.3;
uint8_t fadebyvalue = random(110,120 );
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
uint8_t countstep = 5;
int countdir = 1;
float maxspeedstep = 0.1;
int maxspeeddir = 1;
float maxspeed; // Maximum speed
float currgravity = 1.90;
float currmass = 20;
#define NUM_PARTICLES 60
uint8_t speed = 1;
uint8_t count = 50;
uint8_t boidseperation = 3;
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
float separa = 2.0;
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

// Define the viewport size (same as your physical display)
//boid class to make the particles interact with eaother
#pragma region
class Boid
{
  public:
    PVector location;
    PVector velocity;
    PVector acceleration;
    float maxforce; // Maximum steering force
    uint8_t brightness;
    int hue;
    float desiredseparation = 3; // this this is the variable that keeps the boids at a certain distance from eachother. the higher the number the more space between. keep this lower than the neighbor distance otherwise the boids will probably not move.
    float neighbordist = 8;      // this will determine the length that a boid will interact with another boid. the lower the number the more isolated the boid will be. the higher the number the boid will interact with another boid at a further distance.
    byte colorIndex = 0;
    //float scale;
    float mass;
    //float bounce = 1;
    boolean enabled = true;

    Boid() {}

    Boid(float x, float y)
    {
      acceleration = PVector(0, 0);
      velocity = PVector(randomf(), randomf());
      location = PVector(x, y);
      maxspeed = 2.2;
      maxforce = 0.28;
      brightness = 255;
      //scale = random(0.5, 68.5);
    //  bounce = random(2.1, 2.5);
      mass = random(1.0, 3.0);
      hue = random(10, 255);
    }

    static float randomf()
    {
      return mapfloat(random(0, 255), 0, 255, -.5, .5);
    }

    static float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
    {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    void run(Boid boids[], uint8_t boidCount)
    {
      flock(boids, boidCount);
    }
    // Method to update location
    void update(Boid boids[], uint8_t boidCount)
    {
      flock(boids, boidCount);
      velocity += acceleration;
      velocity.limit(maxspeed);
      location += velocity;
      acceleration *= 0;
    }

    void applyForce(PVector force)
    {
      // We could add mass here if we want A = F / M
      acceleration += force;//(force.x / mass)+ (force.y / mass);
    }

    void repelForce(PVector obstacle, float radius)
    {
      // Force that drives boid away from obstacle.

      PVector futPos = location + velocity; // Calculate future position for more effective behavior.
      PVector dist = obstacle - futPos;
      float d = dist.mag();

      if (d <= radius)
      {
        PVector repelVec = location - obstacle;
        repelVec.normalize(); // no more slow 1/sqrt(float)  to normalize
        if (d != 0)
        { // Don't divide by zero.
          // float boid->boid.scale = 1.0 / d; //The closer to the obstacle, the stronger the force.
          repelVec.normalize(); // no more slow 1/sqrt(float)  to normalize
          repelVec *= (maxforce * 7);
          if (repelVec.mag() < 0)
          { // Don't let the boids turn around to avoid the obstacle.
            repelVec.y = 0;
          }
        }
        applyForce(repelVec);
      }
    }

    // We accumulate a new acceleration each time based on three rules
    void flock(Boid boids[], uint8_t boidCount)
    {
      PVector sep = separate(boids, boidCount); // Separation
      PVector ali = align(boids, boidCount);    // Alignment
      PVector coh = cohesion(boids, boidCount); // Cohesion
      // Arbitrarily weight these forces
      sep *= separa; // 2.8; //change these to change the behavior of the flock
      ali *= alignm; // 1.2; //
      coh *= cohesi; // 1.2; //
      // Add the force vectors to acceleration
      applyForce(sep);
      applyForce(ali);
      applyForce(coh);
    }

    // Separation
    // Method checks for nearby boids and steers away
    PVector separate(Boid boids[], uint8_t boidCount)
    {
      PVector steer = PVector(0, 0);
      int count = 0;
     
      // For every boid in the system, check if it's too close
      for (int i = 0; i < boidCount; i++)
      {
        Boid other = boids[i];
        if (!other.enabled)
          continue;
        float d = location.dist(other.location);
        // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
        if ((d > 0) && (d < desiredseparation))
        {
          // Calculate vector pointing away from neighbor
          PVector diff = location - other.location;
          diff.normalize();
          diff /= d; // Weight by distance
          steer += diff;
          count++; // Keep track of how many
        }
      }
      // Average -- divide by how many
      if (count > 0)
      {
        steer /= (float)count;
      }

      // As long as the vector is greater than 0
      if (steer.mag() > 0)
      {
        // Implement Reynolds: Steering = Desired - Velocity
        steer.normalize(); // no more slow 1/sqrt(float)  to normalize
        steer *= maxspeed;
        steer -= velocity;
        steer.limit(maxforce);
      }
      return steer;
    }

    // Alignment
    PVector align(Boid boids[], uint8_t boidCount)
    { // For every nearby boid in the system, calculate the average velocity
      PVector sum = PVector(0, 0);
      int count = 0;
      for (int i = 0; i < boidCount; i++)
      {
        Boid other = boids[i];
        if (!other.enabled)
          continue;
        float d = location.dist(other.location);
        if ((d > 0) && (d < neighbordist))
        {
          sum += other.velocity;
          count++;
        }
      }
      if (count > 0)
      {
        sum /= (float)count;
        sum.normalize(); // no more slow 1/sqrt(float)  to normalize
        sum *= maxspeed;
        PVector steer = sum - velocity;
        steer.limit(maxforce);
        return steer;
      }
      else
      {
        return PVector(0, 0);
      }
    }
    // Cohesion
    PVector cohesion(Boid boids[], uint8_t boidCount)
    {                              // For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
      PVector sum = PVector(0, 0); // Start with empty vector to accumulate all locations
      int count = 0;
      for (int i = 0; i < boidCount; i++)
      {
        Boid other = boids[i];
        if (!other.enabled)
          continue;
        float d = location.dist(other.location);
        if ((d > 0) && (d < neighbordist))
        {
          sum += other.location; // Add location
          count++;
        }
      }
      if (count > 0)
      {
        sum /= count;
        return seek(sum); // Steer towards the location
      }
      else
      {
        return PVector(0, 0);
      }
    }

    // A method that calculates and applies a steering force towards a target
    // STEER = DESIRED MINUS VELOCITY
    PVector seek(PVector target)
    {
      PVector desired = target - location; // A vector pointing from the location to the target
      // Normalize desired and boid->boid.scale to maximum speed
      desired.normalize(); // no more slow 1/sqrt(float)  to normalize
      desired *= maxspeed;
      // Steering = Desired minus Velocity
      PVector steer = desired - velocity;
      steer.limit(maxforce); // Limit to maximum steering force
      return steer;
    }

    // A method that calculates a steering force towards a target
    // STEER = DESIRED MINUS VELOCITY
    void arrive(PVector target)
    {
      PVector desired = target - location; // A vector pointing from the location to the target
      float d = desired.mag();
      // Normalize desired and boid->boid.scale with arbitrary damping within 100 pixels
      desired.normalize(); // no more slow 1/sqrt(float)  to normalize
      if (d < 4)
      {
        float m = map(d, 0, 100, 0, maxspeed);
        desired *= m;
      }
      else
      {
        desired *= maxspeed;
      }

      // Steering = Desired minus Velocity
      PVector steer = desired - velocity;
      steer.limit(maxforce); // Limit to maximum steering force
      applyForce(steer);
    }
    // bool bounceOffBorders()
    // {
    //   bool bounced = false;

    //   if (location.x >= COLS)
    //   {
    //     location.x = COLS - 1;
    //     velocity.x *= -bounce;
    //     bounced = true;
    //   }
    //   else if (location.x < 0)
    //   {
    //     location.x = 0;
    //     velocity.x *= -bounce;
    //     bounced = true;
    //   }

    //   if (location.y >= ROWS)
    //   {
    //     location.y = ROWS - 1;
    //     velocity.y *= -bounce;
    //     bounced = true;
    //   }
    //   else if (location.y < 0)
    //   {
    //     location.y = 0;
    //     velocity.y *= -bounce;
    //     bounced = true;
    //   }
    //   return bounced;
    // }
    
  void wrapAroundBorders()
  {
    if (location.x < 0) location.x += VIRTUAL_ROWS;
    if (location.y < 0) location.y += VIRTUAL_COLS;
    if (location.x >= VIRTUAL_ROWS) location.x -= VIRTUAL_ROWS;
    if (location.y >= VIRTUAL_COLS) location.y -= VIRTUAL_COLS;
  }

    void avoidBorders()
    {
      PVector desired = velocity;

      if (location.x < 8)
        desired = PVector(maxspeed, velocity.y);
      if (location.x >= ROWS - 8)
        desired = PVector(-maxspeed, velocity.y);
      if (location.y < 8)
        desired = PVector(velocity.x, maxspeed);
      if (location.y >= COLS - 8)
        desired = PVector(velocity.x, -maxspeed);

      if (desired != velocity)
      {
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
    void render()
    {
    }
};
/// end of boid class
#pragma endregion
#pragma region
//start of pixel mapping functions
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
///end of pixel mapping functions
#pragma endregion

#pragma region 
///Pallet definitinos and swapping
const static TProgmemRGBPalette16 GreenAuroraColors_p FL_PROGMEM = {0x000000, 0x003300, 0x006600, 0x009900, 0x00cc00, 0x00ff00, 0x33ff00, 0x66ff00, 0x99ff00, 0xccff00, 0xffff00, 0xffcc00, 0xff9900, 0xff6600, 0xff3300, 0xff0000};
const static TProgmemRGBPalette16 WoodFireColors_p FL_PROGMEM = {CRGB::Black, 0x330e00, 0x661c00, 0x992900, 0xcc3700, CRGB::OrangeRed, 0xff5800, 0xff6b00, 0xff7f00, 0xff9200, CRGB::Orange, 0xffaf00, 0xffb900, 0xffc300, 0xffcd00, CRGB::Gold};             //* Orange
const static TProgmemRGBPalette16 NormalFire_p FL_PROGMEM = {CRGB::Black, 0x330000, 0x660000, 0x990000, 0xcc0000, CRGB::Red, 0xff0c00, 0xff1800, 0xff2400, 0xff3000, 0xff3c00, 0xff4800, 0xff5400, 0xff6000, 0xff6c00, 0xff7800};                             // пытаюсь сделать что-то более приличное
const static TProgmemRGBPalette16 NormalFire2_p FL_PROGMEM = {CRGB::Black, 0x560000, 0x6b0000, 0x820000, 0x9a0011, CRGB::FireBrick, 0xc22520, 0xd12a1c, 0xe12f17, 0xf0350f, 0xff3c00, 0xff6400, 0xff8300, 0xffa000, 0xffba00, 0xffd400};                      // пытаюсь сделать что-то более приличное
const static TProgmemRGBPalette16 LithiumFireColors_p FL_PROGMEM = {CRGB::Black, 0x240707, 0x470e0e, 0x6b1414, 0x8e1b1b, CRGB::FireBrick, 0xc14244, 0xd16166, 0xe08187, 0xf0a0a9, CRGB::Pink, 0xff9ec0, 0xff7bb5, 0xff59a9, 0xff369e, CRGB::DeepPink};        //* Red
const static TProgmemRGBPalette16 SodiumFireColors_p FL_PROGMEM = {CRGB::Black, 0x332100, 0x664200, 0x996300, 0xcc8400, CRGB::Orange, 0xffaf00, 0xffb900, 0xffc300, 0xffcd00, CRGB::Gold, 0xf8cd06, 0xf0c30d, 0xe9b913, 0xe1af1a, CRGB::Goldenrod};           //* Yellow
const static TProgmemRGBPalette16 CopperFireColors_p FL_PROGMEM = {CRGB::Black, 0x001a00, 0x003300, 0x004d00, 0x006600, CRGB::Green, 0x239909, 0x45b313, 0x68cc1c, 0x8ae626, CRGB::GreenYellow, 0x94f530, 0x7ceb30, 0x63e131, 0x4bd731, CRGB::LimeGreen};     //* Green
const static TProgmemRGBPalette16 ZAlcoholFireColors_p FL_PROGMEM = {CRGB::Black, 0x000033, 0x000066, 0x000099, 0x0000cc, CRGB::Blue, 0x0026ff, 0x004cff, 0x0073ff, 0x0099ff, CRGB::DeepSkyBlue, 0x1bc2fe, 0x36c5fd, 0x51c8fc, 0x6ccbfb, CRGB::LightSkyBlue};  //* Blue
const static TProgmemRGBPalette16 ZRubidiumFireColors_p FL_PROGMEM = {CRGB::Red, 0x0f001a, 0x1e0034, 0x2d004e, 0x3c0068, CRGB::Red, CRGB::Indigo, CRGB::Indigo, CRGB::Indigo, CRGB::Indigo, CRGB::Indigo, 0x3c0084, 0x2d0086, 0x1e0087, 0x0f0089, CRGB::Red};        //* Indigo
const static TProgmemRGBPalette16 darkishColors_p FL_PROGMEM = {0xFFcbbcaa,0xFF72ac92,0xFF797997,0xFF756372,0xFF638d54,0xFF606a40,0xFF4e4843,0xFF1a2f27,0xFFb6a588,0xFFa7889f,0xFFa17b6d,0xFF877041,0xFF725356,0xFF4e372c,0xFF3e2230,0xFF000000};
const static TProgmemRGBPalette16 sixteen1Colors_p FL_PROGMEM = {0xFF53437f,0xFFa89fcc,0xFFffffff,0xFFffd9e8,0xFFff9bb6,0xFF9968e2,0xFFbe9bff,0xFF7fceff,0xFF6d81ff,0xFF2c6f99,0xFF00bcaa,0xFFc48f9e,0xFF8e586f,0xFFff5470,0xFFff9b71,0xFFffd9ae};
const static TProgmemRGBPalette16 sixteen2Colors_p FL_PROGMEM = {0xFF4d004c,0xFF8f0076,0xFFc70083,0xFFf50078,0xFFff4764,0xFFff9393,0xFFffd5cc,0xFFfff3f0,0xFF000221,0xFF000769,0xFF00228f,0xFF0050c7,0xFF008bf5,0xFF00bbff,0xFF47edff,0xFF93fff8};
const static TProgmemRGBPalette16 sixteen3Colors_p FL_PROGMEM = {0xFF000000,0xFF7e7e7e,0xFFbebebe,0xFFffffff,0xFF7e0000,0xFFfe0000,0xFF047e00,0xFF06ff04,0xFF7e7e00,0xFFffff04,0xFF00007e,0xFF0000ff,0xFF7e007e,0xFFfe00ff,0xFF047e7e,0xFF06ffff};
const static TProgmemRGBPalette16 sixteen4Colors_p FL_PROGMEM = {0xFF2a1e1e,0xFF664933,0xFF28291d,0xFF576632,0xFF251d29,0xFF643266,0xFF291d22,0xFF663237,0xFF29221d,0xFF665932,0xFF1d2629,0xFF324b66,0xFF1d2920,0xFF32664d,0xFF1f1d29,0xFF483266};
const static TProgmemRGBPalette16 sixteen5Colors_p FL_PROGMEM = {0xFF313432,0xFF323e42,0xFF454b4b,0xFF3a5f3b,0xFF7c4545,0xFF675239,0xFF625055,0xFF516b43,0xFF796c64,0xFF718245,0xFF9e805c,0xFF998579,0xFFac9086,0xFFa6a296,0xFFb4ab8f,0xFFbcb7a5};
const static TProgmemRGBPalette16 sixteen6Colors_p FL_PROGMEM = {0X42243c,0X4e253d,0X59263d,0X64273a,0X6d2936,0X752d30,0X7c3228,0X803a1f,0X814214,0X804c04,0X7c5700,0X756100,0X6b6c00,0X5e7600,0X4b8000,0X2c8a00};
const static TProgmemRGBPalette16 sixteen7Colors_p FL_PROGMEM = {0Xff0000,0Xfd3500,0Xfa4e00,0Xf66300,0Xef7600,0Xe68600,0Xd4a300,0Xc9b000,0Xbdbc00,0Xb0c800,0Xa1d300,0X8fdf00,0X76ea00,0X54f500,0X04ff00,0X00ff00};
const static TProgmemRGBPalette16 sixteen8Colors_p FL_PROGMEM = {0X0036ff,0X5023e6,0X6904ce,0X7600b6,0X7b009a,0X7a0081,0X77006c,0X72005b,0X6c004c,0X65003e,0X5e0033,0X570028,0X50001e,0X450017,0X3b030f,0X300808};
const TProgmemRGBPalette16* currentPalette_p = &GreenAuroraColors_p;

void SetNewPalette(int _palcount)
{  
  switch (_palcount)
  {    
  case 0:    
    currentPalette_p = &GreenAuroraColors_p;
    break;
  case 1:
    currentPalette_p = &WoodFireColors_p;
    break;
  case 2:
    currentPalette_p = &NormalFire_p;
    break;
  case 3:
    currentPalette_p = &NormalFire2_p;
    break;
  case 4:
    currentPalette_p = &LithiumFireColors_p;
    break;
  case 5:
    currentPalette_p = &SodiumFireColors_p;
    break;
  case 6:
    currentPalette_p = &CopperFireColors_p;
    break;
  case 7:
    currentPalette_p = &ZAlcoholFireColors_p;
    break;
  case 8:
    currentPalette_p = &ZRubidiumFireColors_p;
    break;
  case 9:
    currentPalette_p = &PartyColors_p;
    break;
  case 10:
    currentPalette_p = &CloudColors_p;
    break;
  case 11:
    currentPalette_p = &LavaColors_p;
    break;
  case 12:
    currentPalette_p = &OceanColors_p;
    break;
  case 13:
    currentPalette_p = &ForestColors_p;
    break;
  case 14:
    currentPalette_p = &RainbowColors_p;
    break;
  case 15:
    currentPalette_p = &RainbowStripeColors_p;
    break; 
  case 16:
  currentPalette_p = &darkishColors_p;
  break;
  case 17:
  currentPalette_p = &sixteen1Colors_p;
  break;
  case 18:
  currentPalette_p = &sixteen2Colors_p;
  break;
  case 19:
  currentPalette_p = &sixteen3Colors_p;
  break;
  case 20:
  currentPalette_p = &sixteen4Colors_p;
  break;
  case 21:
  currentPalette_p = &sixteen5Colors_p;
  break;  
  case 22:
  currentPalette_p = &sixteen6Colors_p;
  break;
  case 23:
  currentPalette_p = &sixteen7Colors_p;
  break;
  case 24:
  currentPalette_p = &sixteen8Colors_p;
  break;

  default:
    currentPalette_p = &GreenAuroraColors_p;
    break;
  }
}
#pragma endregion
//end of pallet definitions and swapping
#pragma region 
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
  
  Attractor() {
    location = PVector((virtualViewX +12/ 2), (virtualViewY +12/ 2));
    mass = currmass;
    G = currgravity;
  } 
  void setlocation(float x, float y) {
    location = PVector(x,y);//virtualViewX+24/ 2), (virtualViewY +24/ 2));
  }
  void setMass(float m) {
    mass = m;
  }
  void setG(float g) {
    G = g;
  }
void incrementMass() {
  if (delaytimer > massdelaywait) {
    mass += (mass / 8 );// + random(1,2)* massdir;
    G += 0.5 * gdir;
    if (G > 4 || G < 0.50) gdir = -gdir;//G = mass;
  } 
  if (mass > massdelay) 
  {
    delaytimer = 0;
    mass = 1;
    massdelaywait = random(2,50);
    massdelay = random(300,400);
  }
  if (mass == 1) delaytimer += 1;
}
void incrementG() {
  if (G > 15) G = 0;
  G *= GStep * gdir;
}
  PVector attract(Boid m) {
    
    PVector force = location - m.location; // Calculate direction of force
    float d = force.mag(); // Distance between objects
    d = constrain(d, 15.0, 100.0); // Limiting the distance to eliminate "extreme" results for very close or very far objects
    force.normalize(); // Normalize vector (distance doesn't matter here, we just want this vector for direction)        
    float strength = (G * mass * m.mass) / (d * d); // Calculate gravitional force magnitude
    force *= strength; // Get force vector --> magnitude * direction
    return force;
  }
};
//Fades CRGB array towards the background color by amount.  
//fadeAmt > 102 breaks fade but has artistic value(?)

void fadeToColorBy(CRGB* leds, int count, CRGB color, uint8_t fadeAmt) {
    
    
    for (int x = 0; x < count; x++) {
        // don't know why, looks better when r is brought down 2.5 times faster, brought up half as fast
        if (leds[x].r < color.r) {
            leds[x].r = ((leds[x].r << 8) + (((int)(((color.r - leds[x].r) << 7) / rran) * fadeAmt / 255) << 1)) >> 8;
        }
        else {
            leds[x].r = ((leds[x].r << 8) + (((int)(((color.r - leds[x].r) << 7) * rran) * fadeAmt / 255) << 1)) >> 8;
        }
        leds[x].g = ((leds[x].g << 8) + ((((color.g - leds[x].g) << 7) * fadeAmt / 255) << 1)) >> 8;
        leds[x].b = ((leds[x].b << 8) + ((((color.b - leds[x].b) << 7) * fadeAmt / 255) << 1)) >> 8;
    }
}  // fadeToColorBy()

Attractor attractor5;
Attractor attractor1;

Boid boids[NUM_PARTICLES];
int degreestep = 1;
double degree;
int degreedir = 1;
int degreestepdir = 1;
//main functions 
void start()
{
  for (int i = 0; i < count; i++)
  {
    boids[i] = Boid(random(COLS), 0);
  }
   currentPalette_p = &GreenAuroraColors_p;
  attractor1.setlocation((virtualViewX+24/ 2), (virtualViewY+24/ 2));
  attractor1.setMass(100);
  attractor1.setG(4);
   attractor5.setlocation((virtualViewX+36/ 2), (virtualViewY+36/ 2));
  attractor5.setMass(1);
  attractor5.setG(0.1);
}
int massdir = 1;
float massstep = 0.5;

PVector center = PVector(36,36);
bool stopbool = false;
void movetoCenter()
{
  int countto = random(5,25);
    for (int j = 0; j < countto; j++){
      for (int i = 0; i < count; i++) {
          Boid *boid = &boids[i];
          PVector force1 = attractor1.attract(*boid);
          boid->applyForce(force1);
          boid->update(boids, count);
          boid->wrapAroundBorders();
          drawPixelXYF(boid->location.x, boid->location.y, ColorFromPalette(*currentPalette_p, boid->hue * 15, 255, LINEARBLEND_NOWRAP));
          boid->neighbordist = neidist;
          boid->desiredseparation = boidsep;
          if (stopbool == true) 
          {
            boid -> velocity = PVector(0,0);
          }
      }

      //fadeToBlackBy(leds, NUM_LEDS, fadebyvalue);  
        fadeToColorBy(leds,NUM_LEDS,CRGB::Black,40);
      LEDS.show();  
    }
}
void setNewRandomColorFacd()
{
  rran = random(1.5F,4.0F);
gran = random(1.5F, 4.0F);
bran = random(1.5F, 4.0F);
}


void draw() {
    if (firstPass) {
        firstPass = false;
        start();
    }

    EVERY_N_MILLISECONDS(100) {
    
      attractor5.incrementMass();
      // attractor4.incrementG();
        // if (virtualViewX < 0 || virtualViewX >= VIRTUAL_COLS - VIEWPORT_COLS)
        //     viewDirX = -viewDirX;
        // if (virtualViewY < 0 || virtualViewY >= VIRTUAL_ROWS - VIEWPORT_ROWS)
        //     viewDirY = -viewDirY;
        // virtualViewX += viewStep * viewDirX;
        // virtualViewY += viewStep * viewDirY;
          if (fadebyvalue < 100 || fadebyvalue > 120)
            fadebydir = -fadebydir;
        fadebyvalue += 1 * fadebydir;
    }
degreestep += 2;
if (degreestep > 10 || degreestep < 1) degreestepdir = -degreestepdir;

degree += degreestep * degreedir;
if (degree > 45 || degree < 1) degreedir = -degreedir;

       attractor5.location.rotateAroundPoint(center.x,center.y, degree);

    EVERY_N_MILLISECONDS(8000) {
        if (count <= 25 || count >= 60)
           countdir = -countdir;
        count += countstep * countdir;        
        if (maxspeed <= 0.9 || maxspeed >= 2.8)
           maxspeeddir = -maxspeeddir;
        maxspeed += maxspeedstep * maxspeeddir; 
    }
     EVERY_N_MILLISECONDS(10000)
  {
    palcount = random(0,24);
    SetNewPalette(palcount);  
    setNewRandomColorFacd();
  }
  int randomnum = random(0,100);
  int movetocenterrandom = random(0,200);
if (randomnum == 5) stopbool = true;
    for (int i = 0; i < count; i++) {
        Boid *boid = &boids[i];

        PVector force5 = attractor5.attract(*boid);
               boid->applyForce(force5);
        boid->update(boids, count);
        boid->wrapAroundBorders();
        boid-> brightness = map(boid->velocity.x + boid->velocity.y,0,5,50,255);

        drawPixelXYF(boid->location.x, boid->location.y, ColorFromPalette(*currentPalette_p, boid->hue * 15, boid->brightness, LINEARBLEND_NOWRAP));
        boid->neighbordist = neidist;
        boid->desiredseparation = boidsep;
        if (stopbool == true) 
        {
          boid -> velocity = PVector(0,0);
        }
    }

    //fadeToBlackBy(leds, NUM_LEDS, fadebyvalue);

    fadeToColorBy(leds,NUM_LEDS,CRGB::Black,40);
    if (movetocenterrandom == 100)
    {
      movetoCenter();
    }
    if (stopbool == true)
    {     
      stopbool = false;
    }
}

void setup()
{
  delay(3000);
  LEDS.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  LEDS.setBrightness(BRIGHTNESS);
 
}

void loop()
{
  draw();
  LEDS.show();  
 
}