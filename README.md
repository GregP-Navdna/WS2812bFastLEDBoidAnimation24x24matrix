# WS2812B FastLED Boid Animation - 24x24 Matrix

An advanced, highly optimized boid flocking simulation for ESP32-S3 microcontrollers driving a 24x24 WS2812B RGB LED matrix. This project implements Craig Reynolds' boid algorithm with spatial partitioning, SIMD optimizations, dynamic attractors/repulsors, and stunning visual effects.

![Project Type](https://img.shields.io/badge/Type-Embedded%20Art-purple)
![Platform](https://img.shields.io/badge/Platform-ESP32--S3-blue)
![Framework](https://img.shields.io/badge/Framework-Arduino-teal)
![Library](https://img.shields.io/badge/Library-FastLED-orange)

## 📋 Table of Contents

- [Overview](#overview)
- [Hardware Requirements](#hardware-requirements)
- [Key Features](#key-features)
- [Architecture](#architecture)
- [Performance Optimizations](#performance-optimizations)
- [Visual Effects](#visual-effects)
- [Color Palettes](#color-palettes)
- [Installation](#installation)
- [Configuration](#configuration)
- [File Structure](#file-structure)
- [Technical Details](#technical-details)
- [Customization](#customization)

## 🎯 Overview

This project creates mesmerizing organic animations by simulating flocking behavior (boids) on an LED matrix. Each LED particle follows three fundamental rules:
- **Separation**: Avoid crowding neighbors
- **Alignment**: Steer toward the average heading of neighbors  
- **Cohesion**: Steer toward the average position of neighbors

The simulation operates on a 48x48 virtual canvas with a 24x24 viewport, allowing smooth edge-wrapping and dynamic camera movement. Multiple attractors and repulsors create complex, evolving patterns that never repeat.

## 🔧 Hardware Requirements

- **Microcontroller**: ESP32-S3 DevKit C-1 (or compatible)
- **LED Matrix**: 24x24 WS2812B addressable RGB LEDs (576 total LEDs)
- **Power Supply**: 5V DC capable of 30-35A (at full brightness white)
  - Typical usage: 10-15A at normal brightness and color patterns
  - Recommend injection points every 12-16 LEDs for uniform brightness
- **GPIO Pin**: Data line connected to GPIO 42 (configurable)
- **Serpentine Layout**: Code assumes serpentine wiring pattern

### Power Consumption Notes
- Each LED can draw up to 60mA at full white brightness
- Default brightness set to 255 (maximum) - adjust based on power supply
- Consider power injection to prevent voltage drop across the matrix

## ✨ Key Features

### Core Animation
- **Up to 255 Simultaneous Boids**: Dynamic particle count that adjusts during runtime
- **Virtual Canvas**: 48x48 simulation space mapped to 24x24 physical display
- **Edge Wrapping**: Seamless toroidal topology for continuous motion
- **Wu's Algorithm**: Anti-aliased sub-pixel rendering for smooth particle trails
- **Dynamic Parameters**: Speed, separation distance, and neighbor distance vary over time

### Attractor System
- **7 Independent Attractors/Repulsors**: Create complex force fields
- **6 Predefined Patterns**: Automatically cycle through different configurations
  - Single central attractor
  - Four corner attractors
  - Central repulsor with corner attractors
  - Diagonal combinations
  - Repulsor-only mode
- **Orbital Motion**: Attractors orbit in circular patterns for dynamic behavior
- **Adaptive Strength**: Mass and gravitational constant modulate over time

### Visual Effects Engine
- **Ripple Effects**: Expanding circular waves triggered by pattern changes
- **Screen Shake**: Subtle camera shake for impact moments
- **Starfield**: Twinkling star overlay during pause sequences
- **Color Wash**: Directional color gradients (horizontal/vertical/diagonal)
- **Perlin Noise**: Smooth procedural noise overlay for organic aesthetics
- **Random Triggering**: Effects activate autonomously based on timing

### Behavioral Dynamics
- **Random Slowdown**: Periodic dramatic slow-motion sequences
- **Pause State**: Complete stop with starfield effect (3-15 seconds)
- **Move to Center**: Occasional pull toward central attractor
- **Palette Rotation**: Automatic color scheme changes every 10 seconds

## 🏗️ Architecture

### Modular Design
The codebase is organized into specialized components:

```
src/
├── main.cpp              # Main animation loop and initialization
├── boid.h                # Boid class with flocking algorithms
├── vec2.h                # 2D vector mathematics template
├── spatial_grid.h        # Spatial partitioning system
├── lookup_tables.h       # Pre-computed trigonometry tables
├── simd_utils.h          # ESP32 SIMD optimizations
├── matrix_effects.h      # Visual effects manager
├── palettes.h            # Color palette declarations
└── palettes.cpp          # Palette switching logic
```

### Data Flow
1. **Initialization**: Spatial grid and boids spawn randomly
2. **Grid Population**: Each frame, boids are inserted into spatial grid cells
3. **Force Calculation**: Attractors apply gravitational forces
4. **Flocking Update**: Boids query nearby neighbors from spatial grid
5. **Movement**: Physics integration updates positions
6. **Rendering**: Wu's algorithm draws sub-pixel positions
7. **Effects**: Matrix effects overlay applied
8. **Fade**: Trail fading to black
9. **Display**: FastLED.show() updates physical LEDs

## ⚡ Performance Optimizations

### Spatial Grid Partitioning
- **O(n) Complexity**: Reduces neighbor lookup from O(n²) to O(n)
- **8x8 Grid**: Virtual space divided into 64 cells (6x6 pixels each)
- **Neighbor Queries**: Only check adjacent cells for interactions
- **Memory Efficiency**: Pre-allocated vectors with reserved capacity

### SIMD Optimizations
- **Batch Processing**: LEDs processed in groups of 4
- **Cache Utilization**: Improved memory access patterns
- **ESP32 DSP**: Leverages hardware acceleration where available
- **Custom Fade Function**: Optimized RGB interpolation

### Lookup Tables
- **Pre-computed Sin/Cos**: 256-entry tables for trigonometry
- **Fast Square Root**: Table-based sqrt for distances < 1024
- **Quake III Fast Inverse Sqrt**: For vector normalization
- **Fast Atan2**: Approximation for angle calculations

### Memory Management
- **Static Allocation**: No dynamic memory during runtime
- **PROGMEM Storage**: Palettes and lookup tables in flash
- **Minimal Heap Usage**: Spatial grid pre-allocated in setup()
- **Efficient Vectors**: Reserved capacity prevents reallocation

## 🎨 Visual Effects

### Ripple System
- **Multi-Ripple**: Up to 3 concurrent expanding waves
- **Configurable Properties**: Random center, radius, color, intensity
- **Natural Decay**: Intensity fades as ripples expand
- **Triggered Events**: Pattern changes, palette swaps, center movements

### Screen Shake
- **Configurable Duration**: Typically 5-10 frames
- **Intensity Levels**: 1-3 pixel displacement
- **Random Offsets**: Different X/Y displacement each update
- **Visual Impact**: Emphasizes dramatic moments

### Starfield
- **15 Independent Stars**: Twinkling at various brightnesses
- **Fade In/Out**: Smooth brightness modulation
- **Color Variety**: White, blue-tinted, yellow-tinted stars
- **Pause Overlay**: Active during slow-motion pause sequences

### Color Wash
- **Three Directions**: Horizontal, vertical, diagonal sweeps
- **Hue Rotation**: Slowly shifting base color
- **Density Control**: Adjustable gradient steepness
- **Additive Blending**: Overlay on existing animation

### Perlin Noise
- **3D Noise**: Time-varying pattern for organic motion
- **Configurable Scale**: Adjust noise frequency
- **Color Mapping**: Noise values drive hue and brightness
- **Smooth Animation**: Continuous evolution over time

## 🌈 Color Palettes

### Fire Palettes (0-8)
- **Green Aurora**: Aurora borealis-inspired greens
- **Wood Fire**: Orange and yellow flames
- **Normal Fire**: Classic red-orange fire
- **Lithium Fire**: Pink and magenta (chemical fire)
- **Sodium Fire**: Yellow-gold (chemical fire)
- **Copper Fire**: Green flames (chemical fire)
- **Alcohol Fire**: Blue flames
- **Rubidium Fire**: Deep purple-indigo

### Standard Palettes (9-15)
- **Party**: Rainbow party colors
- **Cloud**: Soft blues and whites
- **Lava**: Molten rock reds and oranges
- **Ocean**: Deep sea blues and teals
- **Forest**: Earth tones and greens
- **Rainbow**: Full spectrum
- **Rainbow Stripes**: Banded rainbow

### Custom Palettes (16-24)
- **Darkish**: Muted earth tones
- **Sixteen 1-8**: Various artistic color schemes

**Palette Switching**: Automatically changes every 10 seconds with random selection from 25 palettes.

## 📥 Installation

### Prerequisites
- [PlatformIO](https://platformio.org/) (recommended) or Arduino IDE
- [FastLED Library](https://github.com/FastLED/FastLED) v3.6.0+

### PlatformIO (Recommended)
```bash
# Clone the repository
git clone <repository-url>
cd WS2812bFastLEDBoidAnimation24x24matrix

# Build and upload
pio run --target upload

# Open serial monitor (if enabled)
pio device monitor
```

### Arduino IDE
1. Install FastLED library via Library Manager
2. Open `src/main.cpp` in Arduino IDE
3. Copy all `.h` files from `src/` to the sketch folder
4. Select **ESP32-S3 Dev Module** as board
5. Compile and upload

### First Boot
- **3-Second Delay**: Built-in startup delay for safety
- **Serial Output**: Disabled by default (uncomment in `setup()` if needed)
- **Initial Pattern**: Single central attractor with random palette

## ⚙️ Configuration

### Key Constants (in `main.cpp`)

```cpp
// LED Configuration
#define LED_PIN 42           // GPIO pin for data line
#define BRIGHTNESS 255       // Global brightness (0-255)
#define LED_TYPE WS2812B     // LED chipset
#define COLOR_ORDER GRB      // Color channel order

// Matrix Dimensions
const uint8_t ROWS = 24;
const uint8_t COLS = 24;
const uint8_t VIRTUAL_ROWS = 48;
const uint8_t VIRTUAL_COLS = 48;

// Particle Configuration
#define NUM_PARTICLES 255    // Maximum boid count

// Spatial Grid
const int GRID_CELLS_X = 8;
const int GRID_CELLS_Y = 8;

// Behavior Timing
attractorChangeDuration = 15000;     // 15s between pattern changes
nextSlowDownInterval = 10000-40000;  // 10-40s between slowdowns
pauseDuration = 3000-15000;          // 3-15s pause length
```

### Pin Configuration
Change `LED_PIN` to match your hardware wiring. Ensure the GPIO pin supports output and isn't used by the ESP32-S3 for boot configuration.

### Performance Tuning
- **Reduce `NUM_PARTICLES`**: Lower particle count for slower microcontrollers
- **Adjust `GRID_CELLS_X/Y`**: Smaller grid cells = more precise collisions but higher overhead
- **Lower `BRIGHTNESS`**: Reduces power consumption and heat generation
- **Disable Effects**: Comment out effect triggers in `draw()` for pure boid animation

## 📁 File Structure

### Core Files

**main.cpp** (853 lines)
- Setup and main animation loop
- Attractor management and pattern switching
- Timing control (slowdown, pause, center movement)
- Palette rotation
- Integration of all subsystems

**boid.h** (354 lines)
- Boid class definition
- Flocking algorithms (separation, alignment, cohesion)
- Dual update methods: traditional and spatial-grid optimized
- Boundary handling (wrapping, avoiding, constraining)
- Force application and physics integration

**vec2.h** (186 lines)
- Template-based 2D vector class
- Operator overloading for vector math
- Fast trigonometry using lookup tables
- Normalization, rotation, distance calculations
- Quake III fast inverse square root

**spatial_grid.h** (92 lines)
- Spatial partitioning data structure
- O(1) insertion into grid cells
- Efficient neighbor queries
- Pre-allocated memory management
- Pointer-based to avoid circular dependencies

**lookup_tables.h** (163 lines)
- 256-entry sin/cos tables (PROGMEM)
- 256-entry sqrt table for values 0-1023
- Fast approximation functions
- Radian-to-degree normalization

**simd_utils.h** (72 lines)
- ESP32 SIMD initialization
- Batch LED processing (4 at a time)
- Optimized fade-to-color function
- Cache-friendly memory access

**matrix_effects.h** (499 lines)
- MatrixEffects class managing all visual effects
- Ripple generation and rendering
- Starfield system with twinkling
- Screen shake implementation
- Color wash (directional gradients)
- Perlin noise overlay
- Effect state management

**palettes.h / palettes.cpp** (124 lines total)
- 25 pre-defined color palettes
- PROGMEM storage for memory efficiency
- Palette switching function
- Includes custom and FastLED built-in palettes

## 🔬 Technical Details

### Boid Flocking Algorithm
The implementation follows Craig Reynolds' classic boid rules:

**Separation**
```cpp
PVector steer = (location - neighbor.location).normalize() / distance
```
Produces a steering force away from nearby boids, weighted by inverse distance.

**Alignment**
```cpp
PVector avgVelocity = sum(neighbor.velocity) / count
PVector steer = (avgVelocity.normalize() * maxspeed) - velocity
```
Steers toward the average heading of local neighbors.

**Cohesion**
```cpp
PVector centerOfMass = sum(neighbor.location) / count
PVector steer = seek(centerOfMass)
```
Steers toward the center of the local flock.

Force weights: **Separation: 3.5x**, **Alignment: 1.0x**, **Cohesion: 1.0x**

### Attractor Physics
```cpp
force = (G * mass_attractor * mass_boid) / (distance²)
direction = (attractor.location - boid.location).normalize()
```
Simplified gravitational attraction with distance constraints to prevent extreme forces.

### Wu's Anti-Aliasing
```cpp
intensity = (1 - fractional_x) * (1 - fractional_y)  // For each of 4 pixels
color_contribution = base_color * intensity
```
Distributes particle color across 4 pixels based on sub-pixel position for smooth trails.

### Virtual to Physical Mapping
```cpp
mappedX = virtualX - virtualViewX  // Viewport offset
mappedY = virtualY - virtualViewY
```
The 48x48 virtual canvas is 2x the physical display size, with the viewport currently centered at (24, 24).

### Serpentine Pixel Mapping
```cpp
if (y is odd) {
    index = (y * ROWS) + ((ROWS - 1) - x)  // Reversed
} else {
    index = (y * ROWS) + x  // Normal
}
```
Zigzag wiring pattern reduces data line length and improves signal integrity.

## 🎛️ Customization

### Adjusting Boid Behavior

**More Cohesive Flocking**
```cpp
// In boid.h update() method
sep *= 2.0;   // Less separation
ali *= 2.0;   // More alignment
coh *= 2.0;   // More cohesion
```

**Faster/Slower Boids**
```cpp
// In main.cpp
maxspeed = 1.5;  // Slower (default: 2.5)
maxspeed = 4.0;  // Faster
```

**Tighter/Looser Flocks**
```cpp
neighbordistance = 4;  // Tighter (default: 6)
boidseperation = 4;    // More spacing (default: 2)
```

### Adding New Attractors
```cpp
// In main.cpp - add to initAttractorPatterns()
Attractor myAttractor;
myAttractor.setlocation(x, y);
myAttractor.setMass(30);
myAttractor.setG(2.0);
myAttractor.setRadius(5.0, 100.0);

// Add to attractorArray
attractorArray[7] = &myAttractor;

// Create new pattern in setAttractorPattern()
case 6:
    attractorActive[7] = true;
    break;
```

### Creating Custom Palettes
```cpp
// In palettes.h
const static TProgmemRGBPalette16 MyCustomPalette_p FL_PROGMEM = {
    CRGB::Black, CRGB::DarkBlue, CRGB::Blue, CRGB::Cyan,
    CRGB::Aqua, CRGB::Turquoise, CRGB::Teal, CRGB::Green,
    // ... 8 more colors for 16 total
};

// In palettes.cpp - add case
case 25:
    currentPalette_p = &MyCustomPalette_p;
    break;
```

### Disabling Effects
```cpp
// In draw() function - comment out unwanted triggers
// EVERY_N_SECONDS(15) {
//     matrixEffects->triggerRandomEffect();
// }

// Disable slowdown
// if (!isSlowingDown && !isPaused...) {
//     randomSlowDownAndSpeed();
// }
```

### Changing Matrix Size
```cpp
// Update dimensions
const uint8_t ROWS = 16;  // New physical size
const uint8_t COLS = 16;
const uint8_t VIRTUAL_ROWS = 32;  // Should be 2x physical
const uint8_t VIRTUAL_COLS = 32;

// Adjust viewport center
virtualViewX = 16;
virtualViewY = 16;

// Update NUM_LEDS calculation (automatic)
```

## 📊 Performance Metrics

**Typical Frame Rate**: 30-60 FPS (depends on particle count and active effects)

**Memory Usage**:
- Static RAM: ~40KB (boid array, LED buffer, spatial grid)
- Flash: ~300KB (code, palettes, lookup tables)
- Heap: <5KB (spatial grid vectors)

**Computational Complexity**:
- Without Spatial Grid: O(n²) for n boids
- With Spatial Grid: O(n) average case
- Worst Case: O(n·k) where k = average neighbors per cell

**Power Draw** (5V supply):
- Idle/Dark: ~0.5A
- Typical Animation: 8-12A
- Full White Maximum: 30-35A

## 🐛 Troubleshooting

**LEDs Don't Light Up**
- Check GPIO pin number matches wiring
- Verify 5V power supply is connected
- Confirm WS2812B data line polarity
- Try lowering BRIGHTNESS to 64 for testing

**Flickering or Glitching**
- Add 100-470Ω resistor on data line
- Verify power supply can handle current draw
- Check for loose connections
- Enable power injection for larger matrices

**Slow Frame Rate**
- Reduce NUM_PARTICLES to 128 or lower
- Increase GRID_CELLS_X/Y to 12x12
- Disable some matrix effects
- Ensure ESP32-S3 is running at 240MHz

**Uneven Brightness**
- Implement power injection every 12-16 LEDs
- Use thicker gauge wire for power rails
- Reduce overall BRIGHTNESS setting
- Check voltage drop across matrix

**Compilation Errors**
- Ensure FastLED 3.6.0+ is installed
- Verify all .h files are in src/ directory
- Check PlatformIO platform version is current
- Try cleaning build cache: `pio run --target clean`

## 🤝 Contributing

Contributions are welcome! Areas for improvement:
- Additional attractor patterns
- New visual effects
- Color palette designs
- Performance optimizations
- Documentation enhancements
- Hardware configurations

## 📜 License

This project is released under the MIT License. Feel free to use, modify, and distribute.

## 🙏 Acknowledgments

- **Craig Reynolds**: Original boid flocking algorithm
- **FastLED Team**: Excellent LED control library
- **Processing Foundation**: Vector math inspiration
- **Quake III**: Fast inverse square root algorithm
- **Ken Perlin**: Perlin noise algorithm

## 📧 Contact

For questions, suggestions, or showcasing your build, please open an issue on the repository.

---

**Built with ❤️ for the LED art community**
