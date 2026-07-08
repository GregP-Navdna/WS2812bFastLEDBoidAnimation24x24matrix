/**
 * Simulation - Main draw loop, ported from main.cpp
 * Manages boids, attractors, effects, and timing
 */
import { Vec2 } from './vec2.js';
import { Boid } from './boid.js';
import { Attractor } from './attractor.js';
import { SpatialGrid } from './spatial-grid.js';
import { MatrixEffects } from './matrix-effects.js';
import { ALL_PALETTES, colorFromPalette } from './palettes.js';

// Constants matching C++ defines
const ROWS = 24;
const COLS = 24;
const NUM_LEDS = ROWS * COLS;
const VIRTUAL_ROWS = 48;
const VIRTUAL_COLS = 48;
const VIEWPORT_ROWS = 24;
const VIEWPORT_COLS = 24;
const NUM_PARTICLES = 255;
const GRID_CELLS_X = 8;
const GRID_CELLS_Y = 8;
const MAX_ATTRACTORS = 17;
const NUM_ATTRACTOR_PATTERNS = 11;

export class Simulation {
  constructor() {
    // LED buffer
    this.leds = new Array(NUM_LEDS);
    for (let i = 0; i < NUM_LEDS; i++) {
      this.leds[i] = { r: 0, g: 0, b: 0 };
    }

    // Viewport
    this.virtualViewX = 24;
    this.virtualViewY = 24;

    // Boids
    this.boids = new Array(NUM_PARTICLES);
    this.count = 254;

    // Animation parameters
    this.fadebyvalue = 10 + Math.floor(Math.random() * 110);
    this.neidist = 3 + Math.floor(Math.random() * 2);
    this.boidsep = 3 + Math.floor(Math.random() * 2);
    this.neidistdir = 1;
    this.boidsepdir = 1;
    this.fadebydir = 1;
    this.paletteIndex = 0;
    this.palcount = 0;
    this.velocityBasedHue = false;
    this.countstep = 5;
    this.countdir = 1;
    this.maxspeedstep = 0.1;
    this.maxspeeddir = 1;
    this.maxspeed = 2.5;
    this.currgravity = 2.90;
    this.currmass = 40;
    this.rran = 1 + Math.floor(Math.random() * 3);

    // Degree rotation
    this.degreestep = 1;
    this.degree = 0;
    this.degreedir = 1;
    this.degreestepdir = 1;
    this.center = new Vec2(36, 36);

    // Palette
    this.currentPalette = ALL_PALETTES[0];

    // Spatial grid
    this.spatialGrid = new SpatialGrid(GRID_CELLS_X, GRID_CELLS_Y, VIRTUAL_ROWS, VIRTUAL_COLS);

    // Matrix effects
    this.matrixEffects = new MatrixEffects(ROWS, COLS);

    // Attractors
    this.attractors = [];
    this.attractorActive = new Array(MAX_ATTRACTORS).fill(false);
    this.attractorActive[0] = true;
    this.currentAttractorPattern = 0;

    // Explosion
    this.explosionRepulsor = new Attractor();
    this.explosionActive = false;
    this.explosionStartTime = 0;
    this.lastExplosionTime = 0;

    // Timing
    this.lastSlowDownTime = 0;
    this.nextSlowDownInterval = 10000 + Math.random() * 30000;
    this.isSlowingDown = false;
    this.isPaused = false;
    this.pauseStartTime = 0;
    this.pauseDuration = 0;

    this.lastAttractorChangeTime = 0;
    this.attractorChangeDuration = 10000 + Math.random() * 10000;

    this.lastRippleTime = 0;
    this.rippleInterval = 3000 + Math.random() * 5000;

    this.lastEffectTrigger = 0;
    this.lastPaletteChange = 0;
    this.lastParamUpdate = 0;
    this.lastCountUpdate = 0;

    this.stopbool = false;
    this.firstPass = true;
    this.running = true;
    this.frameCount = 0;

    // Initialize everything
    this._initAttractors();
    this._start();
  }

  _start() {
    for (let i = 0; i < this.count; i++) {
      this.boids[i] = new Boid(Math.random() * COLS, 0);
    }
    // Fill remaining with disabled boids
    for (let i = this.count; i < NUM_PARTICLES; i++) {
      this.boids[i] = new Boid(0, 0);
      this.boids[i].enabled = false;
    }

    // Initialize main attractors
    const a1 = this.attractors[1];
    a1.setLocation(this.virtualViewX + 12, this.virtualViewY + 12);
    a1.setMass(100);
    a1.setG(4);

    const a5 = this.attractors[0];
    a5.setLocation(this.virtualViewX + 18, this.virtualViewY + 18);
    a5.setMass(1);
    a5.setG(0.1);

    this.setAttractorPattern(0);
    this.lastAttractorChangeTime = performance.now();
    this.firstPass = false;
  }

  _initAttractors() {
    // Create all 17 attractors
    for (let i = 0; i < MAX_ATTRACTORS; i++) {
      this.attractors.push(new Attractor());
    }

    const centerX = this.virtualViewX + VIRTUAL_COLS / 2;
    const centerY = this.virtualViewY + VIRTUAL_ROWS / 2;

    // [0] Main central attractor
    // [1] Secondary central attractor

    // [2] Top Left
    this.attractors[2].setLocation(this.virtualViewX + 8, this.virtualViewY + 8);
    this.attractors[2].setMass(20); this.attractors[2].setG(1.5);
    this.attractors[2].setRadius(10.0, 80.0);
    this.attractors[2].color = { r: 0, g: 128, b: 0 };

    // [3] Top Right
    this.attractors[3].setLocation(this.virtualViewX + VIRTUAL_COLS - 8, this.virtualViewY + 8);
    this.attractors[3].setMass(20); this.attractors[3].setG(1.5);
    this.attractors[3].setRadius(10.0, 80.0);

    // [4] Bottom Left
    this.attractors[4].setLocation(this.virtualViewX + 8, this.virtualViewY + VIRTUAL_ROWS - 8);
    this.attractors[4].setMass(20); this.attractors[4].setG(1.5);
    this.attractors[4].setRadius(10.0, 80.0);

    // [5] Bottom Right
    this.attractors[5].setLocation(this.virtualViewX + VIRTUAL_COLS - 8, this.virtualViewY + VIRTUAL_ROWS - 8);
    this.attractors[5].setMass(20); this.attractors[5].setG(1.5);
    this.attractors[5].setRadius(10.0, 80.0);

    // [6] Central repulsor
    this.attractors[6].setLocation(centerX, centerY);
    this.attractors[6].setMass(50); this.attractors[6].setG(3.0);
    this.attractors[6].setRepulsor(true);
    this.attractors[6].setRadius(5.0, 120.0);

    // [7-10] Rotating square
    for (let i = 7; i <= 10; i++) {
      this.attractors[i].setLocation(centerX, centerY);
      this.attractors[i].setMass(25); this.attractors[i].setG(1.8);
      this.attractors[i].setRadius(10.0, 90.0);
    }

    // [11-12] Figure-8
    for (let i = 11; i <= 12; i++) {
      this.attractors[i].setLocation(centerX, centerY);
      this.attractors[i].setMass(30); this.attractors[i].setG(2.0);
      this.attractors[i].setRadius(10.0, 100.0);
    }

    // [13-15] Spiral
    for (let i = 13; i <= 15; i++) {
      this.attractors[i].setLocation(centerX, centerY);
      this.attractors[i].setMass(20); this.attractors[i].setG(1.5);
      this.attractors[i].setRadius(10.0, 85.0);
    }

    // [16] Vortex
    this.attractors[16].setLocation(centerX, centerY);
    this.attractors[16].setMass(40); this.attractors[16].setG(2.5);
    this.attractors[16].setRadius(5.0, 110.0);
    this.attractors[16].setVortex(true, 0.8);

    // Explosion repulsor
    this.explosionRepulsor.setLocation(centerX, centerY);
    this.explosionRepulsor.setMass(60);
    this.explosionRepulsor.setG(4.0);
    this.explosionRepulsor.setRepulsor(true);
    this.explosionRepulsor.setRadius(5.0, 100.0);
  }

  setAttractorPattern(patternIndex) {
    if (patternIndex < 0 || patternIndex >= NUM_ATTRACTOR_PATTERNS) return;

    for (let i = 0; i < MAX_ATTRACTORS; i++) {
      this.attractorActive[i] = false;
    }
    this.attractorActive[0] = true;

    switch (patternIndex) {
      case 0: break; // Default
      case 1: // Corners
        this.attractorActive[2] = this.attractorActive[3] = this.attractorActive[4] = this.attractorActive[5] = true;
        break;
      case 2: // Central repulsor + corners
        this.attractorActive[2] = this.attractorActive[3] = this.attractorActive[4] = this.attractorActive[5] = this.attractorActive[6] = true;
        break;
      case 3: // Diagonal TL-BR
        this.attractorActive[2] = this.attractorActive[5] = true;
        break;
      case 4: // Diagonal TR-BL
        this.attractorActive[3] = this.attractorActive[4] = true;
        break;
      case 5: // Central repulsor only
        this.attractorActive[6] = true;
        break;
      case 6: // Rotating square
        this.attractorActive[7] = this.attractorActive[8] = this.attractorActive[9] = this.attractorActive[10] = true;
        break;
      case 7: // Figure-8
        this.attractorActive[11] = this.attractorActive[12] = true;
        break;
      case 8: break; // Pulsing center
      case 9: // Spiral arms
        this.attractorActive[13] = this.attractorActive[14] = this.attractorActive[15] = true;
        break;
      case 10: // Vortex
        this.attractorActive[16] = true;
        break;
    }

    this.currentAttractorPattern = patternIndex;
  }

  _updateAttractors() {
    const now = performance.now();
    const centerX = this.virtualViewX + VIRTUAL_COLS / 2;
    const centerY = this.virtualViewY + VIRTUAL_ROWS / 2;

    // Main attractor orbit
    if (this.attractorActive[0]) {
      this.attractors[0].orbit(centerX, centerY, 10, 0.2, 0);
      this.attractors[0].incrementMass();
    }

    // Corner attractors orbit
    if (this.attractorActive[2]) {
      this.attractors[2].orbit(this.virtualViewX + VIRTUAL_COLS * 0.25, this.virtualViewY + VIRTUAL_ROWS * 0.25, 5, 0.5, 0);
    }
    if (this.attractorActive[3]) {
      this.attractors[3].orbit(this.virtualViewX + VIRTUAL_COLS * 0.75, this.virtualViewY + VIRTUAL_ROWS * 0.25, 5, 0.5, Math.PI / 2);
    }
    if (this.attractorActive[4]) {
      this.attractors[4].orbit(this.virtualViewX + VIRTUAL_COLS * 0.25, this.virtualViewY + VIRTUAL_ROWS * 0.75, 5, 0.5, Math.PI);
    }
    if (this.attractorActive[5]) {
      this.attractors[5].orbit(this.virtualViewX + VIRTUAL_COLS * 0.75, this.virtualViewY + VIRTUAL_ROWS * 0.75, 5, 0.5, Math.PI * 1.5);
    }

    // Pulse repulsor
    if (this.attractorActive[6]) {
      const pulseFactor = (Math.sin(now / 1000) + 1) / 2;
      this.attractors[6].setG(1.0 + pulseFactor * 3.0);
    }

    // Rotating square (7-10)
    if (this.attractorActive[7] || this.attractorActive[8] || this.attractorActive[9] || this.attractorActive[10]) {
      const squareRadius = 15;
      const squareSpeed = 0.4;
      const time = now / 1000;
      const offsets = [0, Math.PI / 2, Math.PI, Math.PI * 1.5];
      for (let i = 0; i < 4; i++) {
        if (this.attractorActive[7 + i]) {
          const angle = time * squareSpeed + offsets[i];
          this.attractors[7 + i].setLocation(
            centerX + squareRadius * Math.cos(angle),
            centerY + squareRadius * Math.sin(angle)
          );
        }
      }
    }

    // Figure-8 (11-12)
    if (this.attractorActive[11] || this.attractorActive[12]) {
      const time = now / 1000;
      const speed = 0.5;
      const width = 18;
      const height = 10;

      for (let i = 0; i < 2; i++) {
        if (this.attractorActive[11 + i]) {
          const t = time * speed + (i * Math.PI);
          const sinT = Math.sin(t);
          const cosT = Math.cos(t);
          const denom = 1.0 + sinT * sinT;
          if (denom > 0.1) {
            const scale = width / denom;
            this.attractors[11 + i].setLocation(
              centerX + scale * cosT,
              centerY + height * sinT * cosT
            );
          }
        }
      }
    }

    // Pulsing center (pattern 8)
    if (this.currentAttractorPattern === 8 && this.attractorActive[0]) {
      const pulseFactor = (Math.sin(now / 500) + 1) / 2;
      this.attractors[0].setMass(10 + pulseFactor * 60);
      this.attractors[0].setG(0.5 + pulseFactor * 3.5);
    }

    // Spiral (13-15)
    if (this.attractorActive[13] || this.attractorActive[14] || this.attractorActive[15]) {
      const spiralSpeed = 0.6;
      const spiralRadius = 16;
      const phaseOffsets = [0, (2 * Math.PI) / 3, (4 * Math.PI) / 3];
      for (let i = 0; i < 3; i++) {
        if (this.attractorActive[13 + i]) {
          this.attractors[13 + i].orbit(centerX, centerY, spiralRadius, spiralSpeed, phaseOffsets[i]);
        }
      }
    }

    // Vortex (16)
    if (this.attractorActive[16]) {
      this.attractors[16].orbit(centerX, centerY, 5, 0.15, 0);
      const pulseFactor = (Math.sin(now / 800) + 1) / 2;
      this.attractors[16].setG(1.5 + pulseFactor * 2.0);
    }

    // Explosion
    if (this.explosionActive) {
      if (now - this.explosionStartTime > 500) {
        this.explosionActive = false;
      }
    } else {
      if (now - this.lastExplosionTime > 30000 && Math.random() < 0.05) {
        this.explosionActive = true;
        this.explosionStartTime = now;
        this.lastExplosionTime = now;
        let explodeX = centerX + Math.random() * 20 - 10;
        let explodeY = centerY + Math.random() * 20 - 10;
        explodeX = Math.max(this.virtualViewX + 2, Math.min(explodeX, this.virtualViewX + VIRTUAL_COLS - 2));
        explodeY = Math.max(this.virtualViewY + 2, Math.min(explodeY, this.virtualViewY + VIRTUAL_ROWS - 2));
        this.explosionRepulsor.setLocation(explodeX, explodeY);
        this.matrixEffects.startScreenShake(8, 2);
        this.matrixEffects.startRipple();
        this.matrixEffects.startRipple();
      }
    }

    // Pattern change timer
    if (now - this.lastAttractorChangeTime > this.attractorChangeDuration) {
      let newPattern;
      let attempts = 0;
      do {
        newPattern = Math.floor(Math.random() * NUM_ATTRACTOR_PATTERNS);
        attempts++;
      } while (newPattern === this.currentAttractorPattern && attempts < 20);

      this.setAttractorPattern(newPattern);

      if (Math.random() < 0.3) {
        this.velocityBasedHue = !this.velocityBasedHue;
      }

      this.matrixEffects.startRipple();
      this.matrixEffects.startRipple();
      if (Math.random() < 0.7) {
        this.matrixEffects.startScreenShake(5, 1);
      }

      this.lastAttractorChangeTime = now;
      this.attractorChangeDuration = 10000 + Math.random() * 20000;
    }
  }

  _fadeToColorBy(fadeAmt) {
    for (let i = 0; i < NUM_LEDS; i++) {
      const led = this.leds[i];
      // Simplified fade toward black matching the SIMD behavior
      led.r = Math.max(0, Math.round(led.r - (led.r * fadeAmt) / 256));
      led.g = Math.max(0, Math.round(led.g - (led.g * fadeAmt) / 256));
      led.b = Math.max(0, Math.round(led.b - (led.b * fadeAmt) / 256));
    }
  }

  _drawPixelXYF(virtualX, virtualY, color) {
    const mappedX = virtualX - this.virtualViewX;
    const mappedY = virtualY - this.virtualViewY;

    if (mappedX >= 0 && mappedX < VIEWPORT_COLS && mappedY >= 0 && mappedY < VIEWPORT_ROWS) {
      // Wu's antialiased pixel
      const xx = (mappedX - Math.floor(mappedX)) * 255;
      const yy = (mappedY - Math.floor(mappedY)) * 255;
      const ix = 255 - xx;
      const iy = 255 - yy;

      const wu = [
        ((ix * iy + ix + iy) >> 8),
        ((xx * iy + xx + iy) >> 8),
        ((ix * yy + ix + yy) >> 8),
        ((xx * yy + xx + yy) >> 8)
      ];

      for (let i = 0; i < 4; i++) {
        const xn = Math.floor(mappedX) + (i & 1);
        const yn = Math.floor(mappedY) + ((i >> 1) & 1);

        if (xn >= 0 && xn < ROWS && yn >= 0 && yn < COLS) {
          const idx = yn * COLS + xn;
          const w = wu[i];
          this.leds[idx].r = Math.min(255, this.leds[idx].r + ((color.r * w) >> 8));
          this.leds[idx].g = Math.min(255, this.leds[idx].g + ((color.g * w) >> 8));
          this.leds[idx].b = Math.min(255, this.leds[idx].b + ((color.b * w) >> 8));
        }
      }
    }
  }

  _randomSlowDownAndSpeed() {
    if (!this.isSlowingDown && !this.isPaused) {
      this.isSlowingDown = true;
      this.matrixEffects.startScreenShake(8, 1);
    }

    if (this.isSlowingDown) {
      let allStopped = true;
      for (let i = 0; i < this.count; i++) {
        if (this.boids[i].maxspeed > 0.2) {
          this.boids[i].maxspeed -= 0.1;
          allStopped = false;
        } else {
          this.boids[i].maxspeed = 0;
        }
      }

      if (allStopped) {
        this.isSlowingDown = false;
        this.isPaused = true;
        this.pauseStartTime = performance.now();
        this.pauseDuration = 3000 + Math.random() * 12000;
        this.matrixEffects.startStarfield();
      }
    }

    if (this.isPaused && (performance.now() - this.pauseStartTime >= this.pauseDuration)) {
      this.isPaused = false;
      for (let i = 0; i < this.count; i++) {
        this.boids[i].maxspeed = 1.1 + Math.random() * 0.9;
      }
      this.lastSlowDownTime = performance.now();
      this.nextSlowDownInterval = 10000 + Math.random() * 30000;
      this.matrixEffects.stopStarfield();
      this.matrixEffects.startRipple();
      this.matrixEffects.startRipple();
    }
  }

  // Main draw frame - called every animation frame
  draw() {
    if (!this.running) return;

    const now = performance.now();
    const randomnum = Math.floor(Math.random() * 100);
    const movetocenterrandom = Math.floor(Math.random() * 200);
    if (randomnum === 5) this.stopbool = true;

    // Slow down effect
    if (!this.isSlowingDown && !this.isPaused && (now - this.lastSlowDownTime >= this.nextSlowDownInterval)) {
      this._randomSlowDownAndSpeed();
    }
    if (this.isSlowingDown || this.isPaused) {
      this._randomSlowDownAndSpeed();
    }

    // Random matrix effect trigger every ~15 seconds
    if (now - this.lastEffectTrigger > 15000) {
      if (Math.random() < 0.7) {
        this.matrixEffects.triggerRandomEffect();
      }
      this.lastEffectTrigger = now;
    }

    // Random ripples
    if (now - this.lastRippleTime > this.rippleInterval) {
      this.matrixEffects.startRipple();
      this.lastRippleTime = now;
      this.rippleInterval = 100 + Math.random() * 900;
    }

    // Get shake offsets
    const shake = this.matrixEffects.getShakeOffsets();

    // Clear and populate spatial grid
    this.spatialGrid.clear();
    for (let i = 0; i < this.count; i++) {
      const boid = this.boids[i];
      if (boid.enabled) {
        this.spatialGrid.insert(boid, boid.location.x, boid.location.y);
      }
    }

    // Update attractors
    this._updateAttractors();

    // Apply matrix effects
    this.matrixEffects.update(this.leds);

    // Fade
    this._fadeToColorBy(45);

    // Move to center occasionally
    if (movetocenterrandom === 100) {
      this.matrixEffects.startColorWash(1, 8, 25);
      // Simplified moveToCenter: just apply attractor1 force for a few iterations
      for (let j = 0; j < 5; j++) {
        for (let i = 0; i < this.count; i++) {
          const boid = this.boids[i];
          if (!boid.enabled) continue;
          const force = this.attractors[1].attract(boid);
          boid.applyForce(force);
        }
      }
      this.matrixEffects.stopColorWash();
      this.matrixEffects.startRipple();
      this.matrixEffects.startRipple();
    }

    if (this.stopbool) this.stopbool = false;

    // Parameter updates (every ~10ms equivalent, but we do it each frame since JS runs at ~16ms)
    if (now - this.lastParamUpdate > 10) {
      this.attractors[0].incrementMass();
      if (this.fadebyvalue < 10 || this.fadebyvalue > 120) this.fadebydir = -this.fadebydir;
      this.fadebyvalue += this.fadebydir;
      this.lastParamUpdate = now;
    }

    // Degree rotation
    this.degreestep += 3;
    if (this.degreestep > 10 || this.degreestep < 1) this.degreestepdir = -this.degreestepdir;
    this.degree += this.degreestep * this.degreedir;
    if (this.degree > 45 || this.degree < 1) this.degreedir = -this.degreedir;
    this.attractors[0].location.rotateAroundPoint(this.center.x, this.center.y, this.degree);

    // Count and speed updates (every ~500ms)
    if (now - this.lastCountUpdate > 500) {
      if (this.count <= 2 || this.count >= 254) this.countdir = -this.countdir;
      this.count += this.countstep * this.countdir;
      this.count = Math.max(2, Math.min(254, this.count));

      // Enable/disable boids based on count
      for (let i = 0; i < NUM_PARTICLES; i++) {
        this.boids[i].enabled = (i < this.count);
      }

      if (this.maxspeed <= 0.4 || this.maxspeed >= 2.5) this.maxspeeddir = -this.maxspeeddir;
      this.maxspeed += this.maxspeedstep * this.maxspeeddir;

      this.lastCountUpdate = now;
    }

    // Palette change (every ~10 seconds)
    if (now - this.lastPaletteChange > 10000) {
      this.palcount = Math.floor(Math.random() * 24);
      this.currentPalette = ALL_PALETTES[this.palcount];
      this.rran = 1 + Math.floor(Math.random() * 3);

      this.matrixEffects.startRipple();
      this.matrixEffects.startRipple();
      this.lastPaletteChange = now;
    }

    // Update and draw boids
    for (let i = 0; i < this.count; i++) {
      const boid = this.boids[i];
      if (!boid.enabled) continue;

      // Apply active attractor forces
      for (let j = 0; j < MAX_ATTRACTORS; j++) {
        if (this.attractorActive[j]) {
          const force = this.attractors[j].attract(boid);
          boid.applyForce(force);
        }
      }

      // Explosion force
      if (this.explosionActive) {
        const explosionForce = this.explosionRepulsor.attract(boid);
        boid.applyForce(explosionForce);
      }

      // Update boid
      boid.wrapAroundBorders(VIRTUAL_ROWS, VIRTUAL_COLS);
      const vx = boid.velocity.x;
      const vy = boid.velocity.y;
      boid.brightness = Math.floor(Math.max(25, Math.min(255, ((Math.abs(vx) + Math.abs(vy)) / 4.5) * 230 + 25)));
      boid.mass = (255 - this.count) / 6;
      boid.update(this.spatialGrid);

      // Draw with shake offset
      const drawX = boid.location.x + shake.x;
      const drawY = boid.location.y + shake.y;

      // Calculate hue
      let renderHue;
      if (this.velocityBasedHue) {
        const angle = Math.atan2(boid.velocity.y, boid.velocity.x);
        renderHue = Math.floor(((angle + Math.PI) / (2 * Math.PI)) * 255);
      } else {
        renderHue = (boid.hue * 15) % 256;
      }

      const color = colorFromPalette(this.currentPalette, renderHue, boid.brightness, false);
      this._drawPixelXYF(drawX, drawY, color);

      boid.neighbordist = this.neidist;
      boid.desiredseparation = this.boidsep;

      if (this.stopbool) {
        boid.velocity.set(0, 0);
      }
    }

    this.frameCount++;
  }

  // --- Public API for UI controls ---
  togglePause() {
    this.running = !this.running;
    return this.running;
  }

  setPattern(index) {
    this.setAttractorPattern(index);
  }

  setPalette(index) {
    if (index >= 0 && index < ALL_PALETTES.length) {
      this.currentPalette = ALL_PALETTES[index];
      this.palcount = index;
    }
  }

  triggerExplosion() {
    const centerX = this.virtualViewX + VIRTUAL_COLS / 2;
    const centerY = this.virtualViewY + VIRTUAL_ROWS / 2;
    this.explosionActive = true;
    this.explosionStartTime = performance.now();
    this.lastExplosionTime = performance.now();
    this.explosionRepulsor.setLocation(
      centerX + Math.random() * 20 - 10,
      centerY + Math.random() * 20 - 10
    );
    this.matrixEffects.startScreenShake(8, 2);
    this.matrixEffects.startRipple();
    this.matrixEffects.startRipple();
  }

  getStats() {
    return {
      boidCount: this.count,
      pattern: this.currentAttractorPattern,
      palette: this.palcount,
      fps: 0, // filled by main loop
      velocityHue: this.velocityBasedHue,
      maxspeed: this.maxspeed.toFixed(1)
    };
  }
}
