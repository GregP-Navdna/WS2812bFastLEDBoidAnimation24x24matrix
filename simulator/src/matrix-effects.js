/**
 * MatrixEffects - Ported from matrix_effects.h
 * Visual effects: ripples, color wash, starfield, perlin noise, screen shake
 */

export class MatrixEffects {
  constructor(rows, cols) {
    this.rows = rows;
    this.cols = cols;

    // Ripple effect
    this.MAX_RIPPLES = 3;
    this.ripples = [];
    for (let i = 0; i < this.MAX_RIPPLES; i++) {
      this.ripples.push({ x: 0, y: 0, radius: 0, maxRadius: 0, color: 0, intensity: 0, active: false });
    }

    // Color wash
    this.washHue = 0;
    this.washDirection = 0;
    this.washSpeed = 5;
    this.washDensity = 20;
    this.washActive = false;

    // Starfield
    this.MAX_STARS = 15;
    this.stars = [];
    for (let i = 0; i < this.MAX_STARS; i++) {
      this.stars.push({ x: 0, y: 0, brightness: 0, maxBrightness: 0, fadeStep: 0 });
    }
    this.starfieldActive = false;

    // Screen shake
    this.shakeActive = false;
    this.shakeDuration = 0;
    this.shakeIntensity = 0;
    this.shakeOffsetX = 0;
    this.shakeOffsetY = 0;
    this.lastShakeUpdate = 0;

    // Perlin noise
    this.perlinActive = false;
    this.perlinScale = 0.05;
    this.perlinTimeScale = 0.02;
    this.perlinTime = 0.1;
    this.perlinHue = 0;
    this.perlinHueSpeed = 2;
    this.perlinBrightness = 128;
    this.lastWashUpdate = 0;
    this.lastPerlinHueUpdate = 0;

    // --- Meteor Shower ---
    this.MAX_METEORS = 5;
    this.meteors = [];
    for (let i = 0; i < this.MAX_METEORS; i++) {
      this.meteors.push({ x: 0, y: 0, dx: 0, dy: 0, length: 0, hue: 0, brightness: 0, life: 0, active: false });
    }
    this.meteorActive = false;
    this.lastMeteorSpawn = 0;

    // --- Plasma Wave ---
    this.plasmaActive = false;
    this.plasmaTime = 0;
    this.plasmaSpeed = 0.04;
    this.plasmaScale1 = 0.15;
    this.plasmaScale2 = 0.1;
    this.plasmaHueShift = 0;
    this.plasmaBrightness = 60;

    // --- Lightning ---
    this.MAX_BOLTS = 3;
    this.bolts = [];
    for (let i = 0; i < this.MAX_BOLTS; i++) {
      this.bolts.push({ segments: [], brightness: 0, life: 0, active: false });
    }

    // --- Breathing Pulse ---
    this.pulseActive = false;
    this.pulsePhase = 0;
    this.pulseSpeed = 0.03;
    this.pulseHue = 0;
    this.pulseMinBrightness = 5;
    this.pulseMaxBrightness = 50;

    // --- Spiral Wipe ---
    this.spiralActive = false;
    this.spiralAngle = 0;
    this.spiralSpeed = 0.06;
    this.spiralHue = 0;
    this.spiralWidth = 0.4;
    this.spiralArms = 2;
    this.spiralBrightness = 60;

    // --- Fire Shimmer ---
    this.fireActive = false;
    this.fireIntensity = new Float32Array(rows * cols);
    this.fireCooling = 55;
    this.fireSparking = 120;

    // --- Sparkle Burst ---
    this.MAX_SPARKLES = 30;
    this.sparkles = [];
    for (let i = 0; i < this.MAX_SPARKLES; i++) {
      this.sparkles.push({ x: 0, y: 0, brightness: 0, decay: 0, hue: 0, active: false });
    }
    this.sparkleActive = false;

    // --- Gravity Well Distortion ---
    this.gravWellActive = false;
    this.gravWellX = cols / 2;
    this.gravWellY = rows / 2;
    this.gravWellStrength = 0;
    this.gravWellMaxStrength = 4.0;
    this.gravWellPhase = 0;
    this.gravWellHue = 0;

    // Permutation table for Perlin noise (doubled to avoid overflow)
    this.perm = new Uint8Array(512);
    const p = [
      151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,
      140,36,103,30,69,142,8,99,37,240,21,10,23,190,6,148,
      247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,
      57,177,33,88,237,149,56,87,174,20,125,136,171,168,68,175,
      74,165,71,134,139,48,27,166,77,146,158,231,83,111,229,122,
      60,211,133,230,220,105,92,41,55,46,245,40,244,102,143,54,
      65,25,63,161,1,216,80,73,209,76,132,187,208,89,18,169,
      200,196,135,130,116,188,159,86,164,100,109,198,173,186,3,64,
      52,217,226,250,124,123,5,202,38,147,118,126,255,82,85,212,
      207,206,59,227,47,16,58,17,182,189,28,42,223,183,170,213,
      119,248,152,2,44,154,163,70,221,153,101,155,167,43,172,9,
      129,22,39,253,19,98,108,110,79,113,224,232,178,185,112,104,
      218,246,97,228,251,34,242,193,238,210,144,12,191,179,162,241,
      81,51,145,235,249,14,239,107,49,192,214,31,181,199,106,157,
      184,84,204,176,115,121,50,45,127,4,150,254,138,236,205,93,
      222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180
    ];
    for (let i = 0; i < 256; i++) {
      this.perm[i] = p[i];
      this.perm[i + 256] = p[i];
    }
  }

  // --- Ripple ---
  startRipple() {
    for (let i = 0; i < this.MAX_RIPPLES; i++) {
      if (!this.ripples[i].active) {
        this.ripples[i].x = Math.floor(Math.random() * this.cols);
        this.ripples[i].y = Math.floor(Math.random() * this.rows);
        this.ripples[i].radius = 0;
        this.ripples[i].maxRadius = 4 + Math.floor(Math.random() * (Math.min(this.rows, this.cols) / 2 - 4));
        this.ripples[i].color = Math.floor(Math.random() * 256);
        this.ripples[i].intensity = 150 + Math.floor(Math.random() * 105);
        this.ripples[i].active = true;
        break;
      }
    }
  }

  updateRipples(leds) {
    for (let i = 0; i < this.MAX_RIPPLES; i++) {
      const ripple = this.ripples[i];
      if (!ripple.active) continue;

      this._drawRipple(leds, ripple);
      ripple.radius++;
      ripple.intensity = Math.max(0, ripple.intensity - 5);

      if (ripple.radius >= ripple.maxRadius || ripple.intensity === 0) {
        ripple.active = false;
      }
    }
  }

  _drawRipple(leds, ripple) {
    // Bounding box optimization (not in original C++)
    const minX = Math.max(0, ripple.x - ripple.radius - 1);
    const maxX = Math.min(this.cols - 1, ripple.x + ripple.radius + 1);
    const minY = Math.max(0, ripple.y - ripple.radius - 1);
    const maxY = Math.min(this.rows - 1, ripple.y + ripple.radius + 1);

    for (let y = minY; y <= maxY; y++) {
      for (let x = minX; x <= maxX; x++) {
        const dx = x - ripple.x;
        const dy = y - ripple.y;
        const distance = Math.sqrt(dx * dx + dy * dy);

        if (Math.abs(distance - ripple.radius) < 1.0) {
          const fade = 1.0 - Math.abs(distance - ripple.radius);
          const idx = y * this.cols + x;
          // HSV to RGB approximation for the ripple color
          const rgb = this._hsvToRgb(ripple.color / 255, 240 / 255, (ripple.intensity * fade) / 255);
          leds[idx].r = Math.min(255, leds[idx].r + rgb.r);
          leds[idx].g = Math.min(255, leds[idx].g + rgb.g);
          leds[idx].b = Math.min(255, leds[idx].b + rgb.b);
        }
      }
    }
  }

  // --- Color Wash ---
  startColorWash(direction = -1, speed = -1, density = -1) {
    this.washActive = true;
    this.washHue = Math.random();
    this.washDirection = direction >= 0 ? Math.min(direction, 2) : Math.floor(Math.random() * 3);
    this.washSpeed = speed >= 0 ? speed : 3 + Math.floor(Math.random() * 7);
    this.washDensity = density >= 0 ? density : 15 + Math.floor(Math.random() * 15);
  }

  stopColorWash() {
    this.washActive = false;
  }

  updateColorWash(leds) {
    if (!this.washActive) return;

    const now = performance.now();
    if (now - this.lastWashUpdate > 50) {
      this.washHue = (this.washHue + 1 / 256) % 1.0;
      this.lastWashUpdate = now;
    }

    for (let y = 0; y < this.rows; y++) {
      for (let x = 0; x < this.cols; x++) {
        let hueOffset;
        switch (this.washDirection) {
          case 0: hueOffset = x * this.washDensity; break;
          case 1: hueOffset = y * this.washDensity; break;
          case 2: hueOffset = (x + y) * this.washDensity / 2; break;
        }

        const idx = y * this.cols + x;
        const hue = (this.washHue + hueOffset / 256) % 1.0;
        const rgb = this._hsvToRgb(hue, 200 / 255, 40 / 255);
        leds[idx].r = Math.min(255, leds[idx].r + rgb.r);
        leds[idx].g = Math.min(255, leds[idx].g + rgb.g);
        leds[idx].b = Math.min(255, leds[idx].b + rgb.b);
      }
    }
  }

  // --- Starfield ---
  startStarfield() {
    this.starfieldActive = true;
    for (let i = 0; i < this.MAX_STARS; i++) {
      if (Math.random() < 0.8) {
        this.stars[i].x = Math.floor(Math.random() * this.cols);
        this.stars[i].y = Math.floor(Math.random() * this.rows);
        this.stars[i].brightness = 20 + Math.floor(Math.random() * 60);
        this.stars[i].maxBrightness = 100 + Math.floor(Math.random() * 155);
        this.stars[i].fadeStep = 1 + Math.floor(Math.random() * 2);
      }
    }
  }

  stopStarfield() {
    this.starfieldActive = false;
  }

  updateStarfield(leds) {
    if (!this.starfieldActive) return;

    for (let i = 0; i < this.MAX_STARS; i++) {
      const star = this.stars[i];
      if (star.brightness === 0 && star.fadeStep === 0) {
        if (Math.random() < 0.05) {
          star.x = Math.floor(Math.random() * this.cols);
          star.y = Math.floor(Math.random() * this.rows);
          star.brightness = 10;
          star.maxBrightness = 100 + Math.floor(Math.random() * 155);
          star.fadeStep = 1 + Math.floor(Math.random() * 2);
        }
        continue;
      }

      star.brightness = Math.max(0, Math.min(star.brightness + star.fadeStep, star.maxBrightness));

      if (star.brightness >= star.maxBrightness) {
        star.fadeStep = -(1 + Math.floor(Math.random()));
      }

      if (star.brightness <= 0) {
        star.brightness = 0;
        star.fadeStep = 0;
      }

      const idx = star.y * this.cols + star.x;
      const variant = Math.floor(Math.random() * 3);
      const b = star.brightness;
      switch (variant) {
        case 0: // White
          leds[idx].r = Math.min(255, leds[idx].r + b);
          leds[idx].g = Math.min(255, leds[idx].g + b);
          leds[idx].b = Math.min(255, leds[idx].b + b);
          break;
        case 1: // Blueish
          leds[idx].r = Math.min(255, leds[idx].r + (b >> 1));
          leds[idx].g = Math.min(255, leds[idx].g + (b >> 1));
          leds[idx].b = Math.min(255, leds[idx].b + b);
          break;
        case 2: // Yellowish
          leds[idx].r = Math.min(255, leds[idx].r + b);
          leds[idx].g = Math.min(255, leds[idx].g + b);
          leds[idx].b = Math.min(255, leds[idx].b + (b >> 1));
          break;
      }
    }
  }

  // --- Screen Shake ---
  startScreenShake(duration = 10, intensity = 2) {
    this.shakeActive = true;
    this.shakeDuration = duration;
    this.shakeIntensity = intensity;
    this.lastShakeUpdate = performance.now();
  }

  updateScreenShake() {
    if (!this.shakeActive) return false;

    if (this.shakeDuration === 0) {
      this.shakeActive = false;
      this.shakeOffsetX = 0;
      this.shakeOffsetY = 0;
      return false;
    }

    const now = performance.now();
    if (now - this.lastShakeUpdate > 50) {
      this.shakeDuration--;
      this.shakeOffsetX = Math.floor(Math.random() * (2 * this.shakeIntensity + 1)) - this.shakeIntensity;
      this.shakeOffsetY = Math.floor(Math.random() * (2 * this.shakeIntensity + 1)) - this.shakeIntensity;
      this.lastShakeUpdate = now;
    }

    return true;
  }

  getShakeOffsets() {
    return { x: this.shakeOffsetX, y: this.shakeOffsetY };
  }

  // --- Perlin Noise ---
  startPerlinNoise(scale = 0.08, brightness = 128) {
    this.perlinActive = true;
    this.perlinScale = scale;
    this.perlinTime = 0.0;
    this.perlinHue = Math.random();
    this.perlinBrightness = brightness;
    this.perlinHueSpeed = (1 + Math.floor(Math.random() * 4)) / 255;
    this.perlinTimeScale = (1 + Math.floor(Math.random() * 9)) / 100;
  }

  stopPerlinNoise() {
    this.perlinActive = false;
  }

  _fade(t) { return t * t * t * (t * (t * 6 - 15) + 10); }
  _lerp(a, b, t) { return a + t * (b - a); }

  _grad(hash, x, y, z) {
    const h = hash & 15;
    const u = h < 8 ? x : y;
    const v = h < 4 ? y : (h === 12 || h === 14 ? x : z);
    return ((h & 1) === 0 ? u : -u) + ((h & 2) === 0 ? v : -v);
  }

  perlinNoise(x, y, z) {
    const X = Math.floor(x) & 255;
    const Y = Math.floor(y) & 255;
    const Z = Math.floor(z) & 255;

    x -= Math.floor(x);
    y -= Math.floor(y);
    z -= Math.floor(z);

    const u = this._fade(x);
    const v = this._fade(y);
    const w = this._fade(z);

    const A  = this.perm[X] + Y;
    const AA = this.perm[A] + Z;
    const AB = this.perm[A + 1] + Z;
    const B  = this.perm[X + 1] + Y;
    const BA = this.perm[B] + Z;
    const BB = this.perm[B + 1] + Z;

    const result = this._lerp(
      this._lerp(
        this._lerp(this._grad(this.perm[AA], x, y, z), this._grad(this.perm[BA], x - 1, y, z), u),
        this._lerp(this._grad(this.perm[AB], x, y - 1, z), this._grad(this.perm[BB], x - 1, y - 1, z), u),
        v
      ),
      this._lerp(
        this._lerp(this._grad(this.perm[AA + 1], x, y, z - 1), this._grad(this.perm[BA + 1], x - 1, y, z - 1), u),
        this._lerp(this._grad(this.perm[AB + 1], x, y - 1, z - 1), this._grad(this.perm[BB + 1], x - 1, y - 1, z - 1), u),
        v
      ),
      w
    );
    return (result + 1.0) / 2.0;
  }

  updatePerlinNoise(leds) {
    if (!this.perlinActive) return;

    this.perlinTime += this.perlinTimeScale;

    const now = performance.now();
    if (now - this.lastPerlinHueUpdate > 50) {
      this.perlinHue = (this.perlinHue + this.perlinHueSpeed) % 1.0;
      this.lastPerlinHueUpdate = now;
    }

    for (let y = 0; y < this.rows; y++) {
      for (let x = 0; x < this.cols; x++) {
        const noiseValue = this.perlinNoise(
          x * this.perlinScale,
          y * this.perlinScale,
          this.perlinTime
        );

        const brightness = 20 + Math.floor(noiseValue * (this.perlinBrightness - 20));
        const hue = (this.perlinHue + noiseValue * 32 / 256) % 1.0;

        const idx = y * this.cols + x;
        const rgb = this._hsvToRgb(hue, 240 / 255, brightness / 255);
        leds[idx].r = Math.min(255, leds[idx].r + rgb.r);
        leds[idx].g = Math.min(255, leds[idx].g + rgb.g);
        leds[idx].b = Math.min(255, leds[idx].b + rgb.b);
      }
    }
  }

  // =============================================
  // NEW EFFECTS
  // =============================================

  // --- Meteor Shower ---
  startMeteorShower() {
    this.meteorActive = true;
  }

  stopMeteorShower() {
    this.meteorActive = false;
    for (let i = 0; i < this.MAX_METEORS; i++) this.meteors[i].active = false;
  }

  _spawnMeteor() {
    for (let i = 0; i < this.MAX_METEORS; i++) {
      if (!this.meteors[i].active) {
        const m = this.meteors[i];
        // Random direction: mostly diagonal downward
        const angle = (Math.PI / 4) + (Math.random() - 0.5) * (Math.PI / 3);
        const speed = 0.4 + Math.random() * 0.6;
        m.dx = Math.cos(angle) * speed;
        m.dy = Math.sin(angle) * speed;
        // Start from top or left edge
        if (Math.random() < 0.6) {
          m.x = Math.random() * this.cols;
          m.y = -1;
        } else {
          m.x = -1;
          m.y = Math.random() * this.rows * 0.5;
        }
        m.length = 3 + Math.floor(Math.random() * 5);
        m.hue = Math.random();
        m.brightness = 180 + Math.floor(Math.random() * 75);
        m.life = 40 + Math.floor(Math.random() * 40);
        m.active = true;
        return;
      }
    }
  }

  updateMeteorShower(leds) {
    if (!this.meteorActive) return;

    const now = performance.now();
    if (now - this.lastMeteorSpawn > 300 + Math.random() * 500) {
      this._spawnMeteor();
      this.lastMeteorSpawn = now;
    }

    for (let i = 0; i < this.MAX_METEORS; i++) {
      const m = this.meteors[i];
      if (!m.active) continue;

      // Draw trail
      for (let t = 0; t < m.length; t++) {
        const tx = Math.floor(m.x - m.dx * t * 2);
        const ty = Math.floor(m.y - m.dy * t * 2);
        if (tx >= 0 && tx < this.cols && ty >= 0 && ty < this.rows) {
          const fade = 1.0 - (t / m.length);
          const idx = ty * this.cols + tx;
          const brightness = m.brightness * fade * fade;
          const rgb = this._hsvToRgb(m.hue, 0.3 + 0.4 * (1 - fade), brightness / 255);
          leds[idx].r = Math.min(255, leds[idx].r + rgb.r);
          leds[idx].g = Math.min(255, leds[idx].g + rgb.g);
          leds[idx].b = Math.min(255, leds[idx].b + rgb.b);
        }
      }

      // Draw bright head
      const hx = Math.floor(m.x);
      const hy = Math.floor(m.y);
      if (hx >= 0 && hx < this.cols && hy >= 0 && hy < this.rows) {
        const idx = hy * this.cols + hx;
        const headB = Math.min(255, m.brightness);
        leds[idx].r = Math.min(255, leds[idx].r + headB);
        leds[idx].g = Math.min(255, leds[idx].g + headB);
        leds[idx].b = Math.min(255, leds[idx].b + Math.floor(headB * 0.8));
      }

      m.x += m.dx;
      m.y += m.dy;
      m.life--;

      if (m.life <= 0 || m.x > this.cols + 2 || m.y > this.rows + 2 || m.x < -5 || m.y < -5) {
        m.active = false;
      }
    }
  }

  // --- Plasma Wave ---
  startPlasma() {
    this.plasmaActive = true;
    this.plasmaTime = 0;
    this.plasmaHueShift = Math.random();
    this.plasmaSpeed = 0.02 + Math.random() * 0.04;
    this.plasmaScale1 = 0.1 + Math.random() * 0.15;
    this.plasmaScale2 = 0.05 + Math.random() * 0.1;
    this.plasmaBrightness = 40 + Math.floor(Math.random() * 40);
  }

  stopPlasma() {
    this.plasmaActive = false;
  }

  updatePlasma(leds) {
    if (!this.plasmaActive) return;

    this.plasmaTime += this.plasmaSpeed;
    this.plasmaHueShift = (this.plasmaHueShift + 0.001) % 1.0;

    for (let y = 0; y < this.rows; y++) {
      for (let x = 0; x < this.cols; x++) {
        const v1 = Math.sin(x * this.plasmaScale1 + this.plasmaTime);
        const v2 = Math.sin(y * this.plasmaScale2 + this.plasmaTime * 1.3);
        const v3 = Math.sin((x * this.plasmaScale1 + y * this.plasmaScale2 + this.plasmaTime) * 0.7);
        const cx = x - this.cols / 2;
        const cy = y - this.rows / 2;
        const v4 = Math.sin(Math.sqrt(cx * cx + cy * cy) * 0.12 - this.plasmaTime * 0.8);

        const value = (v1 + v2 + v3 + v4) / 4.0; // -1 to 1
        const normalized = (value + 1.0) / 2.0;   // 0 to 1

        const hue = (this.plasmaHueShift + normalized * 0.3) % 1.0;
        const brightness = this.plasmaBrightness * (0.3 + normalized * 0.7);

        const idx = y * this.cols + x;
        const rgb = this._hsvToRgb(hue, 0.85, brightness / 255);
        leds[idx].r = Math.min(255, leds[idx].r + rgb.r);
        leds[idx].g = Math.min(255, leds[idx].g + rgb.g);
        leds[idx].b = Math.min(255, leds[idx].b + rgb.b);
      }
    }
  }

  // --- Lightning ---
  startLightning() {
    for (let i = 0; i < this.MAX_BOLTS; i++) {
      if (!this.bolts[i].active) {
        const bolt = this.bolts[i];
        bolt.segments = [];
        bolt.brightness = 255;
        bolt.life = 6 + Math.floor(Math.random() * 8);
        bolt.active = true;

        // Generate jagged bolt path
        let x = Math.floor(Math.random() * this.cols);
        let y = 0;
        bolt.segments.push({ x, y });

        while (y < this.rows - 1) {
          y += 1;
          x += Math.floor(Math.random() * 5) - 2;
          x = Math.max(0, Math.min(this.cols - 1, x));
          bolt.segments.push({ x, y });

          // Occasional branch
          if (Math.random() < 0.2 && bolt.segments.length < 40) {
            let bx = x;
            let by = y;
            for (let b = 0; b < 3 + Math.floor(Math.random() * 4); b++) {
              bx += Math.floor(Math.random() * 5) - 2;
              by += 1;
              if (bx < 0 || bx >= this.cols || by >= this.rows) break;
              bolt.segments.push({ x: bx, y: by, branch: true });
            }
          }
        }
        break;
      }
    }
  }

  updateLightning(leds) {
    for (let i = 0; i < this.MAX_BOLTS; i++) {
      const bolt = this.bolts[i];
      if (!bolt.active) continue;

      const flashIntensity = bolt.life > 3 ? bolt.brightness : bolt.brightness * (bolt.life / 6);

      for (const seg of bolt.segments) {
        if (seg.x >= 0 && seg.x < this.cols && seg.y >= 0 && seg.y < this.rows) {
          const idx = seg.y * this.cols + seg.x;
          const b = seg.branch ? flashIntensity * 0.5 : flashIntensity;
          // Lightning is white-blue
          leds[idx].r = Math.min(255, leds[idx].r + Math.floor(b * 0.7));
          leds[idx].g = Math.min(255, leds[idx].g + Math.floor(b * 0.75));
          leds[idx].b = Math.min(255, leds[idx].b + Math.floor(b));

          // Glow around main bolt
          if (!seg.branch) {
            for (let dx = -1; dx <= 1; dx++) {
              const nx = seg.x + dx;
              if (nx >= 0 && nx < this.cols) {
                const nidx = seg.y * this.cols + nx;
                const glow = b * 0.2;
                leds[nidx].r = Math.min(255, leds[nidx].r + Math.floor(glow * 0.5));
                leds[nidx].g = Math.min(255, leds[nidx].g + Math.floor(glow * 0.6));
                leds[nidx].b = Math.min(255, leds[nidx].b + Math.floor(glow));
              }
            }
          }
        }
      }

      bolt.life--;
      bolt.brightness = Math.max(0, bolt.brightness - 20);
      // Flicker
      if (Math.random() < 0.3) bolt.brightness = Math.min(255, bolt.brightness + 80);

      if (bolt.life <= 0) {
        bolt.active = false;
      }
    }
  }

  // --- Breathing Pulse ---
  startPulse() {
    this.pulseActive = true;
    this.pulsePhase = 0;
    this.pulseHue = Math.random();
    this.pulseSpeed = 0.02 + Math.random() * 0.03;
    this.pulseMaxBrightness = 30 + Math.floor(Math.random() * 40);
  }

  stopPulse() {
    this.pulseActive = false;
  }

  updatePulse(leds) {
    if (!this.pulseActive) return;

    this.pulsePhase += this.pulseSpeed;
    this.pulseHue = (this.pulseHue + 0.0005) % 1.0;

    const breath = (Math.sin(this.pulsePhase) + 1.0) / 2.0;
    const brightness = this.pulseMinBrightness + (this.pulseMaxBrightness - this.pulseMinBrightness) * breath;

    // Radial gradient from center
    const cx = this.cols / 2;
    const cy = this.rows / 2;
    const maxDist = Math.sqrt(cx * cx + cy * cy);

    for (let y = 0; y < this.rows; y++) {
      for (let x = 0; x < this.cols; x++) {
        const dx = x - cx;
        const dy = y - cy;
        const dist = Math.sqrt(dx * dx + dy * dy);
        const falloff = 1.0 - (dist / maxDist) * 0.6;
        const b = brightness * falloff;

        const idx = y * this.cols + x;
        const rgb = this._hsvToRgb(this.pulseHue, 0.7, b / 255);
        leds[idx].r = Math.min(255, leds[idx].r + rgb.r);
        leds[idx].g = Math.min(255, leds[idx].g + rgb.g);
        leds[idx].b = Math.min(255, leds[idx].b + rgb.b);
      }
    }
  }

  // --- Spiral Wipe ---
  startSpiral() {
    this.spiralActive = true;
    this.spiralAngle = 0;
    this.spiralHue = Math.random();
    this.spiralSpeed = 0.03 + Math.random() * 0.05;
    this.spiralArms = 1 + Math.floor(Math.random() * 4);
    this.spiralWidth = 0.3 + Math.random() * 0.4;
    this.spiralBrightness = 40 + Math.floor(Math.random() * 40);
  }

  stopSpiral() {
    this.spiralActive = false;
  }

  updateSpiral(leds) {
    if (!this.spiralActive) return;

    this.spiralAngle += this.spiralSpeed;
    this.spiralHue = (this.spiralHue + 0.002) % 1.0;

    const cx = this.cols / 2;
    const cy = this.rows / 2;

    for (let y = 0; y < this.rows; y++) {
      for (let x = 0; x < this.cols; x++) {
        const dx = x - cx;
        const dy = y - cy;
        const dist = Math.sqrt(dx * dx + dy * dy);
        const angle = Math.atan2(dy, dx);

        // Spiral function: arms that rotate
        const spiralVal = Math.sin(
          angle * this.spiralArms - dist * 0.4 + this.spiralAngle
        );

        if (spiralVal > (1.0 - this.spiralWidth * 2)) {
          const normalized = (spiralVal - (1.0 - this.spiralWidth * 2)) / (this.spiralWidth * 2);
          const brightness = this.spiralBrightness * normalized * Math.max(0.2, 1.0 - dist / 20);
          const hue = (this.spiralHue + dist * 0.02) % 1.0;

          const idx = y * this.cols + x;
          const rgb = this._hsvToRgb(hue, 0.9, brightness / 255);
          leds[idx].r = Math.min(255, leds[idx].r + rgb.r);
          leds[idx].g = Math.min(255, leds[idx].g + rgb.g);
          leds[idx].b = Math.min(255, leds[idx].b + rgb.b);
        }
      }
    }
  }

  // --- Fire Shimmer ---
  startFire() {
    this.fireActive = true;
    this.fireIntensity.fill(0);
    this.fireCooling = 40 + Math.floor(Math.random() * 30);
    this.fireSparking = 100 + Math.floor(Math.random() * 60);
  }

  stopFire() {
    this.fireActive = false;
  }

  updateFire(leds) {
    if (!this.fireActive) return;

    const w = this.cols;
    const h = this.rows;

    // Cool down every cell
    for (let i = 0; i < w * h; i++) {
      const cooldown = Math.random() * ((this.fireCooling * 10) / h);
      this.fireIntensity[i] = Math.max(0, this.fireIntensity[i] - cooldown);
    }

    // Heat rises: diffuse upward
    for (let y = 0; y < h - 1; y++) {
      for (let x = 0; x < w; x++) {
        const x1 = Math.max(0, x - 1);
        const x2 = Math.min(w - 1, x + 1);
        this.fireIntensity[y * w + x] =
          (this.fireIntensity[(y + 1) * w + x1] +
           this.fireIntensity[(y + 1) * w + x] * 2 +
           this.fireIntensity[(y + 1) * w + x2]) / 4.0;
      }
    }

    // Spark new fire at bottom
    for (let x = 0; x < w; x++) {
      if (Math.random() * 255 < this.fireSparking) {
        const idx = (h - 1) * w + x;
        this.fireIntensity[idx] = Math.min(255, this.fireIntensity[idx] + 160 + Math.random() * 95);
      }
    }

    // Render fire
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const heat = this.fireIntensity[y * w + x];
        if (heat < 2) continue;

        // Heat color: black -> red -> orange -> yellow -> white
        let r, g, b;
        const t = heat / 255;
        if (t < 0.33) {
          const f = t / 0.33;
          r = Math.floor(f * 200);
          g = 0;
          b = 0;
        } else if (t < 0.66) {
          const f = (t - 0.33) / 0.33;
          r = 200 + Math.floor(f * 55);
          g = Math.floor(f * 160);
          b = 0;
        } else {
          const f = (t - 0.66) / 0.34;
          r = 255;
          g = 160 + Math.floor(f * 95);
          b = Math.floor(f * 180);
        }

        // Scale down to be an overlay
        const scale = 0.35;
        const idx = y * this.cols + x;
        leds[idx].r = Math.min(255, leds[idx].r + Math.floor(r * scale));
        leds[idx].g = Math.min(255, leds[idx].g + Math.floor(g * scale));
        leds[idx].b = Math.min(255, leds[idx].b + Math.floor(b * scale));
      }
    }
  }

  // --- Sparkle Burst ---
  startSparkleBurst() {
    this.sparkleActive = true;
    // Burst from a random point
    const cx = Math.floor(Math.random() * this.cols);
    const cy = Math.floor(Math.random() * this.rows);
    const burstHue = Math.random();

    for (let i = 0; i < this.MAX_SPARKLES; i++) {
      const s = this.sparkles[i];
      const angle = Math.random() * Math.PI * 2;
      const dist = Math.random() * Math.min(this.rows, this.cols) * 0.6;
      s.x = cx + Math.cos(angle) * dist;
      s.y = cy + Math.sin(angle) * dist;
      s.brightness = 200 + Math.floor(Math.random() * 55);
      s.decay = 4 + Math.floor(Math.random() * 8);
      s.hue = (burstHue + (Math.random() - 0.5) * 0.1) % 1.0;
      if (s.hue < 0) s.hue += 1.0;
      s.active = true;
    }
  }

  updateSparkles(leds) {
    if (!this.sparkleActive) return;

    let anyActive = false;
    for (let i = 0; i < this.MAX_SPARKLES; i++) {
      const s = this.sparkles[i];
      if (!s.active) continue;
      anyActive = true;

      const ix = Math.floor(s.x);
      const iy = Math.floor(s.y);
      if (ix >= 0 && ix < this.cols && iy >= 0 && iy < this.rows) {
        const idx = iy * this.cols + ix;
        const rgb = this._hsvToRgb(s.hue, 0.4, s.brightness / 255);
        leds[idx].r = Math.min(255, leds[idx].r + rgb.r);
        leds[idx].g = Math.min(255, leds[idx].g + rgb.g);
        leds[idx].b = Math.min(255, leds[idx].b + rgb.b);
      }

      // Flicker
      if (Math.random() < 0.3) {
        s.brightness = Math.min(255, s.brightness + 40);
      }
      s.brightness -= s.decay;
      if (s.brightness <= 0) {
        s.active = false;
      }
    }

    if (!anyActive) {
      this.sparkleActive = false;
    }
  }

  // --- Gravity Well Distortion ---
  startGravityWell() {
    this.gravWellActive = true;
    this.gravWellX = this.cols * (0.2 + Math.random() * 0.6);
    this.gravWellY = this.rows * (0.2 + Math.random() * 0.6);
    this.gravWellStrength = 0;
    this.gravWellMaxStrength = 2.0 + Math.random() * 3.0;
    this.gravWellPhase = 0;
    this.gravWellHue = Math.random();
  }

  stopGravityWell() {
    this.gravWellActive = false;
  }

  updateGravityWell(leds) {
    if (!this.gravWellActive) return;

    this.gravWellPhase += 0.03;
    this.gravWellHue = (this.gravWellHue + 0.002) % 1.0;

    // Ramp up and down
    const lifePhase = Math.sin(this.gravWellPhase);
    this.gravWellStrength = this.gravWellMaxStrength * Math.max(0, lifePhase);

    if (this.gravWellPhase > Math.PI) {
      this.gravWellActive = false;
      return;
    }

    // Slowly orbit
    this.gravWellX += Math.sin(this.gravWellPhase * 2) * 0.1;
    this.gravWellY += Math.cos(this.gravWellPhase * 3) * 0.08;

    for (let y = 0; y < this.rows; y++) {
      for (let x = 0; x < this.cols; x++) {
        const dx = x - this.gravWellX;
        const dy = y - this.gravWellY;
        const dist = Math.sqrt(dx * dx + dy * dy);

        if (dist < 0.5) continue;

        // Warping ring effect
        const warpIntensity = this.gravWellStrength / (dist * 0.5 + 1);
        const ringDist = Math.abs(dist - 6 * this.gravWellStrength / this.gravWellMaxStrength);

        if (ringDist < 2.0) {
          const ringBrightness = (1.0 - ringDist / 2.0) * warpIntensity * 30;
          const hue = (this.gravWellHue + dist * 0.03) % 1.0;
          const idx = y * this.cols + x;
          const rgb = this._hsvToRgb(hue, 0.8, Math.min(1, ringBrightness / 255));
          leds[idx].r = Math.min(255, leds[idx].r + rgb.r);
          leds[idx].g = Math.min(255, leds[idx].g + rgb.g);
          leds[idx].b = Math.min(255, leds[idx].b + rgb.b);
        }

        // Center glow
        if (dist < 3) {
          const centerGlow = (1.0 - dist / 3.0) * this.gravWellStrength * 20;
          const idx = y * this.cols + x;
          leds[idx].r = Math.min(255, leds[idx].r + Math.floor(centerGlow));
          leds[idx].g = Math.min(255, leds[idx].g + Math.floor(centerGlow * 0.7));
          leds[idx].b = Math.min(255, leds[idx].b + Math.floor(centerGlow * 1.2));
        }
      }
    }
  }

  // --- Main update ---
  update(leds) {
    this.updateRipples(leds);
    this.updateColorWash(leds);
    this.updateStarfield(leds);
    this.updatePerlinNoise(leds);
    this.updateMeteorShower(leds);
    this.updatePlasma(leds);
    this.updateLightning(leds);
    this.updatePulse(leds);
    this.updateSpiral(leds);
    this.updateFire(leds);
    this.updateSparkles(leds);
    this.updateGravityWell(leds);
    this.updateScreenShake();
  }

  triggerRandomEffect() {
    const effect = Math.floor(Math.random() * 13);
    switch (effect) {
      case 0: this.startRipple(); break;
      case 1:
        if (this.washActive) this.stopColorWash();
        else this.startColorWash();
        break;
      case 2:
        if (this.starfieldActive) this.stopStarfield();
        else this.startStarfield();
        break;
      case 3: this.startScreenShake(); break;
      case 4:
        if (this.perlinActive) this.stopPerlinNoise();
        else this.startPerlinNoise();
        break;
      case 5:
        if (this.meteorActive) this.stopMeteorShower();
        else this.startMeteorShower();
        break;
      case 6:
        if (this.plasmaActive) this.stopPlasma();
        else this.startPlasma();
        break;
      case 7: this.startLightning(); break;
      case 8:
        if (this.pulseActive) this.stopPulse();
        else this.startPulse();
        break;
      case 9:
        if (this.spiralActive) this.stopSpiral();
        else this.startSpiral();
        break;
      case 10:
        if (this.fireActive) this.stopFire();
        else this.startFire();
        break;
      case 11: this.startSparkleBurst(); break;
      case 12: this.startGravityWell(); break;
    }
  }

  // HSV to RGB conversion (h, s, v all 0-1)
  _hsvToRgb(h, s, v) {
    let r, g, b;
    const i = Math.floor(h * 6);
    const f = h * 6 - i;
    const p = v * (1 - s);
    const q = v * (1 - f * s);
    const t = v * (1 - (1 - f) * s);

    switch (i % 6) {
      case 0: r = v; g = t; b = p; break;
      case 1: r = q; g = v; b = p; break;
      case 2: r = p; g = v; b = t; break;
      case 3: r = p; g = q; b = v; break;
      case 4: r = t; g = p; b = v; break;
      case 5: r = v; g = p; b = q; break;
    }

    return {
      r: Math.round(r * 255),
      g: Math.round(g * 255),
      b: Math.round(b * 255)
    };
  }
}
