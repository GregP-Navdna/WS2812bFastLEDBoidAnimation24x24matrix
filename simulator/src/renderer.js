/**
 * Renderer - Canvas-based 24x24 LED grid renderer
 * Draws the LED buffer to an HTML5 Canvas, simulating pixel appearance
 */

export class Renderer {
  constructor(canvas, rows, cols) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.rows = rows;
    this.cols = cols;

    // LED appearance settings
    this.ledGap = 2;        // Gap between LEDs in pixels
    this.ledRounding = 2;   // Border radius of each LED
    this.showGrid = true;    // Show grid lines

    this.resize();
  }

  resize() {
    // Calculate LED size based on canvas container
    const container = this.canvas.parentElement;
    const availableSize = Math.min(container.clientWidth, container.clientHeight) - 20;
    const totalGap = this.ledGap * (this.cols + 1);
    this.ledSize = Math.floor((availableSize - totalGap) / this.cols);
    
    // Ensure minimum LED size
    this.ledSize = Math.max(this.ledSize, 8);

    // Update canvas size
    const canvasSize = this.ledSize * this.cols + this.ledGap * (this.cols + 1);
    this.canvas.width = canvasSize;
    this.canvas.height = canvasSize;
  }

  /**
   * Render the LED buffer to the canvas
   * @param {Array} leds - Array of {r, g, b} objects, length = rows * cols
   */
  render(leds) {
    const ctx = this.ctx;
    const size = this.ledSize;
    const gap = this.ledGap;

    // Dark background (PCB color)
    ctx.fillStyle = '#111111';
    ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);

    for (let y = 0; y < this.rows; y++) {
      for (let x = 0; x < this.cols; x++) {
        const idx = y * this.cols + x;
        const led = leds[idx];

        const px = gap + x * (size + gap);
        const py = gap + y * (size + gap);

        // LED base (dark)
        ctx.fillStyle = '#0a0a0a';
        this._roundRect(ctx, px, py, size, size, this.ledRounding);
        ctx.fill();

        // LED glow
        if (led.r > 2 || led.g > 2 || led.b > 2) {
          // Inner LED color
          ctx.fillStyle = `rgb(${led.r}, ${led.g}, ${led.b})`;
          this._roundRect(ctx, px + 1, py + 1, size - 2, size - 2, this.ledRounding);
          ctx.fill();

          // Bloom/glow effect for bright LEDs
          const brightness = (led.r + led.g + led.b) / 3;
          if (brightness > 30) {
            const glowRadius = size * 0.8;
            const glowAlpha = Math.min(0.4, brightness / 400);
            const gradient = ctx.createRadialGradient(
              px + size / 2, py + size / 2, size * 0.2,
              px + size / 2, py + size / 2, glowRadius
            );
            gradient.addColorStop(0, `rgba(${led.r}, ${led.g}, ${led.b}, ${glowAlpha})`);
            gradient.addColorStop(1, 'rgba(0, 0, 0, 0)');
            ctx.fillStyle = gradient;
            ctx.fillRect(px - glowRadius / 2, py - glowRadius / 2, size + glowRadius, size + glowRadius);
          }
        }
      }
    }
  }

  _roundRect(ctx, x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.lineTo(x + w - r, y);
    ctx.quadraticCurveTo(x + w, y, x + w, y + r);
    ctx.lineTo(x + w, y + h - r);
    ctx.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
    ctx.lineTo(x + r, y + h);
    ctx.quadraticCurveTo(x, y + h, x, y + h - r);
    ctx.lineTo(x, y + r);
    ctx.quadraticCurveTo(x, y, x + r, y);
    ctx.closePath();
  }
}
