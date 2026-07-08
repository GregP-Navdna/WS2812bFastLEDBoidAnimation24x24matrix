/**
 * Main entry point - Sets up the simulation loop and UI
 */
import { Simulation } from './simulation.js';
import { Renderer } from './renderer.js';
import { PALETTE_NAMES } from './palettes.js';

// --- State ---
let simulation;
let renderer;
let fps = 0;
let frameCount = 0;
let lastFpsUpdate = 0;

// --- Initialization ---
function init() {
  const canvas = document.getElementById('led-canvas');
  renderer = new Renderer(canvas, 24, 24);
  simulation = new Simulation();

  // Build palette selector
  const paletteSelect = document.getElementById('palette-select');
  PALETTE_NAMES.forEach((name, idx) => {
    const option = document.createElement('option');
    option.value = idx;
    option.textContent = name;
    paletteSelect.appendChild(option);
  });

  // Build pattern selector
  const patternSelect = document.getElementById('pattern-select');
  const patternNames = [
    'Default (Center)', 'Four Corners', 'Corners + Repulsor',
    'Diagonal TL-BR', 'Diagonal TR-BL', 'Central Repulsor',
    'Rotating Square', 'Figure-8', 'Pulsing Center',
    'Spiral Arms', 'Central Vortex'
  ];
  patternNames.forEach((name, idx) => {
    const option = document.createElement('option');
    option.value = idx;
    option.textContent = name;
    patternSelect.appendChild(option);
  });

  // --- Event Listeners ---
  document.getElementById('btn-pause').addEventListener('click', () => {
    const running = simulation.togglePause();
    document.getElementById('btn-pause').textContent = running ? '⏸ Pause' : '▶ Resume';
  });

  document.getElementById('btn-explode').addEventListener('click', () => {
    simulation.triggerExplosion();
  });

  // Effect buttons
  const effectButtons = {
    'btn-meteor':    () => { const fx = simulation.matrixEffects; fx.meteorActive ? fx.stopMeteorShower() : fx.startMeteorShower(); },
    'btn-plasma':    () => { const fx = simulation.matrixEffects; fx.plasmaActive ? fx.stopPlasma() : fx.startPlasma(); },
    'btn-lightning': () => { simulation.matrixEffects.startLightning(); },
    'btn-pulse':     () => { const fx = simulation.matrixEffects; fx.pulseActive ? fx.stopPulse() : fx.startPulse(); },
    'btn-spiral':    () => { const fx = simulation.matrixEffects; fx.spiralActive ? fx.stopSpiral() : fx.startSpiral(); },
    'btn-fire':      () => { const fx = simulation.matrixEffects; fx.fireActive ? fx.stopFire() : fx.startFire(); },
    'btn-sparkle':   () => { simulation.matrixEffects.startSparkleBurst(); },
    'btn-gravwell':  () => { simulation.matrixEffects.startGravityWell(); },
    'btn-random':    () => { simulation.matrixEffects.triggerRandomEffect(); },
  };

  for (const [id, handler] of Object.entries(effectButtons)) {
    const el = document.getElementById(id);
    if (el) el.addEventListener('click', handler);
  }

  paletteSelect.addEventListener('change', (e) => {
    simulation.setPalette(parseInt(e.target.value));
  });

  patternSelect.addEventListener('change', (e) => {
    simulation.setPattern(parseInt(e.target.value));
  });

  // Handle resize
  window.addEventListener('resize', () => {
    renderer.resize();
  });

  // Start loop
  lastFpsUpdate = performance.now();
  requestAnimationFrame(loop);
}

// --- Main Loop ---
function loop(timestamp) {
  // Update simulation
  simulation.draw();

  // Render LED buffer to canvas
  renderer.render(simulation.leds);

  // FPS counter
  frameCount++;
  if (timestamp - lastFpsUpdate >= 1000) {
    fps = frameCount;
    frameCount = 0;
    lastFpsUpdate = timestamp;
    updateStats();
  }

  requestAnimationFrame(loop);
}

function updateStats() {
  const stats = simulation.getStats();
  const el = document.getElementById('stats');
  el.innerHTML = `
    <span><b>FPS:</b> ${fps}</span>
    <span><b>Boids:</b> ${stats.boidCount}</span>
    <span><b>Pattern:</b> ${stats.pattern}</span>
    <span><b>Palette:</b> ${stats.palette}</span>
    <span><b>Speed:</b> ${stats.maxspeed}</span>
    <span><b>Vel Hue:</b> ${stats.velocityHue ? 'ON' : 'OFF'}</span>
  `;
}

// --- Boot ---
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', init);
} else {
  init();
}
