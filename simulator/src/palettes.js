/**
 * Color palettes - Ported from palettes.h
 * Each palette is an array of 16 RGB color objects
 */

function hex(h) {
  return {
    r: (h >> 16) & 0xFF,
    g: (h >> 8) & 0xFF,
    b: h & 0xFF
  };
}

const BLACK = { r: 0, g: 0, b: 0 };

export const GreenAuroraColors = [
  hex(0x000000), hex(0x003300), hex(0x006600), hex(0x009900),
  hex(0x00cc00), hex(0x00ff00), hex(0x33ff00), hex(0x66ff00),
  hex(0x99ff00), hex(0xccff00), hex(0xffff00), hex(0xffcc00),
  hex(0xff9900), hex(0xff6600), hex(0xff3300), hex(0xff0000)
];

export const WoodFireColors = [
  BLACK, hex(0x330e00), hex(0x661c00), hex(0x992900),
  hex(0xcc3700), hex(0xFF4500), hex(0xff5800), hex(0xff6b00),
  hex(0xff7f00), hex(0xff9200), hex(0xFFA500), hex(0xffaf00),
  hex(0xffb900), hex(0xffc300), hex(0xffcd00), hex(0xFFD700)
];

export const NormalFire = [
  BLACK, hex(0x330000), hex(0x660000), hex(0x990000),
  hex(0xcc0000), hex(0xFF0000), hex(0xff0c00), hex(0xff1800),
  hex(0xff2400), hex(0xff3000), hex(0xff3c00), hex(0xff4800),
  hex(0xff5400), hex(0xff6000), hex(0xff6c00), hex(0xff7800)
];

export const NormalFire2 = [
  BLACK, hex(0x560000), hex(0x6b0000), hex(0x820000),
  hex(0x9a0011), hex(0xB22222), hex(0xc22520), hex(0xd12a1c),
  hex(0xe12f17), hex(0xf0350f), hex(0xff3c00), hex(0xff6400),
  hex(0xff8300), hex(0xffa000), hex(0xffba00), hex(0xffd400)
];

export const LithiumFireColors = [
  BLACK, hex(0x240707), hex(0x470e0e), hex(0x6b1414),
  hex(0x8e1b1b), hex(0xB22222), hex(0xc14244), hex(0xd16166),
  hex(0xe08187), hex(0xf0a0a9), hex(0xFFC0CB), hex(0xff9ec0),
  hex(0xff7bb5), hex(0xff59a9), hex(0xff369e), hex(0xFF1493)
];

export const SodiumFireColors = [
  BLACK, hex(0x332100), hex(0x664200), hex(0x996300),
  hex(0xcc8400), hex(0xFFA500), hex(0xffaf00), hex(0xffb900),
  hex(0xffc300), hex(0xffcd00), hex(0xFFD700), hex(0xf8cd06),
  hex(0xf0c30d), hex(0xe9b913), hex(0xe1af1a), hex(0xDAA520)
];

export const CopperFireColors = [
  BLACK, hex(0x001a00), hex(0x003300), hex(0x004d00),
  hex(0x006600), hex(0x008000), hex(0x239909), hex(0x45b313),
  hex(0x68cc1c), hex(0x8ae626), hex(0xADFF2F), hex(0x94f530),
  hex(0x7ceb30), hex(0x63e131), hex(0x4bd731), hex(0x32CD32)
];

export const AlcoholFireColors = [
  BLACK, hex(0x000033), hex(0x000066), hex(0x000099),
  hex(0x0000cc), hex(0x0000FF), hex(0x0026ff), hex(0x004cff),
  hex(0x0073ff), hex(0x0099ff), hex(0x00BFFF), hex(0x1bc2fe),
  hex(0x36c5fd), hex(0x51c8fc), hex(0x6ccbfb), hex(0x87CEFA)
];

export const RubidiumFireColors = [
  hex(0xFF0000), hex(0x0f001a), hex(0x1e0034), hex(0x2d004e),
  hex(0x3c0068), hex(0xFF0000), hex(0x4B0082), hex(0x4B0082),
  hex(0x4B0082), hex(0x4B0082), hex(0x4B0082), hex(0x3c0084),
  hex(0x2d0086), hex(0x1e0087), hex(0x0f0089), hex(0xFF0000)
];

export const PartyColors = [
  hex(0x5500AB), hex(0x84007C), hex(0xB5004B), hex(0xE5001B),
  hex(0xE81700), hex(0xB84700), hex(0xAB7700), hex(0xABAB00),
  hex(0xAB5500), hex(0xDD2200), hex(0xF2000E), hex(0xC2003E),
  hex(0x8F0071), hex(0x5F00A1), hex(0x2F00D0), hex(0x0007F9)
];

export const CloudColors = [
  hex(0x0000FF), hex(0x0000FF), hex(0x0000FF), hex(0x0000FF),
  hex(0x0000FF), hex(0x2222FF), hex(0x4444FF), hex(0x5555FF),
  hex(0x6666FF), hex(0x7777FF), hex(0x8888FF), hex(0xAAAAFF),
  hex(0xCCCCFF), hex(0xDDDDFF), hex(0xEEEEFF), hex(0xFFFFFF)
];

export const LavaColors = [
  hex(0x000000), hex(0x800000), hex(0x800000), hex(0x800000),
  hex(0xFF0000), hex(0xFF0000), hex(0xFF0000), hex(0xFF0000),
  hex(0xFF0000), hex(0xFF4500), hex(0xFF8C00), hex(0xFFA500),
  hex(0xFFD700), hex(0xFFFF00), hex(0xFFFF00), hex(0xFFFFFF)
];

export const OceanColors = [
  hex(0x000040), hex(0x000040), hex(0x000080), hex(0x000080),
  hex(0x000080), hex(0x0000FF), hex(0x0000FF), hex(0x008080),
  hex(0x008080), hex(0x00CED1), hex(0x00CED1), hex(0x00BFFF),
  hex(0x00BFFF), hex(0x87CEEB), hex(0x87CEEB), hex(0xFFFFFF)
];

export const ForestColors = [
  hex(0x006400), hex(0x006400), hex(0x006400), hex(0x006400),
  hex(0x006400), hex(0x228B22), hex(0x228B22), hex(0x228B22),
  hex(0x228B22), hex(0x228B22), hex(0x32CD32), hex(0x32CD32),
  hex(0x32CD32), hex(0x32CD32), hex(0x90EE90), hex(0x90EE90)
];

export const RainbowColors = [
  hex(0xFF0000), hex(0xD52A00), hex(0xAB5500), hex(0xAB7F00),
  hex(0xABAB00), hex(0x56D500), hex(0x00FF00), hex(0x00D52A),
  hex(0x00AB55), hex(0x0056AA), hex(0x0000FF), hex(0x2A00D5),
  hex(0x5500AB), hex(0x7F0081), hex(0xAB0055), hex(0xD5002B)
];

export const RainbowStripeColors = [
  hex(0xFF0000), hex(0x000000), hex(0xAB5500), hex(0x000000),
  hex(0xABAB00), hex(0x000000), hex(0x00FF00), hex(0x000000),
  hex(0x00AB55), hex(0x000000), hex(0x0000FF), hex(0x000000),
  hex(0x5500AB), hex(0x000000), hex(0xAB0055), hex(0x000000)
];

export const darkishColors = [
  hex(0xcbbcaa), hex(0x72ac92), hex(0x797997), hex(0x756372),
  hex(0x638d54), hex(0x606a40), hex(0x4e4843), hex(0x1a2f27),
  hex(0xb6a588), hex(0xa7889f), hex(0xa17b6d), hex(0x877041),
  hex(0x725356), hex(0x4e372c), hex(0x3e2230), hex(0x000000)
];

export const sixteen1Colors = [
  hex(0x53437f), hex(0xa89fcc), hex(0xffffff), hex(0xffd9e8),
  hex(0xff9bb6), hex(0x9968e2), hex(0xbe9bff), hex(0x7fceff),
  hex(0x6d81ff), hex(0x2c6f99), hex(0x00bcaa), hex(0xc48f9e),
  hex(0x8e586f), hex(0xff5470), hex(0xff9b71), hex(0xffd9ae)
];

export const sixteen2Colors = [
  hex(0x4d004c), hex(0x8f0076), hex(0xc70083), hex(0xf50078),
  hex(0xff4764), hex(0xff9393), hex(0xffd5cc), hex(0xfff3f0),
  hex(0x000221), hex(0x000769), hex(0x00228f), hex(0x0050c7),
  hex(0x008bf5), hex(0x00bbff), hex(0x47edff), hex(0x93fff8)
];

export const sixteen3Colors = [
  hex(0x000000), hex(0x7e7e7e), hex(0xbebebe), hex(0xffffff),
  hex(0x7e0000), hex(0xfe0000), hex(0x047e00), hex(0x06ff04),
  hex(0x7e7e00), hex(0xffff04), hex(0x00007e), hex(0x0000ff),
  hex(0x7e007e), hex(0xfe00ff), hex(0x047e7e), hex(0x06ffff)
];

export const sixteen4Colors = [
  hex(0x2a1e1e), hex(0x664933), hex(0x28291d), hex(0x576632),
  hex(0x251d29), hex(0x643266), hex(0x291d22), hex(0x663237),
  hex(0x29221d), hex(0x665932), hex(0x1d2629), hex(0x324b66),
  hex(0x1d2920), hex(0x32664d), hex(0x1f1d29), hex(0x483266)
];

export const sixteen5Colors = [
  hex(0x313432), hex(0x323e42), hex(0x454b4b), hex(0x3a5f3b),
  hex(0x7c4545), hex(0x675239), hex(0x625055), hex(0x516b43),
  hex(0x796c64), hex(0x718245), hex(0x9e805c), hex(0x998579),
  hex(0xac9086), hex(0xa6a296), hex(0xb4ab8f), hex(0xbcb7a5)
];

export const sixteen6Colors = [
  hex(0x42243c), hex(0x4e253d), hex(0x59263d), hex(0x64273a),
  hex(0x6d2936), hex(0x752d30), hex(0x7c3228), hex(0x803a1f),
  hex(0x814214), hex(0x804c04), hex(0x7c5700), hex(0x756100),
  hex(0x6b6c00), hex(0x5e7600), hex(0x4b8000), hex(0x2c8a00)
];

export const sixteen7Colors = [
  hex(0xff0000), hex(0xfd3500), hex(0xfa4e00), hex(0xf66300),
  hex(0xef7600), hex(0xe68600), hex(0xd4a300), hex(0xc9b000),
  hex(0xbdbc00), hex(0xb0c800), hex(0xa1d300), hex(0x8fdf00),
  hex(0x76ea00), hex(0x54f500), hex(0x04ff00), hex(0x00ff00)
];

export const sixteen8Colors = [
  hex(0x0036ff), hex(0x5023e6), hex(0x6904ce), hex(0x7600b6),
  hex(0x7b009a), hex(0x7a0081), hex(0x77006c), hex(0x72005b),
  hex(0x6c004c), hex(0x65003e), hex(0x5e0033), hex(0x570028),
  hex(0x50001e), hex(0x450017), hex(0x3b030f), hex(0x300808)
];

// All palettes in order (matches SetNewPalette indices)
export const ALL_PALETTES = [
  GreenAuroraColors,   // 0
  WoodFireColors,      // 1
  NormalFire,          // 2
  NormalFire2,         // 3
  LithiumFireColors,   // 4
  SodiumFireColors,    // 5
  CopperFireColors,    // 6
  AlcoholFireColors,   // 7
  RubidiumFireColors,  // 8
  PartyColors,         // 9
  CloudColors,         // 10
  LavaColors,          // 11
  OceanColors,         // 12
  ForestColors,        // 13
  RainbowColors,       // 14
  RainbowStripeColors, // 15
  darkishColors,       // 16
  sixteen1Colors,      // 17
  sixteen2Colors,      // 18
  sixteen3Colors,      // 19
  sixteen4Colors,      // 20
  sixteen5Colors,      // 21
  sixteen6Colors,      // 22
  sixteen7Colors,      // 23
  sixteen8Colors       // 24
];

export const PALETTE_NAMES = [
  'Green Aurora', 'Wood Fire', 'Normal Fire', 'Normal Fire 2',
  'Lithium Fire', 'Sodium Fire', 'Copper Fire', 'Alcohol Fire',
  'Rubidium Fire', 'Party', 'Cloud', 'Lava',
  'Ocean', 'Forest', 'Rainbow', 'Rainbow Stripe',
  'Darkish', 'Sixteen 1', 'Sixteen 2', 'Sixteen 3',
  'Sixteen 4', 'Sixteen 5', 'Sixteen 6', 'Sixteen 7', 'Sixteen 8'
];

/**
 * ColorFromPalette - matches FastLED's behavior
 * Maps a 0-255 index to a color from a 16-entry palette with interpolation
 */
export function colorFromPalette(palette, index, brightness = 255, blend = true) {
  // Scale 0-255 to palette index
  const scaledIndex = (index / 256.0) * 16.0;
  const idx = Math.floor(scaledIndex) % 16;

  let color;
  if (blend) {
    const nextIdx = (idx + 1) % 16;
    const frac = scaledIndex - Math.floor(scaledIndex);
    const c1 = palette[idx];
    const c2 = palette[nextIdx];
    color = {
      r: Math.round(c1.r + (c2.r - c1.r) * frac),
      g: Math.round(c1.g + (c2.g - c1.g) * frac),
      b: Math.round(c1.b + (c2.b - c1.b) * frac)
    };
  } else {
    color = { ...palette[idx] };
  }

  // Apply brightness
  if (brightness < 255) {
    color.r = Math.round((color.r * brightness) / 255);
    color.g = Math.round((color.g * brightness) / 255);
    color.b = Math.round((color.b * brightness) / 255);
  }

  return color;
}
