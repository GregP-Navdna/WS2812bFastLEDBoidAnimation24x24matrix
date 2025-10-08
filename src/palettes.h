#ifndef PALETTES_H
#define PALETTES_H

#include <FastLED.h>

// Palette definitions
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

// Pointer to the current palette
extern const TProgmemRGBPalette16* currentPalette_p;

// Function to switch palettes
void SetNewPalette(int _palcount);

#endif // PALETTES_H
