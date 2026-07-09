#ifndef FEEDBACK_H
#define FEEDBACK_H

#include <Arduino.h>
#include <FastLED.h>
#include "config.h"
#include "canvas.h"

// ---------------------------------------------------------------------------
// VideoFeedback: Milkdrop/AVS-style video feedback buffer.
//
// Each frame the previous 48x48 virtual canvas is resampled into the current
// one with a small geometric transform (zoom toward/away from a center point,
// rotation, translation drift) and attenuated (decay), then new content (boids,
// effects) is drawn on top. The result recirculates every frame, producing
// tunnel / spiral / echo-trail visuals.
//
// The feedback pass runs in VIRTUAL space (48x48, plain row-major indexing) so
// trails live in world coordinates and survive viewport movement. Serpentine
// mapping only ever happens at viewport extraction time, through Canvas::xy().
//
// Hot-loop math: the per-frame transform is set up once in float (the S3 has
// an FPU), then converted to 16.16 fixed point and applied incrementally per
// row/column - no trig or matrix math per pixel. The existing sin/cos LUT
// (256 steps, ~0.0245 rad resolution) is too coarse for the per-frame angles
// used here (+/-0.005..0.05 rad), so one sinf/cosf pair per frame is used
// instead. Bilinear sampling reuses the same Wu weight formula as
// Canvas::drawPixelF for consistency.
// ---------------------------------------------------------------------------

// Out-of-bounds handling default for the resample: 1 = wrap toroidally
// (matches boid wrapAroundBorders), 0 = clamp to black (harder-edged look).
// Also switchable at runtime via FeedbackParams::wrap.
#ifndef FEEDBACK_WRAP
#define FEEDBACK_WRAP 1
#endif

enum FeedbackPreset : uint8_t {
    FEEDBACK_OFF = 0,     // feedback disabled - original render path
    FEEDBACK_TUNNEL_IN,   // content spirals inward
    FEEDBACK_TUNNEL_OUT,  // content expands outward
    FEEDBACK_SPIRAL,      // strong rotation, drifting center
    FEEDBACK_ECHO_DRIFT,  // no zoom/rotation, directional streaking
    FEEDBACK_PRESET_COUNT
};

// Bilinear/Wu interpolation weight - identical to CANVAS_WU_WEIGHT in
// Canvas::drawPixelF, duplicated here so both stay self-contained.
#define FB_WU_WEIGHT(a, b) ((uint8_t)(((a) * (b) + (a) + (b)) >> 8))

struct FeedbackParams {
    float zoom = 1.0f;        // <1.0 spirals inward, >1.0 expands outward
    float rotation = 0.0f;    // radians per frame
    float centerX = 24.0f;    // feedback focal point (virtual coords)
    float centerY = 24.0f;
    float driftX = 0.0f;      // per-frame translation
    float driftY = 0.0f;
    uint8_t decay = 240;      // 0-255 fade multiplier applied to the resample
    uint8_t intensity = 255;  // 0-255 crossfade of the whole feedback signal
    bool wrap = (FEEDBACK_WRAP != 0);
    bool enabled = false;     // master toggle
};

class VideoFeedback {
public:
    static const uint8_t W = 48;
    static const uint8_t H = 48;

    FeedbackParams params;

    // --- Preset control -----------------------------------------------------
    void setPreset(FeedbackPreset p) {
        bool wasEnabled = params.enabled;
        preset_ = p;
        applyPresetBase();
        // Entering feedback from OFF: start from a clean slate so stale
        // pixels from a previous run don't recirculate.
        if (params.enabled && !wasEnabled) clear();
        #if DEBUG_SERIAL
        Serial.print("[FEEDBACK] Preset: ");
        Serial.println(presetName());
        #endif
    }

    void nextPreset() {
        setPreset((FeedbackPreset)((preset_ + 1) % FEEDBACK_PRESET_COUNT));
    }

    FeedbackPreset preset() const { return preset_; }

    const char* presetName() const {
        static const char* const names[FEEDBACK_PRESET_COUNT] = {
            "OFF", "TUNNEL_IN", "TUNNEL_OUT", "SPIRAL", "ECHO_DRIFT"};
        return names[preset_];
    }

    bool enabled() const { return params.enabled; }

    // Future audio-reactive hook: 0-255 energy scales zoom deviation and
    // rotation speed. Until an external driver calls this, a slow Perlin
    // walk drives the modulation instead.
    void setModulation(uint8_t energy) {
        modEnergy = energy;
        externalMod = true;
    }

    // --- Frame lifecycle ----------------------------------------------------
    // Step 1 of the frame: modulate parameters, then resample prev -> curr
    // with the inverse transform + decay. Writes every pixel of curr, so no
    // separate canvas clear is needed (decay is the clear).
    void beginFrame() {
        if (!params.enabled) return;
        updateModulation();
        uint32_t t0 = micros();
        resample();
        lastPassUs = micros() - t0;
    }

    // Last step of the frame: ping-pong the buffers so everything drawn this
    // frame (feedback + boids) becomes next frame's source.
    void endFrame() {
        if (!params.enabled) return;
        CRGB* t = prev;
        prev = curr;
        curr = t;
    }

    void clear() {
        fill_solid(bufA, W * H, CRGB::Black);
        fill_solid(bufB, W * H, CRGB::Black);
    }

    uint32_t lastPassMicros() const { return lastPassUs; }

    // --- Drawing into the virtual canvas ------------------------------------
    // Sub-pixel additive draw, mirroring Canvas::drawPixelF (Wu weights +
    // qadd8) so boids look identical whether they land on the physical canvas
    // or the feedback buffer. Edge spill wraps when params.wrap is set.
    void drawPixelF(float fx, float fy, CRGB color) {
        if (fx < 0 || fx >= W || fy < 0 || fy >= H) return;

        uint8_t xx = (fx - (int)fx) * 255, yy = (fy - (int)fy) * 255;
        uint8_t ix = 255 - xx, iy = 255 - yy;

        uint8_t wu[4] = {FB_WU_WEIGHT(ix, iy), FB_WU_WEIGHT(xx, iy),
                         FB_WU_WEIGHT(ix, yy), FB_WU_WEIGHT(xx, yy)};

        for (uint8_t i = 0; i < 4; i++) {
            int16_t xn = (int16_t)fx + (i & 1), yn = (int16_t)fy + ((i >> 1) & 1);
            if (params.wrap) {
                if (xn >= W) xn -= W;
                if (yn >= H) yn -= H;
            }
            if (xn >= 0 && xn < W && yn >= 0 && yn < H) {
                CRGB& c = curr[(uint16_t)yn * W + xn];
                c.r = qadd8(c.r, (color.r * wu[i]) >> 8);
                c.g = qadd8(c.g, (color.g * wu[i]) >> 8);
                c.b = qadd8(c.b, (color.b * wu[i]) >> 8);
            }
        }
    }

    // Copy the viewport window out of the virtual canvas into the physical
    // frame buffer. All physical output still goes through Canvas::xy(), so
    // the serpentine mapping is untouched.
    void extractViewport(Canvas& canvas, uint8_t viewX, uint8_t viewY) {
        CRGB* out = canvas.raw();
        for (uint8_t y = 0; y < canvas.height; y++) {
            const CRGB* src = curr + (uint16_t)(viewY + y) * W + viewX;
            for (uint8_t x = 0; x < canvas.width; x++) {
                out[canvas.xy(x, y)] = src[x];
            }
        }
    }

private:
    // Static double buffer (~6.9 KB each) - no heap use in the frame loop.
    CRGB bufA[W * H];
    CRGB bufB[W * H];
    CRGB* prev = bufA;
    CRGB* curr = bufB;

    FeedbackPreset preset_ = FEEDBACK_OFF;
    uint8_t modEnergy = 128;
    bool externalMod = false;
    uint32_t lastPassUs = 0;

    // Static (non-modulated) parameter baseline for each preset.
    void applyPresetBase() {
        params = FeedbackParams();
        params.centerX = W * 0.5f;
        params.centerY = H * 0.5f;

        switch (preset_) {
            case FEEDBACK_OFF:
                params.enabled = false;
                break;
            case FEEDBACK_TUNNEL_IN:
                params.enabled = true;
                params.zoom = 0.975f;
                params.rotation = 0.012f;
                params.decay = 242;
                break;
            case FEEDBACK_TUNNEL_OUT:
                params.enabled = true;
                params.zoom = 1.02f;
                params.rotation = -0.012f;
                params.decay = 238;
                break;
            case FEEDBACK_SPIRAL:
                params.enabled = true;
                params.zoom = 0.99f;
                params.rotation = 0.035f;
                params.decay = 245;
                break;
            case FEEDBACK_ECHO_DRIFT:
                params.enabled = true;
                params.zoom = 1.0f;
                params.rotation = 0.0f;
                params.decay = 250;
                break;
            default:
                break;
        }
    }

    // Slowly evolve zoom / rotation / center / drift with Perlin noise so the
    // feedback character changes organically. modEnergy (time-driven by
    // default, externally drivable via setModulation) scales how far the
    // parameters deviate from their preset baseline.
    void updateModulation() {
        uint32_t t = millis();
        uint8_t energy = externalMod ? modEnergy : inoise8(t >> 4);
        float e = energy / 255.0f;

        // Three independent signed noise channels in -1..1.
        float n1 = (inoise8(t >> 3, 0) - 128) / 128.0f;
        float n2 = (inoise8(0, t >> 3) - 128) / 128.0f;
        float n3 = (inoise8(t >> 4, 1000) - 128) / 128.0f;

        float cx = W * 0.5f, cy = H * 0.5f;

        switch (preset_) {
            case FEEDBACK_TUNNEL_IN:
                params.zoom = 0.975f + n3 * 0.008f * e;
                params.rotation = 0.012f * (0.5f + e) + n1 * 0.006f * e;
                params.centerX = cx;
                params.centerY = cy;
                break;
            case FEEDBACK_TUNNEL_OUT:
                params.zoom = 1.02f + n3 * 0.008f * e;
                params.rotation = -0.012f * (0.5f + e) - n1 * 0.006f * e;
                params.centerX = cx;
                params.centerY = cy;
                break;
            case FEEDBACK_SPIRAL:
                params.zoom = 0.99f + n3 * 0.005f * e;
                params.rotation = 0.035f * (0.5f + e);
                // Center wanders on a slow noise path around the middle.
                params.centerX = cx + n1 * 8.0f;
                params.centerY = cy + n2 * 8.0f;
                break;
            case FEEDBACK_ECHO_DRIFT: {
                // Directional wind: drift heading turns slowly with noise,
                // magnitude scales with energy.
                float ang = (inoise16((uint32_t)(t << 2)) / 65535.0f) * TWO_PI;
                float mag = 0.25f + 0.3f * e;
                params.driftX = cosf(ang) * mag;
                params.driftY = sinf(ang) * mag;
                break;
            }
            default:
                break;
        }
    }

    // The feedback resample: for every destination pixel, apply the inverse
    // transform (translate center to origin, rotate by -angle, scale by
    // 1/zoom, translate back, add drift), bilinearly sample prev, attenuate
    // by decay, and write to curr. Source coordinates are advanced
    // incrementally in 16.16 fixed point - no per-pixel trig.
    void resample() {
        const float invZoom = 1.0f / params.zoom;
        const float ca = cosf(params.rotation) * invZoom;
        const float sa = sinf(params.rotation) * invZoom;
        const float cx = params.centerX, cy = params.centerY;

        // Source coordinate of destination (0,0):
        //   sx = cx + (x-cx)*ca + (y-cy)*sa + driftX
        //   sy = cy - (x-cx)*sa + (y-cy)*ca + driftY
        const float sx00 = cx + (-cx) * ca + (-cy) * sa + params.driftX;
        const float sy00 = cy - (-cx) * sa + (-cy) * ca + params.driftY;

        const int32_t dxCol = (int32_t)lrintf(ca * 65536.0f);
        const int32_t dyCol = (int32_t)lrintf(-sa * 65536.0f);
        const int32_t dxRow = (int32_t)lrintf(sa * 65536.0f);
        const int32_t dyRow = (int32_t)lrintf(ca * 65536.0f);

        int32_t rowSx = (int32_t)lrintf(sx00 * 65536.0f);
        int32_t rowSy = (int32_t)lrintf(sy00 * 65536.0f);

        const int32_t WFP = (int32_t)W << 16;
        const int32_t HFP = (int32_t)H << 16;

        // Fold the intensity crossfade into the decay so the inner loop only
        // scales once per pixel.
        const uint8_t eff = scale8(params.decay, params.intensity);
        const bool wrap = params.wrap;

        const CRGB* src = prev;
        CRGB* dst = curr;

        for (uint8_t y = 0; y < H; y++) {
            int32_t sx = rowSx, sy = rowSy;
            for (uint8_t x = 0; x < W; x++, dst++) {
                int32_t wx = sx, wy = sy;
                if (wrap) {
                    wx %= WFP; if (wx < 0) wx += WFP;
                    wy %= HFP; if (wy < 0) wy += HFP;
                }

                int16_t x0 = (int16_t)(wx >> 16);
                int16_t y0 = (int16_t)(wy >> 16);
                uint8_t xf = (wx >> 8) & 0xFF;
                uint8_t yf = (wy >> 8) & 0xFF;
                uint8_t ixf = 255 - xf, iyf = 255 - yf;

                // Same bilinear weights as the Wu splat in Canvas::drawPixelF.
                uint8_t w00 = FB_WU_WEIGHT(ixf, iyf);
                uint8_t w10 = FB_WU_WEIGHT(xf, iyf);
                uint8_t w01 = FB_WU_WEIGHT(ixf, yf);
                uint8_t w11 = FB_WU_WEIGHT(xf, yf);

                uint16_t r = 0, g = 0, b = 0;

                if (wrap) {
                    int16_t x1 = x0 + 1; if (x1 >= W) x1 = 0;
                    int16_t y1 = y0 + 1; if (y1 >= H) y1 = 0;
                    const CRGB& p00 = src[(uint16_t)y0 * W + x0];
                    const CRGB& p10 = src[(uint16_t)y0 * W + x1];
                    const CRGB& p01 = src[(uint16_t)y1 * W + x0];
                    const CRGB& p11 = src[(uint16_t)y1 * W + x1];
                    r = p00.r * w00 + p10.r * w10 + p01.r * w01 + p11.r * w11;
                    g = p00.g * w00 + p10.g * w10 + p01.g * w01 + p11.g * w11;
                    b = p00.b * w00 + p10.b * w10 + p01.b * w01 + p11.b * w11;
                } else {
                    // Clamp-to-black: out-of-bounds taps contribute nothing.
                    int16_t x1 = x0 + 1, y1 = y0 + 1;
                    bool vx0 = (x0 >= 0 && x0 < W), vx1 = (x1 >= 0 && x1 < W);
                    bool vy0 = (y0 >= 0 && y0 < H), vy1 = (y1 >= 0 && y1 < H);
                    if (vy0) {
                        if (vx0) { const CRGB& p = src[(uint16_t)y0 * W + x0]; r += p.r * w00; g += p.g * w00; b += p.b * w00; }
                        if (vx1) { const CRGB& p = src[(uint16_t)y0 * W + x1]; r += p.r * w10; g += p.g * w10; b += p.b * w10; }
                    }
                    if (vy1) {
                        if (vx0) { const CRGB& p = src[(uint16_t)y1 * W + x0]; r += p.r * w01; g += p.g * w01; b += p.b * w01; }
                        if (vx1) { const CRGB& p = src[(uint16_t)y1 * W + x1]; r += p.r * w11; g += p.g * w11; b += p.b * w11; }
                    }
                }

                dst->r = scale8((uint8_t)(r >> 8), eff);
                dst->g = scale8((uint8_t)(g >> 8), eff);
                dst->b = scale8((uint8_t)(b >> 8), eff);

                sx += dxCol;
                sy += dyCol;
            }
            rowSx += dxRow;
            rowSy += dyRow;
        }
    }
};

#endif // FEEDBACK_H
