#pragma once
#include "Display.h"
#include "Binding.h"

namespace Render {

// Geometry. Arc sweeps from bottom-left around the top to bottom-right.
constexpr int CX = 120;
constexpr int CY = 120;
constexpr int ARC_OUTER_R = 116;
constexpr int ARC_INNER_R = 100;
constexpr float ARC_START_DEG = 150.0f;  // LovyanGFX angles: 0° is right, +CW
constexpr float ARC_END_DEG = 30.0f + 360.0f; // 240° sweep
constexpr float ARC_SPAN_DEG = ARC_END_DEG - ARC_START_DEG;

constexpr uint16_t COLOR_BG = TFT_BLACK;
constexpr uint16_t COLOR_DIM = 0x18C3;     // very dark grey-blue for unfilled arc
constexpr uint16_t COLOR_LABEL = 0x8C71;   // muted grey
constexpr uint16_t COLOR_VALUE = TFT_WHITE;
constexpr uint16_t COLOR_FALLBACK = 0x07FF;

// Find the color of the zone the given display-unit value falls into.
inline uint16_t zoneColor(const ContinuousBinding& cb, float v) {
    for (int i = 0; i < cb.zoneCount; ++i) {
        if (v <= cb.zones[i].to) return cb.zones[i].color;
    }
    return cb.zoneCount > 0 ? cb.zones[cb.zoneCount - 1].color : COLOR_FALLBACK;
}

// Map a display-unit value to a fraction 0..1 across [dispMin..dispMax].
inline float frac(const ContinuousBinding& cb, float v) {
    if (cb.dispMax == cb.dispMin) return 0.0f;
    float f = (v - cb.dispMin) / (cb.dispMax - cb.dispMin);
    if (f < 0) f = 0;
    if (f > 1) f = 1;
    return f;
}

// Draw the full arc gauge for a continuous binding. Background arc + colored
// foreground proportional to value. Center: big value text + small unit. Top:
// short label. Call from the UI task only.
inline void drawContinuous(CrowPanelLGFX& gfx, const ContinuousBinding& cb, float value) {
    gfx.fillScreen(COLOR_BG);

    // Background arc (full sweep, dim).
    gfx.fillArc(CX, CY, ARC_INNER_R, ARC_OUTER_R, ARC_START_DEG, ARC_END_DEG, COLOR_DIM);

    // Foreground arc segments — color comes from the zone at each segment's value.
    // 60 segments across the full sweep at 1° resolution looks smooth.
    float f = frac(cb, value);
    int filledSegments = (int)(60 * f + 0.5f);
    for (int i = 0; i < filledSegments; ++i) {
        float segFrac = (i + 0.5f) / 60.0f;
        float segValue = cb.dispMin + segFrac * (cb.dispMax - cb.dispMin);
        uint16_t color = zoneColor(cb, segValue);
        float a0 = ARC_START_DEG + (ARC_SPAN_DEG * i) / 60.0f;
        float a1 = ARC_START_DEG + (ARC_SPAN_DEG * (i + 1)) / 60.0f;
        gfx.fillArc(CX, CY, ARC_INNER_R, ARC_OUTER_R, a0, a1, color);
    }

    // Label at the top inside the arc.
    gfx.setTextDatum(middle_center);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setTextSize(2);
    gfx.drawString(cb.label, CX, CY - 50);

    // Big value text.
    gfx.setTextColor(COLOR_VALUE, COLOR_BG);
    gfx.setTextSize(5);
    char buf[24];
    snprintf(buf, sizeof(buf), cb.format, value);
    gfx.drawString(buf, CX, CY + 4);

    // Unit text.
    if (cb.unit && cb.unit[0]) {
        gfx.setTextColor(COLOR_LABEL, COLOR_BG);
        gfx.setTextSize(2);
        gfx.drawString(cb.unit, CX, CY + 44);
    }
}

// Full-screen OTA progress: a ring that fills with the update percentage and a
// big "NN%" readout. Call from the UI task only (same as drawContinuous).
inline void drawOTAProgress(CrowPanelLGFX& gfx, int percent) {
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    gfx.fillScreen(COLOR_BG);

    gfx.fillArc(CX, CY, ARC_INNER_R, ARC_OUTER_R, ARC_START_DEG, ARC_END_DEG, COLOR_DIM);
    float f = percent / 100.0f;
    if (f > 0.0f) {
        float a1 = ARC_START_DEG + ARC_SPAN_DEG * f;
        gfx.fillArc(CX, CY, ARC_INNER_R, ARC_OUTER_R, ARC_START_DEG, a1, COLOR_FALLBACK);
    }

    gfx.setTextDatum(middle_center);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setTextSize(2);
    gfx.drawString("UPDATING", CX, CY - 50);

    gfx.setTextColor(COLOR_VALUE, COLOR_BG);
    gfx.setTextSize(5);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", percent);
    gfx.drawString(buf, CX, CY + 4);
}

// Config-mode menu. Three fixed items; `sel` is the highlighted index.
inline void drawConfigMenu(CrowPanelLGFX& gfx, int sel, int deviceId, int fwVersion) {
    gfx.fillScreen(COLOR_BG);
    gfx.setTextDatum(middle_center);

    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setTextSize(2);
    gfx.drawString("CONFIG", CX, 36);

    char idbuf[20];
    snprintf(idbuf, sizeof(idbuf), "Device ID: %d", deviceId);
    const char* labels[3] = { idbuf, "Update firmware", "Exit" };
    for (int i = 0; i < 3; ++i) {
        int y = 92 + i * 36;
        bool s = (i == sel);
        gfx.setTextColor(s ? COLOR_VALUE : COLOR_LABEL, COLOR_BG);
        gfx.setTextSize(2);
        gfx.drawString(labels[i], CX, y);
        if (s) gfx.drawString(">", CX - 96, y);
    }

    char fwbuf[16];
    snprintf(fwbuf, sizeof(fwbuf), "fw %d", fwVersion);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setTextSize(1);
    gfx.drawString(fwbuf, CX, 208);
}

// Device-ID editor: big number, turn to change, click to save.
inline void drawConfigIdEdit(CrowPanelLGFX& gfx, int value) {
    gfx.fillScreen(COLOR_BG);
    gfx.setTextDatum(middle_center);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setTextSize(2);
    gfx.drawString("DEVICE ID", CX, CY - 52);
    gfx.setTextColor(COLOR_VALUE, COLOR_BG);
    gfx.setTextSize(7);
    char b[8];
    snprintf(b, sizeof(b), "%d", value);
    gfx.drawString(b, CX, CY + 2);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setTextSize(1);
    gfx.drawString("turn = change   click = save", CX, CY + 58);
}

inline void drawBootScreen(CrowPanelLGFX& gfx, const char* msg) {
    gfx.fillScreen(COLOR_BG);
    gfx.setTextDatum(middle_center);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setTextSize(2);
    gfx.drawString(msg, CX, CY);
}

} // namespace Render
