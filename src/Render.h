#pragma once
#include "Display.h"
#include "Binding.h"

// All draw functions render into an off-screen LGFX_Sprite (the `gfx` param) and
// blit the finished frame in one pushSprite — so the screen never shows an
// intermediate cleared state (flicker-free). Call only from the UI task.
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
inline void drawContinuous(LGFX_Sprite& gfx, const ContinuousBinding& cb, float value, uint16_t statusDot, int sigLevel) {
    gfx.fillScreen(COLOR_BG);

    // Top indicators: connection-status dot (color set by caller) + WiFi signal bars.
    gfx.fillCircle(CX - 24, CY - 80, 6, statusDot);
    for (int b = 0; b < 4; ++b) {
        int bh = 4 + b * 3;                       // bar heights 4,7,10,13
        gfx.fillRect(CX - 6 + b * 7, (CY - 73) - bh, 4, bh,
                     b < sigLevel ? COLOR_VALUE : COLOR_DIM);
    }

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
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString(cb.label, CX, CY - 50);

    // Big value text.
    gfx.setTextColor(COLOR_VALUE, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold24pt7b);
    char buf[24];
    snprintf(buf, sizeof(buf), cb.format, value);
    gfx.drawString(buf, CX, CY + 4);

    // Unit text.
    if (cb.unit && cb.unit[0]) {
        gfx.setTextColor(COLOR_LABEL, COLOR_BG);
        gfx.setFont(&fonts::FreeSansBold12pt7b);
        gfx.drawString(cb.unit, CX, CY + 44);
    }
    gfx.pushSprite(0, 0);
}

// Full-screen OTA progress: a ring that fills with the update percentage and a
// big "NN%" readout. Call from the UI task only (same as drawContinuous).
inline void drawOTAProgress(LGFX_Sprite& gfx, int percent) {
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
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString("UPDATING", CX, CY - 50);

    gfx.setTextColor(COLOR_VALUE, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold24pt7b);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", percent);
    gfx.drawString(buf, CX, CY + 4);
    gfx.pushSprite(0, 0);
}

// Boot/status splash: an accent-ringed "HeadRush" title with fw/ID and a status
// line. Used for the whole boot sequence (connecting / setting clock / checking
// updates) and the update-check phase, so the unit shows progress before the
// dial appears.
inline void drawSplash(LGFX_Sprite& gfx, int deviceId, int fwVersion, const char* status) {
    gfx.fillScreen(COLOR_BG);
    gfx.fillArc(CX, CY, 117, 120, 0, 360, COLOR_FALLBACK);  // accent ring on the round bezel
    gfx.setTextDatum(middle_center);
    gfx.setTextColor(COLOR_VALUE, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold18pt7b);
    gfx.drawString("HeadRush", CX, CY - 44);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    char info[24];
    snprintf(info, sizeof(info), "fw %d   ID %d", fwVersion, deviceId);
    gfx.drawString(info, CX, CY + 2);
    gfx.setTextColor(COLOR_FALLBACK, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString(status, CX, CY + 48);
    gfx.pushSprite(0, 0);
}

// Config-mode menu. Five items; `sel` is the highlighted index.
inline void drawConfigMenu(LGFX_Sprite& gfx, int sel, int deviceId, const char* paramLabel, int fwVersion) {
    gfx.fillScreen(COLOR_BG);
    gfx.setTextDatum(middle_center);

    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString("CONFIG", CX, 24);

    char idbuf[20], pbuf[24];
    snprintf(idbuf, sizeof(idbuf), "Device ID: %d", deviceId);
    snprintf(pbuf, sizeof(pbuf), "Param: %s", paramLabel);
    const char* labels[5] = { idbuf, pbuf, "WiFi", "Update firmware", "Exit" };
    for (int i = 0; i < 5; ++i) {
        int y = 58 + i * 30;
        bool s = (i == sel);
        gfx.setTextColor(s ? COLOR_VALUE : COLOR_LABEL, COLOR_BG);
        gfx.setFont(&fonts::FreeSansBold12pt7b);
        gfx.drawString(labels[i], CX, y);
        if (s) gfx.drawString(">", CX - 104, y);
    }

    char fwbuf[16];
    snprintf(fwbuf, sizeof(fwbuf), "fw %d", fwVersion);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold9pt7b);
    gfx.drawString(fwbuf, CX, 212);
    gfx.pushSprite(0, 0);
}

// Parameter picker: spinner showing the highlighted entry with dimmed neighbors.
inline void drawParamPick(LGFX_Sprite& gfx, const ContinuousBinding* catalog, int count, int sel) {
    gfx.fillScreen(COLOR_BG);
    gfx.setTextDatum(middle_center);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString("PARAMETER", CX, 38);
    int prev = (sel - 1 + count) % count;
    int next = (sel + 1) % count;
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString(catalog[prev].label, CX, CY - 42);
    gfx.drawString(catalog[next].label, CX, CY + 42);
    gfx.setTextColor(COLOR_VALUE, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold18pt7b);
    gfx.drawString(catalog[sel].label, CX, CY);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold9pt7b);
    gfx.drawString("turn = change   click = save", CX, 208);
    gfx.pushSprite(0, 0);
}

// WiFi network picker: spinner over scanned SSIDs (or "no networks").
inline void drawWifiPicker(LGFX_Sprite& gfx, const char ssids[][33], int count, int sel) {
    gfx.fillScreen(COLOR_BG);
    gfx.setTextDatum(middle_center);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString("SELECT WIFI", CX, 36);
    if (count <= 0) {
        gfx.setTextColor(COLOR_VALUE, COLOR_BG);
        gfx.drawString("no networks", CX, CY);
        gfx.setTextColor(COLOR_LABEL, COLOR_BG);
        gfx.setFont(&fonts::FreeSansBold9pt7b);
        gfx.drawString("click = rescan", CX, 208);
        gfx.pushSprite(0, 0);
        return;
    }
    int prev = (sel - 1 + count) % count;
    int next = (sel + 1) % count;
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold9pt7b);
    gfx.drawString(ssids[prev], CX, CY - 38);
    gfx.drawString(ssids[next], CX, CY + 38);
    gfx.setTextColor(COLOR_VALUE, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString(ssids[sel], CX, CY);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold9pt7b);
    char foot[28];
    snprintf(foot, sizeof(foot), "%d/%d   click = select", sel + 1, count);
    gfx.drawString(foot, CX, 208);
    gfx.pushSprite(0, 0);
}

// WiFi password entry (encoder character picker). `curItem` is the currently
// highlighted character (or "DEL"); `pw` is what's been typed so far.
inline void drawPasswordEntry(LGFX_Sprite& gfx, const char* ssid, const char* pw, const char* curItem) {
    gfx.fillScreen(COLOR_BG);
    gfx.setTextDatum(middle_center);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold9pt7b);
    gfx.drawString(ssid, CX, 26);
    // entered password so far (last 14 chars if long)
    int len = strlen(pw);
    const char* shown = (len > 14) ? pw + (len - 14) : pw;
    gfx.setTextColor(COLOR_VALUE, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString(len ? shown : "( empty )", CX, CY - 42);
    // highlighted character / DEL
    gfx.setTextColor(COLOR_FALLBACK, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold24pt7b);
    gfx.drawString(curItem, CX, CY + 10);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold9pt7b);
    gfx.drawString("hold = back", CX, 206);
    gfx.pushSprite(0, 0);
}

// Device-ID editor: big number, turn to change, click to save.
inline void drawConfigIdEdit(LGFX_Sprite& gfx, int value) {
    gfx.fillScreen(COLOR_BG);
    gfx.setTextDatum(middle_center);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString("DEVICE ID", CX, CY - 52);
    gfx.setTextColor(COLOR_VALUE, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold24pt7b);
    char b[8];
    snprintf(b, sizeof(b), "%d", value);
    gfx.drawString(b, CX, CY + 2);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold9pt7b);
    gfx.drawString("turn = change   click = save", CX, CY + 58);
    gfx.pushSprite(0, 0);
}

} // namespace Render
