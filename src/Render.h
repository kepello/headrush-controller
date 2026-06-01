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

// Connection-status dot (color set by caller) + WiFi signal bars, drawn in the
// open bottom of the arc (the 120° gap below the value) where there's more room
// than crowding the top. Shared by the dial and the tuner.
inline void drawStatusIndicators(LGFX_Sprite& gfx, uint16_t statusDot, int sigLevel) {
    constexpr int yBase = CY + 86;
    gfx.fillCircle(CX - 24, yBase - 6, 6, statusDot);
    for (int b = 0; b < 4; ++b) {
        int bh = 4 + b * 3;                       // bar heights 4,7,10,13
        gfx.fillRect(CX - 6 + b * 7, yBase - bh, 4, bh,
                     b < sigLevel ? COLOR_VALUE : COLOR_DIM);
    }
}

// Draw the full arc gauge for a continuous binding. Background arc + colored
// foreground proportional to value. Center: big value text + small unit. Top:
// short label. Call from the UI task only.
// groupLen/groupFocus drive the knob-group dots: one dot per member with the
// active member highlighted (omitted for a group of one).
inline void drawContinuous(LGFX_Sprite& gfx, const ContinuousBinding& cb, float value, uint16_t statusDot, int sigLevel, int groupLen = 1, int groupFocus = 0) {
    gfx.fillScreen(COLOR_BG);

    drawStatusIndicators(gfx, statusDot, sigLevel);

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

    // Knob-group dots: one per member, active highlighted.
    if (groupLen > 1) {
        int n = groupLen > 8 ? 8 : groupLen;
        constexpr int gap = 12;
        int x0 = CX - (n - 1) * gap / 2;
        for (int i = 0; i < n; ++i)
            gfx.fillCircle(x0 + i * gap, CY + 66, 3, (i == groupFocus) ? COLOR_VALUE : COLOR_LABEL);
    }
    gfx.pushSprite(0, 0);
}

// Tuner view: the arc spans -50..+50 cents with a fixed center target tick. A
// colored marker rides the arc at the live pitch offset; the big center letter
// is the detected note. Green when in tune, amber when off. Same top indicators
// (status dot + signal bars) as the dial. Call from the UI task only.
inline void drawTuner(LGFX_Sprite& gfx, const char* note, float cents, uint16_t statusDot, int sigLevel) {
    gfx.fillScreen(COLOR_BG);

    drawStatusIndicators(gfx, statusDot, sigLevel);

    constexpr uint16_t TUNER_GREEN = 0x07E6;   // in tune
    constexpr uint16_t TUNER_AMBER = 0xFD20;   // off pitch
    bool haveNote = note && note[0] && strcmp(note, "--") != 0;
    bool inTune = haveNote && fabsf(cents) <= 3.0f;
    uint16_t accent = !haveNote ? COLOR_LABEL : (inTune ? TUNER_GREEN : TUNER_AMBER);

    // Background arc + fixed center target tick (12 o'clock = perfectly in tune).
    gfx.fillArc(CX, CY, ARC_INNER_R, ARC_OUTER_R, ARC_START_DEG, ARC_END_DEG, COLOR_DIM);
    float midDeg = ARC_START_DEG + ARC_SPAN_DEG * 0.5f;
    gfx.fillArc(CX, CY, ARC_INNER_R, ARC_OUTER_R, midDeg - 1.2f, midDeg + 1.2f, COLOR_VALUE);

    // Live marker at the cents position (only when we have a reading).
    if (haveNote) {
        float f = (cents + 50.0f) / 100.0f;
        if (f < 0) f = 0;
        if (f > 1) f = 1;
        float a = ARC_START_DEG + ARC_SPAN_DEG * f;
        gfx.fillArc(CX, CY, ARC_INNER_R, ARC_OUTER_R, a - 4.0f, a + 4.0f, accent);
    }

    gfx.setTextDatum(middle_center);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString("TUNER", CX, CY - 50);

    gfx.setTextColor(accent, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold24pt7b);
    gfx.drawString(haveNote ? note : "--", CX, CY + 4);

    char buf[16];
    if (!haveNote)      snprintf(buf, sizeof(buf), "listening");
    else if (inTune)    snprintf(buf, sizeof(buf), "in tune");
    else                snprintf(buf, sizeof(buf), "%s%dc", cents < 0 ? "-" : "+", (int)(fabsf(cents) + 0.5f));
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString(buf, CX, CY + 44);
    gfx.pushSprite(0, 0);
}

// Generic drill-down list (Setlist / Rig browser): a spinner with a back row at
// index 0 (labeled by `backLabel`, e.g. "< Exit" or "< Setlists") and
// items[0..count-1] at indices 1..count. Long names drop to a smaller font.
// Title sits at the top, position at the bottom. UI task only.
inline void drawListNav(LGFX_Sprite& gfx, const char* title, const char* backLabel, const char items[][40], int count, int sel) {
    gfx.fillScreen(COLOR_BG);
    gfx.setTextDatum(middle_center);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString(title, CX, 30);

    int total = count + 1;
    auto label = [&](int i) -> const char* { return (i == 0) ? backLabel : items[i - 1]; };
    int prev = (sel - 1 + total) % total, next = (sel + 1) % total;

    gfx.setFont(&fonts::FreeSansBold9pt7b);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.drawString(label(prev), CX, CY - 38);
    gfx.drawString(label(next), CX, CY + 38);

    const char* cur = label(sel);
    gfx.setTextColor(COLOR_VALUE, COLOR_BG);
    gfx.setFont(strlen(cur) > 9 ? &fonts::FreeSansBold12pt7b : &fonts::FreeSansBold18pt7b);
    gfx.drawString(cur, CX, CY);

    gfx.setFont(&fonts::FreeSansBold9pt7b);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    char pos[16];
    if (sel == 0) snprintf(pos, sizeof(pos), "click = back");
    else          snprintf(pos, sizeof(pos), "%d / %d", sel, count);
    gfx.drawString(pos, CX, 206);
    gfx.pushSprite(0, 0);
}

// Library idle screen: the default "dial-like" view that just shows the rig
// currently loaded on the Prime (big), the setlist (small), and the usual
// status indicators. The user enters the browser by turning the encoder.
inline void drawLibraryIdle(LGFX_Sprite& gfx, const char* rigName, const char* setlistName, uint16_t statusDot, int sigLevel, int groupLen = 1, int groupFocus = 0) {
    gfx.fillScreen(COLOR_BG);
    drawStatusIndicators(gfx, statusDot, sigLevel);
    gfx.setTextDatum(middle_center);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString("LIBRARY", CX, 36);
    const char* rn = (rigName && rigName[0]) ? rigName : "(no rig)";
    gfx.setTextColor(COLOR_VALUE, COLOR_BG);
    gfx.setFont(strlen(rn) > 10 ? &fonts::FreeSansBold12pt7b : &fonts::FreeSansBold18pt7b);
    gfx.drawString(rn, CX, CY - 6);
    if (setlistName && setlistName[0]) {
        gfx.setTextColor(COLOR_LABEL, COLOR_BG);
        gfx.setFont(&fonts::FreeSansBold9pt7b);
        gfx.drawString(setlistName, CX, CY + 32);
    }
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold9pt7b);
    gfx.drawString("double-click to browse", CX, CY + 56);
    if (groupLen > 1) {
        int n = groupLen > 8 ? 8 : groupLen;
        constexpr int gap = 12;
        int x0 = CX - (n - 1) * gap / 2;
        for (int i = 0; i < n; ++i)
            gfx.fillCircle(x0 + i * gap, CY + 74, 3, (i == groupFocus) ? COLOR_VALUE : COLOR_LABEL);
    }
    gfx.pushSprite(0, 0);
}

// Simple centered status (e.g. "loading...") for the list browser while the net
// task fetches/loads over HTTP. UI task only.
inline void drawListLoading(LGFX_Sprite& gfx, const char* msg) {
    gfx.fillScreen(COLOR_BG);
    gfx.setTextDatum(middle_center);
    gfx.setTextColor(COLOR_VALUE, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString(msg, CX, CY);
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

// Simple vertical menu: a title and a list of items, `sel` highlighted with a
// leading ">". Shared by the Board Menu and Settings menu.
inline void drawSimpleMenu(LGFX_Sprite& gfx, const char* title, const char* const* labels, int count, int sel, const char* footer = nullptr) {
    gfx.fillScreen(COLOR_BG);
    gfx.setTextDatum(middle_center);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString(title, CX, 28);
    int y0 = 120 - ((count - 1) * 32) / 2;   // vertically centered block
    for (int i = 0; i < count; ++i) {
        int y = y0 + i * 32;
        bool s = (i == sel);
        gfx.setTextColor(s ? COLOR_VALUE : COLOR_LABEL, COLOR_BG);
        gfx.setFont(&fonts::FreeSansBold12pt7b);
        gfx.drawString(labels[i], CX, y);
        if (s) gfx.drawString(">", CX - 104, y);
    }
    if (footer) {
        gfx.setTextColor(COLOR_LABEL, COLOR_BG);
        gfx.setFont(&fonts::FreeSansBold9pt7b);
        gfx.drawString(footer, CX, 212);
    }
    gfx.pushSprite(0, 0);
}

// Board Menu (hold from Home): the top-level destinations.
inline void drawBoardMenu(LGFX_Sprite& gfx, int sel) {
    static const char* const labels[2] = { "Assign knob", "Settings" };
    drawSimpleMenu(gfx, "MENU", labels, 2, sel, "hold = back");
}

// Settings menu (device/global). Four items; `sel` highlighted. Shows fw at the bottom.
inline void drawConfigMenu(LGFX_Sprite& gfx, int sel, int deviceId, int fwVersion) {
    char idbuf[20], fwbuf[16];
    snprintf(idbuf, sizeof(idbuf), "Device ID: %d", deviceId);
    snprintf(fwbuf, sizeof(fwbuf), "fw %d", fwVersion);
    const char* labels[4] = { idbuf, "WiFi", "Update firmware", "Exit" };
    drawSimpleMenu(gfx, "SETTINGS", labels, 4, sel, fwbuf);
}

// Assign-this-knob multi-select: a spinner over the catalog (dials then Tuner).
// Selected members show their 1-based position in the group; `cursor` is the
// highlighted item. sel[] holds the chosen view indices (param idx, or
// paramCount for Tuner) in order. Hold confirms; click toggles.
inline void drawAssignList(LGFX_Sprite& gfx, const ContinuousBinding* catalog, int paramCount, int cursor, const int* sel, int selLen) {
    gfx.fillScreen(COLOR_BG);
    gfx.setTextDatum(middle_center);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString("ASSIGN KNOB", CX, 32);

    int items = paramCount + 2;   // + Tuner + Rigs/Setlists
    auto label = [&](int i) -> const char* {
        if (i < paramCount) return catalog[i].label;
        return (i == paramCount) ? "Tuner" : "Rigs/Setlists";
    };
    auto orderOf = [&](int i) -> int { for (int k = 0; k < selLen; ++k) if (sel[k] == i) return k + 1; return 0; };
    auto fill = [&](int i, char* buf) {
        int o = orderOf(i);
        if (o) snprintf(buf, 28, "[%d] %s", o, label(i));
        else   snprintf(buf, 28, "[ ] %s", label(i));
    };
    int prev = (cursor - 1 + items) % items, next = (cursor + 1) % items;
    char pb[28], cb[28], nb[28];
    fill(prev, pb); fill(cursor, cb); fill(next, nb);
    gfx.setFont(&fonts::FreeSansBold9pt7b);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.drawString(pb, CX, CY - 40);
    gfx.drawString(nb, CX, CY + 40);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.setTextColor(COLOR_VALUE, COLOR_BG);
    gfx.drawString(cb, CX, CY);
    gfx.setFont(&fonts::FreeSansBold9pt7b);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.drawString("click=toggle  hold=done", CX, 206);
    gfx.pushSprite(0, 0);
}

// Views multi-select: spinner over the selectable views with [x]/[ ] checkboxes,
// plus a trailing "done" slot at index == viewCount. Indices below paramCount are
// dial params (catalog labels); the rest are named special views (Tuner). `sel`
// is 0..viewCount.
inline void drawViewSelect(LGFX_Sprite& gfx, const ContinuousBinding* catalog, int paramCount, int viewCount, int sel, uint32_t mask) {
    gfx.fillScreen(COLOR_BG);
    gfx.setTextDatum(middle_center);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString("VIEWS", CX, 32);
    int total = viewCount + 1;
    char pb[28], cb[28], nb[28];
    auto viewLabel = [&](int i) -> const char* {
        if (i < paramCount) return catalog[i].label;
        return (i == paramCount) ? "Tuner" : "Library";
    };
    auto fill = [&](int i, char* buf) {
        if (i >= viewCount) snprintf(buf, 28, "done");
        else snprintf(buf, 28, "[%c] %s", (mask & (1u << i)) ? 'x' : ' ', viewLabel(i));
    };
    int prev = (sel - 1 + total) % total, next = (sel + 1) % total;
    fill(prev, pb); fill(sel, cb); fill(next, nb);
    gfx.setFont(&fonts::FreeSansBold9pt7b);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.drawString(pb, CX, CY - 40);
    gfx.drawString(nb, CX, CY + 40);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.setTextColor(COLOR_VALUE, COLOR_BG);
    gfx.drawString(cb, CX, CY);
    gfx.setFont(&fonts::FreeSansBold9pt7b);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.drawString(sel < viewCount ? "click = toggle" : "click = done", CX, 206);
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
    int total = count + 1;   // index 0 = "< Back", 1..count = networks
    auto label = [&](int i) -> const char* { return (i == 0) ? "< Back" : ssids[i - 1]; };
    int prev = (sel - 1 + total) % total;
    int next = (sel + 1) % total;
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold9pt7b);
    gfx.drawString(label(prev), CX, CY - 38);
    gfx.drawString(label(next), CX, CY + 38);
    gfx.setTextColor(COLOR_VALUE, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold12pt7b);
    gfx.drawString(label(sel), CX, CY);
    gfx.setTextColor(COLOR_LABEL, COLOR_BG);
    gfx.setFont(&fonts::FreeSansBold9pt7b);
    char foot[28];
    if (sel == 0) snprintf(foot, sizeof(foot), "click = back");
    else          snprintf(foot, sizeof(foot), "%d/%d   click = select", sel, count);
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
