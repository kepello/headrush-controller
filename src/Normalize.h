#pragma once
#include <Arduino.h>

// Most numeric properties on the HeadRush Prime travel as normalized 0..1 floats.
// The schema's minimum/maximum are the *display* range, and x-options.format / grid
// describe how the value is presented to the user.
//
// e.g. Tempo:  min=30, max=240, format="%.2f BPM", grid=0.01 (display units)
//      wire value 0.428 → display 30 + 0.428 * (240-30) = 119.88 BPM
//
// Some props have x-options.normalizeAlgo for log-curves (Q values, HP cutoffs);
// those are noted in the schema but we'll only special-case them when we hit one.

namespace HR {

inline float wireToDisplay(float wire, float dispMin, float dispMax) {
    return dispMin + wire * (dispMax - dispMin);
}

inline float displayToWire(float disp, float dispMin, float dispMax) {
    if (dispMax == dispMin) return 0.0f;
    float v = (disp - dispMin) / (dispMax - dispMin);
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return v;
}

// Quantize a display-unit value to the grid step (e.g. 0.01 BPM, 1 Hz, etc).
inline float snapToGrid(float disp, float grid) {
    if (grid <= 0.0f) return disp;
    return roundf(disp / grid) * grid;
}

// Convenience: read normalized wire, return display-unit string formatted per x-options.format.
// Caller passes the format string verbatim ("%.2f BPM", "%.0f Hz", "%.1f dB", "%.0f %%", ...).
inline String formatWire(float wire, float dispMin, float dispMax, const char* fmt) {
    float disp = wireToDisplay(wire, dispMin, dispMax);
    char buf[32];
    // The "%%" in HeadRush format strings is a literal "%" — snprintf handles that natively.
    snprintf(buf, sizeof(buf), fmt, disp);
    return String(buf);
}

} // namespace HR
