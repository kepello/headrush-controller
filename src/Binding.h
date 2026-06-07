#pragma once
#include <Arduino.h>

// One thing the knob can be doing right now. A board owns a *ring* of these
// and the encoder push (long-press) cycles between them.
//
// Continuous: knob turns adjust a numeric property on the Prime. Arc gauge
// shows current value, big number in center, label at the bottom.
//
// List: knob turns scroll through a list (rig names, setlist names, ...).
// Short push commits via a method call. Display shows the highlighted
// item + index.

struct ContinuousBinding {
    const char* label;       // short on-screen label, e.g. "OUT", "TEMPO"
    // Optional small subtitle under the label: the specific device behind a
    // generic category dial, e.g. label "COMP" + device "Gray Comp". nullptr for
    // globals (Output/Tempo/…) where the label already names the thing.
    const char* device = nullptr;
    const char* path;        // "/Evil/Engine/Patch/Output"
    const char* prop;        // "RigVolume"
    float dispMin, dispMax;  // display units, e.g. -60..+36 dB
    float step;              // display units per detent, e.g. 0.5 dB
    const char* format;      // printf format for the value, e.g. "%+.1f"
    const char* unit;        // suffix shown small below value, e.g. "dB"
    // Color zones for the arc (in display units). Each zone is [from..to, color].
    // Arc segments use the zone color of whichever zone their midpoint is in.
    struct Zone { float to; uint16_t color; };
    Zone zones[5];           // up to 5 zones, terminate with {dispMax, ...}
    int zoneCount;
};

struct ListBinding {
    const char* label;            // "RIG", "SETLIST"
    const char* path;             // "/Evil/API/Rigs"
    const char* namesProp;        // "AllRigNames"
    const char* idsProp;          // "AllRigIds"
    const char* secondaryIdsProp; // optional, e.g. "RigSrIds" — pass alongside
    const char* commitMethod;     // "loadRig"
};

enum class BindingKind { Continuous, List };

struct Binding {
    BindingKind kind;
    ContinuousBinding c;
    ListBinding l;
};
