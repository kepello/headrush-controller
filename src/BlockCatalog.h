// Generic block taxonomy + live-from-device block resolution.
//
// Deliberately holds NO baked per-module snapshot. The Prime's own ModuleTypes
// (index -> name) and categoryOfBlock (name -> category) are the single source of
// truth, fetched at runtime. A stale local copy would let resolution "succeed"
// with the WRONG blocks after a firmware update reorders indices — silently —
// which is worse than failing visibly (this is a controller for a device that
// must be online anyway). What lives here is only our app's fixed taxonomy and
// conventions, which are not device content and change only when WE change them.
#pragma once
#include <Arduino.h>

// Generic block categories (the 20 device categories collapsed to the set the
// knob-default generator reasons about). Order is load-bearing — values are
// cached and compared elsewhere — so append, never reorder.
enum BlockCat : uint8_t {
    BC_NONE = 0,
    BC_AMP = 1,
    BC_CAB = 2,
    BC_CLONE = 3,
    BC_VOCAL = 4,
    BC_DRIVE = 5,
    BC_EQ = 6,
    BC_COMP = 7,
    BC_DELAY = 8,
    BC_REVERB = 9,
    BC_MOD = 10,
    BC_FILTER = 11,
    BC_PITCH = 12,
    BC_DYN = 13,
    BC_RHY = 14,
    BC_SYNTH = 15,
    BC_UTIL = 16,
    BC_FXLOOP = 17,
    BC_COUNT
};

// Live ModuleTypes cache, fetched from the Prime (/Evil/API/Blocks) and filled by
// the net task (loadModuleTypes). Allocated in PSRAM; null/0 until loaded, and a
// module is simply unresolved until then — no stale fallback. Single TU (main.cpp).
const int MAX_MODTYPES = 320;          // room above the current ~272 for firmware additions
char (*gModNameRT)[32] = nullptr;      // [idx] -> name
int  gModCountRT = 0;

inline String moduleName(int idx) {
    if (gModNameRT && idx >= 0 && idx < gModCountRT && gModNameRT[idx][0]) return String(gModNameRT[idx]);
    return String();                   // not loaded -> unresolved (caller handles; never guesses)
}

// Build "/Evil/Engine/Patch/<name>" for a module index (spaces/slash -> _).
inline String moduleBlockPath(int idx) {
    if (idx <= 0) return String();
    String n = moduleName(idx);
    if (n.isEmpty() || n == "Empty Slot") return String();
    for (size_t i = 0; i < n.length(); ++i) { char c = n[i]; if (c == ' ' || c == '/') n[i] = '_'; }
    return String("/Evil/Engine/Patch/") + n;
}

// Map the Prime's own display category string (from categoryOfBlock /
// BlockSelectorCategoriesForDisplay) to a generic BlockCat. This is a translation
// of the device's LIVE output, not a snapshot — it only needs editing if HeadRush
// adds a brand-new category (which would warrant a code change regardless).
inline BlockCat genericCatFromDeviceString(const char* s) {
    if (!s || !s[0]) return BC_NONE;
    struct M { const char* k; BlockCat c; };
    static const M m[] = {
        { "Amp", BC_AMP }, { "Cab/IR", BC_CAB }, { "Clone", BC_CLONE }, { "Vocal", BC_VOCAL },
        { "Overdrive", BC_DRIVE }, { "Distortion/Fuzz", BC_DRIVE }, { "EQ", BC_EQ },
        { "Compressor", BC_COMP }, { "Delay", BC_DELAY }, { "Reverb", BC_REVERB },
        { "Chorus", BC_MOD }, { "Phaser/Flanger", BC_MOD }, { "Vib/Trem/Rotary", BC_MOD },
        { "Wah/Filter", BC_FILTER }, { "Pitch", BC_PITCH }, { "Volume/Dynamics", BC_DYN },
        { "Rhythmic", BC_RHY }, { "Synth", BC_SYNTH }, { "Utility", BC_UTIL }, { "FX-Loop", BC_FXLOOP },
    };
    for (const auto& e : m) if (strcmp(s, e.k) == 0) return e.c;
    return BC_NONE;
}

// --- Per-category knob default ("the primary param to ride") ----------------
//
// For a generic category we expose ONE primary parameter on the dial. Blocks in
// a category aren't identical, so each category carries an ordered list of
// candidate prop names; the resolver picks the first the actual block exposes.
// These are category CONVENTIONS (verified against real blocks), not per-device
// snapshots — a new block in a category exposes one of these, so new/downloaded
// models resolve without any edit here.

struct CatBindSpec {
    BlockCat cat;
    const char* label;            // on-screen label for the dial
    const char* candidates[5];    // ordered prop names; first present on the block wins
};

const CatBindSpec CAT_BIND[] = {
    { BC_DRIVE,  "DRIVE",  { "Drive", "Gain", "Fuzz", "Distortion", nullptr } },
    { BC_AMP,    "GAIN",   { "GainA", "Gain", "Master", nullptr, nullptr } },
    { BC_COMP,   "COMP",   { "Sustain", "Compression", nullptr, nullptr, nullptr } },
    { BC_REVERB, "REVERB", { "Mix", nullptr, nullptr, nullptr, nullptr } },
    { BC_DELAY,  "DELAY",  { "Mix", "DelayFdbk", "Feedback", nullptr, nullptr } },
    { BC_MOD,    "MOD",    { "Mix", "Depth", "RotSpeed", "Rate", nullptr } },
    { BC_FILTER, "FILTER", { "Sensitivity", "Range", "Mix", "Freq", nullptr } },
    { BC_PITCH,  "PITCH",  { "Mix", "Pitch", nullptr, nullptr, nullptr } },
};
const int CAT_BIND_COUNT = sizeof(CAT_BIND) / sizeof(CAT_BIND[0]);

inline const CatBindSpec* catBindSpec(BlockCat c) {
    for (int i = 0; i < CAT_BIND_COUNT; ++i) if (CAT_BIND[i].cat == c) return &CAT_BIND[i];
    return nullptr;
}

// The two complementary buckets for the default dials 2 & 3 (UX_MODEL §5):
// dial 2 = top present GAIN/character source, dial 3 = top present AMBIENCE.
const BlockCat GAIN_BUCKET[]  = { BC_DRIVE, BC_AMP, BC_COMP };
const BlockCat SPACE_BUCKET[] = { BC_REVERB, BC_DELAY, BC_MOD, BC_FILTER, BC_PITCH };
const int GAIN_BUCKET_COUNT  = sizeof(GAIN_BUCKET) / sizeof(GAIN_BUCKET[0]);
const int SPACE_BUCKET_COUNT = sizeof(SPACE_BUCKET) / sizeof(SPACE_BUCKET[0]);
