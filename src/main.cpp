// Stage 1B: knob bound to Master Output (Rig Volume) via a generic Binding,
// with an arc-gauge visualization. Same UI/net task split as 1A.
//
// FLASH WORKFLOW (CrowPanel 1.28" ESP32-S3):
//   1. If a previous app is running: hold BOOT, tap RESET, release BOOT.
//   2. After every successful flash: tap RESET once.
//   3. Port glob: /dev/cu.usbmodem*

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <NetworkClientSecure.h>
#include <HTTPUpdate.h>
#include <Preferences.h>
#include <nvs_flash.h>
#include <esp_random.h>
#include <time.h>
#include <AiEsp32RotaryEncoder.h>
#include "HeadRushClient.h"
#include "Normalize.h"
#include "Hardware.h"
#include "Display.h"
#include "Binding.h"
#include "BlockCatalog.h"
#include "Render.h"
#include "Roots.h"

#if __has_include("secrets.h")
  #include "secrets.h"
#endif
#ifndef WIFI_SSID
  #define WIFI_SSID ""
#endif
#ifndef WIFI_PSK
  #define WIFI_PSK ""
#endif
#ifndef HEADRUSH_HOST_OVERRIDE
  #define HEADRUSH_HOST_OVERRIDE ""
#endif
#ifndef OTA_PASSWORD
  #define OTA_PASSWORD ""
#endif
// Monotonic build number. Local dev builds are 0; the release CI injects
// -DFW_VERSION=N derived from the vN git tag. The pull client compares this
// against the published version.json to decide whether to update.
#ifndef FW_VERSION
  #define FW_VERSION 0
#endif
// Where the fleet pulls firmware from. The GitHub Release "latest" channel:
// version.json carries the build number + the firmware.bin URL. Same URL for
// every unit — one published release updates the whole fleet.
#ifndef FW_MANIFEST_URL
  #define FW_MANIFEST_URL "https://github.com/kepello/headrush-controller/releases/latest/download/version.json"
#endif

// The global controls — rig-independent engine parameters that exist on every
// rig (master out/in, tempo, width, global EQ). Anything device-specific (drive,
// comp, reverb, …) is NOT here; it's resolved live from the loaded rig's chain
// (see resolveLayoutForRig / gPresent). Ranges/formats from the device's API.
const ContinuousBinding PARAM_CATALOG[] = {
    { .label = "OUTPUT", .path = "/Evil/Engine/Patch/Output", .prop = "RigVolume",
      .dispMin = -60.0f, .dispMax = +36.0f, .step = 0.5f, .format = "%+.1f", .unit = "dB",
      .zones = { {-30.0f, 0x6B7F}, {-6.0f, 0x07E0}, {+6.0f, 0xFFE0}, {+36.0f, 0xF800} }, .zoneCount = 4 },
    { .label = "INPUT", .path = "/Evil/Engine/Patch/Input", .prop = "InputGain",
      .dispMin = -60.0f, .dispMax = +12.0f, .step = 0.5f, .format = "%+.1f", .unit = "dB",
      .zones = { {-12.0f, 0x6B7F}, {0.0f, 0x07E0}, {+6.0f, 0xFFE0}, {+12.0f, 0xF800} }, .zoneCount = 4 },
    { .label = "TEMPO", .path = "/Evil/Engine/Tempo", .prop = "Tempo",
      .dispMin = 30.0f, .dispMax = 240.0f, .step = 1.0f, .format = "%.0f", .unit = "BPM",
      .zones = { {240.0f, 0x07E0} }, .zoneCount = 1 },
    { .label = "WIDTH", .path = "/Evil/Engine/Patch/Output", .prop = "RigWidth",
      .dispMin = 0.0f, .dispMax = 100.0f, .step = 1.0f, .format = "%.0f", .unit = "%",
      .zones = { {33.0f, 0x6B7F}, {66.0f, 0x07E0}, {100.0f, 0xFFE0} }, .zoneCount = 3 },
    { .label = "BASS", .path = "/Evil/Engine/GlobalEQMain", .prop = "Gain1",
      .dispMin = -12.0f, .dispMax = +12.0f, .step = 0.5f, .format = "%+.1f", .unit = "dB",
      .zones = { {-3.0f, 0x6B7F}, {+3.0f, 0x07E0}, {+12.0f, 0xFFE0} }, .zoneCount = 3 },
    { .label = "TREBLE", .path = "/Evil/Engine/GlobalEQMain", .prop = "Gain4",
      .dispMin = -12.0f, .dispMax = +12.0f, .step = 0.5f, .format = "%+.1f", .unit = "dB",
      .zones = { {-3.0f, 0x6B7F}, {+3.0f, 0x07E0}, {+12.0f, 0xFFE0} }, .zoneCount = 3 },
};
const int PARAM_COUNT = sizeof(PARAM_CATALOG) / sizeof(PARAM_CATALOG[0]);   // 6 globals
// Selected from the board's view ring in setup(). Read-only after, except a
// short click (cycle) or config swaps the pointer.
const ContinuousBinding* activeBinding = &PARAM_CATALOG[0];

// View indices: 0..PARAM_COUNT-1 are the PARAM_CATALOG dials; then Tuner,
// Setlist, and Rig. A board's Home carries an ordered group of these. Setlist
// and Rig are scroll-to-select lists (turn scrolls names, a pause loads).
const int TUNER_VIEW   = PARAM_COUNT;
const int SETLIST_VIEW = PARAM_COUNT + 1;
const int RIG_VIEW     = PARAM_COUNT + 2;
// A dynamic, per-rig dial. There is ONE marker view; a board's Home can hold
// several, and each dynamic member's pool slot is simply its position in the
// group. The net task resolves each to a concrete block+param for the loaded rig
// (see resolveLayoutForRig); the descriptor in gHomeDesc[pos] says what it is.
const int DYN_VIEW     = PARAM_COUNT + 3;
const int VIEW_COUNT   = PARAM_COUNT + 4;
inline bool viewIsTuner(int v)   { return v == TUNER_VIEW; }
inline bool viewIsSetlist(int v) { return v == SETLIST_VIEW; }
inline bool viewIsRig(int v)     { return v == RIG_VIEW; }
inline bool viewIsList(int v)    { return v == SETLIST_VIEW || v == RIG_VIEW; }
inline bool viewIsDyn(int v)     { return v == DYN_VIEW; }
inline bool viewIsValid(int v)   { return (v >= 0 && v < PARAM_COUNT) || v == TUNER_VIEW
                                          || v == SETLIST_VIEW || v == RIG_VIEW || v == DYN_VIEW; }

const int MAX_GROUP = 8;   // max members in a board's Home group (= dynamic pool size)

// What a dynamic member represents. BUCKET_* track a category by priority (the
// default Gain/Ambience dials); DEVICE pins a specific block+param (an explicit
// Assign pick) with category fallback. Stored in NVS as the user's intent; the
// concrete block/value is always re-resolved live from the Prime on rig load.
enum DescKind : uint8_t { DK_NONE = 0, DK_BUCKET_GAIN = 1, DK_BUCKET_SPACE = 2, DK_DEVICE = 3 };
struct HomeDesc {
    uint8_t kind;     // DescKind
    uint8_t cat;      // BlockCat (DEVICE fallback / bucket result)
    char path[44];    // DK_DEVICE: pinned block path ("" otherwise)
    char prop[20];    // DK_DEVICE: pinned param ("" = the block's primary)
};
HomeDesc gHomeDesc[MAX_GROUP];   // descriptor per group position (valid where homeGroup[pos]==DYN_VIEW)

// Resolved binding per group position (slot). Net writes during resolution, UI
// reads to display. ContinuousBinding holds const char* fields, so the runtime
// strings live in these backing buffers and the struct points into them.
ContinuousBinding gDynBind[MAX_GROUP];
volatile bool     gDynOk[MAX_GROUP];              // did this slot resolve to a block?
char gDynLabel[MAX_GROUP][12];                    // big label = generic category ("COMP")
char gDynDevice[MAX_GROUP][24];                   // small subtitle = actual block ("Gray Comp")
char gDynPath[MAX_GROUP][48];
char gDynProp[MAX_GROUP][20];
char gDynFmt[MAX_GROUP][12];                      // printf format, parsed from device meta
char gDynUnit[MAX_GROUP][8];                      // unit suffix, parsed from device meta
volatile uint16_t gPresentCatMask = 0;            // bit c set => BlockCat c present in rig
volatile bool     layoutDirty = false;            // net -> UI: a new resolved group is published
volatile bool     gResolving = false;             // net -> UI: resolution in flight (show a spinner)
int8_t gCatCache[MAX_MODTYPES];                   // module idx -> BlockCat, -1 = not yet asked (net task)
volatile bool gBlocksOk = false;                  // live ModuleTypes loaded? (no resolution before this)

// The loaded rig's devices, built live from the Chain by the net task (never
// persisted — the rig can change on the Prime while we're disconnected). Doubles
// as the resolver's lookup table and the Assign screen's device list. Entries are
// in signal-chain order; each carries the device's primary param + display spec.
const int MAX_PRESENT = 16;
struct PresentDev {
    uint8_t cat;
    char path[48];
    char device[24];      // display name ("Gray Comp")
    char label[12];       // generic category label ("COMP")
    char prop[20];        // primary param
    float dispMin, dispMax;
    char fmt[16];         // raw device meta format ("%.0f %%"); split when a binding is built
};
PresentDev gPresent[MAX_PRESENT];
volatile int gPresentCount = 0;   // net writes, UI reads (stable between rig changes)

// Split a HeadRush meta format ("%.0f %%", "%.1f Cents", "%+.1f dB") into a bare
// printf format ("%.0f") + unit ("%", "Cents", "dB"). The Prime embeds the unit
// after the conversion; we draw it separately. "%%" decodes to a literal "%".
void splitDeviceFormat(const char* dev, char* fmt, size_t fl, char* unit, size_t ul) {
    if (!dev || !dev[0]) { strncpy(fmt, "%.0f", fl); strncpy(unit, "%", ul); fmt[fl-1]=0; unit[ul-1]=0; return; }
    size_t i = 0;
    while (dev[i]) { char c = dev[i++]; if (c=='f'||c=='d'||c=='g'||c=='e'||c=='i'||c=='u') break; }
    size_t n = i < fl - 1 ? i : fl - 1;
    memcpy(fmt, dev, n); fmt[n] = 0;
    while (dev[i] == ' ') i++;
    size_t u = 0;
    for (; dev[i] && u < ul - 1; ++i) { if (dev[i]=='%' && dev[i+1]=='%') continue; unit[u++] = dev[i]; }
    unit[u] = 0;
}

// Fill gDynBind[slot] from a resolved block: generic category label, the actual
// device name (subtitle), and the display spec (range/format) read live from the
// Prime's object-meta — so any param, with any unit/range, renders correctly.
void buildDynBinding(int slot, const char* label, const char* device, const String& path,
                     const char* prop, float dispMin, float dispMax, const char* devFormat) {
    strncpy(gDynLabel[slot], label, sizeof(gDynLabel[slot]) - 1);   gDynLabel[slot][sizeof(gDynLabel[slot]) - 1] = 0;
    strncpy(gDynDevice[slot], device ? device : "", sizeof(gDynDevice[slot]) - 1); gDynDevice[slot][sizeof(gDynDevice[slot]) - 1] = 0;
    strncpy(gDynPath[slot], path.c_str(), sizeof(gDynPath[slot]) - 1); gDynPath[slot][sizeof(gDynPath[slot]) - 1] = 0;
    strncpy(gDynProp[slot], prop, sizeof(gDynProp[slot]) - 1);      gDynProp[slot][sizeof(gDynProp[slot]) - 1] = 0;
    splitDeviceFormat(devFormat, gDynFmt[slot], sizeof(gDynFmt[slot]), gDynUnit[slot], sizeof(gDynUnit[slot]));
    if (dispMax <= dispMin) { dispMin = 0.0f; dispMax = 100.0f; }
    ContinuousBinding& b = gDynBind[slot];
    b.label = gDynLabel[slot];
    b.device = gDynDevice[slot];
    b.path  = gDynPath[slot];
    b.prop  = gDynProp[slot];
    b.dispMin = dispMin; b.dispMax = dispMax;
    float range = dispMax - dispMin;
    b.step = range > 20.0f ? 1.0f : range / 100.0f;   // ~1 unit/detent for %, finer for small ranges
    b.format = gDynFmt[slot]; b.unit = gDynUnit[slot];
    // Zones scaled to thirds of the range so arc colors work for any unit.
    b.zones[0] = { dispMin + range * 0.33f, 0x6B7F };
    b.zones[1] = { dispMin + range * 0.66f, 0x07E0 };
    b.zones[2] = { dispMax,                 0xFFE0 };
    b.zoneCount = 3;
}

// Tuner view shared state. The UI task sets tunerActive when the current ring
// slot is the Tuner; the net task then mutes the Prime (user's choice) and polls
// FFTCtrl for the live note + cents, publishing them here under stateMux.
const char* TUNER_CFG_PATH = "/Evil/Engine/AudioCtrl/Tuner";   // TunerMuting (bool)
const char* TUNER_FFT_PATH = "/Evil/Engine/FFTCtrl";           // TunerString, TunerCents
// FFTCtrl is a readout object and delivers RAW values, not the 0..1-normalized
// wire other params use (confirmed: TunerRef reads 440, meta -100..+100 "Cents").
// So TunerCents is already in cents — use it directly, don't run wireToDisplay.
volatile bool tunerActive = false;      // UI: current view is the tuner
volatile float tunerCents = 0.0f;       // net: latest pitch offset (display cents)
char tunerNote[8] = "--";               // net writes, UI reads (guarded by stateMux)
float lastTunerCents = -999.0f;         // UI redraw tracking
char lastTunerNote[8] = "";

// Library view shared state. A Setlist -> Rig browser. The UI task sets request
// flags + selection ids; the net task fetches over HTTP and fills the name/id
// buffers, then flips libReqDone. The request/done handshake orders access, so
// the buffers need no mutex (same pattern as the WiFi scan). UUIDs are 36 chars.
const char* LIB_SETLISTS_PATH = "/Evil/API/Setlists";
const char* LIB_RIGS_PATH     = "/Evil/API/Rigs";
const int LIB_MAX_SETLISTS = 16;
const int LIB_MAX_RIGS = 48;
// These library-browser name/id tables (~7KB) live in PSRAM, allocated in setup,
// to keep internal SRAM free for the OTA TLS handshake (which needs a large
// contiguous block). Pointer-to-array, so `libRigNames[i]` indexing is unchanged.
char (*libSetlistNames)[40];
char (*libSetlistIds)[40];
int  libSetlistCount = 0;
char (*libRigNames)[40];
char (*libRigIds)[40];
char (*libRigSrIds)[40];                     // setlist-id context per rig (loadRig needs it)
int  libRigCount = 0;
char libSelSetlistId[40] = "";
char libSelRigId[40] = "";
volatile bool libFetchSetlistsReq = false;  // UI -> net: load the setlist list
volatile bool fetchRigListReq = false;      // UI -> net: load the current setlist's rig list
volatile bool loadSetlistReq = false;       // UI -> net: loadSetlist(libSelSetlistId)
volatile bool libLoadRigReq = false;        // UI -> net: loadRig(libSelRigId, libSelSetlistId)
volatile bool libReqDone = false;           // net -> UI: last request finished
volatile bool libReqOk = false;             // net -> UI: it succeeded
// Currently-loaded rig / setlist (Prime's view). Net writes (HTTP fetch / WS push
// when the Prime swaps a rig); UI reads.
char libCurrentRigName[40] = "";
char libCurrentRigId[40] = "";
char libCurrentSetlistName[40] = "";

// Scroll-list Home state (Setlist / Rig knob types). turn scrolls the fetched
// names; single-click loads the highlight, 10 s idle reverts it. UI-task-local.
constexpr uint32_t LIST_REVERT_MS = 10000;  // idle before a browsed highlight reverts to the loaded item
int  listCursor = 0;
int  listCommitted = -1;                    // currently loaded index (browse previews away from it)
uint32_t lastListScrollMs = 0;
bool listLoading = false;
bool listDirty = false;

constexpr int ENCODER_SIGN = -1;
constexpr uint32_t WRITE_THROTTLE_MS = 30;
constexpr uint32_t ECHO_SUPPRESS_MS = 500;
constexpr uint32_t LONG_PRESS_MS = 1000;      // hold = back / open Board Menu
constexpr uint32_t DOUBLE_CLICK_MS = 250;     // window to detect a double-click (bypass)

// ---- Shared state ----
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
volatile float sharedValue = 0.0f;            // in display units (e.g. dB)
volatile float pendingWriteValue = 0;
volatile bool  pendingWriteValid = false;
volatile uint32_t lastLocalChangeMs = 0;
// Rig-change resolve trigger, debounced: a rig load/edit emits a BURST of WS
// frames (PresetName + every chain slot), so instead of resolving on each we
// push a timestamp and the net loop resolves once the burst goes quiet.
volatile bool     rigResolvePending = false;
volatile uint32_t rigResolveAtMs = 0;
constexpr uint32_t RIG_RESOLVE_DEBOUNCE_MS = 500;
inline void requestRigResolve() { rigResolveAtMs = millis(); rigResolvePending = true; }

// OTA state. Written by ArduinoOTA callbacks on the net task, read by the UI
// task to render the progress screen. Plain volatile is sufficient — these are
// 32-bit aligned and only drive a progress display, not control flow timing.
volatile bool otaActive = false;
volatile int  otaPercent = 0;
volatile bool otaChecking = false;        // true while polling GitHub for a manifest
char otaStatusMsg[40] = "checking...";    // shown on the checking screen (diagnostic)
volatile bool otaCheckRequested = false;  // set by UI long-press, consumed by net task
volatile bool rebindRequested = false;    // set when config picks a new param; net re-primes
volatile bool bypassToggleReq = false;    // UI double-click on Home: toggle focused block bypass (Phase 2)

HeadRushClient hr;
Preferences prefs;
CrowPanelLGFX gfx;
LGFX_Sprite canvas(&gfx);  // off-screen framebuffer; all UI renders here then blits
String gHostname;  // unique per device (MAC-derived); set once in netTask
AiEsp32RotaryEncoder encoder(HW::PIN_ENC_A, HW::PIN_ENC_B, HW::PIN_ENC_BTN, /*vcc=*/-1, /*steps_per_click=*/4);
void IRAM_ATTR encoderISR() { encoder.readEncoder_ISR(); }
long lastEncoderValue = 0;
float lastRenderedValue = -999999.0f;
int lastRenderedOtaPercent = -1;

// Backlight dimming: after a spell with no input or on-screen change, drop the
// LEDC duty to save power; any activity restores full brightness instantly.
constexpr int BL_FULL_DUTY = (DisplayHW::BL_DEFAULT_PCT * 255) / 100;
constexpr int BL_DIM_DUTY  = (12 * 255) / 100;   // ~12% when idle
constexpr uint32_t IDLE_DIM_MS = 30000;
uint32_t lastActivityMs = 0;
bool screenDimmed = false;
void noteActivity() {
    lastActivityMs = millis();
    if (screenDimmed) { screenDimmed = false; ledcWrite(HW::PIN_DISP_BL, BL_FULL_DUTY); }
}
void updateBacklight() {
    if (!screenDimmed && (millis() - lastActivityMs) > IDLE_DIM_MS) {
        screenDimmed = true;
        ledcWrite(HW::PIN_DISP_BL, BL_DIM_DUTY);
    }
}
enum UiMode { UI_GAUGE, UI_SPLASH, UI_UPDATING };
UiMode uiMode = UI_GAUGE;
volatile bool bootComplete = false;   // net task sets true after its boot sequence
char bootMsg[40] = "starting";        // boot-phase status shown on the splash

// On-screen connection indicator: a colored dot the net task drives.
enum ConnStatus { CS_BOOTING, CS_CONNECTING, CS_WIFI, CS_HEADRUSH, CS_WIFI_ERR, CS_HR_LOST };
volatile ConnStatus connStatus = CS_BOOTING;
uint16_t lastStatusColor = 1;  // != any real color, forces first draw
static uint16_t statusColor(ConnStatus s) {
    switch (s) {
        case CS_CONNECTING: return 0xFFE0;  // yellow — joining WiFi
        case CS_WIFI:       return 0x07FF;  // cyan — WiFi up, Prime not yet
        case CS_HEADRUSH:   return 0x07E0;  // green — Prime connected
        case CS_WIFI_ERR:   return 0xF800;  // red — WiFi error / disconnected
        case CS_HR_LOST:    return 0xFD20;  // orange — Prime disconnected
        default:            return 0x8410;  // grey — booting
    }
}
volatile int wifiRssi = 0;   // dBm, updated by the net task when online (0 = unknown)
int lastSignalLevel = -1;    // UI redraw tracking
static int signalLevel(int rssi) {   // 0..4 bars
    if (rssi == 0)    return 0;
    if (rssi >= -60)  return 4;
    if (rssi >= -68)  return 3;
    if (rssi >= -76)  return 2;
    if (rssi >= -84)  return 1;
    return 0;
}

// Per-device identity (1..16), set on-device and persisted in NVS.
volatile int deviceId = 1;
int paramIndex = 0;   // catalog index of the currently focused Home member

// Home assignment: an ordered group of controls the knob carries. Single-click
// cycles the focused member; turn adjusts it. Members are view indices (a global
// dial 0..PARAM_COUNT-1, Tuner/Setlist/Rig, or DYN_VIEW). For a DYN_VIEW member,
// gHomeDesc[pos] says what it is and gDynBind[pos] is its resolved binding.
// Net task writes the resolved group on rig load; the UI reads it.
int  homeGroup[MAX_GROUP] = { 0 };
int  homeGroupLen = 1;
int  homeFocus = 0;
volatile bool homeListActive = false;   // focused Home member is Setlist/Rig (net task reads it)

// Point the net task at the focused Home member. Tuner raises tunerActive; a
// Setlist/Rig list raises homeListActive and kicks off its fetch; a param view
// points activeBinding at the catalog entry. rebindRequested re-primes. UI task only.
void bindControl(int v) {
    tunerActive = false;
    homeListActive = false;
    if (viewIsTuner(v)) {
        tunerActive = true;
    } else if (viewIsList(v)) {
        homeListActive = true;
        listCursor = 0; listCommitted = -1; listDirty = true; listLoading = true;
        libReqDone = false;
        if (viewIsSetlist(v)) libFetchSetlistsReq = true;
        else                  fetchRigListReq = true;
    } else {
        if (v < 0 || v >= PARAM_COUNT) v = 0;
        paramIndex = v;
        activeBinding = &PARAM_CATALOG[v];
    }
    rebindRequested = true;
}

// Bind a dynamic Home member (slot == its group position). Its binding was
// resolved by the net task; if it didn't resolve (sparse rig / removed block)
// fall back to a global so the dial still does something useful.
void bindDynSlot(int slot) {
    tunerActive = false;
    homeListActive = false;
    if (slot >= 0 && slot < MAX_GROUP && gDynOk[slot]) {
        activeBinding = &gDynBind[slot];
    } else {
        int g = (slot >= 0 && slot < MAX_GROUP && gHomeDesc[slot].kind == DK_BUCKET_SPACE) ? 3 : 2;
        activeBinding = &PARAM_CATALOG[g];   // Width / Tempo
    }
    rebindRequested = true;
}

// Focus the i-th member of the Home group (wraps) and bind it.
void focusHomeMember(int i) {
    if (homeGroupLen < 1) homeGroupLen = 1;
    homeFocus = ((i % homeGroupLen) + homeGroupLen) % homeGroupLen;
    int v = homeGroup[homeFocus];
    if (viewIsDyn(v)) bindDynSlot(homeFocus);
    else              bindControl(v);
}

// UI screen model. Every screen obeys the universal grammar: turn = move,
// single-click = enter/next, double-click = bypass (Home only), hold = back.
// Parent links are fixed (handled inline at each screen); the tree is shallow.
enum Screen {
    SC_HOME,            // the assigned knob: value group, tuner, or scroll-list
    SC_BOARD_MENU,      // hold from Home
    SC_ASSIGN,          // multi-select the Home group (rig-derived root list)
    SC_ASSIGN_PARAMS,   // drill-in: pick params of one device
    SC_SETTINGS,        // device/global config menu
    SC_SET_ID,
    SC_WIFI, SC_WIFI_PW, SC_WIFI_CONNECTING
};
Screen screen = SC_HOME;
int boardSel = 0;       // cursor in the Board Menu
int menuIndex = 0;      // cursor in the Settings menu
int editIdValue = 1;

// ---- Assign screens (rig-derived) ----
// The group being edited, as members mirroring homeGroup + gHomeDesc.
int      asgView[MAX_GROUP];
HomeDesc asgDesc[MAX_GROUP];
int      asgLen = 0;

// Root rows, built from gPresent[] + globals + specials when Assign opens. Each
// device contributes a primary row + an "all params" drill row.
const int ASG_GLOBALS = PARAM_COUNT;   // every catalog entry is a rig-independent global
const int ASG_MAXROWS = MAX_PRESENT * 2 + ASG_GLOBALS + 3;
enum AsgRowKind : uint8_t { ARK_DEV_PRIMARY, ARK_DEV_PARAMS, ARK_GLOBAL, ARK_SPECIAL };
struct AsgRow { uint8_t kind; int16_t arg; };   // arg = present idx (devices) or view index
AsgRow asgRows[ASG_MAXROWS];
int    asgRowCount = 0;
int    asgCursor = 0;
bool   asgSel[ASG_MAXROWS];              // per-row selection (for the dot overlay)

// Param drill-in (one device). Fetched live from the Prime via the net task.
struct DevParam { char label[16]; char prop[20]; };
DevParam gDevParam[20];
volatile int  gDevParamCount = 0;
char          devParamsPath[48];        // UI -> net: which block to read params from
volatile bool devParamsReq = false;     // UI -> net
volatile bool devParamsDone = false;    // net -> UI
int  asgParamsDev = -1;                  // present idx being drilled
int  asgParamCursor = 0;
bool pSel[20];                           // per-param selection

// (Assign scratch + rig-derived rows are declared up by the Screen enum.)

// On-device WiFi setup: the net task scans/connects, the UI picks + types.
volatile bool scanRequested = false;  // UI -> net: run a scan
volatile bool scanDone = false;       // net -> UI: results ready
volatile int  scanCount = 0;
char scanSSIDs[16][33];
int  wifiPickIndex = 0;
char selectedSSID[33] = "";
// Password entry (encoder character picker). Spinner item 0 = DEL, 1..N = chars.
static const char PW_CHARS[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#$%^&*()_-+=.,:;/?~ ";
char wifiPassword[64] = "";
int  pwIndex = 1;
volatile bool wifiConnectRequested = false;  // UI -> net: try selectedSSID + wifiPassword
volatile int  wifiConnectResult = 0;          // 0=pending, 1=success, 2=failed
char wifiPassEntry[64] = "";                   // password handed to the net task

struct WifiCreds { String ssid; String psk; String hostOverride; };

WifiCreds loadCreds() {
    WifiCreds c;
    // On-device-saved creds (NVS) win — so a network configured on-device sticks
    // across reboots and OTA. Fall back to compiled secrets.h only when NVS is
    // empty, seeding it so OTA-delivered (CI) firmware can connect the first time.
    prefs.begin("hrctrl", true);
    c.ssid = prefs.getString("ssid", "");
    c.psk = prefs.getString("psk", "");
    c.hostOverride = prefs.getString("host", "");
    prefs.end();
    if (c.ssid.length() == 0 && strlen(WIFI_SSID) > 0) {
        c.ssid = WIFI_SSID;
        c.psk = WIFI_PSK;
        c.hostOverride = HEADRUSH_HOST_OVERRIDE;
        Preferences p;
        if (p.begin("hrctrl", false)) {
            p.putString("ssid", c.ssid);
            p.putString("psk", c.psk);
            p.putString("host", c.hostOverride);
            p.end();
        }
    }
    return c;
}

void saveDeviceId(int id) {
    Preferences p;
    bool ok = p.begin("hrctrl", false);
    size_t n = p.putInt("devid", id);
    int rb = p.getInt("devid", -1);
    p.end();
    Serial.printf("[nvs] devid<-%d (open=%d wrote=%u readback=%d)\n", id, ok, (unsigned)n, rb);
}

// Per-rig HOME assignment (UX_MODEL §5), keyed by the rig's stable loadedID. NVS
// keys are short (<=15 chars), so we hash the UUID into "h<8hex>L" (len) and
// "h<8hex>M" (member blob). The blob stores the user's intent only — view index +
// (for DYN members) the descriptor — never live Prime state. (The old "h..l/g"
// int-only format from v29 is left orphaned; those rigs just regenerate defaults.)
struct NvsMember {
    int16_t view;     // homeGroup view index
    uint8_t kind;     // HomeDesc kind (DK_NONE for non-DYN members)
    uint8_t cat;      // BlockCat
    char path[44];
    char prop[20];
};
void rigNvsKeys(const char* rigId, char* lenKey, char* memKey) {
    uint32_t h = 2166136261u;                         // FNV-1a over the rig id
    for (const char* s = rigId; *s; ++s) { h ^= (uint8_t)*s; h *= 16777619u; }
    snprintf(lenKey, 12, "h%08lxL", (unsigned long)h);
    snprintf(memKey, 12, "h%08lxM", (unsigned long)h);
}

// Load this rig's saved group into homeGroup[]/gHomeDesc[]. Returns false if none
// saved (caller generates defaults). Validation against the *live* rig happens
// later in resolveMembers(); here we only restore intent.
bool loadHomeForRig(const char* rigId) {
    if (!rigId || !rigId[0]) return false;
    char lk[12], mk[12]; rigNvsKeys(rigId, lk, mk);
    Preferences p; p.begin("hrctrl", true);
    int len = p.getInt(lk, 0);
    bool ok = false;
    if (len >= 1 && len <= MAX_GROUP) {
        NvsMember tmp[MAX_GROUP];
        size_t want = len * sizeof(NvsMember);
        if (p.getBytes(mk, tmp, want) == want) {
            int n = 0;
            for (int i = 0; i < len; ++i) {
                if (!viewIsValid(tmp[i].view)) continue;
                homeGroup[n] = tmp[i].view;
                gHomeDesc[n].kind = tmp[i].kind;
                gHomeDesc[n].cat  = tmp[i].cat;
                strncpy(gHomeDesc[n].path, tmp[i].path, sizeof(gHomeDesc[n].path)); gHomeDesc[n].path[sizeof(gHomeDesc[n].path)-1] = 0;
                strncpy(gHomeDesc[n].prop, tmp[i].prop, sizeof(gHomeDesc[n].prop)); gHomeDesc[n].prop[sizeof(gHomeDesc[n].prop)-1] = 0;
                n++;
            }
            if (n > 0) { homeGroupLen = n; ok = true; }
        }
    }
    p.end();
    if (ok) Serial.printf("[nvs] loaded rig layout %s len=%d\n", rigId, homeGroupLen);
    return ok;
}

void saveHomeForRig(const char* rigId) {
    if (!rigId || !rigId[0]) return;
    char lk[12], mk[12]; rigNvsKeys(rigId, lk, mk);
    NvsMember tmp[MAX_GROUP];
    int n = homeGroupLen > MAX_GROUP ? MAX_GROUP : homeGroupLen;
    memset(tmp, 0, sizeof(tmp));
    for (int i = 0; i < n; ++i) {
        tmp[i].view = (int16_t)homeGroup[i];
        tmp[i].kind = gHomeDesc[i].kind;
        tmp[i].cat  = gHomeDesc[i].cat;
        strncpy(tmp[i].path, gHomeDesc[i].path, sizeof(tmp[i].path));
        strncpy(tmp[i].prop, gHomeDesc[i].prop, sizeof(tmp[i].prop));
    }
    Preferences p; p.begin("hrctrl", false);
    p.putInt(lk, n);
    p.putBytes(mk, tmp, n * sizeof(NvsMember));
    p.end();
    Serial.printf("[nvs] saved rig layout %s len=%d\n", rigId, n);
}

// Generate this board's default HOME for the loaded rig (UX_MODEL §5). Role by
// device id (1..4 -> nav / gain / ambience / levels; >4 wraps). The Gain/Ambience
// dials are DYN members whose descriptor tracks a bucket; the net task resolves
// the concrete block. Sets homeGroup[]/gHomeDesc[]/homeGroupLen.
void generateDefaultHome() {
    memset(gHomeDesc, 0, sizeof(gHomeDesc));
    int role = (deviceId - 1) % 4;
    switch (role) {
        case 0:  homeGroup[0] = RIG_VIEW; homeGroup[1] = SETLIST_VIEW; homeGroupLen = 2; break;
        case 1:  homeGroup[0] = DYN_VIEW; homeGroupLen = 1; gHomeDesc[0].kind = DK_BUCKET_GAIN;  break;
        case 2:  homeGroup[0] = DYN_VIEW; homeGroupLen = 1; gHomeDesc[0].kind = DK_BUCKET_SPACE; break;
        default: homeGroup[0] = 0; homeGroup[1] = 1; homeGroupLen = 2; break;  // OUTPUT, INPUT
    }
    Serial.printf("[layout] default home for board %d role %d len %d\n", deviceId, role, homeGroupLen);
}

// ---- Assign screen helpers (UI task) ----
bool asgMatchDevice(int m, const char* path, const char* prop) {
    return asgView[m] == DYN_VIEW && asgDesc[m].kind == DK_DEVICE
        && strcmp(asgDesc[m].path, path) == 0 && strcmp(asgDesc[m].prop, prop) == 0;
}
int asgFindDevice(const char* path, const char* prop) {
    for (int m = 0; m < asgLen; ++m) if (asgMatchDevice(m, path, prop)) return m;
    return -1;
}
int asgFindView(int view) {
    for (int m = 0; m < asgLen; ++m) if (asgView[m] == view && asgDesc[m].kind == DK_NONE) return m;
    return -1;
}
void asgRemoveAt(int m) {
    for (int i = m; i < asgLen - 1; ++i) { asgView[i] = asgView[i + 1]; asgDesc[i] = asgDesc[i + 1]; }
    asgLen--;
}
void asgToggleDevice(uint8_t cat, const char* path, const char* prop) {
    int m = asgFindDevice(path, prop);
    if (m >= 0) { asgRemoveAt(m); return; }
    if (asgLen >= MAX_GROUP) return;
    asgView[asgLen] = DYN_VIEW;
    HomeDesc& d = asgDesc[asgLen];
    d.kind = DK_DEVICE; d.cat = cat;
    strncpy(d.path, path, sizeof(d.path)); d.path[sizeof(d.path) - 1] = 0;
    strncpy(d.prop, prop, sizeof(d.prop)); d.prop[sizeof(d.prop) - 1] = 0;
    asgLen++;
}
void asgToggleView(int view) {
    int m = asgFindView(view);
    if (m >= 0) { asgRemoveAt(m); return; }
    if (asgLen >= MAX_GROUP) return;
    asgView[asgLen] = view;
    asgDesc[asgLen].kind = DK_NONE; asgDesc[asgLen].cat = 0;
    asgDesc[asgLen].path[0] = 0; asgDesc[asgLen].prop[0] = 0;
    asgLen++;
}

// Build the rig-derived root rows: globals first (always available, predictable),
// then the rig's devices (primary + "more…"), then Tuner/Setlist/Rig. Labels are
// computed on the fly when rendering (only the current row shows), so no per-row
// string storage.
void buildAssignRows() {
    asgRowCount = 0;
    for (int g = 0; g < ASG_GLOBALS && asgRowCount < ASG_MAXROWS; ++g)
        asgRows[asgRowCount++] = { ARK_GLOBAL, (int16_t)g };
    int pc = gPresentCount; if (pc > MAX_PRESENT) pc = MAX_PRESENT;
    for (int p = 0; p < pc && asgRowCount < ASG_MAXROWS - 2; ++p) {
        asgRows[asgRowCount++] = { ARK_DEV_PRIMARY, (int16_t)p };
        asgRows[asgRowCount++] = { ARK_DEV_PARAMS, (int16_t)p };
    }
    const int sp[3] = { TUNER_VIEW, SETLIST_VIEW, RIG_VIEW };
    for (int s = 0; s < 3 && asgRowCount < ASG_MAXROWS; ++s)
        asgRows[asgRowCount++] = { ARK_SPECIAL, (int16_t)sp[s] };
}

// Display strings for one root row (big = category/control, small = device / "").
void asgRowLabels(int r, char* big, size_t bl, char* small, size_t sl) {
    big[0] = 0; small[0] = 0;
    if (r < 0 || r >= asgRowCount) return;
    const AsgRow& row = asgRows[r];
    if (row.kind == ARK_DEV_PRIMARY) {
        strncpy(big, gPresent[row.arg].label, bl - 1); big[bl - 1] = 0;
        strncpy(small, gPresent[row.arg].device, sl - 1); small[sl - 1] = 0;
    } else if (row.kind == ARK_DEV_PARAMS) {
        strncpy(big, "more...", bl - 1); big[bl - 1] = 0;
        strncpy(small, gPresent[row.arg].device, sl - 1); small[sl - 1] = 0;
    } else if (row.kind == ARK_GLOBAL) {
        strncpy(big, PARAM_CATALOG[row.arg].label, bl - 1); big[bl - 1] = 0;
    } else {
        const char* n = (row.arg == TUNER_VIEW) ? "TUNER" : (row.arg == SETLIST_VIEW) ? "SETLIST" : "RIG";
        strncpy(big, n, bl - 1); big[bl - 1] = 0;
    }
}

// Recompute which root rows are in the scratch group (for the dot overlay).
void computeAsgSelected() {
    for (int r = 0; r < asgRowCount; ++r) {
        const AsgRow& row = asgRows[r];
        bool sel = false;
        if (row.kind == ARK_DEV_PRIMARY) {
            PresentDev& pd = gPresent[row.arg];
            sel = asgFindDevice(pd.path, pd.prop) >= 0;
        } else if (row.kind == ARK_DEV_PARAMS) {
            PresentDev& pd = gPresent[row.arg];
            for (int m = 0; m < asgLen && !sel; ++m)
                if (asgView[m] == DYN_VIEW && asgDesc[m].kind == DK_DEVICE
                    && strcmp(asgDesc[m].path, pd.path) == 0 && strcmp(asgDesc[m].prop, pd.prop) != 0) sel = true;
        } else {
            sel = asgFindView(row.arg) >= 0;
        }
        asgSel[r] = sel;
    }
}

void seedAssignFromHome() {
    asgLen = homeGroupLen > MAX_GROUP ? MAX_GROUP : homeGroupLen;
    for (int i = 0; i < asgLen; ++i) { asgView[i] = homeGroup[i]; asgDesc[i] = gHomeDesc[i]; }
    asgCursor = 0;
}

// Copy the scratch group into the live group, persist it, and let the net task
// re-resolve + republish (a spinner shows during the brief delay).
void commitAssign() {
    int n = asgLen < 1 ? 1 : asgLen;
    homeGroupLen = n;
    for (int i = 0; i < n; ++i) { homeGroup[i] = asgView[i]; gHomeDesc[i] = asgDesc[i]; }
    if (asgLen < 1) { homeGroup[0] = 0; memset(&gHomeDesc[0], 0, sizeof(HomeDesc)); }
    homeFocus = 0;
    char rigId[40];
    portENTER_CRITICAL(&stateMux);
    strncpy(rigId, libCurrentRigId, sizeof(rigId)); rigId[sizeof(rigId) - 1] = 0;
    portEXIT_CRITICAL(&stateMux);
    saveHomeForRig(rigId);
    requestRigResolve();
}

void renderAssignRoot() {
    computeAsgSelected();
    char big[16], small[24];
    asgRowLabels(asgCursor, big, sizeof(big), small, sizeof(small));
    Render::drawAssignList(canvas, "ASSIGN", big, small, asgRowCount, asgCursor, asgSel);
}

// Render the param drill-in: mark which params are in the group, show the current
// one big. Reads gDevParam (filled by the net task).
void renderAssignParams() {
    if (asgParamsDev < 0 || asgParamsDev >= gPresentCount) return;
    PresentDev& pd = gPresent[asgParamsDev];
    int n = gDevParamCount; if (n > 20) n = 20;
    for (int i = 0; i < n; ++i) pSel[i] = asgFindDevice(pd.path, gDevParam[i].prop) >= 0;
    const char* big = (asgParamCursor >= 0 && asgParamCursor < n) ? gDevParam[asgParamCursor].label : "";
    Render::drawAssignList(canvas, pd.device, big, "", n, asgParamCursor, pSel);
}

void enterAssignParams(int p) {
    asgParamsDev = p;
    asgParamCursor = 0;
    strncpy(devParamsPath, gPresent[p].path, sizeof(devParamsPath)); devParamsPath[sizeof(devParamsPath) - 1] = 0;
    gDevParamCount = 0;
    devParamsDone = false;
    devParamsReq = true;                 // net fetches the device's params
    screen = SC_ASSIGN_PARAMS;
    Render::drawListLoading(canvas, "loading...");
}

bool connectWifi(const WifiCreds& c) {
    if (c.ssid.length() == 0) { Serial.println("No WiFi creds"); return false; }
    Serial.printf("WiFi → %s\n", c.ssid.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(c.ssid.c_str(), c.psk.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 30000) { delay(250); Serial.print('.'); }
    Serial.println();
    if (WiFi.status() != WL_CONNECTED) { Serial.println("WiFi connect failed"); return false; }
    Serial.printf("WiFi OK: %s\n", WiFi.localIP().toString().c_str());
    return true;
}

// Unique, stable per-device name from the low 24 bits of the factory MAC, e.g.
// "headrush-3AF1C2". Lets 4 identical-firmware units coexist on mDNS so each can
// be addressed individually for OTA push.
String makeHostname() {
    uint64_t mac = ESP.getEfuseMac();
    char buf[20];
    snprintf(buf, sizeof(buf), "headrush-%06llX", (unsigned long long)(mac & 0xFFFFFFULL));
    return String(buf);
}

String resolveHost(const WifiCreds& c) {
    if (c.hostOverride.length() > 0) return c.hostOverride;
    if (!MDNS.begin(gHostname.c_str())) Serial.println("mDNS responder failed");
    IPAddress ip = MDNS.queryHost("headrushprime");
    if (ip != IPAddress((uint32_t)0)) return ip.toString();
    return String("headrushprime.local");
}

// Net-task helpers. Called only from net task.
void onValueChanged(const String& path, const String& prop, const JsonVariantConst& value) {
    // Track the currently-loaded rig + setlist names so the library idle display
    // reflects external changes (footswitch, other controller) without polling.
    if (path == "/Evil/Engine/Patch/Rig" && prop == "PresetName" && value.is<const char*>()) {
        const char* nm = value.as<const char*>();
        portENTER_CRITICAL(&stateMux);
        strncpy(libCurrentRigName, nm ? nm : "", 39); libCurrentRigName[39] = 0;
        portEXIT_CRITICAL(&stateMux);
        requestRigResolve();   // rig swapped (footswitch / other controller) — re-resolve layout
    } else if (path == LIB_SETLISTS_PATH && prop == "loadedName" && value.is<const char*>()) {
        const char* nm = value.as<const char*>();
        portENTER_CRITICAL(&stateMux);
        strncpy(libCurrentSetlistName, nm ? nm : "", 39); libCurrentSetlistName[39] = 0;
        portEXIT_CRITICAL(&stateMux);
    }
    // Editing a rig (swap/add/remove a block) changes the Chain but not the rig
    // name, so re-resolve on any Chain structure change too — covers edits made
    // on the Prime or anywhere, not just rig selection.
    if (path == "/Evil/Engine/Patch/Chain") requestRigResolve();
    // Param echo for the active dial.
    if (path != activeBinding->path || prop != activeBinding->prop) return;
    portENTER_CRITICAL(&stateMux);
    uint32_t since = millis() - lastLocalChangeMs;
    portEXIT_CRITICAL(&stateMux);
    if (since < ECHO_SUPPRESS_MS) return;
    float wire = value.as<float>();
    float disp = HR::wireToDisplay(wire, activeBinding->dispMin, activeBinding->dispMax);
    portENTER_CRITICAL(&stateMux);
    sharedValue = disp;
    portEXIT_CRITICAL(&stateMux);
    Serial.printf("← %s %.2f (external)\n", activeBinding->label, disp);
}

void primeInitialValue() {
    JsonDocument doc;
    if (hr.getProperties(activeBinding->path, doc)) {
        float wire = doc[activeBinding->prop].as<float>();
        float disp = HR::wireToDisplay(wire, activeBinding->dispMin, activeBinding->dispMax);
        portENTER_CRITICAL(&stateMux);
        sharedValue = disp;
        portEXIT_CRITICAL(&stateMux);
        Serial.printf("Initial %s = %.2f (wire=%.4f)\n", activeBinding->label, disp, wire);
    }
}

// Reconcile the Prime tuner with the UI's tunerActive flag, and while engaged
// poll FFTCtrl for the live note + cents. Called every net-loop iteration when
// online. Muting follows the user's "mute while tuning" choice.
void serviceTuner() {
    static bool engaged = false;
    static uint32_t lastPollMs = 0;
    if (tunerActive && !engaged) {
        hr.setProperty(TUNER_CFG_PATH, "TunerMuting", true);
        engaged = true;
        lastPollMs = 0;  // poll right away
    } else if (!tunerActive && engaged) {
        hr.setProperty(TUNER_CFG_PATH, "TunerMuting", false);
        engaged = false;
        portENTER_CRITICAL(&stateMux);
        strcpy(tunerNote, "--");
        tunerCents = 0.0f;
        portEXIT_CRITICAL(&stateMux);
    }
    if (engaged && millis() - lastPollMs >= 120) {
        lastPollMs = millis();
        JsonDocument doc;
        if (hr.getProperties(TUNER_FFT_PATH, doc)) {
            float cents = doc["TunerCents"].as<float>();   // raw cents (FFTCtrl is not normalized)
            const char* note = doc["TunerString"].is<const char*>() ? doc["TunerString"].as<const char*>() : "--";
            portENTER_CRITICAL(&stateMux);
            tunerCents = cents;
            strncpy(tunerNote, note, sizeof(tunerNote));
            tunerNote[sizeof(tunerNote) - 1] = 0;
            portEXIT_CRITICAL(&stateMux);
        }
    }
}

// Library net-task helpers. Each fills shared buffers, then flips libReqDone.
void doLibFetchSetlists() {
    libReqOk = false;
    JsonDocument doc;
    if (hr.getProperties(LIB_SETLISTS_PATH, doc)) {
        JsonArrayConst names = doc["SetlistNames"].as<JsonArrayConst>();
        JsonArrayConst ids = doc["SetlistIds"].as<JsonArrayConst>();
        int n = 0;
        for (size_t i = 0; i < names.size() && i < ids.size() && n < LIB_MAX_SETLISTS; ++i) {
            strncpy(libSetlistNames[n], names[i] | "", 39); libSetlistNames[n][39] = 0;
            strncpy(libSetlistIds[n], ids[i] | "", 39);     libSetlistIds[n][39] = 0;
            n++;
        }
        if (n > 0) libSetlistCount = n;   // keep the last good list on a transient/empty fetch
        libReqOk = (n > 0);
        const char* loaded = doc["loadedName"] | "";
        portENTER_CRITICAL(&stateMux);
        strncpy(libCurrentSetlistName, loaded, 39); libCurrentSetlistName[39] = 0;
        portEXIT_CRITICAL(&stateMux);
    }
    libReqDone = true;
}

// Fetch the current setlist's rig list (names/ids/srIds) plus the loaded rig.
void doFetchRigList() {
    libReqOk = false;
    JsonDocument doc;
    if (hr.getProperties(LIB_RIGS_PATH, doc)) {
        JsonArrayConst names = doc["RigNames"].as<JsonArrayConst>();
        JsonArrayConst ids   = doc["RigIds"].as<JsonArrayConst>();
        JsonArrayConst sr    = doc["RigSrIds"].as<JsonArrayConst>();
        int n = 0;
        for (size_t i = 0; i < names.size() && i < ids.size() && n < LIB_MAX_RIGS; ++i) {
            strncpy(libRigNames[n], names[i] | "", 39); libRigNames[n][39] = 0;
            strncpy(libRigIds[n], ids[i] | "", 39);     libRigIds[n][39] = 0;
            strncpy(libRigSrIds[n], (i < sr.size() ? (sr[i] | "") : ""), 39); libRigSrIds[n][39] = 0;
            n++;
        }
        if (n > 0) libRigCount = n;   // keep the last good list on a transient/empty fetch
        libReqOk = (n > 0);
        const char* nm = doc["loadedName"] | "";
        const char* id = doc["loadedID"] | "";
        portENTER_CRITICAL(&stateMux);
        strncpy(libCurrentRigName, nm, 39); libCurrentRigName[39] = 0;
        strncpy(libCurrentRigId, id, 39);   libCurrentRigId[39] = 0;
        portEXIT_CRITICAL(&stateMux);
    }
    libReqDone = true;
}

// Fetch the device's live ModuleTypes (index -> name) once and cache it in PSRAM,
// so resolution uses the Prime's current block list rather than the baked
// snapshot — robust to firmware updates / new blocks. Net task only; idempotent.
void loadModuleTypes() {
    if (gBlocksOk || !gModNameRT) return;
    JsonDocument doc;
    if (!hr.getProperties("/Evil/API/Blocks", doc)) { Serial.println("[blocks] ModuleTypes fetch FAILED — will retry"); return; }
    JsonArrayConst mt = doc["ModuleTypes"].as<JsonArrayConst>();
    int n = 0;
    for (size_t i = 0; i < mt.size() && n < MAX_MODTYPES; ++i) {
        strncpy(gModNameRT[n], mt[i] | "", 31); gModNameRT[n][31] = 0;
        n++;
    }
    if (n > 0) {
        gModCountRT = n;
        gBlocksOk = true;
        memset(gCatCache, -1, sizeof(gCatCache));
        Serial.printf("[blocks] %d module types loaded from device\n", n);
    }
}

// Generic category for a chain module index, asked from the Prime itself
// (categoryOfBlock) so a newly added/downloaded block categorizes correctly,
// cached per index for the session. Falls back to the baked table if the device
// call fails. Net task only.
BlockCat resolveCat(int idx) {
    if (idx <= 0 || idx >= MAX_MODTYPES) return BC_NONE;
    if (gCatCache[idx] >= 0) return (BlockCat)gCatCache[idx];
    String name = moduleName(idx);
    if (name.isEmpty()) return BC_NONE;            // block list not loaded -> unresolved (no guess)
    JsonDocument args; JsonArray a = args.to<JsonArray>(); a.add(name);
    JsonDocument res;
    if (hr.callMethod("/Evil/API/Blocks", "categoryOfBlock", args.as<JsonVariantConst>(), res)) {
        BlockCat c = genericCatFromDeviceString(res["methodReturnValue"] | "");
        gCatCache[idx] = (int8_t)c;                // cache the device's answer (incl. uncategorized)
        return c;
    }
    return BC_NONE;                                // transient failure: don't cache, retry next time
}

// Read one param's display spec (range + format) from object-meta. Falls back to
// 0..100 "%.0f %%" (the common "amount" param) when meta is missing.
void metaSpec(const String& path, const char* prop, float& mn, float& mx, char* fmt, size_t fl) {
    mn = 0; mx = 100; strncpy(fmt, "%.0f %%", fl); fmt[fl - 1] = 0;
    JsonDocument md;
    if (!hr.getMeta(path, md)) return;
    JsonVariantConst pm = md["properties"][prop];
    if (pm.isNull()) return;
    if (pm["minimum"].is<float>()) mn = pm["minimum"].as<float>();
    if (pm["maximum"].is<float>()) mx = pm["maximum"].as<float>();
    const char* f = pm["x-options"]["format"] | "";
    if (f[0]) { strncpy(fmt, f, fl); fmt[fl - 1] = 0; }
}

// Build the loaded rig's live device list (net task): walk the chain in order,
// and for each rideable block record its category, primary param, and display
// spec. Drives both the resolver and the Assign screen. Never persisted.
void buildPresent(JsonDocument& chain) {
    gPresentCount = 0;
    uint16_t mask = 0;
    for (int slot = 1; slot <= 14 && gPresentCount < MAX_PRESENT; ++slot) {
        char key[16]; snprintf(key, sizeof(key), "ModuleType%d", slot);
        int idx = chain[key] | 0;
        BlockCat c = resolveCat(idx);
        const CatBindSpec* spec = catBindSpec(c);
        if (!spec) continue;                          // not a rideable category
        String path = moduleBlockPath(idx);
        if (path.isEmpty()) continue;
        JsonDocument bd;
        if (!hr.getProperties(path, bd)) continue;
        const char* prop = nullptr;
        for (int k = 0; k < 5 && spec->candidates[k]; ++k)
            if (bd[spec->candidates[k]].is<float>()) { prop = spec->candidates[k]; break; }
        if (!prop) continue;
        PresentDev& pd = gPresent[gPresentCount++];
        pd.cat = (uint8_t)c;
        strncpy(pd.path, path.c_str(), sizeof(pd.path) - 1); pd.path[sizeof(pd.path) - 1] = 0;
        strncpy(pd.device, moduleName(idx).c_str(), sizeof(pd.device) - 1); pd.device[sizeof(pd.device) - 1] = 0;
        strncpy(pd.label, spec->label, sizeof(pd.label) - 1); pd.label[sizeof(pd.label) - 1] = 0;
        strncpy(pd.prop, prop, sizeof(pd.prop) - 1); pd.prop[sizeof(pd.prop) - 1] = 0;
        metaSpec(path, prop, pd.dispMin, pd.dispMax, pd.fmt, sizeof(pd.fmt));
        mask |= (uint16_t)(1u << c);
    }
    gPresentCatMask = mask;
}

void buildFromPresent(int slot, int p) {
    PresentDev& pd = gPresent[p];
    buildDynBinding(slot, pd.label, pd.device, String(pd.path), pd.prop, pd.dispMin, pd.dispMax, pd.fmt);
    gDynOk[slot] = true;
}

// The present-device index a bucket would pick first (highest priority cat, first
// in chain order), or -1 if none. Used so cross-fill can skip the other dial's pick.
int bucketTop(const BlockCat* bucket, int n) {
    for (int b = 0; b < n; ++b)
        for (int p = 0; p < gPresentCount; ++p)
            if (gPresent[p].cat == (uint8_t)bucket[b]) return p;
    return -1;
}
// Fill a dynamic slot from a bucket, optionally skipping one present index.
// Returns the present index used, or -1 if none.
int fillBucket(int slot, const BlockCat* bucket, int n, int except = -1) {
    for (int b = 0; b < n; ++b)
        for (int p = 0; p < gPresentCount; ++p)
            if (p != except && gPresent[p].cat == (uint8_t)bucket[b]) { buildFromPresent(slot, p); return p; }
    return -1;
}

// Resolve a pinned DEVICE member: exact block present -> bind it; else a block of
// the same category -> follow to it; else leave unresolved (UI falls to a global).
void resolveDeviceSlot(int slot, HomeDesc& d) {
    for (int p = 0; p < gPresentCount; ++p) {
        if (strcmp(gPresent[p].path, d.path) != 0) continue;
        if (d.prop[0] && strcmp(d.prop, gPresent[p].prop) != 0) {
            float mn, mx; char fmt[16];
            metaSpec(String(d.path), d.prop, mn, mx, fmt, sizeof(fmt));
            const CatBindSpec* spec = catBindSpec((BlockCat)d.cat);
            buildDynBinding(slot, spec ? spec->label : gPresent[p].label, gPresent[p].device,
                            String(d.path), d.prop, mn, mx, fmt);
            gDynOk[slot] = true;
        } else {
            buildFromPresent(slot, p);
        }
        return;
    }
    for (int p = 0; p < gPresentCount; ++p)
        if (gPresent[p].cat == d.cat) { buildFromPresent(slot, p); return; }
    gDynOk[slot] = false;
}

void resolveSlot(int slot) {
    gDynOk[slot] = false;
    HomeDesc& d = gHomeDesc[slot];
    // Cross-fill skips the OTHER bucket's top pick so a sparse rig (e.g. amp only)
    // doesn't show the same device on both the Gain and Ambience dials — the empty
    // one falls back to a global instead.
    if (d.kind == DK_BUCKET_GAIN) {
        if (fillBucket(slot, GAIN_BUCKET, GAIN_BUCKET_COUNT) < 0)
            fillBucket(slot, SPACE_BUCKET, SPACE_BUCKET_COUNT, bucketTop(SPACE_BUCKET, SPACE_BUCKET_COUNT));
    } else if (d.kind == DK_BUCKET_SPACE) {
        if (fillBucket(slot, SPACE_BUCKET, SPACE_BUCKET_COUNT) < 0)
            fillBucket(slot, GAIN_BUCKET, GAIN_BUCKET_COUNT, bucketTop(GAIN_BUCKET, GAIN_BUCKET_COUNT));
    } else if (d.kind == DK_DEVICE) {
        resolveDeviceSlot(slot, d);
    }
}

// Resolve every DYN member of the current group into its binding slot.
void resolveMembers() {
    for (int i = 0; i < homeGroupLen && i < MAX_GROUP; ++i) {
        if (homeGroup[i] == DYN_VIEW) {
            resolveSlot(i);
            if (gDynOk[i]) Serial.printf("[layout] slot%d = %s [%s] %s.%s\n", i,
                gDynBind[i].label, gDynBind[i].device, gDynBind[i].path, gDynBind[i].prop);
            else Serial.printf("[layout] slot%d unresolved -> global fallback\n", i);
        } else {
            gDynOk[i] = false;
        }
    }
}

// Per-rig resolution, owned by the net task (UX_MODEL §5; ownership model in
// PLAN #1): build the live device list, adopt this rig's saved group (or generate
// + persist the default), resolve every member, and publish to the UI. Called on
// rig load and on rig edits (chain change), and after an Assign save.
void resolveLayoutForRig() {
    gResolving = true;
    loadModuleTypes();           // live block list must be cached before we can resolve
    if (!gBlocksOk) {
        // No device block list — do NOT fabricate a layout from stale data. Leave
        // the spinner up; the net loop keeps retrying loadModuleTypes and fires a
        // fresh resolve once it succeeds. A persistent spinner + the connection
        // bars make a real link/API problem visible instead of silently wrong.
        Serial.println("[layout] blocks not loaded — deferring resolve");
        return;
    }
    JsonDocument chain;
    if (hr.getProperties("/Evil/Engine/Patch/Chain", chain)) buildPresent(chain);
    else { Serial.println("[layout] chain fetch FAIL"); gPresentCount = 0; }

    char rigId[40];
    portENTER_CRITICAL(&stateMux);
    strncpy(rigId, libCurrentRigId, sizeof(rigId)); rigId[sizeof(rigId) - 1] = 0;
    portEXIT_CRITICAL(&stateMux);

    if (!loadHomeForRig(rigId)) { generateDefaultHome(); saveHomeForRig(rigId); }
    resolveMembers();

    gResolving = false;
    layoutDirty = true;   // UI: a new resolved group is ready
}

// Fetch one device's named params for the Assign drill-in (net task). The block's
// `order`/`labels` arrays are the Prime's own knob layout; we keep the numeric,
// labeled params (skipping routing/switches) so the user picks from real knobs.
void doFetchDevParams() {
    gDevParamCount = 0;
    JsonDocument doc;
    if (hr.getProperties(devParamsPath, doc)) {
        JsonArrayConst order  = doc["order"].as<JsonArrayConst>();
        JsonArrayConst labels = doc["labels"].as<JsonArrayConst>();
        for (size_t k = 0; k < order.size() && gDevParamCount < 20; ++k) {
            const char* prop = order[k] | "";
            const char* lab  = (k < labels.size()) ? (labels[k] | "") : "";
            if (!prop[0] || !lab[0]) continue;          // unnamed / hidden knob
            if (!doc[prop].is<float>()) continue;        // only numeric, ridable params
            int n = gDevParamCount;
            strncpy(gDevParam[n].label, lab, sizeof(gDevParam[n].label) - 1); gDevParam[n].label[sizeof(gDevParam[n].label) - 1] = 0;
            strncpy(gDevParam[n].prop, prop, sizeof(gDevParam[n].prop) - 1);  gDevParam[n].prop[sizeof(gDevParam[n].prop) - 1] = 0;
            gDevParamCount = n + 1;
        }
    }
    devParamsDone = true;
}

// Load the setlist whose id is in libSelSetlistId.
void doLoadSetlist() {
    JsonDocument args;
    JsonArray a = args.to<JsonArray>();
    a.add(libSelSetlistId);
    a.add(true);   // 2nd arg (boolean) — best-guess; adjust if it misbehaves
    JsonDocument res;
    bool ok = hr.callMethod(LIB_SETLISTS_PATH, "loadSetlist", args.as<JsonVariantConst>(), res);
    libReqOk = ok && (res["methodReturnValue"] | false);
    libReqDone = true;
}

void doLibLoadRig() {
    JsonDocument args;
    JsonArray a = args.to<JsonArray>();
    a.add(libSelRigId);
    a.add(libSelSetlistId);   // loadRig(rigId, srId) — srId is the setlist context
    JsonDocument res;
    bool ok = hr.callMethod(LIB_RIGS_PATH, "loadRig", args.as<JsonVariantConst>(), res);
    libReqOk = ok && (res["methodReturnValue"] | false);
    libReqDone = true;
}

// OTA. Set up once, after WiFi is up. mDNS is already running (resolveHost),
// so begin() just advertises the _arduino._tcp service under headrush-ctrl.
// Callbacks run in this (net) task; they only publish progress for the UI task.
void setupOTA() {
    ArduinoOTA.setHostname(gHostname.c_str());
    if (strlen(OTA_PASSWORD) > 0) ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]() {
        otaPercent = 0;
        otaActive = true;
        Serial.println("[ota] start");
    });
    ArduinoOTA.onProgress([](unsigned int prog, unsigned int total) {
        otaPercent = total ? (int)((prog * 100UL) / total) : 0;
    });
    ArduinoOTA.onEnd([]() {
        otaPercent = 100;
        Serial.println("[ota] done — rebooting");
    });
    ArduinoOTA.onError([](ota_error_t err) {
        Serial.printf("[ota] error %u\n", err);
        otaActive = false;  // resume normal UI; no reboot on failure
    });
    ArduinoOTA.begin();
    Serial.printf("[ota] ready at %s.local\n", gHostname.c_str());
}

bool syncClockFromHttp();  // defined below; used to self-heal the clock before TLS

// Pull update: fetch version.json from GitHub Releases over verified TLS (needs
// the clock set — re-syncs it here with retries), and if it advertises a build
// number higher than ours, download + flash firmware.bin. Runs in the net task;
// a successful flash reboots into the new image. Status shows on the UI.
void checkForUpdate(const char* reason) {
    snprintf(otaStatusMsg, sizeof(otaStatusMsg), "checking...");
    otaChecking = true;

    // Free the Prime WebSocket first. Verified TLS (mbedTLS) needs a large
    // contiguous heap block for the handshake; after the controller has been
    // running, the WS buffers fragment the heap enough that the TLS connection
    // fails ("check failed -1") or the download dies ("update failed 0") — which
    // is why a manual check fails but the same check at boot (clean heap) works.
    // Dropping the WS reclaims that space; the net loop reconnects it afterward
    // (or we reboot into the new image on success).
    hr.stop();
    delay(50);
    Serial.printf("[ota] %s: free=%uk maxblk=%uk\n", reason,
                  (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getMaxAllocHeap() / 1024));

    // Verified TLS needs a real clock; re-sync (with retries) if the boot sync
    // didn't take. NTP is often blocked and a one-shot HTTPS Date can miss.
    for (int i = 0; i < 3 && time(nullptr) < 1700000000; ++i)
        if (!syncClockFromHttp()) delay(800);

    // Fetch the manifest, retrying transient TLS/network failures. A unique
    // cache-busting query + no-cache headers force a fresh version.json: without
    // them a CDN/proxy edge can keep serving a stale manifest, so a board reads
    // an old version and reports "up to date" while others see the new release.
    int code = 0;
    String body;
    for (int attempt = 0; attempt < 3 && code != HTTP_CODE_OK; ++attempt) {
        NetworkClientSecure client;
        client.setCACert(GITHUB_ROOTS);  // pinned GitHub roots (the IDF bundle wouldn't validate)
        HTTPClient http;
        http.setConnectTimeout(10000);
        http.setTimeout(10000);
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);  // GitHub 302s to its CDN
        String url = String(FW_MANIFEST_URL) + "?cb=" + String((uint32_t)millis()) + String(random(1000000));
        if (http.begin(client, url)) {
            http.addHeader("User-Agent", "headrush-controller");
            http.addHeader("Cache-Control", "no-cache");
            http.addHeader("Pragma", "no-cache");
            code = http.GET();
            if (code == HTTP_CODE_OK) body = http.getString();
            http.end();
        }
        if (code != HTTP_CODE_OK) delay(700);
    }
    if (code != HTTP_CODE_OK) {
        snprintf(otaStatusMsg, sizeof(otaStatusMsg), "check failed %d h%uk",
                 code, (unsigned)(ESP.getMaxAllocHeap() / 1024));
        delay(2500); otaChecking = false; return;
    }

    JsonDocument doc;
    if (deserializeJson(doc, body)) {
        snprintf(otaStatusMsg, sizeof(otaStatusMsg), "parse error");
        delay(2500); otaChecking = false; return;
    }
    int remote = doc["version"] | -1;
    String url = doc["url"] | "";
    if (remote <= FW_VERSION || url.isEmpty()) {
        snprintf(otaStatusMsg, sizeof(otaStatusMsg), "up to date (v%d)", remote);
        delay(1500); otaChecking = false; return;
    }
    httpUpdate.rebootOnUpdate(true);
    httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    httpUpdate.onProgress([](int cur, int total) {
        otaPercent = total ? (int)((cur * 100L) / total) : 0;
    });

    // Download + flash, retrying on failure. A partial/aborted download (weak
    // WiFi, AP contention) leaves the app intact — a fresh attempt usually
    // succeeds. Each try uses a new TLS client + cache-buster so it can't resume
    // a broken/stale transfer. A successful flash reboots into the new image.
    t_httpUpdate_return r = HTTP_UPDATE_FAILED;
    const int MAX_TRIES = 3;
    for (int attempt = 1; attempt <= MAX_TRIES; ++attempt) {
        snprintf(otaStatusMsg, sizeof(otaStatusMsg), "updating v%d", remote);
        otaPercent = 0;
        otaActive = true;       // UI switches to the progress ring
        otaChecking = false;
        String dlUrl = url + (url.indexOf('?') >= 0 ? "&cb=" : "?cb=")
                       + String((uint32_t)millis()) + String(random(1000000));
        NetworkClientSecure dlClient;
        dlClient.setCACert(GITHUB_ROOTS);
        r = httpUpdate.update(dlClient, dlUrl);
        // Only reached if the update did NOT succeed (success reboots).
        otaActive = false;
        if (attempt < MAX_TRIES) {
            snprintf(otaStatusMsg, sizeof(otaStatusMsg), "retry %d/%d (e%d)",
                     attempt + 1, MAX_TRIES, httpUpdate.getLastError());
            otaChecking = true; delay(2000); otaChecking = false;
        }
    }
    snprintf(otaStatusMsg, sizeof(otaStatusMsg), "upd fail %d/%d h%uk",
             (int)r, httpUpdate.getLastError(), (unsigned)(ESP.getMaxAllocHeap() / 1024));
    otaChecking = true; delay(3000); otaChecking = false;
}

// Set the system clock from an HTTPS Date header. NTP (UDP/123) is blocked on
// some networks while TCP/443 to GitHub works — and verified TLS needs a real
// clock to check cert validity dates. Reads only the header; trusts nothing.
bool syncClockFromHttp() {
    NetworkClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.setConnectTimeout(8000);
    http.setTimeout(8000);
    if (!http.begin(client, "https://github.com/")) return false;
    const char* keys[] = { "Date" };
    http.collectHeaders(keys, 1);
    http.sendRequest("HEAD");
    String date = http.header("Date");
    http.end();
    struct tm tm = {};
    if (date.length() < 20 || !strptime(date.c_str(), "%a, %d %b %Y %H:%M:%S", &tm)) return false;
    setenv("TZ", "UTC0", 1); tzset();
    time_t t = mktime(&tm);
    if (t < 1700000000) return false;
    struct timeval tv = { t, 0 };
    settimeofday(&tv, nullptr);
    return true;
}

// Scan for WiFi networks (net task only). Fills scanSSIDs/scanCount, deduped,
// then sets scanDone. Works whether or not we're currently connected.
void doWifiScan() {
    scanDone = false;
    WiFi.mode(WIFI_STA);
    int n = WiFi.scanNetworks();
    int k = 0;
    for (int i = 0; i < n && k < 16; ++i) {
        String s = WiFi.SSID(i);
        if (s.length() == 0) continue;
        bool dup = false;
        for (int j = 0; j < k; ++j) if (s.equalsIgnoreCase(scanSSIDs[j])) { dup = true; break; }
        if (dup) continue;
        strncpy(scanSSIDs[k], s.c_str(), 32);
        scanSSIDs[k][32] = 0;
        k++;
    }
    WiFi.scanDelete();
    scanCount = k;
    scanDone = true;
    Serial.printf("[wifi] scan: %d networks\n", k);
}

// Try to join selectedSSID with the entered password (net task). On success,
// persist the creds to NVS and reboot to re-init cleanly on the new network.
void doWifiConnect() {
    wifiConnectResult = 0;  // pending
    WiFi.disconnect();
    WiFi.begin(selectedSSID, wifiPassEntry);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(250);
    if (WiFi.status() == WL_CONNECTED) {
        Preferences p;
        p.begin("hrctrl", false);
        p.putString("ssid", selectedSSID);
        p.putString("psk", wifiPassEntry);
        p.end();
        Serial.printf("[wifi] connected to %s, saved\n", selectedSSID);
        wifiConnectResult = 1;
        delay(800);
        ESP.restart();
    } else {
        Serial.printf("[wifi] connect to %s failed\n", selectedSSID);
        wifiConnectResult = 2;
    }
}

void netTask(void*) {
    Serial.println("[net] starting");
    gHostname = makeHostname();
    Serial.printf("[net] hostname: %s.local\n", gHostname.c_str());
    WifiCreds c = loadCreds();
    snprintf(bootMsg, sizeof(bootMsg), "connecting");
    connStatus = CS_CONNECTING;
    bool online = connectWifi(c);

    if (online) {
        connStatus = CS_WIFI;
        // Set the clock for verified TLS (cert date checks). Try NTP briefly,
        // then fall back to the HTTPS Date header (NTP/UDP is blocked on some
        // networks while TCP/443 works). No real clock = certs look not-yet-valid.
        snprintf(bootMsg, sizeof(bootMsg), "setting clock");
        configTime(0, 0, "pool.ntp.org", "time.cloudflare.com");
        for (uint32_t t0 = millis(); time(nullptr) < 1700000000 && millis() - t0 < 4000; ) delay(200);
        if (time(nullptr) < 1700000000) syncClockFromHttp();
        Serial.printf("[net] time = %ld\n", (long)time(nullptr));

        String host = resolveHost(c);

        // OTA pull check FIRST — on the clean pre-WebSocket heap. Verified TLS
        // needs a large *contiguous* block; connecting the WS (hr.begin) fragments
        // the heap to ~30k and the handshake fails "-1", even with ~60k total free.
        // Run it here, where the heap is clean (the same reason the clock sync above
        // succeeds). Stagger so a rack of boards doesn't all hit TLS + the ~1.4MB
        // download at once. A successful update reboots into the new image.
        if (FW_VERSION > 0) {  // dev builds (fw 0) skip auto-update
            uint32_t jitterMs = (uint32_t)(deviceId - 1) * 3000 + (esp_random() % 2000);
            snprintf(bootMsg, sizeof(bootMsg), "update in %us", (unsigned)((jitterMs + 999) / 1000));
            vTaskDelay(pdMS_TO_TICKS(jitterMs));
            checkForUpdate("boot");
        }

        // Now link the Prime (WebSocket) and set up push-OTA.
        snprintf(bootMsg, sizeof(bootMsg), "linking Prime");
        hr.onValueChanged(onValueChanged);
        hr.onConnection([](bool up) {
            connStatus = up ? CS_HEADRUSH : CS_HR_LOST;
            Serial.printf("WS %s\n", up ? "connected" : "disconnected");
        });
        hr.begin(host);
        primeInitialValue();
        requestRigResolve();   // resolve the loaded rig's layout once the link is up
        setupOTA();
    } else {
        connStatus = CS_WIFI_ERR;
        snprintf(bootMsg, sizeof(bootMsg), "WiFi failed");
    }
    bootComplete = true;  // never delete the task — stay alive so the UI keeps working

    uint32_t lastWriteMs = 0;
    while (true) {
        if (scanRequested) { scanRequested = false; doWifiScan(); }        // works offline too
        if (wifiConnectRequested) { wifiConnectRequested = false; doWifiConnect(); }
        if (online) {
            ArduinoOTA.handle();  // blocks here for the duration of a push update
            if (otaCheckRequested) { otaCheckRequested = false; checkForUpdate("manual"); }
            if (rigResolvePending && (millis() - rigResolveAtMs) > RIG_RESOLVE_DEBOUNCE_MS) {
                rigResolvePending = false; doFetchRigList(); resolveLayoutForRig();   // resolve once the WS burst settles
            }
            // If the live block list never loaded, keep retrying a resolve so a
            // recovered link self-heals rather than sitting on a stale layout.
            if (!gBlocksOk) { static uint32_t lastBlkTry = 0; if (millis() - lastBlkTry > 1500) { lastBlkTry = millis(); requestRigResolve(); } }
            if (rebindRequested) { rebindRequested = false; if (!tunerActive && !homeListActive) primeInitialValue(); }  // new control focused
            if (libFetchSetlistsReq) { libFetchSetlistsReq = false; doLibFetchSetlists(); }
            if (fetchRigListReq)     { fetchRigListReq = false; doFetchRigList(); }
            if (loadSetlistReq)      { loadSetlistReq = false; doLoadSetlist(); }
            if (libLoadRigReq)       { libLoadRigReq = false; doLibLoadRig(); }
            if (devParamsReq)        { devParamsReq = false; doFetchDevParams(); }
            if (bypassToggleReq) {   // Phase 1 stub — Phase 2 toggles the block's enable on the Prime
                bypassToggleReq = false;
                Serial.printf("[bypass] toggle requested for %s (stub — not yet wired)\n", activeBinding->label);
            }
            hr.loop();
            serviceTuner();
            if (WiFi.status() != WL_CONNECTED) { online = false; connStatus = CS_WIFI_ERR; wifiRssi = 0; }
            else { static uint32_t lastRssiMs = 0; if (millis() - lastRssiMs > 1000) { lastRssiMs = millis(); wifiRssi = WiFi.RSSI(); } }
            bool dirty = false;
            float v = 0;
            portENTER_CRITICAL(&stateMux);
            if (pendingWriteValid && millis() - lastWriteMs >= WRITE_THROTTLE_MS) {
                v = pendingWriteValue;
                pendingWriteValid = false;
                dirty = true;
            }
            portEXIT_CRITICAL(&stateMux);
            if (dirty) {
                lastWriteMs = millis();
                float wire = HR::displayToWire(v, activeBinding->dispMin, activeBinding->dispMax);
                if (!hr.setProperty(activeBinding->path, activeBinding->prop, wire))
                    Serial.println("[net] write FAIL");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(online ? 2 : 50));
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.printf("\n=== HeadRush Controller — Stage 1B (fw %d) ===\n", FW_VERSION);

    memset(gCatCache, -1, sizeof(gCatCache));   // block categories asked lazily from the Prime

    // Library tables in PSRAM (frees internal SRAM for the OTA TLS handshake).
    // ps_malloc draws from external RAM; fall back to internal if unavailable.
    auto psAlloc = [](size_t sz) -> void* { void* p = ps_malloc(sz); return p ? p : malloc(sz); };
    libSetlistNames = (char(*)[40])psAlloc(LIB_MAX_SETLISTS * 40);
    libSetlistIds   = (char(*)[40])psAlloc(LIB_MAX_SETLISTS * 40);
    libRigNames     = (char(*)[40])psAlloc(LIB_MAX_RIGS * 40);
    libRigIds       = (char(*)[40])psAlloc(LIB_MAX_RIGS * 40);
    libRigSrIds     = (char(*)[40])psAlloc(LIB_MAX_RIGS * 40);
    gModNameRT      = (char(*)[32])psAlloc(MAX_MODTYPES * 32);   // live ModuleTypes cache

    // Ensure NVS is healthy. If the partition is in a bad/old state, the core
    // may not have reformatted it, making Preferences writes succeed in-session
    // but never persist. Reformat once on that error, then re-init.
    // Defensive: reformat NVS if it's ever in a bad/old state so settings persist.
    esp_err_t nvsErr = nvs_flash_init();
    if (nvsErr == ESP_ERR_NVS_NO_FREE_PAGES || nvsErr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    prefs.begin("hrctrl", true);
    deviceId = prefs.getInt("devid", 0);   // 0 = never provisioned
    prefs.end();
#ifdef PROVISION_ID
    // Fleet provisioning: bake this unit's ID at flash time, but only if it
    // hasn't been set yet — config-mode changes still win afterward.
    if (deviceId == 0) { deviceId = PROVISION_ID; saveDeviceId(deviceId); }
#endif
    if (deviceId < 1 || deviceId > 16) deviceId = 1;
    // Boot placeholder: the per-rig group is net-owned and gets published once the
    // Prime link is up (resolveLayoutForRig). Show Output until then.
    memset(gHomeDesc, 0, sizeof(gHomeDesc));
    homeGroup[0] = 0; homeGroupLen = 1; homeFocus = 0;
    focusHomeMember(0);
    Serial.printf("Device ID: %d (awaiting rig resolve)\n", deviceId);

    pinMode(HW::PIN_PWR_EN_1, OUTPUT); digitalWrite(HW::PIN_PWR_EN_1, HIGH);
    pinMode(HW::PIN_PWR_EN_2, OUTPUT); digitalWrite(HW::PIN_PWR_EN_2, HIGH);

    gfx.init();
    canvas.setColorDepth(16);
    canvas.setPsram(true);
    if (!canvas.createSprite(240, 240)) Serial.println("canvas alloc FAILED");
    ledcAttach(HW::PIN_DISP_BL, DisplayHW::BL_LEDC_FREQ, DisplayHW::BL_LEDC_RESOLUTION_BITS);
    ledcWrite(HW::PIN_DISP_BL, BL_FULL_DUTY);
    lastActivityMs = millis();
    Render::drawSplash(canvas, deviceId, FW_VERSION, bootMsg);

    encoder.begin();
    encoder.setup(encoderISR);
    encoder.setBoundaries(-100000, 100000, false);
    encoder.setAcceleration(0);
    encoder.setEncoderValue(0);
    lastEncoderValue = 0;

    xTaskCreatePinnedToCore(netTask, "net", 8192, NULL, 5, NULL, 0);
    // The splash stays up (showing live boot status) until the net task signals
    // bootComplete; the UI loop drives it, so no fixed delay is needed here.
}

void loop() {
    // Keep the screen lit through boot, update checks, and downloads; otherwise
    // let it dim after a spell of no activity.
    if (otaActive || otaChecking || !bootComplete) noteActivity();
    updateBacklight();

    // A rig change: the net task has published a freshly resolved group. Just
    // refocus to bind + refresh the dial (the group itself is net-owned now). Only
    // act while on HOME — a menu the user is in shouldn't yank out from under them;
    // the new layout takes effect when they return to HOME.
    if (layoutDirty && screen == SC_HOME) {
        layoutDirty = false;
        focusHomeMember(0);
        lastRenderedValue = -999999.0f;   // force a redraw with the new value
    }

    // Net-driven overlays take priority over everything, including config mode:
    // a download/flash may be running on the other core. Redraw only on a mode
    // or percent change so the screen doesn't flicker.
    if (otaActive) {  // downloading + flashing
        if (uiMode != UI_UPDATING) { uiMode = UI_UPDATING; lastRenderedOtaPercent = -1; }
        int p = otaPercent;
        if (p != lastRenderedOtaPercent) { Render::drawOTAProgress(canvas, p); lastRenderedOtaPercent = p; }
        vTaskDelay(pdMS_TO_TICKS(20));
        return;
    }
    // Splash phase: the boot sequence (connecting / setting clock / linking
    // Prime) and the update check both show status on the splash, so the dial
    // only appears once the unit is fully ready.
    if (!bootComplete || otaChecking) {
        const char* msg = otaChecking ? otaStatusMsg : bootMsg;
        static char shown[40] = "";
        if (uiMode != UI_SPLASH || strcmp(shown, msg) != 0) {
            uiMode = UI_SPLASH;
            strncpy(shown, msg, sizeof(shown)); shown[sizeof(shown) - 1] = 0;
            Render::drawSplash(canvas, deviceId, FW_VERSION, msg);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        return;
    }
    if (uiMode != UI_GAUGE) { uiMode = UI_GAUGE; lastRenderedValue = -999999.0f; }

    // --- Read input: a short tap (immediate, for menus), a 1s long-press
    // (back), and the turn delta. ---
    bool clicked = false, longPress = false;
    {
        static uint32_t down = 0;
        static bool fired = false;
        if (encoder.isEncoderButtonDown()) {
            if (down == 0) { down = millis(); fired = false; }
            else if (!fired && millis() - down >= LONG_PRESS_MS) { longPress = true; fired = true; }
        } else {
            if (down != 0 && !fired) {
                uint32_t held = millis() - down;
                if (held >= 40 && held < LONG_PRESS_MS) clicked = true;  // debounced short press
            }
            down = 0;
        }
    }
    // Resolve taps into single vs double click for the Home screen. A lone tap
    // becomes a singleClick only after DOUBLE_CLICK_MS passes with no second tap;
    // a second tap inside the window is a doubleClick. Menus use `clicked`
    // directly, so they have no added latency.
    bool singleClick = false, doubleClick = false;
    {
        static uint32_t pendingMs = 0;
        static bool pending = false;
        if (screen != SC_HOME) {
            pending = false;   // only Home uses single/double; never carry a tap across screens
        } else if (clicked) {
            if (pending && (millis() - pendingMs) <= DOUBLE_CLICK_MS) { doubleClick = true; pending = false; }
            else { pending = true; pendingMs = millis(); }
        } else if (pending && (millis() - pendingMs) > DOUBLE_CLICK_MS) {
            singleClick = true; pending = false;
        }
    }
    long edelta = 0;
    if (encoder.encoderChanged()) {
        long v = encoder.readEncoder();
        edelta = (v - lastEncoderValue) * ENCODER_SIGN;
        lastEncoderValue = v;
    }
    // Velocity-based acceleration: the faster the detents arrive, the larger each
    // step. Values get a stronger curve than list scrolling (so you don't blow
    // past a rig). Creep stays 1:1 for precision. Tunable by feel.
    int valAccel = 1, listAccel = 1;
    if (edelta != 0) {
        static uint32_t lastTurnMs = 0;
        uint32_t dt = millis() - lastTurnMs;
        lastTurnMs = millis();
        if      (dt < 25) { valAccel = 12; listAccel = 6; }   // fast spin
        else if (dt < 60) { valAccel = 4;  listAccel = 3; }   // moderate
    }
    if (edelta != 0 || clicked || longPress) noteActivity();
    // No inactivity auto-return: a board stays on whatever screen it was left on.

    // --- Board Menu (hold from Home) ---
    if (screen == SC_BOARD_MENU) {
        if (longPress) {   // back to Home
            screen = SC_HOME; focusHomeMember(homeFocus);
            lastRenderedValue = -999999.0f; lastTunerCents = -999.0f;
            vTaskDelay(pdMS_TO_TICKS(5)); return;
        }
        if (clicked) {
            if (boardSel == 0) {            // Assign this knob — seed from current group
                seedAssignFromHome();
                buildAssignRows();
                screen = SC_ASSIGN;
                renderAssignRoot();
            } else {                        // Settings
                menuIndex = 0;
                screen = SC_SETTINGS;
                Render::drawConfigMenu(canvas, menuIndex, deviceId, FW_VERSION);
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        if (edelta != 0) {
            boardSel = (int)(((boardSel + edelta) % 2 + 2) % 2);
            Render::drawBoardMenu(canvas, boardSel);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }

    // --- Assign this knob: rig-derived root list (devices + globals + special) ---
    if (screen == SC_ASSIGN) {
        if (asgRowCount <= 0) {   // sparse / not yet resolved — nothing to pick
            if (longPress) { screen = SC_HOME; focusHomeMember(homeFocus); lastRenderedValue = -999999.0f; vTaskDelay(pdMS_TO_TICKS(5)); return; }
            vTaskDelay(pdMS_TO_TICKS(10)); return;
        }
        if (longPress) {                      // commit the group, back to Home
            commitAssign();
            screen = SC_HOME;
            lastRenderedValue = -999999.0f; lastTunerCents = -999.0f;
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        if (clicked) {
            const AsgRow& row = asgRows[asgCursor];
            if (row.kind == ARK_DEV_PRIMARY) {
                PresentDev& pd = gPresent[row.arg];
                asgToggleDevice(pd.cat, pd.path, pd.prop);
            } else if (row.kind == ARK_DEV_PARAMS) {
                enterAssignParams(row.arg);   // drill in
                vTaskDelay(pdMS_TO_TICKS(5));
                return;
            } else {                          // GLOBAL / SPECIAL
                asgToggleView(row.arg);
            }
            renderAssignRoot();
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        if (edelta != 0) {
            asgCursor = (int)(((asgCursor + edelta) % asgRowCount + asgRowCount) % asgRowCount);
            renderAssignRoot();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }

    // --- Assign drill-in: pick params of one device (multi-select) ---
    if (screen == SC_ASSIGN_PARAMS) {
        if (!devParamsDone) {                 // waiting on the net fetch
            vTaskDelay(pdMS_TO_TICKS(20));
            return;
        }
        static bool paramsShown = false;
        if (longPress) {                      // back to the root list
            paramsShown = false;
            screen = SC_ASSIGN;
            renderAssignRoot();
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        if (gDevParamCount <= 0) {            // no ridable params — show briefly, wait for back
            if (!paramsShown) { Render::drawListLoading(canvas, "no params"); paramsShown = true; }
            vTaskDelay(pdMS_TO_TICKS(20));
            return;
        }
        if (!paramsShown) { renderAssignParams(); paramsShown = true; }
        if (clicked) {
            PresentDev& pd = gPresent[asgParamsDev];
            asgToggleDevice(pd.cat, pd.path, gDevParam[asgParamCursor].prop);
            renderAssignParams();
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        if (edelta != 0) {
            int n = gDevParamCount;
            asgParamCursor = (int)(((asgParamCursor + edelta) % n + n) % n);
            renderAssignParams();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }

    // --- Settings menu ---
    if (screen == SC_SETTINGS) {
        if (longPress) { screen = SC_BOARD_MENU; Render::drawBoardMenu(canvas, boardSel); vTaskDelay(pdMS_TO_TICKS(5)); return; }
        if (clicked) {
            if (menuIndex == 0) { editIdValue = deviceId; screen = SC_SET_ID; Render::drawConfigIdEdit(canvas, editIdValue); }
            else if (menuIndex == 1) { screen = SC_WIFI; wifiPickIndex = 1; scanDone = false; scanRequested = true; }  // scan starts
            else if (menuIndex == 2) { otaCheckRequested = true; screen = SC_HOME; focusHomeMember(homeFocus); lastRenderedValue = -999999.0f; lastTunerCents = -999.0f; }
            else { screen = SC_BOARD_MENU; Render::drawBoardMenu(canvas, boardSel); }   // Exit -> Board Menu
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        if (edelta != 0) {
            menuIndex = (int)(((menuIndex + edelta) % 4 + 4) % 4);
            Render::drawConfigMenu(canvas, menuIndex, deviceId, FW_VERSION);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }

    // --- WiFi network picker (Stage 1b: scan + pick; password/connect = Stage 2) ---
    if (screen == SC_WIFI) {
        static bool shownScanning = false;
        static int  shownPick = -2;
        if (!scanDone) {  // scan in progress
            if (!shownScanning) { shownScanning = true; shownPick = -2; Render::drawSplash(canvas, deviceId, FW_VERSION, "scanning WiFi"); }
            vTaskDelay(pdMS_TO_TICKS(30));
            return;
        }
        shownScanning = false;
        if (longPress) { screen = SC_SETTINGS; shownPick = -2; Render::drawConfigMenu(canvas, menuIndex, deviceId, FW_VERSION); vTaskDelay(pdMS_TO_TICKS(5)); return; }
        if (scanCount <= 0) {  // no networks — click to rescan
            if (clicked) { scanDone = false; scanRequested = true; shownPick = -2; vTaskDelay(pdMS_TO_TICKS(5)); return; }
            if (shownPick != -1) { shownPick = -1; Render::drawWifiPicker(canvas, scanSSIDs, 0, 0); }
            vTaskDelay(pdMS_TO_TICKS(20));
            return;
        }
        if (clicked) {
            if (wifiPickIndex == 0) {  // "< Back" -> config menu
                screen = SC_SETTINGS; shownPick = -2;
                Render::drawConfigMenu(canvas, menuIndex, deviceId, FW_VERSION);
                vTaskDelay(pdMS_TO_TICKS(5));
                return;
            }
            strncpy(selectedSSID, scanSSIDs[wifiPickIndex - 1], sizeof(selectedSSID)); selectedSSID[32] = 0;
            Serial.printf("[wifi] picked %s\n", selectedSSID);
            wifiPassword[0] = 0; pwIndex = 3; shownPick = -2;
            screen = SC_WIFI_PW;
            char ib[2] = { PW_CHARS[0], 0 };
            Render::drawPasswordEntry(canvas, selectedSSID, wifiPassword, ib);
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        if (edelta != 0) { int total = scanCount + 1; wifiPickIndex = (int)(((wifiPickIndex + edelta) % total + total) % total); }
        if (shownPick != wifiPickIndex) { shownPick = wifiPickIndex; Render::drawWifiPicker(canvas, scanSSIDs, scanCount, wifiPickIndex); }
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }

    // --- WiFi password entry (encoder picker: 0=OK, 1=DEL, 2+=chars) ---
    if (screen == SC_WIFI_PW) {
        const int items = (int)strlen(PW_CHARS) + 3;
        char itemBuf[4];
        auto labelFor = [&](int idx) -> const char* {
            if (idx == 0) return "OK";
            if (idx == 1) return "DEL";
            if (idx == 2) return "Back";
            itemBuf[0] = PW_CHARS[idx - 3]; itemBuf[1] = 0; return itemBuf;
        };
        if (longPress) {  // cancel back to menu
            screen = SC_SETTINGS;
            Render::drawConfigMenu(canvas, menuIndex, deviceId, FW_VERSION);
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        if (clicked) {
            if (pwIndex == 0) {  // OK -> attempt connect (net task)
                strncpy(wifiPassEntry, wifiPassword, sizeof(wifiPassEntry)); wifiPassEntry[63] = 0;
                wifiConnectResult = 0; wifiConnectRequested = true;
                screen = SC_WIFI_CONNECTING;
                Render::drawSplash(canvas, deviceId, FW_VERSION, "connecting");
            } else if (pwIndex == 1) {  // DEL
                int L = strlen(wifiPassword); if (L) wifiPassword[L - 1] = 0;
                Render::drawPasswordEntry(canvas, selectedSSID, wifiPassword, labelFor(pwIndex));
            } else if (pwIndex == 2) {  // Back -> config menu
                screen = SC_SETTINGS;
                Render::drawConfigMenu(canvas, menuIndex, deviceId, FW_VERSION);
            } else {  // append char
                int L = strlen(wifiPassword); if (L < 63) { wifiPassword[L] = PW_CHARS[pwIndex - 3]; wifiPassword[L + 1] = 0; }
                Render::drawPasswordEntry(canvas, selectedSSID, wifiPassword, labelFor(pwIndex));
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        if (edelta != 0) {
            pwIndex = (int)(((pwIndex + edelta) % items + items) % items);
            Render::drawPasswordEntry(canvas, selectedSSID, wifiPassword, labelFor(pwIndex));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }

    // --- WiFi connecting (net task attempts the join) ---
    if (screen == SC_WIFI_CONNECTING) {
        if (wifiConnectResult == 1) {  // success — net task reboots momentarily
            Render::drawSplash(canvas, deviceId, FW_VERSION, "connected!");
            vTaskDelay(pdMS_TO_TICKS(50));
            return;
        }
        if (wifiConnectResult == 2) {  // failed — back to password entry to retry
            Render::drawSplash(canvas, deviceId, FW_VERSION, "connect failed");
            delay(2500);
            wifiConnectResult = 0; pwIndex = 3;
            screen = SC_WIFI_PW;
            char ib[2] = { PW_CHARS[0], 0 };
            Render::drawPasswordEntry(canvas, selectedSSID, wifiPassword, ib);
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(50));  // pending ("connecting" already shown)
        return;
    }

    // --- Device-ID editor ---
    if (screen == SC_SET_ID) {
        if (longPress) { screen = SC_SETTINGS; Render::drawConfigMenu(canvas, menuIndex, deviceId, FW_VERSION); vTaskDelay(pdMS_TO_TICKS(5)); return; }
        if (clicked) {
            if (editIdValue != deviceId) {
                deviceId = editIdValue;     // identity only; applies immediately
                saveDeviceId(deviceId);
            }
            screen = SC_SETTINGS;
            Render::drawConfigMenu(canvas, menuIndex, deviceId, FW_VERSION);
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        if (edelta != 0) {
            editIdValue += (int)edelta;
            if (editIdValue < 1) editIdValue = 1;
            if (editIdValue > 16) editIdValue = 16;
            Render::drawConfigIdEdit(canvas, editIdValue);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }

    // ============================ HOME ============================
    // While the net task is resolving this rig's buttons, show a spinner and gate
    // home input — the group/bindings are mid-write. (If the block list can't load
    // this spins until the link recovers — a visible signal, not a stale layout.)
    // Long-press still escapes to the Board Menu so the user isn't stuck.
    if (gResolving) {
        if (longPress) { boardSel = 0; screen = SC_BOARD_MENU; Render::drawBoardMenu(canvas, boardSel); vTaskDelay(pdMS_TO_TICKS(10)); return; }
        static int spin = 0;
        Render::drawSpinner(canvas, "resolving", spin++);
        lastRenderedValue = -999999.0f;
        vTaskDelay(pdMS_TO_TICKS(60));
        return;
    }
    // hold opens the Board Menu.
    if (longPress) {
        boardSel = 0; screen = SC_BOARD_MENU;
        Render::drawBoardMenu(canvas, boardSel);
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }
    // Double-click steps to the next group member (no-op for a group of one).
    // (§3: single-click is now "load" on a list / nothing on a value; the member
    // cycle moved here to free it.)
    if (doubleClick && homeGroupLen > 1) {
        focusHomeMember(homeFocus + 1);
        lastRenderedValue = -999999.0f; lastTunerCents = -999.0f;
        vTaskDelay(pdMS_TO_TICKS(5));
        return;
    }

    // --- Setlist / Rig browse-then-load Home (§3/§4) --- turn browses names with
    // nothing loading; single-click loads the highlighted item; 10 s idle reverts
    // the highlight to the loaded item. No more pause-to-load cascade.
    if (homeListActive) {
        bool isSet = viewIsSetlist(homeGroup[homeFocus]);
        if (libReqDone) {                 // a fetch/load finished: sync cursor to the loaded item
            libReqDone = false;
            listLoading = false;
            int cnt = isSet ? libSetlistCount : libRigCount;
            int cur = 0;
            if (isSet) {
                char nm[40]; portENTER_CRITICAL(&stateMux); strncpy(nm, libCurrentSetlistName, 40); portEXIT_CRITICAL(&stateMux); nm[39] = 0;
                for (int i = 0; i < cnt; ++i) if (strcmp(libSetlistNames[i], nm) == 0) { cur = i; break; }
            } else {
                char id[40]; portENTER_CRITICAL(&stateMux); strncpy(id, libCurrentRigId, 40); portEXIT_CRITICAL(&stateMux); id[39] = 0;
                for (int i = 0; i < cnt; ++i) if (strcmp(libRigIds[i], id) == 0) { cur = i; break; }
            }
            listCursor = cur; listCommitted = cur; listDirty = true;
        }
        if (listLoading) {
            if (listDirty) { Render::drawListLoading(canvas, "loading..."); listDirty = false; }
            vTaskDelay(pdMS_TO_TICKS(20));
            return;
        }
        int count = isSet ? libSetlistCount : libRigCount;
        if (count <= 0) {
            if (listDirty) { Render::drawListLoading(canvas, isSet ? "no setlists" : "no rigs"); listDirty = false; }
            vTaskDelay(pdMS_TO_TICKS(50));
            return;
        }
        if (listCursor >= count) listCursor = count - 1;
        if (edelta != 0) {
            listCursor = (((listCursor + edelta * listAccel) % count) + count) % count;
            lastListScrollMs = millis();
            listDirty = true;
        }
        // 10 s idle while browsed away from the loaded item -> revert (no load).
        if (listCursor != listCommitted && (millis() - lastListScrollMs) > LIST_REVERT_MS) {
            listCursor = listCommitted; listDirty = true;
        }
        // Single-click loads the highlighted item (only if it's a different one).
        if (singleClick && listCursor != listCommitted) {
            listCommitted = listCursor;
            if (isSet) {
                strncpy(libSelSetlistId, libSetlistIds[listCursor], sizeof(libSelSetlistId)); libSelSetlistId[39] = 0;
                portENTER_CRITICAL(&stateMux);
                strncpy(libCurrentSetlistName, libSetlistNames[listCursor], 39); libCurrentSetlistName[39] = 0;
                portEXIT_CRITICAL(&stateMux);
                loadSetlistReq = true;
            } else {
                strncpy(libSelRigId, libRigIds[listCursor], sizeof(libSelRigId)); libSelRigId[39] = 0;
                strncpy(libSelSetlistId, libRigSrIds[listCursor], sizeof(libSelSetlistId)); libSelSetlistId[39] = 0;
                portENTER_CRITICAL(&stateMux);
                strncpy(libCurrentRigName, libRigNames[listCursor], 39); libCurrentRigName[39] = 0;
                strncpy(libCurrentRigId, libRigIds[listCursor], 39);     libCurrentRigId[39] = 0;
                portEXIT_CRITICAL(&stateMux);
                libLoadRigReq = true;
            }
            listLoading = true; listDirty = true;
            noteActivity();
        }
        uint16_t sc = statusColor(connStatus);
        int sl = signalLevel(wifiRssi);
        if (listDirty || sc != lastStatusColor || sl != lastSignalLevel) {
            Render::drawScrollList(canvas, isSet ? "SETLIST" : "RIG",
                                   isSet ? libSetlistNames : libRigNames, count, listCursor,
                                   sc, sl, homeGroupLen, homeFocus);
            lastStatusColor = sc; lastSignalLevel = sl; listDirty = false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }

    // --- Tuner Home --- (no encoder writes; the net task drives the readout)
    if (tunerActive) {
        char note[8];
        float cents;
        portENTER_CRITICAL(&stateMux);
        strncpy(note, tunerNote, sizeof(note));
        cents = tunerCents;
        portEXIT_CRITICAL(&stateMux);
        note[sizeof(note) - 1] = 0;
        uint16_t sc = statusColor(connStatus);
        int sl = signalLevel(wifiRssi);
        bool tunerChanged = fabsf(cents - lastTunerCents) >= 1.0f || strcmp(note, lastTunerNote) != 0;
        if (tunerChanged) noteActivity();  // an active note keeps the screen lit while tuning
        if (tunerChanged || sc != lastStatusColor || sl != lastSignalLevel) {
            Render::drawTuner(canvas, note, cents, sc, sl);
            lastTunerCents = cents;
            strncpy(lastTunerNote, note, sizeof(lastTunerNote));
            lastTunerNote[sizeof(lastTunerNote) - 1] = 0;
            lastStatusColor = sc;
            lastSignalLevel = sl;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }

    if (edelta != 0) {
        portENTER_CRITICAL(&stateMux);
        float nv = sharedValue + edelta * valAccel * activeBinding->step;
        if (nv < activeBinding->dispMin) nv = activeBinding->dispMin;
        if (nv > activeBinding->dispMax) nv = activeBinding->dispMax;
        sharedValue = nv;
        pendingWriteValue = nv;
        pendingWriteValid = true;
        lastLocalChangeMs = millis();
        portEXIT_CRITICAL(&stateMux);
    }

    float v;
    portENTER_CRITICAL(&stateMux);
    v = sharedValue;
    portEXIT_CRITICAL(&stateMux);
    uint16_t sc = statusColor(connStatus);
    int sl = signalLevel(wifiRssi);
    bool valueChanged = fabsf(v - lastRenderedValue) >= activeBinding->step * 0.5f;
    if (valueChanged) noteActivity();  // a moving value (local or external) keeps the screen awake
    if (valueChanged || sc != lastStatusColor || sl != lastSignalLevel) {
        Render::drawContinuous(canvas, *activeBinding, v, sc, sl, homeGroupLen, homeFocus);
        lastRenderedValue = v;
        lastStatusColor = sc;
        lastSignalLevel = sl;
    }

    vTaskDelay(pdMS_TO_TICKS(2));
}
