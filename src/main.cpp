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

// Catalog of assignable parameters. Each unit picks one entry in config mode
// (stored in NVS), so identical firmware can drive any of these. Output/Input/
// Tempo/Width/EQ are rig-independent; Comp/Reverb write to effect blocks that
// must be present in the loaded rig. Ranges/formats are from the device's own
// API schema (headrush-api-tree.json).
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
    { .label = "COMP", .path = "/Evil/Engine/Patch/Gray_Comp", .prop = "Sustain",
      .dispMin = 0.0f, .dispMax = 100.0f, .step = 1.0f, .format = "%.0f", .unit = "%",
      .zones = { {33.0f, 0x6B7F}, {66.0f, 0x07E0}, {100.0f, 0xFFE0} }, .zoneCount = 3 },
    { .label = "REVERB", .path = "/Evil/Engine/Patch/AIR_Reverb", .prop = "Mix",
      .dispMin = 0.0f, .dispMax = 100.0f, .step = 1.0f, .format = "%.0f", .unit = "%",
      .zones = { {33.0f, 0x6B7F}, {66.0f, 0x07E0}, {100.0f, 0xFFE0} }, .zoneCount = 3 },
};
const int PARAM_COUNT = sizeof(PARAM_CATALOG) / sizeof(PARAM_CATALOG[0]);
// Selected from the board's view ring in setup(). Read-only after, except a
// short click (cycle) or config swaps the pointer.
const ContinuousBinding* activeBinding = &PARAM_CATALOG[0];

// View indices: 0..PARAM_COUNT-1 are the PARAM_CATALOG dials; then Tuner,
// Setlist, and Rig. A board's Home carries an ordered group of these. Setlist
// and Rig are scroll-to-select lists (turn scrolls names, a pause loads).
const int TUNER_VIEW   = PARAM_COUNT;
const int SETLIST_VIEW = PARAM_COUNT + 1;
const int RIG_VIEW     = PARAM_COUNT + 2;
// Dynamic, per-rig category dials (Gain bucket / Ambience bucket). These resolve
// to whatever block fills the slot in the loaded rig (see resolveLayoutForRig).
// gDynBind[0] backs DYN_GAIN_VIEW, gDynBind[1] backs DYN_SPACE_VIEW.
const int DYN_GAIN_VIEW  = PARAM_COUNT + 3;
const int DYN_SPACE_VIEW = PARAM_COUNT + 4;
const int VIEW_COUNT   = PARAM_COUNT + 5;
inline bool viewIsTuner(int v)   { return v == TUNER_VIEW; }
inline bool viewIsSetlist(int v) { return v == SETLIST_VIEW; }
inline bool viewIsRig(int v)     { return v == RIG_VIEW; }
inline bool viewIsList(int v)    { return v == SETLIST_VIEW || v == RIG_VIEW; }
inline bool viewIsDyn(int v)     { return v == DYN_GAIN_VIEW || v == DYN_SPACE_VIEW; }
inline bool viewIsValid(int v)   { return (v >= 0 && v < PARAM_COUNT) || v == TUNER_VIEW
                                          || v == SETLIST_VIEW || v == RIG_VIEW || viewIsDyn(v); }

// Resolved dynamic dials for the loaded rig. [0] = Gain bucket, [1] = Ambience.
// The net task fills these from the Chain on every rig change; the UI binds to
// them via DYN_*_VIEW. ContinuousBinding holds const char* fields, so the runtime
// strings live in these backing buffers and the struct points into them.
ContinuousBinding gDynBind[2];
volatile bool     gDynOk[2] = { false, false };   // did the bucket resolve to a block?
char gDynLabel[2][12];                             // big label = generic category ("COMP")
char gDynDevice[2][24];                            // small subtitle = actual block ("Gray Comp")
char gDynPath[2][48];
char gDynProp[2][20];
char gDynFmt[2][12];                               // printf format, parsed from device meta
char gDynUnit[2][8];                               // unit suffix, parsed from device meta
volatile uint16_t gPresentCatMask = 0;            // bit c set => BlockCat c present in rig
volatile bool     layoutDirty = false;            // net -> UI: rebuild Home for the new rig
int8_t gCatCache[MODULE_TYPE_COUNT];              // module idx -> BlockCat, -1 = not yet asked (net task)

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
char libSetlistNames[LIB_MAX_SETLISTS][40];
char libSetlistIds[LIB_MAX_SETLISTS][40];
int  libSetlistCount = 0;
char libRigNames[LIB_MAX_RIGS][40];
char libRigIds[LIB_MAX_RIGS][40];
char libRigSrIds[LIB_MAX_RIGS][40];          // setlist-id context per rig (loadRig needs it)
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
// names; after a pause the highlighted item is loaded. UI-task-local.
constexpr uint32_t LIST_COMMIT_MS = 600;    // pause before a scrolled item loads
int  listCursor = 0;
int  listCommitted = -1;                    // last loaded index (avoids reloading)
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
volatile bool  rigChangedReq = false;   // WS push said the rig changed; net re-resolves the layout

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
// cycles the focused member; turn adjusts it. Members are view indices — a param
// dial (0..PARAM_COUNT-1) or TUNER_VIEW. Persisted per board in NVS.
int  homeGroup[VIEW_COUNT] = { 0 };
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
    } else if (viewIsDyn(v)) {
        int s = v - DYN_GAIN_VIEW;             // 0 = Gain, 1 = Ambience
        if (gDynOk[s]) {
            activeBinding = &gDynBind[s];
        } else {
            // Sparse rig: the bucket had no block. Fall back to a global so the
            // dial still does something useful (Tempo for gain, Width for space).
            activeBinding = &PARAM_CATALOG[s == 0 ? 2 : 3];
        }
    } else {
        if (v < 0 || v >= PARAM_COUNT) v = 0;
        paramIndex = v;
        activeBinding = &PARAM_CATALOG[v];
    }
    rebindRequested = true;
}

// Focus the i-th member of the Home group (wraps) and bind it.
void focusHomeMember(int i) {
    if (homeGroupLen < 1) homeGroupLen = 1;
    homeFocus = ((i % homeGroupLen) + homeGroupLen) % homeGroupLen;
    bindControl(homeGroup[homeFocus]);
}

// UI screen model. Every screen obeys the universal grammar: turn = move,
// single-click = enter/next, double-click = bypass (Home only), hold = back.
// Parent links are fixed (handled inline at each screen); the tree is shallow.
enum Screen {
    SC_HOME,            // the assigned knob: value group, tuner, or scroll-list
    SC_BOARD_MENU,      // hold from Home
    SC_ASSIGN,          // multi-select the Home group
    SC_SETTINGS,        // device/global config menu
    SC_SET_ID,
    SC_WIFI, SC_WIFI_PW, SC_WIFI_CONNECTING
};
Screen screen = SC_HOME;
int boardSel = 0;       // cursor in the Board Menu
int menuIndex = 0;      // cursor in the Settings menu
int editIdValue = 1;

// Assign (multi-select) scratch: an ordered selection of view indices being
// built into the Home group. assignCursor spans 0..PARAM_COUNT (==Tuner).
int assignCursor = 0;
int assignSel[VIEW_COUNT];
int assignSelLen = 0;

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

void saveHomeGroup() {
    Preferences p;
    p.begin("hrctrl", false);
    p.putInt("hglen", homeGroupLen);
    p.putBytes("hgrp", homeGroup, homeGroupLen * sizeof(int));
    p.end();
    Serial.printf("[nvs] home group len=%d focus=%d\n", homeGroupLen, homeFocus);
}

// Per-rig HOME assignment (UX_MODEL §5). Keyed by the rig's stable loadedID so a
// board adopts that rig's saved layout on load and falls back to generated
// defaults otherwise. NVS keys are short (<=15 chars), so we hash the UUID into
// "h<8 hex>l" (len) / "h<8 hex>g" (group bytes).
void rigNvsKeys(const char* rigId, char* lenKey, char* grpKey) {
    uint32_t h = 2166136261u;                         // FNV-1a over the rig id
    for (const char* s = rigId; *s; ++s) { h ^= (uint8_t)*s; h *= 16777619u; }
    snprintf(lenKey, 12, "h%08lxl", (unsigned long)h);
    snprintf(grpKey, 12, "h%08lxg", (unsigned long)h);
}

bool loadHomeForRig(const char* rigId) {
    if (!rigId || !rigId[0]) return false;
    char lk[12], gk[12]; rigNvsKeys(rigId, lk, gk);
    Preferences p; p.begin("hrctrl", true);
    int len = p.getInt(lk, 0);
    int tmp[VIEW_COUNT];
    bool ok = false;
    if (len >= 1 && len <= VIEW_COUNT && p.getBytes(gk, tmp, len * sizeof(int)) == len * sizeof(int)) {
        int n = 0;
        for (int i = 0; i < len; ++i) if (viewIsValid(tmp[i])) homeGroup[n++] = tmp[i];
        if (n > 0) { homeGroupLen = n; ok = true; }
    }
    p.end();
    if (ok) Serial.printf("[nvs] loaded rig layout %s len=%d\n", rigId, homeGroupLen);
    return ok;
}

void saveHomeForRig(const char* rigId) {
    if (!rigId || !rigId[0]) return;
    char lk[12], gk[12]; rigNvsKeys(rigId, lk, gk);
    Preferences p; p.begin("hrctrl", false);
    p.putInt(lk, homeGroupLen);
    p.putBytes(gk, homeGroup, homeGroupLen * sizeof(int));
    p.end();
    Serial.printf("[nvs] saved rig layout %s len=%d\n", rigId, homeGroupLen);
}

// Generate this board's default HOME for the loaded rig (UX_MODEL §5). The role
// is by device id (1..4 -> nav / gain / ambience / levels); >4 wraps. The Gain
// and Ambience dials are DYN views whose concrete block was resolved by the net
// task; here we only decide which dial this board carries.
void generateDefaultHome() {
    int role = (deviceId - 1) % 4;
    switch (role) {
        case 0:  homeGroup[0] = RIG_VIEW; homeGroup[1] = SETLIST_VIEW; homeGroupLen = 2; break;
        case 1:  homeGroup[0] = DYN_GAIN_VIEW;  homeGroupLen = 1; break;
        case 2:  homeGroup[0] = DYN_SPACE_VIEW; homeGroupLen = 1; break;
        default: homeGroup[0] = 0; homeGroup[1] = 1; homeGroupLen = 2; break;  // OUTPUT, INPUT
    }
    Serial.printf("[layout] default home for board %d role %d len %d\n", deviceId, role, homeGroupLen);
}

// Rebuild HOME when a new rig loads: adopt the saved per-rig layout or fall back
// to the generated default, then refocus (which re-primes the displayed value).
// Called from the UI task when the net task raises layoutDirty.
void rebuildHomeForRig() {
    char rigId[40];
    portENTER_CRITICAL(&stateMux);
    strncpy(rigId, libCurrentRigId, sizeof(rigId)); rigId[sizeof(rigId) - 1] = 0;
    portEXIT_CRITICAL(&stateMux);
    if (!loadHomeForRig(rigId)) generateDefaultHome();
    homeFocus = 0;
    focusHomeMember(0);
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
        rigChangedReq = true;   // rig swapped (footswitch / other controller) — re-resolve layout
    } else if (path == LIB_SETLISTS_PATH && prop == "loadedName" && value.is<const char*>()) {
        const char* nm = value.as<const char*>();
        portENTER_CRITICAL(&stateMux);
        strncpy(libCurrentSetlistName, nm ? nm : "", 39); libCurrentSetlistName[39] = 0;
        portEXIT_CRITICAL(&stateMux);
    }
    // Editing a rig (swap/add/remove a block) changes the Chain but not the rig
    // name, so re-resolve on any Chain structure change too — covers edits made
    // on the Prime or anywhere, not just rig selection.
    if (path == "/Evil/Engine/Patch/Chain") rigChangedReq = true;
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
    libSetlistCount = 0;
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
        libSetlistCount = n;
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
    libRigCount = 0;
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
        libRigCount = n;
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

// Generic category for a chain module index, asked from the Prime itself
// (categoryOfBlock) so a newly added/downloaded block categorizes correctly,
// cached per index for the session. Falls back to the baked table if the device
// call fails. Net task only.
BlockCat resolveCat(int idx) {
    if (idx <= 0 || idx >= MODULE_TYPE_COUNT) return BC_NONE;
    if (gCatCache[idx] >= 0) return (BlockCat)gCatCache[idx];
    BlockCat c = moduleCategory(idx);          // baked fallback (pre-seeded snapshot)
    String name = moduleName(idx);
    if (name.length()) {
        JsonDocument args; JsonArray a = args.to<JsonArray>(); a.add(name);
        JsonDocument res;
        if (hr.callMethod("/Evil/API/Blocks", "categoryOfBlock", args.as<JsonVariantConst>(), res)) {
            BlockCat rc = genericCatFromDeviceString(res["methodReturnValue"] | "");
            if (rc != BC_NONE) c = rc;
        }
    }
    gCatCache[idx] = (int8_t)c;
    return c;
}

// Resolve the per-rig dynamic dials from the loaded rig's signal chain (net task).
// Walks the 14 chain slots in order, maps each to a generic category (asked from
// the Prime), then fills the Gain dial (Drive>Amp>Comp) and Ambience dial
// (Reverb>Delay>Mod>Filter>Pitch) from the first present block in each bucket.
// Empty buckets cross-fill from the other. The primary param is the first
// candidate the actual block exposes; its range/format come from object-meta.
void resolveLayoutForRig() {
    gDynOk[0] = gDynOk[1] = false;
    JsonDocument chain;
    if (!hr.getProperties("/Evil/Engine/Patch/Chain", chain)) {
        Serial.println("[layout] chain fetch FAIL");
        layoutDirty = true;   // still let the UI rebuild (dials fall back to globals)
        return;
    }
    // First chain slot (1-based, chain order) holding each generic category.
    int firstSlotForCat[BC_COUNT];
    for (int i = 0; i < BC_COUNT; ++i) firstSlotForCat[i] = -1;
    uint16_t mask = 0;
    for (int slot = 1; slot <= 14; ++slot) {
        char key[16]; snprintf(key, sizeof(key), "ModuleType%d", slot);
        int idx = chain[key] | 0;
        BlockCat c = resolveCat(idx);
        if (c == BC_NONE) continue;
        mask |= (uint16_t)(1u << c);
        if (firstSlotForCat[c] < 0) firstSlotForCat[c] = idx;   // store module idx
    }
    gPresentCatMask = mask;

    // Pick the first present category from a bucket whose block exposes a usable
    // param, build gDynBind[slot], and return the category used (or BC_NONE).
    auto fill = [&](int dslot, const BlockCat* bucket, int bn, BlockCat exclude) -> BlockCat {
        for (int i = 0; i < bn; ++i) {
            BlockCat c = bucket[i];
            if (c == exclude || firstSlotForCat[c] < 0) continue;
            const CatBindSpec* spec = catBindSpec(c);
            if (!spec) continue;
            int idx = firstSlotForCat[c];
            String path = moduleBlockPath(idx);
            if (path.isEmpty()) continue;
            JsonDocument bd;
            if (!hr.getProperties(path, bd)) continue;
            for (int k = 0; k < 5 && spec->candidates[k]; ++k) {
                const char* prop = spec->candidates[k];
                if (!bd[prop].is<float>()) continue;
                // Range + format live from the device so any unit renders right.
                float mn = 0, mx = 100; const char* fmt = "%.0f %%";
                JsonDocument md;
                if (hr.getMeta(path, md)) {
                    JsonVariantConst pm = md["properties"][prop];
                    if (!pm.isNull()) {
                        if (pm["minimum"].is<float>()) mn = pm["minimum"].as<float>();
                        if (pm["maximum"].is<float>()) mx = pm["maximum"].as<float>();
                        const char* f = pm["x-options"]["format"] | "";
                        if (f[0]) fmt = f;
                    }
                }
                String dev = moduleName(idx);
                buildDynBinding(dslot, spec->label, dev.c_str(), path, prop, mn, mx, fmt);
                gDynOk[dslot] = true;
                Serial.printf("[layout] dial%d = %s [%s] %s.%s\n", dslot + 1, spec->label,
                              dev.c_str(), path.c_str(), prop);
                return c;
            }
        }
        return BC_NONE;
    };

    BlockCat gainCat  = fill(0, GAIN_BUCKET, GAIN_BUCKET_COUNT, BC_NONE);
    BlockCat spaceCat = fill(1, SPACE_BUCKET, SPACE_BUCKET_COUNT, gainCat);
    // Cross-fill: a board with an empty bucket borrows the other bucket's leftover.
    if (!gDynOk[0]) gainCat  = fill(0, SPACE_BUCKET, SPACE_BUCKET_COUNT, spaceCat);
    if (!gDynOk[1]) spaceCat = fill(1, GAIN_BUCKET,  GAIN_BUCKET_COUNT,  gainCat);

    layoutDirty = true;   // tell the UI to rebuild HOME from the new layout
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
        snprintf(bootMsg, sizeof(bootMsg), "linking Prime");
        hr.onValueChanged(onValueChanged);
        hr.onConnection([](bool up) {
            connStatus = up ? CS_HEADRUSH : CS_HR_LOST;
            Serial.printf("WS %s\n", up ? "connected" : "disconnected");
        });
        hr.begin(host);
        primeInitialValue();
        rigChangedReq = true;   // resolve the loaded rig's layout once the link is up
        setupOTA();
        if (FW_VERSION > 0) {  // dev builds (fw 0) skip auto-update
            // Stagger the boot update-check so a rack of boards powered on together
            // don't all hit WiFi/TLS + the ~1.4MB download at once (which starved
            // one unit and made it time out). Per-device base slot guarantees
            // separation; esp_random() jitter keeps them from relocking each boot.
            uint32_t jitterMs = (uint32_t)(deviceId - 1) * 3000 + (esp_random() % 2000);
            snprintf(bootMsg, sizeof(bootMsg), "update in %us", (unsigned)((jitterMs + 999) / 1000));
            vTaskDelay(pdMS_TO_TICKS(jitterMs));
            checkForUpdate("boot");
        }
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
            if (rigChangedReq) { rigChangedReq = false; doFetchRigList(); resolveLayoutForRig(); }  // rig swapped -> re-resolve dials
            if (rebindRequested) { rebindRequested = false; if (!tunerActive && !homeListActive) primeInitialValue(); }  // new control focused
            if (libFetchSetlistsReq) { libFetchSetlistsReq = false; doLibFetchSetlists(); }
            if (fetchRigListReq)     { fetchRigListReq = false; doFetchRigList(); }
            if (loadSetlistReq)      { loadSetlistReq = false; doLoadSetlist(); }
            if (libLoadRigReq)       { libLoadRigReq = false; doLibLoadRig(); }
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
    homeGroupLen = prefs.getInt("hglen", 0);
    if (homeGroupLen >= 1 && homeGroupLen <= VIEW_COUNT) {
        prefs.getBytes("hgrp", homeGroup, homeGroupLen * sizeof(int));
    } else {
        // Migrate an older single-param install (NVS key "param") to a 1-member group.
        int p = prefs.getInt("param", 0);
        if (p < 0 || p >= PARAM_COUNT) p = 0;
        homeGroup[0] = p; homeGroupLen = 1;
    }
    prefs.end();
#ifdef PROVISION_ID
    // Fleet provisioning: bake this unit's ID at flash time, but only if it
    // hasn't been set yet — config-mode changes still win afterward.
    if (deviceId == 0) { deviceId = PROVISION_ID; saveDeviceId(deviceId); }
#endif
    if (deviceId < 1 || deviceId > 16) deviceId = 1;
    // Validate group members against the known view set.
    int n = 0;
    for (int i = 0; i < homeGroupLen && i < VIEW_COUNT; ++i) {
        int v = homeGroup[i];
        if (viewIsValid(v)) homeGroup[n++] = v;
    }
    homeGroupLen = (n > 0) ? n : 1;
    if (n == 0) homeGroup[0] = 0;
    homeFocus = 0;
    focusHomeMember(0);
    Serial.printf("Device ID: %d, home group len=%d, view %s\n", deviceId, homeGroupLen,
                  tunerActive ? "Tuner" : activeBinding->label);

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

    // A rig change (net task resolved a new layout): adopt this rig's saved HOME
    // assignment or the generated default, then refocus to refresh the dial. Only
    // rebuild while on HOME — a menu the user is in shouldn't yank out from under
    // them; the new layout takes effect when they return to HOME.
    if (layoutDirty && screen == SC_HOME) {
        layoutDirty = false;
        rebuildHomeForRig();
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
                assignSelLen = homeGroupLen;
                for (int i = 0; i < homeGroupLen; ++i) assignSel[i] = homeGroup[i];
                assignCursor = 0;
                screen = SC_ASSIGN;
                Render::drawAssignList(canvas, PARAM_CATALOG, PARAM_COUNT, assignCursor, assignSel, assignSelLen);
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

    // --- Assign this knob (multi-select -> ordered Home group) ---
    if (screen == SC_ASSIGN) {
        const int items = PARAM_COUNT + 3;   // dials, Tuner, Setlist, Rig
        int cursorView = (assignCursor < PARAM_COUNT) ? assignCursor
                        : (assignCursor == PARAM_COUNT) ? TUNER_VIEW
                        : (assignCursor == PARAM_COUNT + 1) ? SETLIST_VIEW : RIG_VIEW;
        if (longPress) {                      // confirm the group, back to Home
            if (assignSelLen > 0) {
                homeGroupLen = assignSelLen;
                for (int i = 0; i < assignSelLen; ++i) homeGroup[i] = assignSel[i];
                saveHomeGroup();              // global fallback (used until a rig is known)
                char rigId[40];              // and persist this layout for the loaded rig
                portENTER_CRITICAL(&stateMux);
                strncpy(rigId, libCurrentRigId, sizeof(rigId)); rigId[sizeof(rigId) - 1] = 0;
                portEXIT_CRITICAL(&stateMux);
                saveHomeForRig(rigId);
                focusHomeMember(0);
            }
            screen = SC_HOME;
            lastRenderedValue = -999999.0f; lastTunerCents = -999.0f;
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        if (clicked) {                        // toggle the cursor control in/out of the group
            int at = -1;
            for (int i = 0; i < assignSelLen; ++i) if (assignSel[i] == cursorView) { at = i; break; }
            if (at >= 0) {                    // remove (preserve order)
                for (int i = at; i < assignSelLen - 1; ++i) assignSel[i] = assignSel[i + 1];
                assignSelLen--;
            } else if (assignSelLen < VIEW_COUNT) {
                assignSel[assignSelLen++] = cursorView;   // append in selection order
            }
            Render::drawAssignList(canvas, PARAM_CATALOG, PARAM_COUNT, assignCursor, assignSel, assignSelLen);
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        if (edelta != 0) {
            assignCursor = (int)(((assignCursor + edelta) % items + items) % items);
            Render::drawAssignList(canvas, PARAM_CATALOG, PARAM_COUNT, assignCursor, assignSel, assignSelLen);
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
    // hold opens the Board Menu.
    if (longPress) {
        boardSel = 0; screen = SC_BOARD_MENU;
        Render::drawBoardMenu(canvas, boardSel);
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }
    // single-click steps to the next group member (no-op for a group of one).
    if (singleClick && homeGroupLen > 1) {
        focusHomeMember(homeFocus + 1);
        lastRenderedValue = -999999.0f; lastTunerCents = -999.0f;
        vTaskDelay(pdMS_TO_TICKS(5));
        return;
    }
    // double-click = the focused member's action: toggle the focused effect
    // block's bypass (Phase 2 wires the real call). No-op for tuner/list types.
    if (doubleClick && !tunerActive && !homeListActive) {
        bypassToggleReq = true;
    }

    // --- Setlist / Rig scroll-list Home --- turn scrolls the names; after a
    // brief pause the highlighted item is loaded (no click-to-select).
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
            listCursor = (((listCursor + edelta) % count) + count) % count;
            lastListScrollMs = millis();
            listDirty = true;
        }
        // Pause-commit: a moment after the last turn, load the highlighted item.
        if (listCursor != listCommitted && (millis() - lastListScrollMs) > LIST_COMMIT_MS) {
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
        float nv = sharedValue + edelta * activeBinding->step;
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
