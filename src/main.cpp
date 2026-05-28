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
#include <time.h>
#include <AiEsp32RotaryEncoder.h>
#include "HeadRushClient.h"
#include "Normalize.h"
#include "Hardware.h"
#include "Display.h"
#include "Binding.h"
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
// Selected from the saved parameter index in setup(), before the net task
// starts. Read-only after.
const ContinuousBinding* activeBinding = &PARAM_CATALOG[0];

constexpr int ENCODER_SIGN = -1;
constexpr uint32_t WRITE_THROTTLE_MS = 30;
constexpr uint32_t ECHO_SUPPRESS_MS = 500;

// ---- Shared state ----
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
volatile float sharedValue = 0.0f;            // in display units (e.g. dB)
volatile float pendingWriteValue = 0;
volatile bool  pendingWriteValid = false;
volatile uint32_t lastLocalChangeMs = 0;

// OTA state. Written by ArduinoOTA callbacks on the net task, read by the UI
// task to render the progress screen. Plain volatile is sufficient — these are
// 32-bit aligned and only drive a progress display, not control flow timing.
volatile bool otaActive = false;
volatile int  otaPercent = 0;
volatile bool otaChecking = false;        // true while polling GitHub for a manifest
char otaStatusMsg[40] = "checking...";    // shown on the checking screen (diagnostic)
volatile bool otaCheckRequested = false;  // set by UI long-press, consumed by net task
volatile bool rebindRequested = false;    // set when config picks a new param; net re-primes

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

// Per-device identity (1..16) and chosen parameter (index into PARAM_CATALOG),
// both set on-device in config mode and persisted in NVS.
volatile int deviceId = 1;
int paramIndex = 0;
// Config mode is a UI-task-local modal state, entered by a long (~5s) press.
enum ConfigState { CFG_NONE, CFG_MENU, CFG_EDIT_ID, CFG_EDIT_PARAM };
ConfigState cfg = CFG_NONE;
int menuIndex = 0;
int editIdValue = 1;
int editParamValue = 0;

struct WifiCreds { String ssid; String psk; String hostOverride; };

WifiCreds loadCreds() {
    WifiCreds c;
    if (strlen(WIFI_SSID) > 0) {
        c.ssid = WIFI_SSID;
        c.psk = WIFI_PSK;
        c.hostOverride = HEADRUSH_HOST_OVERRIDE;
        // Seed NVS so OTA-delivered (CI) firmware — which has no compiled-in
        // secrets.h — can still join WiFi. NVS survives OTA, so a single local
        // flash provisions the unit for all future over-the-air updates.
        Preferences p;
        if (p.begin("hrctrl", false)) {
            p.putString("ssid", c.ssid);
            p.putString("psk", c.psk);
            p.putString("host", c.hostOverride);
            p.end();
        }
        return c;
    }
    prefs.begin("hrctrl", true);
    c.ssid = prefs.getString("ssid", "");
    c.psk = prefs.getString("psk", "");
    c.hostOverride = prefs.getString("host", "");
    prefs.end();
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

void saveParamIndex(int idx) {
    Preferences p;
    bool ok = p.begin("hrctrl", false);
    size_t n = p.putInt("param", idx);
    int rb = p.getInt("param", -1);
    p.end();
    Serial.printf("[nvs] param<-%d %s (open=%d wrote=%u readback=%d)\n",
                  idx, PARAM_CATALOG[idx].label, ok, (unsigned)n, rb);
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

    // Verified TLS needs a real clock; re-sync (with retries) if the boot sync
    // didn't take. NTP is often blocked and a one-shot HTTPS Date can miss.
    for (int i = 0; i < 3 && time(nullptr) < 1700000000; ++i)
        if (!syncClockFromHttp()) delay(800);

    // Fetch the manifest, retrying transient TLS/network failures.
    int code = 0;
    String body;
    for (int attempt = 0; attempt < 3 && code != HTTP_CODE_OK; ++attempt) {
        NetworkClientSecure client;
        client.setCACert(GITHUB_ROOTS);  // pinned GitHub roots (the IDF bundle wouldn't validate)
        HTTPClient http;
        http.setConnectTimeout(10000);
        http.setTimeout(10000);
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);  // GitHub 302s to its CDN
        if (http.begin(client, FW_MANIFEST_URL)) {
            http.addHeader("User-Agent", "headrush-controller");
            code = http.GET();
            if (code == HTTP_CODE_OK) body = http.getString();
            http.end();
        }
        if (code != HTTP_CODE_OK) delay(700);
    }
    if (code != HTTP_CODE_OK) {
        snprintf(otaStatusMsg, sizeof(otaStatusMsg), "check failed (%d)", code);
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

    snprintf(otaStatusMsg, sizeof(otaStatusMsg), "updating v%d", remote);
    otaPercent = 0;
    otaActive = true;       // UI switches to the progress ring
    otaChecking = false;
    NetworkClientSecure dlClient;
    dlClient.setCACert(GITHUB_ROOTS);
    httpUpdate.rebootOnUpdate(true);
    httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    httpUpdate.onProgress([](int cur, int total) {
        otaPercent = total ? (int)((cur * 100L) / total) : 0;
    });
    t_httpUpdate_return r = httpUpdate.update(dlClient, url);
    // Only reached if the update did NOT succeed (success reboots into the new image).
    otaActive = false;
    snprintf(otaStatusMsg, sizeof(otaStatusMsg), "update failed %d", (int)r);
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
        setupOTA();
        if (FW_VERSION > 0) checkForUpdate("boot");  // dev builds (fw 0) skip auto-update
    } else {
        connStatus = CS_WIFI_ERR;
        snprintf(bootMsg, sizeof(bootMsg), "WiFi failed");
    }
    bootComplete = true;  // never delete the task — stay alive so the UI keeps working

    uint32_t lastWriteMs = 0;
    while (true) {
        if (online) {
            ArduinoOTA.handle();  // blocks here for the duration of a push update
            if (otaCheckRequested) { otaCheckRequested = false; checkForUpdate("manual"); }
            if (rebindRequested) { rebindRequested = false; primeInitialValue(); }  // new param picked
            hr.loop();
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
    deviceId = prefs.getInt("devid", 1);
    paramIndex = prefs.getInt("param", 0);
    prefs.end();
    if (paramIndex < 0 || paramIndex >= PARAM_COUNT) paramIndex = 0;
    activeBinding = &PARAM_CATALOG[paramIndex];
    Serial.printf("Device ID: %d, param %s\n", deviceId, activeBinding->label);

    pinMode(HW::PIN_PWR_EN_1, OUTPUT); digitalWrite(HW::PIN_PWR_EN_1, HIGH);
    pinMode(HW::PIN_PWR_EN_2, OUTPUT); digitalWrite(HW::PIN_PWR_EN_2, HIGH);

    gfx.init();
    canvas.setColorDepth(16);
    canvas.setPsram(true);
    if (!canvas.createSprite(240, 240)) Serial.println("canvas alloc FAILED");
    ledcAttach(HW::PIN_DISP_BL, DisplayHW::BL_LEDC_FREQ, DisplayHW::BL_LEDC_RESOLUTION_BITS);
    ledcWrite(HW::PIN_DISP_BL, (DisplayHW::BL_DEFAULT_PCT * 255) / 100);
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
    if (uiMode != UI_GAUGE && cfg == CFG_NONE) { uiMode = UI_GAUGE; lastRenderedValue = -999999.0f; }

    // --- Read input once: a short click, a ~5s long-press, and the turn delta ---
    bool clicked = false, longPress = false;
    {
        static uint32_t down = 0;
        static bool fired = false;
        if (encoder.isEncoderButtonDown()) {
            if (down == 0) { down = millis(); fired = false; }
            else if (!fired && millis() - down >= 5000) { longPress = true; fired = true; }
        } else {
            if (down != 0 && !fired) {
                uint32_t held = millis() - down;
                if (held >= 40 && held < 5000) clicked = true;  // debounced short press
            }
            down = 0;
        }
    }
    long edelta = 0;
    if (encoder.encoderChanged()) {
        long v = encoder.readEncoder();
        edelta = (v - lastEncoderValue) * ENCODER_SIGN;
        lastEncoderValue = v;
    }

    // --- Config menu ---
    if (cfg == CFG_MENU) {
        if (longPress) { cfg = CFG_NONE; uiMode = UI_GAUGE; lastRenderedValue = -999999.0f; vTaskDelay(pdMS_TO_TICKS(5)); return; }
        if (clicked) {
            if (menuIndex == 0) { editIdValue = deviceId; cfg = CFG_EDIT_ID; Render::drawConfigIdEdit(canvas, editIdValue); }
            else if (menuIndex == 1) { editParamValue = paramIndex; cfg = CFG_EDIT_PARAM; Render::drawParamPick(canvas, PARAM_CATALOG, PARAM_COUNT, editParamValue); }
            else if (menuIndex == 2) { otaCheckRequested = true; cfg = CFG_NONE; uiMode = UI_GAUGE; lastRenderedValue = -999999.0f; }
            else { cfg = CFG_NONE; uiMode = UI_GAUGE; lastRenderedValue = -999999.0f; }
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        if (edelta != 0) {
            menuIndex = (int)(((menuIndex + edelta) % 4 + 4) % 4);
            Render::drawConfigMenu(canvas, menuIndex, deviceId, PARAM_CATALOG[paramIndex].label, FW_VERSION);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }

    // --- Device-ID editor ---
    if (cfg == CFG_EDIT_ID) {
        if (longPress) { cfg = CFG_MENU; Render::drawConfigMenu(canvas, menuIndex, deviceId, PARAM_CATALOG[paramIndex].label, FW_VERSION); vTaskDelay(pdMS_TO_TICKS(5)); return; }
        if (clicked) {
            if (editIdValue != deviceId) {
                deviceId = editIdValue;     // identity only; applies immediately
                saveDeviceId(deviceId);
            }
            cfg = CFG_MENU;
            Render::drawConfigMenu(canvas, menuIndex, deviceId, PARAM_CATALOG[paramIndex].label, FW_VERSION);
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

    // --- Parameter picker ---
    if (cfg == CFG_EDIT_PARAM) {
        if (longPress) { cfg = CFG_MENU; Render::drawConfigMenu(canvas, menuIndex, deviceId, PARAM_CATALOG[paramIndex].label, FW_VERSION); vTaskDelay(pdMS_TO_TICKS(5)); return; }
        if (clicked) {
            if (editParamValue != paramIndex) {
                // Apply immediately: swap the binding and have the net task
                // re-prime the new parameter's current value. No reboot.
                paramIndex = editParamValue;
                activeBinding = &PARAM_CATALOG[paramIndex];
                saveParamIndex(paramIndex);
                rebindRequested = true;
            }
            cfg = CFG_MENU;
            Render::drawConfigMenu(canvas, menuIndex, deviceId, PARAM_CATALOG[paramIndex].label, FW_VERSION);
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        if (edelta != 0) {
            editParamValue = (int)(((editParamValue + edelta) % PARAM_COUNT + PARAM_COUNT) % PARAM_COUNT);
            Render::drawParamPick(canvas, PARAM_CATALOG, PARAM_COUNT, editParamValue);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }

    // --- Normal gauge mode ---
    if (longPress) {  // enter config
        cfg = CFG_MENU;
        menuIndex = 0;
        Render::drawConfigMenu(canvas, menuIndex, deviceId, PARAM_CATALOG[paramIndex].label, FW_VERSION);
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
    if (fabsf(v - lastRenderedValue) >= activeBinding->step * 0.5f || sc != lastStatusColor || sl != lastSignalLevel) {
        Render::drawContinuous(canvas, *activeBinding, v, sc, sl);
        lastRenderedValue = v;
        lastStatusColor = sc;
        lastSignalLevel = sl;
    }

    vTaskDelay(pdMS_TO_TICKS(2));
}
