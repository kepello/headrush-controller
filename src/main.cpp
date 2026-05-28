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
#include <AiEsp32RotaryEncoder.h>
#include "HeadRushClient.h"
#include "Normalize.h"
#include "Hardware.h"
#include "Display.h"
#include "Binding.h"
#include "Render.h"

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

// ---- Knob 1 starter ring (step A: just one binding) ----
// Master output: /Evil/Engine/Patch/Output.RigVolume, range -60..+36 dB.
// Color zones: dim blue safely below stage, green nominal, yellow getting
// hot, red clipping risk.
const ContinuousBinding MASTER_OUTPUT = {
    .label   = "OUTPUT",
    .path    = "/Evil/Engine/Patch/Output",
    .prop    = "RigVolume",
    .dispMin = -60.0f,
    .dispMax = +36.0f,
    .step    = 0.5f,
    .format  = "%+.1f",
    .unit    = "dB",
    .zones = {
        { -30.0f, 0x6B7F },  // soft blue (low)
        { -6.0f,  0x07E0 },  // green (nominal)
        { +6.0f,  0xFFE0 },  // yellow (hot)
        { +36.0f, 0xF800 },  // red (clipping risk)
    },
    .zoneCount = 4,
};
const ContinuousBinding* activeBinding = &MASTER_OUTPUT;

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
volatile bool otaCheckRequested = false;  // set by UI long-press, consumed by net task

HeadRushClient hr;
Preferences prefs;
CrowPanelLGFX gfx;
String gHostname;  // unique per device (MAC-derived); set once in netTask
AiEsp32RotaryEncoder encoder(HW::PIN_ENC_A, HW::PIN_ENC_B, HW::PIN_ENC_BTN, /*vcc=*/-1, /*steps_per_click=*/4);
void IRAM_ATTR encoderISR() { encoder.readEncoder_ISR(); }
long lastEncoderValue = 0;
float lastRenderedValue = -999999.0f;
int lastRenderedOtaPercent = -1;
enum UiMode { UI_GAUGE, UI_CHECKING, UI_UPDATING };
UiMode uiMode = UI_GAUGE;

// Per-device identity (1..16), set on-device in config mode, persisted in NVS.
volatile int deviceId = 1;
// Config mode is a UI-task-local modal state, entered by a long (~5s) press.
enum ConfigState { CFG_NONE, CFG_MENU, CFG_EDIT_ID };
ConfigState cfg = CFG_NONE;
int menuIndex = 0;
int editIdValue = 1;

struct WifiCreds { String ssid; String psk; String hostOverride; };

WifiCreds loadCreds() {
    WifiCreds c;
    if (strlen(WIFI_SSID) > 0) {
        c.ssid = WIFI_SSID;
        c.psk = WIFI_PSK;
        c.hostOverride = HEADRUSH_HOST_OVERRIDE;
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
    p.begin("hrctrl", false);
    p.putInt("devid", id);
    p.end();
    Serial.printf("Device ID saved: %d\n", id);
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

// Full Mozilla CA bundle, embedded in the IDF mbedtls library (esp_crt_bundle).
// Referenced directly so TLS verification needs no generated/committed bundle.
extern const uint8_t x509_crt_bundle_start[] asm("_binary_x509_crt_bundle_start");
extern const uint8_t x509_crt_bundle_end[]   asm("_binary_x509_crt_bundle_end");

// Pull update: fetch version.json from GitHub Releases, and if it advertises a
// build number higher than ours, download + flash firmware.bin. Runs in the net
// task. On a successful flash the device reboots (never returns from update()).
void checkForUpdate(const char* reason) {
    otaChecking = true;
    Serial.printf("[upd] checking (%s)\n", reason);

    NetworkClientSecure client;
    // Verify the server cert against the full Mozilla CA bundle. Covers both
    // github.com (USERTrust root) and the release CDN (ISRG/Let's Encrypt root),
    // and survives CA rotation — so a MITM can't serve forged firmware.
    client.setCACertBundle(x509_crt_bundle_start, x509_crt_bundle_end - x509_crt_bundle_start);

    HTTPClient http;
    http.setConnectTimeout(8000);
    http.setTimeout(8000);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);  // GitHub 302s to its CDN
    if (!http.begin(client, FW_MANIFEST_URL)) {
        Serial.println("[upd] http begin failed");
        otaChecking = false; return;
    }
    http.addHeader("User-Agent", "headrush-controller");
    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        Serial.printf("[upd] manifest HTTP %d\n", code);
        http.end(); otaChecking = false; return;
    }
    String body = http.getString();
    http.end();

    JsonDocument doc;
    if (deserializeJson(doc, body)) {
        Serial.println("[upd] manifest parse error");
        otaChecking = false; return;
    }
    int remote = doc["version"] | -1;
    String url = doc["url"] | "";
    Serial.printf("[upd] remote v%d, local v%d\n", remote, FW_VERSION);
    if (remote <= FW_VERSION || url.isEmpty()) {
        Serial.println("[upd] up to date");
        otaChecking = false; return;
    }

    Serial.printf("[upd] updating → v%d\n", remote);
    otaPercent = 0;
    otaActive = true;       // UI switches to the progress ring
    otaChecking = false;
    httpUpdate.rebootOnUpdate(true);
    httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    httpUpdate.onProgress([](int cur, int total) {
        otaPercent = total ? (int)((cur * 100L) / total) : 0;
    });
    t_httpUpdate_return r = httpUpdate.update(client, url);
    // Only reached if the update did NOT succeed (success reboots into the new image).
    otaActive = false;
    Serial.printf("[upd] update failed (%d): %s\n", (int)r, httpUpdate.getLastErrorString().c_str());
}

void netTask(void*) {
    Serial.println("[net] starting");
    gHostname = makeHostname();
    Serial.printf("[net] hostname: %s.local\n", gHostname.c_str());
    WifiCreds c = loadCreds();
    if (!connectWifi(c)) { vTaskDelete(NULL); return; }
    String host = resolveHost(c);
    hr.onValueChanged(onValueChanged);
    hr.onConnection([](bool up) { Serial.printf("WS %s\n", up ? "connected" : "disconnected"); });
    hr.begin(host);
    primeInitialValue();
    setupOTA();
    if (FW_VERSION > 0) checkForUpdate("boot");  // dev builds (fw 0) skip auto-update

    uint32_t lastWriteMs = 0;
    while (true) {
        ArduinoOTA.handle();  // blocks here for the duration of a push update
        if (otaCheckRequested) { otaCheckRequested = false; checkForUpdate("manual"); }
        hr.loop();
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
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.printf("\n=== HeadRush Controller — Stage 1B (fw %d) ===\n", FW_VERSION);

    prefs.begin("hrctrl", true);
    deviceId = prefs.getInt("devid", 1);
    prefs.end();
    Serial.printf("Device ID: %d\n", deviceId);

    pinMode(HW::PIN_PWR_EN_1, OUTPUT); digitalWrite(HW::PIN_PWR_EN_1, HIGH);
    pinMode(HW::PIN_PWR_EN_2, OUTPUT); digitalWrite(HW::PIN_PWR_EN_2, HIGH);

    gfx.init();
    ledcAttach(HW::PIN_DISP_BL, DisplayHW::BL_LEDC_FREQ, DisplayHW::BL_LEDC_RESOLUTION_BITS);
    ledcWrite(HW::PIN_DISP_BL, (DisplayHW::BL_DEFAULT_PCT * 255) / 100);
    char boot[24];
    snprintf(boot, sizeof(boot), "ID %d  booting...", deviceId);
    Render::drawBootScreen(gfx, boot);

    encoder.begin();
    encoder.setup(encoderISR);
    encoder.setBoundaries(-100000, 100000, false);
    encoder.setAcceleration(0);
    encoder.setEncoderValue(0);
    lastEncoderValue = 0;

    xTaskCreatePinnedToCore(netTask, "net", 8192, NULL, 5, NULL, 0);
}

void loop() {
    // Net-driven overlays take priority over everything, including config mode:
    // a download/flash may be running on the other core. Redraw only on a mode
    // or percent change so the screen doesn't flicker.
    if (otaActive) {  // downloading + flashing
        if (uiMode != UI_UPDATING) { uiMode = UI_UPDATING; lastRenderedOtaPercent = -1; }
        int p = otaPercent;
        if (p != lastRenderedOtaPercent) { Render::drawOTAProgress(gfx, p); lastRenderedOtaPercent = p; }
        vTaskDelay(pdMS_TO_TICKS(20));
        return;
    }
    if (otaChecking) {  // polling GitHub for a manifest
        if (uiMode != UI_CHECKING) { uiMode = UI_CHECKING; Render::drawBootScreen(gfx, "checking update..."); }
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
            if (menuIndex == 0) { editIdValue = deviceId; cfg = CFG_EDIT_ID; Render::drawConfigIdEdit(gfx, editIdValue); }
            else if (menuIndex == 1) { otaCheckRequested = true; cfg = CFG_NONE; uiMode = UI_GAUGE; lastRenderedValue = -999999.0f; }
            else { cfg = CFG_NONE; uiMode = UI_GAUGE; lastRenderedValue = -999999.0f; }
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        if (edelta != 0) {
            menuIndex = (int)(((menuIndex + edelta) % 3 + 3) % 3);
            Render::drawConfigMenu(gfx, menuIndex, deviceId, FW_VERSION);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }

    // --- Device-ID editor ---
    if (cfg == CFG_EDIT_ID) {
        if (longPress) { cfg = CFG_MENU; Render::drawConfigMenu(gfx, menuIndex, deviceId, FW_VERSION); vTaskDelay(pdMS_TO_TICKS(5)); return; }
        if (clicked) {
            deviceId = editIdValue;
            saveDeviceId(editIdValue);
            cfg = CFG_MENU;
            Render::drawConfigMenu(gfx, menuIndex, deviceId, FW_VERSION);
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        if (edelta != 0) {
            editIdValue += (int)edelta;
            if (editIdValue < 1) editIdValue = 1;
            if (editIdValue > 16) editIdValue = 16;
            Render::drawConfigIdEdit(gfx, editIdValue);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }

    // --- Normal gauge mode ---
    if (longPress) {  // enter config
        cfg = CFG_MENU;
        menuIndex = 0;
        Render::drawConfigMenu(gfx, menuIndex, deviceId, FW_VERSION);
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
    if (fabsf(v - lastRenderedValue) >= activeBinding->step * 0.5f) {
        Render::drawContinuous(gfx, *activeBinding, v);
        lastRenderedValue = v;
    }

    vTaskDelay(pdMS_TO_TICKS(2));
}
