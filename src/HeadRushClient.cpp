#include "HeadRushClient.h"
#include <HTTPClient.h>
#include <WiFi.h>

void HeadRushClient::begin(const String& host) {
    _host = host;
    _baseUrl = "http://" + host + "/api/v1";

    _ws.begin(host.c_str(), 80, "/");
    _ws.setReconnectInterval(2000);
    _ws.enableHeartbeat(15000, 3000, 2);
    _ws.onEvent([this](WStype_t type, uint8_t* payload, size_t len) {
        onWsEvent(type, payload, len);
    });
}

void HeadRushClient::loop() {
    _ws.loop();
}

void HeadRushClient::stop() {
    _ws.disconnect();
    _wsConnected = false;
}

bool HeadRushClient::httpRequest(const char* method, const String& url, const String& body, JsonDocument* outDoc) {
    HTTPClient http;
    http.setReuse(true);
    http.setTimeout(3000);
    if (!http.begin(url)) return false;

    int code;
    if (strcmp(method, "GET") == 0) {
        code = http.GET();
    } else {
        http.addHeader("Content-Type", "application/json");
        code = http.sendRequest(method, body);
    }

    bool ok = (code >= 200 && code < 300);
    if (ok && outDoc) {
        DeserializationError err = deserializeJson(*outDoc, http.getStream());
        if (err) ok = false;
    }
    http.end();
    return ok;
}

bool HeadRushClient::getProperties(const String& path, JsonDocument& out) {
    return httpRequest("GET", _baseUrl + "/object-properties" + path, String(), &out);
}

bool HeadRushClient::getMeta(const String& path, JsonDocument& out) {
    return httpRequest("GET", _baseUrl + "/object-meta" + path, String(), &out);
}

bool HeadRushClient::setProperties(const String& path, const JsonVariantConst& propsObj) {
    String body;
    serializeJson(propsObj, body);
    return httpRequest("PUT", _baseUrl + "/object-properties" + path, body, nullptr);
}

bool HeadRushClient::callMethod(const String& path, const String& method, const JsonVariantConst& args, JsonDocument& out) {
    JsonDocument doc;
    doc["arguments"] = args;
    String body;
    serializeJson(doc, body);
    return httpRequest("POST", _baseUrl + "/object-method" + path + "/" + method, body, &out);
}

void HeadRushClient::onWsEvent(WStype_t type, uint8_t* payload, size_t len) {
    switch (type) {
        case WStype_CONNECTED:
            _wsConnected = true;
            if (_connCb) _connCb(true);
            break;
        case WStype_DISCONNECTED:
            _wsConnected = false;
            if (_connCb) _connCb(false);
            break;
        case WStype_TEXT: {
            JsonDocument doc;
            if (deserializeJson(doc, payload, len)) return;
            const char* notif = doc["notification"];
            if (!notif) return;
            if (strcmp(notif, "propertyValueChanged") == 0 && _valueCb) {
                _valueCb(doc["objectPath"].as<String>(),
                         doc["propertyName"].as<String>(),
                         doc["propertyValue"]);
            }
            // objectAdded / objectRemoved / objectPropertyAdded/Removed / webAccessChanged
            // are ignored for now; add hooks here when a page needs them.
            break;
        }
        default: break;
    }
}
