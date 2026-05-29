#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <functional>

// Thin wrapper around the HeadRush Prime's local protocol.
//
// HTTP REST at http://<host>/api/v1
//   GET    /object-properties{path}       read current values
//   PUT    /object-properties{path}       write {prop:value, ...}
//   POST   /object-method{path}/{method}  body {"arguments":[...]}, returns {"methodReturnValue":...}
//   GET    /object-meta{path}             JSON-schema-style metadata
//   GET    /subtree{path}                 recursive dump
//
// WebSocket at ws://<host>/   push-only frames:
//   {"notification":"propertyValueChanged","objectPath":"/Evil/...","propertyName":"X","propertyValue":...}
//
// Numeric properties travel as normalized 0..1 floats; see Normalize.h.

class HeadRushClient {
public:
    using ValueChangedCb = std::function<void(const String& path, const String& prop, const JsonVariantConst& value)>;
    using ConnectionCb   = std::function<void(bool connected)>;

    void begin(const String& host);
    void loop();
    void stop();   // drop the WebSocket (frees its buffers) — used to reclaim heap for OTA TLS

    // HTTP — blocking. Returns true on 2xx.
    bool getProperties(const String& path, JsonDocument& out);
    bool setProperties(const String& path, const JsonVariantConst& propsObj);
    bool callMethod(const String& path, const String& method, const JsonVariantConst& args, JsonDocument& out);

    // Convenience: write a single property. T can be any JSON-assignable type
    // (bool, int, float, const char*, String, JsonVariantConst, ...).
    template <typename T>
    bool setProperty(const String& path, const String& prop, T value) {
        JsonDocument doc;
        doc[prop] = value;
        return setProperties(path, doc.as<JsonVariantConst>());
    }

    void onValueChanged(ValueChangedCb cb) { _valueCb = std::move(cb); }
    void onConnection(ConnectionCb cb) { _connCb = std::move(cb); }

    bool wsConnected() const { return _wsConnected; }

private:
    String _host;
    String _baseUrl;       // http://host/api/v1
    WebSocketsClient _ws;
    bool _wsConnected = false;
    ValueChangedCb _valueCb;
    ConnectionCb _connCb;

    bool httpRequest(const char* method, const String& url, const String& body, JsonDocument* outDoc);
    void onWsEvent(WStype_t type, uint8_t* payload, size_t len);
};
