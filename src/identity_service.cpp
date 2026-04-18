#include "identity_service.h"

#include <ArduinoJson.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>

namespace identity_service {
namespace {

static bool findStringFieldRecursive(JsonVariantConst node, const char *key, String &valueOut) {
  if (node.is<JsonObjectConst>()) {
    JsonObjectConst obj = node.as<JsonObjectConst>();

    JsonVariantConst directValue = obj[key];
    if (!directValue.isNull()) {
      if (directValue.is<const char *>()) {
        valueOut = String(directValue.as<const char *>());
        valueOut.trim();
        if (valueOut.length() > 0) {
          return true;
        }
      }
    }

    for (JsonPairConst kv : obj) {
      if (findStringFieldRecursive(kv.value(), key, valueOut)) {
        return true;
      }
    }

    return false;
  }

  if (node.is<JsonArrayConst>()) {
    JsonArrayConst array = node.as<JsonArrayConst>();
    for (JsonVariantConst item : array) {
      if (findStringFieldRecursive(item, key, valueOut)) {
        return true;
      }
    }
  }

  return false;
}

static bool extractUuidFromJson(const String &payload, String &uuidOut) {
  uuidOut = "";

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    return false;
  }

  const char *keys[] = {"hashed_uuid", "uuid", "identity", "deviceId", "id"};
  JsonVariantConst root = doc.as<JsonVariantConst>();

  for (size_t i = 0; i < (sizeof(keys) / sizeof(keys[0])); ++i) {
    String candidate;
    if (findStringFieldRecursive(root, keys[i], candidate) && candidate.length() > 0) {
      uuidOut = candidate;
      return true;
    }
  }

  return false;
}

static bool extractJsonStringFieldFallback(const String &json, const char *key, String &valueOut) {
  valueOut = "";

  String needle = String("\"") + key + "\"";
  int keyIndex = json.indexOf(needle);
  if (keyIndex < 0) {
    return false;
  }

  int colonIndex = json.indexOf(':', keyIndex + needle.length());
  if (colonIndex < 0) {
    return false;
  }

  int firstQuote = json.indexOf('"', colonIndex + 1);
  if (firstQuote < 0) {
    return false;
  }

  String value;
  bool escaped = false;
  for (size_t i = static_cast<size_t>(firstQuote + 1); i < json.length(); ++i) {
    char c = json[i];
    if (escaped) {
      switch (c) {
        case 'n':
          value += '\n';
          break;
        case 'r':
          value += '\r';
          break;
        case 't':
          value += '\t';
          break;
        case '\\':
          value += '\\';
          break;
        case '"':
          value += '"';
          break;
        default:
          value += c;
          break;
      }
      escaped = false;
      continue;
    }

    if (c == '\\') {
      escaped = true;
      continue;
    }

    if (c == '"') {
      value.trim();
      valueOut = value;
      return valueOut.length() > 0;
    }

    value += c;
  }

  return false;
}

}  // namespace

String normalizeBackendUrl(const String &url) {
  String normalized = url;
  normalized.trim();

  while (normalized.endsWith("/")) {
    normalized.remove(normalized.length() - 1);
  }

  return normalized;
}

String buildValidationUrl(const String &backendUrl) {
  String normalized = normalizeBackendUrl(backendUrl);

  if (normalized.endsWith("/api/gaze/validate/uuid")) {
    return normalized;
  }

  if (normalized.endsWith("/api")) {
    return normalized + "/gaze/validate/uuid";
  }

  return normalized + "/api/gaze/validate/uuid";
}

bool requestUuidFromBackend(const String &backendUrl,
                            const String &email,
                            const String &accountPassword,
                            String &uuidOut,
                            String &errorOut) {
  uuidOut = "";
  errorOut = "";

  String requestUrl = buildValidationUrl(backendUrl);
  if (!(requestUrl.startsWith("http://") || requestUrl.startsWith("https://"))) {
    errorOut = "Backend URL must start with http:// or https://";
    return false;
  }

  if (WiFi.status() != WL_CONNECTED) {
    errorOut = "Wi-Fi is not connected";
    return false;
  }

  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  WiFiClient *transportClient = &plainClient;
  if (requestUrl.startsWith("https://")) {
    secureClient.setInsecure();
    transportClient = &secureClient;
  }

  HTTPClient http;
  if (!http.begin(*transportClient, requestUrl)) {
    errorOut = "Failed to initialize HTTP client";
    return false;
  }

  http.setTimeout(10000);
  http.addHeader("Content-Type", "application/json");

  JsonDocument bodyDoc;
  bodyDoc["email"] = email;
  bodyDoc["password"] = accountPassword;
  String body;
  serializeJson(bodyDoc, body);

  int code = http.POST(body);
  String response = http.getString();
  http.end();

  if (code < 200 || code >= 300) {
    errorOut = "Backend verification failed (HTTP " + String(code) + ")";
    return false;
  }

  if (extractUuidFromJson(response, uuidOut)) {
    return true;
  }

  const char *keys[] = {"hashed_uuid", "uuid", "identity", "deviceId", "id"};
  for (size_t i = 0; i < (sizeof(keys) / sizeof(keys[0])); ++i) {
    String candidate;
    if (extractJsonStringFieldFallback(response, keys[i], candidate) && candidate.length() > 0) {
      uuidOut = candidate;
      return true;
    }
  }

  errorOut = "Backend response did not include a UUID";
  return false;
}

String generateAnonymousUuid() {
  randomSeed(ESP.getChipId() ^ micros() ^ millis());

  uint8_t bytes[16];
  for (size_t i = 0; i < sizeof(bytes); ++i) {
    bytes[i] = static_cast<uint8_t>(random(0, 256));
  }

  bytes[6] = (bytes[6] & 0x0F) | 0x40;
  bytes[8] = (bytes[8] & 0x3F) | 0x80;

  char out[37];
  snprintf(out,
           sizeof(out),
           "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
           bytes[0],
           bytes[1],
           bytes[2],
           bytes[3],
           bytes[4],
           bytes[5],
           bytes[6],
           bytes[7],
           bytes[8],
           bytes[9],
           bytes[10],
           bytes[11],
           bytes[12],
           bytes[13],
           bytes[14],
           bytes[15]);
  return String(out);
}

}  // namespace identity_service
