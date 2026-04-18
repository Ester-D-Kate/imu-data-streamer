#include "wifi_manager.h"

#include <ArduinoJson.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>

#include "device_config.h"
#include "identity_service.h"

namespace wifi_manager {
namespace {

static const char *AP_SSID = "EyeTracker-Setup";
static const char *AP_PASS = "12345678";
static const char *MQTT_BROKER = "broker.hivemq.com";
static const uint16_t MQTT_PORT = 1883;
static const char *MDNS_HOST = "esp12e.gyro";

static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
static const uint32_t MQTT_RETRY_INTERVAL_MS = 3000;

static ESP8266WebServer configServer(80);
static WiFiClient mqttTransport;
static PubSubClient mqttClient(mqttTransport);

static device_config::StoredConfig activeConfig;
static bool apMode = false;
static bool routesInstalled = false;
static bool serverStarted = false;
static bool mdnsReady = false;

static String mqttClientId;
static String mqttGyroTopic;
static String mqttCmdTopic;
static unsigned long lastMqttAttemptMs = 0;

static String escapeJsString(const String &input) {
  String output;
  output.reserve(input.length() + 8);

  for (size_t i = 0; i < input.length(); ++i) {
    char c = input[i];
    switch (c) {
      case '\\':
        output += "\\\\";
        break;
      case '"':
        output += "\\\"";
        break;
      case '\n':
        output += "\\n";
        break;
      case '\r':
        output += "\\r";
        break;
      case '\t':
        output += "\\t";
        break;
      default:
        output += c;
        break;
    }
  }

  return output;
}

static void rebuildRouting() {
  String uuid = activeConfig.deviceUuid;
  uuid.trim();
  if (uuid.length() == 0) {
    uuid = "anonymous";
  }

  mqttGyroTopic = "eyetracker/" + uuid + "/gyro";
  mqttCmdTopic = "eyetracker/" + uuid + "/cmd";

  String compact = uuid;
  compact.replace("-", "");
  mqttClientId = "imu-" + compact;
  if (mqttClientId.length() > 20) {
    mqttClientId = mqttClientId.substring(0, 20);
  }
  mqttClientId += "-" + String(ESP.getChipId(), HEX);
}

static bool ensureUuid() {
  if (activeConfig.deviceUuid.length() > 0) {
    return true;
  }

  activeConfig.deviceUuid = identity_service::generateAnonymousUuid();
  bool saved = device_config::save(activeConfig);
  if (!saved) {
    Serial.println("[net] Failed to save anonymous UUID to EEPROM");
  }
  return saved;
}

static bool connectToWiFiBlocking(const String &ssid,
                                  const String &wifiPassword,
                                  bool keepApEnabled,
                                  unsigned long timeoutMs) {
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.mode(keepApEnabled ? WIFI_AP_STA : WIFI_STA);
  WiFi.begin(ssid.c_str(), wifiPassword.c_str());

  unsigned long startMs = millis();
  while (millis() - startMs < timeoutMs) {
    wl_status_t status = WiFi.status();
    if (status == WL_CONNECTED) {
      return true;
    }
    delay(250);
  }

  return WiFi.status() == WL_CONNECTED;
}

static void sendJsonResponse(int code, bool success, const String &message) {
  JsonDocument doc;
  doc["success"] = success;
  doc["message"] = message;
  String json;
  serializeJson(doc, json);
  configServer.send(code, "application/json", json);
}

static void startMdns() {
  if (mdnsReady || WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (MDNS.begin(MDNS_HOST)) {
    MDNS.addService("http", "tcp", 80);
    mdnsReady = true;
    Serial.print("[net] mDNS ready: http://");
    Serial.print(MDNS_HOST);
    Serial.println(".local/credentials");
  } else {
    Serial.println("[net] mDNS start failed");
  }
}

static void stopMdns() {
  if (!mdnsReady) {
    return;
  }

  MDNS.close();
  mdnsReady = false;
}

static void connectMqttIfNeeded() {
  if (apMode || WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (mqttClient.connected()) {
    return;
  }

  if (millis() - lastMqttAttemptMs < MQTT_RETRY_INTERVAL_MS) {
    return;
  }

  lastMqttAttemptMs = millis();
  if (mqttClientId.length() == 0 || mqttGyroTopic.length() == 0) {
    rebuildRouting();
  }

  Serial.print("[net] MQTT connecting as ");
  Serial.println(mqttClientId);
  if (mqttClient.connect(mqttClientId.c_str())) {
    mqttClient.subscribe(mqttCmdTopic.c_str());
    Serial.print("[net] MQTT connected, publish topic: ");
    Serial.println(mqttGyroTopic);
  } else {
    Serial.print("[net] MQTT connect failed, state=");
    Serial.println(mqttClient.state());
  }
}

static void handleCredentialsPage() {
  String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP12 Credentials</title>
  <style>
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Segoe UI", Tahoma, sans-serif;
      min-height: 100vh;
      background: linear-gradient(135deg, #0b3c5d, #328cc1);
      color: #102a43;
      padding: 18px;
    }
    .panel {
      max-width: 560px;
      margin: 0 auto;
      background: #ffffff;
      border-radius: 14px;
      padding: 22px;
      box-shadow: 0 12px 28px rgba(0, 0, 0, 0.18);
    }
    h1 { margin: 0 0 12px; color: #0b3c5d; font-size: 24px; }
    p.meta { margin: 0 0 14px; color: #486581; font-size: 13px; }
    label { display: block; font-size: 13px; margin: 10px 0 4px; color: #243b53; }
    input {
      width: 100%;
      padding: 11px;
      border: 1px solid #bcccdc;
      border-radius: 8px;
      font-size: 14px;
      outline: none;
    }
    input:focus { border-color: #328cc1; }
    .row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }
    .buttons {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-top: 14px;
    }
    button {
      padding: 11px;
      border: none;
      border-radius: 8px;
      font-size: 14px;
      font-weight: 600;
      cursor: pointer;
    }
    .primary { background: #0b3c5d; color: #fff; }
    .warn { background: #d64545; color: #fff; }
    .scan { margin-top: 10px; background: #328cc1; color: #fff; width: 100%; }
    #status {
      margin-top: 12px;
      padding: 10px;
      border-radius: 8px;
      background: #f0f4f8;
      color: #243b53;
      font-size: 13px;
      min-height: 18px;
    }
    .nets { margin-top: 10px; }
    .net {
      border: 1px solid #d9e2ec;
      border-radius: 8px;
      padding: 9px;
      margin-bottom: 6px;
      cursor: pointer;
      display: flex;
      justify-content: space-between;
      color: #102a43;
    }
    .net:hover { border-color: #328cc1; background: #f7fbff; }
  </style>
</head>
<body>
  <div class="panel">
    <h1>ESP12 Credential Portal</h1>
    <p class="meta">AP URL: / (192.168.4.1) | LAN URL: /credentials (esp12e.gyro.local) | Current UUID: <span id="uuidLabel"></span></p>

    <label for="ssid">Wi-Fi SSID (required)</label>
    <input id="ssid" placeholder="Wi-Fi network name">

    <label for="wifiPassword">Wi-Fi Password (required)</label>
    <input id="wifiPassword" type="password" placeholder="Wi-Fi password">

    <button class="scan" onclick="scanNetworks()">Scan Networks</button>
    <div id="networks" class="nets"></div>

    <label for="backendUrl">Backend URL (optional unless account fields are used)</label>
    <input id="backendUrl" placeholder="https://server.example.com or full /api/gaze/validate/uuid URL">

    <div class="row">
      <div>
        <label for="email">Account Email (optional)</label>
        <input id="email" placeholder="user@example.com">
      </div>
      <div>
        <label for="accountPassword">Account Password (optional)</label>
        <input id="accountPassword" type="password" placeholder="Account password">
      </div>
    </div>

    <div class="buttons">
      <button class="primary" onclick="submitCredentials()">Save and Apply</button>
      <button class="warn" onclick="clearCredentials()">Delete Stored Data</button>
    </div>

    <div id="status"></div>
  </div>

  <script>
    const savedSsid = "__SAVED_SSID__";
    const savedBackend = "__SAVED_BACKEND__";
    const currentUuid = "__CURRENT_UUID__";

    function setStatus(msg) {
      document.getElementById('status').textContent = msg;
    }

    document.getElementById('ssid').value = savedSsid;
    document.getElementById('backendUrl').value = savedBackend;
    document.getElementById('uuidLabel').textContent = currentUuid || '(not set)';
    setStatus('Wi-Fi SSID and Wi-Fi password are required. Email/password trigger backend UUID verification.');

    async function scanNetworks() {
      setStatus('Scanning networks...');
      try {
        const response = await fetch('/scan');
        const list = await response.json();
        const host = document.getElementById('networks');
        host.innerHTML = '';
        list.forEach((item) => {
          const row = document.createElement('div');
          row.className = 'net';
          row.innerHTML = '<span>' + item.ssid + '</span><span>' + item.rssi + ' dBm</span>';
          row.onclick = () => {
            document.getElementById('ssid').value = item.ssid;
          };
          host.appendChild(row);
        });
        setStatus('Scan complete: ' + list.length + ' networks.');
      } catch (e) {
        setStatus('Network scan failed.');
      }
    }

    async function submitCredentials() {
      const ssid = document.getElementById('ssid').value.trim();
      const wifiPassword = document.getElementById('wifiPassword').value;
      const backendUrl = document.getElementById('backendUrl').value.trim();
      const email = document.getElementById('email').value.trim();
      const accountPassword = document.getElementById('accountPassword').value;

      if (!ssid || !wifiPassword) {
        setStatus('Wi-Fi SSID and Wi-Fi password are mandatory.');
        return;
      }

      const hasEmail = email.length > 0;
      const hasAccountPassword = accountPassword.length > 0;
      if ((hasEmail && !hasAccountPassword) || (!hasEmail && hasAccountPassword)) {
        setStatus('Provide both account email and account password, or leave both empty.');
        return;
      }

      if ((hasEmail || hasAccountPassword) && !backendUrl) {
        setStatus('Backend URL is required when account email/password are provided.');
        return;
      }

      setStatus('Applying credentials. The device may restart.');

      const url = '/connect?ssid=' + encodeURIComponent(ssid)
                + '&wifipass=' + encodeURIComponent(wifiPassword)
                + '&backendUrl=' + encodeURIComponent(backendUrl)
                + '&email=' + encodeURIComponent(email)
                + '&accpass=' + encodeURIComponent(accountPassword);

      try {
        const response = await fetch(url);
        const data = await response.json();
        if (data.success) {
          setStatus('Saved. UUID: ' + (data.uuid || '(none)') + '. Restarting...');
        } else {
          setStatus(data.message || 'Save failed.');
        }
      } catch (e) {
        setStatus('Request failed.');
      }
    }

    async function clearCredentials() {
      if (!confirm('Delete stored Wi-Fi/backend/UUID credentials?')) {
        return;
      }

      try {
        await fetch('/clear');
        setStatus('Stored credentials cleared. Restarting...');
      } catch (e) {
        setStatus('Clear request failed.');
      }
    }
  </script>
</body>
</html>
)rawliteral";

  page.replace("__SAVED_SSID__", escapeJsString(activeConfig.ssid));
  page.replace("__SAVED_BACKEND__", escapeJsString(activeConfig.backendUrl));
  page.replace("__CURRENT_UUID__", escapeJsString(activeConfig.deviceUuid));

  configServer.send(200, "text/html", page);
}

static void handleScan() {
  JsonDocument doc;
  JsonArray list = doc.to<JsonArray>();

  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; ++i) {
    JsonObject net = list.add<JsonObject>();
    net["ssid"] = WiFi.SSID(i);
    net["rssi"] = WiFi.RSSI(i);
  }

  String json;
  serializeJson(list, json);
  configServer.send(200, "application/json", json);
  WiFi.scanDelete();
}

static void handleConnect() {
  String ssid = configServer.arg("ssid");
  String wifiPassword = configServer.arg("wifipass");
  String backendUrl = identity_service::normalizeBackendUrl(configServer.arg("backendUrl"));
  String email = configServer.arg("email");
  String accountPassword = configServer.arg("accpass");

  ssid.trim();
  wifiPassword.trim();
  email.trim();

  if (ssid.length() == 0 || wifiPassword.length() == 0) {
    sendJsonResponse(200, false, "Wi-Fi SSID and Wi-Fi password are required");
    return;
  }

  bool hasEmail = email.length() > 0;
  bool hasAccountPassword = accountPassword.length() > 0;

  if ((hasEmail && !hasAccountPassword) || (!hasEmail && hasAccountPassword)) {
    sendJsonResponse(200, false, "Provide both account email and account password, or neither");
    return;
  }

  if ((hasEmail || hasAccountPassword) && backendUrl.length() == 0) {
    sendJsonResponse(200, false, "Backend URL is required when account credentials are used");
    return;
  }

  Serial.println("[net] Testing Wi-Fi credentials from portal...");
  if (!connectToWiFiBlocking(ssid, wifiPassword, true, WIFI_CONNECT_TIMEOUT_MS)) {
    sendJsonResponse(200, false, "Wi-Fi connection failed");
    return;
  }

  String uuidToStore = activeConfig.deviceUuid;
  bool anonymousMode = false;

  if (hasEmail && hasAccountPassword) {
    String backendUuid;
    String verifyError;
    if (!identity_service::requestUuidFromBackend(backendUrl, email, accountPassword, backendUuid, verifyError)) {
      sendJsonResponse(200, false, verifyError);
      return;
    }
    uuidToStore = backendUuid;
  } else {
    if (uuidToStore.length() == 0) {
      uuidToStore = identity_service::generateAnonymousUuid();
    }
    if (backendUrl.length() == 0) {
      anonymousMode = true;
    }
  }

  if (uuidToStore.length() == 0) {
    sendJsonResponse(200, false, "UUID is missing and could not be generated");
    return;
  }

  device_config::StoredConfig nextConfig;
  nextConfig.ssid = ssid;
  nextConfig.wifiPassword = wifiPassword;
  nextConfig.backendUrl = backendUrl;
  nextConfig.deviceUuid = uuidToStore;

  if (!device_config::save(nextConfig)) {
    sendJsonResponse(200, false, "Failed to write credentials to EEPROM");
    return;
  }

  activeConfig = nextConfig;
  rebuildRouting();

  JsonDocument responseDoc;
  responseDoc["success"] = true;
  responseDoc["message"] = "Credentials saved";
  responseDoc["uuid"] = activeConfig.deviceUuid;
  responseDoc["anonymous"] = anonymousMode;
  String responseJson;
  serializeJson(responseDoc, responseJson);
  configServer.send(200, "application/json", responseJson);

  delay(800);
  ESP.restart();
}

static void handleClear() {
  if (!device_config::clear()) {
    sendJsonResponse(500, false, "Failed to clear EEPROM credentials");
    return;
  }

  sendJsonResponse(200, true, "Stored credentials deleted");
  delay(800);
  ESP.restart();
}

static void handleRoot() {
  if (apMode) {
    handleCredentialsPage();
    return;
  }

  configServer.sendHeader("Location", "/credentials", true);
  configServer.send(302, "text/plain", "Redirecting to /credentials");
}

static void handleCredentialsAlias() {
  if (apMode) {
    configServer.sendHeader("Location", "/", true);
    configServer.send(302, "text/plain", "Redirecting to /");
    return;
  }

  handleCredentialsPage();
}

static void installRoutesIfNeeded() {
  if (routesInstalled) {
    return;
  }

  configServer.on("/", HTTP_GET, handleRoot);
  configServer.on("/credentials", HTTP_GET, handleCredentialsAlias);
  configServer.on("/scan", HTTP_GET, handleScan);
  configServer.on("/connect", HTTP_GET, handleConnect);
  configServer.on("/clear", HTTP_GET, handleClear);

  configServer.on("/generate_204", HTTP_GET, handleRoot);
  configServer.on("/hotspot-detect.html", HTTP_GET, handleRoot);
  configServer.on("/ncsi.txt", HTTP_GET, handleRoot);

  configServer.onNotFound(handleRoot);
  routesInstalled = true;
}

static void startWebServerIfNeeded() {
  installRoutesIfNeeded();
  if (serverStarted) {
    return;
  }

  configServer.begin();
  serverStarted = true;
  Serial.println("[net] Credential web server started on port 80");
}

static void startConfigApMode() {
  apMode = true;
  mqttClient.disconnect();
  stopMdns();

  WiFi.disconnect();
  delay(100);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  startWebServerIfNeeded();
  Serial.print("[net] AP mode active at ");
  Serial.println(WiFi.softAPIP());
  Serial.println("[net] Open http://192.168.4.1");
}

static bool startStationModeFromStoredConfig() {
  if (!device_config::hasWiFiCredentials(activeConfig)) {
    Serial.println("[net] Missing stored Wi-Fi credentials");
    return false;
  }

  Serial.print("[net] Connecting to Wi-Fi SSID: ");
  Serial.println(activeConfig.ssid);

  if (!connectToWiFiBlocking(activeConfig.ssid, activeConfig.wifiPassword, false, WIFI_CONNECT_TIMEOUT_MS)) {
    Serial.println("[net] Wi-Fi connection failed, switching to AP mode");
    return false;
  }

  if (!ensureUuid()) {
    Serial.println("[net] Failed to ensure UUID");
    return false;
  }

  apMode = false;
  rebuildRouting();

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  startWebServerIfNeeded();
  startMdns();

  Serial.print("[net] Wi-Fi connected, IP: ");
  Serial.println(WiFi.localIP());
  return true;
}

}  // namespace

void begin() {
  activeConfig = {};
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);

  if (!device_config::load(activeConfig)) {
    Serial.println("[net] No valid stored device config in EEPROM");
    startConfigApMode();
    return;
  }

  activeConfig.backendUrl = identity_service::normalizeBackendUrl(activeConfig.backendUrl);
  if (!startStationModeFromStoredConfig()) {
    startConfigApMode();
  }
}

void loop() {
  if (serverStarted) {
    configServer.handleClient();
  }

  if (mdnsReady) {
    MDNS.update();
  }

  if (apMode) {
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[net] Wi-Fi lost, entering AP mode");
    startConfigApMode();
    return;
  }

  startMdns();
  connectMqttIfNeeded();
  mqttClient.loop();
}

bool publishGyro(const gyro::Reading &reading) {
  if (apMode || WiFi.status() != WL_CONNECTED) {
    return false;
  }

  if (!mqttClient.connected()) {
    connectMqttIfNeeded();
    return false;
  }

  char payload[48];
  snprintf(payload,
           sizeof(payload),
           "%.2f,%.2f,%.2f",
           reading.angleX,
           reading.angleY,
           reading.angleZ);
  return mqttClient.publish(mqttGyroTopic.c_str(), payload);
}

bool isConnected() {
  return WiFi.status() == WL_CONNECTED;
}

String getGyroTopic() {
  return mqttGyroTopic;
}

String getDeviceUuid() {
  return activeConfig.deviceUuid;
}

}  // namespace wifi_manager
