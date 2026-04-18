#pragma once

#include <Arduino.h>

namespace device_config {

struct StoredConfig {
  String ssid;
  String wifiPassword;
  String backendUrl;
  String deviceUuid;
};

bool beginStorage();
bool load(StoredConfig &outConfig);
bool save(const StoredConfig &config);
bool clear();
bool hasWiFiCredentials(const StoredConfig &config);

}  // namespace device_config
