#include "device_config.h"

#include <EEPROM.h>
#include <string.h>

#include "eeprom_layout.h"

namespace device_config {
namespace {

static const uint32_t CONFIG_MAGIC = 0x44435631u;  // DCV1
static const uint16_t CONFIG_VERSION = 1;

struct EepromConfigBlob {
  uint32_t magic;
  uint16_t version;
  char ssid[33];
  char wifiPassword[65];
  char backendUrl[129];
  char deviceUuid[41];
  uint32_t checksum;
};

static uint32_t checksumBytes(const uint8_t *data, size_t length) {
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < length; ++i) {
    hash ^= data[i];
    hash *= 16777619u;
  }
  return hash;
}

static uint32_t checksumBlob(EepromConfigBlob blob) {
  blob.checksum = 0;
  return checksumBytes(reinterpret_cast<const uint8_t *>(&blob), sizeof(EepromConfigBlob));
}

static void copyToBuffer(const String &value, char *target, size_t size) {
  if (size == 0) {
    return;
  }

  memset(target, 0, size);
  size_t copyLen = value.length();
  if (copyLen >= size) {
    copyLen = size - 1;
  }

  if (copyLen > 0) {
    memcpy(target, value.c_str(), copyLen);
  }
}

static String normalizeBackendUrl(const String &url) {
  String normalized = url;
  normalized.trim();

  while (normalized.endsWith("/")) {
    normalized.remove(normalized.length() - 1);
  }

  return normalized;
}

}  // namespace

bool beginStorage() {
  EEPROM.begin(appcfg::EEPROM_TOTAL_BYTES);
  return true;
}

bool load(StoredConfig &outConfig) {
  outConfig = {};

  if (!beginStorage()) {
    return false;
  }

  EepromConfigBlob blob = {};
  EEPROM.get(appcfg::EEPROM_ADDR_DEVICE_CONFIG, blob);

  if (blob.magic != CONFIG_MAGIC || blob.version != CONFIG_VERSION) {
    return false;
  }

  uint32_t expectedChecksum = blob.checksum;
  if (expectedChecksum != checksumBlob(blob)) {
    return false;
  }

  outConfig.ssid = String(blob.ssid);
  outConfig.wifiPassword = String(blob.wifiPassword);
  outConfig.backendUrl = normalizeBackendUrl(String(blob.backendUrl));
  outConfig.deviceUuid = String(blob.deviceUuid);
  outConfig.ssid.trim();
  outConfig.wifiPassword.trim();
  outConfig.deviceUuid.trim();
  return true;
}

bool save(const StoredConfig &config) {
  if (!beginStorage()) {
    return false;
  }

  EepromConfigBlob blob = {};
  blob.magic = CONFIG_MAGIC;
  blob.version = CONFIG_VERSION;

  String ssid = config.ssid;
  String wifiPassword = config.wifiPassword;
  String backendUrl = normalizeBackendUrl(config.backendUrl);
  String deviceUuid = config.deviceUuid;

  ssid.trim();
  wifiPassword.trim();
  deviceUuid.trim();

  copyToBuffer(ssid, blob.ssid, sizeof(blob.ssid));
  copyToBuffer(wifiPassword, blob.wifiPassword, sizeof(blob.wifiPassword));
  copyToBuffer(backendUrl, blob.backendUrl, sizeof(blob.backendUrl));
  copyToBuffer(deviceUuid, blob.deviceUuid, sizeof(blob.deviceUuid));

  blob.checksum = checksumBlob(blob);
  EEPROM.put(appcfg::EEPROM_ADDR_DEVICE_CONFIG, blob);
  return EEPROM.commit();
}

bool clear() {
  if (!beginStorage()) {
    return false;
  }

  EepromConfigBlob blank = {};
  EEPROM.put(appcfg::EEPROM_ADDR_DEVICE_CONFIG, blank);
  return EEPROM.commit();
}

bool hasWiFiCredentials(const StoredConfig &config) {
  return config.ssid.length() > 0 && config.wifiPassword.length() > 0;
}

}  // namespace device_config
