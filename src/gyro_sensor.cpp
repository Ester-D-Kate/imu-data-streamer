#include "gyro_internal.h"

#include <EEPROM.h>
#include <Wire.h>

namespace gyro {
namespace {

static const uint8_t MPU_ADDR_CANDIDATES[] = {0x68, 0x69};
static const uint8_t SDA_PIN = 4;
static const uint8_t SCL_PIN = 5;

static const uint8_t REG_WHO_AM_I = 0x75;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

static const uint8_t WHO_AM_I_MPU6050 = 0x68;
static const uint8_t WHO_AM_I_MPU6500 = 0x70;

static const uint32_t CALIBRATION_MAGIC = 0x4759524Fu;
static const uint16_t CALIBRATION_VERSION = 2;
static const size_t EEPROM_SIZE = 512;

static uint8_t mpuAddr = 0xFF;
static SensorInfo currentSensor = {0xFF, 0x00, SENSOR_UNKNOWN};

static uint32_t checksumBytes(const uint8_t *data, size_t length) {
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < length; i++) {
    hash ^= data[i];
    hash *= 16777619u;
  }
  return hash;
}

static SensorKind sensorKindFromWhoAmI(uint8_t whoAmI) {
  if (whoAmI == WHO_AM_I_MPU6050) {
    return SENSOR_MPU6050;
  }
  if (whoAmI == WHO_AM_I_MPU6500) {
    return SENSOR_MPU6500;
  }
  return SENSOR_UNKNOWN;
}

static bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool readWhoAmI(uint8_t address, uint8_t &whoAmI) {
  Wire.beginTransmission(address);
  Wire.write(REG_WHO_AM_I);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom((int)address, 1, (int)true) != 1) {
    return false;
  }

  whoAmI = Wire.read();
  return true;
}

static bool detectSensor(SensorInfo &sensor) {
  for (uint8_t addr : MPU_ADDR_CANDIDATES) {
    uint8_t whoAmI = 0;
    if (!readWhoAmI(addr, whoAmI)) {
      continue;
    }

    SensorKind kind = sensorKindFromWhoAmI(whoAmI);
    if (kind == SENSOR_UNKNOWN) {
      continue;
    }

    sensor.address = addr;
    sensor.whoAmI = whoAmI;
    sensor.kind = kind;
    return true;
  }

  return false;
}

static bool configureSensor() {
  if (!writeReg(REG_PWR_MGMT_1, 0x00)) {
    return false;
  }

  if (!writeReg(REG_GYRO_CONFIG, 0x08)) {
    return false;
  }

  if (!writeReg(REG_ACCEL_CONFIG, 0x00)) {
    return false;
  }

  return true;
}

static bool readRawInternal(RawSample &sample) {
  uint8_t buffer[14];
  Wire.beginTransmission(mpuAddr);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom((int)mpuAddr, 14, (int)true) != 14) {
    return false;
  }

  for (uint8_t i = 0; i < 14; i++) {
    buffer[i] = Wire.read();
  }

  sample.axRaw = (int16_t)((buffer[0] << 8) | buffer[1]);
  sample.ayRaw = (int16_t)((buffer[2] << 8) | buffer[3]);
  sample.azRaw = (int16_t)((buffer[4] << 8) | buffer[5]);
  sample.gxRaw = (int16_t)((buffer[8] << 8) | buffer[9]);
  sample.gyRaw = (int16_t)((buffer[10] << 8) | buffer[11]);
  sample.gzRaw = (int16_t)((buffer[12] << 8) | buffer[13]);
  return true;
}

}  // namespace

bool sensorSetup(SensorInfo &sensor) {
  EEPROM.begin(EEPROM_SIZE);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(100);

  mpuAddr = 0xFF;
  currentSensor = {0xFF, 0x00, SENSOR_UNKNOWN};

  for (uint8_t attempt = 0; attempt < 5; attempt++) {
    if (detectSensor(currentSensor)) {
      mpuAddr = currentSensor.address;
      sensor = currentSensor;
      return configureSensor();
    }
    delay(200);
  }

  sensor = currentSensor;
  return false;
}

bool sensorReadRaw(RawSample &sample) {
  if (mpuAddr == 0xFF) {
    return false;
  }

  return readRawInternal(sample);
}

bool sensorLoadStoredCalibration(StoredCalibration &stored) {
  EEPROM.get(0, stored);
  if (stored.magic != CALIBRATION_MAGIC || stored.version != CALIBRATION_VERSION) {
    return false;
  }

  uint32_t expectedChecksum = stored.checksum;
  stored.checksum = 0;
  uint32_t actualChecksum = checksumBytes(reinterpret_cast<const uint8_t *>(&stored), sizeof(StoredCalibration) - sizeof(uint32_t));
  stored.checksum = expectedChecksum;
  return expectedChecksum == actualChecksum;
}

bool sensorSaveStoredCalibration(StoredCalibration stored) {
  stored.magic = CALIBRATION_MAGIC;
  stored.version = CALIBRATION_VERSION;
  stored.reserved = 0;
  stored.checksum = 0;
  stored.checksum = checksumBytes(reinterpret_cast<const uint8_t *>(&stored), sizeof(StoredCalibration) - sizeof(uint32_t));
  EEPROM.put(0, stored);
  return EEPROM.commit();
}

}  // namespace gyro
