#pragma once

#include "gyro.h"

namespace gyro {

enum SensorKind : uint8_t {
  SENSOR_UNKNOWN = 0,
  SENSOR_MPU6050 = 1,
  SENSOR_MPU6500 = 2,
};

struct StoredCalibration {
  uint32_t magic;
  uint16_t version;
  uint8_t address;
  uint8_t whoAmI;
  uint8_t sensorKind;
  uint8_t reserved;
  float gyroBiasRawX;
  float gyroBiasRawY;
  float gyroBiasRawZ;
  float gyroNoiseStdDpsX;
  float gyroNoiseStdDpsY;
  float gyroNoiseStdDpsZ;
  float accelNoiseStdG;
  uint32_t checksum;
};

struct SensorInfo {
  uint8_t address;
  uint8_t whoAmI;
  SensorKind kind;
};

struct RawSample {
  int16_t axRaw;
  int16_t ayRaw;
  int16_t azRaw;
  int16_t gxRaw;
  int16_t gyRaw;
  int16_t gzRaw;
};

bool sensorSetup(SensorInfo &sensor);
bool sensorReadRaw(RawSample &sample);
bool sensorLoadStoredCalibration(StoredCalibration &stored);
bool sensorSaveStoredCalibration(StoredCalibration stored);

void fusionApplyCalibration(const StoredCalibration &stored);
void fusionResetFromAccel(float axG, float ayG, float azG);
void fusionUpdate(const RawSample &sample, float dt);
Reading fusionGetReading();

}
