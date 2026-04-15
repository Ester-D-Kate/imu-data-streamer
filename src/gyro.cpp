#include "gyro_internal.h"

#include <math.h>

namespace gyro {
namespace {

static const uint8_t CAL_BUTTON_PIN = D3;
static const uint32_t CALIBRATION_DURATION_MS = 5000;
static const uint16_t CALIBRATION_SAMPLE_DELAY_MS = 5;
static const float ACCEL_SCALE = 16384.0f;
static const float GYRO_SCALE = 65.5f;

static SensorInfo currentSensor = {0xFF, 0x00, SENSOR_UNKNOWN};
static bool ready = false;
static unsigned long lastMs = 0;
static bool lastButtonState = HIGH;

static bool calibrateAndStore() {
  ready = false;

  float sumGx = 0.0f, sumGy = 0.0f, sumGz = 0.0f;
  float sumGxSq = 0.0f, sumGySq = 0.0f, sumGzSq = 0.0f;
  float sumAx = 0.0f, sumAy = 0.0f, sumAz = 0.0f;
  float sumAxSq = 0.0f, sumAySq = 0.0f, sumAzSq = 0.0f;
  uint32_t samples = 0;
  unsigned long startMs = millis();

  while (millis() - startMs < CALIBRATION_DURATION_MS) {
    RawSample sample = {};
    if (sensorReadRaw(sample)) {
      float axG = (float)sample.axRaw / ACCEL_SCALE;
      float ayG = (float)sample.ayRaw / ACCEL_SCALE;
      float azG = (float)sample.azRaw / ACCEL_SCALE;

      sumGx += (float)sample.gxRaw;
      sumGy += (float)sample.gyRaw;
      sumGz += (float)sample.gzRaw;
      sumGxSq += (float)sample.gxRaw * (float)sample.gxRaw;
      sumGySq += (float)sample.gyRaw * (float)sample.gyRaw;
      sumGzSq += (float)sample.gzRaw * (float)sample.gzRaw;

      sumAx += axG;
      sumAy += ayG;
      sumAz += azG;
      sumAxSq += axG * axG;
      sumAySq += ayG * ayG;
      sumAzSq += azG * azG;

      samples++;
    }

    delay(CALIBRATION_SAMPLE_DELAY_MS);
  }

  if (samples == 0) {
    return false;
  }

  float count = (float)samples;
  float gyroMeanX = sumGx / count;
  float gyroMeanY = sumGy / count;
  float gyroMeanZ = sumGz / count;

  float gyroVarX = (sumGxSq / count) - (gyroMeanX * gyroMeanX);
  float gyroVarY = (sumGySq / count) - (gyroMeanY * gyroMeanY);
  float gyroVarZ = (sumGzSq / count) - (gyroMeanZ * gyroMeanZ);

  float accelMeanX = sumAx / count;
  float accelMeanY = sumAy / count;
  float accelMeanZ = sumAz / count;

  float accelVarX = (sumAxSq / count) - (accelMeanX * accelMeanX);
  float accelVarY = (sumAySq / count) - (accelMeanY * accelMeanY);
  float accelVarZ = (sumAzSq / count) - (accelMeanZ * accelMeanZ);

  if (gyroVarX < 0.0f) gyroVarX = 0.0f;
  if (gyroVarY < 0.0f) gyroVarY = 0.0f;
  if (gyroVarZ < 0.0f) gyroVarZ = 0.0f;
  if (accelVarX < 0.0f) accelVarX = 0.0f;
  if (accelVarY < 0.0f) accelVarY = 0.0f;
  if (accelVarZ < 0.0f) accelVarZ = 0.0f;

  StoredCalibration stored = {};
  stored.address = currentSensor.address;
  stored.whoAmI = currentSensor.whoAmI;
  stored.sensorKind = static_cast<uint8_t>(currentSensor.kind);
  stored.gyroBiasRawX = gyroMeanX;
  stored.gyroBiasRawY = gyroMeanY;
  stored.gyroBiasRawZ = gyroMeanZ;
  stored.gyroNoiseStdDpsX = sqrtf(gyroVarX) / GYRO_SCALE;
  stored.gyroNoiseStdDpsY = sqrtf(gyroVarY) / GYRO_SCALE;
  stored.gyroNoiseStdDpsZ = sqrtf(gyroVarZ) / GYRO_SCALE;
  stored.accelNoiseStdG = sqrtf(accelVarX + accelVarY + accelVarZ);

  if (!sensorSaveStoredCalibration(stored)) {
    return false;
  }

  fusionApplyCalibration(stored);
  fusionResetFromAccel(accelMeanX, accelMeanY, accelMeanZ);
  lastMs = millis();
  ready = true;
  return true;
}

static bool maybeHandleManualCalibration() {
  bool buttonState = digitalRead(CAL_BUTTON_PIN);
  bool shouldCalibrate = (lastButtonState == HIGH && buttonState == LOW);
  lastButtonState = buttonState;

  if (!shouldCalibrate) {
    return false;
  }

  return calibrateAndStore();
}

}  // namespace

void begin() {
  pinMode(CAL_BUTTON_PIN, INPUT_PULLUP);
  lastButtonState = digitalRead(CAL_BUTTON_PIN);

  ready = false;
  currentSensor = {0xFF, 0x00, SENSOR_UNKNOWN};

  if (!sensorSetup(currentSensor)) {
    return;
  }

  StoredCalibration stored = {};
  bool hasStoredCalibration = sensorLoadStoredCalibration(stored);
  bool calibrationMatchesSensor = hasStoredCalibration &&
                                 stored.address == currentSensor.address &&
                                 stored.whoAmI == currentSensor.whoAmI;

  if (calibrationMatchesSensor) {
    fusionApplyCalibration(stored);

    RawSample sample = {};
    if (!sensorReadRaw(sample)) {
      return;
    }

    fusionResetFromAccel((float)sample.axRaw / ACCEL_SCALE,
                         (float)sample.ayRaw / ACCEL_SCALE,
                         (float)sample.azRaw / ACCEL_SCALE);
    lastMs = millis();
    ready = true;
    return;
  }

  if (!calibrateAndStore()) {
    return;
  }
}

bool update() {
  if (!ready) {
    return false;
  }

  if (maybeHandleManualCalibration()) {
    return false;
  }

  RawSample sample = {};
  if (!sensorReadRaw(sample)) {
    ready = false;
    return false;
  }

  unsigned long nowMs = millis();
  float dt = (nowMs - lastMs) / 1000.0f;
  lastMs = nowMs;

  fusionUpdate(sample, dt);
  return true;
}

bool isReady() {
  return ready;
}

Reading getReading() {
  return fusionGetReading();
}

}  // namespace gyro
