#include "gyro_internal.h"

#include <math.h>

namespace gyro {
namespace {

static const float GYRO_SCALE = 65.5f;
static const float ACCEL_SCALE = 16384.0f;
static const float GYRO_RATE_DEADBAND_DPS_MIN = 0.03f;
static const float GYRO_RATE_DEADBAND_DPS_MAX = 0.08f;
static const float ACCEL_TRUST_TOLERANCE_G_MIN = 0.15f;
static const float ACCEL_TRUST_TOLERANCE_G_MAX = 0.45f;
static const float FUSION_GAIN_MIN = 1.6f;
static const float FUSION_GAIN_MAX = 3.0f;
static const float RATE_LIMIT_DT_S = 0.25f;
static const float YAW_RATE_DEADBAND_DPS = 0.05f;
static const float YAW_STEP_DEADBAND_DEG = 0.05f;
static const float YAW_OUTPUT_DEADZONE_DEG = 0.50f;
static const float BIAS_LEARN_ACCEL_TOL_G = 0.08f;
static const float BIAS_LEARN_GYRO_LIMIT_DPS = 1.2f;
static const float BIAS_ADAPT_GAIN_PER_SEC = 0.05f;

static float gyroBiasRawX = 0.0f;
static float gyroBiasRawY = 0.0f;
static float gyroBiasRawZ = 0.0f;
static float gyroNoiseStdDpsX = 0.10f;
static float gyroNoiseStdDpsY = 0.10f;
static float gyroNoiseStdDpsZ = 0.10f;
static float accelNoiseStdG = 0.02f;
static float dynamicGyroBiasRawX = 0.0f;
static float dynamicGyroBiasRawY = 0.0f;
static float dynamicGyroBiasRawZ = 0.0f;

static float gyroRateDeadbandDps = 0.05f;
static float accelTrustToleranceG = 0.25f;
static float fusionGain = 2.2f;

static float gyroXRateDps = 0.0f;
static float gyroYRateDps = 0.0f;
static float gyroZRateDps = 0.0f;

static float quatW = 1.0f;
static float quatX = 0.0f;
static float quatY = 0.0f;
static float quatZ = 0.0f;

static float displayRollDeg = 0.0f;
static float displayPitchDeg = 0.0f;
static float displayYawDeg = 0.0f;

static float rollZeroDeg = 0.0f;
static float pitchZeroDeg = 0.0f;
static float yawZeroDeg = 0.0f;

static bool displayInitialized = false;

static float clampFloat(float value, float lower, float upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

static float wrapAngle180(float angleDeg) {
  while (angleDeg > 180.0f) {
    angleDeg -= 360.0f;
  }
  while (angleDeg < -180.0f) {
    angleDeg += 360.0f;
  }
  return angleDeg;
}

static void refreshFusionTuning() {
  float gyroNoiseMeanDps = (gyroNoiseStdDpsX + gyroNoiseStdDpsY + gyroNoiseStdDpsZ) / 3.0f;
  gyroRateDeadbandDps = clampFloat(gyroNoiseMeanDps * 1.5f, GYRO_RATE_DEADBAND_DPS_MIN, GYRO_RATE_DEADBAND_DPS_MAX);
  accelTrustToleranceG = clampFloat(accelNoiseStdG * 10.0f + 0.10f, ACCEL_TRUST_TOLERANCE_G_MIN, ACCEL_TRUST_TOLERANCE_G_MAX);
  fusionGain = clampFloat(2.8f - accelNoiseStdG * 2.0f, FUSION_GAIN_MIN, FUSION_GAIN_MAX);
}

static void normalizeQuaternion() {
  float norm = sqrtf(quatW * quatW + quatX * quatX + quatY * quatY + quatZ * quatZ);
  if (norm <= 0.0f) {
    quatW = 1.0f;
    quatX = 0.0f;
    quatY = 0.0f;
    quatZ = 0.0f;
    return;
  }

  float invNorm = 1.0f / norm;
  quatW *= invNorm;
  quatX *= invNorm;
  quatY *= invNorm;
  quatZ *= invNorm;
}

static void setQuaternionFromEulerRad(float rollRad, float pitchRad, float yawRad) {
  float halfRoll = rollRad * 0.5f;
  float halfPitch = pitchRad * 0.5f;
  float halfYaw = yawRad * 0.5f;

  float cr = cosf(halfRoll);
  float sr = sinf(halfRoll);
  float cp = cosf(halfPitch);
  float sp = sinf(halfPitch);
  float cy = cosf(halfYaw);
  float sy = sinf(halfYaw);

  quatW = cr * cp * cy + sr * sp * sy;
  quatX = sr * cp * cy - cr * sp * sy;
  quatY = cr * sp * cy + sr * cp * sy;
  quatZ = cr * cp * sy - sr * sp * cy;
  normalizeQuaternion();
}

static void quaternionToFusedDeg(float &rollDeg, float &pitchDeg, float &yawDeg) {
  float fusedRollSin = 2.0f * (quatW * quatX + quatY * quatZ);
  float fusedPitchSin = 2.0f * (quatW * quatY - quatX * quatZ);
  fusedRollSin = clampFloat(fusedRollSin, -1.0f, 1.0f);
  fusedPitchSin = clampFloat(fusedPitchSin, -1.0f, 1.0f);

  rollDeg = asinf(fusedRollSin) * 180.0f / PI;
  pitchDeg = asinf(fusedPitchSin) * 180.0f / PI;

  float yawNum = 2.0f * (quatW * quatZ + quatX * quatY);
  float yawDen = quatW * quatW + quatX * quatX - quatY * quatY - quatZ * quatZ;
  yawDeg = atan2f(yawNum, yawDen) * 180.0f / PI;
}

static void updateOrientation(float axG, float ayG, float azG,
                              float gxDps, float gyDps, float gzDps,
                              float dt) {
  float accelMagG = sqrtf(axG * axG + ayG * ayG + azG * azG);
  float accelTrust = 0.0f;
  if (accelMagG > 0.0001f) {
    float invAccelMag = 1.0f / accelMagG;
    axG *= invAccelMag;
    ayG *= invAccelMag;
    azG *= invAccelMag;

    accelTrust = 1.0f - fabsf(accelMagG - 1.0f) / accelTrustToleranceG;
    accelTrust = clampFloat(accelTrust, 0.0f, 1.0f);
  }

  float gxRad = gxDps * (PI / 180.0f);
  float gyRad = gyDps * (PI / 180.0f);
  float gzRad = gzDps * (PI / 180.0f);

  if (accelTrust > 0.0f) {
    float vx = 2.0f * (quatX * quatZ - quatW * quatY);
    float vy = 2.0f * (quatW * quatX + quatY * quatZ);
    float vz = quatW * quatW - quatX * quatX - quatY * quatY + quatZ * quatZ;

    float ex = ayG * vz - azG * vy;
    float ey = azG * vx - axG * vz;
    float ez = axG * vy - ayG * vx;

    float gain = fusionGain * accelTrust;
    gxRad += gain * ex;
    gyRad += gain * ey;
    gzRad += gain * ez;
  }

  float halfDt = 0.5f * dt;
  float currentW = quatW;
  float currentX = quatX;
  float currentY = quatY;
  float currentZ = quatZ;

  quatW += (-currentX * gxRad - currentY * gyRad - currentZ * gzRad) * halfDt;
  quatX += ( currentW * gxRad + currentY * gzRad - currentZ * gyRad) * halfDt;
  quatY += ( currentW * gyRad - currentX * gzRad + currentZ * gxRad) * halfDt;
  quatZ += ( currentW * gzRad + currentX * gyRad - currentY * gxRad) * halfDt;

  normalizeQuaternion();
}

static void updateDynamicGyroBias(float gxDps, float gyDps, float gzDps,
                                  float accelMagG, float dt) {
  if (fabsf(accelMagG - 1.0f) > BIAS_LEARN_ACCEL_TOL_G) {
    return;
  }

  if (fabsf(gxDps) > BIAS_LEARN_GYRO_LIMIT_DPS ||
      fabsf(gyDps) > BIAS_LEARN_GYRO_LIMIT_DPS ||
      fabsf(gzDps) > BIAS_LEARN_GYRO_LIMIT_DPS) {
    return;
  }

  float gain = clampFloat(dt * BIAS_ADAPT_GAIN_PER_SEC, 0.0f, 0.02f);
  if (gain <= 0.0f) {
    return;
  }

  dynamicGyroBiasRawX += (gxDps * GYRO_SCALE) * gain;
  dynamicGyroBiasRawY += (gyDps * GYRO_SCALE) * gain;
}

}  // namespace

void fusionApplyCalibration(const StoredCalibration &stored) {
  gyroBiasRawX = stored.gyroBiasRawX;
  gyroBiasRawY = stored.gyroBiasRawY;
  gyroBiasRawZ = stored.gyroBiasRawZ;
  gyroNoiseStdDpsX = stored.gyroNoiseStdDpsX;
  gyroNoiseStdDpsY = stored.gyroNoiseStdDpsY;
  gyroNoiseStdDpsZ = stored.gyroNoiseStdDpsZ;
  accelNoiseStdG = stored.accelNoiseStdG;
  refreshFusionTuning();
}

void fusionResetFromAccel(float axG, float ayG, float azG) {
  float rollRad = atan2f(ayG, azG);
  float pitchRad = atan2f(-axG, sqrtf(ayG * ayG + azG * azG));
  setQuaternionFromEulerRad(rollRad, pitchRad, 0.0f);

  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;
  float yawDeg = 0.0f;
  quaternionToFusedDeg(rollDeg, pitchDeg, yawDeg);

  displayRollDeg = rollDeg;
  displayPitchDeg = pitchDeg;
  displayYawDeg = 0.0f;
  rollZeroDeg = displayRollDeg;
  pitchZeroDeg = displayPitchDeg;
  yawZeroDeg = 0.0f;
  displayInitialized = true;

  dynamicGyroBiasRawX = 0.0f;
  dynamicGyroBiasRawY = 0.0f;
  dynamicGyroBiasRawZ = 0.0f;
  gyroXRateDps = 0.0f;
  gyroYRateDps = 0.0f;
  gyroZRateDps = 0.0f;
}

void fusionUpdate(const RawSample &sample, float dt) {
  if (dt <= 0.0f || dt > RATE_LIMIT_DT_S) {
    dt = 0.01f;
  }

  float axG = (float)sample.axRaw / ACCEL_SCALE;
  float ayG = (float)sample.ayRaw / ACCEL_SCALE;
  float azG = (float)sample.azRaw / ACCEL_SCALE;
  float accelMagG = sqrtf(axG * axG + ayG * ayG + azG * azG);

  gyroXRateDps = ((float)sample.gxRaw - gyroBiasRawX - dynamicGyroBiasRawX) / GYRO_SCALE;
  gyroYRateDps = ((float)sample.gyRaw - gyroBiasRawY - dynamicGyroBiasRawY) / GYRO_SCALE;
  gyroZRateDps = ((float)sample.gzRaw - gyroBiasRawZ - dynamicGyroBiasRawZ) / GYRO_SCALE;

  if (fabsf(gyroXRateDps) < gyroRateDeadbandDps) gyroXRateDps = 0.0f;
  if (fabsf(gyroYRateDps) < gyroRateDeadbandDps) gyroYRateDps = 0.0f;
  if (fabsf(gyroZRateDps) < YAW_RATE_DEADBAND_DPS) gyroZRateDps = 0.0f;

  updateDynamicGyroBias(gyroXRateDps, gyroYRateDps, gyroZRateDps, accelMagG, dt);
  updateOrientation(axG, ayG, azG, gyroXRateDps, gyroYRateDps, gyroZRateDps, dt);

  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;
  float ignoredYawDeg = 0.0f;
  quaternionToFusedDeg(rollDeg, pitchDeg, ignoredYawDeg);
  displayRollDeg = rollDeg;
  displayPitchDeg = pitchDeg;

  float yawGyroDeltaDeg = gyroZRateDps * dt;
  if (fabsf(yawGyroDeltaDeg) <= YAW_STEP_DEADBAND_DEG) {
    yawGyroDeltaDeg = 0.0f;
  }
  displayYawDeg = wrapAngle180(displayYawDeg + yawGyroDeltaDeg);
  if (fabsf(displayYawDeg) < YAW_OUTPUT_DEADZONE_DEG) {
    displayYawDeg = 0.0f;
  }

  displayInitialized = true;
}

Reading fusionGetReading() {
  return {gyroXRateDps,
          gyroYRateDps,
          gyroZRateDps,
          wrapAngle180(displayRollDeg - rollZeroDeg),
          wrapAngle180(displayPitchDeg - pitchZeroDeg),
          wrapAngle180(displayYawDeg - yawZeroDeg)};
}

}  // namespace gyro
