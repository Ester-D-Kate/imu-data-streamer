#include <Arduino.h>
#include "gyro.h"

static void printReading(const gyro::Reading &reading) {
  Serial.print("Fused deg Roll/Pitch/Yaw: ");
  Serial.print(reading.angleX, 2);
  Serial.print(",");
  Serial.print(reading.angleY, 2);
  Serial.print(",");
  Serial.println(reading.angleZ, 2);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  gyro::begin();
}

void loop() {
  if (!gyro::isReady()) {
    static unsigned long lastRetryMs = 0;
    if (millis() - lastRetryMs > 3000) {
      lastRetryMs = millis();
      gyro::begin();
    
    delay(100);
    return;
  }

  if (gyro::update()) {
    printReading(gyro::getReading());
  }

  delay(100);
}