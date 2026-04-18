#include <Arduino.h>
#include "gyro.h"
#include "wifi_manager.h"

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

  Serial.println();
  Serial.println("imu-data-streamer boot");

  gyro::begin();
  wifi_manager::begin();
}

void loop() {
  wifi_manager::loop();

  if (!gyro::isReady()) {
    static unsigned long lastRetryMs = 0;
    if (millis() - lastRetryMs > 3000) {
      lastRetryMs = millis();
      gyro::begin();
    }

    delay(100);
    return;
  }

  if (gyro::update()) {
    const gyro::Reading reading = gyro::getReading();
    wifi_manager::publishGyro(reading);
    printReading(reading);
  }

  delay(10);
}