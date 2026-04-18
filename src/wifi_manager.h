#pragma once

#include <Arduino.h>

#include "gyro.h"

namespace wifi_manager {

void begin();
void loop();
bool publishGyro(const gyro::Reading &reading);
bool isConnected();
String getGyroTopic();
String getDeviceUuid();

}  // namespace wifi_manager
