#pragma once

#include <Arduino.h>

namespace gyro {

struct Reading {
  float gxDps;
  float gyDps;
  float gzDps;
  float angleX;
  float angleY;
  float angleZ;
};

void begin();
bool update();
bool isReady();
Reading getReading();

}
