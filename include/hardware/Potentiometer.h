#pragma once
#include <Arduino.h>
#include "../util/Math.h"

class Potentiometer
{
public:
  explicit Potentiometer(uint8_t analogPin = A2) : _pin(analogPin) {}
  void begin() { pinMode(_pin, INPUT); }

  int readRaw() const { return analogRead(_pin); } // 0..1023 (or 4095)
  float read01() const
  { // 0..1
    float raw = static_cast<float>(readRaw());
    float maxADC = 1023.0f; // change to 4095.0f if your MCU is 12-bit
    return util::clamp(raw / maxADC, 0.0f, 1.0f);
  }

  uint16_t readMicroseconds(uint16_t minUs, uint16_t maxUs) const
  {
    float t = read01();
    return static_cast<uint16_t>(minUs + t * (maxUs - minUs));
  }

private:
  uint8_t _pin;
};
