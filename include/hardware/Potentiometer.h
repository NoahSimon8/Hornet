#pragma once
#include <Arduino.h>
#include "../util/Math.h"

class Potentiometer
{
public:
  explicit Potentiometer(uint8_t analogPin, uint8_t resolutionBits = 12)
      : _pin(analogPin), _resBits(resolutionBits) {}

  void begin()
  {
    analogReadResolution(_resBits);
    analogReadAveraging(8);
    pinMode(_pin, INPUT);
  }

  int readRaw() const
  {
    return analogRead(_pin);
  }

  float read01() const
  {
    const int raw = readRaw();
    const float maxAdc = float((1UL << _resBits) - 1);
    return util::clamp(raw / maxAdc, 0.0f, 1.0f);
  }

private:
  uint8_t _pin;
  uint8_t _resBits;
};
