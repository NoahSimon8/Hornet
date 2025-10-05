#pragma once
#include <Arduino.h>
#include "PWMDriver.h"

class ESC
{
public:
  ESC(PWMDriver &driver, uint8_t channel, uint16_t minUs = 1000, uint16_t maxUs = 2000)
      : _driver(driver), _ch(channel), _min(minUs), _max(maxUs) {}

  void arm(uint16_t armUs = 1000, uint32_t msHold = 1000)
  {
    _driver.writeMicroseconds(_ch, armUs);
    delay(msHold);
  }

  void setMicroseconds(uint16_t us)
  {
    if (us < _min)
      us = _min;
    if (us > _max)
      us = _max;
    _driver.writeMicroseconds(_ch, us);
    _lastUs = us;
  }

  void setThrottleFloat(float t01)
  { // 0..1 -> min..max µs
    t01 = constrain(t01, 0.0f, 1.0f);
    uint16_t us = static_cast<uint16_t>(_min + t01 * (_max - _min));
    setMicroseconds(us);
  }

  uint16_t lastUs() const { return _lastUs; }

private:
  PWMDriver &_driver;
  uint8_t _ch;
  uint16_t _min, _max;
  uint16_t _lastUs{1000};
};
