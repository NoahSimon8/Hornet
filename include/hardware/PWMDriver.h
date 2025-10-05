#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class PWMDriver
{
public:
  explicit PWMDriver(uint8_t i2cAddr = 0x40) : _pwm(i2cAddr) {}

  bool begin(float freqHz = 50.0f)
  {
    _pwm.begin();
    _pwm.setPWMFreq(freqHz);
    _freq = freqHz;
    return true;
  }

  // Write a raw 12-bit pulse (0..4095) on a channel
  void writeRaw(uint8_t ch, uint16_t on, uint16_t off)
  {
    _pwm.setPWM(ch, on, off);
  }

  // Write a pulse in microseconds (converts to 12-bit ticks)
  void writeMicroseconds(uint8_t ch, uint16_t microseconds)
  {
    microseconds = constrain<uint16_t>(microseconds, (uint16_t)1000, (uint16_t)2000u);
    const float period_us = 1e6f / _freq; // e.g., 20000 µs at 50 Hz
    const float ticks_per_us = 4096.0f / period_us;
    uint16_t ticks = static_cast<uint16_t>(microseconds * ticks_per_us);
    _pwm.setPWM(ch, 0, ticks);
  }

private:
  Adafruit_PWMServoDriver _pwm;
  float _freq{50.0f};
};
