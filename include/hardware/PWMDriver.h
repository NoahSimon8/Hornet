#pragma once
#include <Arduino.h>

// Direct Teensy 4.1 PWM output driver (no PCA9685).
//
// Keeps the same API as the old PCA9685 wrapper:
//   - begin(freqHz)
//   - writeMicroseconds(channel, us)
//
// NEW: you must map logical channels to Teensy pins once via attach(channel, pin).
//
// Notes for Teensy 4.1:
// - We generate "servo style" pulses by running hardware PWM at ~50 Hz and varying duty cycle.
// - This works for typical RC servos and ESCs (1–2 ms high pulse in a ~20 ms frame).
class PWMDriver
{
public:
  static constexpr uint8_t kMaxChannels = 16;

  PWMDriver() = default;

  // Initialize output frequency (typically 50 Hz for servos/ESCs).
  // resolutionBits is clamped to 8..16 for stable duty computation.
  bool begin(float freqHz = 50.0f, uint8_t resolutionBits = 16)
  {
    if (freqHz <= 0.0f)
      freqHz = 50.0f;

    _freq = freqHz;

    if (resolutionBits < 8)
      resolutionBits = 8;
    if (resolutionBits > 16)
      resolutionBits = 16;

    _resolutionBits = resolutionBits;
    _maxDuty = (1u << _resolutionBits) - 1u;
    _periodUs = static_cast<uint32_t>(1000000.0f / _freq + 0.5f);

#if defined(TEENSYDUINO)
    analogWriteResolution(_resolutionBits);
#endif

    return true;
  }

  // Map a logical channel to a Teensy pin.
  // Call once in setup() for each output you use.
  bool attach(uint8_t ch, uint8_t pin, uint16_t initialUs = 1500)
  {
    if (ch >= kMaxChannels)
      return false;

    _pins[ch] = pin;
    _attached[ch] = true;
    pinMode(pin, OUTPUT);

    #if defined(TEENSYDUINO) || defined(ARDUINO_TEENSY41)
      // On Teensy, PWM frequency can be set per-pin.
      analogWriteFrequency(pin, _freq);
    #endif  

    // Initialize to a safe neutral pulse.
    writeMicroseconds(ch, initialUs);
    return true;
  }

  void detach(uint8_t ch)
  {
    if (ch >= kMaxChannels || !_attached[ch])
      return;
    const uint8_t pin = _pins[ch];
    analogWrite(pin, 0);
    _attached[ch] = false;
  }

  // Write a pulse in microseconds.
  void writeMicroseconds(uint8_t ch, uint16_t microseconds)
  {
    if (ch >= kMaxChannels || !_attached[ch])
      return;

    microseconds = constrain(microseconds, (uint16_t)500, (uint16_t)2500);

    // duty = pulse_us / period_us
    // analogWrite value = duty * maxDuty
    const uint32_t pin = _pins[ch];
    const uint32_t value =
        (static_cast<uint64_t>(microseconds) * _maxDuty + (_periodUs / 2u)) / _periodUs;

    analogWrite(pin, (int)value);
  }

  float freqHz() const { return _freq; }
  uint32_t periodUs() const { return _periodUs; }

private:
  float _freq{50.0f};
  uint32_t _periodUs{20000};
  uint8_t _resolutionBits{16};
  uint32_t _maxDuty{65535};

  uint8_t _pins[kMaxChannels]{0};
  bool _attached[kMaxChannels]{false};
};
