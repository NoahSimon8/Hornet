#pragma once
#include <Arduino.h>

namespace util {
  template <typename T>
  inline T clamp(T v, T lo, T hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
  }

  inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  // degrees ↔ microseconds (generic 1000–2000 µs)
  inline uint16_t angleDegToUs(float deg, float minDeg, float maxDeg, uint16_t minUs, uint16_t maxUs) {
    deg = clamp(deg, minDeg, maxDeg);
    return static_cast<uint16_t>(mapFloat(deg, minDeg, maxDeg, minUs, maxUs));
  }
}
