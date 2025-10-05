#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

struct LinearAccel
{
  float x{0}, y{0}, z{0}; // m/sÂ²
};

struct Euler
{
  float yaw{0}, pitch{0}, roll{0}; // degrees
  Euler() = default;
  Euler(float yawDeg, float pitchDeg, float rollDeg)
      : yaw(yawDeg), pitch(pitchDeg), roll(rollDeg) {}
};

struct QuaternionF
{
  float w{1}, x{0}, y{0}, z{0}; // unit quaternion
};

class IMU
{
public:
  explicit IMU(uint8_t i2cAddr = 0x4B) : _addr(i2cAddr) {}

  bool begin()
  {
    if (!_bno.begin_I2C(_addr))
      return false;

    // Prefer AR/VR stabilized rotation vector; fall back to gyro-integrated
    _bno.enableReport(SH2_ARVR_STABILIZED_RV, 5000); // 200 Hz â†’ 5000 Âµs? (BNO uses Âµs period)
    _bno.enableReport(SH2_GYRO_INTEGRATED_RV, 5000);
    return true;
  }

  // Call frequently to pull/freshen data
  void update()
  {
    sh2_SensorValue_t val;
    while (_bno.getSensorEvent(&val))
    {
      if (val.sensorId == SH2_ARVR_STABILIZED_RV || val.sensorId == SH2_GYRO_INTEGRATED_RV)
      {
        _quat.w = val.un.arvrStabilizedRV.real;
        _quat.x = val.un.arvrStabilizedRV.i;
        _quat.y = val.un.arvrStabilizedRV.j;
        _quat.z = val.un.arvrStabilizedRV.k;
        _euler = quatToEulerDeg(_quat);
        _hasData = true;

        // FILL IN LINEAR STRUCT
      }
    }
  }

  bool hasData() const { return _hasData; }
  QuaternionF quat() const { return _quat; }
  Euler euler() const { return _euler; }
  LinearAccel linearAccel() const { return _linearAccel; }
  float yawDeg() const { return _euler.yaw; }
  float pitchDeg() const { return _euler.pitch; }
  float rollDeg() const { return _euler.roll; }

private:
  static Euler quatToEulerDeg(const QuaternionF &q)
  {
    // Standard Tait-Bryan ZYX (yaw-pitch-roll)
    const float ysqr = q.y * q.y;

    // roll (x-axis)
    float t0 = +2.0f * (q.w * q.x + q.y * q.z);
    float t1 = +1.0f - 2.0f * (q.x * q.x + ysqr);
    float roll = atan2f(t0, t1);

    // pitch (y-axis)
    float t2 = +2.0f * (q.w * q.y - q.z * q.x);
    t2 = t2 > 1.0f ? 1.0f : (t2 < -1.0f ? -1.0f : t2);
    float pitch = asinf(t2);

    // yaw (z-axis)
    float t3 = +2.0f * (q.w * q.z + q.x * q.y);
    float t4 = +1.0f - 2.0f * (ysqr + q.z * q.z);
    float yaw = atan2f(t3, t4);

    const float RAD2DEG = 57.2957795f;
    return Euler{yaw * RAD2DEG, pitch * RAD2DEG, roll * RAD2DEG};
  }

  uint8_t _addr;
  Adafruit_BNO08x _bno;
  QuaternionF _quat{};
  Euler _euler{};
  LinearAccel _linearAccel{};
  bool _hasData{false};
};
