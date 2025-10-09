#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <sh2.h>
#include <sh2_err.h>

struct LinearAccel
{
  float x{0}, y{0}, z{0}; // m/s^2
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
  explicit IMU(uint8_t i2cAddr = 0x4B, TwoWire &wirePort = Wire2)
      : _addr(i2cAddr), _wire(&wirePort) {}

  bool begin()
  {
    _wire->begin();
    if (!_bno.begin_I2C(_addr, _wire))
      return false;

    // Enable the reports this application consumes
    _bno.enableReport(SH2_ARVR_STABILIZED_RV, 10000);
    _bno.enableReport(SH2_GYRO_INTEGRATED_RV, 10000);
    _bno.enableReport(SH2_LINEAR_ACCELERATION, 10000);
    _bno.enableReport(SH2_ACCELEROMETER, 10000);
    return true;
  }

  void update()
  {
    if (_bno.wasReset())
    {
      Serial.println(F("[fly] IMU was reset."));
      _calibrationSaved = false;
      _calibrationHighSinceMs = 0;
      _lastCalSaveAttemptMs = 0;
      _rvAccuracy = 0;
      _rvAccuracyRad = 0.0f;
    }

    sh2_SensorValue_t val;
    while (_bno.getSensorEvent(&val))
    {
      switch (val.sensorId)
      {
      case SH2_ARVR_STABILIZED_RV:
        _quat.w = val.un.arvrStabilizedRV.real;
        _quat.x = val.un.arvrStabilizedRV.i;
        _quat.y = val.un.arvrStabilizedRV.j;
        _quat.z = val.un.arvrStabilizedRV.k;
        _rvAccuracyRad = val.un.arvrStabilizedRV.accuracy;
        _rvAccuracy = val.status & 0x03;
        _euler = quatToEulerDeg(_quat);
        _hasData = true;
        break;

      case SH2_LINEAR_ACCELERATION:
        _linearAccel.x = val.un.linearAcceleration.x;
        _linearAccel.y = val.un.linearAcceleration.y;
        _linearAccel.z = val.un.linearAcceleration.z;
        break;

      case SH2_ACCELEROMETER:
        _rawAccel.x = val.un.accelerometer.x;
        _rawAccel.y = val.un.accelerometer.y;
        _rawAccel.z = val.un.accelerometer.z;
        break;

      default:
        break;
      }
    }

    if (_rvAccuracy >= 3)
    {
      if (_calibrationHighSinceMs == 0)
        _calibrationHighSinceMs = millis();
    }
    else
    {
      _calibrationHighSinceMs = 0;
      if (_rvAccuracy <= 1)
        _calibrationSaved = false;
    }

    const uint32_t now = millis();
    if (!_calibrationSaved && _calibrationHighSinceMs != 0)
    {
      constexpr uint32_t kMinStableMs = 3000;
      constexpr uint32_t kRetryMs = 1000;
      bool stableLongEnough = (now - _calibrationHighSinceMs) >= kMinStableMs;
      bool retryElapsed = (now - _lastCalSaveAttemptMs) >= kRetryMs;
      if (stableLongEnough && retryElapsed)
      {
        if (saveCalibration())
        {
          Serial.println(F("[fly] IMU calibration saved."));
          _calibrationHighSinceMs = 0;
        }
        else
        {
          Serial.println(F("[fly] IMU calibration save failed, will retry."));
          _lastCalSaveAttemptMs = now;
        }
      }
    }
  }

  bool hasData() const { return _hasData; }
  QuaternionF quat() const { return _quat; }
  Euler euler() const { return _euler; }
  LinearAccel linearAccel() const { return _linearAccel; }
  LinearAccel accelerometer() const { return _rawAccel; }
  uint8_t rotationAccuracy() const { return _rvAccuracy; }
  float rotationAccuracyRad() const { return _rvAccuracyRad; }
  bool isFullyCalibrated() const { return _rvAccuracy >= 3; }

  bool saveCalibration()
  {
    const int rc = sh2_saveDcdNow();
    _lastCalSaveAttemptMs = millis();
    if (rc == SH2_OK)
    {
      _calibrationSaved = true;
      return true;
    }
    return false;
  }

  float yawDeg() const { return _euler.yaw; }
  float pitchDeg() const { return _euler.pitch; }
  float rollDeg() const { return _euler.roll; }

private:
  static Euler quatToEulerDeg(const QuaternionF &q)
  {
    const float ysqr = q.y * q.y;

    float t0 = +2.0f * (q.w * q.x + q.y * q.z);
    float t1 = +1.0f - 2.0f * (q.x * q.x + ysqr);
    float roll = atan2f(t0, t1);

    float t2 = +2.0f * (q.w * q.y - q.z * q.x);
    t2 = t2 > 1.0f ? 1.0f : (t2 < -1.0f ? -1.0f : t2);
    float pitch = asinf(t2);

    float t3 = +2.0f * (q.w * q.z + q.x * q.y);
    float t4 = +1.0f - 2.0f * (ysqr + q.z * q.z);
    float yaw = atan2f(t3, t4);

    constexpr float RAD2DEG = 57.2957795f;
    return Euler{yaw * RAD2DEG, pitch * RAD2DEG, roll * RAD2DEG};
  }

  uint8_t _addr;
  TwoWire *_wire;
  Adafruit_BNO08x _bno;
  QuaternionF _quat{};
  Euler _euler{};
  LinearAccel _linearAccel{};
  LinearAccel _rawAccel{};
  bool _hasData{false};
  uint8_t _rvAccuracy{0};
  float _rvAccuracyRad{0.0f};
  bool _calibrationSaved{false};
  uint32_t _calibrationHighSinceMs{0};
  uint32_t _lastCalSaveAttemptMs{0};
};
