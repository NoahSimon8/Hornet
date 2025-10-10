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

struct BodyAxesF
{
  float forwardX{1}, forwardY{0}, forwardZ{0};
  float rightX{0}, rightY{1}, rightZ{0};
  float upX{0}, upY{0}, upZ{1};
};

struct ProjectedAngles
{
  float tiltAboutX{0};   // deg, rotation about body X (roll)
  float tiltAboutY{0};   // deg, rotation about body Y (pitch)
  float totalTilt{0};    // deg from vertical
  float heading{0};      // deg, yaw derived from forward vector projection
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
    _bno.enableReport(SH2_ROTATION_VECTOR, 10000);
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
      case SH2_ROTATION_VECTOR:
        _quat.w = val.un.rotationVector.real;
        _quat.x = val.un.rotationVector.i;
        _quat.y = val.un.rotationVector.j;
        _quat.z = val.un.rotationVector.k;
        _quat = normalize(_quat);
        _bodyQuat = applyMountingCorrection(_quat);
        _bodyAxes = quatToBodyAxes(_bodyQuat);
        _projectedAngles = computeProjectedAngles(_bodyAxes);
        _euler = quatToEulerDeg(_bodyQuat);
        _rvAccuracyRad = val.un.rotationVector.accuracy;
        _rvAccuracy = val.status & 0x03;
        _hasData = true;
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
  QuaternionF bodyQuat() const { return _bodyQuat; }
  Euler euler() const { return _euler; }
  LinearAccel accelerometer() const { return _rawAccel; }
  uint8_t rotationAccuracy() const { return _rvAccuracy; }
  float rotationAccuracyRad() const { return _rvAccuracyRad; }
  bool isFullyCalibrated() const { return _rvAccuracy >= 3; }
  BodyAxesF bodyAxes() const { return _bodyAxes; }
  ProjectedAngles projectedAngles() const { return _projectedAngles; }

  void setSensorToBodyEuler(float yawDeg, float pitchDeg, float rollDeg)
  {
    _sensorToBody = normalize(quatFromEulerDeg(yawDeg, pitchDeg, rollDeg));
  }

  void setSensorToBodyQuat(const QuaternionF &q)
  {
    _sensorToBody = normalize(q);
  }

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
  static constexpr float RAD2DEG = 57.2957795f;
  static constexpr float DEG2RAD = 0.0174532925f;

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

    return Euler{yaw * RAD2DEG, pitch * RAD2DEG, roll * RAD2DEG};
  }

  static QuaternionF normalize(const QuaternionF &q)
  {
    const float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm <= 0.0f)
      return QuaternionF{};
    const float inv = 1.0f / norm;
    return QuaternionF{q.w * inv, q.x * inv, q.y * inv, q.z * inv};
  }

  static QuaternionF conjugate(const QuaternionF &q)
  {
    return QuaternionF{q.w, -q.x, -q.y, -q.z};
  }

  static QuaternionF multiply(const QuaternionF &a, const QuaternionF &b)
  {
    return QuaternionF{
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
  }

  static QuaternionF quatFromEulerDeg(float yawDeg, float pitchDeg, float rollDeg)
  {
    const float cy = cosf(0.5f * yawDeg * DEG2RAD);
    const float sy = sinf(0.5f * yawDeg * DEG2RAD);
    const float cp = cosf(0.5f * pitchDeg * DEG2RAD);
    const float sp = sinf(0.5f * pitchDeg * DEG2RAD);
    const float cr = cosf(0.5f * rollDeg * DEG2RAD);
    const float sr = sinf(0.5f * rollDeg * DEG2RAD);

    QuaternionF q{
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr};
    return normalize(q);
  }

  static void rotateVector(const QuaternionF &q, float x, float y, float z, float &ox, float &oy, float &oz)
  {
    const QuaternionF vec{0.0f, x, y, z};
    const QuaternionF qc = conjugate(q);
    const QuaternionF tmp = multiply(q, vec);
    const QuaternionF res = multiply(tmp, qc);
    ox = res.x;
    oy = res.y;
    oz = res.z;
  }

  static float clampf(float v, float lo, float hi)
  {
    return v < lo ? lo : (v > hi ? hi : v);
  }

  static BodyAxesF quatToBodyAxes(const QuaternionF &q)
  {
    BodyAxesF axes{};
    rotateVector(q, 1.0f, 0.0f, 0.0f, axes.forwardX, axes.forwardY, axes.forwardZ);
    rotateVector(q, 0.0f, 1.0f, 0.0f, axes.rightX, axes.rightY, axes.rightZ);
    rotateVector(q, 0.0f, 0.0f, 1.0f, axes.upX, axes.upY, axes.upZ);
    return axes;
  }

  static ProjectedAngles computeProjectedAngles(const BodyAxesF &axes)
  {
    ProjectedAngles proj{};
    proj.tiltAboutX = atan2f(-axes.upY, axes.upZ) * RAD2DEG;
    proj.tiltAboutY = atan2f(axes.upX, axes.upZ) * RAD2DEG;
    proj.totalTilt = acosf(clampf(axes.upZ, -1.0f, 1.0f)) * RAD2DEG;

    const float heading = atan2f(axes.forwardY, axes.forwardX);
    proj.heading = heading * RAD2DEG;
    return proj;
  }

  QuaternionF applyMountingCorrection(const QuaternionF &sensorQuat) const
  {
    return normalize(multiply(sensorQuat, conjugate(_sensorToBody)));
  }

  uint8_t _addr;
  TwoWire *_wire;
  Adafruit_BNO08x _bno;
  QuaternionF _quat{};
  QuaternionF _bodyQuat{};
  Euler _euler{};
  LinearAccel _rawAccel{};
  bool _hasData{false};
  uint8_t _rvAccuracy{0};
  float _rvAccuracyRad{0.0f};
  bool _calibrationSaved{false};
  uint32_t _calibrationHighSinceMs{0};
  uint32_t _lastCalSaveAttemptMs{0};
  QuaternionF _sensorToBody{};
  BodyAxesF _bodyAxes{};
  ProjectedAngles _projectedAngles{};
};
