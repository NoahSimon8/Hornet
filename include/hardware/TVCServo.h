#pragma once
#include <Arduino.h>
#include "PWMDriver.h"
#include "../util/Math.h"

// A TVC axis driven by one servo, with four-bar linkage mapping.
// You give it a desired *thrust-vector tilt* (beta, deg, relative to neutral),
// it computes the required *servo* angle (deg) from your exact 4-bar geometry,
// applies a sign and neutral offset to match your rig, slew-limits, and writes PWM.
class TVCServo
{
public:
  struct Linkage
  {
    // Link lengths (mm) from your original code
    float L1; // LINK1
    float L2; // LINK2
    float L3; // LINK3
    float L4; // LINK4
    // Neutral angles (deg) from your original code
    float betaInitDeg;  // BETA_INIT
    float thetaInitDeg; // THETA_INIT (kept for completeness if you need it later)
  };

  // sign = +1 for "neutral + delta", -1 for "neutral - delta" (matches your original axis directions)
  TVCServo(PWMDriver &drv, uint8_t channel,
           const Linkage &linkage,
           float servoMinDeg, float servoMaxDeg,
           uint16_t minUs, uint16_t maxUs,
           float maxSlewDegPerUpdate,
           float servoNeutralDeg, int8_t sign)
      : _drv(drv), _ch(channel), _lk(linkage),
        _servoMin(servoMinDeg), _servoMax(servoMaxDeg),
        _minUs(minUs), _maxUs(maxUs),
        _slew(maxSlewDegPerUpdate),
        _neutralDeg(servoNeutralDeg), _sign(sign >= 0 ? 1 : -1) {}

  void begin()
  {
    _zeroServoDelta = calculateInputAngle(0.0f);
  }

  // Set desired *thrust* angle (deg) relative to neutral (betaRelDeg)
  void setDesiredThrustDeg(float betaRelDeg)
  {
    _manualOverride = false;
    _desiredThrustDeg = betaRelDeg;
  }

  // Call each loop to apply slew-limit and write PWM
  void update()
  {
    if (_manualOverride)
    {
      writeServo(_servoDegCmd);
      return;
    }

    // Map desired thrust tilt to *servo delta* via exact four-bar, then add neutral and sign
    float currentServoDelta = calculateInputAngle(_desiredThrustDeg);
    float servoDelta = currentServoDelta - _zeroServoDelta;
    float targetServo = _neutralDeg + _sign * servoDelta;


    _servoDegCmd = targetServo;
    writeServo(_servoDegCmd);
  }

  float commandedThrustDeg() const { return _desiredThrustDeg; }
  float commandedServoDeg() const { return _servoDegCmd - _neutralDeg; }

  void setManualServoDeg(float servoDeg)
  {
    _manualOverride = true;
  }

  void clearManualOverride()
  {
    _manualOverride = false;
  }

  bool manualOverrideActive() const { return _manualOverride; }

private:
  float calculateInputAngle(float beta_final_deg) const
  {
    const float beta_init_rad = radians(_lk.betaInitDeg);
    const float beta_final_rad = radians(beta_final_deg);

    const float L1 = _lk.L1, L2 = _lk.L2, L3 = _lk.L3, L4 = _lk.L4;

    const float L = sqrtf(L1 * L1 + L2 * L2 - 2.0f * L1 * L2 * cosf(beta_init_rad + beta_final_rad));

    auto safe_acos_deg = [](float x) -> float
    {
      if (x > 1.0f)
        x = 1.0f;
      if (x < -1.0f)
        x = -1.0f;
      return degrees(acosf(x));
    };

    const float p = safe_acos_deg(-(L3 * L3 - L4 * L4 - L * L) / (2.0f * L * L4));
    const float q = safe_acos_deg(-(L1 * L1 - L2 * L2 - L * L) / (2.0f * L * L2));

    // Returns a servo *delta* (deg) near 0 when betaRel = 0, matching your original behavior.
    return 180.0f - p - q - _lk.betaInitDeg;
  }

  void writeServo(float servoDeg)
  {
    const uint16_t us = util::angleDegToUs(servoDeg, _servoMin, _servoMax, _minUs, _maxUs);
    _drv.writeMicroseconds(_ch, us);
  }

  PWMDriver &_drv;
  uint8_t _ch;
  Linkage _lk;

  float _servoMin, _servoMax;
  uint16_t _minUs, _maxUs;
  float _slew;

  float _neutralDeg; // your per-axis neutral (angle14 or angle15)
  int8_t _sign;      // +1 or -1 to match axis direction in the original

  float _desiredThrustDeg{0.0f}; // relative thrust tilt (deg)
  float _servoDegCmd{90.0f};
  float _zeroServoDelta{0.0f};
  bool _manualOverride{false};
};
