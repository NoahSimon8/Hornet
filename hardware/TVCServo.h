#pragma once
#include <Arduino.h>
#include "PWMDriver.h"
#include "../util/Math.h"

// A single servo that moves the TVC linkage on one axis.
// It accepts a desired *thrust-vector tilt* (beta) in degrees
// and maps it to a servo angle using a four-bar linkage model,
// then writes a PWM pulse via the PCA9685 driver.

// 150-600 tick different fromo 1000-2000 us?
class TVCServo
{
public:
    struct Linkage
    {
        // geometry (mm) — copy from your current constants
        float L1, L2, L3, L4;
        float betaInitDeg;  // neutral thrust angle
        float thetaInitDeg; // neutral servo angle
    };

    TVCServo(PWMDriver &drv, uint8_t channel,
             const Linkage &linkage,
             float servoMinDeg = 0.0f, float servoMaxDeg = 180.0f,
             uint16_t minUs = 1000, uint16_t maxUs = 2000,
             float maxSlewDegPerUpdate = 3.0f)
        : _drv(drv), _ch(channel), _lk(linkage), _servoMin(servoMinDeg), _servoMax(servoMaxDeg),
          _minUs(minUs), _maxUs(maxUs), _slew(maxSlewDegPerUpdate) {}

    void begin(float initialServoDeg = 90.0f)
    {
        _servoDegCmd = util::clamp(initialServoDeg, _servoMin, _servoMax);
        writeServo(_servoDegCmd);
    }

    // Set desired *thrust* angle (deg) relative to neutral; apply in update()
    void setDesiredThrustDeg(float betaDeg)
    {
        _desiredThrustDeg = betaDeg;
    }

    // Call each loop to apply slew-limit and write PWM
    void update()
    {
        float targetServo = thrustToServoDeg(_desiredThrustDeg);
        // slew limit
        float delta = targetServo - _servoDegCmd;
        float step = util::clamp(delta, -_slew, _slew);
        _servoDegCmd = util::clamp(_servoDegCmd + step, _servoMin, _servoMax);
        writeServo(_servoDegCmd);
    }

    float commandedServoDeg() const { return _servoDegCmd; }

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
        uint16_t us = util::angleDegToUs(servoDeg, _servoMin, _servoMax, _minUs, _maxUs);
        _drv.writeMicroseconds(_ch, us);
    }

    PWMDriver &_drv;
    uint8_t _ch;
    Linkage _lk;
    float _servoMin, _servoMax;
    uint16_t _minUs, _maxUs;
    float _slew;
    float _desiredThrustDeg{0.0f};
    float _servoDegCmd{90.0f};

    float _neutralDeg; // your per-axis neutral (angle14 or angle15)
    int8_t _sign;      // +1 or -1 to match axis direction in the original
};
