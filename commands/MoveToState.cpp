#include <Arduino.h>
#include "hardware/IMU.h"
#include "hardware/Potentiometer.h"
#include "hardware/PWMDriver.h"
#include "hardware/ESC.h"
#include "hardware/TVCServo.h"
#include "util/Math.h"
#include "utl/pid.h"
#include "utl/SerialCommands.h"

// ---------- Pins / channels ----------
constexpr uint8_t PIN_POT = A3;
constexpr uint8_t PCA9685_ADDR = 0x40;

// PCA9685 channels
constexpr uint8_t CH_ESC1 = 12;
constexpr uint8_t CH_ESC2 = 13;
constexpr uint8_t CH_SERVO_X = 14;
constexpr uint8_t CH_SERVO_Y = 15;

// ---------- Global hardware objects ----------
PWMDriver pwm(PCA9685_ADDR);
ESC esc1(pwm, CH_ESC1, 1000, 2000);
ESC esc2(pwm, CH_ESC2, 1000, 2000);
Potentiometer pot(PIN_POT);
IMU imu(0x4B);

// Linkage numbers — copy your real values here
TVCServo::Linkage linkX{/*L1*/ 44, /*L2*/ 76.8608, /*L3*/ 75.177, /*L4*/ 28, /*beta0*/ 77.98, /*theta0*/ 77.98};
TVCServo::Linkage linkY{/*L1*/ 44, /*L2*/ 76.8608, /*L3*/ 75.177, /*L4*/ 28, /*beta0*/ 77.98, /*theta0*/ 77.98};

float maxThrottleN = 2.5; // newtons, this is an estimate
float maxGimble = 12.0;   // degrees, this is an estimate

TVCServo tvcX(pwm, CH_SERVO_X, linkX, 0, 180, 1000, 2000, 2.0f);
TVCServo tvcY(pwm, CH_SERVO_Y, linkY, 0, 180, 1000, 2000, 2.0f);

PID pidRoll(0.8, 0.05, 0.1);  // needs tuning
PID pidPitch(0.8, 0.05, 0.1); // needs tuning
PID pidYaw(0.8, 0.0, 0.0);    // needs tuning
PID pidZ(1, 0, 0);            // needs tuning
PID pidZVelocity(1, 0, 0);    // needs tuning

double loopFreq = 100.0; // Hz
double prevIMUTimestampUs = 0.0;

// ---------- Simple runtime state ----------
struct Ref
{
    float yaw0{0}, pitch0{0}, roll0{0};
    bool set{false};
} ref;

void printStatus(uint16_t thrUs)
{
    if (!imu.hasData())
        return;
    const auto e = imu.euler();
    Serial.print("thr_us=");
    Serial.print(thrUs);
    Serial.print(", yaw=");
    Serial.print(e.yaw, 2);
    Serial.print(", pitch=");
    Serial.print(e.pitch, 2);
    Serial.print(", roll=");
    Serial.print(e.roll, 2);
    Serial.print(", svx=");
    Serial.print(tvcX.commandedServoDeg(), 1);
    Serial.print(", svy=");
    Serial.print(tvcY.commandedServoDeg(), 1);
    Serial.println();
}

// Utility that doesn’t belong to a single device: basic centering & neutralization
void centerTVC()
{
    tvcX.setDesiredThrustDeg(0.0f);
    tvcY.setDesiredThrustDeg(0.0f);
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    // I/O
    pot.begin();

    // PWM driver for servos/ESC
    pwm.begin(50.0f);

    // Bring servos to neutral
    centerTVC();

    // IMU
    while (!imu.begin())
    {
        Serial.println(F("[fly] IMU failed to start."));
        delay(1000);
    }
    Serial.println(F("[fly] IMU ok."));

    // Arm ESCs at minimum
    esc1.arm(1000, 1000);
    esc2.arm(1000, 1000);
    esc1.setMicroseconds(1000);
    esc2.setMicroseconds(1000);

    Serial.println(F("[fly] setup complete."));
}

std::pair<float, float> rotationalPID(float targetPitchDeg, float targetRollDeg, float dt)
{
    // Compute PID outputs
    float outX = pidRoll.calculate(targetRollDeg, imu.rollDeg(), dt);
    float outY = pidPitch.calculate(targetPitchDeg, imu.pitchDeg(), dt);

    return {outX, outY};
}

std::pair<float, float> lateralPID(float targetX, float targetY, float dt)
{
    // Compute PID outputs
    float desRoll = pidRoll.calculate(targetX, 0, dt);
    float desPitch = pidPitch.calculate(targetY, 0, dt);

    return rotationalPID(desPitch, desRoll, dt);
}

// returns desired throttle 0..1
float verticalPID(float targetZ, float dt)
{
    float desZVelocity = pidZ.calculate(targetZ, 0, dt);
    float desThrottle = pidZVelocity.calculate(desZVelocity, 0, dt);
    return util::clamp(desThrottle / maxThrottleN, 0.0f, 1.0f);
}

// returns desired throttle 0..1
float potentiometerThrottle()
{
    return util::clamp(pot.read01(), 0.0f, 1.0f);
}

// returns a desired constant, -1..1 to add to ESC 1, subtract from ESC 2
float yawPID(float targetYawDeg, float dt)
{
    float yawError = targetYawDeg - imu.yawDeg();
    if (yawError > 180.0f)
        yawError -= 360.0f;
    if (yawError < -180.0f)
        yawError += 360.0f;

    float out = pidYaw.calculate(0.0f, -yawError, dt); // negative error maybe?
    return out;
}

void loop()
{
    // For maintaining a steady loop rate
    double loopstartTimestampUS = micros();

    // Check for serial commands
    processSerialCommands();

    // Refresh IMU data
    imu.update();

    // record timestamp for PID use
    double IMUTimestampUs = micros();
    double deltaTime = (IMUTimestampUs - prevIMUTimestampUs) * 1e-6; // seconds
    prevIMUTimestampUs = IMUTimestampUs;

    // get desired thrust angles
    auto [outX, outY] = rotationalPID(0.0f, 0.0f, deltaTime); // level flight (swap for lateralPID(x,y,dt) for position hold)

    // Apply to servos as desired thrust angles
    tvcX.setDesiredThrustDeg(outX);
    tvcY.setDesiredThrustDeg(outY);

    // get base throttle
    float throttle01 = potentiometerThrottle(); // swap to verticalPID(z, dt) for altitude hold
 
    // calculate yaw correction
    float yawAdjust = yawPID(0.0f, deltaTime); 
    float throttle01a = util::clamp(throttle01 + yawAdjust, 0.0f, 1.0f);
    float throttle01b = util::clamp(throttle01 - yawAdjust, 0.0f, 1.0f);

    // Apply to ESCs
    uint16_t throttleUsA = static_cast<uint16_t>(util::map(throttle01a, 0.0f, 1.0f, 1000, 2000));
    uint16_t throttleUsB = static_cast<uint16_t>(util::map(throttle01b, 0.0f, 1.0f, 1000, 2000));
    esc1.setMicroseconds(throttleUsA);
    esc2.setMicroseconds(throttleUsB);

    // Apply servo updates (slew limiting + PWM)
    tvcX.update();
    tvcY.update();

    // Print a quick status line
    printStatus(throttleUs);

    // Remember for next loop PID

    // Maintain a steady loop rate
    double loopdt = (micros() - loopstartTimestampUS) * 1e-6; // seconds
    delay((1 / loopFreq) - loopdt);
}
