#include <Arduino.h>
#include <array>
#include "hardware/IMU.h"
#include "hardware/Potentiometer.h"
#include "hardware/PWMDriver.h"
#include "hardware/ESC.h"
#include "hardware/TVCServo.h"
#include "util/Math.h"
#include "util/PID.h"

// ---------- Pins / channels ----------
constexpr uint8_t PIN_POT = A3;
constexpr uint8_t PCA9685_ADDR = 0x40;

// PCA9685 channels
constexpr uint8_t CH_ESC1 = 12;
constexpr uint8_t CH_ESC2 = 13;
constexpr uint8_t CH_SERVO_X = 14;
constexpr uint8_t CH_SERVO_Y = 15;

// ---------- Constants ----------

const float maxThrottleN = 2.5; // newtons, this is an estimate
const float maxGimble = 12.0;   // degrees, this is an estimate
const float angleXNeutral = 108.0f;
const float angleYNeutral = 47.0f;
const double loopFreq = 100.0; // Hz

// ---------- Global hardware objects ----------
PWMDriver pwm(PCA9685_ADDR);
ESC esc1(pwm, CH_ESC1, 1000, 2000);
ESC esc2(pwm, CH_ESC2, 1000, 2000);
Potentiometer pot(PIN_POT);
IMU imu(0x4B);

// Linkage numbers — copy your real values here
TVCServo::Linkage linkX{/*L1*/ 44, /*L2*/ 76.8608, /*L3*/ 75.177, /*L4*/ 28, /*beta0*/ 77.98, /*theta0*/ 77.98};
TVCServo::Linkage linkY{/*L1*/ 44, /*L2*/ 76.8608, /*L3*/ 75.177, /*L4*/ 28, /*beta0*/ 77.98, /*theta0*/ 77.98};

TVCServo tvcX(pwm, CH_SERVO_X, linkX, 0, 180, 1000, 2000, 2.0f, angleXNeutral, +1);
TVCServo tvcY(pwm, CH_SERVO_Y, linkY, 0, 180, 1000, 2000, 2.0f, angleYNeutral, -1);

// ---------- PID controllers ----------
// PID pidRoll(0.8, 0.05, 0.1);  // needs tuning
// PID pidPitch(0.8, 0.05, 0.1); // needs tuning
// PID pidYaw(0.8, 0.0, 0.0);    // needs tuning
// PID pidX(1, 0, 0);            // needs tuning
// PID pidY(1, 0, 0);            // needs tuning
// PID pidZ(1, 0, 0);            // needs tuning
// PID pidZVelocity(1, 0, 0);    // needs tuning

PID pidRoll(0, 0, 0);      // needs tuning
PID pidPitch(0, 0, 0);     // needs tuning
PID pidYaw(0, 0, 0);       // needs tuning
PID pidX(0, 0, 0);         // needs tuning
PID pidY(0, 0, 0);         // needs tuning
PID pidZ(0, 0, 0);         // needs tuning
PID pidZVelocity(0, 0, 0); // needs tuning
// ---------- Simple runtime state ----------

struct Ref
{
    float yaw0{0}, pitch0{0}, roll0{0};
    float x0{0}, y0{0}, z0{0};
    bool set{false};
} ref;

struct State
{
    float yaw{0}, pitch{0}, roll{0};
    float yawRate{0}, pitchRate{0}, rollRate{0};
    float x{0}, y{0}, z{0};
    float xVel{0}, yVel{0}, zVel{0};
    float xAcc{0}, yAcc{0}, zAcc{0};
} state;

struct desState
{
    float yaw{0}, pitch{0}, roll{0};
    float x{0}, y{0}, z{0};
    float zVel{0};
} desState;

// ---------- Global state ----------

double prevIMUTimestampUs = 0.0;
int loopCount = 0;

// ---------- data logging ----------

void printStatus(uint16_t thrUsA, uint16_t thrUsB)
{
    if (!imu.hasData())
    {
        Serial.println(F("[fly] IMU has no data."));
        return;
    }
    Serial.print("thr_usA=");
    Serial.print(thrUsA);
    Serial.print(", thr_usB=");
    Serial.print(thrUsB);
    Serial.print(", pot=");
    Serial.print(pot.read01(), 3);
    
    Serial.print(", yaw=");
    Serial.print(state.yaw, 2);
    Serial.print(", pitch=");
    Serial.print(state.pitch, 2);
    Serial.print(", roll=");
    Serial.print(state.roll, 2);
    Serial.print(", x=");
    Serial.print(state.x, 2);
    Serial.print(", y=");
    Serial.print(state.y, 2);
    Serial.print(", z=");
    Serial.print(state.z, 2);
    Serial.print(", xVel=");
    Serial.print(state.xVel, 2);
    Serial.print(", yVel=");
    Serial.print(state.yVel, 2);
    Serial.print(", zVel=");
    Serial.print(state.zVel, 2);
    Serial.print(", xAcc=");
    Serial.print(state.xAcc, 2);
    Serial.print(", yAcc=");
    Serial.print(state.yAcc, 2);
    Serial.print(", zAcc=");
    Serial.print(state.zAcc, 2);

    Serial.print(", desPitch=");
    Serial.print(desState.pitch, 2);
    Serial.print(", y=");
    Serial.print(desState.roll, 2);
    Serial.print(", desZVel=");
    Serial.print(desState.zVel, 2);

    Serial.print(", svx=");
    Serial.print(tvcX.commandedServoDeg(), 1);
    Serial.print(", svy=");
    Serial.print(tvcY.commandedServoDeg(), 1);
    Serial.println();
}

// ---------- Main program ----------

// Utility that doesn’t belong to a single device: basic centering & neutralization
void centerTVC()
{
    tvcX.setDesiredThrustDeg(0.0f);
    tvcY.setDesiredThrustDeg(0.0f);
}

void setup()
{

    Serial.begin(115200);
    Serial.println(F("[fly] starting setup..."));
    delay(500);

    // I/O
    pot.begin();

    // PWM driver for servos/ESC
    pwm.begin(50.0f);

    // Bring servos to neutral
    // centerTVC();

    // IMU
    while (!imu.begin())
    {
        Serial.println(F("[fly] IMU failed to start."));
        delay(1000);
    }

    ref.yaw0 = imu.yawDeg();
    ref.pitch0 = imu.pitchDeg();
    ref.roll0 = imu.rollDeg();
    ref.set = true;

    Serial.println(F("[fly] reference orientation captured."));
    Serial.println(F("[fly] IMU ok."));

    // Arm ESCs at minimum
    esc1.arm(1000, 1000);
    esc2.arm(1000, 1000);
    esc1.setMicroseconds(1000);
    esc2.setMicroseconds(1000);

    Serial.println(F("[fly] setup complete."));
}

// Parse a token that might be "kp=...", "ki=...", "kd=...", or single-char commands.
void parseToken(const String &token)
{
    if (token.startsWith("kp="))
    {
        String val = token.substring(3);
        float newKp = val.toFloat();
        if (newKp != 0.0 || val == "0" || val == "0.0")
        {
            pidRoll.setP(newKp);
            pidPitch.setP(newKp);
            Serial.print("Updated kp to: ");
            Serial.println(newKp);
        }
    }
    else if (token.startsWith("ki="))
    {
        String val = token.substring(3);
        float newKi = val.toFloat();
        if (newKi != 0.0 || val == "0" || val == "0.0")
        {
            pidRoll.setI(newKi);
            pidPitch.setI(newKi);
            Serial.print("Updated ki to: ");
            Serial.println(newKi);
        }
    }
    else if (token.startsWith("kd="))
    {
        String val = token.substring(3);
        float newKd = val.toFloat();
        if (newKd != 0.0 || val == "0" || val == "0.0")
        {
            pidRoll.setD(newKd);
            pidPitch.setD(newKd);
            Serial.print("Updated kd to: ");
            Serial.println(newKd);
        }
    }
}

// Function to process incoming serial commands
void processSerialCommands()
{
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() == 0)
            return;

        int startIndex = 0;
        while (true)
        {
            int spaceIndex = input.indexOf(' ', startIndex);
            String token;
            if (spaceIndex == -1)
            {
                token = input.substring(startIndex);
                token.trim();
                if (token.length() > 0)
                {
                    parseToken(token);
                }
                break;
            }
            else
            {
                token = input.substring(startIndex, spaceIndex);
                token.trim();
                if (token.length() > 0)
                {
                    parseToken(token);
                }
                startIndex = spaceIndex + 1;
            }
        }
    }
}

// returns {xOutput, yOutput} for thrust angles
std::array<float, 2> rotationalPID(float dt)
{
    // Compute PID outputs
    std::array<float, 2> out{};

    out[0] = pidRoll.calculate(desState.roll, state.roll, dt);
    out[1] = pidPitch.calculate(desState.pitch, state.pitch, dt);

    return out;
}

// returns {xOutput, yOutput} for thruster angles
std::array<float, 2> lateralPID(float dt)
{
    if (loopCount % 3 == 0)
    {
        // Compute PID outputs
        desState.roll = pidX.calculate(desState.x, 0, dt);
        desState.pitch = pidY.calculate(desState.y, 0, dt);
    }

    return rotationalPID(dt);
}

// returns desired throttle 0..1
float verticalPID(float dt)
{
    desState.zVel = pidZ.calculate(desState.z, 0, dt);
    float desThrottle = pidZVelocity.calculate(desState.zVel, 0, dt);
    return util::clamp(desThrottle / maxThrottleN, 0.0f, 1.0f);
}

// returns desired throttle 0..1
float potentiometerThrottle()
{
    return util::clamp(pot.read01(), 0.0f, 1.0f);
}

// returns a desired constant, -1..1 to add to ESC 1, subtract from ESC 2
float yawPID(float dt)
{
    float yawError = desState.yaw - state.yaw;
    if (yawError > 180.0f)
        yawError -= 360.0f;
    if (yawError < -180.0f)
        yawError += 360.0f;

    float out = pidYaw.calculate(0.0f, -yawError, dt); // negative error maybe?
    return out;
}

void updateState()
{
    if (!imu.hasData())
        return;
    auto e = imu.euler();
    state.yaw = e.yaw - ref.yaw0;
    state.pitch = e.pitch - ref.pitch0;
    state.roll = e.roll - ref.roll0;

    // MEKF data for position and rates

    auto la = imu.linearAccel();
    state.xAcc = la.x;
    state.yAcc = la.y;
    state.zAcc = la.z;
}

void loop()
{
    // For maintaining a steady loop rate
    double loopstartTimestampUS = micros();

    // Check for serial commands
    processSerialCommands();

    // Refresh IMU data
    imu.update();
    updateState();

    // record timestamp for PID use
    double IMUTimestampUs = micros();
    double deltaTime = (IMUTimestampUs - prevIMUTimestampUs) * 1e-6; // seconds
    prevIMUTimestampUs = IMUTimestampUs;

    // get desired thrust angles
    std::array<float, 2> pidOut = lateralPID(deltaTime); // level flight at origin

    // Apply to servos as desired thrust angles
    tvcX.setDesiredThrustDeg(pidOut[0]);
    tvcY.setDesiredThrustDeg(pidOut[1]);

    // get base throttle
    float throttle01 = potentiometerThrottle(); // swap to verticalPID(z, dt) for altitude hold

    // calculate yaw correction
    float yawAdjust = yawPID(deltaTime);
    float throttle01a = util::clamp(throttle01 + yawAdjust, 0.0f, 1.0f);
    float throttle01b = util::clamp(throttle01 - yawAdjust, 0.0f, 1.0f);

    // Apply to ESCs
    uint16_t throttleUsA = static_cast<uint16_t>(util::mapFloat(throttle01a, 0.0f, 1.0f, 1000, 2000));
    uint16_t throttleUsB = static_cast<uint16_t>(util::mapFloat(throttle01b, 0.0f, 1.0f, 1000, 2000));
    esc1.setMicroseconds(throttleUsA);
    esc2.setMicroseconds(throttleUsB);

    // Apply servo updates (slew limiting + PWM)
    tvcX.update();
    tvcY.update();

    // Print a quick status line
    printStatus(throttleUsA, throttleUsB);

    // Maintain a steady loop rate
    double loopdt = (micros() - loopstartTimestampUS) * 1e-6; // seconds
    delay(((1 / loopFreq) - loopdt) * 1000.0);
}
