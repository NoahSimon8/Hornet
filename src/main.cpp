#include <Arduino.h>
#include <array>
#include <Wire.h>
#include "hardware/IMU.h"
#include "hardware/Potentiometer.h"
#include "hardware/PWMDriver.h"
#include "hardware/ESC.h"
#include "hardware/TVCServo.h"
#include "util/Math.h"
#include "util/PID.h"
#include "util/mekf2.h"

// ---------- Pins / channels ----------
constexpr uint8_t PIN_POT = A2;
constexpr uint8_t PCA9685_ADDR = 0x40;

// PCA9685 channels
constexpr uint8_t CH_ESC1 = 12;
constexpr uint8_t CH_ESC2 = 13;
constexpr uint8_t CH_SERVO_X = 14;
constexpr uint8_t CH_SERVO_Y = 15;

// ---------- Constants ----------

const float maxThrottleN = 2.5; // newtons, this is an estimate
const float maxGimble = 12.0;   // degrees, this is an estimate
const float maxTiltDeg = 15.0;  // What we dont want the rocket to tilt more than (deg)
const float angleXNeutral = 108.0f;
const float angleYNeutral = 47.0f;
const double loopFreq = 100.0; // Hz
constexpr float DEG2RAD_F = 0.0174532925f;

// Mounting offset between IMU sensor frame and rocket body frame (deg)
constexpr float SENSOR_TO_BODY_YAW_DEG = 45.0f;
constexpr float SENSOR_TO_BODY_PITCH_DEG = -90.0f;
constexpr float SENSOR_TO_BODY_ROLL_DEG = 0.0f;

// ---------- Global hardware objects ----------
PWMDriver pwm(PCA9685_ADDR);
ESC esc1(pwm, CH_ESC1, 1000, 2000);
ESC esc2(pwm, CH_ESC2, 1000, 2000);
Potentiometer pot(A2);
IMU imu(0x4B, Wire2);

// Linkage numbers — copy your real values here
TVCServo::Linkage linkX{/*L1*/ 44, /*L2*/ 76.8608, /*L3*/ 75.177, /*L4*/ 28, /*beta0*/ 77.98, /*theta0*/ 77.98};
TVCServo::Linkage linkY{/*L1*/ 44, /*L2*/ 76.8608, /*L3*/ 75.177, /*L4*/ 28, /*beta0*/ 77.98, /*theta0*/ 77.98};

TVCServo tvcX(pwm, CH_SERVO_X, linkX, 0, 180, 1000, 2000, 2.0f, angleXNeutral, +1);
TVCServo tvcY(pwm, CH_SERVO_Y, linkY, 0, 180, 1000, 2000, 2.0f, angleYNeutral, -1);

// ---------- PID controllers ----------
// PID pidRoll(0.8, 0.05, 0.1);  // needs tuning
// PID pidPitch(0.8, 0.05, 0.1); // needs tuning
// PID pidHeading(0.8, 0.0, 0.0);    // needs tuning
// PID pidX(1, 0, 0);            // needs tuning
// PID pidY(1, 0, 0);            // needs tuning
// PID pidZ(1, 0, 0);            // needs tuning
// PID pidZVelocity(1, 0, 0);    // needs tuning

PID pidRoll(0, 0, 0);      // needs tuning
PID pidPitch(0, 0, 0);     // needs tuning
PID pidHeading(0, 0, 0);   // needs tuning
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
    float pitch{0}, roll{0};
    float heading{0}, tiltX{0}, tiltY{0};
    float x{0}, y{0}, z{0};
    float xVel{0}, yVel{0}, zVel{0};
} state;

struct desState
{
    float pitch{0}, roll{0};
    float heading{0}, tiltX{0}, tiltY{0};
    float x{0}, y{0}, z{0};
    float zVel{0};
} desState;

// ---------- Global state ----------

double loopTime = 0.0;
double prevIMUTimestampUs = 0.0;
int loopCount = 0;
bool stopped = false;

// ---------- data logging ----------

void printWithComma(float value, int precision=3)
{
    Serial.print(value, precision);
    Serial.print(",");
}

void printStatus(float throttle01A, float throttle01B)
{
    // if (!imu.hasData())
    // {
    //     Serial.println(F("[fly] IMU has no data."));
    // }

    printWithComma(throttle01A);
    printWithComma(throttle01B);
    printWithComma(tvcX.commandedServoDeg());
    printWithComma(tvcY.commandedServoDeg());
    printWithComma(static_cast<int>(imu.rotationAccuracy()));

    printWithComma(state.pitch);
    printWithComma(state.roll);
    printWithComma(state.tiltX);
    printWithComma(state.tiltY);
    printWithComma(state.heading);
    printWithComma(state.x);
    printWithComma(state.y);
    printWithComma(state.z);
    printWithComma(state.xVel);
    printWithComma(state.yVel);
    printWithComma(state.zVel);

    printWithComma(desState.pitch);
    printWithComma(desState.roll);
    printWithComma(desState.zVel);
    printWithComma(desState.tiltX);
    printWithComma(desState.tiltY);
    printWithComma(desState.heading);

    printWithComma((loopTime) / 1000000.0, 6);

    Serial.println();
}

// ---------- Main program ----------

// Utility that doesn’t belong to a single device: basic centering & neutralization
void centerTVC()
{
    tvcX.setDesiredThrustDeg(5.0f);
    tvcY.setDesiredThrustDeg(-10.0f);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 5000)
    {
    }
    // Header for CSV logging
    Serial.println("# throttleA,throttleB,tvcX,tvcY,rvAcc,pitch,roll,tiltX,tiltY,heading,x,y,z,xVel,yVel,zVel,desPitch,desRoll,desZVel,desTiltX,desTiltY,desHeading,loopTime");

    // PWM driver for servos/ESC
    pwm.begin(50.0f);

    // Bring servos to neutral
    centerTVC();

    // IMU
    Wire2.begin();          // Teensy 4.1: SDA2=25, SCL2=24
    Wire2.setClock(400000); // start at 100 kHz
    Serial.println("I2C scan on Wire2...");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        Wire2.beginTransmission(addr);
        uint8_t err = Wire2.endTransmission();
        if (err == 0)
        {
            Serial.printf("Found device at 0x%02X\n", addr);
        }
    }

    while (!imu.begin())
    {
        Serial.println(F("[fly] IMU failed to start."));
        delay(1000);
    }
    Serial.println(F("[fly] IMU started"));
    imu.setSensorToBodyEuler(SENSOR_TO_BODY_YAW_DEG, SENSOR_TO_BODY_PITCH_DEG, SENSOR_TO_BODY_ROLL_DEG);

    // Wait for valid data
    imu.update();
    while (!imu.hasData())
    {
        imu.update();
        Serial.println(F("[fly] waiting for IMU data..."));
        delay(500);
    }

    // ref.yaw0 = imu.yawDeg();
    // ref.pitch0 = imu.pitchDeg();
    // ref.roll0 = imu.rollDeg();
    Serial.print(F("[fly] reference orientation: yaw0="));
    Serial.print(ref.yaw0);
    Serial.print(F(", pitch0="));
    Serial.print(ref.pitch0);
    Serial.print(F(", roll0="));
    Serial.println(ref.roll0);
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
        {
            stopped = !stopped;
            if (stopped)
                Serial.println(F("[fly] Stopped"));
            else
                Serial.println(F("[fly] Restarted"));
            return;
        }

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
    // outerloop on position
    if (loopCount % 3 == 0)
    {
        // Compute desired tilt in world frame
        desState.tiltX = pidX.calculate(desState.x, 0, dt);
        desState.tiltY = pidY.calculate(desState.y, 0, dt);

        // Rotate desired tilt from world frame to body frame
        const float headingRad = state.heading * DEG2RAD_F;
        desState.roll = cosf(headingRad) * desState.tiltX + sinf(headingRad) * desState.tiltY;
        desState.pitch = -sinf(headingRad) * desState.tiltX + cosf(headingRad) * desState.tiltY;
        desState.roll = util::clamp(desState.roll, -maxGimble, +maxGimble);
        desState.pitch = util::clamp(desState.pitch, -maxGimble, +maxGimble);
    }
    // innerloop on rotation
    return rotationalPID(dt);
}

// returns desired throttle 0..1
float verticalPID(float dt)
{
    // outerloop on position
    if (loopCount % 3 == 0)
    {
        desState.zVel = pidZ.calculate(desState.z, 0, dt);
    }
    // innerloop on velocity
    float desThrottle = pidZVelocity.calculate(desState.zVel, 0, dt);
    return util::clamp(desThrottle / maxThrottleN, 0.0f, 1.0f);
}

// returns a desired constant, -1..1 to add to ESC 1, subtract from ESC 2
float headingPID(float dt)
{
    float headingError = desState.heading - state.heading;
    if (headingError > 180.0f)
        headingError -= 360.0f;
    if (headingError < -180.0f)
        headingError += 360.0f;

    float out = pidHeading.calculate(0.0f, -headingError, dt); // negative error maybe?
    return out;
}

void updateState()
{

    auto e = imu.euler();
    state.pitch = e.pitch - ref.pitch0;
    state.roll = e.roll - ref.roll0;
    
    auto a = imu.projectedAngles();
    state.tiltX = a.tiltAboutX;
    state.tiltY = a.tiltAboutY;
    state.heading = a.heading;

    // MEKF data for position and rates

    // auto la = imu.accelerometer();
    // state.xAcc = la.x;
    // state.yAcc = la.y;
    // state.zAcc = la.z;
}

void loop()
{
    // For maintaining a steady loop rate
    double loopstartTimestampUS = micros();
    processSerialCommands();
    if (stopped)
    {
        esc1.setMicroseconds(1000);
        esc2.setMicroseconds(1000);
        centerTVC();
        tvcX.update();
        tvcY.update();
        delay(1000);
        return;
    }

    // // Refresh IMU data
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
    float throttle01 = verticalPID(deltaTime); // swap to verticalPID(dt) for altitude hold

    // calculate yaw correction
    float yawAdjust = headingPID(deltaTime);
    float throttle01a = util::clamp(throttle01 + yawAdjust, 0.0f, 1.0f);
    float throttle01b = util::clamp(throttle01 - yawAdjust, 0.0f, 1.0f);

    throttle01a = 0.3;
    throttle01b = 0.3;

    esc1.setThrottle01(throttle01a);
    esc2.setThrottle01(throttle01b);

    // Apply servo updates (slew limiting + PWM)
    tvcX.update();
    tvcY.update();

    // display data 5 times per second, assuming no loop-time overrun
    if (loopCount % static_cast<int>(loopFreq / 0.2) == 0)
    {   
        // printStatus(throttle01a, throttle01b);
    }
    if (loopCount % static_cast<int>(loopFreq / 2) == 0)
    {   
        Serial.println(esc1.lastUs());
    }

    // // Maintain a steady loop rate
    double loopdt = (micros() - loopstartTimestampUS) * 1e-6; // seconds
    double targetPeriod = 1.0 / loopFreq;
    double remaining = targetPeriod - loopdt;
    loopCount++;
    if (remaining > 0)
    {
        delayMicroseconds(static_cast<uint32_t>(remaining * 1e6));
    }
    loopTime = (micros() - loopstartTimestampUS);
}
