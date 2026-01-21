#include <Arduino.h>
#include <array>
#include <Wire.h>
#include "hardware/IMU.h"
#include "hardware/PWMDriver.h"
#include "hardware/ESC.h"
#include "hardware/TVCServo.h"
#include "hardware/LIDAR.h"
#include "util/Math.h"
#include "util/PID.h"
#include "util/mekf2.h"

// ---------- Pins / channels ----------
// Teensy 4.1 direct PWM outputs (no PCA9685)
// TVC servos:
constexpr uint8_t PIN_SERVO_X = 12;
constexpr uint8_t PIN_SERVO_Y = 11;
// ESCs:
constexpr uint8_t PIN_ESC1 = 9;
constexpr uint8_t PIN_ESC2 = 10;

// Logical channels inside PWMDriver (kept so ESC/TVCServo code stays the same)
constexpr uint8_t CH_SERVO_X = 0;
constexpr uint8_t CH_SERVO_Y = 1;
constexpr uint8_t CH_ESC1 = 2;
constexpr uint8_t CH_ESC2 = 3;

constexpr uint8_t PIN_POT = A2;

// ---------- Global hardware objects ----------

// ---------- Constants ----------

const float maxThrottleN = 2.5; // newtons, this is an estimate
const float maxGimble = 12.0;   // degrees, this is an estimate
const float maxTiltDeg = 15.0;  // What we dont want the rocket to tilt more than (deg)
const float angleXNeutral = 0.0f;
const float angleYNeutral = 0.0f;
const double loopFreq = 100.0; // Hz
constexpr float DEG2RAD_F = 0.0174532925f;

// Mounting offset between IMU sensor frame and rocket body frame (deg)
float SENSOR_TO_BODY_YAW_DEG = -9.0f;
constexpr float SENSOR_TO_BODY_PITCH_DEG = 90.0f;
constexpr float SENSOR_TO_BODY_ROLL_DEG = 180.0f;

// ---------- Global hardware objects ----------
PWMDriver pwm;
ESC esc1(pwm, CH_ESC1, 1120, 2000);
ESC esc2(pwm, CH_ESC2, 1120, 2000);
IMU imu(0x4B, Wire);
LIDAR lidarX(Wire2); // Not actually sure which one is X or Y, need to check later by cross-referencing TVC movement
// LIDAR lidarY(Wire1);  <---- and this isn't currently plugged in

// Linkage numbers — copy your real values here
TVCServo::Linkage linkX{/*L1*/ 44, /*L2*/ 76.8608, /*L3*/ 75.177, /*L4*/ 28, /*beta0*/ 77.98, /*theta0*/ 77.98};
TVCServo::Linkage linkY{/*L1*/ 44, /*L2*/ 76.8608, /*L3*/ 75.177, /*L4*/ 28, /*beta0*/ 77.98, /*theta0*/ 77.98};
TVCServo tvcX(pwm, CH_SERVO_X, linkX, -12, 12, -90, 90, -90, 90, 500, 2500, 2.0f, angleXNeutral, -1);
TVCServo tvcY(pwm, CH_SERVO_Y, linkY, -12, 12, -90, 90, -90, 90, 500, 2500, 2.0f, angleYNeutral, +1);

// ---------- PID controllers ----------
// PID pidRoll(0.8, 0.05, 0.1);  // needs tuning
// PID pidPitch(0.8, 0.05, 0.1); // needs tuning
// PID pidHeading(0.8, 0.0, 0.0);    // needs tuning
// PID pidX(1, 0, 0);            // needs tuning
// PID pidY(1, 0, 0);            // needs tuning
// PID pidZ(1, 0, 0);            // needs tuning
// PID pidZVelocity(1, 0, 0);    // needs tuning

PID pidRoll(0.3, 0.01, 0.0);  // needs tuning 0.8, 0.1, 0.0
PID pidPitch(0.3, 0.01, 0.0); // needs tuning
PID pidHeading(0, 0, 0);      // needs tuning
PID pidX(0, 0, 0);            // needs tuning
PID pidY(0, 0, 0);            // needs tuning
PID pidZ(0, 0, 0);            // needs tuning
PID pidZVelocity(0, 0, 0);    // needs tuning
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
bool stopped = true;
float targetThrottlePower = 0.0;
float throttlePower = 0.0;
float throttleRateLimitPerSecond = 0.2; // how fast we can change throttle (per second)
int calibrationMode = 0;                  // 0=normal, 1=high pwm, 2=low pwm
// ---------- Host heartbeat failsafe ----------
// The PC-side live plotter sends "hb" lines periodically.
// If we stop receiving them for > HEARTBEAT_TIMEOUT_MS (after we've seen at least one),
// we force stopped=true to put outputs into a safe state.
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 800; // must be > heartbeat period on PC
static uint32_t lastHeartbeatMs = 0;
static bool heartbeatSeen = false;
static bool heartbeatLostLatched = false;

static inline void noteHeartbeat()
{
    lastHeartbeatMs = millis();
    heartbeatSeen = true;
}

static inline void checkHeartbeatFailsafe()
{
    if (!heartbeatSeen)
        return; // don't enforce until we've seen the host at least once
    if (stopped)
        return; // already stopped
    const uint32_t now = millis();
    if ((uint32_t)(now - lastHeartbeatMs) > HEARTBEAT_TIMEOUT_MS)
    {
        stopped = true;
        if (!heartbeatLostLatched)
        {
            heartbeatLostLatched = true;
            // NOTE: This line will show up in the plotter "console" pane (non-CSV)
            Serial.println(F("[fly] Heartbeat lost -> STOPPED"));
        }
    }
}

// ---------- data logging ----------

void printWithComma(float value, int precision = 3)
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

    printWithComma(esc1.lastUs());
    printWithComma(throttle01A);
    printWithComma(tvcX.commandedThrustDeg());
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
    tvcX.setDesiredThrustDeg(0);

    tvcY.setDesiredThrustDeg(0);
}
void resetPIDs()
{
    pidHeading.reset();
    pidRoll.reset();
    pidPitch.reset();
    pidX.reset();
    pidY.reset();
    pidZ.reset();
    pidZVelocity.reset();
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 5000)
    {
    }

#ifdef TEENSYDUINO
    Serial.print("TEENSYDUINO defined, version = ");
    Serial.println(TEENSYDUINO);
#else
    Serial.println("TENSYDUINO NOT defined");
#endif

#ifdef ARDUINO_TEENSY41
    Serial.println("ARDUINO_TEENSY41 defined");
#else
    Serial.println("ARDUINO_TEENSY41 NOT defined");
#endif

    // PWM driver for servos/ESCx
    pwm.begin(50.0f);
    tvcX.begin();
    tvcY.begin();

    // Attach logical channels to Teensy pins
    pwm.attach(CH_SERVO_X, PIN_SERVO_X, 1500);
    pwm.attach(CH_SERVO_Y, PIN_SERVO_Y, 1500);
    pwm.attach(CH_ESC1, PIN_ESC1, 1000);
    pwm.attach(CH_ESC2, PIN_ESC2, 1000);

    // Bring servos to neutral
    centerTVC();
    // // Arm ESCs at minimum
    esc1.arm(1000, 1000);
    esc2.arm(1000, 1000);
    esc1.setMicroseconds(1000);
    esc2.setMicroseconds(1000);

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


    lidarX.update();    
    while (!lidarX.hasData())
    {
        lidarX.update();
        Serial.println(F("[fly] waiting for LIDAR data..."));
        delay(500);
    }

    pidRoll.setIntegralLimits(-maxGimble / 3, maxGimble / 3);
    pidPitch.setIntegralLimits(-maxGimble / 3, maxGimble / 3);

    Serial.println(F("[fly] setup complete."));

    // Header for CSV logging
    Serial.println("# throttleA,throttleB,tvcX,tvcY,rvAcc,pitch,roll,tiltX,tiltY,heading,x,y,z,xVel,yVel,zVel,desPitch,desRoll,desZVel,desTiltX,desTiltY,desHeading,loopTime");
}

// Parse a token that might be "kp=...", "ki=...", "kd=...", or single-char commands.
void parseToken(const String &token)
{

    if (token.startsWith("t="))
    {
        String val = token.substring(3);
        float thPow = val.toFloat();
        if (thPow != 0.0 || val == "0" || val == "0.0")
        {
            targetThrottlePower = thPow;
            Serial.print("tp= ");
            Serial.println(thPow);
        }
    }
    else if (token.equals("]"))
    {
        targetThrottlePower += 0.005;
        Serial.print("tp= ");
        Serial.println(targetThrottlePower);
    }
    else if (token.equals("["))
    {
        targetThrottlePower -= 0.005;
        Serial.print("tp= ");
        Serial.println(targetThrottlePower);
    }
    else if (token.equals("r-pid"))
    {
        resetPIDs();
        Serial.println("PIDs reset.");
    }
    else if (token.equals("r-h"))
    {
        SENSOR_TO_BODY_YAW_DEG += imu.yawDeg();
        imu.setSensorToBodyEuler(SENSOR_TO_BODY_YAW_DEG, SENSOR_TO_BODY_PITCH_DEG, SENSOR_TO_BODY_ROLL_DEG);
    }
    else if (token.startsWith("kp="))
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
    else if (token.startsWith("sx="))
    {
        String val = token.substring(3);
        val.trim();
        if (val.equalsIgnoreCase("auto"))
        {
            tvcX.clearManualOverride();
            Serial.println(F("TVC X manual override disabled"));
        }
        else
        {
            float angle = val.toFloat();
            if (angle != 0.0f || val == "0" || val == "0.0")
            {
                tvcX.setDesiredThrustDeg(angle);
                Serial.print(F("TVC X manual angle set to "));
                Serial.println(angle);
            }
        }
    }
    else if (token.startsWith("sy="))
    {
        String val = token.substring(3);
        val.trim();
        if (val.equalsIgnoreCase("auto"))
        {
            tvcY.clearManualOverride();
            Serial.println(F("TVC Y manual override disabled"));
        }
        else
        {
            float angle = val.toFloat();
            if (angle != 0.0f || val == "0" || val == "0.0")
            {
                tvcY.setDesiredThrustDeg(angle);
                Serial.print(F("TVC Y manual angle set to "));
                Serial.println(angle);
            }
        }
    }
    else if (token.equals("cal"))
    {
        calibrationMode += 1;
        if (calibrationMode > 2)
        {
            calibrationMode = 0;
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
        // Any line from the host counts as a heartbeat (including "hb")
        noteHeartbeat();
        if (input.length() == 0)
        {
            stopped = !stopped;
            if (stopped)
                Serial.println(F("[fly] Stopped"));
            else
            {
                // allow future heartbeat-loss message after a manual restart
                heartbeatLostLatched = false;
                Serial.println(F("[fly] Restarted"));
            }
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
        desState.zVel = pidZ.calculate(desState.z, state.z, dt);
    }
    // innerloop on velocity
    float desThrottle = pidZVelocity.calculate(desState.zVel, state.zVel, dt);
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
    imu.update();
    lidarX.update();

    if (imu.hasData())
    {
        // IMU orientation data
        auto e = imu.euler();
        state.pitch = e.pitch - ref.pitch0;
        state.roll = e.roll - ref.roll0;
    
        auto a = imu.projectedAngles();
        state.tiltX = a.tiltAboutX;
        state.tiltY = a.tiltAboutY;
        state.heading = a.heading;
    }

    if (lidar.hasData())
    {
        // LIDAR altitude data
        float prevZ = state.z;
        state.z = lidar.getDistance() * 0.01f; // convert cm to m
        state.zVel = (state.z - prevZ) / loopFreq; // shoddy velocity measure (temperary) m/s
    }

    // MEKF data for position and rates

    // auto la = imu.accelerometer();
    // state.xAcc = la.x;
    // state.yAcc = la.y;
    // state.zAcc = la.z;
}

void calibration(){
    switch (calibrationMode)
    {
    case 1:
        // high pwm for ESC calibration
        esc1.setMicroseconds(2000);
        esc2.setMicroseconds(2000);
        break;
    case 2:
        // low pwm for ESC calibration
        esc1.setMicroseconds(1000);
        esc2.setMicroseconds(1000);
        break;
    default:
        calibrationMode = 0;
        break;
    }
}

void loop()
{
    double loopstartTimestampUS = micros();
    processSerialCommands();
    checkHeartbeatFailsafe(); // checks for computer disconnection
    updateState(); // updates state variabels with sensor data

    if (calibrationMode){
        calibration();
        centerTVC();
        tvcX.update();
        tvcY.update();
        throttlePower = 0;
        targetThrottlePower = 0;
        resetPIDs();
        printStatus(0.0f, 0.0f);
        return;   
    }    

    if (stopped)
    {
        esc1.setMicroseconds(1000);
        esc2.setMicroseconds(1000);
        centerTVC();
        tvcX.update();
        tvcY.update();
        throttlePower = 0;
        targetThrottlePower = 0;
        resetPIDs();
        printStatus(0.0f, 0.0f);
        delay(50);
        return;
    }

    // record timestamp for PID use
    double IMUTimestampUs = micros();
    double deltaTime = (IMUTimestampUs - prevIMUTimestampUs) * 1e-6; // seconds
    prevIMUTimestampUs = IMUTimestampUs;

    // get desired thrust angles
    std::array<float, 2> pidOut = lateralPID(deltaTime); // level flight at origin

    // // Apply to servos as desired thrust angles
    tvcX.setDesiredThrustDeg(pidOut[0]);
    tvcY.setDesiredThrustDeg(pidOut[1]);

    // // Apply servo updates (slew limiting + PWM)
    tvcX.update();
    tvcY.update();

    // get base throttle
    float throttle01 = verticalPID(deltaTime); // swap to verticalPID(dt) for altitude hold

    targetThrottlePower = util::clamp(throttle01, 0.0f, 1.0f); // for testing only

    // Rate Limiter for Throttle Power
    if (throttlePower < targetThrottlePower)
    {
        throttlePower += throttleRateLimitPerSecond * deltaTime;
        if (throttlePower > targetThrottlePower)
            throttlePower = targetThrottlePower;
    }
    else if (throttlePower > targetThrottlePower)
    {
        throttlePower -= throttleRateLimitPerSecond * deltaTime;
        if (throttlePower < targetThrottlePower)
            throttlePower = targetThrottlePower;
    }

    // calculate yaw correction
    float yawAdjust = headingPID(deltaTime);

    float throttle01a = util::clamp(throttlePower + yawAdjust, 0.0f, 1.0f);
    float throttle01b = util::clamp(throttlePower - yawAdjust, 0.0f, 1.0f);
    throttle01a = 0.0;
    throttle01b = 0.0;


    esc1.setThrottle01(throttle01a);
    esc2.setThrottle01(throttle01b);
    esc1.update();
    esc2.update();

    // don't want PID gaining windup when throttle is zero
    if (throttlePower == 0.0)
    {
        resetPIDs();
    }

    // display data x times per second, as in loopFreq / xf, assuming no loop-time overrun

    if (loopCount % static_cast<int>(loopFreq / 20) == 0)
    {
        printStatus(throttle01a, throttle01b);
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