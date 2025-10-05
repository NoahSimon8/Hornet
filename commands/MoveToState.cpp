#include <Arduino.h>
#include "hardware/IMU.h"
#include "hardware/Potentiometer.h"
#include "hardware/PWMDriver.h"
#include "hardware/ESC.h"
#include "hardware/TVCServo.h"
#include "util/Math.h"
#include "utl/pid.h"

// ---------- Pins / channels (adjust to your wiring) ----------
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
TVCServo::Linkage linkX{/*L1*/ 44, /*L2*/ 76.8608, /*L3*/ 75.177, /*L4*/ 28, /*beta0*/  77.98, /*theta0*/  77.98};
TVCServo::Linkage linkY{/*L1*/ 44, /*L2*/ 76.8608, /*L3*/ 75.177, /*L4*/ 28, /*beta0*/  77.98, /*theta0*/  77.98};

TVCServo tvcX(pwm, CH_SERVO_X, linkX, 0, 180, 1000, 2000, 2.0f);
TVCServo tvcY(pwm, CH_SERVO_Y, linkY, 0, 180, 1000, 2000, 2.0f);

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

// Optionally zero the IMU frame at startup pose
void captureReferenceIfNeeded()
{
    if (ref.set || !imu.hasData())
        return;
    auto e = imu.euler();
    ref = {e.yaw, e.pitch, e.roll, true};
    Serial.println(F("[fly] reference orientation captured."));
}

void setup()
{
    Serial.begin(115200);
    delay(200);

    // I/O
    pot.begin();

    // PWM driver for servos/ESC
    pwm.begin(50.0f);

    // Bring servos to neutral
    tvcX.begin(90.0f);
    tvcY.begin(90.0f);

    // IMU
    if (!imu.begin())
    {
        Serial.println(F("[fly] IMU failed to start."));
        while (true)
        {
            delay(1000);
        }
    }
    Serial.println(F("[fly] IMU ok."));

    // Arm ESCs at minimum
    esc1.arm(1000, 1000);
    esc2.arm(1000, 1000);
    esc1.setMicroseconds(1000);
    esc2.setMicroseconds(1000);

    Serial.println(F("[fly] setup complete."));
}

void loop()
{
    // Refresh sensors
    imu.update();
    captureReferenceIfNeeded();

    // Read pilot throttle from pot (no PID)
    uint16_t throttleUs = pot.readMicroseconds(1000, 2000);
    esc1.setMicroseconds(throttleUs);
    esc2.setMicroseconds(throttleUs);

    // Example TVC behavior *without* PID:
    // - keep servos near neutral but add small manual offsets if desired.
    //   Here we simply center them (they still slew toward neutral in update()).
    centerTVC();

    // Apply servo updates (slew limiting + PWM)
    tvcX.update();
    tvcY.update();

    // Print a quick status line
    printStatus(throttleUs);

    // delay(10); // ~100 Hz loop
}
