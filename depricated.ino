#define _PWM_LOGLEVEL_ 1

#include "nRF52_PWM.h"
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_BNO08x.h>  // <<-- Replaced LSM6DS3 include with the new BNO08x
#include <math.h>

// === Pin Definitions ===
#define ESC_PIN1 A2  // PWM-capable digital pin connected to ESC1's signal wire
#define ESC_PIN2 A1  // PWM-capable digital pin connected to ESC2's signal wire
#define POT_PIN A3   // Analog pin connected to the potentiometer's wiper

// === PID Constants ===
float kp = 0.8; //0.4;   // Proportional gain (for servos)
float ki = 0.1; //0.1;   // Integral gain (for servos)
float kd = 0.2; //0.4;   // Derivative gain (for servos)

// === PWM Parameters for ESC (nRF52 side, left in place per original code) ===
const float PWM_FREQUENCY_ESC = 50.0f;   // 50Hz for ESCs
const float DUTY_CYCLE_MIN_ESC = 5.0f;   // 5% duty cycle (~1ms pulse)
const float DUTY_CYCLE_MAX_ESC = 10.0f;  // 10% duty cycle (~2ms pulse)
const int PWM_RESOLUTION_ESC = 12;       // 12-bit resolution (0-4095)

// Four-bar linkages constants -- determined by the geometry of the design
const float THETA_INIT = 77.98;
const float BETA_INIT = 77.98;
const float LINK1 = 44.0;
const float LINK2 = 76.8608;
const float LINK3 = 75.177;
const float LINK4 = 28.0;

// === PWM Instances for ESCs (nRF52 side, left in place per original code) ===
nRF52_PWM* PWM_Instance_ESC1;
nRF52_PWM* PWM_Instance_ESC2;

// === Servo PWM Driver and IMU ===
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Replace old LSM6DS3 object with new BNO08x object.
// We keep the name 'myIMU' to avoid changing references elsewhere:
Adafruit_BNO08x myIMU(-1);

// We'll track BNO08x events in this structure:
sh2_SensorValue_t sensorValue;

// For simplicity, we'll enable ARVR_STABILIZED_RV (typical usage). 5000us = ~200Hz
static const sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
static const long reportIntervalUs = 5000;

// === Servo Configuration ===
int angle14 = 108; 
int angle15 = 47; 
const int offset = 24;  // Maximum movement from center in degrees

// === IMU Variables ===
float angleX = 0, angleY = 0;
float gyroX = 0, gyroY = 0;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;  // Store calibrated offsets
float gyroRoll = 0, gyroPitch = 0, gyroYaw = 0;           // Integrated gyro values
float accelAngleX = 0, accelAngleY = 0;
unsigned long lastIMUUpdate = 0;
long lastInterval;  // For tracking time between updates in microseconds

// === PID Variables (Servo tilt) ===
float errorX = 0, errorY = 0;
float lastErrorX = 0, lastErrorY = 0;
float integralX = 0, integralY = 0;
float derivativeX = 0, derivativeY = 0;
unsigned long lastTime_PID = 0;

// === Servo Position Tracking ===
float lastServo14Pos = (float)angle14;
float lastServo15Pos = (float)angle15;
float maxServoChangePerUpdate = 2.0f;

// === Potentiometer Filtering (originally used for old throttle logic) ===
static float filteredPotValue = 0.0f;  // Filtered reading from the potentiometer
const float ALPHA = 0.1f;              // Smoothing factor for the low-pass filter

// === NEW Throttle Logic Defines (PCA9685) ===
#define ESC_CHANNEL_1 12  // ESC on PCA9685 channel 12
#define ESC_CHANNEL_2 13  // ESC on PCA9685 channel 13

// Define ESC pulse width range for PCA9685
#define MIN_PULSE 1000  // 1.0ms (full stop)
#define MAX_PULSE 2000  // 2.0ms (full throttle)
#define FREQUENCY 50    // 50Hz for ESCs

// Variable for manual increment/decrement of pulse width
int manualThrottleAdjustment = 0;

// We'll track the final throttle setting so that we can ramp properly
static int currentThrottle = MIN_PULSE; // Initialize to min throttle

// We define a small struct to hold yaw/pitch/roll from the BNO08x, matching new code usage
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

// Store initial IMU orientation so that at boot, that becomes the zero reference
float xRef = 0;
float yRef = 0;

// --- Yaw PID (plus/minus) variables ---
// For now Ki and Kd are set to zero, but you can adjust as needed.
float yawKp = 0.2f;
float yawKi = 0.0f;
float yawKd = 0.0f;
float yawIntegral = 0.0f;
float yawDerivative = 0.0f;
float lastYawError = 0.0f;

// We'll also store the last yaw reading (from fused sensor) to compute yaw rate.
float lastYaw = 0.0f;

// Helper to compute minimal angular difference in degrees (handles wrap-around)
float angleDifference(float current, float previous) {
  float diff = current - previous;
  while(diff > 180.0f)  diff -= 360.0f;
  while(diff < -180.0f) diff += 360.0f;
  return diff;
}

// Helper to convert quaternion to Euler
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0f * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0f * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw   *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll  *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real,
                    rotational_vector->i,
                    rotational_vector->j,
                    rotational_vector->k,
                    ypr,
                    degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real,
                    rotational_vector->i,
                    rotational_vector->j,
                    rotational_vector->k,
                    ypr,
                    degrees);
}

// Function to map angle to pulse length (for servos on PCA9685)
uint16_t angleToPulse(int angle) {
  int pulseMin = 150;
  int pulseMax = 600;
  return map(angle, 0, 180, pulseMin, pulseMax);
}

// Function to set ESC pulse width using PCA9685 (NEW throttle logic)
void setESC(uint8_t channel, int pulseWidth) {
  // Convert microseconds to 12-bit value over 20ms (50Hz)
  int pwmValue = (pulseWidth * 4096) / 20000;
  pwm.setPWM(channel, 0, pwmValue);
}

// Ramps from currentThrottle to targetThrottle in ~20 steps (~200ms total)
void rampThrottleTo(int targetThrottle) {
  int start = currentThrottle;
  int steps = 20;     
  int stepDelay = 10; 

  for (int i = 1; i <= steps; i++) {
    float fraction = (float)i / (float)steps;
    currentThrottle = start + (int)(fraction * (targetThrottle - start));
    currentThrottle = constrain(currentThrottle, MIN_PULSE, MAX_PULSE);

    setESC(ESC_CHANNEL_1, currentThrottle);
    setESC(ESC_CHANNEL_2, currentThrottle);

    delay(stepDelay);
  }
}

// Function to process IMU data (now replaced with BNO08x logic)
// We keep the function name, structure, and comments from the original code.
void processIMUData() {
  long currentTime = micros();
  lastInterval = currentTime - lastIMUUpdate;
  lastIMUUpdate = currentTime;

  // In the new sensor, we check if there's any sensor event ready:
  while (myIMU.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        // More stable at ~200Hz
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        break;
      case SH2_GYRO_INTEGRATED_RV:
        // Potentially faster but noisier
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
      default:
        // ignore other reports
        break;
    }
  }

  // Pitch on new sensor corresponds to X on old code
  // Roll on new sensor corresponds to Y on old code
  // *** Here we reference the initial IMU readings as our 0 orientation ***
  angleX = (-ypr.roll) - xRef;
  angleY = (-ypr.pitch) - yRef;

  // Update PID errors (kept from original code)
  errorX = angleX;
  errorY = angleY;
}

// Function to calibrate IMU at startup
// We keep the same comments and print statements even though BNO08x self-calibrates.
void calibrateIMU() {
  Serial.println("Calibrating IMU - keep the device still...");

  // BNO08x handles offsets internally, so we won't manually collect offsets:
  // We'll just preserve these lines so we don't change comments:
  gyroXOffset = 0;
  gyroYOffset = 0;
  gyroZOffset = 0;

  // Print the same message for consistency:
  Serial.print("Calibration complete! Offsets - X: ");
  Serial.print(gyroXOffset);
  Serial.print(" Y: ");
  Serial.print(gyroYOffset);
  Serial.print(" Z: ");
  Serial.println(gyroZOffset);
}

float calculateInputAngle(float beta_final) {
  float beta_init_rad = radians(BETA_INIT);
  float beta_final_rad = radians(beta_final);

  float L = sqrt(pow(LINK1, 2) + pow(LINK2, 2)
                 - 2 * LINK1 * LINK2 * cos(beta_init_rad + beta_final_rad));

  float p = degrees(acos(-(pow(LINK3, 2) - pow(LINK4, 2) - pow(L, 2)) / (2 * L * LINK4)));
  float q = degrees(acos(-(pow(LINK1, 2) - pow(LINK2, 2) - pow(L, 2)) / (2 * L * LINK2)));
  return 180 - p - q - BETA_INIT;
}

// Function to move servos with PID output
void moveServosWithPID(float outputX, float outputY) {
  // Constrain PID outputs to mechanical limits
  float desiredThrustAngleY = constrain(-outputY, -offset, offset);
  float desiredThrustAngleX = constrain(-outputX, -offset, offset);

  // Convert desired thrust angles to required servo angles using four-bar linkage
  float servoAngleY = calculateInputAngle(desiredThrustAngleY);
  float servoAngleX = calculateInputAngle(desiredThrustAngleX);

  float targetServo14Pos = angle14 - servoAngleY;
  float targetServo15Pos = angle15 + servoAngleX;

  float newServo14Pos = constrain(targetServo14Pos,
                                  lastServo14Pos - maxServoChangePerUpdate,
                                  lastServo14Pos + maxServoChangePerUpdate);
  float newServo15Pos = constrain(targetServo15Pos,
                                  lastServo15Pos - maxServoChangePerUpdate,
                                  lastServo15Pos + maxServoChangePerUpdate);

  lastServo14Pos = newServo14Pos;
  lastServo15Pos = newServo15Pos;

  int servo14Int = (int)round(newServo14Pos);
  int servo15Int = (int)round(newServo15Pos);

  pwm.setPWM(14, 0, angleToPulse(servo14Int));
  pwm.setPWM(15, 0, angleToPulse(servo15Int));

  Serial.print(",desiredThrustY:");
  Serial.print(desiredThrustAngleY);
  Serial.print(",desiredThrustX:");
  Serial.print(desiredThrustAngleX);
}

void setup() {
  // Initialize nRF52 PWM instances (original code - unchanged)
  PWM_Instance_ESC1 = new nRF52_PWM(ESC_PIN1, PWM_FREQUENCY_ESC, DUTY_CYCLE_MAX_ESC);
  PWM_Instance_ESC2 = new nRF52_PWM(ESC_PIN2, PWM_FREQUENCY_ESC, DUTY_CYCLE_MAX_ESC);

  Serial.begin(115200);
  while (!Serial && millis() < 5000)
    ;

  delay(500);

  Serial.println("\n=== Combined Motor and Servo Control with Enhanced IMU Processing ===");

  Serial.println("ESC1 and ESC2 PWMs initialized at 5% duty cycle (1ms pulse) for 3 seconds...");
  // delay(3000);
  Serial.println("ESC setup complete.");

  // Initialize the filteredPotValue with the first potentiometer reading
  filteredPotValue = analogRead(POT_PIN);

  // Initialize PCA9685 for both servos AND ESC channels
  pwm.begin();
  pwm.setPWMFreq(50);

  // Initialize BNO08x in place of old IMU
  if (!myIMU.begin_I2C(0x4B)) {
    Serial.println("IMU Device error");
    while (1)
      ;
  }
  // Enable the rotation vector at ~200Hz
  myIMU.enableReport(reportType, reportIntervalUs);

  calibrateIMU();
  // Acquire one IMU reading so we can store it as our zero reference
  while (!myIMU.getSensorEvent(&sensorValue)) { delay(10); }
  if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
    quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
  } else if (sensorValue.sensorId == SH2_GYRO_INTEGRATED_RV) {
    quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
  }
  xRef = -ypr.roll;
  yRef = -ypr.pitch;
  // Initialize lastYaw with the initial fused heading.
  lastYaw = ypr.yaw;
  
  Serial.println("Reference orientation set to current orientation as zero.");

  moveServosWithPID(0, 0);

  lastIMUUpdate = micros();
  lastTime_PID = micros();

  Serial.println("Ready!");
}

// Parse single-character commands (like 's','a','w','q') which may be repeated.
void parseSingleCharCommands(const String& token) {
  for (int i = 0; i < token.length(); i++) {
    char c = token.charAt(i);
    if (c == 's') {
      manualThrottleAdjustment += 10;
    }
    else if (c == 'a') {
      manualThrottleAdjustment -= 10;
    }
    else if (c == 'w') {
      Serial.println("Ramping to MAX throttle...");
      rampThrottleTo(MAX_PULSE);
      manualThrottleAdjustment = MAX_PULSE - map(analogRead(POT_PIN), 0, 1023, MIN_PULSE, MAX_PULSE);
    }
    else if (c == 'q') {
      Serial.println("Ramping to MIN throttle...");
      rampThrottleTo(MIN_PULSE);
      manualThrottleAdjustment = MIN_PULSE - map(analogRead(POT_PIN), 0, 1023, MIN_PULSE, MAX_PULSE);
    }
    else if ((c == ' ') || (c == '\t')) {
      // ignore whitespace
    }
    else {
      Serial.print("Unknown command: ");
      Serial.println(c);
    }
  }
}

// Parse a token that might be "kp=...", "ki=...", "kd=...", or single-char commands.
void parseToken(const String& token) {
  if (token.startsWith("kp=")) {
    String val = token.substring(3);
    float newKp = val.toFloat();
    if (newKp != 0.0 || val == "0" || val == "0.0") {
      kp = newKp;
      Serial.print("Updated kp to: ");
      Serial.println(kp);
    }
  }
  else if (token.startsWith("ki=")) {
    String val = token.substring(3);
    float newKi = val.toFloat();
    if (newKi != 0.0 || val == "0" || val == "0.0") {
      ki = newKi;
      Serial.print("Updated ki to: ");
      Serial.println(ki);
    }
  }
  else if (token.startsWith("kd=")) {
    String val = token.substring(3);
    float newKd = val.toFloat();
    if (newKd != 0.0 || val == "0" || val == "0.0") {
      kd = newKd;
      Serial.print("Updated kd to: ");
      Serial.println(kd);
    }
  }
  else if (token.startsWith("a=")) {
    String val = token.substring(3);
    float newAlpha = val.toFloat();
    if (newAlpha != 0.0 || val == "0" || val == "0.0") {
      float COMP_ALPHA = newAlpha; // does nothing now but left in for structure
      Serial.print("Updated comp alpha to: ");
      Serial.println(newAlpha);
    }
  }
  // Here we allow m= to change maxServoChangePerUpdate (a float)
  else if (token.startsWith("m=")) {
    String val = token.substring(2);
    float newM = val.toFloat();
    // Accept zero and non-zero floats
    if (newM != 0.0 || val == "0" || val == "0.0") {
      maxServoChangePerUpdate = newM;
      Serial.print("Updated maxServoChangePerUpdate to: ");
      Serial.println(maxServoChangePerUpdate);
    }
  }
  else {
    parseSingleCharCommands(token);
  }
}

// Function to process incoming serial commands
void processSerialCommands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    int startIndex = 0;
    while (true) {
      int spaceIndex = input.indexOf(' ', startIndex);
      String token;
      if (spaceIndex == -1) {
        token = input.substring(startIndex);
        token.trim();
        if (token.length() > 0) {
          parseToken(token);
        }
        break;
      } else {
        token = input.substring(startIndex, spaceIndex);
        token.trim();
        if (token.length() > 0) {
          parseToken(token);
        }
        startIndex = spaceIndex + 1;
      }
    }
  }
}

void loop() {
  processSerialCommands();

  // === NEW Throttle Logic (read potentiometer, map to base throttle) ===
  int potValue = analogRead(POT_PIN);
  int baseThrottle = map(potValue, 0, 1023, MIN_PULSE, MAX_PULSE);
  int finalThrottle = baseThrottle + manualThrottleAdjustment;
  finalThrottle = constrain(finalThrottle, MIN_PULSE, MAX_PULSE);
  currentThrottle = finalThrottle;

  // === Process IMU Data ===
  processIMUData();

  // === PID Control for servo angles (X/Y) ===
  unsigned long currentTime = micros();
  float deltaTime = (currentTime - lastTime_PID) / 1000000.0f;
  lastTime_PID = currentTime;

  integralX = constrain(integralX + errorX * deltaTime, -10, 10);
  integralY = constrain(integralY + errorY * deltaTime, -10, 10);

  float rawDerivativeX = (errorX - lastErrorX) / deltaTime;
  float rawDerivativeY = (errorY - lastErrorY) / deltaTime;

  derivativeX = 0.95f * derivativeX + 0.05f * rawDerivativeX;
  derivativeY = 0.95f * derivativeY + 0.05f * rawDerivativeY;

  float outputX = kp * errorX + ki * integralX + kd * derivativeX;
  float outputY = kp * errorY + ki * integralY + kd * derivativeY;

  lastErrorX = errorX;
  lastErrorY = errorY;

  moveServosWithPID(outputX, outputY);

  // === Yaw Rate PID (plus/minus approach) ===
  // Compute the yaw rate based on the change in fused heading (ypr.yaw).
  float currentYaw = ypr.yaw;
  float dYaw = angleDifference(currentYaw, lastYaw);
  float yawRate = dYaw / deltaTime;  // degrees per second
  lastYaw = currentYaw;
  // Serial.print(",currentYaw:");
  // Serial.print(currentYaw);
  // Serial.print(",yawRate:");
  // Serial.println(yawRate);

  // Use yaw rate as the error (we want it to be zero)
  float yawError = yawRate;  
  yawIntegral += yawError * deltaTime;
  yawIntegral = constrain(yawIntegral, -50.0f, 50.0f);
  float rawYawD = (yawError - lastYawError) / deltaTime;
  yawDerivative = 0.95f * yawDerivative + 0.05f * rawYawD;
  lastYawError = yawError;
  float yawOutput = yawKp * yawError + yawKi * yawIntegral + yawKd * yawDerivative;

  // Split the throttle using the yaw PID output:
  int motor1Output = finalThrottle - (int)round(yawOutput);
  int motor2Output = finalThrottle + (int)round(yawOutput);

  motor1Output = constrain(motor1Output, MIN_PULSE, MAX_PULSE);
  motor2Output = constrain(motor2Output, MIN_PULSE, MAX_PULSE);

  // Send the differential throttle to the ESCs
  setESC(ESC_CHANNEL_1, motor1Output);
  setESC(ESC_CHANNEL_2, motor2Output);

  // === Debug Output ===
  Serial.print("min:");
  Serial.print(-90);
  Serial.print(",max:");
  Serial.print(90);
  Serial.print(",");

  Serial.print("angleX:");
  Serial.print(angleX);
  Serial.print(",angleY:");
  Serial.print(angleY);

  Serial.print(",proportional_x:");
  Serial.print(kp * errorX);
  Serial.print(",integral_x:");
  Serial.print(ki * integralX);
  Serial.print(",derivitive_x:");
  Serial.print(kd * derivativeX);

  Serial.print(",kp:");
  Serial.print(kp);
  Serial.print(",ki:");
  Serial.print(ki);
  Serial.print(",kd:");
  Serial.print(kd);

  Serial.print(",accelX:");
  Serial.print(accelAngleX);
  Serial.print(",accelY:");
  Serial.print(accelAngleY);
  Serial.print(",gyroRoll:");
  Serial.print(gyroRoll);
  Serial.print(",gyroPitch:");
  Serial.print(gyroPitch);
  Serial.print(",gyroYaw:");
  Serial.print(gyroYaw);

  Serial.print(",pidX:");
  Serial.print(outputX);
  Serial.print(",pidY:");
  Serial.print(outputY);

  Serial.print(",m:");
  Serial.print(maxServoChangePerUpdate);
  Serial.print(",potValue:");
  Serial.print(potValue);

  Serial.print(",baseThrottle:");
  Serial.print(finalThrottle);

  Serial.print(",motor1Output:");
  Serial.print(motor1Output);
  Serial.print(",motor2Output:");
  Serial.println(motor2Output);

  // delay(20);  // 50Hz update rate
}
