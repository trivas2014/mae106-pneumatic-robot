/*
*  MAE 106 FINAL PROJECT - Group 13 
*  Version 12.0: Optimized Navigation with ISR
*  STRATEGY: 
*    - Leg 1: Fast start to center (48").
*    - Leg 2: Controlled turn then max speed in final stretch (550").
*    - Leg 3: Bonus maneuver (120" West).
*    - Dynamic Speed: Fast on straights, Slow on turns to prevent tipping [2].
*/

#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include <Servo.h>

// --- 1. PROPULSION TIMING ---
int pistonOnTime       = 150;  
int pistonOffTimeFast  = 180;   // High speed for straights
int pistonOffTimeSlow  = 300;   // Stability for turns

// --- 2. NAVIGATION SETTINGS ---
float headingThreshold = 15.0;  // Error threshold to trigger slow-down, degrees
float distToCenter     = 48.0;  
float distToFinish     = 800.0; 
float INCHES_PER_TICK  = 8.51 / 2.0; 

// --- 3. HARDWARE ---
const int SOLENOID_PIN = 2;
const int SERVO_PIN    = 3;
const int REED_PIN     = 2; // ISR requires Pin 2 or 3 on Uno

LIS3MDL mag;
LSM6 imu;
Servo steeringServo;
// --- Calibrations --- MUST DO IF MAGNETOMETER READS WEIRD
LIS3MDL::vector<int16_t> m_min = {+6278, +418, +3526};
LIS3MDL::vector<int16_t> m_max = {+8590, +761, +6945};

// --- 4. GLOBALS & ISR VARIABLES ---
volatile int tickCount = 0;      // Must be volatile for ISR use
unsigned long lastTickTime = 0;
float targetHeading = 49.0;      // Starting Leg 1 Angle
float filteredHeading = 0.0;
int currentLeg = 1;
bool courseFinished = false;
unsigned long lastPistonCycle = 0;
bool pistonState = LOW;

/**
 * ISR: handleReedSwitch
 * Why we use an ISR instead of polling: Polling the Reed switch in the main loop 
 * can cause "missed ticks" if the processor is busy with complex magnetometer 
 * filtering or serial diagnostics. The ISR ensures that every wheel rotation 
 * is captured immediately by interrupting the main code execution, providing 
 * much higher distance-tracking accuracy 
 */
void handleReedSwitch() {
  if (millis() - lastTickTime > 100) { // Debounce
    tickCount++;
    lastTickTime = millis();
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(REED_PIN, INPUT_PULLUP);
  
  // Attach ISR to the Reed Switch pin
  attachInterrupt(digitalPinToInterrupt(REED_PIN), handleReedSwitch, FALLING);
  
  steeringServo.attach(SERVO_PIN);
  if (!mag.init() || !imu.init()) { while (1); }
  mag.enableDefault();
  imu.enableDefault();

  // Wiggle at start to confirm ready
  steeringServo.write(60); delay(500);
  steeringServo.write(120); delay(500);
  steeringServo.write(90); delay(500);
  delay(3000); 
}

void loop() {
  if (courseFinished) {
    digitalWrite(SOLENOID_PIN, LOW);
    return;
  }

  // 1. DISTANCE CALCULATION
  noInterrupts(); // Temporarily disable to read volatile variable safely
  float currentDist = tickCount * INCHES_PER_TICK;
  interrupts();

  // 2. NAVIGATION STATE MACHINE [6]
  if (currentLeg == 1 && currentDist >= distToCenter) {
    targetHeading = 139.0; // Angle for Leg 2
    noInterrupts(); tickCount = 0; interrupts();
    currentLeg = 2;
  } else if (currentLeg == 2 && currentDist >= distToFinish) {
    courseFinished = true;
  }

  // 3. SENSOR FILTERING
  mag.read(); imu.read();
  float rawHeading = computeHeading();
  float diff = rawHeading - filteredHeading;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  filteredHeading += 0.8 * diff; // Filtered to 20% alpha [7]

  // 4. DYNAMIC PROPULSION [8]
  unsigned long now = millis();
  float error = targetHeading - filteredHeading;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  // Slow down if steering error is high to prevent tipping [2]
  int currentOffTime = (abs(error) <= headingThreshold) ? pistonOffTimeFast : pistonOffTimeSlow;

  if (pistonState == LOW && (now - lastPistonCycle >= currentOffTime)) {
    pistonState = HIGH;
    digitalWrite(SOLENOID_PIN, HIGH);
    lastPistonCycle = now;
  } else if (pistonState == HIGH && (now - lastPistonCycle >= pistonOnTime)) {
    pistonState = LOW;
    digitalWrite(SOLENOID_PIN, LOW);
    lastPistonCycle = now;
  }

  // 5. STEERING
  int servoAngle = 90 + (int)(error * -0.25);
  steeringServo.write(constrain(servoAngle, 60, 120));
}

float computeHeading() {
  LIS3MDL::vector<int> from = {1, 0, 0}; // X-Axis Forward 
  LIS3MDL::vector<int32_t> temp_m = {mag.m.x, mag.m.y, mag.m.z}; 
  LIS3MDL::vector<int16_t> a = {imu.a.x, imu.a.y, imu.a.z}; 
  
  // Apply hard-iron offsets
  temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2; 
  temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2; 
  temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2; 
  
  LIS3MDL::vector<float> E, N; 
  LIS3MDL::vector_cross(&temp_m, &a, &E); 
  LIS3MDL::vector_normalize(&E); 
  LIS3MDL::vector_cross(&a, &E, &N); 
  LIS3MDL::vector_normalize(&N); 
  
  float h = atan2(LIS3MDL::vector_dot(&E, &from), LIS3MDL::vector_dot(&N, &from)) * 180 / PI; 
  if (h < 0) h += 360; 
  return h;
}
