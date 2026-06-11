// AccelRanger

#include "MuxSensor.h"
#include <Wire.h>
#include <VL53L1X.h>

// ToF config
VL53L1X tof;
#define OBSTACLE_MM          135
#define TOF_INTERVAL_MS       15   // match the timing budget (50 000 µs = 50 ms)
#define OBSTACLE_CONFIRM       1

bool     tofOk           = false;
uint16_t tofDistance     = 9999;
uint32_t tofLastRead     = 0;
uint8_t  obstacleCounter = 0;

// line sensor pins
#define PIN_S0   12
#define PIN_S1   11
#define PIN_S2   10
#define PIN_S3    8
#define PIN_COM  A0

#define STOP_BT 7

#define AVOID_TURN_SPEED 100

MuxSensor sensor(PIN_S0, PIN_S1, PIN_S2, PIN_S3, PIN_COM, POLARITY_DARK_LOW);
uint8_t digital[MUX_NUM_CHANNELS];

// motor pins
#define LEFT_A    6
#define LEFT_B    9
#define RIGHT_A   5
#define RIGHT_B   3

// PID config
int   baseSpeed          = 160;
float kp                 = 0.07f;
float ki                 = 0.0005f;
float kd                 = 2.8f;
int   sharpTurnThreshold = 40;
int   minTurnSpeed       = 80;
float iClamp             = 800.0f;

int leftLost = 130;
int rightLost = 255;

int avoidedCNT = 0;

// PID state
int   last_error = 0;
float integral   = 0.0f;

// ── Motor helpers ─────────────────────────────────────
void setMotors(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);
  analogWrite(LEFT_A,  left  > 0 ?  left  : 0);
  analogWrite(LEFT_B,  left  < 0 ? -left  : 0);
  analogWrite(RIGHT_A, right > 0 ?  right : 0);
  analogWrite(RIGHT_B, right < 0 ? -right : 0);
}
void stop() { setMotors(0, 0); }

// ── ToF: returns true on a fresh valid reading ────────
bool updateToF() {
  if (!tofOk) return false;
  if (millis() - tofLastRead < TOF_INTERVAL_MS) return false;
  if (!tof.dataReady()) return false;

  tof.read(false);
  tofLastRead = millis();

  if (tof.ranging_data.range_status == VL53L1X::RangeValid && tof.ranging_data.range_mm > 5) {
    tofDistance = tof.ranging_data.range_mm;
    //Serial.print("distance TOF: ");
    //Serial.println(tofDistance);
    return true;   // ← caller knows a real reading just landed
  }
  return false;
}

// ── Obstacle detection: only call with a fresh reading ──
bool obstacleDetected(bool freshReading) {
  if (!freshReading) return false;

  if (tofDistance < OBSTACLE_MM) {
    obstacleCounter++;
    Serial.print("Obstacle counter: ");
    Serial.print(obstacleCounter);
    Serial.print("  dist=");
    Serial.println(tofDistance);
  } else {
    obstacleCounter = 0;
  }

  return obstacleCounter >= OBSTACLE_CONFIRM;
}

// ── Line present? (any sensor active) ────────────────
bool lineVisible() {
  sensor.getDigital(digital);
  for (uint8_t i = 0; i < MUX_NUM_CHANNELS; i++) {
    if (digital[i]) return true;
  }
  return false;
}

// ── Weighted position ─────────────────────────────────
int readPosition() {
  sensor.getDigital(digital);
  long weightedSum = 0;
  int  activeCount = 0;
  for (uint8_t i = 0; i < MUX_NUM_CHANNELS; i++) {
    if (digital[i]) {
      weightedSum += (long)i * 1000;
      activeCount++;
    }
  }
  if (activeCount == 0) return -1;
  return (int)(weightedSum / activeCount);
}

// ── Adaptive speed ────────────────────────────────────
int getAdaptiveSpeed(int error) {
  int absError = abs(error);
  if (absError <= sharpTurnThreshold) return baseSpeed;
  float t = (float)(absError - sharpTurnThreshold) / (7500 - sharpTurnThreshold);
  t = constrain(t, 0.0f, 1.0f);
  return (int)(baseSpeed - (baseSpeed - minTurnSpeed) * t);
}

void avoidObstacle() {
  Serial.println("new");
  delay(150);
  setMotors(255, -255); // turn right
  delay(150);
  setMotors(0,0);
  delay(250);
  setMotors(255,255);
  delay(500);
  setMotors(0,0);
  delay(250);
  setMotors(-255,255);
  delay(225);
  setMotors(0,0);
  delay(100);
  setMotors(255,255);
  delay(400);
  setMotors(0,0);

  Serial.println("Scanning for line...");
  uint32_t scanStart = millis();
  bool found = false;
  while (millis() - scanStart < 500) {
    if (lineVisible()) {
      found = true;
      break;
    }
    setMotors(AVOID_TURN_SPEED, AVOID_TURN_SPEED / 2);
    delay(20);
  }

  if (!found) {
    Serial.println("Line not found, spinning...");
    scanStart = millis();
    while (millis() - scanStart < 500) {
      setMotors(AVOID_TURN_SPEED, -AVOID_TURN_SPEED);
      delay(20);
      if (lineVisible()) break;
    }
  }

  stop();
  delay(50);

  // Reset PID state so there's no kick from stale error
  last_error     = 0;
  integral       = 0.0f;
  obstacleCounter = 0;

  tof.stopContinuous();
  delay(5);  // brief settle
  tof.startContinuous(TOF_INTERVAL_MS);
  Serial.println("Avoidance done, resuming line follow.");
}

// ── PID step ──────────────────────────────────────────
void pidStep() {

  int position = readPosition();
  if (position < 0) {
    setMotors(leftLost, rightLost);
    return;
 }

  int error = position - 7500;

  if (abs(error) < abs(last_error)) {
    integral *= 0.85f;
  }

  integral += error;
  integral  = constrain(integral, -iClamp, iClamp);

  int correction = (int)(kp * error)
                 + (int)(ki * integral)
                 + (int)(kd * (error - last_error));

  int speed = getAdaptiveSpeed(error);
  setMotors(speed + correction, speed - correction);
  last_error = error;
}

// ── Setup ─────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  pinMode(LEFT_A,      OUTPUT);
  pinMode(LEFT_B,      OUTPUT);
  pinMode(RIGHT_A,     OUTPUT);
  pinMode(RIGHT_B,     OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(STOP_BT,     INPUT_PULLUP);

  Wire.begin();
  Wire.setClock(400000);

  tof.setTimeout(500);
  if (tof.init()) {
    tof.setDistanceMode(VL53L1X::Short);
    tof.setMeasurementTimingBudget(50000);
    tof.startContinuous(TOF_INTERVAL_MS);
    tofOk = true;
    Serial.println("VL53L1X OK");
  } else {
    Serial.println("VL53L1X not found!");
  }

  sensor.begin();
  stop();

  Serial.println("Calibrating...");
  digitalWrite(LED_BUILTIN, HIGH);
  bool ok = sensor.calibrate(12000UL);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println(ok ? "Calibration OK" : "Calibration low-contrast!");

  delay(1000);
}

// ── Loop ──────────────────────────────────────────────
void loop() {
  if (millis() >= 30000) {
   leftLost = 255;
   rightLost = 100;
   digitalWrite(LED_BUILTIN, HIGH);
  }

  bool fresh = updateToF();
  if (digitalRead(STOP_BT) == LOW) {
    stop();
    Serial.print("pressed");
    while (true);
  }

  if (obstacleDetected(fresh)) {
    stop();
    avoidObstacle();
  } else {
    pidStep();
  }
}