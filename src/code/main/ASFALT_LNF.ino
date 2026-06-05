// AccelRanger

#include "MuxSensor.h"
#include <Wire.h>
#include <VL53L1X.h>

// ── ToF config ────────────────────────────────────────
VL53L1X tof;

#define OBSTACLE_MM          200
#define TOF_TIMING_BUDGET_US 50000UL   // 50 ms in microseconds (VL53L1X API uses µs)
#define TOF_INTERVAL_MS        50UL    // poll interval — must match budget
#define OBSTACLE_CONFIRM        4      // consecutive readings needed to trigger

bool     tofOk           = false;
uint16_t tofDistance     = 9999;
uint32_t tofLastRead     = 0;
uint8_t  obstacleCounter = 0;

// ── Line sensor pins ──────────────────────────────────
#define PIN_S0   12
#define PIN_S1   11
#define PIN_S2   10
#define PIN_S3    8
#define PIN_COM  A0

MuxSensor sensor(PIN_S0, PIN_S1, PIN_S2, PIN_S3, PIN_COM, POLARITY_DARK_LOW);
uint8_t digital[MUX_NUM_CHANNELS];

#define POSITION_CENTER  7500

// ── Motor pins ────────────────────────────────────────
#define LEFT_A    6
#define LEFT_B    9
#define RIGHT_A   5
#define RIGHT_B   3

// ── PID config ────────────────────────────────────────
int   baseSpeed          = 255;
float kp                 = 0.07f;
float ki                 = 0.0005f;
float kd                 = 2.8f;
int   sharpTurnThreshold = 2000;
int   minTurnSpeed       = 160;
float iClamp             = 800.0f;

// ── PID state ─────────────────────────────────────────
int   last_error  = 0;
float integral    = 0.0f;
int   lastValidError = 0;

#define AVOID_TURN_SPEED 100

enum AvoidState {
  AVOID_IDLE,
  AVOID_BACK,
  AVOID_TURN_LEFT,
  AVOID_FWD1,
  AVOID_TURN_RIGHT,
  AVOID_FWD2,
  AVOID_SCAN_ARC,
  AVOID_SCAN_SPIN,
  AVOID_DONE
};

AvoidState avoidState   = AVOID_IDLE;
uint32_t   avoidStateTs = 0;

// ── Motor helpers ─────────────────────────────────────
void setMotors(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);
  analogWrite(LEFT_A,  left  > 0 ?  left  : 0);
  analogWrite(LEFT_B,  left  < 0 ? -left  : 0);
  analogWrite(RIGHT_A, right > 0 ?  right : 0);
  analogWrite(RIGHT_B, right < 0 ? -right : 0);
}
void stopMotors() { setMotors(0, 0); }

bool updateToF() {
  if (!tofOk) return false;
  if (millis() - tofLastRead < TOF_INTERVAL_MS) return false;
  if (!tof.dataReady()) return false;

  tof.read(false);
  tofLastRead = millis();

  if (tof.ranging_data.range_status == VL53L1X::RangeValid
      && tof.ranging_data.range_mm > 20) {
    tofDistance = tof.ranging_data.range_mm;
    return true;
  }
  return false;
}

bool obstacleDetected(bool freshReading) {
  if (!tofOk || !freshReading) return false;

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

// ── Line helpers ──────────────────────────────────────
bool lineVisible() {
  sensor.getDigital(digital);
  for (uint8_t i = 0; i < MUX_NUM_CHANNELS; i++) {
    if (digital[i]) return true;
  }
  return false;
}

// Returns weighted-average position [0..15000], or -1 if no line.
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
  // max possible error = 7500 (position 0 or 15000 vs centre 7500)
  const int maxError = POSITION_CENTER;
  float t = (float)(absError - sharpTurnThreshold) / (maxError - sharpTurnThreshold);
  t = constrain(t, 0.0f, 1.0f);
  return (int)(baseSpeed - (baseSpeed - minTurnSpeed) * t);
}

bool runAvoidance() {
  uint32_t now = millis();

  switch (avoidState) {

    case AVOID_BACK:
      setMotors(-200, -200);
      if (now - avoidStateTs >= 500) {
        avoidState   = AVOID_TURN_LEFT;
        avoidStateTs = now;
        Serial.println("Avoid: turning left");
      }
      break;

    case AVOID_TURN_LEFT:
      setMotors(-200, 200);
      if (now - avoidStateTs >= 500) {
        avoidState   = AVOID_FWD1;
        avoidStateTs = now;
        Serial.println("Avoid: forward past obstacle");
      }
      break;

    case AVOID_FWD1:
      setMotors(200, 200);
      if (now - avoidStateTs >= 500) {
        avoidState   = AVOID_TURN_RIGHT;
        avoidStateTs = now;
        Serial.println("Avoid: arc right");
      }
      break;

    case AVOID_TURN_RIGHT:
      setMotors(200, -200);
      if (now - avoidStateTs >= 500) {
        avoidState   = AVOID_FWD2;
        avoidStateTs = now;
        Serial.println("Avoid: forward toward line");
      }
      break;

    case AVOID_FWD2:
      setMotors(200, 200);
      if (now - avoidStateTs >= 500) {
        avoidState   = AVOID_SCAN_ARC;
        avoidStateTs = now;
        Serial.println("Avoid: scanning for line (arc)");
      }
      break;

    case AVOID_SCAN_ARC:
      if (lineVisible()) {
        avoidState = AVOID_DONE;
        break;
      }
      setMotors(AVOID_TURN_SPEED, AVOID_TURN_SPEED / 2);
      if (now - avoidStateTs >= 500) {
        avoidState   = AVOID_SCAN_SPIN;
        avoidStateTs = now;
        Serial.println("Avoid: line not found, spinning");
      }
      break;

    case AVOID_SCAN_SPIN:
      if (lineVisible()) {
        avoidState = AVOID_DONE;
        break;
      }
      setMotors(AVOID_TURN_SPEED, -AVOID_TURN_SPEED);
      if (now - avoidStateTs >= 1500) {
        Serial.println("Avoid: line not found after spin, giving up");
        avoidState = AVOID_DONE;
      }
      break;

    // ── 8. Cleanup ──
    case AVOID_DONE:
      stopMotors();
      delay(50);

      last_error      = 0;
      lastValidError  = 0;
      integral        = 0.0f;
      obstacleCounter = 0;

      if (tofOk) {
        tof.stopContinuous();
        tof.startContinuous(TOF_TIMING_BUDGET_US);
      }
      tofLastRead = millis();

      avoidState = AVOID_IDLE;
      Serial.println("Avoidance done, resuming line follow.");
      return true;

    default:
      avoidState = AVOID_IDLE;
      break;
  }

  return false;
}

// ── PID step ──────────────────────────────────────────
void pidStep() {
  int position = readPosition();

  int error;
  if (position < 0) {
    error = lastValidError;
  } else {
    error          = position - POSITION_CENTER;
    lastValidError = error;
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

  Wire.begin();
  Wire.setClock(400000);

  tof.setTimeout(500);
  if (tof.init()) {
    tof.setDistanceMode(VL53L1X::Short);
    tof.setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);
    tof.startContinuous(TOF_TIMING_BUDGET_US);
    tofOk       = true;
    tofLastRead = millis();
    Serial.println("VL53L1X OK");
  } else {
    Serial.println("VL53L1X not found — obstacle avoidance disabled");
  }

  sensor.begin();
  stopMotors();

  Serial.println("Calibrating...");
  digitalWrite(LED_BUILTIN, HIGH);
  bool ok = sensor.calibrate(12000UL);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println(ok ? "Calibration OK" : "Calibration low-contrast!");

  delay(1000);
}

// ── Loop ──────────────────────────────────────────────
void loop() {
  bool fresh = updateToF();

  if (avoidState != AVOID_IDLE) {
    runAvoidance();
    return;
  }

  if (obstacleDetected(fresh)) {
    stopMotors();
    Serial.println("Obstacle confirmed! Starting avoidance.");
    tof.stopContinuous();
    avoidState   = AVOID_BACK;
    avoidStateTs = millis();
    return;
  }

  pidStep();
}
