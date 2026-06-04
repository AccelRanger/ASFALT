// AccelRanger

#include "MuxSensor.h"
#include <Wire.h>
#include <VL53L1X.h>

// tof config
VL53L1X tof;
#define OBSTACLE_MM          200
#define TOF_INTERVAL_MS       33
#define OBSTACLE_CONFIRM       8

bool     tofOk           = false;
uint16_t tofDistance     = 9999;
uint32_t tofLastRead     = 0;
uint8_t  obstacleCounter = 0;
bool     freshReading    = false;

// line sensor pins
#define PIN_S0   12
#define PIN_S1   11
#define PIN_S2   10
#define PIN_S3    8
#define PIN_COM  A0

// sensor initialization
MuxSensor sensor(PIN_S0, PIN_S1, PIN_S2, PIN_S3, PIN_COM, POLARITY_DARK_LOW);
uint8_t digital[MUX_NUM_CHANNELS];

// motor pins
#define LEFT_A    6
#define LEFT_B    9
#define RIGHT_A   5
#define RIGHT_B   3

// PID config
int   baseSpeed          = 255;
float kp                 = 0.10f;
float ki                 = 0.001f;
float kd                 = 1.5f;
int   sharpTurnThreshold = 50;
int   minTurnSpeed       = 40;
float iClamp             = 3000.0f;

// config for obstacle avoidance
#define AVOID_BACK_MS     300
#define AVOID_TURN_MS     400
#define AVOID_FWD_MS      600
#define AVOID_RETURN_MS   400
#define AVOID_TURN_SPEED  180

// pid states
int   last_error = 0;
float integral   = 0.0f;

// motor helpers
void setMotors(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);
  analogWrite(LEFT_A,  left  > 0 ?  left  : 0);
  analogWrite(LEFT_B,  left  < 0 ? -left  : 0);
  analogWrite(RIGHT_A, right > 0 ?  right : 0);
  analogWrite(RIGHT_B, right < 0 ? -right : 0);
}

void stop() { setMotors(0, 0); }

// non blocking tof
void updateToF() {
  freshReading = false;
  if (!tofOk) return;
  if (millis() - tofLastRead < 50) return;
  if (!tof.dataReady()) return;

  tof.read(false);
  tofLastRead = millis();

  if (tof.ranging_data.range_status == VL53L1X::RangeValid
      && tof.ranging_data.range_mm > 20) {
    tofDistance  = tof.ranging_data.range_mm;
    freshReading = true;
  }
}

bool obstacleDetected() {
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

// weighted position
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

// adaptive speed
int getAdaptiveSpeed(int error) {
  int absError = abs(error);
  if (absError <= sharpTurnThreshold) return baseSpeed;
  float t = (float)(absError - sharpTurnThreshold) / (7500 - sharpTurnThreshold);
  t = constrain(t, 0.0f, 1.0f);
  return (int)(baseSpeed - (baseSpeed - minTurnSpeed) * t * t);
}

// obstacle avoidance
void avoidObstacle() {
  Serial.println("Obstacle! Avoiding...");

  setMotors(-baseSpeed, -baseSpeed);
  delay(AVOID_BACK_MS);

  setMotors(AVOID_TURN_SPEED, -AVOID_TURN_SPEED);
  delay(AVOID_TURN_MS);

  setMotors(baseSpeed, baseSpeed);
  delay(AVOID_FWD_MS);

  setMotors(-AVOID_TURN_SPEED, AVOID_TURN_SPEED);
  delay(AVOID_RETURN_MS);

  last_error = 0;
  integral   = 0.0f;

  tof.startContinuous(TOF_INTERVAL_MS);

  Serial.println("Avoidance done, resuming line follow.");
}

// pid step
void pidStep() {
  int position = readPosition();
  if (position < 0) {
    setMotors(baseSpeed, baseSpeed);
    return;
  }

  int error = position - 7500;
  integral += error;
  integral  = constrain(integral, -iClamp, iClamp);

  int correction = (int)(kp * error)
                 + (int)(ki * integral)
                 + (int)(kd * (error - last_error));

  int speed = getAdaptiveSpeed(error);
  setMotors(speed + correction, speed - correction);
  last_error = error;
}

// setup
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
    tof.setMeasurementTimingBudget(50000);
    tof.startContinuous(50);
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
  updateToF();

  if (obstacleDetected()) {
    stop();
    obstacleCounter = 0;
    avoidObstacle();
  } else {
    pidStep();
  }
}