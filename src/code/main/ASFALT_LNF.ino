// ASFALT LINE FOLLOWER CORE

/*
 * PID tuning guide
 *   1. Set Ki = Kd = 0. Raise Kp until the robot oscillates on a straight line.
 *   2. Halve Kp. Raise Kd until oscillation is damped.
 *   3. Add a small Ki only if the robot drifts to one side on a long straight.
 */

#include "MuxSensor.h"

// MUX DEFINES
#define PIN_S0   12
#define PIN_S1   11
#define PIN_S2   10
#define PIN_S3    8
#define PIN_COM  A0

// MOTORS
#define LEFT_A   5
#define LEFT_B   3
#define RIGHT_A  9
#define RIGHT_B  6

// SPEED
#define BASE_SPEED   100
#define MAX_SPEED    200
#define MIN_SPEED      0

// PID CONFIG
const float KP = 25.0f;
const float KI =  0.0f;
const float KD = 10.0f;

static const uint8_t SENS_FIRST = 0;
static const uint8_t SENS_LAST  = 15;
static const uint8_t SENS_COUNT = SENS_LAST - SENS_FIRST + 1; // 16 channel sensor

// variables
MuxSensor sensor(PIN_S0, PIN_S1, PIN_S2, PIN_S3, PIN_COM, POLARITY_DARK_LOW);

float         g_lastError  = 0.0f;
float         g_integral   = 0.0f;
unsigned long g_lastTimeUs = 0;

//std:clamp ( stupid arduino ide )
static int iclamp(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void setMotor(uint8_t pinA, uint8_t pinB, int speed) {
  speed = iclamp(speed, 0, 255);
  analogWrite(pinA, speed);
  analogWrite(pinB, 0);
}

static void stopMotors() {
  analogWrite(LEFT_A,  0); analogWrite(LEFT_B,  0);
  analogWrite(RIGHT_A, 0); analogWrite(RIGHT_B, 0);
}

//  PID strcuture
//    -(SENS_COUNT-1)/2  …  +(SENS_COUNT-1)/2
//  i.e. –7.5 … +7.5 for 16 channels.
//    0        = line perfectly centred
//    positive = line to the RIGHT of centre
//    negative = line to the LEFT  of centre


static float computePosition(const uint8_t digital[MUX_NUM_CHANNELS]) {
  static float lastPos = 0.0f;

  long  weightedSum = 0;
  uint8_t count     = 0;

  for (uint8_t i = SENS_FIRST; i <= SENS_LAST; i++) {
    if (digital[i]) {
      int centred = (int)(i - SENS_FIRST) * 2 - (int)(SENS_COUNT - 1);
      weightedSum += centred;
      count++;
    }
  }

  if (count == 0) return lastPos; // line lost

  lastPos = (float)weightedSum / (2.0f * count);
  return lastPos;
}

//calibratiion feedback
static void blinkLED(int times, int onMs, int offMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(onMs);
    digitalWrite(LED_BUILTIN, LOW);  delay(offMs);
  }
}


// setup


void setup() {
  Serial.begin(9600);
  Serial.println(F("LineFollower starting..."));

  // Output pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEFT_A,  OUTPUT);
  pinMode(LEFT_B,  OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);
  stopMotors();

  // init sensor

  sensor.begin();

  // ── Calibration ───────────────────────────────────────────────────────────
  Serial.println(F("=== CALIBRATION ==="));
  blinkLED(3, 150, 150); // "ready" signal: 3 quick blinks

  bool ok = sensor.calibrate(12000UL); // blocking – 12 seconds

  if (ok) {
    Serial.println(F("=== Calibration OK ==="));
    blinkLED(6, 60, 60);  // rapid burst = success
  } else {
    Serial.println(F("=== Calibration WARNING ==="));
    blinkLED(3, 500, 200); // slow blinks = warning
  }

 // Serial.println(F("\nCH  MIN   MAX   THR"));
  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ch++) {
    if (ch < 10) Serial.print(' ');
    Serial.print(ch);    Serial.print("  ");
    uint16_t mn = sensor.getCalibMin(ch);
    uint16_t mx = sensor.getCalibMax(ch);
    uint16_t th = sensor.getThreshold(ch);
    if (mn < 100) Serial.print(' ');
    if (mn <  10) Serial.print(' ');
    Serial.print(mn);    Serial.print("   ");
    if (mx < 100) Serial.print(' ');
    if (mx <  10) Serial.print(' ');
    Serial.print(mx);    Serial.print("   ");
    Serial.println(th);
  }

  digitalWrite(LED_BUILTIN, HIGH);
  g_lastTimeUs = micros();
  Serial.println(F("\n=== RUNNING ==="));
}


// loop


void loop() {
  uint8_t digital[MUX_NUM_CHANNELS];

  if (!sensor.getDigital(digital)) {
    stopMotors();
    return;
  }

  // time delta
  unsigned long nowUs = micros();
  float dt = (float)(nowUs - g_lastTimeUs) * 1e-6f; // seconds
  if (dt < 1e-5f) dt = 1e-5f;                        // guard against zero
  g_lastTimeUs = nowUs;

  // ── Position & error ──────────────────────────────────────────────────────
  float error = computePosition(digital);

  // pid
  g_integral += error * dt;
  g_integral  = constrain(g_integral, -60.0f, 60.0f); // anti-windup

  float derivative = (error - g_lastError) / dt;
  g_lastError = error;

  float correction = KP * error + KI * g_integral + KD * derivative;

  // ── Motor speeds ──────────────────────────────────────────────────────────
  int leftSpeed  = iclamp((int)(BASE_SPEED + correction), MIN_SPEED, MAX_SPEED);
  int rightSpeed = iclamp((int)(BASE_SPEED - correction), MIN_SPEED, MAX_SPEED);

  setMotor(LEFT_A,  LEFT_B,  leftSpeed);
  setMotor(RIGHT_A, RIGHT_B, rightSpeed);

  // Serial.print(error,2);        Serial.print('\t');
  // Serial.print(correction,2);   Serial.print('\t');
  // Serial.print(leftSpeed);      Serial.print('\t');
  // Serial.println(rightSpeed);
}
