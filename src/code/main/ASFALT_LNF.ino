#include "MuxSensor.h"

// ── Sensor ────────────────────────────────────────────
#define PIN_S0   12
#define PIN_S1   11
#define PIN_S2   10
#define PIN_S3    8
#define PIN_COM  A0

MuxSensor sensor(PIN_S0, PIN_S1, PIN_S2, PIN_S3, PIN_COM, POLARITY_DARK_LOW);
uint8_t  digital[MUX_NUM_CHANNELS];

// ── Motors ────────────────────────────────────────────
#define LEFT_A    6   // PWM  left  motor
#define LEFT_B    9   // DIR  left  motor
#define RIGHT_A   5   // PWM  right motor
#define RIGHT_B   3   // DIR  right motor

// ── Tuning ────────────────────────────────────────────
int   baseSpeed = 255;
float kp        = 0.10f;
float ki        = 0.002f;   // integral gain — start small, increase if drift persists
float kd        = 1.5f;

int   sharpTurnThreshold = 50;
int   minTurnSpeed       = 40;
float iClamp             = 3000.0f;  // anti-windup: caps the integral term

// ── State ─────────────────────────────────────────────
int   last_error    = 0;
float integral      = 0.0f;

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

// ── Weighted position (0 – 15000, centre = 7500) ──────
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
  return (int)(baseSpeed - (baseSpeed - minTurnSpeed) * t * t);
}

// ── PID step ──────────────────────────────────────────
void pidStep() {
  int position = readPosition();

  // Line lost → go straight forward and freeze the integrator
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

// ── Setup ─────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  pinMode(LEFT_A,      OUTPUT);
  pinMode(LEFT_B,      OUTPUT);
  pinMode(RIGHT_A,     OUTPUT);
  pinMode(RIGHT_B,     OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

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
  pidStep();
}