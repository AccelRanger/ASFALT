#include "MuxSensor.h"

// ── Tuning ───────────────────────────────────────────────────────────────────
#define BASE_SPEED            160   // cruise speed (0-255)
#define MAX_SPEED             255   // absolute motor cap
#define MIN_DRIVE_SPEED        60   // minimum forward speed (prevents stall)
#define REVERSE_SPEED          80   // speed used when spinning to recover line

// PID gains  (tune KP first, then KD, then KI last)
#define KP                   10.0f
#define KI                    0.04f
#define KD                    6.0f
#define INTEGRAL_CLAMP       60.0f  // prevents integral windup

// Dynamic speed: BASE_SPEED is scaled down when |error| is large
// effectiveBase = BASE_SPEED - SPEED_REDUCE_K * |error|  (clamped to MIN_DRIVE_SPEED)
#define SPEED_REDUCE_K         4.0f

// Line-lost recovery: spin for up to this long before giving up
#define LINE_LOST_TIMEOUT_MS  600UL

#define SERIAL_DEBUG           1

// ── Pins ─────────────────────────────────────────────────────────────────────
#define PIN_S0   12
#define PIN_S1   11
#define PIN_S2   10
#define PIN_S3    8
#define PIN_COM  A0

// Motor pins: each motor has a FORWARD and BACKWARD PWM pin
// Speed the motor forward  → analogWrite(FWD, speed), analogWrite(BWD, 0)
// Speed the motor backward → analogWrite(FWD, 0),     analogWrite(BWD, speed)
#define PIN_LEFT_FWD   9
#define PIN_LEFT_BWD   6
#define PIN_RIGHT_FWD  3
#define PIN_RIGHT_BWD  5

// ─────────────────────────────────────────────────────────────────────────────
// POLARITY_DARK_HIGH → dark (black) sensor reads a higher raw ADC value.
// Flip to POLARITY_DARK_LOW if your board is wired the other way.
MuxSensor sensor(PIN_S0, PIN_S1, PIN_S2, PIN_S3, PIN_COM, POLARITY_DARK_HIGH);

// ── Motor helpers ─────────────────────────────────────────────────────────────
// Signed speed: positive = forward, negative = backward, 0 = coast
static void motorSet(uint8_t pinFwd, uint8_t pinBwd, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    analogWrite(pinFwd, speed);
    analogWrite(pinBwd, 0);
  } else {
    analogWrite(pinFwd, 0);
    analogWrite(pinBwd, -speed);
  }
}

static void driveMotors(int leftSpeed, int rightSpeed) {
  motorSet(PIN_LEFT_FWD,  PIN_LEFT_BWD,  leftSpeed);
  motorSet(PIN_RIGHT_FWD, PIN_RIGHT_BWD, rightSpeed);
}

static void stopMotors() {
  analogWrite(PIN_LEFT_FWD,  0); analogWrite(PIN_LEFT_BWD,  0);
  analogWrite(PIN_RIGHT_FWD, 0); analogWrite(PIN_RIGHT_BWD, 0);
}

static void blinkLED(int n, int onMs, int offMs) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(onMs);
    digitalWrite(LED_BUILTIN, LOW);  delay(offMs);
  }
}

#if SERIAL_DEBUG
static void printDebug(float error, float correction, int lSpd, int rSpd) {
  Serial.print(F("err="));
  Serial.print(error, 2);
  Serial.print(F(" cor="));
  Serial.print(correction, 1);
  Serial.print(F(" L=")); Serial.print(lSpd);
  Serial.print(F(" R=")); Serial.println(rSpd);
}
#endif

// ── State ─────────────────────────────────────────────────────────────────────
static float    pidIntegral   = 0.0f;
static float    prevError     = 0.0f;
static float    lastGoodError = 0.0f;   // sign tells us which side line was on
static uint32_t lineLostAt    = 0;
static bool     lineWasSeen   = false;
static uint32_t lastLoopUs    = 0;      // for dt calculation

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_LEFT_FWD,  OUTPUT); pinMode(PIN_LEFT_BWD,  OUTPUT);
  pinMode(PIN_RIGHT_FWD, OUTPUT); pinMode(PIN_RIGHT_BWD, OUTPUT);
  stopMotors();

  sensor.begin();

  Serial.println(F("CALIBRATING 12 s: sweep sensor over the full track surface..."));
  blinkLED(3, 150, 150);

  bool ok = sensor.calibrate(12000UL);

  if (ok) {
    Serial.println(F("Calibration OK"));
    blinkLED(6, 60, 60);
  } else {
    Serial.println(F("Calibration WARNING — results may be poor"));
    blinkLED(3, 500, 200);
  }

  // Print calibration table
  Serial.println(F("\nCH  MIN  MAX"));
  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ch++) {
    if (ch < 10) Serial.print(' ');
    Serial.print(ch);
    Serial.print(F("   "));
    Serial.print(sensor.getCalibMin(ch));
    Serial.print(F("   "));
    Serial.println(sensor.getCalibMax(ch));
  }

  lastLoopUs = micros();
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println(F("=== RUNNING ==="));
}

void loop() {
  // ── Delta-time for the integral term (seconds) ────────────────────────────
  uint32_t now  = micros();
  float    dt   = (float)(now - lastLoopUs) * 1e-6f;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;  // clamp runaway dt on first loop
  lastLoopUs = now;

  // ── Read analog values ────────────────────────────────────────────────────
  // getAnalog() returns calibrated 0..1000 per channel (1000 = darkest = line).
  // We use the analog signal directly so the centroid is smooth and continuous.
  uint16_t analog[MUX_NUM_CHANNELS];
  if (!sensor.getAnalog(analog)) {
    stopMotors();
    return;
  }

  // ── Weighted centroid (analog) ────────────────────────────────────────────
  // Positions 0..15, centre = 7.5
  // error > 0 → line is to the RIGHT of centre → must steer right
  // error < 0 → line is to the LEFT  of centre → must steer left
  int32_t  weightedSum  = 0;
  int32_t  totalWeight  = 0;

  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ch++) {
    weightedSum += (int32_t)analog[ch] * ch;
    totalWeight += analog[ch];
  }

  // ── Line-lost handling ────────────────────────────────────────────────────
  if (totalWeight < 200) {   // threshold: all sensors see white
    if (!lineWasSeen) {
      stopMotors();
      return;
    }

    uint32_t nowMs = millis();

    if ((uint32_t)(nowMs - lineLostAt) > LINE_LOST_TIMEOUT_MS) {
      // Gave up — full stop and reset PID so it doesn't lurch on next find
      stopMotors();
      pidIntegral = 0.0f;
      prevError   = 0.0f;
      return;
    }

    // Spin toward the side the line was last seen on.
    // lastGoodError > 0 → line was right → spin right (left fwd, right back)
    // lastGoodError < 0 → line was left  → spin left  (right fwd, left back)
    if (lastGoodError >= 0) {
      driveMotors( REVERSE_SPEED, -REVERSE_SPEED);
    } else {
      driveMotors(-REVERSE_SPEED,  REVERSE_SPEED);
    }
    return;
  }

  // ── Good centroid reading ─────────────────────────────────────────────────
  float centroid = (float)weightedSum / (float)totalWeight;  // 0.0 .. 15.0
  float error    = centroid - 7.5f;                          // -7.5 .. +7.5

  lastGoodError = error;
  lineLostAt    = millis();
  lineWasSeen   = true;

  // ── PID ───────────────────────────────────────────────────────────────────
  pidIntegral += error * dt;
  pidIntegral  = constrain(pidIntegral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);

  float derivative = (error - prevError) / dt;
  prevError = error;

  float correction = KP * error
                   + KI * pidIntegral
                   + KD * derivative;

  // ── Dynamic base speed (slow on sharp bends) ──────────────────────────────
  float absErr      = fabsf(error);
  int   effectiveBase = (int)(BASE_SPEED - SPEED_REDUCE_K * absErr);
  effectiveBase = max(effectiveBase, MIN_DRIVE_SPEED);

  // error > 0 → line RIGHT → increase rightSpeed, decrease leftSpeed
  int leftSpeed  = effectiveBase - (int)correction;
  int rightSpeed = effectiveBase + (int)correction;

  // Clamp to signed range; allows brief reversal on very sharp turns
  leftSpeed  = constrain(leftSpeed,  -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  driveMotors(leftSpeed, rightSpeed);

#if SERIAL_DEBUG
  printDebug(error, correction, leftSpeed, rightSpeed);
#endif
}
