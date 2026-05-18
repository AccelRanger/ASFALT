#include "MuxSensor.h"

// ── Pins ──────────────────────────────────────────────────────────────────────
#define PIN_S0   12
#define PIN_S1   11
#define PIN_S2   10
#define PIN_S3    8
#define PIN_COM  A0

#define LEFT_A    5
#define LEFT_B    3
#define RIGHT_A   6 
#define RIGHT_B   9 

// ── Tuning ────────────────────────────────────────────────────────────────────
#define BASE_SPEED      130

#define KP               8

#define MIN_TURN_SPEED  30

#define LINE_LOST_TIMEOUT_MS  300UL

#define SENSOR_CENTRE  (MUX_NUM_CHANNELS - 1) * 2 / 2   // 15

// ─────────────────────────────────────────────────────────────────────────────
MuxSensor sensor(PIN_S0, PIN_S1, PIN_S2, PIN_S3, PIN_COM, POLARITY_DARK_LOW);

static void motorForward(uint8_t pinA, uint8_t pinB, int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(pinA, speed);
  analogWrite(pinB, 0);
}

static void stopMotors() {
  analogWrite(LEFT_A,  0); analogWrite(LEFT_B,  0);
  analogWrite(RIGHT_A, 0); analogWrite(RIGHT_B, 0);
}

static void blinkLED(int n, int onMs, int offMs) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(onMs);
    digitalWrite(LED_BUILTIN, LOW);  delay(offMs);
  }
}

// ── State ─────────────────────────────────────────────────────────────────────
static int      lastError      = 0;       // remembered for line-lost recovery
static uint32_t lineLostAt     = 0;       // timestamp when line was last seen
static bool     lineWasSeen    = false;
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  Serial.println("Start MCU");
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEFT_A,  OUTPUT); pinMode(LEFT_B,  OUTPUT);
  pinMode(RIGHT_A, OUTPUT); pinMode(RIGHT_B, OUTPUT);
  stopMotors();

  sensor.begin();

  Serial.println(F("CALIBRATING: sweep robot over black & white for 12 s..."));
  blinkLED(3, 150, 150);

  bool ok = sensor.calibrate(12000UL);

  if (ok) {
    Serial.println(F("Calibration OK"));
    blinkLED(6, 60, 60);
  } else {
    Serial.println(F("Calibration WARNING: zones overlap on some channels"));
    blinkLED(3, 500, 200);
  }

  // Print calibration table for debugging
  Serial.println(F("\nCH  BLACK_RAW  [zone]       WHITE_RAW  [zone]"));
  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ch++) {
    uint16_t mn = sensor.getCalibMin(ch);
    uint16_t mx = sensor.getCalibMax(ch);
    uint16_t bLo = (mn >= MUX_CALIB_MARGIN) ? mn - MUX_CALIB_MARGIN : 0;
    uint16_t bHi = mn + MUX_CALIB_MARGIN;
    uint16_t wLo = (mx >= MUX_CALIB_MARGIN) ? mx - MUX_CALIB_MARGIN : 0;
    uint16_t wHi = mx + MUX_CALIB_MARGIN;
    if (ch < 10) Serial.print(' ');
    Serial.print(ch);  Serial.print("  ");
    Serial.print(mn);  Serial.print("       [");
    Serial.print(bLo); Serial.print("-");
    Serial.print(bHi); Serial.print("]   ");
    Serial.print(mx);  Serial.print("       [");
    Serial.print(wLo); Serial.print("-");
    Serial.print(wHi); Serial.println("]");
  }

  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println(F("\n=== RUNNING ==="));
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  uint8_t digital[MUX_NUM_CHANNELS];

  if (!sensor.getDigital(digital)) {
    stopMotors();
    return;
  }

  int32_t weightedSum = 0;
  int     blackCount  = 0;

  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ch++) {
    if (digital[ch]) {
      weightedSum += (int32_t)ch * 2;
      blackCount++;
    }
  }

  if (blackCount == 0) {
    if (!lineWasSeen) {
      stopMotors();
      return;
    }
    uint32_t now = millis();
    if ((uint32_t)(now - lineLostAt) > LINE_LOST_TIMEOUT_MS) {
      stopMotors();
      lastError = 0;
      return;
    }
  } else {
    int centroid = (int)(weightedSum / blackCount);
    lastError    = centroid - SENSOR_CENTRE;          // negative=left, positive=right
    lineLostAt   = millis();
    lineWasSeen  = true;
  }

  // ── Proportional speed calculation ─────────────────────────────────────────
  int correction = KP * lastError;

  int leftSpeed  = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  if (blackCount > 0) {
    leftSpeed  = constrain(leftSpeed,  MIN_TURN_SPEED, 255);
    rightSpeed = constrain(rightSpeed, MIN_TURN_SPEED, 255);
  }

  motorForward(LEFT_A,  LEFT_B,  leftSpeed);
  motorForward(RIGHT_A, RIGHT_B, rightSpeed);

  Serial.print(F("err=")); Serial.print(lastError);
  Serial.print(F(" L="));  Serial.print(leftSpeed);
  Serial.print(F(" R="));  Serial.println(rightSpeed);
}