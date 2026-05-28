#include "MuxSensor.h"

// ── Pins ──────────────────────────────────────────────────────────────────────
#define PIN_S0   12
#define PIN_S1   11
#define PIN_S2   10
#define PIN_S3    8
#define PIN_COM  A0

#define LEFT_A    9
#define LEFT_B    6
#define RIGHT_A   3
#define RIGHT_B   5

// ── Sensor logic ──────────────────────────────────────────────────────────────
// true  → getDigital() returns 1 when sensor is ON the line  (POLARITY_DARK_LOW)
// false → getDigital() returns 1 when sensor is OFF the line (POLARITY_DARK_HIGH)
#define SENSOR_ACTIVE_HIGH  true

#if SENSOR_ACTIVE_HIGH
  #define SENSOR_POLARITY POLARITY_DARK_LOW
  #define ON_LINE(v)  ((v) == 1)
#else
  #define SENSOR_POLARITY POLARITY_DARK_HIGH
  #define ON_LINE(v)  ((v) == 0)
#endif

// ── Tuning ────────────────────────────────────────────────────────────────────
#define BASE_SPEED          130
#define KP                  8
#define MIN_TURN_SPEED      30
#define LINE_LOST_TIMEOUT_MS  300UL

// Centroid scale: each channel contributes ch*2, centre = (15)*2/2 = 15
#define SENSOR_CENTRE  ((MUX_NUM_CHANNELS - 1) * 2 / 2)

// ─────────────────────────────────────────────────────────────────────────────
MuxSensor sensor(PIN_S0, PIN_S1, PIN_S2, PIN_S3, PIN_COM, SENSOR_POLARITY);

static int      lastError   = 0;
static uint32_t lineLostAt  = 0;
static bool     lineWasSeen = false;

// ─────────────────────────────────────────────────────────────────────────────
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

// Print sensor bar: '-' = on line, '.' = not on line
static void printSensorBar(const uint8_t digital[MUX_NUM_CHANNELS]) {
  Serial.print(F("S:["));
  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ch++) {
    Serial.print(ON_LINE(digital[ch]) ? 'X' : '.');
  }
  Serial.print(F("] "));
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  Serial.println(F("Start MCU"));

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

  // Print calibration table
  Serial.println(F("\nCH  MIN   [black zone]   MAX   [white zone]"));
  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ch++) {
    uint16_t mn  = sensor.getCalibMin(ch);
    uint16_t mx  = sensor.getCalibMax(ch);
    uint16_t bLo = (mn >= MUX_CALIB_MARGIN) ? mn - MUX_CALIB_MARGIN : 0;
    uint16_t bHi = mn + MUX_CALIB_MARGIN;
    uint16_t wLo = (mx >= MUX_CALIB_MARGIN) ? mx - MUX_CALIB_MARGIN : 0;
    uint16_t wHi = mx + MUX_CALIB_MARGIN;
    if (ch < 10) Serial.print(' ');
    Serial.print(ch);  Serial.print("  ");
    Serial.print(mn);  Serial.print("  ["); Serial.print(bLo); Serial.print('-'); Serial.print(bHi); Serial.print("]   ");
    Serial.print(mx);  Serial.print("  ["); Serial.print(wLo); Serial.print('-'); Serial.print(wHi); Serial.println("]");
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
    if (ON_LINE(digital[ch])) {
      weightedSum += (int32_t)ch * 2;
      blackCount++;
    }
  }

  if (blackCount == 0) {
    if (!lineWasSeen) {
      stopMotors();
      return;
    }
    // Keep last correction for LINE_LOST_TIMEOUT_MS, then stop
    if ((uint32_t)(millis() - lineLostAt) > LINE_LOST_TIMEOUT_MS) {
      stopMotors();
      lastError = 0;
      return;
    }
  } else {
    int centroid = (int)(weightedSum / blackCount);
    lastError   = centroid - SENSOR_CENTRE;   // negative = left, positive = right
    lineLostAt  = millis();
    lineWasSeen = true;
  }

  int correction = KP * lastError;
  int leftSpeed  = constrain(BASE_SPEED + correction, MIN_TURN_SPEED, 255);
  int rightSpeed = constrain(BASE_SPEED - correction, MIN_TURN_SPEED, 255);

  motorForward(LEFT_A,  LEFT_B,  leftSpeed);
  motorForward(RIGHT_A, RIGHT_B, rightSpeed);

  printSensorBar(digital);
  Serial.print(F("err=")); Serial.print(lastError);
  Serial.print(F(" L="));  Serial.print(leftSpeed);
  Serial.print(F(" R="));  Serial.println(rightSpeed);
}
