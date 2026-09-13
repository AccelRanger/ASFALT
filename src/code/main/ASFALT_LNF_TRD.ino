// AccelRanger

#include "MuxSensor.h"

// line sensor pins
#define PIN_S0   12
#define PIN_S1   11
#define PIN_S2   10
#define PIN_S3    8
#define PIN_COM  A0

#define STOP_BT 7

MuxSensor sensor(PIN_S0, PIN_S1, PIN_S2, PIN_S3, PIN_COM, POLARITY_DARK_LOW);
uint8_t digital[MUX_NUM_CHANNELS];

// motor pins
#define LEFT_A    6
#define LEFT_B    9
#define RIGHT_A   5
#define RIGHT_B   3

#define FOLLOW_RIGHT_EDGE   1    // 1 = RIGHT; 0 = LEFT

#if FOLLOW_RIGHT_EDGE
  int edgeSetpoint = 12000;   // roughly half the right-hand sensors on black
#else
  int edgeSetpoint = 3000;    // roughly half the left-hand sensors on black
#endif

#define LOST_ERROR_MAG 7500

// PID config
int   baseSpeed          = 160;
float kp                 = 0.07f;
float ki                 = 0.0005f;
float kd                 = 2.8f;
int   sharpTurnThreshold = 40;
int   minTurnSpeed       = 80;
float iClamp             = 800.0f;

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
  if (activeCount == 0)               return -1;
  if (activeCount == MUX_NUM_CHANNELS) return -2;
  return (int)(weightedSum / activeCount);
}

// ── Adaptive speed ────────────────────────────────────
int getAdaptiveSpeed(int error) {
  int absError = abs(error);
  if (absError <= sharpTurnThreshold) return baseSpeed;
  float t = (float)(absError - sharpTurnThreshold) / (LOST_ERROR_MAG - sharpTurnThreshold);
  t = constrain(t, 0.0f, 1.0f);
  return (int)(baseSpeed - (baseSpeed - minTurnSpeed) * t);
}

// ── PID step (edge-following) ────────────────────────
void pidStep() {
  int position = readPosition();
  int error;

  if (position == -1) {
    #if FOLLOW_RIGHT_EDGE
      error = LOST_ERROR_MAG;
    #else
      error = -LOST_ERROR_MAG;
    #endif
  } else if (position == -2) {

    #if FOLLOW_RIGHT_EDGE
      error = -LOST_ERROR_MAG;
    #else
      error = LOST_ERROR_MAG;
    #endif
  } else {
    error = position - edgeSetpoint;
  }

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
  //if (digitalRead(STOP_BT) == LOW) {
  //  stop();
  //  Serial.print("pressed");
  //  while (true);
  //}

  pidStep();
}