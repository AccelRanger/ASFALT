// AccelRanger
#include "MuxSensor.h"

// line sensor pins
#define PIN_S0   12
#define PIN_S1   11
#define PIN_S2   10
#define PIN_S3    8
#define PIN_COM  A0

// #define STOP_BT 7

MuxSensor sensor(PIN_S0, PIN_S1, PIN_S2, PIN_S3, PIN_COM, POLARITY_DARK_LOW);
uint8_t digital[MUX_NUM_CHANNELS];

// motor pins
#define LEFT_A    6
#define LEFT_B    9
#define RIGHT_A   5
#define RIGHT_B   3

#define EDGE_LEFT   0
#define EDGE_RIGHT  1
#define EDGE_SIDE   EDGE_LEFT     // EDGE_LEFT/EDGER_RIGHT

#if EDGE_SIDE == EDGE_LEFT
  const uint8_t edgeChannels[] = {0, 1, 2, 3};
#else
  const uint8_t edgeChannels[] = {12, 13, 14, 15};
#endif
const uint8_t edgeChannelCount = sizeof(edgeChannels) / sizeof(edgeChannels[0]);

// (index * 1000). 4 channels 0-3000
#define EDGE_SETPOINT      500     // tune here?
#define EDGE_MIN_SIGNAL    150     // lost here?

int avoidedCNT = 0; // placeholder

// PID config
int   baseSpeed          = 160;
float kp                 = 0.07f;
float ki                 = 0.0005f;
float kd                 = 2.8f;
int   sharpTurnThreshold = 40;
int   minTurnSpeed       = 80;
float iClamp             = 800.0f;

// recovery ( my beloved save me )
int leftLost  = 130;
int rightLost = 255;

// pid stuff
int   last_error = 0;
float integral   = 0.0f;

// motor helper 
void setMotors(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);
  analogWrite(LEFT_A,  left  > 0 ?  left  : 0);
  analogWrite(LEFT_B,  left  < 0 ? -left  : 0);
  analogWrite(RIGHT_A, right > 0 ?  right : 0);
  analogWrite(RIGHT_B, right < 0 ? -right : 0);
}
void stop() { setMotors(0, 0); }

bool lineVisible() { // pls work
  sensor.getDigital(digital);
  for (uint8_t i = 0; i < MUX_NUM_CHANNELS; i++) {
    if (digital[i]) return true;
  }
  return false;
}

int readEdgePosition() {
  uint16_t raw[MUX_NUM_CHANNELS];
  sensor.getRawAnalogValues(raw);

  long weightedSum  = 0;
  long weightTotal  = 0;

  for (uint8_t i = 0; i < edgeChannelCount; i++) {
    uint8_t ch = edgeChannels[i];

    uint16_t calMin = sensor.getCalibMin(ch);
    uint16_t calMax = sensor.getCalibMax(ch);
    if (calMax <= calMin) continue; // uncalibrated

    int norm = map(raw[ch], calMin, calMax, 0, 1000);
    norm = constrain(norm, 0, 1000);

    int blackness = 1000 - norm;

    weightedSum += (long)blackness * ((long)i * 1000);
    weightTotal += blackness;
  }

  if (weightTotal < EDGE_MIN_SIGNAL) return -1; // lost
  return (int)(weightedSum / weightTotal);
}

// ── Adaptive speed ────────────────────────────────────
int getAdaptiveSpeed(int error) {
  int absError = abs(error);
  if (absError <= sharpTurnThreshold) return baseSpeed;
  float t = (float)(absError - sharpTurnThreshold) / (3000 - sharpTurnThreshold);
  t = constrain(t, 0.0f, 1.0f);
  return (int)(baseSpeed - (baseSpeed - minTurnSpeed) * t);
}

// ── PID step (edge-follow) ────────────────────────────
void pidStep() {
  int position = readEdgePosition();

  if (position < 0) {
    // Edge lost: fall back to a gentle recovery turn.
    // NOTE: direction may need flipping depending on EDGE_SIDE once tested.
    setMotors(leftLost, rightLost);
    return;
  }

  int error = position - EDGE_SETPOINT;

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
  // pinMode(STOP_BT,     INPUT_PULLUP);   // -- disabled for now --

  sensor.begin();
  stop();

  Serial.println("Calibrating...");
  digitalWrite(LED_BUILTIN, HIGH);
  bool ok = sensor.calibrate(12000UL);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println(ok ? "Calibration OK" : "Calibration low-contrast!");

  delay(1000);
}

// loop
void loop() {
  // if (digitalRead(STOP_BT) == LOW) {
  //   stop();
  //   Serial.print("pressed");
  //   while (true);
  // }

  pidStep();
}
