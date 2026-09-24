#include "MuxSensor.h"

void setMotors(int left, int right);
void stop();
int  findEdge();
void detectEdgeSide();
void setEdgeSide(bool left);
void handleSerial();
int  getAdaptiveSpeed(int error);
void recover();
void pidStep();
void setup();
void loop();

// line sensor
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

bool followLeftEdge = true;

#define DEFAULT_FOLLOW_LEFT_EDGE  true
#define AUTO_DETECT_SIDE          1     // 0 = always use the default above

#define N_SENS      16                  // array size
#define POS_MAX     ((N_SENS - 1) * 1000)   // 15000
#define EDGE_TARGET 8000                // boundary sensor
#define ERROR_MAX   8000

// 1.0 = no filtering, 0.2 = heavy. wobble fixing
#define EDGE_ALPHA  0.45f

float   edgeFilt  = (float)EDGE_TARGET;
int     lastEdge  = EDGE_TARGET;
uint8_t darkCount = 0;

// PID config
int   baseSpeed          = 160;
float kp                 = 0.07f;
float ki                 = 0.0005f;
float kd                 = 0.9f;    // lowered: digital edge steps are 1000 counts
int   sharpTurnThreshold = 40;
int   minTurnSpeed       = 80;
float iClamp             = 800.0f;

int rightLossSpeed  = 90;
int rightLossTurn   = 40;

int leftLossSpeed  = 90;
int leftLossTurn   = 40;

// ── PID state ─────────────────────────────────────────
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

int findEdge() {
  sensor.getDigital(digital);

  darkCount = 0;
  for (uint8_t i = 0; i < N_SENS; i++) {
    if (digital[i]) darkCount++;
  }

  if (darkCount == 0 || darkCount == N_SENS) return -1;   // uniform, no boundary

  int  best     = -1;
  long bestDist = 0x7FFFFFFF;

  for (uint8_t i = 0; i + 1 < N_SENS; i++) {
    bool isEdge = followLeftEdge ? (!digital[i] &&  digital[i + 1])   // white > black
                                 : ( digital[i] && !digital[i + 1]);  // black > white
    if (!isEdge) continue;

    int  pos = (int)i * 1000 + 500;
    long d   = labs((long)pos - (long)lastEdge);
    if (d < bestDist) { bestDist = d; best = pos; }
  }
  return best;
}

void detectEdgeSide() {
  long rightWeight = 0;
  long totalDark   = 0;

  for (uint8_t s = 0; s < 10; s++) {
    sensor.getDigital(digital);
    for (uint8_t i = 0; i < N_SENS; i++) {
      if (digital[i]) {
        totalDark++;
        if (i >= N_SENS / 2) rightWeight++;
      }
    }
    delay(10);
  }

  if (totalDark == 0) {
    followLeftEdge = DEFAULT_FOLLOW_LEFT_EDGE;
    Serial.println("Auto-detect: no black seen, using default side.");
    return;
  }

  followLeftEdge = (rightWeight * 2 > totalDark);   // a lot of black
  Serial.print("Auto-detect: black is on the ");
  Serial.print(followLeftEdge ? "RIGHT" : "LEFT");
  Serial.print(" -> following ");
  Serial.print(followLeftEdge ? "LEFT" : "RIGHT");
  Serial.println(" edge.");
}

// side switching
void setEdgeSide(bool left) {
  followLeftEdge = left;
  lastEdge   = EDGE_TARGET;
  edgeFilt   = (float)EDGE_TARGET;
  last_error = 0;
  integral   = 0.0f;
  Serial.print("Now following the ");
  Serial.print(left ? "LEFT" : "RIGHT");
  Serial.println(" edge of the black area.");
}

// yay useless serial stuff but why not
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'l' || c == 'L') setEdgeSide(true);
    if (c == 'r' || c == 'R') setEdgeSide(false);
  }
}

// adaptive speed
int getAdaptiveSpeed(int error) {
  int absError = abs(error);
  if (absError <= sharpTurnThreshold) return baseSpeed;
  float t = (float)(absError - sharpTurnThreshold) / (ERROR_MAX - sharpTurnThreshold);
  t = constrain(t, 0.0f, 1.0f);
  return (int)(baseSpeed - (baseSpeed - minTurnSpeed) * t);
}

// recovery
void recover() {
  if (!followLeftEdge) {
    setMotors(rightLossSpeed, rightLossSpeed - rightLossTurn);
    return;
  }

  setMotors(leftLossSpeed - leftLossTurn, leftLossSpeed);
}

// PID trouble
void pidStep() {
  int e = findEdge();

  if (e < 0) {
    recover();
    return;
  }

  lastEdge = e;
  edgeFilt += EDGE_ALPHA * ((float)e - edgeFilt);

  int error = (int)edgeFilt - EDGE_TARGET;

  if (!followLeftEdge) error = -error;

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

// setup
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

  followLeftEdge = DEFAULT_FOLLOW_LEFT_EDGE;
#if AUTO_DETECT_SIDE
  // side detection!! 
  detectEdgeSide();
#endif

  lastEdge = EDGE_TARGET;
  edgeFilt = (float)EDGE_TARGET;

  Serial.println("Send 'l' or 'r' to switch edge side at any time.");
  delay(1000);
}

// loop
void loop() {
  handleSerial();

  if (digitalRead(STOP_BT) == LOW) {
    stop();
    Serial.println("pressed");
    while (true);
  }

  pidStep();
}