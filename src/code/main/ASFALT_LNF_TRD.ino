// AccelRanger — EDGE FOLLOWING version (16-channel, digital-only sensor)
// Follows the black/white boundary and holds it at EDGE_TARGET (8000).
// The edge side (black left / black right) is switchable at runtime.
// ToF obstacle detection/avoidance removed — line following + calibration only.
//
// .cpp build notes (not needed for the .ino):
//   - <Arduino.h> must be included explicitly; the IDE adds it only for .ino files.
//   - Function prototypes are declared below; the IDE normally generates these.
//   - Keep this file in the sketch folder alongside MuxSensor.h, or add it to
//     your PlatformIO src/ directory.

#include <Arduino.h>
#include "MuxSensor.h"

// ── forward declarations ──────────────────────────────
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

// ── line sensor pins ──────────────────────────────────
#define PIN_S0   12
#define PIN_S1   11
#define PIN_S2   10
#define PIN_S3    8
#define PIN_COM  A0

#define STOP_BT 7

MuxSensor sensor(PIN_S0, PIN_S1, PIN_S2, PIN_S3, PIN_COM, POLARITY_DARK_LOW);
uint8_t digital[MUX_NUM_CHANNELS];

// ── motor pins ────────────────────────────────────────
#define LEFT_A    6
#define LEFT_B    9
#define RIGHT_A   5
#define RIGHT_B   3

// ══════════════════════════════════════════════════════
//  EDGE FOLLOWING CONFIG
// ══════════════════════════════════════════════════════
// followLeftEdge == true  → track the LEFT edge of the black area
//                           (the black band lies to the RIGHT of the array)
// followLeftEdge == false → track the RIGHT edge of the black area
//                           (the black band lies to the LEFT of the array)
//
// Set three ways:
//   1. DEFAULT_FOLLOW_LEFT_EDGE below (used if auto-detect is off or fails)
//   2. AUTO_DETECT_SIDE at startup — place the robot on the edge, it decides
//   3. Serial: send 'l' or 'r' at any time to switch sides live
bool followLeftEdge = true;

#define DEFAULT_FOLLOW_LEFT_EDGE  true
#define AUTO_DETECT_SIDE          1     // 0 = always use the default above

#define N_SENS      16                  // your array size
#define POS_MAX     ((N_SENS - 1) * 1000)   // 15000
#define EDGE_TARGET 8000                // where we want the boundary to sit
#define ERROR_MAX   8000                // largest |error| we expect

// Smoothing of the coarse digital edge position.
// 1.0 = no filtering, 0.2 = heavy. Lower this first if the robot wobbles.
#define EDGE_ALPHA  0.45f

float   edgeFilt  = (float)EDGE_TARGET;
int     lastEdge  = EDGE_TARGET;
uint8_t darkCount = 0;

// ── PID config ────────────────────────────────────────
int   baseSpeed          = 160;
float kp                 = 0.07f;
float ki                 = 0.0005f;
float kd                 = 0.9f;    // lowered: digital edge steps are 1000 counts
int   sharpTurnThreshold = 40;
int   minTurnSpeed       = 80;
float iClamp             = 800.0f;

// Recovery speeds when no edge is visible
int recoverOuter = 150;   // wheel on the outside of the recovery turn
int recoverInner = -60;   // wheel on the inside (negative = pivot)

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

// ══════════════════════════════════════════════════════
//  EDGE DETECTION
//  Returns the boundary position on the same 0..15000 scale,
//  or -1 if no edge of the tracked polarity is in view.
//  A boundary between sensor i and i+1 sits at i*1000 + 500.
//  When several candidate edges exist (junctions, the band
//  curling back on itself), the one nearest the previous edge wins.
// ══════════════════════════════════════════════════════
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
    bool isEdge = followLeftEdge ? (!digital[i] &&  digital[i + 1])   // white → black
                                 : ( digital[i] && !digital[i + 1]);  // black → white
    if (!isEdge) continue;

    int  pos = (int)i * 1000 + 500;
    long d   = labs((long)pos - (long)lastEdge);
    if (d < bestDist) { bestDist = d; best = pos; }
  }
  return best;
}

// ── Auto-detect which side the black area is on ───────
// Averages several samples; if the dark sensors sit mostly on the right
// half of the array, the band is on the right → follow its left edge.
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

  followLeftEdge = (rightWeight * 2 > totalDark);   // majority of dark on the right
  Serial.print("Auto-detect: black is on the ");
  Serial.print(followLeftEdge ? "RIGHT" : "LEFT");
  Serial.print(" -> following ");
  Serial.print(followLeftEdge ? "LEFT" : "RIGHT");
  Serial.println(" edge.");
}

// ── Switch sides cleanly (resets control state) ───────
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

// ── Serial commands: 'l' / 'r' to switch live ─────────
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'l' || c == 'L') setEdgeSide(true);
    if (c == 'r' || c == 'R') setEdgeSide(false);
  }
}

// ── Adaptive speed ────────────────────────────────────
int getAdaptiveSpeed(int error) {
  int absError = abs(error);
  if (absError <= sharpTurnThreshold) return baseSpeed;
  float t = (float)(absError - sharpTurnThreshold) / (ERROR_MAX - sharpTurnThreshold);
  t = constrain(t, 0.0f, 1.0f);
  return (int)(baseSpeed - (baseSpeed - minTurnSpeed) * t);
}

// ── Recovery when the edge leaves the array ───────────
void recover() {
  // darkCount was refreshed by findEdge()
  bool allWhite = (darkCount == 0);

  // Following the left edge: black lives on the right.
  //   all white → band drifted off right  → turn right
  //   all black → we drove into the band  → turn left
  // Mirrored when following the right edge.
  bool turnRight = followLeftEdge ? allWhite : !allWhite;

  if (turnRight) setMotors(recoverOuter, recoverInner);
  else           setMotors(recoverInner, recoverOuter);
}

// ── PID step ──────────────────────────────────────────
void pidStep() {
  int e = findEdge();

  if (e < 0) {
    recover();
    return;
  }

  lastEdge = e;
  edgeFilt += EDGE_ALPHA * ((float)e - edgeFilt);

  int error = (int)edgeFilt - EDGE_TARGET;

  // The sign of the correction flips with the edge side: on the left edge,
  // a boundary too far right means steer right; on the right edge it is the
  // opposite, because the black area is on the other side of the array.
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

  followLeftEdge = DEFAULT_FOLLOW_LEFT_EDGE;
#if AUTO_DETECT_SIDE
  // Place the robot straddling the edge before this runs.
  detectEdgeSide();
#endif

  lastEdge = EDGE_TARGET;
  edgeFilt = (float)EDGE_TARGET;

  Serial.println("Send 'l' or 'r' to switch edge side at any time.");
  delay(1000);
}

// ── Loop ──────────────────────────────────────────────
void loop() {
  handleSerial();

  if (digitalRead(STOP_BT) == LOW) {
    stop();
    Serial.println("pressed");
    while (true);
  }

  pidStep();
}