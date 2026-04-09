// ════════════════════════════════════════════════════════════════
//  LINE FOLLOWER  –  Arduino Nano (ATmega328P)
//  Sensor  : 16× TCRT5000 in U-shape via analog mux (MuxReader)
//  Driver  : PWM H-bridge
//
//  Sensor layout (U-shape):
//    LEFT  side  : CH0  – CH3   (4 sensors, point left)
//    FRONT bar   : CH4  – CH11  (8 sensors, point forward)
//    RIGHT side  : CH12 – CH15  (4 sensors, point right)
//
//  Motor wiring:
//    LEFT  motor → 1A : D3  |  1B : D5
//    RIGHT motor → 2A : D6  |  2B : D9
//
//  Mux wiring:
//    PORT A (S0) → D8  |  PORT B (S1) → D10
//    PORT C (S2) → D11 |  PORT D (S3) → D12
//    OUT (SIG)   → A0
//
// ════════════════════════════════════════════════════════════════
//  TUNING GUIDE
//  ─────────────────────────────────────────────────────────────
//  BASE_SPEED      Start at 100. Raise once steering is correct.
//
//  TURN_SPEED      Speed of the INNER wheel during a sharp turn.
//                  Lower = tighter turn. Try 0–40 to start.
//
//  KP_FRONT        Front bar proportional gain. Raise until the
//                  robot oscillates on a straight, then back off ~20%.
//                  Start: 0.05
//
//  KD_FRONT        Damps front bar oscillation. Raise if it wiggles.
//                  Start: 0.15
//
//  KP_SIDE         How hard the side sensors trigger a sharp turn.
//                  Raise if the robot misses corners.
//                  Start: 60 (added directly as correction units)
//
//  SIDE_THRESHOLD  Min ADC sum on one side-arm before it triggers.
//                  Lower if corners are missed, raise to ignore noise.
//                  Start: 200
//
//  FRONT_THRESHOLD Min ADC sum on front bar to count as line seen.
//                  Start: 300
// ════════════════════════════════════════════════════════════════

#include "MuxReader.h"

// ── Mux pins ──────────────────────────────────────────────────────
#define MUX_PIN_A    8
#define MUX_PIN_B   10
#define MUX_PIN_C   11
#define MUX_PIN_D   12
#define MUX_SIG_PIN A0

// ── Motor pins ────────────────────────────────────────────────────
#define LEFT_A   3
#define LEFT_B   5
#define RIGHT_A  6
#define RIGHT_B  9

// ── Speed settings ────────────────────────────────────────────────
#define BASE_SPEED   100    // forward cruise speed (0-255)
#define TURN_SPEED   20     // inner wheel speed during sharp corner turn
#define MAX_SPEED    220    // motor ceiling

// ── PID gains for the front 8-sensor bar ─────────────────────────
#define KP_FRONT   0.05f
#define KI_FRONT   0.0f
#define KD_FRONT   0.15f

// ── Side sensor correction strength ──────────────────────────────
// When a side arm detects the line, this value overrides the PID
// and forces a sharp turn. Higher = more aggressive corner turning.
#define KP_SIDE    60

// ── Detection thresholds (ADC sum per sensor group) ──────────────
#define SIDE_THRESHOLD   200   // min sum on left/right arm to trigger
#define FRONT_THRESHOLD  300   // min sum on front bar to count as on-line

// ── Sensor channel layout ─────────────────────────────────────────
#define LEFT_CH_START    0
#define LEFT_CH_END      3
#define FRONT_CH_START   4
#define FRONT_CH_END     11
#define RIGHT_CH_START   12
#define RIGHT_CH_END     15

// ════════════════════════════════════════════════════════════════

MuxReader mux(MUX_PIN_A, MUX_PIN_B, MUX_PIN_C, MUX_PIN_D, MUX_SIG_PIN);

// Raw channel buffer
int ch[16];

// PID state (front bar only)
float pidIntegral  = 0.0f;
float pidLastError = 0.0f;
unsigned long lastTime = 0;

// ── Motor helpers ─────────────────────────────────────────────────
void motorLeft(int speed) {
    speed = constrain(speed, -255, 255);
    if (speed >= 0) { analogWrite(LEFT_A, speed);   analogWrite(LEFT_B, 0);      }
    else            { analogWrite(LEFT_A, 0);        analogWrite(LEFT_B, -speed); }
}

void motorRight(int speed) {
    // Right motor physically reversed – A/B swapped
    speed = constrain(speed, -255, 255);
    if (speed >= 0) { analogWrite(RIGHT_A, 0);       analogWrite(RIGHT_B, speed);  }
    else            { analogWrite(RIGHT_A, -speed);  analogWrite(RIGHT_B, 0);      }
}

void stopMotors() {
    analogWrite(LEFT_A, 0); analogWrite(LEFT_B, 0);
    analogWrite(RIGHT_A, 0); analogWrite(RIGHT_B, 0);
}

// ── Sensor helpers ────────────────────────────────────────────────
// Sum ADC values across a channel range (inverted: black = high)
int sensorSum(int fromCh, int toCh) {
    int s = 0;
    for (int i = fromCh; i <= toCh; i++) {
        s += (1023 - ch[i]);   // invert: black line = high value
    }
    return s;
}

// Weighted position across a channel range → 0 to (range*1000)
// Center = (range/2) * 1000
int sensorPosition(int fromCh, int toCh) {
    long weighted = 0;
    long total    = 0;
    for (int i = fromCh; i <= toCh; i++) {
        int val = (1023 - ch[i]);
        weighted += (long)(i - fromCh) * 1000L * val;
        total    += val;
    }
    if (total < 1) return ((toCh - fromCh) * 1000) / 2;  // default center
    return (int)(weighted / total);
}

// ════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(9600);

    pinMode(LEFT_A,  OUTPUT); pinMode(LEFT_B,  OUTPUT);
    pinMode(RIGHT_A, OUTPUT); pinMode(RIGHT_B, OUTPUT);
    stopMotors();

    mux.begin();
    mux.setInvert(false);   // we invert manually per-group above
    mux.setSettleTime(10);

    // ── Startup: forward 0.5s → backward 0.5s ────────────────────
    Serial.println("Startup: forward...");
    motorLeft(BASE_SPEED); motorRight(BASE_SPEED);
    delay(500);
    stopMotors(); delay(100);

    Serial.println("Startup: backward...");
    motorLeft(-BASE_SPEED); motorRight(-BASE_SPEED);
    delay(500);
    stopMotors(); delay(100);

    Serial.println("Line follower started.");
    lastTime = millis();
}

// ════════════════════════════════════════════════════════════════
void loop() {

    // ── 1. Read all 16 channels into buffer ──────────────────────
    mux.readAll(ch);

    // ── 2. Calculate sums per zone ───────────────────────────────
    int sumLeft  = sensorSum(LEFT_CH_START,  LEFT_CH_END);    // 0-4092
    int sumFront = sensorSum(FRONT_CH_START, FRONT_CH_END);   // 0-8184
    int sumRight = sensorSum(RIGHT_CH_START, RIGHT_CH_END);   // 0-4092

    bool lineLeft  = (sumLeft  > SIDE_THRESHOLD);
    bool lineFront = (sumFront > FRONT_THRESHOLD);
    bool lineRight = (sumRight > SIDE_THRESHOLD);

    // ── 3. No line anywhere → stop ───────────────────────────────
    if (!lineLeft && !lineFront && !lineRight) {
        stopMotors();
        pidIntegral = 0.0f; pidLastError = 0.0f;
        Serial.println("NO LINE");
        return;
    }

    // ── 4. Compute dt for PID ────────────────────────────────────
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    lastTime = now;

    // ── 5. Determine correction ───────────────────────────────────
    //
    //  Priority order:
    //    A) Side sensors only (sharp corner) → hard turn
    //    B) Side + front together (entering corner) → blended turn
    //    C) Front bar only (straight / gentle curve) → PID
    //
    float correction = 0.0f;

    if (lineLeft && !lineRight) {
        // Line coming from left side → sharp left turn
        // Scale by how many left sensors are active
        float sideStrength = (float)sumLeft / (4 * 1023.0f);
        correction = -(KP_SIDE + KP_SIDE * sideStrength);

    } else if (lineRight && !lineLeft) {
        // Line coming from right side → sharp right turn
        float sideStrength = (float)sumRight / (4 * 1023.0f);
        correction = +(KP_SIDE + KP_SIDE * sideStrength);

    } else if (lineFront) {
        // Front bar PID – weighted position of line on front bar
        // Front bar center = channel 3.5 relative → position 3500
        int frontPos   = sensorPosition(FRONT_CH_START, FRONT_CH_END);
        float error    = (float)(frontPos - 3500);  // 3500 = center of 8 sensors

        pidIntegral   += error * dt;
        pidIntegral    = constrain(pidIntegral, -3000.0f, 3000.0f);
        float deriv    = (error - pidLastError) / dt;
        pidLastError   = error;

        correction = KP_FRONT * error + KI_FRONT * pidIntegral + KD_FRONT * deriv;

    } else {
        // Only side sensors, both active (straddle) → go straight
        correction = 0.0f;
    }

    // ── 6. Apply correction to motors ────────────────────────────
    int leftSpeed  = constrain((int)(BASE_SPEED + correction), TURN_SPEED, MAX_SPEED);
    int rightSpeed = constrain((int)(BASE_SPEED - correction), TURN_SPEED, MAX_SPEED);

    motorLeft(leftSpeed);
    motorRight(rightSpeed);

    // ── 7. Debug ─────────────────────────────────────────────────
    Serial.print("L:"); Serial.print(sumLeft);
    Serial.print(" F:"); Serial.print(sumFront);
    Serial.print(" R:"); Serial.print(sumRight);
    Serial.print(" | Cor:"); Serial.print((int)correction);
    Serial.print(" ML:"); Serial.print(leftSpeed);
    Serial.print(" MR:"); Serial.println(rightSpeed);
}
