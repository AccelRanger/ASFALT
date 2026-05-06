// --- MUX ---
#define MUX_PIN_A    8
#define MUX_PIN_B   10
#define MUX_PIN_C   11
#define MUX_PIN_D   12
#define MUX_SIG_PIN A0

// --- Motors (LEFT_A/B swapped to fix reverse) ---
#define LEFT_A   5
#define LEFT_B   3
#define RIGHT_A  9
#define RIGHT_B  6

// --- Sensor config ---
#define NUM_SENSORS       16
#define SENSOR_THRESHOLD  500

// --- Speed config ---
#define BASE_SPEED          100
#define ERROR_ACCEL_SPEED   120
#define TURN_SPEED          80
#define ERROR_ACCEL_THRESH  1500

float Kp = 0.03f;
float Ki = 0.0f;
float Kd = 0.4f;

// --- PID state ---
float lastError = 0;
float integral  = 0;

// ============================================================
void selectMuxChannel(uint8_t ch) {
  digitalWrite(MUX_PIN_A, (ch >> 0) & 1);
  digitalWrite(MUX_PIN_B, (ch >> 1) & 1);
  digitalWrite(MUX_PIN_C, (ch >> 2) & 1);
  digitalWrite(MUX_PIN_D, (ch >> 3) & 1);
}

float readSensors() {
  long weightedSum = 0;
  long activeSum   = 0;
  bool anyActive   = false;

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    selectMuxChannel(i);
    delay(2);                        // wait for MUX to switch and signal to settle
    int val = analogRead(MUX_SIG_PIN);

    if (val < SENSOR_THRESHOLD) {
      int strength = SENSOR_THRESHOLD - val;
      long weight  = (long)i * 1000;
      weightedSum += weight * strength;
      activeSum   += strength;
      anyActive    = true;
    }
  }

  if (!anyActive) return lastError;

  float position = (float)weightedSum / (float)activeSum;
  return position - 7500.0f;
}

// ============================================================
void driveLeft(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { analogWrite(LEFT_A, speed); analogWrite(LEFT_B, 0); }
  else            { analogWrite(LEFT_A, 0);     analogWrite(LEFT_B, -speed); }
}

void driveRight(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { analogWrite(RIGHT_A, speed); analogWrite(RIGHT_B, 0); }
  else            { analogWrite(RIGHT_A, 0);      analogWrite(RIGHT_B, -speed); }
}

// ============================================================
void setup() {
  pinMode(MUX_PIN_A, OUTPUT);
  pinMode(MUX_PIN_B, OUTPUT);
  pinMode(MUX_PIN_C, OUTPUT);
  pinMode(MUX_PIN_D, OUTPUT);

  pinMode(LEFT_A,  OUTPUT);
  pinMode(LEFT_B,  OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);

  Serial.begin(115200);
}

// ============================================================
void loop() {
  float error      = readSensors();
  integral        += error;
  integral         = constrain(integral, -5000, 5000);
  float derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError        = error;

  // Adaptive speed: small error = fast, large error = slow
  float t         = constrain(fabs(error) / ERROR_ACCEL_THRESH, 0.0f, 1.0f);
  int   baseSpeed = (int)(ERROR_ACCEL_SPEED - t * (ERROR_ACCEL_SPEED - TURN_SPEED));

  driveLeft(baseSpeed  - (int)correction);
  driveRight(baseSpeed + (int)correction);

  // Uncomment to tune PID over serial:
  // Serial.print("err:"); Serial.print(error);
  // Serial.print(" cor:"); Serial.print(correction);
  // Serial.print(" spd:"); Serial.println(baseSpeed);
}
