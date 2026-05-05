// AccelRanger

#define MUX_PIN_A    8
#define MUX_PIN_B   10
#define MUX_PIN_C   11
#define MUX_PIN_D   12
#define MUX_SIG_PIN A0

#define LEFT_A   3
#define LEFT_B   5
#define RIGHT_A  6
#define RIGHT_B  9

#define NUM_SENSORS      16
#define SENSOR_THRESHOLD 500

void selectMuxChannel(uint8_t ch) {
  digitalWrite(MUX_PIN_A, (ch >> 0) & 1);
  digitalWrite(MUX_PIN_B, (ch >> 1) & 1);
  digitalWrite(MUX_PIN_C, (ch >> 2) & 1);
  digitalWrite(MUX_PIN_D, (ch >> 3) & 1);
}

void stopMotors() {
  analogWrite(LEFT_A,  0); analogWrite(LEFT_B,  0);
  analogWrite(RIGHT_A, 0); analogWrite(RIGHT_B, 0);
}

void setup() {
  pinMode(MUX_PIN_A, OUTPUT);
  pinMode(MUX_PIN_B, OUTPUT);
  pinMode(MUX_PIN_C, OUTPUT);
  pinMode(MUX_PIN_D, OUTPUT);

  pinMode(LEFT_A,  OUTPUT);
  pinMode(LEFT_B,  OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);

  stopMotors(); // ensure motors are off on boot

  Serial.begin(115200);
  Serial.println("=== DIAGNOSTIC START ===");
  Serial.println("Commands: 's' = sensors, 'f' = forward, 'l' = left spin, 'r' = right spin, 'x' = stop");
}

void loop() {
  if (!Serial.available()) return;
  char cmd = Serial.read();

  if (cmd == 's') {
    Serial.println("--- Sensor raw values ---");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      selectMuxChannel(i);
      delay(2);
      int val = analogRead(MUX_SIG_PIN);

      Serial.print("S"); Serial.print(i); Serial.print(":");
      Serial.print(val);

      if (val < SENSOR_THRESHOLD) Serial.print("(LINE)");
      else                        Serial.print("(    )");

      Serial.print("  ");
      if (i == 7) Serial.println();
    }
    Serial.println();
    Serial.println();

  } else if (cmd == 'f') {
    Serial.println("FORWARD 80 — both wheels should spin forward");
    analogWrite(LEFT_A,  80); analogWrite(LEFT_B,  0);
    analogWrite(RIGHT_A, 80); analogWrite(RIGHT_B, 0);

  } else if (cmd == 'l') {
    Serial.println("LEFT SPIN — robot should rotate left");
    analogWrite(LEFT_A,  0);  analogWrite(LEFT_B,  80);
    analogWrite(RIGHT_A, 80); analogWrite(RIGHT_B, 0);

  } else if (cmd == 'r') {
    Serial.println("RIGHT SPIN — robot should rotate right");
    analogWrite(LEFT_A,  80); analogWrite(LEFT_B,  0);
    analogWrite(RIGHT_A, 0);  analogWrite(RIGHT_B, 80);

  } else if (cmd == 'x') {
    Serial.println("STOP");
    stopMotors();
  }
}
