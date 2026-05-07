//AccelRanger

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

  stopMotors();

  Serial.begin(9600);
  Serial.println("=== MCU START ===");
}

void loop() {
  if (!Serial.available()) return;
  char cmd = Serial.read();

  if (cmd == 's') {
    Serial.println("--- Sensor raw values ---");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      selectMuxChannel(i);
      delay(20); // mux switching time
      int val = analogRead(MUX_SIG_PIN);

      Serial.print("S"); Serial.print(i); Serial.print(":");
      Serial.print(val);

      if (val < SENSOR_THRESHOLD) Serial.print("( 1 )");
      else                        Serial.print("( 0 )");

      Serial.print("  ");
      if (i == 7) Serial.println();
    }
    Serial.println();
    Serial.println();

  } else if (cmd == 'l') {
    analogWrite(LEFT_A,  80); analogWrite(LEFT_B,  0);
    analogWrite(RIGHT_A, 80); analogWrite(RIGHT_B, 0);

  } else if (cmd == 'r') {
    analogWrite(LEFT_A,  0); analogWrite(LEFT_B,  80);
    analogWrite(RIGHT_A, 0); analogWrite(RIGHT_B, 80);

  } else if (cmd == 'b') { //backwards
    analogWrite(LEFT_A,  0);  analogWrite(LEFT_B,  80);
    analogWrite(RIGHT_A, 80); analogWrite(RIGHT_B, 0);

  } else if (cmd == 'f') {
    analogWrite(LEFT_A,  80); analogWrite(LEFT_B,  0);
    analogWrite(RIGHT_A, 0);  analogWrite(RIGHT_B, 80);

  } else if (cmd == 'x') {
    stopMotors();
  }
}
