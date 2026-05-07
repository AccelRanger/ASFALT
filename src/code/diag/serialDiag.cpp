//AccelRanger

#define MUX_PIN_S0   8
#define MUX_PIN_S1  10
#define MUX_PIN_S2  11
#define MUX_PIN_S3  12
#define MUX_SIG_PIN A0

#define LEFT_A   3
#define LEFT_B   5
#define RIGHT_A  6
#define RIGHT_B  9

#define SENSOR_THRESHOLD 500

// Truth table from CD74HC4067 datasheet {S0, S1, S2, S3}
const uint8_t MUX_TRUTH_TABLE[16][4] = {
  {0, 0, 0, 0},  // ch 0  = S1
  {1, 0, 0, 0},  // ch 1
  {0, 1, 0, 0},  // ch 2  = S3
  {1, 1, 0, 0},  // ch 3  = S4
  {0, 0, 1, 0},  // ch 4  = S5
  {1, 0, 1, 0},  // ch 5  = S6
  {0, 1, 1, 0},  // ch 6
  {1, 1, 1, 0},  // ch 7  = S8
  {0, 0, 0, 1},  // ch 8
  {1, 0, 0, 1},  // ch 9  = S10
  {0, 1, 0, 1},  // ch 10
  {1, 1, 0, 1},  // ch 11 = S12
  {0, 0, 1, 1},  // ch 12 = S13
  {1, 0, 1, 1},  // ch 13 = S14
  {0, 1, 1, 1},  // ch 14
  {1, 1, 1, 1},  // ch 15 = S16
};

// Only the 10 connected sensors: {physical name, mux channel}
const uint8_t ACTIVE_SENSORS = 10;
const char*   SENSOR_NAMES[] = { "SENS1", "SENS3", "SENS4", "SENS5", "SENS9", "SENS10", "SENS11", "SENS13", "SENS14", "SENS15", "SENS16" };
const uint8_t SENSOR_CHANNELS[] = { 0, 4, 12, 2, 5, 13, 9, 3, 11, 7, 15 };

void selectMuxChannel(uint8_t ch) {
  digitalWrite(MUX_PIN_S0, MUX_TRUTH_TABLE[ch][0]);
  digitalWrite(MUX_PIN_S1, MUX_TRUTH_TABLE[ch][1]);
  digitalWrite(MUX_PIN_S2, MUX_TRUTH_TABLE[ch][2]);
  digitalWrite(MUX_PIN_S3, MUX_TRUTH_TABLE[ch][3]);
}

void stopMotors() {
  analogWrite(LEFT_A,  0); analogWrite(LEFT_B,  0);
  analogWrite(RIGHT_A, 0); analogWrite(RIGHT_B, 0);
}

void setup() {
  pinMode(MUX_PIN_S0, OUTPUT);
  pinMode(MUX_PIN_S1, OUTPUT);
  pinMode(MUX_PIN_S2, OUTPUT);
  pinMode(MUX_PIN_S3, OUTPUT);

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
    for (uint8_t i = 0; i < ACTIVE_SENSORS; i++) {
      selectMuxChannel(SENSOR_CHANNELS[i]);
      delay(20);
      int val = analogRead(MUX_SIG_PIN);

      Serial.print(SENSOR_NAMES[i]);
      Serial.print(":");
      Serial.print(val);

      if (val < SENSOR_THRESHOLD) Serial.print("( 1 )");
      else                        Serial.print("( 0 )");

      Serial.print("  ");
      if (i == 4) Serial.println();
    }
    Serial.println();
    Serial.println();

  } else if (cmd == 'l') {
    analogWrite(LEFT_A,  80); analogWrite(LEFT_B,  0);
    analogWrite(RIGHT_A, 80); analogWrite(RIGHT_B, 0);

  } else if (cmd == 'r') {
    analogWrite(LEFT_A,  0); analogWrite(LEFT_B,  80);
    analogWrite(RIGHT_A, 0); analogWrite(RIGHT_B, 80);

  } else if (cmd == 'b') { // backwards
    analogWrite(LEFT_A,  0);  analogWrite(LEFT_B,  80);
    analogWrite(RIGHT_A, 80); analogWrite(RIGHT_B, 0);

  } else if (cmd == 'f') {
    analogWrite(LEFT_A,  80); analogWrite(LEFT_B,  0);
    analogWrite(RIGHT_A, 0);  analogWrite(RIGHT_B, 80);

  } else if (cmd == 'x') {
    stopMotors();
  }
}