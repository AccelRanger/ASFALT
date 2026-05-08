#define PIN_S0  12
#define PIN_S1  11
#define PIN_S2  10
#define PIN_S3   8
#define PIN_COM A0

#define NUM_CHANNELS 16
#define SETTLE_MS    1

const uint8_t MUX_TABLE[NUM_CHANNELS][4] = {
  // S3  S2  S1  S0   CH
  {  0,  0,  0,  0 }, //  0  — 0b0000
  {  0,  0,  0,  1 }, //  1  — 0b0001
  {  0,  0,  1,  0 }, //  2  — 0b0010
  {  0,  0,  1,  1 }, //  3  — 0b0011
  {  0,  1,  0,  0 }, //  4  — 0b0100
  {  0,  1,  0,  1 }, //  5  — 0b0101
  {  0,  1,  1,  0 }, //  6  — 0b0110
  {  0,  1,  1,  1 }, //  7  — 0b0111
  {  1,  0,  0,  0 }, //  8  — 0b1000
  {  1,  0,  0,  1 }, //  9  — 0b1001
  {  1,  0,  1,  0 }, // 10  — 0b1010
  {  1,  0,  1,  1 }, // 11  — 0b1011
  {  1,  1,  0,  0 }, // 12  — 0b1100
  {  1,  1,  0,  1 }, // 13  — 0b1101
  {  1,  1,  1,  0 }, // 14  — 0b1110
  {  1,  1,  1,  1 }, // 15  — 0b1111
};

uint16_t readings[NUM_CHANNELS];

void selectChannel(uint8_t ch) {
  digitalWrite(PIN_S3, MUX_TABLE[ch][0]);
  digitalWrite(PIN_S2, MUX_TABLE[ch][1]);
  digitalWrite(PIN_S1, MUX_TABLE[ch][2]);
  digitalWrite(PIN_S0, MUX_TABLE[ch][3]);
}

void readAllChannels() {
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
    selectChannel(ch);
    delay(SETTLE_MS);
    readings[ch] = analogRead(PIN_COM);
  }
}

void printReadings() {
  Serial.println("=== MUX output ===");
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
    Serial.print("CH");
    if (ch < 10) Serial.print("0");
    Serial.print(ch);
    Serial.print(": ");
    Serial.println(readings[ch]);
  }
  Serial.println("==================");
}

void setup() {
  Serial.println("Start MCU");
  pinMode(PIN_S0, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    while (Serial.available() > 0) Serial.read();

    if (cmd == 's' || cmd == 'S') {
      readAllChannels();
      printReadings();
    }
  }
}