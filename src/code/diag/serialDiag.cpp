#include "CD74HC4067.h"

const uint8_t PIN_S0  = 12;
const uint8_t PIN_S1  = 11;
const uint8_t PIN_S2  = 10;
const uint8_t PIN_S3  = 8;
const uint8_t PIN_COM = A0;

const uint8_t NUM_SENSORS = 16;

CD74HC4067 mux(PIN_S0, PIN_S1, PIN_S2, PIN_S3, PIN_COM);

void setup() {
    Serial.begin(9600);
    while (!Serial) { }
    mux.setup();

    Serial.println(F("=== Line Sensor Diagnostic ==="));
    Serial.print(F("Channels: "));
    Serial.println(NUM_SENSORS);
    Serial.println(F("Type 's' to read all sensors."));
    Serial.println(F("------------------------------"));
}

void printReadings() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(F("S"));
        Serial.print(i + 1);
        Serial.print(F(":"));
        Serial.print(mux.readChannel(i));
        if (i < NUM_SENSORS - 1) Serial.print(F(" ; "));
    }
    Serial.println();
}

void loop() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 's' || cmd == 'S') {
            printReadings();
        }
    }
}
