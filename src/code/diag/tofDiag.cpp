#include <Wire.h>
#include <VL53L1X.h>


// config
#define DISTANCE_MODE   VL53L1X::Long   // Short / Medium / Long
#define TIMING_BUDGET   50000           // µs  — 20000 to 1000000

VL53L1X sensor;
uint16_t distance_mm = 0;   // last measurement in millimetres
uint8_t  range_status = 0;  // 0 = valid, nonzero = error code

void initSensor() {
  Wire.begin();
  Wire.setClock(400000);

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("ERROR: VL53L1X not detected — check wiring!");
    while (true) {} 
  }

  sensor.setDistanceMode(DISTANCE_MODE);
  sensor.setMeasurementTimingBudget(TIMING_BUDGET);

  sensor.startContinuous(TIMING_BUDGET / 1000);
}

void readSensor() {
  distance_mm  = sensor.read();
  range_status = sensor.ranging_data.range_status;
}

void printReading() {
  Serial.println("=== TOF output ===");
  Serial.print("Distance: ");
  if (range_status == 0) {
    Serial.print(distance_mm);
    Serial.println(" mm");
  } else {
    Serial.print("ERR (status ");
    Serial.print(range_status);
    Serial.println(")");
  }
  Serial.print("Status  : ");
  Serial.println(VL53L1X::rangeStatusToString(range_status));
  Serial.println("==================");
}

void setup() {
  Serial.begin(9600);
  Serial.println("Start MCU");
  initSensor();
  Serial.println("VL53L1X ready");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    while (Serial.available() > 0) Serial.read();

    if (cmd == 's' || cmd == 'S') {
      readSensor();
      printReading();
    }
  }
}