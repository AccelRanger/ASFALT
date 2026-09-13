// motion test

#define LEFT_A    6
#define LEFT_B    9
#define RIGHT_A   5
#define RIGHT_B   3

#define SPEED 200

void setMotors(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);
  analogWrite(LEFT_A,  left  > 0 ?  left  : 0);
  analogWrite(LEFT_B,  left  < 0 ? -left  : 0);
  analogWrite(RIGHT_A, right > 0 ?  right : 0);
  analogWrite(RIGHT_B, right < 0 ? -right : 0);
}

void stop() { setMotors(0, 0); }

void setup() {
  pinMode(LEFT_A,  OUTPUT);
  pinMode(LEFT_B,  OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);
  stop();
}

void loop() {
  // Forward
  setMotors(SPEED, SPEED);
  delay(1000);

  // Backward
  setMotors(-SPEED, -SPEED);
  delay(1000);

  //Left
  setMotors(-SPEED, SPEED);
  delay(1000);

  // Right
  setMotors(SPEED, -SPEED);
  delay(1000);
}