#include <AFMotor.h>
#include <Servo.h>
#include <NewPing.h>

#define RIGHT A3  // Right IR sensor connected to analog pin A2 of Arduino Uno:
#define LEFT A4
#define trig_Pin A1
#define echo_Pin A0
#define MAX_SPEED 190
#define MAX_SPEED_OFFSET 20
#define MAX_DISTANCE 200

unsigned int distance = 0;     //Variable to store ultrasonic sensor distance:
unsigned int Right_Value = 0;  //Variable to store Right IR sensor value:
unsigned int Left_Value = 0;

NewPing sonar(trig_Pin, echo_Pin, MAX_DISTANCE);

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);
Servo myservo;

boolean goesForward = false;
int speedSet = 0;
int Time = 0;
int pos = 0;

void setup() {
  Serial.begin(9600);
  myservo.attach(10);
  {
    for (pos = 100; pos <= 0; pos += 1) {
      myservo.write(pos);
      delay(25);
    }
    for (pos = 0; pos >= 100; pos -= 1) {
      myservo.write(pos);
      delay(25);
    }
    for (pos = 100; pos <= 45; pos += 1) {
      myservo.write(pos);
      delay(25);
    }
  }
  pinMode(RIGHT, INPUT);  //set analog pin RIGHT as an input:
  pinMode(LEFT, INPUT);   //set analog pin RIGHT as an input:
  pinMode(trig_Pin, OUTPUT);
  pinMode(echo_Pin, INPUT);
}

void loop() {
  distance = obstacle();
  unsigned int distance = sonar.ping_cm();
  int distanceR = 0;
  int distanceL = 0;
  int Right_Value = digitalRead(RIGHT);
  int Left_Value = digitalRead(LEFT);
  delay(40);

  if (distance >= 20 && distance <= 50) {
    moveStop();
    delay(100);
    moveBackward();
    delay(200);
    moveStop();
    delay(200);
    distanceR = lookRight();
    delay(200);
    distanceL = lookLeft();
    delay(200);

    if (distanceR >= distanceL) {
      if (distanceR <= 50 && distance <= 50) {

        moveStop();
        delay(200);
      } else {
        turnRight();
        moveStop();
      }
      // turnRight();
      // moveStop();
    } else {
      if (distanceL <= 50 && distance <= 50) {

        moveStop();
        delay(200);
      } else {
        turnLeft();
        moveStop();
      }
      // turnLeft();
      // moveStop();
    }
  } else {
    moveForward();
  }

  if (distance >= 10 && distance <= 20) {
    moveForward();
    delay(200);
  } else if ((Right_Value == 0) && (Left_Value == 1)) {
    turnLeft();
    delay(200);
  } else if ((Right_Value == 1) && (Left_Value == 0)) {
    turnRight();
    delay(200);
  } else if (distance > 20) {
    moveForward();
  }
}





int obstacle() {
  digitalWrite(trig_Pin, HIGH);
  delayMicroseconds(1000);
  digitalWrite(trig_Pin, LOW);
  Time = pulseIn(echo_Pin, HIGH);
  distance = (Time / 2) / 29.1;
  if (distance < 0)
    distance = 0;
  if (distance > 100)
    distance = 100;
  return distance;
}
int lookRight() {
  myservo.write(0);
  delay(500);
  distance = obstacle();
  delay(200);
  myservo.write(45);
  return distance;
}

int lookLeft() {
  myservo.write(100);
  delay(500);
  distance = obstacle();
  delay(200);
  myservo.write(45);
  return distance;
}

void moveStop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void moveForward() {

  if (!goesForward) {
    goesForward = true;
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 5)  // slowly bring the speed up to avoid loading down the batteries too quickly
    {
      motor1.setSpeed(speedSet);
      motor2.setSpeed(speedSet);
      motor3.setSpeed(speedSet);
      motor4.setSpeed(speedSet);
      delay(5);
    }
  }
}

void moveBackward() {
  goesForward = false;
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 5)  // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    motor1.setSpeed(speedSet);
    motor2.setSpeed(speedSet);
    motor3.setSpeed(speedSet);
    motor4.setSpeed(speedSet);
    delay(5);
  }
}

void turnLeft() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(2000);
  motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
}

void turnRight() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(2000);
  motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
}