#include <Servo.h>
#include <Arduino.h>

Servo thumbServo;
Servo indexServo;
Servo middleServo;
Servo ringServo;
Servo pinkyServo;

const int thumbPin = 9;
const int indexPin = 10;
const int middlePin = 11;
const int ringPin = 5;
const int pinkyPin = 6;

int fingerStates[5] = {0, 0, 0, 0, 0};

void setServoPositions();  

void setup() {
  thumbServo.attach(thumbPin);
  indexServo.attach(indexPin);
  middleServo.attach(middlePin);
  ringServo.attach(ringPin);
  pinkyServo.attach(pinkyPin);

  Serial.begin(9600);

  setServoPositions();
}

void loop() {
  if (Serial.available() >= 5) {
    String input = "";

    while (input.length() < 5) {
      if (Serial.available() > 0) {
        char c = Serial.read();
        if (isDigit(c)) {
          input += c; 
        }
      }
    }

    for (int i = 0; i < 5; i++) {
      fingerStates[i] = input.charAt(i) - '0'; 
    }

    setServoPositions();
  }
}

void setServoPositions() {
  int thumbAngle = (fingerStates[0] == 1) ? 120 : 0;
  int indexAngle = (fingerStates[1] == 1) ? 0 : 130; 
  int middleAngle = (fingerStates[2] == 1) ? 0 : 130; 
  int ringAngle = (fingerStates[3] == 1) ? 140 : 0;
  int pinkyAngle = (fingerStates[4] == 1) ? 140 : 0;

  thumbServo.write(thumbAngle); 
  indexServo.write(indexAngle);
  middleServo.write(middleAngle);
  ringServo.write(ringAngle);
  pinkyServo.write(pinkyAngle);
}
