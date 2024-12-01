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

unsigned long previousMillis = 0; // Store the last time the servos were updated
const long interval = 20;         // Update interval in milliseconds (20 ms for 50 Hz)

void setup() {
  Serial.begin(9600);  
  thumbServo.attach(thumbPin);  
  indexServo.attach(indexPin);
  middleServo.attach(middlePin);
  ringServo.attach(ringPin);
  pinkyServo.attach(pinkyPin);
 
}

void loop() {
  if (Serial.available()) {
    String angles = Serial.readStringUntil('\n');  
    
    // Only process if there's valid data
    if (angles.length() > 0) {
      float angleArray[5];  
      int idx = 0;
      int startIdx = 0;
      
      for (int i = 0; i < angles.length(); i++) {
        if (angles.charAt(i) == ',') {
          angleArray[idx] = angles.substring(startIdx, i).toFloat();
          startIdx = i + 1;
          idx++;
        }
      }
      angleArray[idx] = angles.substring(startIdx).toFloat();  

      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        // Update servos after interval has passed
        thumbServo.write(angleArray[0]);    
        indexServo.write(angleArray[1]);   
        middleServo.write(angleArray[2]);  
        ringServo.write(angleArray[3]);     
        pinkyServo.write(angleArray[4]);    

        previousMillis = currentMillis; 
      }
    }
  }
}
