#include <Servo.h>

Servo clawServo;        // Pin 9
Servo secondArmServo;   // Pin 10  
Servo firstArmServo;    // Pin 11
Servo baseServo;        // Pin 6

void setup() {
  Serial.begin(9600);
  Serial.println("Servo Arm Ready!");

  clawServo.attach(9);
  secondArmServo.attach(10);
  firstArmServo.attach(11);
  baseServo.attach(6);

  clawServo.write(90);        // open
  secondArmServo.write(0);    // up
  firstArmServo.write(45);    // middle
  baseServo.write(90);        // center
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() > 1) {
      char id = command.charAt(0);
      int angle = command.substring(1).toInt();

      if (id == 'C') { // Claw
        angle = constrain(angle, 0, 90);   // 0 = closed, 90 = open
        clawServo.write(angle);
        Serial.println("Claw -> " + String(angle));
      } 
      else if (id == 'H') { // Vertical angle
        angle = constrain(angle, 0, 90);
        secondArmServo.write(angle);
        Serial.println("Height -> " + String(angle));
      } 
      else if (id == 'E') { // Extension
        angle = constrain(angle, 0, 90);
        firstArmServo.write(angle);
        Serial.println("Extension -> " + String(angle));
      } 
      else if (id == 'B') { // Base rotation
        angle = constrain(angle, 70, 110);
        baseServo.write(angle);
        Serial.println("Base -> " + String(angle));
      } 
      else {
        Serial.println("Unknown command: " + command);
      }
    }
  }
}
