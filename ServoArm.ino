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
    String line = Serial.readStringUntil('\n');
    line.trim();

    int cIndex = line.indexOf('C');
    int hIndex = line.indexOf('H');
    int eIndex = line.indexOf('E');
    int bIndex = line.indexOf('B');

    if (cIndex != -1 && hIndex != -1 && eIndex != -1 && bIndex != -1) {
      int clawAngle = line.substring(cIndex+1, hIndex).toInt();
      int heightAngle = line.substring(hIndex+1, eIndex).toInt();
      int extensionAngle = line.substring(eIndex+1, bIndex).toInt();
      int baseAngle = line.substring(bIndex+1).toInt();

      clawAngle = constrain(clawAngle, 0, 90);
      heightAngle = constrain(heightAngle, 0, 90);
      extensionAngle = constrain(extensionAngle, 0, 90);
      baseAngle = constrain(baseAngle, 70, 110);

      clawServo.write(clawAngle);
      secondArmServo.write(heightAngle);
      firstArmServo.write(extensionAngle);
      baseServo.write(baseAngle);
    }
  }
}
