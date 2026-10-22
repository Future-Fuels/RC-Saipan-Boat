#include <Arduino.h>
#include <ESP32Servo.h>
#include "shared.h"

Servo servo1;

void setServoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  servo1.write(angle);
  Serial.printf("servo angle: %d\n", angle);
}

void runSweep() {
  for (int angle = 0; angle <= 180; angle += 10) {
    setServoAngle(angle);
    delay(100);
  }

  for (int angle = 180; angle >= 0; angle -= 10) {
    setServoAngle(angle);
    delay(100);
  }

  setServoAngle(SERVO_DEFAULT_ANGLE);
}

void printHelp() {
  Serial.println("Enter an angle from 0 to 180, c for center, or s for sweep.");
}

void setup() {
  Serial.begin(115200);
  servo1.attach(SERVO_1);
  setServoAngle(SERVO_DEFAULT_ANGLE);
  printHelp();
}

void loop() {
  if (Serial.available() == 0) {
    return;
  }

  String command = Serial.readStringUntil('\n');
  command.trim();

  if (command.equalsIgnoreCase("c")) {
    setServoAngle(SERVO_DEFAULT_ANGLE);
  } else if (command.equalsIgnoreCase("s")) {
    runSweep();
  } else {
    int angle = command.toInt();
    if (command.length() > 0 && angle >= 0 && angle <= 180) {
      setServoAngle(angle);
    } else {
      printHelp();
    }
  }
}