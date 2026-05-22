#include "shared.h"
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>

BluetoothSerial SerialBT;
Servo servo;

void motorOff(bool left, bool right) {
  if (left) {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    analogWrite(ENA, 0);
  }
  if (right) {
    digitalWrite(RIGHT_IN3, LOW);
    digitalWrite(RIGHT_IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void motorOn(int left, int right) {
  if (left > 0) {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
    analogWrite(ENA, left);
  } else if (left < 0) {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
    analogWrite(ENA, abs(left));
  } else {
    motorOff(true, false);
  }

  if (right > 0) {
    digitalWrite(RIGHT_IN3, LOW);
    digitalWrite(RIGHT_IN4, HIGH);
    analogWrite(ENB, right);
  } else if (right < 0) {
    digitalWrite(RIGHT_IN3, HIGH);
    digitalWrite(RIGHT_IN4, LOW);
    analogWrite(ENB, abs(right));
  } else {
    motorOff(false, true);
  }
}

bool readBluetoothPacket(BTPacket &btPacket) {
  if (SerialBT.available() >= sizeof(BTPacket)) {
    SerialBT.readBytes((uint8_t *)&btPacket, sizeof(BTPacket));
    while (SerialBT.available() > 0) {
      SerialBT.read();
    }
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);

  // bt master begin:
  SerialBT.begin(BT_RX_NAME, true);
  Serial.println("bluetooth initialized...");

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  servo.attach(SERVO);
  servo.write(90);

  motorOff(true, true);
}

bool active = false;
int lastPacketButton = 0;
bool connected = false;
BTPacket btPacket;

void loop() {
  if (!connected) {
    Serial.println("connecting to rcboat_tx...");
    if (SerialBT.connect(String(BT_TX_NAME))) {
      Serial.println("connected to rcboat_tx!");
      connected = true;
    } else {
      Serial.println("rcboat_tx not found. retrying in 3 seconds...");
      delay(3000);
    }
    return;
  }

  if (!SerialBT.connected()) {
    Serial.println("connection lost...");
    connected = false;
    return;
  }

  int leftSpeed = 0;
  int rightSpeed = 0;

  if (readBluetoothPacket(btPacket)) {
    if (btPacket.button == 1 && lastPacketButton == 0) {
      active = !active;
      Serial.print("state toggled: ");
      Serial.println(active ? "on" : "off");
    }

    lastPacketButton = btPacket.button;

    if (active) {
      int yAxis = btPacket.y;
      int xAxis = btPacket.x;

      if (abs(yAxis) < 25) {
        yAxis = 0;
      }

      leftSpeed = yAxis;
      rightSpeed = yAxis;

      int servoAngle = map(xAxis, -255, 255, 0, 180);

      servo.write(servoAngle);
    } else {
      servo.write(SERVO_DEFAULT_ANGLE);
    }

    Serial.print("from tx: x=");
    Serial.print(btPacket.x);
    Serial.print(", y=");
    Serial.print(btPacket.y);
    Serial.print(", button=");
    Serial.println(btPacket.button);
  }

  motorOn(leftSpeed, rightSpeed);

  delay(50);
}
