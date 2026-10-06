// clang-format off
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <BluetoothSerial.h>
#include <MPU6500_WE.h>
#include <xyzFloat.h>
#include "shared.h"
// clang-format on

BluetoothSerial SerialBT;
Servo servo1;
MPU6500_WE imu;

int cappedSpeed = 255;

void printPacket(const BT_TX_Packet &packet) {
  Serial.printf(
      "j1: x=%d, y=%d | j2: x=%d, y=%d | button=%d | speed=%d | calibrate=%d"
      " | servo_cmd_us=%d\n",
      packet.j1x, packet.j1y, packet.j2x, packet.j2y, packet.button,
      packet.speed, packet.calibrate, servo1.readMicroseconds());
}

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
  left = constrain(left, -cappedSpeed, cappedSpeed);
  right = constrain(right, -cappedSpeed, cappedSpeed);

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

void calibrateImu() {
  Serial.println("calibrating imu...");
  motorOff(true, true);
  imu.autoOffsets();
  Serial.println("calibrated");
}

bool readBluetoothPacket(BT_TX_Packet &packet) {
  if (SerialBT.available() >= sizeof(BT_TX_Packet)) {
    SerialBT.readBytes((uint8_t *)&packet, sizeof(BT_TX_Packet));
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

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  servo1.attach(SERVO_1);
  servo1.write(SERVO_DEFAULT_ANGLE);

  motorOff(true, true);

  // imu setup
  // Wire.begin(IMU_SDA, IMU_SCL);
  // if (!imu.init()) {
  //   Serial.println("imu failed to start");
  // } else {
  //   Serial.println("calibrating imu...");
  // }

  delay(1000);
  // calibrateImu();
}

bool active = false;
int lastPacketButton = 0;
int lastPacketCalibrate = 0;
bool connected = false;
BT_TX_Packet btTxPacket;
BT_RX_Packet btRxPacket;
int motorSpeed = 0;

void loop() {
  if (!connected) {
    Serial.println("connecting to rcboat_tx...");
    if (SerialBT.connect(String(BT_TX_NAME))) {
      Serial.println("connected to rcboat_tx!");
      digitalWrite(LED_BUILTIN, HIGH);
      connected = true;
    } else {
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("rcboat_tx not found. retrying in 3 seconds...");
      delay(3000);
    }
    return;
  }

  if (!SerialBT.connected()) {
    Serial.println("connection lost...");
    motorOff(true, true);
    motorSpeed = 0;
    servo1.write(SERVO_DEFAULT_ANGLE);
    active = false;
    connected = false;
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }

  if (readBluetoothPacket(btTxPacket)) {
    if (btTxPacket.calibrate == 1 && lastPacketCalibrate == 0) {
      // calibrateImu();
      motorSpeed = 0;
    }

    if (btTxPacket.button == 1 && lastPacketButton == 0) {
      active = !active;

      Serial.printf("state: %s -> %s\n", active ? "off" : "on",
                    active ? "on" : "off");
    }

    if (btTxPacket.speed != cappedSpeed) {
      cappedSpeed = btTxPacket.speed;
      Serial.print("capped speed updated: ");
      Serial.println(cappedSpeed);
    }

    lastPacketButton = btTxPacket.button;
    lastPacketCalibrate = btTxPacket.calibrate;

    if (active) {
      // j1 up and down motor for forward reverse
      int j1y = btTxPacket.j1y;

      if (abs(j1y) < 25) {
        j1y = 0;
      }

      motorSpeed = j1y;

      // j2 left and right changes servo angles
      int j2x = constrain(btTxPacket.j2x, -255, 255);
      if (abs(j2x) < 25) {
        j2x = 0;
      }
      int servo1Angle = map(j2x, -255, 255, 0, 180);
      servo1.write(servo1Angle);
    } else {
      servo1.write(SERVO_DEFAULT_ANGLE);
      motorSpeed = 0;
    }

    printPacket(btTxPacket);
  }

  if (connected) {
    // xyzFloat accValues = imu.getGValues();
    // xyzFloat gyroValues = imu.getGyrValues();
    // float resG = imu.getResultantG(accValues);
    //
    // btRxPacket.accX = accValues.x;
    // btRxPacket.accY = accValues.y;
    // btRxPacket.accZ = accValues.z;
    // btRxPacket.gyroX = gyroValues.x;
    // btRxPacket.gyroY = gyroValues.y;
    // btRxPacket.gyroZ = gyroValues.z;
    // btRxPacket.resG = resG;

    SerialBT.write((uint8_t *)&btRxPacket, sizeof(btRxPacket));
  }

  motorOn(0, motorSpeed);

  delay(50);
}
