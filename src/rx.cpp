#include <Arduino.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

#define BT_RX_NAME "rcboat_rx"
#define BT_TX_NAME "rcboat_tx"

#define LEFT_IN1 33
#define LEFT_IN2 25
#define RIGHT_IN3 26
#define RIGHT_IN4 27
#define ENA 32
#define ENB 14

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

struct BTPacket {
  int x;
  int y;
  int button;
};

bool readBluetoothStruct(BTPacket &btPacket) {
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

  if (readBluetoothStruct(btPacket)) {

    if (btPacket.button == 1 && lastPacketButton == 0) {
      active = !active;
      Serial.print("Boat state toggled: ");
      Serial.println(active ? "ACTIVE" : "STANDBY");
    }
    lastPacketButton = btPacket.button;

    if (active) {
      leftSpeed = btPacket.y + btPacket.x;
      rightSpeed = btPacket.y - btPacket.x;

      leftSpeed = map(leftSpeed, -200, 200, -255, 255);
      rightSpeed = map(rightSpeed, -200, 200, -255, 255);
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
