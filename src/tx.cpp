#include "shared.h"
#include <Arduino.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  delay(1000);

  SerialBT.begin(BT_TX_NAME);
  Serial.println("waiting for rx connection...");

  pinMode(BUTTON1, INPUT_PULLUP);

  analogSetWidth(10);
}

int lastState = HIGH;
bool connected = false;
void loop() {
  BTPacket btPacket;

  int xRaw = analogRead(J_VRX);
  int yRaw = analogRead(J_VRY);
  int xMapped = map(xRaw, 0, 4095, -255, 255);
  int yMapped = map(yRaw, 0, 4095, 255, -255);

  btPacket.x = xMapped;
  btPacket.y = yMapped;

  int state = digitalRead(BUTTON1);
  btPacket.button = (state == LOW) ? 1 : 0;

  if (SerialBT.connected()) {
    SerialBT.write((uint8_t *)&btPacket, sizeof(btPacket));

    Serial.println("sent packet");
  } else {
    Serial.println("not connected");
  }

  delay(50);
}
