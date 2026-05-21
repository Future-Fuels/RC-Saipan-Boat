#include <Arduino.h>
#include <BluetoothSerial.h>

#define BT_RX_NAME "rcboat_rx"
#define BT_TX_NAME "rcboat_tx"

#define J_VRX 34
#define J_VRY 35

#define BUTTON1 13

struct BTPacket {
  int x;
  int y;
  int button;
};

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  delay(1000);

  SerialBT.begin(BT_TX_NAME);
  Serial.println("waiting for rx connection...");

  pinMode(BUTTON1, INPUT_PULLUP);
}

int lastState = HIGH;
void loop() {
  BTPacket btPacket;

  int xRaw = analogRead(J_VRX);
  int yRaw = analogRead(J_VRY);
  btPacket.x = map(xRaw, 0, 4095, -100, 100);
  btPacket.y = map(yRaw, 0, 4095, -100, 100);

  int state = digitalRead(BUTTON1);
  btPacket.button = (state == LOW) ? 1 : 0;

  if (SerialBT.connected()) {
    SerialBT.write((uint8_t *)&btPacket, sizeof(btPacket));

    Serial.println("sent packet");
  }

  delay(50);
}
