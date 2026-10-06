// clang-format off
#include <Arduino.h>
#include <BluetoothSerial.h>
#include "shared.h"
// clang-format on

BluetoothSerial SerialBT;

struct JoystickState {
  int x;
  int y;
};

JoystickState readJoystick(int vrxPin, int vryPin) {
  int xRaw = analogRead(vrxPin);
  int yRaw = analogRead(vryPin);
  int xMapped = map(xRaw, 0, 4095, -255, 255);
  int yMapped = map(yRaw, 0, 4095, 255, -255);

  return {xMapped, yMapped};
}

volatile int rotaryValue = 0;
int lastClk;
void IRAM_ATTR updateEncoder() {
  int clk = digitalRead(ROTARY_CLK_PIN);
  if (clk != lastClk) {
    if (digitalRead(ROTARY_DT_PIN) != clk) {
      if (rotaryValue > ROTARY_MIN) {
        rotaryValue -= ROTARY_STEP;
      }
    } else {
      if (rotaryValue < ROTARY_MAX) {
        rotaryValue += ROTARY_STEP;
      }
    }
    lastClk = clk;
  }
}

bool readBluetoothPacket(BT_RX_Packet &packet) {
  if (SerialBT.available() >= sizeof(BT_RX_Packet)) {
    SerialBT.readBytes((uint8_t *)&packet, sizeof(BT_RX_Packet));
    while (SerialBT.available() > 0) {
      SerialBT.read();
    }
    return true;
  }
  return false;
}

void sendMonitorJson(const BT_TX_Packet &tx, const BT_RX_Packet &rx,
                     bool connected) {
  Serial.printf("{\"j1x\":%d,\"j1y\":%d,\"j2x\":%d,\"j2y\":%d,"
                "\"button\":%d,\"speed\":%d,\"calibrate\":%d,"
                "\"accX\":%.3f,\"accY\":%.3f,\"accZ\":%.3f,"
                "\"resG\":%.3f,\"gyroX\":%.3f,\"gyroY\":%.3f,\"gyroZ\":%.3f,"
                "\"connected\":%s}\n",
                tx.j1x, tx.j1y, tx.j2x, tx.j2y, tx.button, tx.speed,
                tx.calibrate, rx.accX, rx.accY, rx.accZ, rx.resG, rx.gyroX,
                rx.gyroY, rx.gyroZ, connected ? "true" : "false");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  SerialBT.begin(BT_TX_NAME);

  // setup rotary
  pinMode(ROTARY_CLK_PIN, INPUT_PULLUP);
  pinMode(ROTARY_DT_PIN, INPUT_PULLUP);
  lastClk = digitalRead(ROTARY_CLK_PIN);

  attachInterrupt(digitalPinToInterrupt(ROTARY_CLK_PIN), updateEncoder, CHANGE);

  analogSetWidth(12);
}

int lastState = HIGH;
bool calibrateRequested = false;
BT_TX_Packet btPacket;
BT_RX_Packet btRxPacket;

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') {
      calibrateRequested = true;
      Serial.println("calibrate signal sent");
    }
  }

  JoystickState joystick1 = readJoystick(J1_VRX, J1_VRY);
  JoystickState joystick2 = readJoystick(J2_VRX, J2_VRY);

  btPacket.j1x = joystick1.x;
  btPacket.j1y = joystick1.y; // motor forward/reverse and speed(? maybe)
  btPacket.j2x = joystick2.x; // sevo angle
  btPacket.j2y = joystick2.y;
  btPacket.speed = rotaryValue;
  btPacket.calibrate = calibrateRequested ? 1 : 0;
  calibrateRequested = false;

  int state = digitalRead(BUTTON1);
  btPacket.button = (state == LOW) ? 1 : 0;

  bool isConnected = SerialBT.connected();

  if (isConnected) {
    digitalWrite(LED_BUILTIN, HIGH);

    SerialBT.write((uint8_t *)&btPacket, sizeof(btPacket));

    readBluetoothPacket(btRxPacket);
  } else {
    digitalWrite(LED_BUILTIN, LOW);

    memset(&btRxPacket, 0, sizeof(btRxPacket));
  }

  sendMonitorJson(btPacket, btRxPacket, isConnected);

  delay(100);
}
