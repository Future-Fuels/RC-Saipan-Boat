#include <Arduino.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

uint8_t slaveAddress[] = {0x88, 0xF1, 0x55, 0x30, 0xA0, 0x58};

void setup() {
  Serial.begin(115200);
  SerialBT.begin("rcboat_master", true);
  Serial.println("init");

  if (SerialBT.connect("rcboat_slave")) {
    Serial.println("connected to rcboat_slave");
  } else {
    Serial.println("unable to conncet to rcboat_slave");
  }
}

void loop() {
  SerialBT.println("hello");

  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }

  delay(1000);
}
