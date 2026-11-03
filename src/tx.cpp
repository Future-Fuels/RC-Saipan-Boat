#include <Arduino.h>
#include <BluetoothSerial.h>

#include "shared.h"

#define J1_VRX 34
#define J1_VRY 35
#define J2_VRX 36
#define J2_VRY 39

BluetoothSerial bluetooth;
PacketReader bluetoothReader;
MonitorTelemetryPacket telemetry;
ControlPacket &control = telemetry.control;

uint32_t lastSend = 0;
uint32_t lastReport = 0;
uint32_t lastTelemetry = 0;

bool receivedTelemetry = false;
bool receivedTelemetryOnLink = false;

String serialLine;
bool serialOverflow = false;

bool applyCommand(String &line) {
  line.replace('\t', ' ');

  int space = line.indexOf(' ');
  String command = line;
  String argument;

  if (space >= 0) {
    command = line.substring(0, space);
    argument = line.substring(space + 1);
    argument.trim();
  }

  if (argument.length() == 0) {
    if (command == "stop") {
      control.controlStopped = 1;
    } else if (command == "resume") {
      control.controlStopped = 0;
    } else if (command == "unlock") {
      control.controlServoAngle = SERVO_FREE;
      control.controlMotorPwm = MOTOR_FREE;
      control.controlSpeedLimit = LIMIT_FREE;
      control.controlStopped = 0;
    } else {
      return false;
    }

    return true;
  }

  if (command == "gps" && argument == "reset") {
    ++control.controlGpsResetId;

    return true;
  }

  if (command == "servo-config") {
    int center, deflection;

    if (sscanf(argument.c_str(), "%d %d", &center, &deflection) != 2) {
      return false;
    }

    control.controlServoCenter = center;
    control.controlServoDeflection = deflection;

    return true;
  }

  bool release = argument == "free";
  int value = argument.toInt();

  if (command == "servo") {
    control.controlServoAngle = release ? SERVO_FREE : value;
  } else if (command == "motor") {
    control.controlMotorPwm = release ? MOTOR_FREE : value;
  } else if (command == "limit") {
    control.controlSpeedLimit = release ? LIMIT_FREE : value;
  } else {
    return false;
  }

  return true;
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char next = Serial.read();

    if (next == '\r' || next == '\n') {
      if (serialOverflow || serialLine.length() > 0) {
        serialLine.trim();
        serialLine.toLowerCase();

        CommandAckPacket ack;
        ack.controlSequence = control.controlSequence;
        ack.commandAccepted = !serialOverflow && applyCommand(serialLine);

        sendPacket(Serial, PacketType::CommandAck, ack);
      }

      serialLine = "";
      serialOverflow = false;
    } else if (serialLine.length() < 63 && next != '\0') {
      serialLine += next;
    } else {
      serialOverflow = true;
    }
  }
}

void readTelemetry() {
  while (bluetoothReader.read(bluetooth)) {
    if (bluetoothReader.type() == PacketType::RxTelemetry &&
        bluetoothReader.copy(telemetry.rx)) {
      lastTelemetry = millis();
      receivedTelemetry = true;
      receivedTelemetryOnLink = true;
      ++telemetry.txTelemetryPackets;
    } else if (bluetoothReader.type() == PacketType::GpsSentence) {
      GpsSentencePacket sentence;

      if (bluetoothReader.copy(sentence)) {
        sendPacket(Serial, PacketType::GpsSentence, sentence);
      }
    }
  }
}

void sendControls(bool connected) {
  telemetry.rawTxJ1X = analogRead(J1_VRX);
  telemetry.rawTxJ1Y = analogRead(J1_VRY);
  telemetry.rawTxJ2X = analogRead(J2_VRX);
  telemetry.rawTxJ2Y = analogRead(J2_VRY);

  control.controlJ1X = map(telemetry.rawTxJ1X, 0, 4095, -255, 255);
  control.controlJ1Y = map(telemetry.rawTxJ1Y, 0, 4095, 255, -255);
  control.controlJ2X = map(telemetry.rawTxJ2X, 0, 4095, -255, 255);
  control.controlJ2Y = map(telemetry.rawTxJ2Y, 0, 4095, 255, -255);

  if (connected) {
    ++control.controlSequence;
    sendPacket(bluetooth, PacketType::Control, control);
  }
}

void reportTelemetry(bool connected) {
  telemetry.txUptimeMs = millis();
  telemetry.txTelemetryAgeMs = receivedTelemetry
                                    ? telemetry.txUptimeMs - lastTelemetry
                                    : UINT32_MAX;

  telemetry.txConnected = connected;
  telemetry.txTelemetryFresh = connected && receivedTelemetryOnLink &&
                               telemetry.txTelemetryAgeMs < CONTROL_TIMEOUT_MS;

  sendPacket(Serial, PacketType::MonitorTelemetry, telemetry);
}

void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(SERIAL_BAUD);

  pinMode(LED_BUILTIN, OUTPUT);

  analogSetWidth(12);

  serialLine.reserve(64);

  bluetooth.begin(BT_TX_NAME);
}

void loop() {
  bool connected = bluetooth.connected();
  digitalWrite(LED_BUILTIN, connected ? HIGH : LOW);

  if (connected) {
    readTelemetry();
  } else {
    bluetoothReader.reset();
    receivedTelemetryOnLink = false;
  }

  readSerialCommands();

  uint32_t now = millis();

  if (now - lastSend >= CONTROL_INTERVAL_MS) {
    lastSend = now;
    sendControls(connected);
  }

  if (now - lastReport >= TELEMETRY_INTERVAL_MS) {
    lastReport = now;
    reportTelemetry(connected);
  }

  delay(1);
}
