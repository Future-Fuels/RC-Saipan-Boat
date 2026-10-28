#include <Arduino.h>
#include <BluetoothSerial.h>
#include <errno.h>

#include "shared.h"

BluetoothSerial bluetooth;
ControlPacket control;
volatile int rotaryValue = 0;
int lastClock = HIGH;
uint32_t lastSend = 0;
uint32_t lastReport = 0;
uint32_t lastTelemetry = 0;

String serialLine;
String bluetoothLine;
String receiverTelemetry;
bool serialOverflow = false;
bool bluetoothOverflow = false;
char commandLine[CONTROL_LINE_SIZE];
char jsonLine[TELEMETRY_JSON_SIZE];

void IRAM_ATTR updateEncoder() {
  int clock = digitalRead(ROTARY_CLK_PIN);

  if (clock != lastClock) {
    int direction = digitalRead(ROTARY_DT_PIN) != clock ? -1 : 1;
    rotaryValue = constrain(rotaryValue + direction * ROTARY_STEP, ROTARY_MIN,
                            ROTARY_MAX);
    lastClock = clock;
  }
}

bool readLine(Stream &input, String &line, bool &overflow, size_t limit) {
  while (input.available() > 0) {
    char c = input.read();

    if (c == '\n' || c == '\r') {
      bool ready = !overflow && line.length() > 0;
      overflow = false;

      if (ready) {
        return true;
      }

      line = "";
    } else if (line.length() < limit - 1 && c != '\0') {
      line += c;
    } else {
      overflow = true;
    }
  }

  return false;
}

bool applyCommand(String line) {
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
      control.stopped = 1;
    } else if (command == "resume") {
      control.stopped = 0;
    } else if (command == "unlock") {
      control.servoLock = SERVO_FREE;
      control.motorLock = MOTOR_FREE;
      control.speedLimit = LIMIT_FREE;
      control.stopped = 0;
    } else {
      return false;
    }

    return true;
  }

  if (command == "gps" && argument == "reset") {
    ++control.gpsResetId;
    return true;
  }

  bool release = argument == "free";
  long value = 0;

  if (!release) {
    char *end;
    errno = 0;
    value = strtol(argument.c_str(), &end, 10);

    if (errno != 0 || end == argument.c_str() || *end != '\0') {
      return false;
    }
  }

  if (command == "servo" && (release || (value >= 0 && value <= 180))) {
    control.servoLock = release ? SERVO_FREE : value;
  } else if (command == "motor" &&
             (release || (value >= -255 && value <= 255))) {
    control.motorLock = release ? MOTOR_FREE : value;
  } else if (command == "limit" && (release || (value >= 0 && value <= 255))) {
    control.speedLimit = release ? LIMIT_FREE : value;
  } else {
    return false;
  }

  return true;
}

void readSerialCommands() {
  while (readLine(Serial, serialLine, serialOverflow, 64)) {
    serialLine.trim();
    serialLine.toLowerCase();

    bool accepted = applyCommand(serialLine);
    Serial.printf("{\"commandAccepted\":%s}\n", accepted ? "true" : "false");

    serialLine = "";
  }
}

void readTelemetry() {
  while (readLine(bluetooth, bluetoothLine, bluetoothOverflow,
                  TELEMETRY_JSON_SIZE)) {
    if (bluetoothLine.startsWith("{") && bluetoothLine.endsWith("}")) {
      receiverTelemetry = bluetoothLine;
      lastTelemetry = millis();
    }

    bluetoothLine = "";
  }
}

void sendControls(bool connected) {
  control.j1x = map(analogRead(J1_VRX), 0, 4095, -255, 255);
  control.j1y = map(analogRead(J1_VRY), 0, 4095, 255, -255);
  control.j2x = map(analogRead(J2_VRX), 0, 4095, -255, 255);
  control.j2y = map(analogRead(J2_VRY), 0, 4095, 255, -255);
  control.speed = rotaryValue;

  if (!connected) {
    return;
  }

  snprintf(commandLine, sizeof(commandLine),
           "C %d %d %d %d %d %d %d %d %d %u\n", control.j1x, control.j1y,
           control.j2x, control.j2y, control.speed, control.servoLock,
           control.motorLock, control.speedLimit, control.stopped,
           control.gpsResetId);

  bluetooth.print(commandLine);
}

void reportTelemetry(bool connected) {
  bool fresh = connected && receiverTelemetry.length() > 0 &&
               millis() - lastTelemetry < CONTROL_TIMEOUT_MS;
  const char *data =
      "\"accX\":0,\"accY\":0,\"accZ\":0,\"resG\":0,"
      "\"gyroX\":0,\"gyroY\":0,\"gyroZ\":0,\"imuAvailable\":false,"
      "\"gpsFix\":false,\"filterInitialized\":false,\"gpsResetId\":0,"
      "\"active\":false,\"motorSpeed\":0,\"servoAngle\":90,"
      "\"servoLocked\":false,\"motorLocked\":false,\"speedLimit\":-1,"
      "\"stopped\":"
      "false}";

  if (fresh) {
    data = receiverTelemetry.c_str() + 1;
  }

  int length = snprintf(
      jsonLine, sizeof(jsonLine),
      "{\"j1x\":%d,\"j1y\":%d,\"j2x\":%d,\"j2y\":%d,\"speed\":%d,"
      "\"connected\":%s,\"telemetryFresh\":%s,%s\n",
      control.j1x, control.j1y, control.j2x, control.j2y, control.speed,
      connected ? "true" : "false", fresh ? "true" : "false", data);

  if (length > 0 && length < TELEMETRY_JSON_SIZE) {
    Serial.write(jsonLine, length);
  }
}

void setup() {
  Serial.setTxBufferSize(TELEMETRY_JSON_SIZE);
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ROTARY_CLK_PIN, INPUT_PULLUP);
  pinMode(ROTARY_DT_PIN, INPUT_PULLUP);
  lastClock = digitalRead(ROTARY_CLK_PIN);
  attachInterrupt(digitalPinToInterrupt(ROTARY_CLK_PIN), updateEncoder, CHANGE);
  analogSetWidth(12);

  serialLine.reserve(64);
  bluetoothLine.reserve(TELEMETRY_JSON_SIZE);
  receiverTelemetry.reserve(TELEMETRY_JSON_SIZE);
  bluetooth.begin(BT_TX_NAME);
}

void loop() {
  bool connected = bluetooth.connected();
  digitalWrite(LED_BUILTIN, connected ? HIGH : LOW);
  readSerialCommands();

  if (connected) {
    readTelemetry();
  } else {
    receiverTelemetry = "";
    bluetoothLine = "";
    bluetoothOverflow = false;
  }

  if (millis() - lastSend >= CONTROL_INTERVAL_MS) {
    lastSend = millis();
    sendControls(connected);
  }

  if (millis() - lastReport >= TELEMETRY_INTERVAL_MS) {
    lastReport = millis();
    reportTelemetry(connected);
  }

  delay(1);
}
