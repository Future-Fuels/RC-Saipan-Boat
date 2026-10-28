#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include <errno.h>
#include <esp_now.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "shared.h"

extern const char monitorHtml[] asm("_binary_monitor_esp_now_index_html_start");
extern const char monitorJs[] asm("_binary_monitor_esp_now_app_js_start");

ControlPacket control;
EspNowTelemetry receiver{};
WebServer server(80);
volatile int rotaryValue = 0;
int lastClock = HIGH;
uint32_t lastSend = 0;
uint32_t lastTelemetry = 0;
uint32_t telemetryPackets = 0;
String serialLine;
bool serialOverflow = false;
bool radioReady = false;
volatile bool radioBusy = false;
uint8_t rxMac[] = ESPNOW_RX_MAC;

struct PendingTelemetry {
  EspNowTelemetry packet;
  uint32_t receivedAt;
};

QueueHandle_t telemetryQueue;
SemaphoreHandle_t stateMutex;

static_assert(sizeof(ControlPacket) == 40, "Update the monitor control decoder");
static_assert(sizeof(TelemetryPacket) == 136, "Update the monitor sensor decoder");
static_assert(offsetof(MonitorPacket, receiver) == 56 && sizeof(MonitorPacket) == 232,
              "Update the monitor packet decoder");
static_assert(sizeof(EspNowTelemetry) <= ESP_NOW_MAX_DATA_LEN,
              "Telemetry must fit in one ESP-NOW packet");

void IRAM_ATTR updateEncoder() {
  int clock = digitalRead(ROTARY_CLK_PIN);

  if (clock != lastClock) {
    int direction = digitalRead(ROTARY_DT_PIN) != clock ? -1 : 1;
    rotaryValue =
        constrain(rotaryValue + direction * ROTARY_STEP, ROTARY_MIN, ROTARY_MAX);
    lastClock = clock;
  }
}

bool readLine(Stream& input, String& line, bool& overflow, size_t limit) {
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
  line.trim();
  line.toLowerCase();
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
    char* end;
    errno = 0;
    value = strtol(argument.c_str(), &end, 10);

    if (errno != 0 || end == argument.c_str() || *end != '\0') {
      return false;
    }
  }

  if (command == "servo" && (release || (value >= 0 && value <= 180))) {
    control.servoLock = release ? SERVO_FREE : value;
  } else if (command == "motor" && (release || (value >= -255 && value <= 255))) {
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
    applyCommand(serialLine);
    serialLine = "";
  }
}

void onTelemetryReceived(const uint8_t* mac, const uint8_t* data, int length) {
  if (length != sizeof(EspNowTelemetry) || memcmp(mac, rxMac, sizeof(rxMac)) != 0) {
    return;
  }

  PendingTelemetry pending;
  memcpy(&pending.packet, data, sizeof(pending.packet));
  pending.receivedAt = millis();
  xQueueOverwrite(telemetryQueue, &pending);
}

void onControlSent(const uint8_t* mac, esp_now_send_status_t status) {
  radioBusy = false;
}

bool setupRadio() {
  uint8_t localMac[] = ESPNOW_TX_MAC;
  uint8_t key[] = ESPNOW_KEY;

  WiFi.persistent(false);
  if (esp_base_mac_addr_set(localMac) != ESP_OK || !WiFi.mode(WIFI_AP_STA)) {
    return false;
  }

  WiFi.setSleep(false);
  if (!WiFi.softAP(MONITOR_SSID, MONITOR_PASSWORD, ESPNOW_CHANNEL) ||
      esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK ||
      esp_now_init() != ESP_OK) {
    return false;
  }

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, rxMac, sizeof(rxMac));
  memcpy(peer.lmk, key, sizeof(key));
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = true;

  return esp_now_add_peer(&peer) == ESP_OK &&
         esp_now_register_recv_cb(onTelemetryReceived) == ESP_OK &&
         esp_now_register_send_cb(onControlSent) == ESP_OK;
}

void readTelemetry() {
  PendingTelemetry pending;
  if (xQueueReceive(telemetryQueue, &pending, 0) == pdTRUE) {
    receiver = pending.packet;
    lastTelemetry = pending.receivedAt;
    ++telemetryPackets;
  }
}

bool telemetryFresh() {
  return telemetryPackets > 0 && millis() - lastTelemetry < CONTROL_TIMEOUT_MS;
}

void sendControls() {
  control.j1x = map(analogRead(J1_VRX), 0, 4095, -255, 255);
  control.j1y = map(analogRead(J1_VRY), 0, 4095, 255, -255);
  control.j2x = map(analogRead(J2_VRX), 0, 4095, -255, 255);
  control.j2y = map(analogRead(J2_VRY), 0, 4095, 255, -255);
  control.speed = rotaryValue;

  if (!radioReady || radioBusy) {
    return;
  }

  radioBusy = true;
  if (esp_now_send(rxMac, (uint8_t*)&control, sizeof(control)) != ESP_OK) {
    radioBusy = false;
  }
}

void serveTelemetry() {
  MonitorPacket packet{};
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  packet.uptimeMs = millis();
  packet.telemetryPackets = telemetryPackets;
  packet.telemetryAgeMs =
      telemetryPackets > 0 ? packet.uptimeMs - lastTelemetry : UINT32_MAX;
  packet.control = control;
  packet.receiver = receiver;
  xSemaphoreGive(stateMutex);

  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(sizeof(packet));
  server.send(200, "application/octet-stream", "");
  server.sendContent((char*)&packet, sizeof(packet));
}

void serveCommand() {
  String line = server.arg("plain");
  bool accepted = false;
  if (line.length() > 0 && line.length() < 64) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    accepted = applyCommand(line);
    xSemaphoreGive(stateMutex);
  }

  if (!accepted) {
    server.send(400, "text/plain", "Invalid command");
    return;
  }

  server.send(200, "text/plain", "");
}

void setupMonitor() {
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-cache");
    server.send_P(200, "text/html; charset=utf-8", monitorHtml);
  });
  server.on("/app.js", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-cache");
    server.send_P(200, "text/javascript; charset=utf-8", monitorJs);
  });
  server.on("/api/telemetry", HTTP_GET, serveTelemetry);
  server.on("/api/command", HTTP_POST, serveCommand);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.enableDelay(false);
  server.begin();
}

void monitorLoop(void* parameter) {
  while (true) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ROTARY_CLK_PIN, INPUT_PULLUP);
  pinMode(ROTARY_DT_PIN, INPUT_PULLUP);
  lastClock = digitalRead(ROTARY_CLK_PIN);
  attachInterrupt(digitalPinToInterrupt(ROTARY_CLK_PIN), updateEncoder, CHANGE);
  analogSetWidth(12);
  serialLine.reserve(64);

  receiver.telemetry.servoAngle = SERVO_DEFAULT_ANGLE;
  receiver.telemetry.controlAgeMs = UINT32_MAX;
  receiver.telemetry.gpsAgeMs = UINT32_MAX;
  telemetryQueue = xQueueCreate(1, sizeof(PendingTelemetry));
  stateMutex = xSemaphoreCreateMutex();
  if (telemetryQueue) {
    radioReady = setupRadio();
  }

  setupMonitor();
  if (!stateMutex || xTaskCreatePinnedToCore(monitorLoop, "monitor", 8192, nullptr, 1,
                                             nullptr, 0) != pdPASS) {
    Serial.println("Monitor task setup failed");
  }

  Serial.println(radioReady ? "TX ESP-NOW ready" : "TX ESP-NOW setup failed");
  Serial.printf("Monitor: http://%s\n", WiFi.softAPIP().toString().c_str());
}

void loop() {
  if (!stateMutex) {
    delay(100);
    return;
  }

  // Share snapshots and settings with the web task; never hold this during HTTP I/O.
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  readSerialCommands();
  if (radioReady) {
    readTelemetry();
  }

  if (millis() - lastSend >= CONTROL_INTERVAL_MS) {
    lastSend = millis();
    sendControls();
  }

  digitalWrite(LED_BUILTIN, telemetryFresh() ? HIGH : LOW);
  xSemaphoreGive(stateMutex);
  delay(1);
}
