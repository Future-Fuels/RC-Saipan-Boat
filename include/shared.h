#pragma once

#include <stddef.h>
#include <stdint.h>

// bt
#define BT_RX_NAME "rcboat_rx"
#define BT_TX_NAME "rcboat_tx"
#define CONTROL_INTERVAL_MS 50UL
#define TELEMETRY_INTERVAL_MS 100UL
#define CONTROL_TIMEOUT_MS 500UL
#define CONNECT_RETRY_MS 3000UL

// esp-now stuff
#define ESPNOW_CHANNEL 1
#define ESPNOW_TX_MAC {0x02, 0x52, 0x43, 0x42, 0x00, 0x10}
#define ESPNOW_RX_MAC {0x02, 0x52, 0x43, 0x42, 0x00, 0x20}
#define ESPNOW_KEY                                 \
  {0x72, 0x63, 0x62, 0x6f, 0x61, 0x74, 0x2d, 0x65, \
   0x73, 0x70, 0x6e, 0x6f, 0x77, 0x2d, 0x30, 0x31}
#define MONITOR_SSID "futurefuels_rcboat"
#define MONITOR_PASSWORD "12345678"
#define MONITOR_PACKET_VERSION 1

// joystick/motor response
#define JOYSTICK_DEADBAND 25
#define JOYSTICK_EXPO 0.65f
#define MOTOR_SLEW_PER_SECOND 600.0f
#define SERVO_SLEW_PER_SECOND 240.0f

// bno055 imu
#define IMU_ADDR 0x29
#define IMU_SDA 21
#define IMU_SCL 22

// gps
#define GPS_RX 16
#define GPS_TX 17

// l298n motor
#define LEFT_IN1 33
#define LEFT_IN2 25
#define RIGHT_IN3 26
#define RIGHT_IN4 27
#define ENA 32
#define ENB 14

// servo
#define SERVO_1 12
#define SERVO_2 13
#define SERVO_DEFAULT_ANGLE 90

/// tx pins
#define J1_VRX 34  // orange
#define J1_VRY 35  // yellow
#define J2_VRX 36  // red
#define J2_VRY 39  // brown

#define ROTARY_DT_PIN 32
#define ROTARY_CLK_PIN 33
#define ROTARY_MAX 255
#define ROTARY_MIN 0
#define ROTARY_STEP 5

#define CONTROL_LINE_SIZE 192
#define TELEMETRY_JSON_SIZE 2048
#define SERVO_FREE (-1)
#define MOTOR_FREE (-256)
#define LIMIT_FREE (-1)

// sent from tx to rx to set values etc.
struct ControlPacket {
  int j1x = 0;
  int j1y = 0;
  int j2x = 0;
  int j2y = 0;
  int speed = 0;

  int servoLock = SERVO_FREE;
  int motorLock = MOTOR_FREE;
  int speedLimit = LIMIT_FREE;
  int stopped = 0;
  unsigned int gpsResetId = 0;
};

// acceleration in g, gyro in deg/s, position in meters, velocity in m/s.
struct TelemetryPacket {
  float accX;
  float accY;
  float accZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float resG;
  float linearAccX;  // m/s^2, gravity removed
  float linearAccY;
  float linearAccZ;
  float quatW;
  float quatX;
  float quatY;
  float quatZ;
  double latitude;
  double longitude;
  float altitude;   // meters
  float gpsSpeed;   // m/s
  float gpsCourse;  // degrees
  float east;       // filtered local position, meters from the first fix
  float north;
  float eastVelocity;  // m/s
  float northVelocity;
  unsigned long gpsAgeMs;
  unsigned long satellites;
  int motorSpeed;  // applied signed PWM, -255..255
  int servoAngle;  // applied angle, degrees
  unsigned long controlPackets;
  unsigned long controlAgeMs;

  uint8_t active;
  uint8_t imuAvailable;
  uint8_t gpsFix;
  uint8_t gpsSpeedValid;
  uint8_t gpsCourseValid;
  uint8_t gpsAltitudeValid;
  uint8_t filterInitialized;
  uint8_t systemCalibration;
  uint8_t gyroCalibration;
  uint8_t accelCalibration;
  uint8_t magCalibration;
};

struct EspNowTelemetry {
  TelemetryPacket telemetry;
  ControlPacket applied;
};

struct MonitorPacket {
  uint32_t version = MONITOR_PACKET_VERSION;
  uint32_t uptimeMs = 0;
  uint32_t telemetryPackets = 0;
  uint32_t telemetryAgeMs = UINT32_MAX;
  ControlPacket control;
  EspNowTelemetry receiver;
};
