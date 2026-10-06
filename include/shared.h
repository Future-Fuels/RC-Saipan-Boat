#pragma once

#define BT_RX_NAME "rcboat_rx"
#define BT_TX_NAME "rcboat_tx"

struct BT_TX_Packet {
  int j1x;
  int j1y;
  int j2x;
  int j2y;
  int button;
  int speed;
  int calibrate;
};

struct BT_RX_Packet {
  float accX;
  float accY;
  float accZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float resG;
};

/// rx
#define BT_RX_NAME "rcboat_rx"
#define BT_TX_NAME "rcboat_tx"

// imu 6-axis
#define IMU_ADDR 0x68
#define IMU_SDA 21
#define IMU_SCL 22

// motors
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

/// tx
#define TX_WIFI_AP_SSID "futurefuels_boat"
#define TX_WIFI_AP_PASS "12345678"

#define J1_VRX 34 // orange
#define J1_VRY 35 // yellow
#define J2_VRX 36 // red
#define J2_VRY 39 // brown
#define BUTTON1 13

#define ROTARY_DT_PIN 32
#define ROTARY_CLK_PIN 33
#define ROTARY_MAX 255
#define ROTARY_MIN 0
#define ROTARY_STEP 5
