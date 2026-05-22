#pragma once

#define BT_RX_NAME "rcboat_rx"
#define BT_TX_NAME "rcboat_tx"

struct BTPacket {
  int x;
  int y;
  int button;
};

/// rx
#define BT_RX_NAME "rcboat_rx"
#define BT_TX_NAME "rcboat_tx"

// motors
#define LEFT_IN1 33
#define LEFT_IN2 25
#define RIGHT_IN3 26
#define RIGHT_IN4 27
#define ENA 32
#define ENB 14

// servo
#define SERVO 12

/// tx
#define J_VRX 34
#define J_VRY 35
#define BUTTON1 13
