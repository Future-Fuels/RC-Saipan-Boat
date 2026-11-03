#pragma once

#include "shared.h"

#define IMU_ADDR 0x29
#define IMU_SDA 21
#define IMU_SCL 22

#define GPS_RX 16
#define GPS_TX 17

#define LEFT_IN1 33
#define LEFT_IN2 25
#define RIGHT_IN3 26
#define RIGHT_IN4 27
#define ENA 32
#define ENB 14

#define SERVO_1 12

struct AxisFilter {
  float position = 0;
  float velocity = 0;

  float positionVariance = 25;
  float positionVelocityCovariance = 0;
  float velocityVariance = 0.25f;

  void predict(float dt, float acceleration, float accelerationNoise) {
    position += velocity * dt + 0.5f * acceleration * dt * dt;
    velocity += acceleration * dt;

    // continuous acceleration noise keeps uncertainty independent of loop rate.
    positionVariance += 2 * dt * positionVelocityCovariance +
                        dt * dt * velocityVariance +
                        accelerationNoise * dt * dt * dt / 3;
    positionVelocityCovariance += dt * velocityVariance +
                                  accelerationNoise * dt * dt / 2;
    velocityVariance += accelerationNoise * dt;
  }

  void correctPosition(float measurement, float variance) {
    float residual = measurement - position;
    float total = positionVariance + variance;

    float positionGain = positionVariance / total;
    float velocityGain = positionVelocityCovariance / total;

    position += positionGain * residual;
    velocity += velocityGain * residual;

    velocityVariance -= velocityGain * positionVelocityCovariance;
    positionVariance = variance * positionGain;
    positionVelocityCovariance = variance * velocityGain;
  }

  void correctVelocity(float measurement, float variance) {
    float residual = measurement - velocity;
    float total = velocityVariance + variance;

    float positionGain = positionVelocityCovariance / total;
    float velocityGain = velocityVariance / total;

    position += positionGain * residual;
    velocity += velocityGain * residual;

    positionVariance -= positionGain * positionVelocityCovariance;
    positionVelocityCovariance = variance * positionGain;
    velocityVariance = variance * velocityGain;
  }
};
