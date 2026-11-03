#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

#define BT_RX_NAME "rcboat_rx"
#define BT_TX_NAME "rcboat_tx"

#define CONTROL_INTERVAL_MS 50UL
#define TELEMETRY_INTERVAL_MS 50UL
#define SERIAL_BAUD 460800UL
#define CONTROL_TIMEOUT_MS 500UL
#define CONNECT_RETRY_MS 3000UL

#define SERVO_DEFAULT_ANGLE 60
#define SERVO_MAX_DEFLECTION 30

#define SERVO_FREE (-1)
#define MOTOR_FREE (-256)
#define LIMIT_FREE (-1)

#pragma pack(push, 1)

struct ControlPacket {
  uint32_t controlSequence = 0;
  uint32_t controlGpsResetId = 0;

  int16_t controlJ1X = 0;
  int16_t controlJ1Y = 0;
  int16_t controlJ2X = 0;
  int16_t controlJ2Y = 0;

  int16_t controlServoAngle = SERVO_FREE;
  int16_t controlMotorPwm = MOTOR_FREE;
  int16_t controlSpeedLimit = LIMIT_FREE;

  int16_t controlServoCenter = SERVO_DEFAULT_ANGLE;
  int16_t controlServoDeflection = SERVO_MAX_DEFLECTION;
  uint8_t controlStopped = 0;
};

struct RxTelemetryPacket {
  uint32_t rxUptimeMs = 0;
  uint32_t rxImuSampleMs = 0;
  uint32_t rxGpsSampleMs = 0;
  uint32_t rxControlAgeMs = UINT32_MAX;
  uint32_t rxControlPackets = 0;
  uint32_t rxControlSequence = 0;

  double rawImuAccelX = 0;
  double rawImuAccelY = 0;
  double rawImuAccelZ = 0;

  double rawImuGyroX = 0;
  double rawImuGyroY = 0;
  double rawImuGyroZ = 0;

  double rawImuMagX = 0;
  double rawImuMagY = 0;
  double rawImuMagZ = 0;

  double rawImuEulerX = 0;
  double rawImuEulerY = 0;
  double rawImuEulerZ = 0;

  double rawImuLinearAccelX = 0;
  double rawImuLinearAccelY = 0;
  double rawImuLinearAccelZ = 0;

  double rawImuGravityX = 0;
  double rawImuGravityY = 0;
  double rawImuGravityZ = 0;

  double rawImuQuatW = 0;
  double rawImuQuatX = 0;
  double rawImuQuatY = 0;
  double rawImuQuatZ = 0;

  double rawGpsLatitude = 0;
  double rawGpsLongitude = 0;
  double rawGpsAltitude = 0;
  double rawGpsSpeed = 0;
  double rawGpsCourse = 0;
  double rawGpsHdop = 0;
  double rawGpsUtc = 0;
  uint32_t rawGpsDate = 0;
  uint32_t rawGpsSatellites = 0;
  int8_t rawImuTemperature = 0;
  uint8_t rawGpsFixQuality = 0;
  uint8_t rawGpsMode = 0;

  uint8_t imuAvailable = 0;
  uint8_t imuSystemCalibration = 0;
  uint8_t imuGyroCalibration = 0;
  uint8_t imuAccelCalibration = 0;
  uint8_t imuMagCalibration = 0;

  uint32_t gpsPositionAgeMs = UINT32_MAX;
  uint32_t gpsSpeedAgeMs = UINT32_MAX;
  uint32_t gpsCourseAgeMs = UINT32_MAX;
  uint32_t gpsAltitudeAgeMs = UINT32_MAX;

  uint32_t gpsPositionUpdates = 0;
  uint32_t gpsPositionChanges = 0;
  uint32_t gpsSpeedUpdates = 0;

  uint32_t gpsCharsProcessed = 0;
  uint32_t gpsPassedChecksum = 0;
  uint32_t gpsFailedChecksum = 0;
  uint32_t gpsSentencesWithFix = 0;

  uint8_t gpsPositionValid = 0;
  uint8_t gpsSpeedValid = 0;
  uint8_t gpsCourseValid = 0;
  uint8_t gpsAltitudeValid = 0;
  uint8_t gpsHdopValid = 0;
  uint8_t gpsSatellitesValid = 0;

  uint8_t gpsOriginReady = 0;
  double gpsOriginLatitude = 0;
  double gpsOriginLongitude = 0;
  double gpsEast = 0;
  double gpsNorth = 0;

  float imuEast = 0;
  float imuNorth = 0;
  float imuEastVelocity = 0;
  float imuNorthVelocity = 0;
  float imuEastAcceleration = 0;
  float imuNorthAcceleration = 0;

  float filterEast = 0;
  float filterNorth = 0;
  float filterEastVelocity = 0;
  float filterNorthVelocity = 0;
  float filterEastAcceleration = 0;
  float filterNorthAcceleration = 0;
  uint8_t filterInitialized = 0;
  uint8_t filterImuUsed = 0;

  int16_t outputMotorPwm = 0;
  int16_t outputServoAngle = 0;
  uint8_t outputActive = 0;

  ControlPacket control;
};

struct MonitorTelemetryPacket {
  uint32_t txUptimeMs = 0;
  uint32_t txTelemetryAgeMs = UINT32_MAX;
  uint32_t txTelemetryPackets = 0;

  uint16_t rawTxJ1X = 0;
  uint16_t rawTxJ1Y = 0;
  uint16_t rawTxJ2X = 0;
  uint16_t rawTxJ2Y = 0;

  uint8_t txConnected = 0;
  uint8_t txTelemetryFresh = 0;

  ControlPacket control;
  RxTelemetryPacket rx;
};

struct GpsSentencePacket {
  uint32_t rxUptimeMs = 0;
  char rawGpsSentence[128]{};
};

struct CommandAckPacket {
  uint32_t controlSequence = 0;
  uint8_t commandAccepted = 0;
};

#pragma pack(pop)

enum class PacketType : uint8_t {
  Control = 1,
  RxTelemetry = 2,
  MonitorTelemetry = 3,
  GpsSentence = 4,
  CommandAck = 5,
};

#define PACKET_VERSION 2
#define PACKET_MAX_PAYLOAD (sizeof(MonitorTelemetryPacket))
#define PACKET_MAX_SIZE (PACKET_MAX_PAYLOAD + 8)

inline uint16_t packetCrc(const uint8_t *data, size_t length) {
  uint16_t crc = 0xffff;

  for (size_t i = 0; i < length; ++i) {
    crc ^= uint16_t(data[i]) << 8;

    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
    }
  }

  return crc;
}

// rb, version, type, uint16 length, payload, crc16-ccitt-false.
template <typename T>
bool sendPacket(Stream &output, PacketType type, const T &packet) {
  if (sizeof(T) > PACKET_MAX_PAYLOAD) {
    return false;
  }

  uint8_t frame[sizeof(T) + 8];
  frame[0] = 'R';
  frame[1] = 'B';
  frame[2] = PACKET_VERSION;
  frame[3] = static_cast<uint8_t>(type);
  frame[4] = sizeof(T) & 0xff;
  frame[5] = (sizeof(T) >> 8) & 0xff;

  memcpy(frame + 6, &packet, sizeof(T));

  uint16_t crc = packetCrc(frame + 2, sizeof(T) + 4);
  frame[sizeof(T) + 6] = crc & 0xff;
  frame[sizeof(T) + 7] = crc >> 8;

  return output.write(frame, sizeof(frame)) == sizeof(frame);
}

class PacketReader {
public:
  bool read(Stream &input) {
    ready = false;

    while (input.available() > 0) {
      int next = input.read();
      if (next < 0)
        break;

      if (used == 0 && next != 'R')
        continue;
      if (used == 1 && next != 'B') {
        used = next == 'R' ? 1 : 0;
        continue;
      }

      buffer[used++] = static_cast<uint8_t>(next);

      if (used == 6) {
        frameLength = (buffer[4] | (uint16_t(buffer[5]) << 8)) + 8;
        if (buffer[2] != PACKET_VERSION || frameLength > sizeof(buffer)) {
          used = 0;
          continue;
        }
      }

      if (used >= 8 && used == frameLength) {
        used = 0;
        uint16_t crc =
            buffer[frameLength - 2] | (uint16_t(buffer[frameLength - 1]) << 8);
        ready = crc == packetCrc(buffer + 2, frameLength - 4);

        if (ready)
          return true;
      }
    }

    return false;
  }

  template <typename T>
  bool read(Stream &input, PacketType expected, T &packet) {
    while (read(input)) {
      if (type() == expected && copy(packet))
        return true;
    }

    return false;
  }

  PacketType type() const { return static_cast<PacketType>(buffer[3]); }

  template <typename T> bool copy(T &packet) const {
    if (!ready || frameLength != sizeof(T) + 8) {
      return false;
    }

    memcpy(&packet, buffer + 6, sizeof(T));

    return true;
  }

  void reset() {
    used = 0;
    frameLength = 0;
    ready = false;
  }

private:
  uint8_t buffer[PACKET_MAX_SIZE]{};
  size_t used = 0;
  size_t frameLength = 0;
  bool ready = false;
};
