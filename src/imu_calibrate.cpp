#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>

#include "rx.h"

#define CALIBRATION_REPORT_MS 500

Adafruit_BNO055 bno(55, IMU_ADDR, &Wire);

bool imuAvailable = false;
bool saved = false;

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(SERVO_1, OUTPUT);
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(SERVO_1, LOW);

  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setTimeOut(20);

  imuAvailable = bno.begin(OPERATION_MODE_NDOF);

  if (!imuAvailable) {
    Serial.println("bno055 not found; check address 0x29 and i2c wiring");
    return;
  }

  bno.setExtCrystalUse(true);

  Serial.println("bno055 calibration: 0 = uncalibrated, 3 = calibrated");
  Serial.println("gyro: hold still for a few seconds");
  Serial.println("accel: hold each of the six faces still for a few seconds");
  Serial.println("mag: rotate through different directions or figure eights");
  Serial.println("saves automatically when system, gyro, accel and mag reach 3");
}

void loop() {
  delay(CALIBRATION_REPORT_MS);

  if (!imuAvailable)
    return;

  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  Serial.printf("system: %u  gyro: %u  accel: %u  mag: %u%s\n", system,
                gyro, accel, mag, saved ? "  saved" : "");

  if (saved)
    return;

  adafruit_bno055_offsets_t offsets{};

  if (!bno.getSensorOffsets(offsets))
    return;

  Preferences preferences;

  if (!preferences.begin(IMU_CALIBRATION_NAMESPACE)) {
    Serial.println("cannot open calibration storage; retrying");
    return;
  }

  saved = preferences.putBytes(IMU_CALIBRATION_KEY, &offsets, sizeof(offsets)) ==
          sizeof(offsets);
  preferences.end();

  if (!saved) {
    Serial.println("could not save calibration; retrying");
    return;
  }

  digitalWrite(LED_BUILTIN, HIGH);

  Serial.printf("accel offsets: %d %d %d  radius: %d\n",
                offsets.accel_offset_x, offsets.accel_offset_y,
                offsets.accel_offset_z, offsets.accel_radius);
  Serial.printf("gyro offsets: %d %d %d\n", offsets.gyro_offset_x,
                offsets.gyro_offset_y, offsets.gyro_offset_z);
  Serial.printf("mag offsets: %d %d %d  radius: %d\n", offsets.mag_offset_x,
                offsets.mag_offset_y, offsets.mag_offset_z, offsets.mag_radius);

  Serial.println("calibration saved; upload rx to this same esp32");
}
