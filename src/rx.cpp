#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <stdlib.h>

#include "rx.h"

#define GPS_BAUD 9600UL
#define GPS_MAX_AGE_MS 2000UL
#define METERS_PER_DEGREE 111320.0

#define GPS_POSITION_VARIANCE 4.0f
#define GPS_VELOCITY_VARIANCE 0.04f
#define FILTER_ACCELERATION_NOISE 0.25f // m^2/s^3

#define IMU_ACCELERATION_DEADBAND 0.10f
#define IMU_DECLINATION_DEGREES 0.0f // east-positive magnetic declination

#define JOYSTICK_DEADBAND 25
#define JOYSTICK_EXPO 0.65f
#define MOTOR_SLEW_PER_SECOND 600.0f
#define SERVO_SLEW_PER_SECOND 240.0f

BluetoothSerial bluetooth;
PacketReader controlReader;
RxTelemetryPacket telemetry;
ControlPacket &control = telemetry.control;

Servo servo;
Adafruit_BNO055 bno(55, IMU_ADDR, &Wire);

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

AxisFilter eastFilter;
AxisFilter northFilter;

uint32_t lastControlMs = 0;
uint32_t lastOutputMs = 0;
uint32_t lastImuMs = 0;
uint32_t lastPredictionMs = 0;
uint32_t lastReportMs = 0;

uint32_t lastGpsPositionMs = 0;
uint32_t lastGpsSpeedMs = 0;
uint32_t lastGpsCourseMs = 0;
uint32_t lastGpsAltitudeMs = 0;
uint32_t lastGpsGgaMs = 0;

double lastGpsPositionEpoch = -1;
double lastGpsVelocityEpoch = -1;

bool receivedControl = false;
bool imuOrientationValid = false;

bool gpsPositionPresent = false;
bool gpsSpeedPresent = false;
bool gpsCoursePresent = false;
bool gpsAltitudePresent = false;
bool gpsHdopPresent = false;
bool gpsSatellitesPresent = false;

bool haveGpsPosition = false;
bool haveGpsSpeed = false;
bool haveGpsCourse = false;
bool haveGpsAltitude = false;

double previousGpsLatitude = 0;
double previousGpsLongitude = 0;

float motorPwm = 0;
float servoAngle = SERVO_DEFAULT_ANGLE;

char gpsLine[128]{};
size_t gpsLineLength = 0;
bool gpsLineOverflow = false;

float joystickResponse(int value) {
  int magnitude = abs(value);

  if (magnitude <= JOYSTICK_DEADBAND)
    return 0;

  float x = (magnitude - JOYSTICK_DEADBAND) / (255.0f - JOYSTICK_DEADBAND);
  float response = (1 - JOYSTICK_EXPO) * x + JOYSTICK_EXPO * x * x * x;

  return value < 0 ? -response : response;
}

float approach(float current, float target, float step) {
  return current + constrain(target - current, -step, step);
}

void resetGpsOrigin() {
  eastFilter = AxisFilter{};
  northFilter = AxisFilter{};

  telemetry.gpsOriginReady = 0;
  telemetry.gpsOriginLatitude = telemetry.gpsOriginLongitude = 0;
  telemetry.gpsEast = telemetry.gpsNorth = 0;

  telemetry.imuEast = telemetry.imuNorth = 0;
  telemetry.imuEastVelocity = telemetry.imuNorthVelocity = 0;

  telemetry.filterInitialized = 0;

  lastGpsPositionEpoch = lastGpsVelocityEpoch = -1;
}

void predictFilter(uint32_t now) {
  float dt = (now - lastPredictionMs) / 1000.0f;
  lastPredictionMs = now;

  if (!telemetry.filterInitialized || dt <= 0)
    return;

  bool imuFresh =
      telemetry.imuAvailable && now - telemetry.rxImuSampleMs <= 100;
  float eastAcceleration = imuFresh ? telemetry.filterEastAcceleration : 0;
  float northAcceleration = imuFresh ? telemetry.filterNorthAcceleration : 0;

  eastFilter.predict(dt, eastAcceleration, FILTER_ACCELERATION_NOISE);
  northFilter.predict(dt, northAcceleration, FILTER_ACCELERATION_NOISE);

  if (imuFresh && imuOrientationValid) {
    telemetry.imuEast += telemetry.imuEastVelocity * dt +
                         0.5f * telemetry.imuEastAcceleration * dt * dt;
    telemetry.imuNorth += telemetry.imuNorthVelocity * dt +
                          0.5f * telemetry.imuNorthAcceleration * dt * dt;

    telemetry.imuEastVelocity += telemetry.imuEastAcceleration * dt;
    telemetry.imuNorthVelocity += telemetry.imuNorthAcceleration * dt;
  }
}

void sampleImu(uint32_t now) {
  predictFilter(now);

  telemetry.filterImuUsed = 0;
  telemetry.filterEastAcceleration = telemetry.filterNorthAcceleration = 0;
  telemetry.imuEastAcceleration = telemetry.imuNorthAcceleration = 0;
  imuOrientationValid = false;

  if (!telemetry.imuAvailable)
    return;

  auto accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  auto gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  auto mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  auto euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  auto linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  auto gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  auto orientation = bno.getQuat();

  telemetry.rxImuSampleMs = now;

  telemetry.rawImuAccelX = accel.x();
  telemetry.rawImuAccelY = accel.y();
  telemetry.rawImuAccelZ = accel.z();

  telemetry.rawImuGyroX = gyro.x();
  telemetry.rawImuGyroY = gyro.y();
  telemetry.rawImuGyroZ = gyro.z();

  telemetry.rawImuMagX = mag.x();
  telemetry.rawImuMagY = mag.y();
  telemetry.rawImuMagZ = mag.z();

  telemetry.rawImuEulerX = euler.x();
  telemetry.rawImuEulerY = euler.y();
  telemetry.rawImuEulerZ = euler.z();

  telemetry.rawImuLinearAccelX = linear.x();
  telemetry.rawImuLinearAccelY = linear.y();
  telemetry.rawImuLinearAccelZ = linear.z();

  telemetry.rawImuGravityX = gravity.x();
  telemetry.rawImuGravityY = gravity.y();
  telemetry.rawImuGravityZ = gravity.z();

  telemetry.rawImuQuatW = orientation.w();
  telemetry.rawImuQuatX = orientation.x();
  telemetry.rawImuQuatY = orientation.y();
  telemetry.rawImuQuatZ = orientation.z();

  telemetry.rawImuTemperature = bno.getTemp();
  bno.getCalibration(
      &telemetry.imuSystemCalibration, &telemetry.imuGyroCalibration,
      &telemetry.imuAccelCalibration, &telemetry.imuMagCalibration);

  double normSquared =
      orientation.w() * orientation.w() + orientation.x() * orientation.x() +
      orientation.y() * orientation.y() + orientation.z() * orientation.z();

  if (!isfinite(linear.magnitude()) || !isfinite(normSquared) ||
      normSquared < 0.9 || normSquared > 1.1)
    return;

  orientation.normalize();
  auto world = orientation.rotateVector(linear);

  float declination = IMU_DECLINATION_DEGREES * DEG_TO_RAD;

  telemetry.imuEastAcceleration =
      world.x() * cosf(declination) + world.y() * sinf(declination);
  telemetry.imuNorthAcceleration =
      world.y() * cosf(declination) - world.x() * sinf(declination);

  imuOrientationValid = true;

  if (telemetry.imuSystemCalibration == 0 ||
      telemetry.imuGyroCalibration != 3 || telemetry.imuAccelCalibration != 3 ||
      telemetry.imuMagCalibration != 3)
    return;

  telemetry.filterImuUsed = 1;

  if (fabsf(telemetry.imuEastAcceleration) >= IMU_ACCELERATION_DEADBAND)
    telemetry.filterEastAcceleration = telemetry.imuEastAcceleration;

  if (fabsf(telemetry.imuNorthAcceleration) >= IMU_ACCELERATION_DEADBAND)
    telemetry.filterNorthAcceleration = telemetry.imuNorthAcceleration;
}

const char *gpsField(unsigned index) {
  const char *field = gpsLine;

  while (index--) {
    field = strchr(field, ',');

    if (!field)
      return "";

    ++field;
  }

  return *field == ',' || *field == '*' ? "" : field;
}

void updateGpsPosition(double latitude, double longitude, double epoch,
                       uint32_t now) {
  lastGpsPositionMs = now;

  if (!haveGpsPosition || latitude != previousGpsLatitude ||
      longitude != previousGpsLongitude) {
    ++telemetry.gpsPositionChanges;
    previousGpsLatitude = latitude;
    previousGpsLongitude = longitude;
  }

  haveGpsPosition = true;

  if (!telemetry.gpsOriginReady) {
    telemetry.gpsOriginLatitude = latitude;
    telemetry.gpsOriginLongitude = longitude;
    telemetry.gpsOriginReady = telemetry.filterInitialized = 1;
    lastPredictionMs = now;
  }

  telemetry.gpsEast = (longitude - telemetry.gpsOriginLongitude) *
                      METERS_PER_DEGREE *
                      cos(telemetry.gpsOriginLatitude * DEG_TO_RAD);
  telemetry.gpsNorth =
      (latitude - telemetry.gpsOriginLatitude) * METERS_PER_DEGREE;

  if (epoch == lastGpsPositionEpoch)
    return;

  lastGpsPositionEpoch = epoch;
  ++telemetry.gpsPositionUpdates;

  predictFilter(now);
  eastFilter.correctPosition(telemetry.gpsEast, GPS_POSITION_VARIANCE);
  northFilter.correctPosition(telemetry.gpsNorth, GPS_POSITION_VARIANCE);
}

void processGpsSentence(bool connected) {
  uint32_t now = millis();

  GpsSentencePacket raw;
  raw.rxUptimeMs = now;
  memcpy(raw.rawGpsSentence, gpsLine, gpsLineLength + 1);

  sendPacket(Serial, PacketType::GpsSentence, raw);

  if (connected)
    sendPacket(bluetooth, PacketType::GpsSentence, raw);

  bool decoded = false;

  for (size_t i = 0; i < gpsLineLength; ++i) {
    decoded |= gps.encode(gpsLine[i]);
  }

  decoded |= gps.encode('\n');

  telemetry.gpsPassedChecksum = gps.passedChecksum();
  telemetry.gpsFailedChecksum = gps.failedChecksum();
  telemetry.gpsSentencesWithFix = gps.sentencesWithFix();

  if (!decoded || gpsLineLength < 7)
    return;

  bool rmc = strncmp(gpsLine + 3, "RMC,", 4) == 0;
  bool gga = strncmp(gpsLine + 3, "GGA,", 4) == 0;

  if ((!rmc && !gga) || gpsLine[1] != 'G' || !strchr("PNABL", gpsLine[2]))
    return;

  telemetry.rxGpsSampleMs = now;

  bool hasTime = *gpsField(1);
  double epoch = hasTime ? strtod(gpsField(1), nullptr) : -1;
  telemetry.rawGpsUtc = hasTime ? epoch : NAN;

  bool fix = rmc ? *gpsField(2) == 'A' : *gpsField(6) > '0';
  unsigned latitudeField = rmc ? 3 : 2;
  bool coordinatesPresent =
      *gpsField(latitudeField) && *gpsField(latitudeField + 1) &&
      *gpsField(latitudeField + 2) && *gpsField(latitudeField + 3);
  gpsPositionPresent =
      fix && hasTime && coordinatesPresent && gps.location.isUpdated();

  double latitude = gps.location.lat();
  double longitude = gps.location.lng();
  telemetry.rawGpsLatitude = gpsPositionPresent ? latitude : NAN;
  telemetry.rawGpsLongitude = gpsPositionPresent ? longitude : NAN;

  if (gpsPositionPresent)
    updateGpsPosition(latitude, longitude, epoch, now);

  if (gga) {
    telemetry.rawGpsFixQuality = *gpsField(6) ? *gpsField(6) - '0' : 0;

    gpsSatellitesPresent = *gpsField(7) && gps.satellites.isUpdated();
    gpsHdopPresent = *gpsField(8) && gps.hdop.isUpdated();
    gpsAltitudePresent =
        fix && *gpsField(9) && *gpsField(10) == 'M' && gps.altitude.isUpdated();

    telemetry.rawGpsSatellites =
        gpsSatellitesPresent ? gps.satellites.value() : 0;
    telemetry.rawGpsHdop = gpsHdopPresent ? gps.hdop.hdop() : NAN;
    telemetry.rawGpsAltitude = gpsAltitudePresent ? gps.altitude.meters() : NAN;

    lastGpsGgaMs = now;

    if (gpsAltitudePresent) {
      lastGpsAltitudeMs = now;
      haveGpsAltitude = true;
    }

    return;
  }

  telemetry.rawGpsDate = *gpsField(9) ? gps.date.value() : 0;
  telemetry.rawGpsMode = *gpsField(12);

  gpsSpeedPresent = fix && *gpsField(7) && gps.speed.isUpdated();
  gpsCoursePresent = fix && *gpsField(8) && gps.course.isUpdated();

  telemetry.rawGpsSpeed = gpsSpeedPresent ? gps.speed.mps() : NAN;
  telemetry.rawGpsCourse = gpsCoursePresent ? gps.course.deg() : NAN;

  if (gpsSpeedPresent) {
    haveGpsSpeed = true;
    lastGpsSpeedMs = now;
    ++telemetry.gpsSpeedUpdates;
  }

  if (gpsCoursePresent) {
    haveGpsCourse = true;
    lastGpsCourseMs = now;
  }

  if (!gpsPositionPresent || !gpsSpeedPresent || !gpsCoursePresent ||
      telemetry.rawGpsSpeed <= 0 || epoch == lastGpsVelocityEpoch)
    return;

  lastGpsVelocityEpoch = epoch;

  predictFilter(now);

  double course = telemetry.rawGpsCourse * DEG_TO_RAD;
  eastFilter.correctVelocity(telemetry.rawGpsSpeed * sin(course),
                             GPS_VELOCITY_VARIANCE);
  northFilter.correctVelocity(telemetry.rawGpsSpeed * cos(course),
                              GPS_VELOCITY_VARIANCE);
}

void readGps(bool connected) {
  while (gpsSerial.available()) {
    char value = gpsSerial.read();
    ++telemetry.gpsCharsProcessed;

    if (value == '$') {
      gpsLineLength = 0;
      gpsLineOverflow = false;
    }

    if (value == '\r')
      continue;

    if (value == '\n') {
      if (gpsLineLength && !gpsLineOverflow) {
        gpsLine[gpsLineLength] = '\0';
        processGpsSentence(connected);
      }

      gpsLineLength = 0;
      gpsLineOverflow = false;
    } else if (gpsLineLength || value == '$') {
      if (gpsLineLength < sizeof(gpsLine) - 1)
        gpsLine[gpsLineLength++] = value;
      else
        gpsLineOverflow = true;
    }
  }
}

void readControls() {
  ControlPacket next;

  while (controlReader.read(bluetooth, PacketType::Control, next)) {
    if (next.controlGpsResetId != control.controlGpsResetId)
      resetGpsOrigin();

    control = next;

    lastControlMs = millis();
    receivedControl = true;
    ++telemetry.rxControlPackets;
    telemetry.rxControlSequence = control.controlSequence;
  }
}

void updateOutputs(bool connected, uint32_t now) {
  float dt = min(uint32_t(now - lastOutputMs), uint32_t(100)) / 1000.0f;
  lastOutputMs = now;

  bool fresh = receivedControl && now - lastControlMs < CONTROL_TIMEOUT_MS;
  telemetry.outputActive = connected && fresh && !control.controlStopped;

  int center = constrain(control.controlServoCenter, 0, 180);
  int deflection =
      constrain(control.controlServoDeflection, 0, min(center, 180 - center));
  int minimum = center - deflection;
  int maximum = center + deflection;

  int limit = control.controlSpeedLimit == LIMIT_FREE
                  ? 255
                  : control.controlSpeedLimit;
  limit = constrain(limit, 0, 255);

  if (telemetry.outputActive) {
    float targetMotor = control.controlMotorPwm == MOTOR_FREE
                            ? joystickResponse(control.controlJ1Y) * limit
                            : control.controlMotorPwm;

    float targetServo =
        control.controlServoAngle == SERVO_FREE
            ? center + joystickResponse(control.controlJ2X) * deflection
            : control.controlServoAngle;

    targetMotor = constrain(targetMotor, -limit, limit);
    targetServo = constrain(targetServo, minimum, maximum);

    motorPwm = approach(motorPwm, targetMotor, MOTOR_SLEW_PER_SECOND * dt);
    motorPwm = constrain(motorPwm, -limit, limit);
    servoAngle = approach(servoAngle, targetServo, SERVO_SLEW_PER_SECOND * dt);
  } else {
    motorPwm = 0;
    servoAngle = center;
  }

  servoAngle = constrain(servoAngle, minimum, maximum);

  int motor = lroundf(motorPwm);

  if ((motor > 0 && telemetry.outputMotorPwm < 0) ||
      (motor < 0 && telemetry.outputMotorPwm > 0))
    analogWrite(ENB, 0);

  digitalWrite(RIGHT_IN3, motor < 0 ? HIGH : LOW);
  digitalWrite(RIGHT_IN4, motor > 0 ? HIGH : LOW);
  analogWrite(ENB, abs(motor));

  telemetry.outputMotorPwm = motor;
  telemetry.outputServoAngle = lroundf(servoAngle);
  telemetry.rxControlAgeMs = receivedControl ? now - lastControlMs : UINT32_MAX;

  servo.write(telemetry.outputServoAngle);
}

void reportTelemetry(bool connected, uint32_t now) {
  telemetry.rxUptimeMs = now;

  telemetry.gpsPositionAgeMs =
      haveGpsPosition ? now - lastGpsPositionMs : UINT32_MAX;
  telemetry.gpsSpeedAgeMs = haveGpsSpeed ? now - lastGpsSpeedMs : UINT32_MAX;
  telemetry.gpsCourseAgeMs = haveGpsCourse ? now - lastGpsCourseMs : UINT32_MAX;
  telemetry.gpsAltitudeAgeMs =
      haveGpsAltitude ? now - lastGpsAltitudeMs : UINT32_MAX;

  telemetry.gpsPositionValid =
      gpsPositionPresent && telemetry.gpsPositionAgeMs < GPS_MAX_AGE_MS;
  telemetry.gpsSpeedValid =
      gpsSpeedPresent && telemetry.gpsSpeedAgeMs < GPS_MAX_AGE_MS;
  telemetry.gpsCourseValid =
      gpsCoursePresent && telemetry.gpsCourseAgeMs < GPS_MAX_AGE_MS;
  telemetry.gpsAltitudeValid =
      gpsAltitudePresent && telemetry.gpsAltitudeAgeMs < GPS_MAX_AGE_MS;
  telemetry.gpsHdopValid =
      gpsHdopPresent && now - lastGpsGgaMs < GPS_MAX_AGE_MS;
  telemetry.gpsSatellitesValid =
      gpsSatellitesPresent && now - lastGpsGgaMs < GPS_MAX_AGE_MS;

  telemetry.filterEast = eastFilter.position;
  telemetry.filterNorth = northFilter.position;
  telemetry.filterEastVelocity = eastFilter.velocity;
  telemetry.filterNorthVelocity = northFilter.velocity;

  sendPacket(Serial, PacketType::RxTelemetry, telemetry);

  if (connected)
    sendPacket(bluetooth, PacketType::RxTelemetry, telemetry);
}

void connectTransmitter(void *) {
  for (;;) {
    if (!bluetooth.connected())
      bluetooth.connect(String(BT_TX_NAME));
    vTaskDelay(pdMS_TO_TICKS(CONNECT_RETRY_MS));
  }
}

void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(SERIAL_BAUD);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(ENA, LOW);

  servo.attach(SERVO_1);
  updateOutputs(false, millis());

  gpsSerial.setRxBufferSize(4096);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setTimeOut(20);

  telemetry.imuAvailable = bno.begin(OPERATION_MODE_NDOF);

  if (telemetry.imuAvailable)
    bno.setExtCrystalUse(true);

  bluetooth.begin(BT_RX_NAME, true);

  xTaskCreate(connectTransmitter, "bt connect", 4096, nullptr, 1, nullptr);
}

void loop() {
  bool connected = bluetooth.connected();
  digitalWrite(LED_BUILTIN, connected ? HIGH : LOW);

  if (connected) {
    readControls();
  } else {
    receivedControl = false;
    controlReader.reset();
  }

  updateOutputs(connected, millis());
  readGps(connected);

  uint32_t now = millis();

  if (now - lastImuMs >= CONTROL_INTERVAL_MS) {
    lastImuMs = now;
    sampleImu(now);
  }

  if (now - lastReportMs >= TELEMETRY_INTERVAL_MS) {
    lastReportMs = now;
    reportTelemetry(connected, now);
  }

  delay(1);
}
