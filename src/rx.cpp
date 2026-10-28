#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <TinyGPS++.h>
#include <Wire.h>

#include "shared.h"

#define METERS_PER_DEGREE 111320.0f
#define MAX_BOAT_SPEED_MPS 1.0f
#define MAX_BOAT_ACCELERATION_MPS2 0.25f
#define STATIONARY_SPEED_MPS 0.15f
#define MOVING_POSITION_VARIANCE 16.0f
#define STATIONARY_POSITION_VARIANCE 100.0f
#define GPS_SPEED_VARIANCE (0.20f * 0.20f)
#define MAX_FILTER_SPEED_MPS (MAX_BOAT_SPEED_MPS * 1.5f)

class GpsEkf {
public:
  void predict(float deltaSeconds) {
    if (!initialized_ || deltaSeconds <= 0.0f || deltaSeconds > 2.0f) {
      return;
    }

    // update the predicted state based on the current velocity and time elapsed
    state_[0] += state_[2] * deltaSeconds;
    state_[1] += state_[3] * deltaSeconds;

    float dt = deltaSeconds;
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt2 * dt2;
    // Keep process noise appropriate for a small, slow boat.
    float accelerationVariance =
        MAX_BOAT_ACCELERATION_MPS2 * MAX_BOAT_ACCELERATION_MPS2;

    // Update the covariance matrix based on the state transition model and
    // process noise works independently for east and north
    covariance_[0][0] +=
        2.0f * dt * covariance_[0][2] + dt2 * covariance_[2][2] +
        accelerationVariance * dt4 * 0.25f; // estimated position variance
    covariance_[0][1] +=
        dt * (covariance_[0][3] + covariance_[2][1]) + dt2 * covariance_[2][3];
    covariance_[1][0] = covariance_[0][1];
    covariance_[1][1] +=
        2.0f * dt * covariance_[1][3] + dt2 * covariance_[3][3] +
        accelerationVariance * dt4 * 0.25f; // estimated position variance
    covariance_[0][2] +=
        dt * covariance_[2][2] + accelerationVariance * dt3 * 0.5f;
    covariance_[2][0] = covariance_[0][2];
    covariance_[0][3] += dt * covariance_[2][3];
    covariance_[3][0] = covariance_[0][3];
    covariance_[1][2] += dt * covariance_[2][3];
    covariance_[2][1] = covariance_[1][2];
    covariance_[1][3] +=
        dt * covariance_[3][3] + accelerationVariance * dt3 * 0.5f;
    covariance_[3][1] = covariance_[1][3];
    covariance_[2][2] += accelerationVariance * dt2;
    covariance_[3][3] += accelerationVariance * dt2;
  }

  void updatePosition(float eastMeters, float northMeters,
                      float measurementVariance = MOVING_POSITION_VARIANCE) {
    if (!initialized_) {
      state_[0] = eastMeters;
      state_[1] = northMeters;
      initialized_ = true;
      return;
    }

    updateScalar(0, eastMeters, measurementVariance);
    updateScalar(1, northMeters, measurementVariance);
  }

  void updateVelocity(float eastMetersPerSecond, float northMetersPerSecond) {
    if (!initialized_) {
      return;
    }

    updateScalar(2, eastMetersPerSecond, GPS_SPEED_VARIANCE);
    updateScalar(3, northMetersPerSecond, GPS_SPEED_VARIANCE);
  }

  bool initialized() { return initialized_; }
  float east() { return state_[0]; }
  float north() { return state_[1]; }
  float eastVelocity() { return state_[2]; }
  float northVelocity() { return state_[3]; }
  bool isStationary(float speedMetersPerSecond) {
    return speedMetersPerSecond < STATIONARY_SPEED_MPS;
  }

private:
  void updateScalar(uint8_t stateIndex, float measurement,
                    float measurementVariance) {
    float innovation = measurement - state_[stateIndex];
    float innovationVariance =
        covariance_[stateIndex][stateIndex] + measurementVariance;
    float gain[4];

    for (uint8_t row = 0; row < 4; row++) {
      gain[row] = covariance_[row][stateIndex] / innovationVariance;
      state_[row] += gain[row] * innovation;
    }

    float updatedCovariance[4][4];
    for (uint8_t row = 0; row < 4; row++) {
      for (uint8_t column = 0; column < 4; column++) {
        updatedCovariance[row][column] =
            covariance_[row][column] -
            gain[row] * covariance_[stateIndex][column];
      }
    }

    for (uint8_t row = 0; row < 4; row++) {
      for (uint8_t column = 0; column < 4; column++) {
        covariance_[row][column] = updatedCovariance[row][column];
      }
    }
  }

  bool initialized_ = false;
  // state: east position, north position, east velocity, north velocity
  float state_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  /*
  Initial covariance matrix.
  The diagonal elements represent the variance of each state variable.
  estimating initial error of 5 meters for position and 0.5 m/s for velocity.
  The off-diagonal elements are set to zero, indicating no initial correlation.
  */
  float covariance_[4][4] = {
      {25.0f, 0.0f, 0.0f, 0.0f},
      {0.0f, 25.0f, 0.0f, 0.0f},
      {0.0f, 0.0f, 0.25f, 0.0f},
      {0.0f, 0.0f, 0.0f, 0.25f},
  };
};

ControlPacket control;
TelemetryPacket telemetry{};
BluetoothSerial bluetooth;
Servo servo;
Adafruit_BNO055 bno(55, IMU_ADDR, &Wire);
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
GpsEkf ekf;

#define GPS_BAUD 9600UL
#define GPS_MAX_AGE_MS 2000UL

double referenceLatitude = 0;
double referenceLongitude = 0;
uint32_t lastPrediction = 0;
uint32_t lastControlTime = 0;
uint32_t lastOutputUpdate = 0;
uint32_t lastReport = 0;
uint32_t lastConnectAttempt = 0;
bool receivedControl = false;

String controlLine;
bool controlOverflow = false;
float motorSpeed = 0;
float servoAngle = SERVO_DEFAULT_ANGLE;
char jsonLine[TELEMETRY_JSON_SIZE];

const char *jsonBool(bool value) { return value ? "true" : "false"; }

float joystickResponse(int value) {
  int magnitude = abs(value);

  if (magnitude <= JOYSTICK_DEADBAND) {
    return 0;
  }

  float x = (magnitude - JOYSTICK_DEADBAND) / (255.0f - JOYSTICK_DEADBAND);
  float curved = (1 - JOYSTICK_EXPO) * x + JOYSTICK_EXPO * x * x * x;
  return value < 0 ? -curved : curved;
}

float approach(float current, float target, float step) {
  return current + constrain(target - current, -step, step);
}

void setupSensors() {
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setTimeOut(20);
  telemetry.imuAvailable = bno.begin(OPERATION_MODE_NDOF);
  if (telemetry.imuAvailable) {
    bno.setExtCrystalUse(true);
    Serial.println("BNO055 ready");
  } else {
    Serial.println("BNO055 not found, using GPS only");
  }
  Serial.println("GPS ready");
  lastPrediction = millis();
}

void resetGpsFilter() {
  ekf = GpsEkf();
  lastPrediction = millis();
}

void updateGpsFilter() {
  if (!gps.location.isUpdated() || !gps.location.isValid()) {
    return;
  }
  double latitude = gps.location.lat();
  double longitude = gps.location.lng();
  bool speedValid = gps.speed.isValid() && gps.speed.age() < GPS_MAX_AGE_MS;
  float speed = speedValid ? gps.speed.mps() : 0.0f;
  if (speedValid && speed > MAX_FILTER_SPEED_MPS) {
    return;
  }
  if (!ekf.initialized()) {
    referenceLatitude = latitude;
    referenceLongitude = longitude;
  }
  float east = (longitude - referenceLongitude) * METERS_PER_DEGREE *
               cos(referenceLatitude * DEG_TO_RAD);
  float north = (latitude - referenceLatitude) * METERS_PER_DEGREE;
  bool stationary = speedValid && ekf.isStationary(speed);
  ekf.updatePosition(east, north,
                     stationary ? STATIONARY_POSITION_VARIANCE
                                : MOVING_POSITION_VARIANCE);
  if (stationary) {
    ekf.updateVelocity(0.0f, 0.0f);
  } else if (speedValid && gps.course.isValid() &&
             gps.course.age() < GPS_MAX_AGE_MS) {
    float course = gps.course.deg() * DEG_TO_RAD;
    ekf.updateVelocity(speed * sin(course), speed * cos(course));
  }
}

void readSensors() {
  uint32_t now = millis();
  if (now - lastPrediction >= CONTROL_INTERVAL_MS) {
    ekf.predict((now - lastPrediction) / 1000.0f);
    lastPrediction = now;
  }
  while (gpsSerial.available() > 0) {
    // Consume each completed sentence before another can overwrite its fix.
    if (gps.encode(gpsSerial.read())) {
      updateGpsFilter();
    }
  }
}

void sampleSensors() {
  if (telemetry.imuAvailable) {
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Quaternion orientation = bno.getQuat();

    telemetry.accX = accel.x() / 9.80665f;
    telemetry.accY = accel.y() / 9.80665f;
    telemetry.accZ = accel.z() / 9.80665f;
    telemetry.resG = sqrtf(telemetry.accX * telemetry.accX +
                           telemetry.accY * telemetry.accY +
                           telemetry.accZ * telemetry.accZ);
    telemetry.gyroX = gyro.x();
    telemetry.gyroY = gyro.y();
    telemetry.gyroZ = gyro.z();
    telemetry.linearAccX = linear.x();
    telemetry.linearAccY = linear.y();
    telemetry.linearAccZ = linear.z();
    telemetry.quatW = orientation.w();
    telemetry.quatX = orientation.x();
    telemetry.quatY = orientation.y();
    telemetry.quatZ = orientation.z();

    bno.getCalibration(&telemetry.systemCalibration, &telemetry.gyroCalibration,
                       &telemetry.accelCalibration, &telemetry.magCalibration);
  }

  telemetry.gpsAgeMs = gps.location.age();
  telemetry.gpsFix =
      gps.location.isValid() && telemetry.gpsAgeMs < GPS_MAX_AGE_MS;
  telemetry.gpsSpeedValid =
      gps.speed.isValid() && gps.speed.age() < GPS_MAX_AGE_MS;
  telemetry.gpsCourseValid =
      gps.course.isValid() && gps.course.age() < GPS_MAX_AGE_MS;
  telemetry.gpsAltitudeValid =
      gps.altitude.isValid() && gps.altitude.age() < GPS_MAX_AGE_MS;

  telemetry.latitude = gps.location.isValid() ? gps.location.lat() : 0.0;
  telemetry.longitude = gps.location.isValid() ? gps.location.lng() : 0.0;
  telemetry.altitude =
      telemetry.gpsAltitudeValid ? gps.altitude.meters() : 0.0f;
  telemetry.gpsSpeed = telemetry.gpsSpeedValid ? gps.speed.mps() : 0.0f;
  telemetry.gpsCourse = telemetry.gpsCourseValid ? gps.course.deg() : 0.0f;
  telemetry.satellites =
      gps.satellites.isValid() && gps.satellites.age() < GPS_MAX_AGE_MS
          ? gps.satellites.value()
          : 0;

  telemetry.filterInitialized = ekf.initialized();
  telemetry.east = ekf.east();
  telemetry.north = ekf.north();
  telemetry.eastVelocity = ekf.eastVelocity();
  telemetry.northVelocity = ekf.northVelocity();
}

void acceptControl(String &line) {
  ControlPacket next;
  char extra;
  int fields = sscanf(line.c_str(), "C %d %d %d %d %d %d %d %d %d %u %c",
                      &next.j1x, &next.j1y, &next.j2x, &next.j2y, &next.speed,
                      &next.servoLock, &next.motorLock, &next.speedLimit,
                      &next.stopped, &next.gpsResetId, &extra);

  if (fields != 10)
    return;

  bool newRequest = !receivedControl || next.servoLock != control.servoLock ||
                    next.motorLock != control.motorLock ||
                    next.speedLimit != control.speedLimit ||
                    next.stopped != control.stopped ||
                    next.gpsResetId != control.gpsResetId;

  if (next.gpsResetId != control.gpsResetId) {
    resetGpsFilter();
  }

  control = next;
  lastControlTime = millis();
  receivedControl = true;
  ++telemetry.controlPackets;

  if (newRequest) {
    Serial.printf(
        "control: servo=%d motor=%d limit=%d stopped=%d gpsReset=%u\n",
        control.servoLock, control.motorLock, control.speedLimit,
        control.stopped, control.gpsResetId);
  }
}

void readControls() {
  while (bluetooth.available() > 0) {
    char c = bluetooth.read();

    if (c == '\n') {
      if (!controlOverflow)
        acceptControl(controlLine);

      controlLine = "";
      controlOverflow = false;
    } else if (c != '\r') {
      if (controlLine.length() < CONTROL_LINE_SIZE - 1) {
        controlLine += c;
      } else {
        controlOverflow = true;
      }
    }
  }
}

void updateOutputs(bool connected) {
  uint32_t now = millis();
  unsigned long elapsed = now - lastOutputUpdate;

  if (elapsed > 100) {
    elapsed = 100;
  }

  float seconds = elapsed / 1000.0f;
  lastOutputUpdate = now;

  bool fresh = receivedControl && now - lastControlTime < CONTROL_TIMEOUT_MS;
  telemetry.active = connected && fresh && !control.stopped;

  if (telemetry.active) {
    int limit =
        control.speedLimit == LIMIT_FREE ? control.speed : control.speedLimit;

    float motorTarget = joystickResponse(control.j1y) * limit;
    float servoTarget =
        SERVO_DEFAULT_ANGLE + joystickResponse(control.j2x) * 90;

    if (control.motorLock != MOTOR_FREE)
      motorTarget = constrain(control.motorLock, -limit, limit);

    if (control.servoLock != SERVO_FREE)
      servoTarget = control.servoLock;

    motorSpeed =
        approach(motorSpeed, motorTarget, MOTOR_SLEW_PER_SECOND * seconds);
    motorSpeed = constrain(motorSpeed, -limit, limit);
    servoAngle =
        approach(servoAngle, servoTarget, SERVO_SLEW_PER_SECOND * seconds);
  } else {
    motorSpeed = 0;
    servoAngle = SERVO_DEFAULT_ANGLE;
  }

  telemetry.motorSpeed = lroundf(motorSpeed);
  telemetry.servoAngle = lroundf(servoAngle);
  telemetry.controlAgeMs = receivedControl ? now - lastControlTime : UINT32_MAX;

  static int previousSpeed = 0;
  int speed = telemetry.motorSpeed;

  if ((speed > 0 && previousSpeed < 0) || (speed < 0 && previousSpeed > 0)) {
    analogWrite(ENB, 0);
  }

  digitalWrite(RIGHT_IN3, speed < 0 ? HIGH : LOW);
  digitalWrite(RIGHT_IN4, speed > 0 ? HIGH : LOW);
  analogWrite(ENB, abs(speed));
  servo.write(telemetry.servoAngle);
  previousSpeed = speed;
}

void sendTelemetry() {
  int length = snprintf(
      jsonLine, sizeof(jsonLine),
      "{\"accX\":%.3f,\"accY\":%.3f,\"accZ\":%.3f,\"resG\":%.3f,"
      "\"gyroX\":%.3f,\"gyroY\":%.3f,\"gyroZ\":%.3f,"
      "\"linearAccX\":%.3f,\"linearAccY\":%.3f,\"linearAccZ\":%.3f,"
      "\"quatW\":%.4f,\"quatX\":%.4f,\"quatY\":%.4f,\"quatZ\":%.4f,"
      "\"imuAvailable\":%s,\"systemCalibration\":%u,\"gyroCalibration\":%u,"
      "\"accelCalibration\":%u,\"magCalibration\":%u,"
      "\"gpsFix\":%s,\"gpsAgeMs\":%lu,\"satellites\":%lu,"
      "\"latitude\":%.6f,\"longitude\":%.6f,\"altitude\":%.2f,"
      "\"gpsSpeed\":%.3f,\"gpsCourse\":%.2f,"
      "\"gpsSpeedValid\":%s,\"gpsCourseValid\":%s,\"gpsAltitudeValid\":%s,"
      "\"filterInitialized\":%s,\"east\":%.2f,\"north\":%.2f,"
      "\"eastVelocity\":%.3f,\"northVelocity\":%.3f,"
      "\"active\":%s,\"motorSpeed\":%d,\"servoAngle\":%d,"
      "\"controlPackets\":%lu,\"controlAgeMs\":%lu,"
      "\"servoLocked\":%s,\"motorLocked\":%s,\"speedLimit\":%d,\"stopped\":%s,"
      "\"gpsResetId\":%u}\n",
      telemetry.accX, telemetry.accY, telemetry.accZ, telemetry.resG,
      telemetry.gyroX, telemetry.gyroY, telemetry.gyroZ, telemetry.linearAccX,
      telemetry.linearAccY, telemetry.linearAccZ, telemetry.quatW,
      telemetry.quatX, telemetry.quatY, telemetry.quatZ,
      jsonBool(telemetry.imuAvailable), telemetry.systemCalibration,
      telemetry.gyroCalibration, telemetry.accelCalibration,
      telemetry.magCalibration, jsonBool(telemetry.gpsFix), telemetry.gpsAgeMs,
      telemetry.satellites, telemetry.latitude, telemetry.longitude,
      telemetry.altitude, telemetry.gpsSpeed, telemetry.gpsCourse,
      jsonBool(telemetry.gpsSpeedValid), jsonBool(telemetry.gpsCourseValid),
      jsonBool(telemetry.gpsAltitudeValid),
      jsonBool(telemetry.filterInitialized), telemetry.east, telemetry.north,
      telemetry.eastVelocity, telemetry.northVelocity,
      jsonBool(telemetry.active), telemetry.motorSpeed, telemetry.servoAngle,
      telemetry.controlPackets, telemetry.controlAgeMs,
      jsonBool(control.servoLock != SERVO_FREE),
      jsonBool(control.motorLock != MOTOR_FREE), control.speedLimit,
      jsonBool(control.stopped), control.gpsResetId);

  if (length <= 0 || length >= TELEMETRY_JSON_SIZE) {
    return;
  }

  bluetooth.print(jsonLine);
}

void setup() {
  Serial.begin(115200);
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
  updateOutputs(false);
  controlLine.reserve(CONTROL_LINE_SIZE);
  setupSensors();
  bluetooth.begin(BT_RX_NAME, true);
  Serial.println("rx ready");
  lastConnectAttempt = millis() - CONNECT_RETRY_MS;
}

void loop() {
  bool connected = bluetooth.connected();
  digitalWrite(LED_BUILTIN, connected ? HIGH : LOW);

  if (connected) {
    readControls();
  } else {
    receivedControl = false;
    controlLine = "";
    controlOverflow = false;
  }

  updateOutputs(connected);
  readSensors();

  if (millis() - lastReport >= TELEMETRY_INTERVAL_MS) {
    lastReport = millis();

    sampleSensors();
    updateOutputs(connected);
    if (connected) {
      sendTelemetry();
    }
  }

  if (!connected && millis() - lastConnectAttempt >= CONNECT_RETRY_MS) {
    bluetooth.connect(String(BT_TX_NAME));

    lastConnectAttempt = millis();
  }

  delay(1);
}
