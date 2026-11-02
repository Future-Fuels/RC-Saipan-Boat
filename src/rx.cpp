#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <TinyGPS++.h>
#include <Wire.h>

#include "shared.h"

// GPS and filter tuning. Position variances are m^2; velocity variance is (m/s)^2.
constexpr unsigned long GPS_BAUD = 9600UL;
constexpr unsigned long GPS_MAX_AGE_MS = 2000UL;
constexpr float METERS_PER_DEGREE = 111320.0f;
constexpr float MAX_BOAT_SPEED_MPS = 1.0f;
constexpr float MAX_BOAT_ACCELERATION_MPS2 = 0.25f;
constexpr float MAX_FILTER_SPEED_MPS = MAX_BOAT_SPEED_MPS * 1.5f;
constexpr float MOVING_POSITION_VARIANCE = 4.0f;
constexpr float STATIONARY_POSITION_VARIANCE = 16.0f;
constexpr float GPS_SPEED_VARIANCE = 0.20f * 0.20f;

// Stop detection and the deadband applied after IMU bias correction.
constexpr float STATIONARY_SPEED_MPS = 0.15f;
constexpr unsigned long STATIONARY_DWELL_MS = 2000UL;
constexpr float STATIONARY_ACCELERATION_MPS2 = 0.30f;
constexpr float STATIONARY_GYRO_DPS = 2.0f;
constexpr float IMU_ACCELERATION_DEADBAND_MPS2 = 0.10f;

// Linear Kalman filter: east/north position and velocity, driven by acceleration.
class PositionKalmanFilter {
public:
  void predict(float deltaSeconds, float eastAcceleration,
               float northAcceleration) {
    if (!initialized_ || stationary_ || deltaSeconds <= 0.0f ||
        deltaSeconds > 2.0f) {
      return;
    }

    // Predict position and velocity with constant acceleration over this step.
    state_[0] += state_[2] * deltaSeconds +
                 0.5f * eastAcceleration * deltaSeconds * deltaSeconds;
    state_[1] += state_[3] * deltaSeconds +
                 0.5f * northAcceleration * deltaSeconds * deltaSeconds;
    state_[2] += eastAcceleration * deltaSeconds;
    state_[3] += northAcceleration * deltaSeconds;

    float dt = deltaSeconds;
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt2 * dt2;
    float accelerationVariance =
        MAX_BOAT_ACCELERATION_MPS2 * MAX_BOAT_ACCELERATION_MPS2;

    // Propagate covariance with independent acceleration noise on each axis.
    covariance_[0][0] +=
        2.0f * dt * covariance_[0][2] + dt2 * covariance_[2][2] +
        accelerationVariance * dt4 * 0.25f;
    covariance_[0][1] +=
        dt * (covariance_[0][3] + covariance_[2][1]) + dt2 * covariance_[2][3];
    covariance_[1][0] = covariance_[0][1];
    covariance_[1][1] +=
        2.0f * dt * covariance_[1][3] + dt2 * covariance_[3][3] +
        accelerationVariance * dt4 * 0.25f;
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

    if (stationary_) {
      return;
    }
    updateScalar(0, eastMeters, measurementVariance);
    updateScalar(1, northMeters, measurementVariance);
  }

  void updateVelocity(float eastMetersPerSecond, float northMetersPerSecond) {
    if (!initialized_ || stationary_) {
      return;
    }

    updateScalar(2, eastMetersPerSecond, GPS_SPEED_VARIANCE);
    updateScalar(3, northMetersPerSecond, GPS_SPEED_VARIANCE);
  }

  void setStationary(bool stationary) {
    if (stationary_ == stationary) {
      return;
    }
    stationary_ = stationary;
    if (!stationary) {
      return;
    }

    state_[2] = state_[3] = 0.0f;
    // A confirmed stop constrains velocity without moving the position anchor.
    for (uint8_t i = 0; i < 4; ++i) {
      covariance_[i][2] = covariance_[2][i] = 0.0f;
      covariance_[i][3] = covariance_[3][i] = 0.0f;
    }
    covariance_[2][2] = covariance_[3][3] = GPS_SPEED_VARIANCE;
  }

  bool initialized() const { return initialized_; }
  bool stationary() const { return stationary_; }
  float east() const { return state_[0]; }
  float north() const { return state_[1]; }
  float eastVelocity() const { return state_[2]; }
  float northVelocity() const { return state_[3]; }

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

  bool initialized_ = false; // True after the first valid GPS position.
  bool stationary_ = false;
  // East position, north position, east velocity, north velocity.
  float state_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  // Initial uncertainty: 5 m position and 0.5 m/s velocity, with no correlation.
  float covariance_[4][4] = {
      {25.0f, 0.0f, 0.0f, 0.0f},
      {0.0f, 25.0f, 0.0f, 0.0f},
      {0.0f, 0.0f, 0.25f, 0.0f},
      {0.0f, 0.0f, 0.0f, 0.25f},
  };
};

ControlPacket control; // Latest commands received from the transmitter.
TelemetryPacket telemetry{};
BluetoothSerial bluetooth;
Servo servo;
Adafruit_BNO055 bno(55, IMU_ADDR, &Wire);
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
PositionKalmanFilter positionFilter;

// Loop timing and control input.
uint32_t lastPrediction = 0;
uint32_t lastControlTime = 0;
uint32_t lastOutputUpdate = 0;
uint32_t lastReport = 0;
uint32_t lastConnectAttempt = 0;
bool receivedControl = false;
String controlLine;
bool controlOverflow = false;

// Applied actuator commands and the telemetry output buffer.
float motorSpeed = 0;
float servoAngle = SERVO_DEFAULT_ANGLE;
char jsonLine[TELEMETRY_JSON_SIZE];

// Shared GPS origin, filter acceleration (m/s^2), and raw diagnostic positions (m).
double referenceLatitude = 0;
double referenceLongitude = 0;
float eastAcceleration = 0.0f;
float northAcceleration = 0.0f;
float rawGpsEast = 0.0f;
float rawGpsNorth = 0.0f;
float rawImuEast = 0.0f;
float rawImuNorth = 0.0f;
float rawImuEastVelocity = 0.0f;
float rawImuNorthVelocity = 0.0f;

// IMU bias learning, stop detection, and GPS sentence deduplication.
imu::Vector<3> accelerationBias;
bool accelerationBiasReady = false;
uint32_t biasSamples = 0;
bool stationaryCandidate = false;
uint32_t stationarySince = 0;
uint32_t lastPositionEpoch = UINT32_MAX;
uint32_t lastVelocityEpoch = UINT32_MAX;

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
  positionFilter = PositionKalmanFilter();
  eastAcceleration = 0.0f;
  northAcceleration = 0.0f;
  rawGpsEast = rawGpsNorth = 0.0f;
  rawImuEast = rawImuNorth = 0.0f;
  rawImuEastVelocity = rawImuNorthVelocity = 0.0f;
  accelerationBiasReady = false;
  biasSamples = 0;
  stationaryCandidate = false;
  lastPositionEpoch = lastVelocityEpoch = UINT32_MAX;
  lastPrediction = millis();
}

void accelerationToEastNorth(const imu::Vector<3> &linear,
                             const imu::Quaternion &orientation, float &east,
                             float &north) {
  float qw = orientation.w();
  float qx = orientation.x();
  float qy = orientation.y();
  float qz = orientation.z();
  float scale = 1.0f / sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
  qw *= scale;
  qx *= scale;
  qy *= scale;
  qz *= scale;
  float x = linear.x();
  float y = linear.y();
  float z = linear.z();

  east = (1.0f - 2.0f * (qy * qy + qz * qz)) * x +
         2.0f * (qx * qy - qw * qz) * y + 2.0f * (qx * qz + qw * qy) * z;
  north = 2.0f * (qx * qy + qw * qz) * x +
          (1.0f - 2.0f * (qx * qx + qz * qz)) * y +
          2.0f * (qy * qz - qw * qx) * z;
}

bool updateStationaryDetection() {
  // Detect rest from physical readings; compass calibration is not needed here.
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  bool quiet = motorSpeed == 0.0f && gps.location.isValid() &&
               gps.location.age() < GPS_MAX_AGE_MS && gps.speed.isValid() &&
               gps.speed.age() < GPS_MAX_AGE_MS &&
               gps.speed.mps() < STATIONARY_SPEED_MPS &&
               isfinite(accel.magnitude()) && isfinite(gyro.magnitude()) &&
               gyro.magnitude() < STATIONARY_GYRO_DPS &&
               fabs(accel.magnitude() - 9.80665) < STATIONARY_ACCELERATION_MPS2;
  if (quiet) {
    if (!stationaryCandidate) {
      stationaryCandidate = true;
      stationarySince = millis();
    }
  } else {
    stationaryCandidate = false;
    biasSamples = 0;
  }
  bool stopped =
      stationaryCandidate && millis() - stationarySince >= STATIONARY_DWELL_MS;
  positionFilter.setStationary(stopped);
  return quiet;
}

void updateRawImuEstimate(const imu::Vector<3> &linear,
                          const imu::Quaternion &orientation,
                          float deltaSeconds) {
  // Diagnostic only: starts at the first GPS fix with zero velocity, without
  // calibration gating, bias correction, GPS correction, or stationary hold.
  if (positionFilter.initialized() && deltaSeconds > 0.0f && deltaSeconds <= 2.0f) {
    float east, north;
    accelerationToEastNorth(linear, orientation, east, north);
    rawImuEast += rawImuEastVelocity * deltaSeconds +
                  0.5f * east * deltaSeconds * deltaSeconds;
    rawImuNorth += rawImuNorthVelocity * deltaSeconds +
                   0.5f * north * deltaSeconds * deltaSeconds;
    rawImuEastVelocity += east * deltaSeconds;
    rawImuNorthVelocity += north * deltaSeconds;
  }
}

void updateImuBias(const imu::Vector<3> &linear, bool quiet) {
  bool stopped = positionFilter.stationary();
  if (quiet && !accelerationBiasReady) {
    ++biasSamples;
    if (biasSamples == 1) {
      accelerationBias = linear;
    } else {
      accelerationBias =
          accelerationBias + (linear - accelerationBias) * (1.0 / biasSamples);
    }
    accelerationBiasReady =
        stopped && biasSamples >= STATIONARY_DWELL_MS / CONTROL_INTERVAL_MS;
  } else if (stopped) {
    accelerationBias = accelerationBias * 0.95 + linear * 0.05;
  }
}

void updateImuAcceleration(float deltaSeconds) {
  eastAcceleration = northAcceleration = 0.0f;
  if (!telemetry.imuAvailable) {
    accelerationBiasReady = false;
    biasSamples = 0;
    stationaryCandidate = false;
    positionFilter.setStationary(false);
    return;
  }

  bno.getCalibration(&telemetry.systemCalibration, &telemetry.gyroCalibration,
                     &telemetry.accelCalibration, &telemetry.magCalibration);
  bool quiet = updateStationaryDetection();
  imu::Vector<3> linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Quaternion orientation = bno.getQuat();
  float qw = orientation.w();
  float qx = orientation.x();
  float qy = orientation.y();
  float qz = orientation.z();
  float normSquared = qw * qw + qx * qx + qy * qy + qz * qz;
  if (!isfinite(linear.magnitude()) || !isfinite(normSquared) ||
      normSquared < 0.9f || normSquared > 1.1f) {
    accelerationBiasReady = false;
    biasSamples = 0;
    return;
  }

  updateRawImuEstimate(linear, orientation, deltaSeconds);

  // Calibration gates filter acceleration, not the raw diagnostic or stop hold.
  if (telemetry.systemCalibration == 0 || telemetry.gyroCalibration != 3 ||
      telemetry.accelCalibration != 3 || telemetry.magCalibration != 3) {
    accelerationBiasReady = false;
    biasSamples = 0;
    return;
  }
  updateImuBias(linear, quiet);

  // Use GPS only until a calibrated, confirmed stop provides a bias estimate.
  if (positionFilter.stationary() || !accelerationBiasReady) {
    return;
  }

  accelerationToEastNorth(linear - accelerationBias, orientation,
                          eastAcceleration, northAcceleration);
  // Suppress residual jitter in the filter input; keep raw diagnostics untouched.
  if (fabsf(eastAcceleration) < IMU_ACCELERATION_DEADBAND_MPS2) {
    eastAcceleration = 0.0f;
  }
  if (fabsf(northAcceleration) < IMU_ACCELERATION_DEADBAND_MPS2) {
    northAcceleration = 0.0f;
  }
}

void updateGpsFilter() {
  if (!gps.location.isUpdated() || !gps.location.isValid() ||
      !gps.time.isValid()) {
    return;
  }
  double latitude = gps.location.lat();  // Current GPS latitude.
  double longitude = gps.location.lng(); // Current GPS longitude.
  uint32_t epoch = gps.time.value();
  bool newSpeed = gps.speed.isUpdated();
  bool newCourse = gps.course.isUpdated();
  bool speedValid = gps.speed.isValid() && gps.speed.age() < GPS_MAX_AGE_MS;
  float speed = speedValid ? gps.speed.mps() : 0.0f; // GPS speed in m/s.
  speedValid = speedValid && speed <= MAX_FILTER_SPEED_MPS;
  if (!speedValid || speed >= STATIONARY_SPEED_MPS || motorSpeed != 0.0f) {
    stationaryCandidate = false;
    positionFilter.setStationary(false);
  }
  if (!positionFilter.initialized()) {
    referenceLatitude = latitude;
    referenceLongitude = longitude;
    lastPrediction = millis();
  }
  float east = (longitude - referenceLongitude) * METERS_PER_DEGREE *
               cos(referenceLatitude * DEG_TO_RAD);
  float north = (latitude - referenceLatitude) * METERS_PER_DEGREE;
  rawGpsEast = east;
  rawGpsNorth = north;
  bool gpsStationary = speedValid && speed < STATIONARY_SPEED_MPS;
  // RMC and GGA can report the same fix; velocity may arrive after position.
  if (epoch != lastPositionEpoch) {
    positionFilter.updatePosition(east, north,
                                  gpsStationary ? STATIONARY_POSITION_VARIANCE
                                                : MOVING_POSITION_VARIANCE);
    lastPositionEpoch = epoch;
  }
  if (!newSpeed || !speedValid || epoch == lastVelocityEpoch) {
    return;
  }
  lastVelocityEpoch = epoch;
  if (gpsStationary) {
    positionFilter.updateVelocity(0.0f, 0.0f);
  } else if (newCourse && gps.course.isValid() &&
             gps.course.age() < GPS_MAX_AGE_MS) {
    float course = gps.course.deg() * DEG_TO_RAD;
    positionFilter.updateVelocity(speed * sin(course), speed * cos(course));
  }
}

void readSensors() {
  uint32_t now = millis();
  if (now - lastPrediction >= CONTROL_INTERVAL_MS) {
    float deltaSeconds = (now - lastPrediction) / 1000.0f;
    updateImuAcceleration(deltaSeconds);
    positionFilter.predict(deltaSeconds, eastAcceleration, northAcceleration);
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

  telemetry.filterInitialized = positionFilter.initialized();
  telemetry.east = positionFilter.east();
  telemetry.north = positionFilter.north();
  telemetry.eastVelocity = positionFilter.eastVelocity();
  telemetry.northVelocity = positionFilter.northVelocity();
}

void acceptControl(const String &line) {
  ControlPacket next;
  int fields = sscanf(line.c_str(), "C %d %d %d %d %d %d %d %d %d %u %d %d",
                      &next.j1x, &next.j1y, &next.j2x, &next.j2y, &next.speed,
                      &next.servoLock, &next.motorLock, &next.speedLimit,
                      &next.stopped, &next.gpsResetId, &next.servoDefaultAngle,
                      &next.servoMaxDeflection);

  if (fields != 12) {
    return;
  }

  bool newRequest = !receivedControl || next.servoLock != control.servoLock ||
                    next.motorLock != control.motorLock ||
                    next.speedLimit != control.speedLimit ||
                    next.stopped != control.stopped ||
                    next.gpsResetId != control.gpsResetId ||
                    next.servoDefaultAngle != control.servoDefaultAngle ||
                    next.servoMaxDeflection != control.servoMaxDeflection;

  if (next.gpsResetId != control.gpsResetId) {
    resetGpsFilter();
  }

  control = next;
  lastControlTime = millis();
  receivedControl = true;
  ++telemetry.controlPackets;

  if (newRequest) {
    Serial.printf(
        "control: servo=%d motor=%d limit=%d stopped=%d gpsReset=%u "
        "center=%d deflection=%d\n",
        control.servoLock, control.motorLock, control.speedLimit,
        control.stopped, control.gpsResetId, control.servoDefaultAngle,
        control.servoMaxDeflection);
  }
}

void readControls() {
  while (bluetooth.available() > 0) {
    char c = bluetooth.read();

    if (c == '\n') {
      if (!controlOverflow) {
        acceptControl(controlLine);
      }

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
  // The monitor validates commands; keep physical limits at the outputs.
  int center = constrain(control.servoDefaultAngle, 0, 180);
  int deflection = constrain(control.servoMaxDeflection, 0, min(center, 180 - center));
  float servoMinimum = center - deflection;
  float servoMaximum = center + deflection;

  if (telemetry.active) {
    int limit =
        control.speedLimit == LIMIT_FREE ? control.speed : control.speedLimit;
    limit = constrain(limit, 0, 255);

    float motorTarget = joystickResponse(control.j1y) * limit;
    float servoTarget = center + joystickResponse(control.j2x) * deflection;

    if (control.motorLock != MOTOR_FREE) {
      motorTarget = constrain(control.motorLock, -limit, limit);
    }

    if (control.servoLock != SERVO_FREE) {
      servoTarget = control.servoLock;
    }
    servoTarget = constrain(servoTarget, servoMinimum, servoMaximum);

    motorSpeed =
        approach(motorSpeed, motorTarget, MOTOR_SLEW_PER_SECOND * seconds);
    motorSpeed = constrain(motorSpeed, -limit, limit);
    servoAngle =
        approach(servoAngle, servoTarget, SERVO_SLEW_PER_SECOND * seconds);
  } else {
    motorSpeed = 0;
    servoAngle = center;
  }
  servoAngle = constrain(servoAngle, servoMinimum, servoMaximum);

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
      "{\"rxUptimeMs\":%lu,\"accX\":%.3f,\"accY\":%.3f,\"accZ\":%.3f,\"resG\":%.3f,"
      "\"gyroX\":%.3f,\"gyroY\":%.3f,\"gyroZ\":%.3f,"
      "\"linearAccX\":%.3f,\"linearAccY\":%.3f,\"linearAccZ\":%.3f,"
      "\"quatW\":%.4f,\"quatX\":%.4f,\"quatY\":%.4f,\"quatZ\":%.4f,"
      "\"imuAvailable\":%s,\"systemCalibration\":%u,\"gyroCalibration\":%u,"
      "\"accelCalibration\":%u,\"magCalibration\":%u,"
      "\"gpsFix\":%s,\"gpsAgeMs\":%lu,\"satellites\":%lu,"
      "\"gpsCharsProcessed\":%lu,\"gpsPassedChecksum\":%lu,"
      "\"gpsFailedChecksum\":%lu,\"gpsSentencesWithFix\":%lu,"
      "\"latitude\":%.6f,\"longitude\":%.6f,\"altitude\":%.2f,"
      "\"gpsSpeed\":%.3f,\"gpsCourse\":%.2f,"
      "\"gpsSpeedValid\":%s,\"gpsCourseValid\":%s,\"gpsAltitudeValid\":%s,"
      "\"filterInitialized\":%s,\"filterStationary\":%s,\"east\":%.2f,"
      "\"north\":%.2f,"
      "\"gpsOriginSamples\":%lu,\"gpsOriginElapsedMs\":%lu,"
      "\"eastVelocity\":%.3f,\"northVelocity\":%.3f,"
      "\"rawGpsEast\":%.2f,\"rawGpsNorth\":%.2f,"
      "\"rawImuEast\":%.2f,\"rawImuNorth\":%.2f,"
      "\"eastAcceleration\":%.3f,\"northAcceleration\":%.3f,\"imuFusionReady\":"
      "%s,"
      "\"active\":%s,\"motorSpeed\":%d,\"servoAngle\":%d,"
      "\"servoDefaultAngle\":%d,\"servoMaxDeflection\":%d,"
      "\"controlPackets\":%lu,\"controlAgeMs\":%lu,"
      "\"servoLocked\":%s,\"motorLocked\":%s,\"speedLimit\":%d,\"stopped\":%s,"
      "\"gpsResetId\":%u}\n",
      millis(), telemetry.accX, telemetry.accY, telemetry.accZ, telemetry.resG,
      telemetry.gyroX, telemetry.gyroY, telemetry.gyroZ, telemetry.linearAccX,
      telemetry.linearAccY, telemetry.linearAccZ, telemetry.quatW,
      telemetry.quatX, telemetry.quatY, telemetry.quatZ,
      jsonBool(telemetry.imuAvailable), telemetry.systemCalibration,
      telemetry.gyroCalibration, telemetry.accelCalibration,
      telemetry.magCalibration, jsonBool(telemetry.gpsFix), telemetry.gpsAgeMs,
      telemetry.satellites, gps.charsProcessed(), gps.passedChecksum(),
      gps.failedChecksum(), gps.sentencesWithFix(), telemetry.latitude,
      telemetry.longitude, telemetry.altitude, telemetry.gpsSpeed, telemetry.gpsCourse,
      jsonBool(telemetry.gpsSpeedValid), jsonBool(telemetry.gpsCourseValid),
      jsonBool(telemetry.gpsAltitudeValid),
      jsonBool(telemetry.filterInitialized), jsonBool(positionFilter.stationary()),
      telemetry.east, telemetry.north,
      // Keep origin telemetry: one fix, with no averaging delay.
      positionFilter.initialized() ? 1UL : 0UL, 0UL, telemetry.eastVelocity,
      telemetry.northVelocity, rawGpsEast, rawGpsNorth, rawImuEast, rawImuNorth,
      eastAcceleration, northAcceleration, jsonBool(accelerationBiasReady),
      jsonBool(telemetry.active), telemetry.motorSpeed, telemetry.servoAngle,
      control.servoDefaultAngle, control.servoMaxDeflection,
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
