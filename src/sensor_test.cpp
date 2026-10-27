#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include "shared.h"

constexpr uint32_t GPS_BAUD = 9600;
constexpr uint32_t GPS_REPORT_INTERVAL_MS = 1000;
constexpr float METERS_PER_DEGREE = 111320.0f;
constexpr float MAX_BOAT_SPEED_MPS = 1.0f;
constexpr float MAX_BOAT_ACCELERATION_MPS2 = 0.25f;
constexpr float STATIONARY_SPEED_MPS = 0.15f;
constexpr float MOVING_POSITION_VARIANCE = 16.0f;
constexpr float STATIONARY_POSITION_VARIANCE = 100.0f;
constexpr float GPS_SPEED_VARIANCE = 0.20f * 0.20f;
constexpr float MAX_FILTER_SPEED_MPS = MAX_BOAT_SPEED_MPS * 1.5f;

Adafruit_BNO055 bno(55, IMU_ADDR, &Wire);
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
bool imuAvailable = false;
uint32_t lastGpsReport = 0;

class GpsEkf {
 public:
  void predict(float deltaSeconds) {
    if (!initialized_ || deltaSeconds <= 0.0f || deltaSeconds > 2.0f) {
      return;
    }

    // update the predicted state based on the current velocity and time elapsed
    state_[0] += state_[2] * deltaSeconds;
    state_[1] += state_[3] * deltaSeconds;

    const float dt = deltaSeconds;
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;
    // Keep process noise appropriate for a small, slow boat.
    const float accelerationVariance =
      MAX_BOAT_ACCELERATION_MPS2 * MAX_BOAT_ACCELERATION_MPS2;

    // Update the covariance matrix based on the state transition model and process noise
    // works independently for east and north
    covariance_[0][0] += 2.0f * dt * covariance_[0][2] +
                         dt2 * covariance_[2][2] +
                         accelerationVariance * dt4 * 0.25f; // estimated position variance
    covariance_[0][1] += dt * (covariance_[0][3] + covariance_[2][1]) +
                         dt2 * covariance_[2][3];
    covariance_[1][0] = covariance_[0][1];
    covariance_[1][1] += 2.0f * dt * covariance_[1][3] +
                         dt2 * covariance_[3][3] +
                         accelerationVariance * dt4 * 0.25f; // estimated position variance
    covariance_[0][2] += dt * covariance_[2][2] +
                         accelerationVariance * dt3 * 0.5f;
    covariance_[2][0] = covariance_[0][2];
    covariance_[0][3] += dt * covariance_[2][3];
    covariance_[3][0] = covariance_[0][3];
    covariance_[1][2] += dt * covariance_[2][3];
    covariance_[2][1] = covariance_[1][2];
    covariance_[1][3] += dt * covariance_[3][3] +
                         accelerationVariance * dt3 * 0.5f;
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

  // Ready for the BNO055 later: acceleration is expressed in local east/north.
  void updateAcceleration(float eastMetersPerSecondSquared,
                          float northMetersPerSecondSquared,
                          float deltaSeconds) {
    if (!initialized_ || deltaSeconds <= 0.0f) {
      return;
    }

    state_[0] += 0.5f * eastMetersPerSecondSquared * deltaSeconds * deltaSeconds;
    state_[1] +=
        0.5f * northMetersPerSecondSquared * deltaSeconds * deltaSeconds;
    state_[2] += eastMetersPerSecondSquared * deltaSeconds;
    state_[3] += northMetersPerSecondSquared * deltaSeconds;
  }

  bool initialized() const { return initialized_; }
  float east() const { return state_[0]; }
  float north() const { return state_[1]; }
  float eastVelocity() const { return state_[2]; }
  float northVelocity() const { return state_[3]; }
  bool isStationary(float speedMetersPerSecond) const {
    return speedMetersPerSecond < STATIONARY_SPEED_MPS;
  }

 private:
  void updateScalar(uint8_t stateIndex, float measurement,
                    float measurementVariance) {
    float innovation = measurement - state_[stateIndex];
    float innovationVariance = covariance_[stateIndex][stateIndex] +
                               measurementVariance;
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

GpsEkf ekf;
double referenceLatitude = 0.0;
double referenceLongitude = 0.0;
bool referenceSet = false;
uint32_t lastFilterUpdate = 0;

void gpsToLocalMeters(double latitude, double longitude, float& eastMeters,
                     float& northMeters) {
  const double longitudeScale =
      METERS_PER_DEGREE * cos(referenceLatitude * DEG_TO_RAD);
  eastMeters = static_cast<float>((longitude - referenceLongitude) *
                                  longitudeScale);
  northMeters = static_cast<float>((latitude - referenceLatitude) *
                                   METERS_PER_DEGREE);
}

void updateFilterFromGps() {
  uint32_t now = millis();
  float deltaSeconds = (now - lastFilterUpdate) / 1000.0f;
  lastFilterUpdate = now;
  ekf.predict(deltaSeconds);

  if (!gps.location.isUpdated() || !gps.location.isValid()) {
    return;
  }

  if (!referenceSet) {
    referenceLatitude = gps.location.lat();
    referenceLongitude = gps.location.lng();
    referenceSet = true;
  }

  float eastMeters;
  float northMeters;
  gpsToLocalMeters(gps.location.lat(), gps.location.lng(), eastMeters,
                   northMeters);

  if (gps.speed.isValid()) {
    float speed = gps.speed.mps();
    if (speed > MAX_FILTER_SPEED_MPS) {
      return;
    }

    bool stationary = ekf.isStationary(speed);
    ekf.updatePosition(eastMeters, northMeters,
                       stationary ? STATIONARY_POSITION_VARIANCE
                                  : MOVING_POSITION_VARIANCE);

    if (stationary) {
      // Course is undefined at rest; do not turn GPS noise into motion.
      ekf.updateVelocity(0.0f, 0.0f);
      return;
    }

    if (!gps.course.isValid()) {
      return;
    }

    float courseRadians = gps.course.deg() * DEG_TO_RAD;
    ekf.updateVelocity(speed * sin(courseRadians), speed * cos(courseRadians));
    return;
  }

  ekf.updatePosition(eastMeters, northMeters);
}

void printImuData() {
  if (!imuAvailable) {
    return;
  }

  imu::Vector<3> linearAccel =
      bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Quaternion orientation = bno.getQuat();

  uint8_t systemStatus;
  uint8_t selfTestResult;
  uint8_t systemError;
  bno.getSystemStatus(&systemStatus, &selfTestResult, &systemError);

  uint8_t systemCalibration;
  uint8_t gyroCalibration;
  uint8_t accelCalibration;
  uint8_t magCalibration;
  bno.getCalibration(&systemCalibration, &gyroCalibration, &accelCalibration,
                     &magCalibration);

  Serial.printf(
      "imu: accel: x=%+.3f y=%+.3f z=%+.3f m/s^2 | "
      "imu: quat: w=%+.3f x=%+.3f y=%+.3f z=%+.3f | "
      "imu: cal: sys=%u gyro=%u accel=%u mag=%u | status=%u error=%u\n",
      linearAccel.x(), linearAccel.y(), linearAccel.z(), orientation.w(),
      orientation.x(), orientation.y(), orientation.z(), systemCalibration,
      gyroCalibration, accelCalibration, magCalibration, systemStatus,
      systemError);
}

void printGpsData() {
  if (!gps.location.isValid()) {
    Serial.printf(
        "GPS: waiting | chars=%lu valid=%lu "
        "passed_checksum=%lu failed_checksum=%lu\n",
        gps.charsProcessed(), gps.sentencesWithFix(), gps.passedChecksum(),
        gps.failedChecksum());
    return;
  }

  Serial.printf(
      "GPS: lat=%+.6f lon=%+.6f alt=%.1f m speed=%.2f km/h "
      "course=%.1f deg sats=%lu\n",
      gps.location.lat(), gps.location.lng(), gps.altitude.meters(),
      gps.speed.kmph(), gps.course.deg(), gps.satellites.value());

  if (ekf.initialized()) {
    Serial.printf("EKF: east=%+.2f m north=%+.2f m velocity=(%+.2f, %+.2f) m/s\n",
                  ekf.east(), ekf.north(), ekf.eastVelocity(),
                  ekf.northVelocity());
  }
}

void readGps() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Wire.begin(IMU_SDA, IMU_SCL);

  delay(1000); // wait for serial monitor to open

  Serial.println("starting GPS and BNO055 sensor test...");
  Serial.printf("GPS UART: %lu baud, RX=%d, TX=%d\n", GPS_BAUD, GPS_RX,
                GPS_TX);

  imuAvailable = bno.begin(OPERATION_MODE_NDOF);
  if (imuAvailable) {
    bno.setExtCrystalUse(true);
    Serial.println("BNO055 initialized.");
  } else {
    Serial.println("BNO055 unavailable; continuing with GPS only.");
  }
}

void loop() {
  readGps();

  uint32_t now = millis();
  updateFilterFromGps();

  if (now - lastGpsReport >= GPS_REPORT_INTERVAL_MS) {
    lastGpsReport = now;
    printImuData();
    printGpsData();
  }
}