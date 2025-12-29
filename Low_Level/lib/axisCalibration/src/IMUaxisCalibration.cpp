#include "IMUaxisCalibration.h"
#include <Arduino.h>
#include <math.h>

Vec3 rodAxisIMU;
Vec3 planeRefIMU;


IMUaxisCalibration::IMUaxisCalibration()
: active(false),
  startTimeMs(0),
  sampleCount(0),
  gyroSum{0,0,0},
  rodAxisIMU{0,0,0},
  planeRefIMU{0,0,0}
{}

void IMUaxisCalibration::begin() {
    gyroSum = {0,0,0};
    sampleCount = 0;
    active = true;
    startTimeMs = millis();

    Serial.println("IMU axis calibration started");
    Serial.println("Rotate internal plane about rod axis");
}

void IMUaxisCalibration::update(const Vec3& gyro) {

    if (!active) return;

    float mag = sqrtf(gyro.x*gyro.x +
                      gyro.y*gyro.y +
                      gyro.z*gyro.z);

    // accumulate only meaningful rotation
    if (mag > GYRO_THRESH) {
        gyroSum.x += gyro.x;
        gyroSum.y += gyro.y;
        gyroSum.z += gyro.z;
        sampleCount++;
    }

    // wait until calibration window expires
    if (millis() - startTimeMs < CAL_TIME_MS) return;

    // require sufficient motion
    if (sampleCount < 50) {
        begin(); // restart
        return;
    }

    // rod axis from average rotation axis
    rodAxisIMU = normalize(gyroSum);

    // construct plane reference automatically
    Vec3 candidate = {1.0f, 0.0f, 0.0f};
    if (fabs(dot(candidate, rodAxisIMU)) > 0.9f) {
        candidate = {0.0f, 1.0f, 0.0f};
    }

    planeRefIMU = subtract(
        candidate,
        scale(rodAxisIMU, dot(candidate, rodAxisIMU))
    );
    planeRefIMU = normalize(planeRefIMU);

    active = false;

    Serial.println("IMU axis calibration finished");
}

bool IMUaxisCalibration::calibrating() const {
    return active;
}

Vec3 IMUaxisCalibration::rodAxis() const {
    return rodAxisIMU;
}

Vec3 IMUaxisCalibration::planeRef() const {
    return planeRefIMU;
}

/* =========================
   Math helpers
   ========================= */

Vec3 IMUaxisCalibration::normalize(const Vec3& v) {
    float n = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
    if (n < 1e-6f) return {0,0,0};
    return { v.x/n, v.y/n, v.z/n };
}

float IMUaxisCalibration::dot(const Vec3& a, const Vec3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vec3 IMUaxisCalibration::scale(const Vec3& v, float s) {
    return { s*v.x, s*v.y, s*v.z };
}

Vec3 IMUaxisCalibration::subtract(const Vec3& a, const Vec3& b) {
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}
