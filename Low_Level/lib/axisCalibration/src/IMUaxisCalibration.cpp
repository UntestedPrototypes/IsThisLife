#include "IMUaxisCalibration.h"
#include <Arduino.h>
#include <math.h>

/* =========================================================
   Constructor
   ========================================================= */

IMUaxisCalibration::IMUaxisCalibration()
: active(false),
  stage(CalStage::IDLE),
  startTimeMs(0),
  sampleCount(0),
  gyroSum(0.0f, 0.0f, 0.0f),
  rodAxisIMU(0.0f, 0.0f, 0.0f),
  planeRefIMU(0.0f, 0.0f, 0.0f)
{}

/* =========================================================
   Start calibration
   ========================================================= */

void IMUaxisCalibration::begin()
{
    gyroSum = Vec3(0.0f, 0.0f, 0.0f);
    sampleCount = 0;
    active = true;
    stage = CalStage::ROD_AXIS;
    startTimeMs = millis();

    Serial.println("IMU calibration: STAGE 1");
    Serial.println("Rotate about ROD AXIS");
}

/* =========================================================
   Update (call repeatedly with gyro samples)
   ========================================================= */

void IMUaxisCalibration::update(const Vec3& gyro)
{
    if (!active) return;

    const uint32_t now = millis();

    /* ---------- STAGE 1: Rod axis calibration ---------- */
    if (stage == CalStage::ROD_AXIS) {

        float mag = vecNorm(gyro);
        if (mag > GYRO_THRESH) {
            gyroSum += vecNormalize(gyro);   // accumulate direction
            sampleCount++;
        }

        if (now - startTimeMs < CAL_TIME_MS) return;

        if (sampleCount < MIN_SAMPLES) {
            begin();   // restart stage 1
            return;
        }

        rodAxisIMU = vecNormalize(gyroSum);

        // prepare stage 2
        gyroSum = Vec3(0.0f, 0.0f, 0.0f);
        sampleCount = 0;
        startTimeMs = now;
        stage = CalStage::PLANE_AXIS;

        Serial.println("IMU calibration: STAGE 2");
        Serial.println("Rotate about PLANE (secondary) AXIS");
        return;
    }

    /* ---------- STAGE 2: Plane reference axis calibration ---------- */
    if (stage == CalStage::PLANE_AXIS) {

        // remove rod-axis component
        Vec3 omega_perp =
            gyro - vecDot(gyro, rodAxisIMU) * rodAxisIMU;

        float mag = vecNorm(omega_perp);
        if (mag > GYRO_THRESH) {
            gyroSum += vecNormalize(omega_perp);
            sampleCount++;
        }

        if (now - startTimeMs < CAL_TIME_MS) return;

        if (sampleCount < MIN_SAMPLES) {
            // restart stage 2 only
            gyroSum = Vec3(0.0f, 0.0f, 0.0f);
            sampleCount = 0;
            startTimeMs = now;
            return;
        }

        planeRefIMU = vecNormalize(gyroSum);

        active = false;
        stage = CalStage::DONE;

        Serial.println("IMU axis calibration finished");
    }
}

/* =========================================================
   Status
   ========================================================= */

bool IMUaxisCalibration::calibrating() const
{
    return active;
}

/* =========================================================
   Results
   ========================================================= */

Vec3 IMUaxisCalibration::rodAxis() const
{
    return rodAxisIMU;
}

Vec3 IMUaxisCalibration::planeRef() const
{
    return planeRefIMU;
}
