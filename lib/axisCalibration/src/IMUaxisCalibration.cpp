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
  time_rod(0),
  gyroDirSum(0.0f, 0.0f, 0.0f),
  rodAxisIMU(0.0f, 0.0f, 0.0f),
  planeRefIMU(0.0f, 0.0f, 0.0f)
{}

/* =========================================================
   Start calibration
   ========================================================= */
void IMUaxisCalibration::begin()
{
    gyroDirSum  = Vec3(0.0f, 0.0f, 0.0f);
    sampleCount = 0;

    active      = true;
    stage       = CalStage::ROD_AXIS;
    startTimeMs = millis();
    time_rod    = startTimeMs;

    sendToLaptop("CALIBRATION START");
    sendToLaptop("Step 1: Rotate entire robot around the ROD axis.");
    sendToLaptop("You can rotate back and forth.");
}

/* =========================================================
   Update
   ========================================================= */
void IMUaxisCalibration::update(const Vec3& gyroIMU)
{
    if (!active) return;

    const uint32_t now = millis();
    float mag = vecNorm(gyroIMU);
    if (mag < GYRO_THRESH) return; 

    Vec3 currentDir;

    /* ---------- STAGE 1: Rod axis calibration ---------- */
    if (stage == CalStage::ROD_AXIS) {
        currentDir = vecNormalize(gyroIMU);

        if (sampleCount > 0 && vecDot(currentDir, gyroDirSum) < 0) {
            currentDir = -1.0f * currentDir;
        }

        gyroDirSum += currentDir;
        sampleCount++;

        if (now - startTimeMs < CAL_TIME_MS) return;

        if (sampleCount < MIN_SAMPLES) {
            sendToLaptop("Failed: Not enough motion. Retrying Stage 1...");
            begin(); 
            return;
        }

        rodAxisIMU = vecNormalize(gyroDirSum);

        gyroDirSum  = Vec3(0.0f, 0.0f, 0.0f);
        sampleCount = 0;
        startTimeMs = now;
        stage       = CalStage::PLANE_REF;

        sendToLaptop("Stage 1 Done.");
        time_rod = now;
        sendToLaptop("Step 2: Spin the internal PLANE around its normal.");
        return;
    }
    /* ---------- STAGE 2: Plane reference calibration ---------- */
    if (stage == CalStage::PLANE_REF) {
        if (now - time_rod < 5000) return;
        if (sampleCount == 0) sendToLaptop("Collecting data for Plane Reference...");
        Vec3 omegaPerp = gyroIMU - vecDot(gyroIMU, rodAxisIMU) * rodAxisIMU;
        
        if (vecNorm(omegaPerp) < GYRO_THRESH) return;
        currentDir = vecNormalize(omegaPerp);

        if (sampleCount > 0 && vecDot(currentDir, gyroDirSum) < 0) {
            currentDir = -1.0f * currentDir;
        }

        gyroDirSum += currentDir;
        sampleCount++;

        if (now - startTimeMs < CAL_TIME_MS) return;

        if (sampleCount < MIN_SAMPLES) {
            sendToLaptop("Failed: Not enough motion. Retrying Stage 2...");
            gyroDirSum = Vec3(0.0f, 0.0f, 0.0f);
            sampleCount = 0;
            startTimeMs = now;
            return;
        }

        planeRefIMU = vecNormalize(gyroDirSum);
        active = false;
        stage  = CalStage::DONE;

        sendToLaptop("Calibration Finished Success.");
    }
  
}

/* =========================================================
   Missing Implementation Getters (Fixes Linker Errors)
   ========================================================= */

bool IMUaxisCalibration::calibrating() const {
    return active;
}

Vec3 IMUaxisCalibration::getRodAxisIMU() const {
    return rodAxisIMU;
}

Vec3 IMUaxisCalibration::getPlaneRefIMU() const {
    return planeRefIMU;
}