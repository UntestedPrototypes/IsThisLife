#ifndef IMU_AXIS_CALIBRATION_H
#define IMU_AXIS_CALIBRATION_H

#include <Arduino.h>
#include <stdint.h>
#include "vectors.h"

/* =========================================================
   External logging hook (provided elsewhere)
   ========================================================= */

void sendToLaptop(const String& msg);

/* =========================================================
   IMU Axis Calibration
   ========================================================= */

class IMUaxisCalibration {
public:
    IMUaxisCalibration();

    /* ---- control ---- */
    void begin();
    void update(const Vec3& gyroIMU);

    /* ---- status ---- */
    bool calibrating() const;

    /* ---- results (IMU/body frame, unit vectors) ---- */
    Vec3 getRodAxisIMU() const;
    Vec3 getPlaneRefIMU() const;

private:
    enum class CalStage {
        IDLE,
        ROD_AXIS,
        PLANE_REF,
        DONE
    };

private:
    bool active;
    CalStage stage;

    uint32_t startTimeMs;
    uint32_t sampleCount;
    uint32_t time_rod;

    /* accumulated gyro directions */
    Vec3 gyroDirSum;

    /* calibrated axes (body frame) */
    Vec3 rodAxisIMU;
    Vec3 planeRefIMU;
};

/* =========================================================
   Calibration parameters
   ========================================================= */

/*
 * These can stay here or move to a config header.
 * They are part of calibration policy, not math.
 */
constexpr float    GYRO_THRESH  = 0.05f;     // rad/s
constexpr uint32_t CAL_TIME_MS  = 1000;      // per stage
constexpr uint32_t MIN_SAMPLES  = 100;

#endif // IMU_AXIS_CALIBRATION_H
