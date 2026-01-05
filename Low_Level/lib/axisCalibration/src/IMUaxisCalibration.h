#ifndef IMU_AXIS_CALIBRATION_H
#define IMU_AXIS_CALIBRATION_H

#include <stdint.h>
#include "vectors.h"

class IMUaxisCalibration {
public:
    IMUaxisCalibration();

    // Start calibration sequence
    void begin();

    // Feed gyro sample (call repeatedly)
    void update(const Vec3& gyro);

    // True while calibration is active
    bool calibrating() const;

    // Results (valid only when calibrating() == false)
    Vec3 rodAxis() const;
    Vec3 planeRef() const;

private:
    // --- calibration stages ---
    enum class CalStage {
        IDLE,
        ROD_AXIS,
        PLANE_AXIS,
        DONE
    };

    // internal state
    bool active;
    CalStage stage;            // <-- ADD THIS
    uint32_t startTimeMs;
    uint32_t sampleCount;

    Vec3 gyroSum;
    Vec3 rodAxisIMU;
    Vec3 planeRefIMU;

    // parameters
    static constexpr uint32_t CAL_TIME_MS = 6000;
    static constexpr uint32_t MIN_SAMPLES = 50;     // <-- ADD THIS
    static constexpr float    GYRO_THRESH = 1e-4f;  // adjust as needed
};

#endif // IMU_AXIS_CALIBRATION_H
