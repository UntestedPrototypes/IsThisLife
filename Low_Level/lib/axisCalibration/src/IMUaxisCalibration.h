#ifndef IMU_AXIS_CALIBRATION_H
#define IMU_AXIS_CALIBRATION_H

#include <stdint.h>
#include "vectors.h"

/*
  IMUaxisCalibration

  Fully self-contained calibration of:
    - rodAxisIMU  (rotation axis)
    - planeRefIMU (in-plane reference)

  Intended usage:

    IMUaxisCalibration IMUCal;

    IMUCal.begin();
    while (IMUCal.calibrating()) {
        IMUCal.update(gyro);
        delay(10);
    }

    Vec3 rod  = IMUCal.rodAxis();
    Vec3 ref  = IMUCal.planeRef();
*/

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
    // internal state
    bool active;
    uint32_t startTimeMs;
    uint32_t sampleCount;

    Vec3 gyroSum;
    Vec3 rodAxisIMU;
    Vec3 planeRefIMU;

    // parameters
    static constexpr uint32_t CAL_TIME_MS = 6000;
    static constexpr float GYRO_THRESH   = 1e-3f;

    // math helpers
    static Vec3 normalize(const Vec3& v);
    static float dot(const Vec3& a, const Vec3& b);
    static Vec3 scale(const Vec3& v, float s);
    static Vec3 subtract(const Vec3& a, const Vec3& b);
};

#endif // IMU_AXIS_CALIBRATION_H
