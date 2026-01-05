#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Adafruit_BNO055.h>

#include "vectors.h"
#include "IMUaxisCalibration.h"

/* ============================================================
   IMU class
   ============================================================ */

class IMU {
public:
    // Constructor: fully defines one IMU
    IMU(const char* name,
        uint8_t i2cAddress,
        int rstPin);

    // Hardware control
    bool begin();
    bool update();

    // Calibration control
    void startCalibration();
    bool isCalibrated() const;

    // Sensor outputs (raw or calibrated as needed)
    Vec3 getAccel();
    Vec3 getGyro();
    Vec3 getMag();

    // Orientation
    Quaternion getQuaternion() const;

    // Metadata
    const char* getName() const { return name; }

    Vec3 getRodAxisIMU() const;
    Vec3 getPlaneRefIMU() const;


private:
    // Low-level helpers
    void hardwareReset();

    Vec3 readAccelRaw();
    Vec3 readGyroRaw();
    Vec3 readMagRaw();

private:
    // Identification
    const char* name;

    // Hardware configuration
    uint8_t i2cAddress;
    int rstPin;

    // Sensor driver (owned)
    Adafruit_BNO055 bno;

    // State
    Quaternion orientation;

    IMUaxisCalibration axisCal;
    bool calibrated;
};

#endif // IMU_H
