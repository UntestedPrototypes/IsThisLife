#pragma once

#include <stdint.h>
#include <vectors.h>
#include <Adafruit_BNO055.h>

class IMU {
public:
    IMU(
        uint8_t id,
        uint8_t i2cAddr,
        int8_t rstPin,
        const Vec3& rodAxisIMU,
        const Vec3& planeRefIMU
    );

    bool begin();
    void update();
    bool isCalibrated() const;

    // Accessors
    const Quaternion& getQuaternion() const { return orientation; }
    const Vec3& getRodAxisIMU() const { return _rodAxisIMU; }
    const Vec3& getPlaneRefIMU() const { return _planeRefIMU; }

private:
    void readRaw();
    // void applyCalibration();  // REMOVE unless implemented

    uint8_t _id;
    uint8_t _addr;
    int8_t  _rstPin;

    Adafruit_BNO055* _bno;

    // Sensor outputs
    Vec3 accelRaw;
    Quaternion orientation;

    // Mounting geometry
    Vec3 _rodAxisIMU;
    Vec3 _planeRefIMU;
};
