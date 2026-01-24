#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO055.h>
#include "vectors.h"
#include "IMUaxisCalibration.h"
inline float wrapAngle180(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}
struct IMUAngles;

class IMU {
public:
    IMU(const char* name, uint8_t address);

    bool begin();
    void update();

    void startCalibration();
    void zeroIMUAngles();
    bool isCalibrated() const;
    bool isCalibrating() const;


    // These cannot be 'const' because the BNO055 library isn't 'const' safe
    IMUAngles getAngles();
    Quaternion getQuaternion();
    Vec3 getGyro();

    Vec3 getRodAxis() const { return axisRod; }
    Vec3 getPlaneRef() const { return axisPlane; }
    const char* getName() const { return _name; }

private:
    Adafruit_BNO055 bno;
    IMUaxisCalibration calibrator;
    
    const char* _name;
    bool _isCalibrated;

    Vec3 axisRod;
    Vec3 axisPlane;
};

#endif