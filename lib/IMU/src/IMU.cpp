#include "imu.h"
#include "angles.h"
#include <Arduino.h>
#include <cmath>



/* =========================================================
   Constructor - Fixed ambiguous initializer list
   ========================================================= */
IMU::IMU(const char* name, uint8_t address) 
    : bno(55, address),
      _name(name),
      _isCalibrated(false),
      axisRod(Vec3(0.0f, 1.0f, 0.0f)),   // Explicitly call Vec3 constructor
      axisPlane(Vec3(0.0f, 0.0f, 1.0f))  // Explicitly call Vec3 constructor
{
}

/* =========================================================
   Lifecycle
   ========================================================= */
bool IMU::begin() {
    if (!bno.begin()) {
        return false; 
    }
    
    bno.setExtCrystalUse(true);
    bno.setMode(OPERATION_MODE_NDOF); 
    
    delay(100); 
    return true;
}

void IMU::update() {
    if (calibrator.calibrating()) {
        Vec3 gyro = getGyro();
        calibrator.update(gyro);
        
        if (!calibrator.calibrating()) {
            axisRod   = calibrator.getRodAxisIMU();
            axisPlane = calibrator.getPlaneRefIMU();
            _isCalibrated = true;
        }
    }
}



/* =========================================================
   Math Service Call
   ========================================================= */
IMUAngles IMU::getAngles() {
    return performDecomposition(getQuaternion(), axisPlane, axisRod);}




/* =========================================================
   Sensor Data - Removed 'const' to satisfy Adafruit Lib
   ========================================================= */
Quaternion IMU::getQuaternion() {
    imu::Quaternion q = bno.getQuat();
    return Quaternion(q.w(), q.x(), q.y(), q.z());
}

Vec3 IMU::getGyro() {
    imu::Vector<3> g = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    return Vec3(g.x(), g.y(), g.z());
}

/* =========================================================
   Calibration Wrappers
   ========================================================= */
void IMU::startCalibration() {
    _isCalibrated = false;
    calibrator.begin();
}

bool IMU::isCalibrated() const {
    return _isCalibrated;
}

bool IMU::isCalibrating() const {
    return calibrator.calibrating();
}



// NOTE: getRodAxis() and getPlaneRef() removed from here 
// because they are already defined inline in imu.h