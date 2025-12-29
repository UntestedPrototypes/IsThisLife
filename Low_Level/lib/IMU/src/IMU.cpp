#include "imu.h"

#include <Wire.h>
#include <math.h>

/* ============================================================
   Constructor
   ============================================================ */

IMU::IMU(
    uint8_t id,
    uint8_t i2cAddr,
    int8_t rstPin,
    const Vec3& rodAxisIMU,
    const Vec3& planeRefIMU
)
: _id(id)
, _addr(i2cAddr)
, _rstPin(rstPin)
, _bno(nullptr)
, _rodAxisIMU(vecNormalize(rodAxisIMU))
, _planeRefIMU(vecNormalize(planeRefIMU))
{
    // Optional safety check: ensure orthogonality
    // Caller error if violated
    if (fabsf(vecDot(_rodAxisIMU, _planeRefIMU)) > 1e-3f) {
        // Configuration error — halt
        while (true) { }
    }
}

/* ============================================================
   Startup
   ============================================================ */

bool IMU::begin() {
    _bno = new Adafruit_BNO055(_id, _addr);

    if (_rstPin >= 0) {
        pinMode(_rstPin, OUTPUT);
        digitalWrite(_rstPin, LOW);
        delay(20);          // datasheet: >10 ms
        digitalWrite(_rstPin, HIGH);
        delay(650);         // allow full reboot
    }

    if (!_bno->begin()) {
        return false;
    }

    _bno->setExtCrystalUse(true);
    return true;
}

/* ============================================================
   Public update
   ============================================================ */

void IMU::update() {
    readRaw();
}

/* ============================================================
   Raw sensor read
   ============================================================ */

void IMU::readRaw() {
    imu::Vector<3> a =
        _bno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    accelRaw = { a.x(), a.y(), a.z() };

    imu::Quaternion q = _bno->getQuat();

    // IMPORTANT: correct component order
    orientation = {
        q.w(),
        q.x(),
        q.y(),
        q.z()
    };
}

/* ============================================================
   Calibration status
   ============================================================ */

bool IMU::isCalibrated() const {
    uint8_t s, g, a, m;
    _bno->getCalibration(&s, &g, &a, &m);
    return (s == 3 && g == 3 && a == 3 && m == 3);
}
