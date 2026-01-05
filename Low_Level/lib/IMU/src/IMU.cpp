#include "IMU.h"
#include <Wire.h>
#include <cmath>

/* ============================================================
   Internal helpers
   ============================================================ */

static bool readImuQuaternion(Adafruit_BNO055 &imu, Quaternion &out)
{
    imu::Quaternion q = imu.getQuat();

    out.w = static_cast<float>(q.w());
    out.x = static_cast<float>(q.x());
    out.y = static_cast<float>(q.y());
    out.z = static_cast<float>(q.z());

    return true;
}

/* ============================================================
   IMU implementation
   ============================================================ */

IMU::IMU(const char* name_,
         uint8_t i2cAddress_,
         int rstPin_)
: name(name_),
  i2cAddress(i2cAddress_),
  rstPin(rstPin_),
  bno(55, i2cAddress_),
  calibrated(false)
{
    orientation = quatIdentity;
}

void IMU::hardwareReset()
{
    if (rstPin < 0) return;

    pinMode(rstPin, OUTPUT);
    digitalWrite(rstPin, LOW);
    delay(20);
    digitalWrite(rstPin, HIGH);
    delay(50);
}

bool IMU::begin()
{
    hardwareReset();

    if (!bno.begin())
        return false;

    delay(20);
    bno.setExtCrystalUse(true);
    return true;
}

bool IMU::update()
{
    // Read orientation
    readImuQuaternion(bno, orientation);

    // Feed gyro samples into axis calibration
    if (axisCal.calibrating()) {
        Vec3 gyro = readGyroRaw();
        axisCal.update(gyro);

        if (!axisCal.calibrating()) {
            calibrated = true;
        }
    }

    return true;
}

/* ============================================================
   Raw sensor access
   ============================================================ */

Vec3 IMU::readAccelRaw()
{
    imu::Vector<3> a =
        bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    return Vec3(a);
}

Vec3 IMU::readGyroRaw()
{
    imu::Vector<3> g =
        bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    return Vec3(g);
}

Vec3 IMU::readMagRaw()
{
    imu::Vector<3> m =
        bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    return Vec3(m);
}

/* ============================================================
   Calibration control
   ============================================================ */

void IMU::startCalibration()
{
    calibrated = false;
    axisCal.begin();
}

bool IMU::isCalibrated() const
{
    return calibrated;
}

/* ============================================================
   Orientation access
   ============================================================ */

Quaternion IMU::getQuaternion() const
{
    return orientation;
}

Vec3 IMU::getRodAxisIMU() const
{
    return axisCal.rodAxis();
}

Vec3 IMU::getPlaneRefIMU() const
{
    return axisCal.planeRef();
}
