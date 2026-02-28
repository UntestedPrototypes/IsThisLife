#ifdef ROLE_ROBOT

#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_INA219.h>

// Initialization
bool initSensors();

// Sensor reading functions
uint16_t readBattery(); // Returns mV
int16_t readTemp(); // Returns degrees C
uint8_t getErrorFlags();

// IMU angle reading functions (returns degrees)
float readMainIMUAngle();       // Read pitch from main IMU (imuMain - 0x29) - Euler angle
float readMainIMUMotorAxisRotation();  // Read rotation around main motor's roll axis (accounts for 3D tumbling)
float readSecondaryIMUAngle();  // Read pitch from secondary IMU (imuSecondary - 0x28)
float readSecondaryIMUMotorAxisRotation();  // Read rotation around secondary motor's pitch axis (accounts for 3D tumbling)

// Quaternion functions (robust, no gimbal lock)
void readMainIMUQuaternion(float* qw, float* qx, float* qy, float* qz);
void readSecondaryIMUQuaternion(float* qw, float* qx, float* qy, float* qz);
void quaternionToEuler(float qw, float qx, float qy, float qz, float* roll, float* pitch, float* yaw);

// Expose sensors for advanced logic (Sequence/PID)
extern Adafruit_BNO055 imuMain;
extern Adafruit_BNO055 imuSecondary;
extern bool sensorsReady;

#endif // SENSORS_H
#endif // ROLE_ROBOT