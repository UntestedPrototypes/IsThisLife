#ifdef ROLE_ROBOT

#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <stdint.h>

bool initIMU();
void waitForIMUCalibration();
void runGravityAlignmentCalibration();

// Needed by system_monitor.cpp
int16_t getIMUTemp();

// --- Raw Orientation Data ---
void readMainIMUQuaternion(float* qw, float* qx, float* qy, float* qz);
void readSecondaryIMUQuaternion(float* qw, float* qx, float* qy, float* qz);

// --- Processed Orientation Data ---
void getMainAxisOrientation(float* roll, float* pitch, float* yaw);
void getFullPendulumOrientation(float* internalRoll, float* internalPitch);
float readSecondaryIMUAngle();

void printSecondaryIMUAngles();
void getSecondaryIMUCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);
void tareSecondaryIMU();
void getZeroedSecondaryIMUAngles(float* roll, float* pitch, float* yaw);

#endif // IMU_HANDLER_H
#endif // ROLE_ROBOT