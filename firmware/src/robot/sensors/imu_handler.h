#ifdef ROLE_ROBOT

#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <stdint.h>

bool initIMU();
void waitForIMUCalibration();
void CalibrateIMUOffset();

// Needed by system_monitor.cpp
int16_t getIMUTemp();

// --- Processed & Zeroed Orientation Data ---
void readMainIMU(float* roll, float* pitch, float* yaw);
void readSecondaryIMU(float* roll, float* pitch, float* yaw);

void printIMU();

// --- Calibration Status ---
// Returns the lowest calibration state (0-3) among all sensors on both IMUs
void getIMUCalibrationState(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);

#endif // IMU_HANDLER_H
#endif // ROLE_ROBOT