#ifdef ROLE_ROBOT

#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <stdint.h>

bool initIMU();
int16_t getIMUTemp();

// --- Processed & Zeroed Orientation Data ---
void readMainIMU(float* roll, float* pitch, float* yaw);
void readSecondaryIMU(float* roll, float* pitch, float* yaw);
void printIMU();

// --- Calibration Status ---
void getIMUCalibrationState(uint8_t* mGyro, uint8_t* mAccel, uint8_t* mMag, 
                            uint8_t* sGyro, uint8_t* sAccel, uint8_t* sMag);
bool isIMUCalibrated();

// --- Non-Blocking Offset Calibration Helpers ---
void resetOffsetAccumulator();
bool accumulateOffsetSample(); 
void saveUprightVector();
void calculateAndSaveOffsets();

#endif // IMU_HANDLER_H
#endif // ROLE_ROBOT