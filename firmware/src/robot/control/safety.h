#ifdef ROLE_ROBOT

#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>

// Safety state
extern volatile bool estopActive;
extern volatile bool motorsEnabled;
extern volatile bool calibrationRequired;

// Functions
bool isEstopActive();
void activateEstop();
bool clearEstop();

bool areMotorsEnabled();

bool isCalibrationRequired();
void setCalibrationRequired(bool required);

#endif // SAFETY_H
#endif // ROLE_ROBOT