#ifdef ROLE_ROBOT

#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>

// Safety state
extern volatile bool estopActive;
extern volatile bool motorsEnabled;

// Functions
void activateEstop();
void clearEstop();
bool isEstopActive();
bool areMotorsEnabled();

#endif // SAFETY_H
#endif // ROLE_ROBOT