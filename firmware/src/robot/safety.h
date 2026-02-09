#ifdef ROLE_ROBOT

#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>

// Safety state
extern bool estopActive;
extern bool motorsEnabled;

// Functions
void activateEstop();
void clearEstop();
bool isEstopActive();
bool areMotorsEnabled();

#endif // SAFETY_H
#endif // ROLE_ROBOT