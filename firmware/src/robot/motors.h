#ifdef ROLE_ROBOT

#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

// Motor control functions
void setMotors(int8_t vx, int8_t vy, int8_t omega);
void stopMotors();

#endif // MOTORS_H
#endif // ROLE_ROBOT