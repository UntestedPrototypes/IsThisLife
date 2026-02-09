#ifdef ROLE_ROBOT

#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

// Sensor reading functions
uint16_t readBattery();
int16_t readMotorTemp();
uint8_t getErrorFlags();

#endif // SENSORS_H
#endif // ROLE_ROBOT