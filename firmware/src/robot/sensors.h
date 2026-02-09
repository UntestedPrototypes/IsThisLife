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
int16_t readMotorTemp(); // Returns degrees C
uint8_t getErrorFlags();

// Expose sensors for advanced logic (Sequence/PID)
extern Adafruit_BNO055 imuL;
extern Adafruit_BNO055 imuR;
extern bool sensorsReady;

#endif // SENSORS_H
#endif // ROLE_ROBOT