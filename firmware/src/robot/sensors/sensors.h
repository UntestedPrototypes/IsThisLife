#ifdef ROLE_ROBOT

#ifndef SENSORS_H
#define SENSORS_H

// This is a wrapper header to keep all existing modules working seamlessly
#include "../math/quaternion_math.h"
#include "system_monitor.h"
#include "imu_handler.h"

// Expose the global readiness flag
extern bool sensorsReady;

// Core initialization function
bool initSensors();

#endif // SENSORS_H
#endif // ROLE_ROBOT