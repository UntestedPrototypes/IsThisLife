#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <stdint.h>

// Mode definitions
#define MODE_DIRECT 0
#define MODE_STABILIZED 1

void setControlMode(uint8_t mode);
uint8_t getControlMode();
void setTargetVelocities(uint16_t vx_us, uint16_t vy_us, uint16_t omega_us);

void resetPIDs();
void updateStabilizer();

#endif