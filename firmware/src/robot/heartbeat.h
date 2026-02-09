#ifdef ROLE_ROBOT

#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#include <stdint.h>
#include "robot_config.h"

// Heartbeat monitoring state
extern uint32_t lastValidHeartbeatTime;
extern uint32_t hbTimes[WINDOW_SIZE];
extern uint8_t hbCount;

// Functions
void recordHeartbeat(uint32_t timestamp);
bool heartbeatValid();

#endif // HEARTBEAT_H
#endif // ROLE_ROBOT