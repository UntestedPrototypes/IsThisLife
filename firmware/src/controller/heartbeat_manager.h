#ifdef ROLE_CONTROLLER
#ifndef HEARTBEAT_MANAGER_H
#define HEARTBEAT_MANAGER_H

#include <stdint.h>

// Heartbeat timing state
extern uint32_t lastHeartbeatTime;

// Functions
void sendPeriodicHeartbeat();

#endif // HEARTBEAT_MANAGER_H
#endif // ROLE_CONTROLLER
