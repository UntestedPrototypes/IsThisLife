#ifdef ROLE_CONTROLLER
#include "heartbeat_manager.h"
#include "controller_config.h"
#include "robot_commands.h"
#include <Arduino.h>

// Heartbeat timing state
uint32_t lastHeartbeatTime = 0;

void sendPeriodicHeartbeat() {
    if (millis() - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatTime = millis();
        sendHeartbeat();
    }
}
#endif // ROLE_CONTROLLER
