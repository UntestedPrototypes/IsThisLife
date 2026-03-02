#ifdef ROLE_ROBOT

#include "heartbeat.h"
#include "../config/robot_preferences.h"
#include <Arduino.h>

// Heartbeat monitoring state
uint32_t lastValidHeartbeatTime = 0;
uint32_t hbTimes[WINDOW_SIZE];
uint8_t hbCount = 0;

void recordHeartbeat(uint32_t timestamp) {
    hbTimes[hbCount % WINDOW_SIZE] = timestamp;
    hbCount++;
}

bool heartbeatValid() {
    uint32_t now = millis();
    
    uint8_t validCount = 0;
    for (int i = 0; i < WINDOW_SIZE && i < hbCount; i++) {
        if (now - hbTimes[i] <= WINDOW_TIME_MS) {
            validCount++;
        }
    }
    
    bool isValid = (validCount >= MIN_VALID);
    
    if (isValid) {
        lastValidHeartbeatTime = now;
    }
    
    if (!isValid && (now - lastValidHeartbeatTime > robotSettings.heartbeat_loss_timeout_ms)) {
        return false;
    }
    
    return (lastValidHeartbeatTime > 0);
}
#endif // ROLE_ROBOT