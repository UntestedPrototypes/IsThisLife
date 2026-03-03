#ifdef ROLE_ROBOT
#include "debug.h"

// Default states
bool dbg_general = true;
bool dbg_imu = false;
bool dbg_packets = false;

// Pause state tracking
volatile bool dbg_paused = false;

void pauseDebug() {
    dbg_paused = true;
}

void resumeDebug() {
    if (dbg_paused) {
        dbg_paused = false;
        Serial.println("\n--- [Debug Output Resumed] ---");
    }
}

#endif // ROLE_ROBOT