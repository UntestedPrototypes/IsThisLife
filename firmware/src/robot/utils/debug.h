#ifdef ROLE_ROBOT
#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

// Global debug state flags
extern bool dbg_general;
extern bool dbg_imu;
extern bool dbg_packets;

// --- NEW: Manual Pause State ---
extern volatile bool dbg_paused;
void pauseDebug();
void resumeDebug();

// --- Debug Macros ---
#define DEBUG_PRINTLN(x) do { if(dbg_general && !dbg_paused) { Serial.println(x); } } while(0)
#define DEBUG_PRINTF(fmt, ...) do { if(dbg_general && !dbg_paused) { Serial.printf(fmt, ##__VA_ARGS__); } } while(0)

#define DEBUG_IMU_PRINTLN(x) do { if(dbg_imu && !dbg_paused) { Serial.println(x); } } while(0)
#define DEBUG_IMU_PRINTF(fmt, ...) do { if(dbg_imu && !dbg_paused) { Serial.printf(fmt, ##__VA_ARGS__); } } while(0)

#define DEBUG_PKT_PRINTLN(x) do { if(dbg_packets && !dbg_paused) { Serial.println(x); } } while(0)
#define DEBUG_PKT_PRINTF(fmt, ...) do { if(dbg_packets && !dbg_paused) { Serial.printf(fmt, ##__VA_ARGS__); } } while(0)

#endif // DEBUG_H
#endif // ROLE_ROBOT