#ifdef ROLE_ROBOT

#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

#include <stdint.h>

// --- Explicit Error Flags ---
enum SensorErrorFlags {
    ERROR_NONE              = 0x00,       // 00000000 - All good
    ERROR_BATT_UNDERVOLTAGE = (1 << 0),   // 00000001 - Battery < 10V
    ERROR_BATT_OVERVOLTAGE  = (1 << 1),   // 00000010 - Battery > 13V
    ERROR_TEMP_OVERHEAT     = (1 << 2),   // 00000100 - Temp >= 40C
    ERROR_FEEDBACK_INVERTED = (1 << 3),   // 00001000 - IMU Feedback Issue
    ERROR_FEEDBACK_STALLED  = (1 << 4),   // 00010000 - Output stalled but error persists
    ERROR_SENSOR_OFFLINE    = 0xFF        // 11111111 - I2C/Sensor Failure
};

bool initSystemMonitor();

void raiseErrorFlag(uint8_t flag); // New function to set an error bit
void clearErrorFlags();            // Optional: to reset software errors

// System Status and Diagnostics
uint16_t readBattery();      // Returns battery voltage in mV
int16_t readTemp();          // Returns system temperature in Celsius
uint8_t getErrorFlags();     // Returns bitmask of SensorErrorFlags

#endif // SYSTEM_MONITOR_H
#endif // ROLE_ROBOT