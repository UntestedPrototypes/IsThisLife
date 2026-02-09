#ifdef ROLE_CONTROLLER
#include "robot_telemetry.h"
#include <Arduino.h>

// Array of robot telemetry data
RobotTelemetry robots[NUM_ROBOTS];

void updateRobotTelemetry(uint8_t robot_id, uint32_t heartbeat, uint8_t status, 
                          uint16_t battery_mv, int16_t motor_temp, uint8_t error_flags) {
    int idx = robot_id - 1;
    if (idx >= 0 && idx < NUM_ROBOTS) {
        RobotTelemetry &r = robots[idx];
        r.lastAckHeartbeat = heartbeat;
        r.status = status;
        r.batteryMv = battery_mv;
        r.motorTemp = motor_temp;
        r.errorFlags = error_flags;
    }
}

void printRobotTelemetry(int idx) {
    if (idx < 0 || idx >= NUM_ROBOTS) return;
    
    RobotTelemetry &r = robots[idx];
    Serial.printf(
        "Robot %d: HB=%u lastSeen=%u status=%d battery=%u mv temp=%d err=0x%02X latency=%u ms\n",
        idx + 1,
        r.lastAckHeartbeat,
        r.roundTripLatency,
        r.status,
        r.batteryMv,
        r.motorTemp,
        r.errorFlags
    );
}
#endif // ROLE_CONTROLLER
