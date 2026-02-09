#ifdef ROLE_CONTROLLER
#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#include <stdint.h>
#include "controller_config.h"

// Robot telemetry structure
struct RobotTelemetry {
    uint32_t lastAckHeartbeat;
    uint32_t roundTripLatency; // RTT in ms
    uint8_t status;
    uint16_t batteryMv;
    int16_t motorTemp;
    uint8_t errorFlags;
};

// Array of robot telemetry data
extern RobotTelemetry robots[NUM_ROBOTS];

// Functions
void updateRobotTelemetry(uint8_t robot_id, uint32_t heartbeat, uint8_t status, 
                          uint16_t battery_mv, int16_t motor_temp, uint8_t error_flags);
void printRobotTelemetry(int idx);

#endif // ROBOT_TELEMETRY_H
#endif // ROLE_CONTROLLER
