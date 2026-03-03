#ifdef ROLE_CONTROLLER
#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#include <stdint.h>
#include "packets.h"

struct RobotTelemetry {
    uint8_t robot_id;
    uint32_t last_seen;
    uint8_t status;
    uint16_t battery_mv;
    int16_t motor_temp;
    uint8_t error_flags;
    uint16_t imu_calibration;
    
    // --- IMU Telemetry Data ---
    float main_roll;
    float main_pitch;
    float pend_roll;
    float pend_pitch;
};

void initTelemetry();

// Check if robot is new or reconnecting
void checkNewRobot(uint8_t robot_id);

// Updated signature to accept IMU data
void updateRobotTelemetry(uint8_t robot_id, uint32_t heartbeat, 
                          uint8_t status, uint16_t battery_mv, 
                          int16_t motor_temp, uint8_t error_flags,
                          uint16_t imu_calibration,
                          float main_roll, float main_pitch,
                          float pend_roll, float pend_pitch);

RobotTelemetry* getRobotTelemetry(uint8_t robot_id);

#endif // ROBOT_TELEMETRY_H
#endif // ROLE_CONTROLLER