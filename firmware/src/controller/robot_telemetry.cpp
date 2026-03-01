#ifdef ROLE_CONTROLLER

#include "robot_telemetry.h"
#include "controller_config.h"
#include <Arduino.h>

RobotTelemetry robots[NUM_ROBOTS + 1];

void initTelemetry() {
    for (int i = 0; i <= NUM_ROBOTS; i++) {
        robots[i].robot_id = i;
        robots[i].last_seen = 0;
        robots[i].status = 0;
        robots[i].battery_mv = 0;
        robots[i].motor_temp = 0;
        robots[i].error_flags = 0;
        
        // Init IMU data
        robots[i].main_roll = 0.0f;
        robots[i].main_pitch = 0.0f;
        robots[i].pend_roll = 0.0f;
        robots[i].pend_pitch = 0.0f;
    }
}

void checkNewRobot(uint8_t robot_id) {
    if (robot_id < 1 || robot_id > NUM_ROBOTS) return;

    uint32_t now = millis();
    bool isNew = (robots[robot_id].last_seen == 0);
    bool isReconnected = (now - robots[robot_id].last_seen > 2000); // 2s timeout

    if (isNew || isReconnected) {
        Serial.printf("DEBUG: New robot detected via ESP-NOW: ID %d\n", robot_id);
    }
    
    robots[robot_id].last_seen = now;
}

// Updated to receive and store IMU data
void updateRobotTelemetry(uint8_t robot_id, uint32_t heartbeat, 
                          uint8_t status, uint16_t battery_mv, 
                          int16_t motor_temp, uint8_t error_flags,
                          float main_roll, float main_pitch,
                          float pend_roll, float pend_pitch) {
    if (robot_id < 1 || robot_id > NUM_ROBOTS) return;

    robots[robot_id].last_seen = millis();
    
    robots[robot_id].status = status;
    robots[robot_id].battery_mv = battery_mv;
    robots[robot_id].motor_temp = motor_temp;
    robots[robot_id].error_flags = error_flags;
    
    // Store IMU data
    robots[robot_id].main_roll = main_roll;
    robots[robot_id].main_pitch = main_pitch;
    robots[robot_id].pend_roll = pend_roll;
    robots[robot_id].pend_pitch = pend_pitch;
}

RobotTelemetry* getRobotTelemetry(uint8_t robot_id) {
    if (robot_id < 1 || robot_id > NUM_ROBOTS) return nullptr;
    return &robots[robot_id];
}

#endif // ROLE_CONTROLLER