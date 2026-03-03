#ifdef ROLE_ROBOT
#ifndef ROBOT_PREFERENCES_H
#define ROBOT_PREFERENCES_H

#include <Arduino.h>

struct RobotSettings {
    // Secondary IMU Mounting Offsets
    float imu_off_w;
    float imu_off_x;
    float imu_off_y;
    float imu_off_z;

    // --- NEW: Identification & Connectivity ---
    uint8_t controller_mac[6];
    uint8_t robot_id;

    // --- NEW: Timings & Intervals ---
    uint32_t heartbeat_loss_timeout_ms;
    uint32_t telemetry_interval; 
    uint32_t confirm_timeout_ms;
};

extern RobotSettings robotSettings;

// Core functions
void loadAllPreferences();
void saveIMUOffsets(float qw, float qx, float qy, float qz);

// New save functions
void saveNetworkSettings(uint8_t* mac, uint8_t id);
void saveTimingSettings(uint32_t heartbeat, uint32_t telemetry, uint32_t confirm);

#endif // ROBOT_PREFERENCES_H
#endif // ROLE_ROBOT