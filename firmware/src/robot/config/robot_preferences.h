#ifdef ROLE_ROBOT
#ifndef ROBOT_PREFERENCES_H
#define ROBOT_PREFERENCES_H

#include <Arduino.h>

struct RobotSettings {
    float imu_off_w;
    float imu_off_x;
    float imu_off_y;
    float imu_off_z;

    uint8_t controller_mac[6];
    uint8_t robot_id;

    uint32_t heartbeat_loss_timeout_ms;
    uint32_t telemetry_interval; 
    uint32_t confirm_timeout_ms;

    int32_t encoder_limit_min;
    int32_t encoder_limit_max;

    float kp_pitch; float ki_pitch; float kd_pitch;
    float kp_roll;  float ki_roll;  float kd_roll;
    float kp_outer_roll; float ki_outer_roll; float kd_outer_roll; // <-- NEW
    float pitch_dir; float roll_dir;

    float wobble_gain;
    float wobble_threshold;
    float wobble_min_vel;

    float d_alpha;
    float out_alpha;
    float deadband;

    bool dev_mode;
};

extern RobotSettings robotSettings;

void loadAllPreferences();
void saveIMUOffsets(float qw, float qx, float qy, float qz);

void saveNetworkSettings(uint8_t* mac, uint8_t id);
void saveTimingSettings(uint32_t heartbeat, uint32_t telemetry, uint32_t confirm);
void saveDebugSettings(bool gen, bool imu, bool pkt_rx, bool pkt_tx);
void saveEncoderLimits(int32_t min_limit, int32_t max_limit);

// <-- SIGNATURE UPDATED 
void savePidSettings(float kpp, float kip, float kdp, 
                     float kpr, float kir, float kdr, 
                     float kpor, float kior, float kdor, 
                     float pdir, float rdir);

void saveFilterSettings(float d_alpha, float out_alpha, float deadband);
void saveWobbleSettings(float gain, float threshold, float min_vel);

#endif // ROBOT_PREFERENCES_H
#endif // ROLE_ROBOT