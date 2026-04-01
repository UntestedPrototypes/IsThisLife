#ifdef ROLE_ROBOT

#include "pid_controller.h"
#include "motors.h"
#include "../sensors/imu_handler.h"
#include "../sensors/system_monitor.h"
#include "../config/robot_preferences.h"
#include "../utils/debug.h"
#include <Arduino.h>

// --- Internal State ---
static uint8_t current_control_mode = MODE_DIRECT;
static uint16_t target_vx = 1500;
static uint16_t target_vy = 1500;
static uint16_t target_omega = 1500;

// Cascade Inner loop state for PITCH
static float pitch_inner_integral = 0.0f, pitch_inner_prev_error = 0.0f;
static float pitch_inner_prev_derivative = 0.0f, pitch_inner_prev_output = 0.0f;

// Cascade Outer loop state for PITCH
static float pitch_outer_integral = 0.0f, pitch_outer_prev_error = 0.0f;
static float pitch_outer_prev_derivative = 0.0f, pitch_outer_prev_output = 0.0f;

// Cascade Inner loop state for ROLL
static float roll_inner_integral = 0.0f, roll_inner_prev_error = 0.0f;
static float roll_inner_prev_derivative = 0.0f, roll_inner_prev_output = 0.0f;

// Cascade Outer loop state for ROLL
static float roll_outer_integral = 0.0f, roll_outer_prev_error = 0.0f;
static float roll_outer_prev_derivative = 0.0f, roll_outer_prev_output = 0.0f;

static float prev_pitch = 0.0f;
static float prev_roll = 0.0f;

static bool pid_first_run = true;

const float MAX_OUTER_TARGET_ANGLE = 45.0f; // Shared limit for secondary target tilt

// --- Private Helper Prototypes ---
float calculateAxisPID(float error, float& integral, float& prev_error, float& prev_derivative, float& prev_output, float kp, float ki, float kd, float dt, float dir, bool first_run);
float calculateOuterPID(float error, float& integral, float& prev_error, float& prev_derivative, float& prev_output, float kp, float ki, float kd, float dt, float limit, bool first_run);

float applyRollSafety(float output, float currentRoll, float dir) {
    float intended_roll_direction = output * dir;

    if (currentRoll > 45.0f && intended_roll_direction > 0.0f) {
        return 0.0f; // Block further positive tilt
    }
    if (currentRoll < -45.0f && intended_roll_direction < 0.0f) {
        return 0.0f; // Block further negative tilt
    }
    return output; 
}

// --- Public Interface ---
void setControlMode(uint8_t mode) {
    current_control_mode = mode;
    if (current_control_mode == MODE_STABILIZED) resetPIDs();
}

uint8_t getControlMode() { return current_control_mode; }

void setTargetVelocities(uint16_t vx, uint16_t vy, uint16_t omega) {
    target_vx = vx; target_vy = vy; target_omega = omega;
}

void resetPIDs() {
    pitch_inner_integral = 0.0f; pitch_inner_prev_error = 0.0f;
    pitch_inner_prev_derivative = 0.0f; pitch_inner_prev_output = 0.0f;
    
    pitch_outer_integral = 0.0f; pitch_outer_prev_error = 0.0f;
    pitch_outer_prev_derivative = 0.0f; pitch_outer_prev_output = 0.0f;

    roll_inner_integral = 0.0f; roll_inner_prev_error = 0.0f;
    roll_inner_prev_derivative = 0.0f; roll_inner_prev_output = 0.0f;

    roll_outer_integral = 0.0f; roll_outer_prev_error = 0.0f;
    roll_outer_prev_derivative = 0.0f; roll_outer_prev_output = 0.0f;
}

float mapRcInput(uint16_t input_us) {
    input_us = constrain(input_us, 1000, 2000);
    return (float)(input_us - 1500) / 500.0f;
}

static float prev_mRoll_angle = 0.0f;
static float prev_sPitch_angle = 0.0f;

float calculateWobbleDamping(float angle, float angular_velocity, float gain, float angle_threshold, float min_velocity) {
    float proximity_multiplier = 1.0f - (abs(angle) / angle_threshold);
    if (proximity_multiplier <= 0.0f) return 0.0f;

    float abs_vel = abs(angular_velocity);
    float velocity_multiplier = 1.0f;
    
    if (abs_vel < min_velocity) {
        velocity_multiplier = abs_vel / min_velocity; 
    }

    return -angular_velocity * gain * proximity_multiplier * velocity_multiplier;
}

void updateStabilizer() {
    float inputX = mapRcInput(target_vx);
    float inputY = mapRcInput(target_vy);

    if (current_control_mode == MODE_STABILIZED) {
        float mRoll, mPitch, mYaw; 
        readMainIMU(&mRoll, &mPitch, &mYaw);
        
        float sRoll, sPitch, sYaw; 
        readSecondaryIMU(&sRoll, &sPitch, &sYaw);
        
        const float dt = 0.01f;

        if (pid_first_run) {
            prev_mRoll_angle = mRoll;
            prev_sPitch_angle = sPitch;
        }
        
        float mRoll_vel = (mRoll - prev_mRoll_angle) / dt;
        float sPitch_vel = (sPitch - prev_sPitch_angle) / dt;
        
        prev_mRoll_angle = mRoll;
        prev_sPitch_angle = sPitch;
        
        float targetPitch = inputX * 45.0f;
        float targetRoll = inputY * 30.0f;

        // --- PITCH AXIS (CASCADING CONTROLLER) ---
        // 1. Outer Loop: Find target Secondary Pitch based on Main Pitch Error
        float pOuterError = targetPitch - mPitch;
        float target_sPitch = calculateOuterPID(pOuterError, pitch_outer_integral, pitch_outer_prev_error,
                                                pitch_outer_prev_derivative, pitch_outer_prev_output,
                                                robotSettings.kp_outer_pitch, robotSettings.ki_outer_pitch, robotSettings.kd_outer_pitch, 
                                                dt, MAX_OUTER_TARGET_ANGLE, pid_first_run);

        // 2. Inner Loop: Hit the Secondary Pitch target generated above
        float pInnerError = target_sPitch - sPitch;
        float outX = calculateAxisPID(pInnerError, pitch_inner_integral, pitch_inner_prev_error, 
                                      pitch_inner_prev_derivative, pitch_inner_prev_output, 
                                      robotSettings.kp_pitch, robotSettings.ki_pitch, robotSettings.kd_pitch, 
                                      dt, robotSettings.pitch_dir, pid_first_run);

        
        // --- ROLL AXIS (CASCADING CONTROLLER) ---
        // 1. Outer Loop: Find target Secondary Roll based on Main Roll Error
        float rOuterError = targetRoll - mRoll;
        float target_sRoll = calculateOuterPID(rOuterError, roll_outer_integral, roll_outer_prev_error,
                                               roll_outer_prev_derivative, roll_outer_prev_output,
                                               robotSettings.kp_outer_roll, robotSettings.ki_outer_roll, robotSettings.kd_outer_roll, 
                                               dt, MAX_OUTER_TARGET_ANGLE, pid_first_run);

        // 2. Inner Loop: Hit the Secondary Roll target generated above
        float rInnerError = target_sRoll - sRoll;
        float outY = calculateAxisPID(rInnerError, roll_inner_integral, roll_inner_prev_error, 
                                      roll_inner_prev_derivative, roll_inner_prev_output, 
                                      robotSettings.kp_roll, robotSettings.ki_roll, robotSettings.kd_roll, 
                                      dt, robotSettings.roll_dir, pid_first_run);

        // Apply Damping
        float pitch_damping = calculateWobbleDamping(sPitch, sPitch_vel, robotSettings.wobble_gain, robotSettings.wobble_threshold, robotSettings.wobble_min_vel);
        float roll_damping = calculateWobbleDamping(mRoll, mRoll_vel, robotSettings.wobble_gain, robotSettings.wobble_threshold, robotSettings.wobble_min_vel);

        outX += (pitch_damping * robotSettings.pitch_dir);
        outY += (roll_damping * robotSettings.roll_dir);

        if (outX > 1.0f) outX = 1.0f;
        if (outX < -1.0f) outX = -1.0f;
        if (outY > 1.0f) outY = 1.0f;
        if (outY < -1.0f) outY = -1.0f;

        pid_first_run = false; 
        outY = applyRollSafety(outY, mRoll, robotSettings.roll_dir);
        
        setMotorSpeed(outX, outY);

    } else {
        pid_first_run = true;
        setMotorSpeed(inputX * robotSettings.pitch_dir, inputY * robotSettings.roll_dir);
    }
}


float calculateAxisPID(float error, float& integral, float& prev_error, float& prev_derivative, float& prev_output, float kp, float ki, float kd, float dt, float dir, bool first_run) {
    const float DEADBAND = 1.5f; 
    if (abs(error) < robotSettings.deadband) {
        error = 0.0f; 
    }

    if (first_run) {
        prev_error = error; 
        prev_derivative = 0.0f; 
        prev_output = 0.0f;     
    }
    
    if (error != 0.0f) {
        integral += error * dt;
    }
    if (ki > 0.0001f) {
        float max_i = 1.0f / ki; 
        if (integral > max_i) integral = max_i;
        if (integral < -max_i) integral = -max_i;
    }
    
    float raw_derivative = (error - prev_error) / dt;
    float derivative = (raw_derivative * robotSettings.d_alpha) + (prev_derivative * (1.0f - robotSettings.d_alpha));
    
    prev_derivative = derivative;
    prev_error = error;
    
    float raw_output = (kp * error) + (ki * integral) + (kd * derivative);
    float smoothed_output = (raw_output * robotSettings.out_alpha) + (prev_output * (1.0f - robotSettings.out_alpha));
    prev_output = smoothed_output;
    
    smoothed_output *= dir;
    if (smoothed_output > 1.0f) smoothed_output = 1.0f;
    if (smoothed_output < -1.0f) smoothed_output = -1.0f;
    
    return smoothed_output;
}

float calculateOuterPID(float error, float& integral, float& prev_error, float& prev_derivative, float& prev_output, float kp, float ki, float kd, float dt, float limit, bool first_run) {
    if (first_run) {
        prev_error = error; 
        prev_derivative = 0.0f; 
        prev_output = 0.0f;     
    }
    
    if (error != 0.0f) {
        integral += error * dt;
    }
    
    if (ki > 0.0001f) {
        float max_i = limit / ki; 
        if (integral > max_i) integral = max_i;
        if (integral < -max_i) integral = -max_i;
    }
    
    float derivative = (error - prev_error) / dt;
    prev_derivative = derivative;
    prev_error = error;
    
    float output = (kp * error) + (ki * integral) + (kd * derivative);
    
    if (output > limit) output = limit;
    if (output < -limit) output = -limit;
    
    return output;
}

#endif