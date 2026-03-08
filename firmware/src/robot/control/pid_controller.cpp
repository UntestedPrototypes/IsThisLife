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

static float pitch_integral = 0.0f, pitch_prev_error = 0.0f;
static float roll_integral = 0.0f, roll_prev_error = 0.0f;
static bool pid_first_run = true;

// --- Private Helper Prototypes ---
float calculateAxisPID(float error, float& integral, float& prev_error, float kp, float ki, float kd, float dt, float dir, bool first_run);

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
    pitch_integral = 0.0f; pitch_prev_error = 0.0f;
    roll_integral = 0.0f; roll_prev_error = 0.0f;
}

float mapRcInput(uint16_t input_us) {
    input_us = constrain(input_us, 1000, 2000);
    return (float)(input_us - 1500) / 500.0f;
}

/**
 * Simplified 100Hz Orchestrator
 * Uses RC input as target angles for basic stabilization.
 */
void updateStabilizer() {
    // Map RC inputs (-1.0 to 1.0)
    float inputX = mapRcInput(target_vx);
    float inputY = mapRcInput(target_vy);

    if (current_control_mode == MODE_STABILIZED) {
        float mRoll, mPitch, mYaw; 
        readMainIMU(&mRoll, &mPitch, &mYaw);
        
        float sRoll, sPitch, sYaw; 
        readSecondaryIMU(&sRoll, &sPitch, &sYaw);
        
        const float dt = 0.01f;
        
        // Define target angles based on input (e.g., +/- 30 degrees max)
        float targetPitch = inputX * 45.0f;
        float targetRoll = inputY * 30.0f;

        // Calculate Errors:
        // Secondary IMU used for pitch axis targeting
        float pError = targetPitch - sPitch;
        // Main IMU used for roll axis targeting
        float rError = targetRoll - mRoll;

        // Calculate PID with new safe startup flag
        float outX = calculateAxisPID(pError, pitch_integral, pitch_prev_error, 
                                     robotSettings.kp_pitch, robotSettings.ki_pitch, robotSettings.kd_pitch, dt, robotSettings.pitch_dir, pid_first_run);
        float outY = calculateAxisPID(rError, roll_integral, roll_prev_error, 
                                     robotSettings.kp_roll, robotSettings.ki_roll, robotSettings.kd_roll, dt, robotSettings.roll_dir, pid_first_run);

        pid_first_run = false; // Mark that we've completed our safe startup
        // Apply outputs directly to motors
        setMotorSpeed(outX, outY);

        DEBUG_PRINTF("DEBUG: Stabilized Mode | Target P: %.1f R: %.1f | Meas P: %.1f R: %.1f | Out P: %.3f R: %.3f\n", 
                     targetPitch, targetRoll, sPitch, mRoll, outX, outY);
    } else {
        // Direct Mode: Pass through mapped inputs
        pid_first_run = true;
        setMotorSpeed(inputX, inputY);
    }
}

float calculateAxisPID(float error, float& integral, float& prev_error, float kp, float ki, float kd, float dt, float dir, bool first_run) {
    // 1. Prevent Derivative Kick on startup
    if (first_run) {
        prev_error = error; 
    }
    
    // 2. Calculate Integral with Anti-Windup
    integral += error * dt;
    if (ki > 0.0001f) {
        float max_i = 1.0f / ki; // Prevent integral from demanding more than 100% motor speed
        if (integral > max_i) integral = max_i;
        if (integral < -max_i) integral = -max_i;
    }
    
    // 3. Calculate Derivative
    float derivative = (error - prev_error) / dt;
    prev_error = error;
    
    // 4. Sum PID Output
    float output = (kp * error) + (ki * integral) + (kd * derivative);
    
    // 5. Apply direction modifier and hard clamp the output
    output *= dir;
    if (output > 1.0f) output = 1.0f;
    if (output < -1.0f) output = 1.0f;
    
    return output;
}

#endif