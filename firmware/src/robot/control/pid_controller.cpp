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

// --- NEW: State variables for the filters ---
static float pitch_prev_derivative = 0.0f, pitch_prev_output = 0.0f;
static float roll_prev_derivative = 0.0f, roll_prev_output = 0.0f;

static bool pid_first_run = true;

// --- Private Helper Prototypes ---
// NEW SIGNATURE: Added prev_derivative and prev_output to maintain filter states across loops
float calculateAxisPID(float error, float& integral, float& prev_error, float& prev_derivative, float& prev_output, float kp, float ki, float kd, float dt, float dir, bool first_run);

float applyRollSafety(float output, float currentRoll, float dir) {
    // Translate the raw motor command back into the IMU's physical coordinate space
    float intended_roll_direction = output * dir;

    if (currentRoll > 45.0f && intended_roll_direction > 0.0f) {
        return 0.0f; // Block further positive tilt
    }
    if (currentRoll < -45.0f && intended_roll_direction < 0.0f) {
        return 0.0f; // Block further negative tilt
    }
    return output; // Allow movement
}

// --- Public Interface ---
void setControlMode(uint8_t mode) {
    current_control_mode = mode;
    if (current_control_mode == MODE_STABILIZED) resetPIDs();
}

uint8_t getControlMode() { return current_control_mode; }

// backward Compatiblity for other code, should be updated once PID is locked down
void setTargetVelocities(uint16_t vx, uint16_t vy, uint16_t omega) {
    target_vx = vx; target_vy = vy; target_omega = omega;
}

void resetPIDs() {
    pitch_integral = 0.0f; pitch_prev_error = 0.0f;
    roll_integral = 0.0f; roll_prev_error = 0.0f;
    
    // NEW: Reset filter states when PID is reset
    pitch_prev_derivative = 0.0f; pitch_prev_output = 0.0f;
    roll_prev_derivative = 0.0f; roll_prev_output = 0.0f;
}

// Maps RC input pulse (1000-2000) to normalized control (-1.0 to 1.0)
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

        // Calculate PID with new safe startup flag AND new filter states
        float outX = calculateAxisPID(pError, pitch_integral, pitch_prev_error, 
                                      pitch_prev_derivative, pitch_prev_output, 
                                      robotSettings.kp_pitch, robotSettings.ki_pitch, robotSettings.kd_pitch, dt, robotSettings.pitch_dir, pid_first_run);
        
        float outY = calculateAxisPID(rError, roll_integral, roll_prev_error, 
                                      roll_prev_derivative, roll_prev_output, 
                                      robotSettings.kp_roll, robotSettings.ki_roll, robotSettings.kd_roll, dt, robotSettings.roll_dir, pid_first_run);

        pid_first_run = false; // Mark that we've completed our safe startup

        outY = applyRollSafety(outY, mRoll, robotSettings.roll_dir);
        
        // Apply outputs directly to motors
        setMotorSpeed(outX, outY);

        DEBUG_PRINTF("DEBUG: IMUs | Main R: %6.1f P: %6.1f Y: %6.1f | Sec R: %6.1f P: %6.1f Y: %6.1f | PID: P_err: %6.1f R_err: %6.1f\n", 
                     mRoll, mPitch, mYaw, sRoll, sPitch, sYaw, pError, rError);
    } else {
        // Direct Mode: Pass through mapped inputs
        pid_first_run = true;
        setMotorSpeed(inputX, inputY);
    }
}

float calculateAxisPID(float error, float& integral, float& prev_error, float& prev_derivative, float& prev_output, float kp, float ki, float kd, float dt, float dir, bool first_run) {
    
    // --- CHANGE 1: Error Deadband ---
    const float DEADBAND = 1.5f; 
    if (abs(error) < robotSettings.deadband) {
        error = 0.0f; 
    }

    // 1. Prevent Derivative Kick on startup
    if (first_run) {
        prev_error = error; 
        prev_derivative = 0.0f; 
        prev_output = 0.0f;     
    }
    
    // 2. Calculate Integral with Anti-Windup
    if (error != 0.0f) {
        integral += error * dt;
    }
    if (ki > 0.0001f) {
        float max_i = 1.0f / ki; 
        if (integral > max_i) integral = max_i;
        if (integral < -max_i) integral = -max_i;
    }
    
    // 3. Calculate Derivative
    float raw_derivative = (error - prev_error) / dt;
    
    // --- CHANGE 2: Low-Pass Filter on Derivative ---
    // Use the dynamically loaded value from NVS memory
    float derivative = (raw_derivative * robotSettings.d_alpha) + (prev_derivative * (1.0f - robotSettings.d_alpha));
    
    // Save states for next loop
    prev_derivative = derivative;
    prev_error = error;
    
    // 4. Sum PID Output
    float raw_output = (kp * error) + (ki * integral) + (kd * derivative);
    
    // --- CHANGE 3: Slew Rate Limiting (Output Smoothing) ---
    // Use the dynamically loaded value from NVS memory
    float smoothed_output = (raw_output * robotSettings.out_alpha) + (prev_output * (1.0f - robotSettings.out_alpha));
    prev_output = smoothed_output;
    
    // 5. Apply direction modifier and hard clamp the output
    smoothed_output *= dir;
    if (smoothed_output > 1.0f) smoothed_output = 1.0f;
    if (smoothed_output < -1.0f) smoothed_output = -1.0f;
    
    return smoothed_output;
}

#endif