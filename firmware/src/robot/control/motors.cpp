#ifdef ROLE_ROBOT

#include "motors.h"
#include "../config/robot_config.h"
#include "../config/robot_preferences.h"
#include "Servo_ST3215.h"
#include <Arduino.h>

Vec3 default_pid(0.01f, 0.01f, 0.0f); 

MotorChannel mainMotor(MAIN_MOTOR_PIN, 1000, 2000, 1500, 500, 0, false, default_pid, 40.0f);
Servo_ST3215 pendServos(1, 2);
const int MAX_ST3215_SPEED = 3400; 

// Storage for target velocities asynchronously set by SystemTask
static uint16_t target_vx = 1500;
static uint16_t target_vy = 1500;
static uint16_t target_omega = 1500;

bool initMotors() {
    Serial.println("DEBUG: Initializing Motors...");
    if (!mainMotor.begin()) {
        Serial.println("ERROR: Failed to initialize Main Motor!");
        return false;
    }
    if (!pendServos.begin(Serial1, SERVO_RX_PIN, SERVO_TX_PIN)) {
        Serial.println("ERROR: Failed to init ST3215 Servos!");
        return false;
    } else {
        // --- MODIFIED: Use the preferences instead of hardcoded numbers ---
        pendServos.setOuterLimits(robotSettings.encoder_limit_min, robotSettings.encoder_limit_max); 
        pendServos.enableMotors();
    }
    return true;
}

float mapRcInput(uint16_t input_us) {
    if (input_us < 1000) input_us = 1000;
    if (input_us > 2000) input_us = 2000;
    return (float)(input_us - 1500) / 500.0f;
}

void setTargetVelocities(uint16_t vx_us, uint16_t vy_us, uint16_t omega_us) {
    target_vx = vx_us;
    target_vy = vy_us;
    target_omega = omega_us;
}

void setMotors(uint16_t vx_us, uint16_t vy_us, uint16_t omega_us) {
    setTargetVelocities(vx_us, vy_us, omega_us);
}

// Strictly called by ControlTask on Core 1 to apply targets safely
static float current_normVx = 0.0f;
static float current_normVy = 0.0f;

void executeMotorCommands() {
    float target_normVx = mapRcInput(target_vx);
    float target_normVy = mapRcInput(target_vy);

    // Exponential Moving Average Filter
    const float alpha = 0.4f; 
    current_normVx = (alpha * target_normVx) + ((1.0f - alpha) * current_normVx);
    current_normVy = (alpha * target_normVy) + ((1.0f - alpha) * current_normVy);

    // Asymptote Snapping to prevent float micro-jitter
    if (abs(current_normVx - target_normVx) < 0.002f) current_normVx = target_normVx;
    if (abs(current_normVy - target_normVy) < 0.002f) current_normVy = target_normVy;

    // ==============================================================
    // NEW: 50Hz PWM Hardware Limiter
    // ==============================================================
    // Only update the physical PWM pin every 20ms (50Hz) to prevent 
    // chopping the ESC's waveform, while allowing the RTOS to run at 100Hz.
    static uint32_t last_pwm_update = 0;
    uint32_t now = millis();
    if (now - last_pwm_update >= 20) {
        float locked_normVx = round(target_normVx * 50.0f) / 50.0f;
        mainMotor.command(locked_normVx);
        last_pwm_update = now;
    }

    // ==============================================================
    // ST3215 Servos (Digital UART)
    // ==============================================================
    // These take digital serial commands, so they can easily handle 
    // the full 100Hz update rate without glitching.
    int targetVelocity = (int)(current_normVy * MAX_ST3215_SPEED);
    pendServos.setVelocity(targetVelocity);
}

void stopMotors() { 
    mainMotor.writeNeutral();
    pendServos.setVelocity(0);
    pendServos.stop();
    
    // Reset smoothers so it doesn't "ramp up" from an old value when re-enabled
    current_normVx = 0.0f;
    current_normVy = 0.0f;
}

void updateMotorLoop() {
    //pendServos.update();
}

// --- MotorChannel Implementation (Unchanged) ---
MotorChannel::MotorChannel(uint8_t pin, uint16_t min_us, uint16_t max_us, 
                           uint16_t neutral_us, uint16_t speed_range_us, 
                           uint16_t min_delta_us, bool direction_inverted, 
                           Vec3 PID, float angle_range)
: _pin(pin), _min_us(min_us), _max_us(max_us), _neutral_us(neutral_us),
  _speed_range_us(speed_range_us), _min_delta_us(min_delta_us),
  _direction_inverted(direction_inverted), _PID(PID), _angle_range(angle_range),
  _current_pulse(0)
{}

bool MotorChannel::begin() {
    int r = _servo.attach(_pin, _min_us, _max_us);
    if (!attached()) Serial.printf("ERROR: Motor on pin %d failed to attach!\n", _pin);
    writeNeutral();
    return attached();
}

bool MotorChannel::attached() { return _servo.attached(); }

uint16_t MotorChannel::command(float controlNorm) {
    if (_direction_inverted) controlNorm = -controlNorm;
    if (controlNorm > 1.0f)  controlNorm = 1.0f;
    if (controlNorm < -1.0f) controlNorm = -1.0f;
    return writeMicroseconds(computePulse(controlNorm));
}

uint16_t MotorChannel::writeNeutral() { return writeMicroseconds(_neutral_us); }

uint16_t MotorChannel::writeMicroseconds(uint16_t pulse) {
    if (attached()) {
        _servo.writeMicroseconds(pulse);
        _current_pulse = pulse;
    }
    
    return pulse;
}

uint16_t MotorChannel::computePulse(float controlNorm) const {
    int32_t pulse = (int32_t)_neutral_us + (int32_t)(controlNorm * (float)_speed_range_us);
    int32_t delta = abs(pulse - (int32_t)_neutral_us);
    if ((uint32_t)delta < (uint32_t)_min_delta_us) pulse = _neutral_us;
    return clampPulse(pulse);
}

uint16_t MotorChannel::clampPulse(int32_t pulse) const {
    if (pulse < (int32_t)_min_us) pulse = _min_us;
    if (pulse > (int32_t)_max_us) pulse = _max_us;
    return (uint16_t)pulse;
}

#endif // ROLE_ROBOT