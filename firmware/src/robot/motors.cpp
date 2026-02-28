#ifdef ROLE_ROBOT

#include "motors.h"
#include "robot_config.h"
#include "Servo_ST3215.h"
#include <Arduino.h>

Vec3 default_pid(0.01f, 0.01f, 0.0f); 

// Main Motor: 1000-2000us range, 1500 neutral
MotorChannel mainMotor(MAIN_MOTOR_PIN, 1000, 2000, 1500, 500, 0, false, default_pid, 40.0f);

// ST3215 Servos (replaces servoL and servoR)
Servo_ST3215 pendServos(1, 2);
const int MAX_ST3215_SPEED = 2400; // Adjust this to match your desired max velocity

bool initMotors() {
    Serial.println("DEBUG: Initializing Motors...");
    
    if (!mainMotor.begin()) {
        Serial.println("ERROR: Failed to initialize Main Motor!");
        return false;
    }

    // Initialize Serial1 for the ST3215 servos (using pins from robot_config.h)
    if (!pendServos.begin(Serial1, SERVO_RX_PIN, SERVO_TX_PIN)) {
        Serial.println("ERROR: Failed to init ST3215 Servos!");
        return false;
    } else {
        pendServos.enableMotors();
    }
    return true;
}

// Helper to map 1000-2000us to -1.0 to 1.0
float mapRcInput(uint16_t input_us) {
    // Clamp for safety
    if (input_us < 1000) input_us = 1000;
    if (input_us > 2000) input_us = 2000;
    
    // Center at 1500, range +/- 500
    return (float)(input_us - 1500) / 500.0f;
}

void setMotors(uint16_t vx_us, uint16_t vy_us, uint16_t omega_us) {
    // 1. Normalize RC inputs (1000-2000) to Float (-1.0 to 1.0)
    float normVx = mapRcInput(vx_us);
    float normVy = mapRcInput(vy_us);
    // float normOmega = mapRcInput(omega_us); // Ignored: ST3215 library syncs both motors

    // 2. Apply Kinematics
    mainMotor.command(normVx);

    // 3. Command the ST3215 Servos
    int targetVelocity = (int)(normVy * MAX_ST3215_SPEED);
    pendServos.setVelocity(targetVelocity);
}

void stopMotors() { 
    mainMotor.writeNeutral();
    pendServos.stop();
}

// Ensure this is called in roleLoop() so the library can track encoder wraps!
void updateMotorLoop() {
    //pendServos.update();
}

// --- MotorChannel Implementation (Unchanged, for Main Motor) ---
MotorChannel::MotorChannel(uint8_t pin, uint16_t min_us, uint16_t max_us, 
                           uint16_t neutral_us, uint16_t speed_range_us, 
                           uint16_t min_delta_us, bool direction_inverted, 
                           Vec3 PID, float angle_range)
: _pin(pin), _min_us(min_us), _max_us(max_us), _neutral_us(neutral_us),
  _speed_range_us(speed_range_us), _min_delta_us(min_delta_us),
  _direction_inverted(direction_inverted), _PID(PID), _angle_range(angle_range)
{}

bool MotorChannel::begin() {
    int r = _servo.attach(_pin, _min_us, _max_us);

    if (!attached()) {
        Serial.printf("ERROR: Motor on pin %d failed to attach!\n", _pin);
    }

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
    if (_servo.attached()) _servo.writeMicroseconds(pulse);
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