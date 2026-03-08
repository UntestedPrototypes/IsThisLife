#ifdef ROLE_ROBOT

#include "motors.h"
#include "../config/robot_config.h"
#include "../config/robot_preferences.h"
#include "../utils/debug.h"
#include "Servo_ST3215.h"
#include <Arduino.h>

// Cleaned up initialization without the old PID/angle arguments
MotorChannel mainMotor(MAIN_MOTOR_PIN, 1000, 2000, 1500, 500, 0, false, 0.10f, 0.10f);
Servo_ST3215 pendServos(1, 2);
const int MAX_ST3215_SPEED = 3400; 

static float current_normVx = 0.0f;
static float current_normVy = 0.0f;

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
        pendServos.setOuterLimits(robotSettings.encoder_limit_min, robotSettings.encoder_limit_max); 
        pendServos.enableMotors();
    }
    return true;
}

void setEncoderLimits(int32_t min_limit, int32_t max_limit) {
    // 1. Update the hardware object limits
    pendServos.setOuterLimits(min_limit, max_limit);
}

void setMotorSpeed(float target_normVx, float target_normVy) {
    
    // 1. Apply smoothing/filtering
    const float alpha = 0.4f; 
    current_normVx = (alpha * target_normVx) + ((1.0f - alpha) * current_normVx);
    current_normVy = (alpha * target_normVy) + ((1.0f - alpha) * current_normVy);

    if (abs(current_normVx - target_normVx) < 0.002f) current_normVx = target_normVx;
    if (abs(current_normVy - target_normVy) < 0.002f) current_normVy = target_normVy;

    // 2. Command Main Motor Hardware (Max 50Hz update rate for standard PWM)
    static uint32_t last_pwm_update = 0;
    uint32_t now = millis();
    if (now - last_pwm_update >= 20) {
        float locked_normVx = round(current_normVx * 50.0f) / 50.0f;
        mainMotor.command(locked_normVx);
        last_pwm_update = now;
    }

    // 3. Command Servo Hardware
    int targetVelocity = (int)(current_normVy * MAX_ST3215_SPEED);
    pendServos.setVelocity(targetVelocity);
}

void stopMotors() { 
    mainMotor.writeNeutral();
    pendServos.setVelocity(0);
    pendServos.stop();
    
    current_normVx = 0.0f;
    current_normVy = 0.0f;
}
// --- MotorChannel Implementation ---
MotorChannel::MotorChannel(uint8_t pin, uint16_t min_us, uint16_t max_us, 
                           uint16_t neutral_us, uint16_t speed_range_us, 
                           uint16_t min_delta_us, bool direction_inverted,
                           float min_fwd, float min_rev)
: _pin(pin), _min_us(min_us), _max_us(max_us), _neutral_us(neutral_us),
  _speed_range_us(speed_range_us), _min_delta_us(min_delta_us),
  _direction_inverted(direction_inverted), _min_fwd(min_fwd), _min_rev(min_rev),
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
    float remappedNorm = 0.0f;

    // Deadzone Skip Logic
    if (controlNorm > 0.001f) {
        remappedNorm = _min_fwd + (controlNorm * (1.0f - _min_fwd));
    } else if (controlNorm < -0.001f) {
        float absNorm = abs(controlNorm);
        remappedNorm = -(_min_rev + (absNorm * (1.0f - _min_rev)));
    }

    int32_t pulse = (int32_t)_neutral_us + (int32_t)(remappedNorm * (float)_speed_range_us);
    
    // Maintain support for legacy hard-cutoff min_delta if needed
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