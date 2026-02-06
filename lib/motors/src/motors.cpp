#include "motors.h"
#include "communication.h"

MotorChannel::MotorChannel(uint8_t pin,
                           uint16_t min_us,
                           uint16_t max_us,
                           uint16_t neutral_us,
                           uint16_t speed_range_us,
                           uint16_t min_delta_us,
                           bool direction_inverted,
                           Vec3 PID,
                           float angle_range)
: _pin(pin),
  _min_us(min_us),
  _max_us(max_us),
  _neutral_us(neutral_us),
  _speed_range_us(speed_range_us),
  _min_delta_us(min_delta_us),
  _direction_inverted(direction_inverted),
  _enabled(true),
  _last_pulse(neutral_us),
  _angle_range(angle_range),
  _PID(PID),
  _i_accum(0.0f),
  _prev_error(0.0f),
  _prev_us(0),
  _pid_init(false),
  _estimated_angle_deg(0.0f),
  _last_update_us(0),
  _failsafe_enabled(false)
{}

bool MotorChannel::begin() {
  // Optional (ESP32Servo supports it): set refresh rate
  // _servo.setPeriodHertz(50);

  int r = _servo.attach(_pin, _min_us, _max_us);
  writeNeutral();
  return (r != 0);
}

void MotorChannel::end() {
  if (_servo.attached()) _servo.detach();
}

bool MotorChannel::attached() {
  return _servo.attached();
}

void MotorChannel::setEnabled(bool en) {
  _enabled = en;
  if (!en) {
    writeNeutral();
    _i_accum = 0.0f;
    _prev_error = 0.0f;
    _pid_init = false;
  }
}

bool MotorChannel::enabled() const {
  return _enabled;
}

void MotorChannel::setDirectionInverted(bool inv) {
  _direction_inverted = inv;
}

bool MotorChannel::directionInverted() const {
  return _direction_inverted;
}

void MotorChannel::setLimits(uint16_t min_us, uint16_t max_us) {
  _min_us = min_us;
  _max_us = max_us;
  if (_servo.attached()) {
    _servo.detach();
    _servo.attach(_pin, _min_us, _max_us);
    writeNeutral();
  }
}

void MotorChannel::setNeutral(uint16_t neutral_us) {
  _neutral_us = neutral_us;
  _last_pulse = neutral_us;
}

void MotorChannel::setSpeedRange(uint16_t speed_range_us) {
  _speed_range_us = speed_range_us;
}

void MotorChannel::setDeadband(uint16_t min_delta_us) {
  _min_delta_us = min_delta_us;
}

void MotorChannel::setAngleRange(float angle_range) {
  _angle_range = angle_range;
}

float MotorChannel::angleRange() const {
  return _angle_range;
}

uint16_t MotorChannel::command(float controlNorm) {
  if (!_enabled) return writeNeutral();

  if (_direction_inverted) controlNorm = -controlNorm;

  if (controlNorm > 1.0f)  controlNorm = 1.0f;
  if (controlNorm < -1.0f) controlNorm = -1.0f;

  // --- FAIL-SAFE ANGLE ESTIMATION ---
  if (_failsafe_enabled) {
    const uint32_t now_us = micros();
    float dt = 0.0f;

    if (_last_update_us != 0) {
      const uint32_t du = now_us - _last_update_us;
      dt = (du > 0) ? (du * 1e-6f) : 0.0f;
    }
    _last_update_us = now_us;

    // Approximate degrees/sec at full command
    const float MAX_DEG_PER_SEC = 90.0f;  // conservative estimate
    float omega = controlNorm * MAX_DEG_PER_SEC;

    _estimated_angle_deg += omega * dt;

    // Enforce hard limits
    if (_estimated_angle_deg > 45.0f || _estimated_angle_deg < -45.0f) {
      _estimated_angle_deg = constrain(_estimated_angle_deg, -45.0f, 45.0f);
      return writeNeutral();  // STOP MOTOR IMMEDIATELY
    }
  }

  uint16_t pulse = computePulse(controlNorm);
  return writeMicroseconds(pulse);
}

uint16_t MotorChannel::commandAngle(float commandNorm, float measuredAngle) {
  if (!_enabled) {
    _i_accum = 0.0f;
    _prev_error = 0.0f;
    _pid_init = false;
    return writeNeutral();
  }

  // ignore small commands
  if (fabsf(commandNorm) < 0.2f) commandNorm = 0.0f;

  // clamp + invert
  if (commandNorm > 1.0f)  commandNorm = 1.0f;
  if (commandNorm < -1.0f) commandNorm = -1.0f;


  // map to target angle
  const float targetAngle = commandNorm * _angle_range;
  //sendToLaptop("Target Angle: " + String(targetAngle) + ", Measured Angle: " + String(measuredAngle));
  // dt
  const uint32_t now_us = micros();
  float dt = 0.0f;

  if (!_pid_init) {
    _pid_init = true;
    _prev_us = now_us;
    _prev_error = targetAngle - measuredAngle;
    _i_accum = 0.0f;
    dt = 0.0f;
  } else {
    const uint32_t du = now_us - _prev_us;
    _prev_us = now_us;
    dt = (du > 0) ? (du * 1e-6f) : 0.0f;
  }

  const float error = targetAngle - measuredAngle;

  // integral with basic anti-windup so Ki*I stays ~[-1..1]
  if (dt > 0.0f && _PID.y != 0.0f) {
    _i_accum += error * dt;
    const float i_limit = 1.0f / fabsf(_PID.y);
    if (_i_accum >  i_limit) _i_accum =  i_limit;
    if (_i_accum < -i_limit) _i_accum = -i_limit;
  }

  float dterm = 0.0f;
  if (dt > 0.0f) dterm = (error - _prev_error) / dt;
  _prev_error = error;

  float u = (_PID.x * error) + (_PID.y * _i_accum) + (_PID.z * dterm);

  if (u > 1.0f)  u = 1.0f;
  if (u < -1.0f) u = -1.0f;

  // Prevent PID from fighting the failsafe
  if (_failsafe_enabled) {
    if ((_estimated_angle_deg >= 45.0f && u > 0.0f) ||
        (_estimated_angle_deg <= -45.0f && u < 0.0f)) {
      return writeNeutral();
    }
  }

  return command(u);
}

uint16_t MotorChannel::writeNeutral() {
  return writeMicroseconds(_neutral_us);
}

uint16_t MotorChannel::writeMicroseconds(uint16_t pulse) {
  _last_pulse = pulse;
  if (_servo.attached()) _servo.writeMicroseconds(pulse);
  return pulse;
}

void MotorChannel::resetAngleEstimate() {
  _estimated_angle_deg = 0.0f;
  _last_update_us = micros();
}

void MotorChannel::enableFailsafe(bool enable) {
  _failsafe_enabled = enable;
}

float MotorChannel::estimatedAngle() const {
  return _estimated_angle_deg;
}

uint16_t MotorChannel::computePulse(float controlNorm) const {
  int32_t pulse = (int32_t)_neutral_us + (int32_t)(controlNorm * (float)_speed_range_us);

  int32_t delta = pulse - (int32_t)_neutral_us;
  if (delta < 0) delta = -delta;
  if ((uint32_t)delta < (uint32_t)_min_delta_us) {
    pulse = (int32_t)_neutral_us;
  }

  return clampPulse(pulse);
}

uint16_t MotorChannel::clampPulse(int32_t pulse) const {
  if (pulse < (int32_t)_min_us) pulse = (int32_t)_min_us;
  if (pulse > (int32_t)_max_us) pulse = (int32_t)_max_us;
  return (uint16_t)pulse;
}

// Getters
uint8_t  MotorChannel::pin() const { return _pin; }
uint16_t MotorChannel::minUs() const { return _min_us; }
uint16_t MotorChannel::maxUs() const { return _max_us; }
uint16_t MotorChannel::neutralUs() const { return _neutral_us; }
uint16_t MotorChannel::speedRangeUs() const { return _speed_range_us; }
uint16_t MotorChannel::deadbandUs() const { return _min_delta_us; }
uint16_t MotorChannel::lastPulse() const { return _last_pulse; }
