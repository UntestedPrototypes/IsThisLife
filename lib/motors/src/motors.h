#pragma once

#include <Arduino.h>
#include <math.h>
#include <ESP32Servo.h>
#include "vectors.h"   // Vec3

class MotorChannel {
public:
  MotorChannel(uint8_t pin,
               uint16_t min_us,
               uint16_t max_us,
               uint16_t neutral_us,
               uint16_t speed_range_us,
               uint16_t min_delta_us = 0,
               bool direction_inverted = false,
               Vec3 PID = {0.0f, 0.0f, 0.0f},
               float angle_range = 0.0f);

  bool begin();
  void end();
  bool attached();

  void setEnabled(bool en);
  bool enabled() const;

  void setDirectionInverted(bool inv);
  bool directionInverted() const;

  void setLimits(uint16_t min_us, uint16_t max_us);
  void setNeutral(uint16_t neutral_us);
  void setSpeedRange(uint16_t speed_range_us);
  void setDeadband(uint16_t min_delta_us);

  // Angle range (same unit as measuredAngle: degrees or radians)
  void setAngleRange(float angle_range);
  float angleRange() const;

  // Normalized command in [-1..1]
  uint16_t command(float controlNorm);

  // Angle command with PID:
  // - deadzone: |commandNorm| < 0.1 => 0
  // - targetAngle = commandNorm * angle_range
  // - PID drives measuredAngle -> targetAngle
  uint16_t commandAngle(float commandNorm, float measuredAngle);

  uint16_t writeNeutral();
  uint16_t writeMicroseconds(uint16_t pulse);

  // Getters
  uint8_t  pin() const;
  uint16_t minUs() const;
  uint16_t maxUs() const;
  uint16_t neutralUs() const;
  uint16_t speedRangeUs() const;
  uint16_t deadbandUs() const;
  uint16_t lastPulse() const;

private:
  uint16_t computePulse(float controlNorm) const;
  uint16_t clampPulse(int32_t pulse) const;

  uint8_t  _pin;
  uint16_t _min_us;
  uint16_t _max_us;
  uint16_t _neutral_us;
  uint16_t _speed_range_us;
  uint16_t _min_delta_us;
  bool     _direction_inverted;
  bool     _enabled;
  uint16_t _last_pulse;

  float    _angle_range;

  Vec3     _PID;          // x=Kp, y=Ki, z=Kd
  float    _i_accum;      // integral accumulator (error * seconds)
  float    _prev_error;   // previous error
  uint32_t _prev_us;      // previous micros timestamp
  bool     _pid_init;

  Servo    _servo;        // from ESP32Servo
};
