#ifdef ROLE_ROBOT

#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include <ESP32Servo.h>
#include "vectors.h"

bool initMotors();

// Call this every loop cycle to run the PID
void updateMotorLoop();

// Helper to set the TARGETS
void setTargetVelocities(uint16_t vx_us, uint16_t vy_us, uint16_t omega_us);
void setMotors(uint16_t vx_us, uint16_t vy_us, uint16_t omega_us);

// Called safely by ControlTask to write target states to hardware pins
void executeMotorCommands();
void stopMotors();

// MotorChannel class remains exactly the same below...
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
  uint16_t command(float controlNorm);
  uint16_t writeNeutral();
  float computePID(float targetAngle, float currentAngle, float dt);
  void resetPID();
  bool attached();

private:
  uint16_t computePulse(float controlNorm) const;
  uint16_t clampPulse(int32_t pulse) const;
  uint16_t writeMicroseconds(uint16_t pulse);

  uint8_t  _pin;
  uint16_t _min_us;
  uint16_t _max_us;
  uint16_t _neutral_us;
  uint16_t _speed_range_us;
  uint16_t _min_delta_us;
  bool     _direction_inverted;
  Vec3     _PID;
  float    _angle_range;
  float    _prevError;
  float    _integral;
  Servo    _servo;

  uint16_t _current_pulse;
};

#endif // MOTORS_H
#endif // ROLE_ROBOT