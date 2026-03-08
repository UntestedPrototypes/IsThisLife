#ifdef ROLE_ROBOT

#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include <ESP32Servo.h>

bool initMotors();

void setMotorSpeed(float target_normVx, float target_normVy);
void stopMotors();

void setEncoderLimits(int32_t min_limit, int32_t max_limit);

class MotorChannel {
public:
  MotorChannel(uint8_t pin,
               uint16_t min_us,
               uint16_t max_us,
               uint16_t neutral_us,
               uint16_t speed_range_us,
               uint16_t min_delta_us = 0,
               bool direction_inverted = false,
               float min_fwd = 0.0f,    // Deadzone skip for forward (0.0 - 1.0)
               float min_rev = 0.0f);   // Deadzone skip for reverse (0.0 - 1.0)

  bool begin();
  uint16_t command(float controlNorm);
  uint16_t writeNeutral();
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
  float    _min_fwd; // Internal storage for deadzone skip
  float    _min_rev;
  Servo    _servo;

  uint16_t _current_pulse;
};
#endif // MOTORS_H
#endif // ROLE_ROBOT