#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ESP32Servo.h>
#include <angles.h>
#include <vectors.h>
// ----------------- Class definition -----------------
class motorChannel {
public:
  uint8_t pin;              // Arduino pin number
  unsigned int min_us;      // minimum pulse width in microseconds
  unsigned int max_us;      // maximum pulse width in microseconds
  unsigned int neutral_us;  // neutral (center) pulse width
  unsigned int speed_range_us; // range around neutral
  unsigned int min_delta_us; // minimum change in pulse width to register movement
  bool direction_inverted;  // direction inversion flag
  bool enabled;          // motor enabled flag
  float pidIntegral;     // integral term for PID control
  float lastError;       // last error for derivative term

  // Constructor 
  motorChannel(uint8_t pin_,
         unsigned int min_us_,
         unsigned int max_us_,
         unsigned int neutral_us_,
         unsigned int speed_range_us_,
         unsigned int min_delta_us_ = 0,
         bool direction_inverted_ = false)
    : pin(pin_),
      min_us(min_us_),
      max_us(max_us_),
      neutral_us(neutral_us_),
      speed_range_us(speed_range_us_),
      min_delta_us(min_delta_us_),
      direction_inverted(direction_inverted_) {
      };
};



// ----------------- Configuration -----------------
// I2C pins 
#define SDA_PIN 21
#define SCL_PIN 22
// Motor control pins
#define MAIN_MOTOR_PIN 27
#define SERVO_L_PIN 25
#define SERVO_R_PIN 33
// PID
float Kp = 0.8f;
float Ki = 0.05f;
float Kd = 0.1f;

// When error is smaller than this, we stop the motor and zero integral
const float HOLD_DEADBAND_DEG   = 0.7f;

// Only integrate when |error| is smaller than this
const float INTEGRAL_ZONE_DEG   = 15.0f;

// Integral clamp
const float INTEGRAL_LIMIT      = 80.0f;


// Motor configuration
motorChannel mainMotor(MAIN_MOTOR_PIN, 1000, 2000, 1500, 500, 0, false); // ESC connected to pin 27
motorChannel servoL(SERVO_L_PIN, 500, 2500, 1500, 500,75, false); // TD-8120MG 360° Left
motorChannel servoR(SERVO_R_PIN, 500, 2500, 1500, 500,75, true);    // TD-8120MG 360° Right

// BNO055 IMU configuration
Adafruit_BNO055 IMU1 = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 IMU2 = Adafruit_BNO055(56, 0x29); 


// ----------------- Global variables -----------------
float desiredPitch = 0.0f; // desired pitch angle in degrees
float desiredRoll  = 0.0f; // desired roll angle in degrees

// ----------------- Functions -----------------
float wrapAngle(float angle) {
  /*
  Wrap angle to [-180, 180]
  */
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}


int commandServo(float controlNorm, motorChannel motor) {
  /* 
  Convert normalized control [-1, 1] to pulse width for given motor channel 

  Inputs:
  controlNorm: normalized control input [-1, 1]
  motor: motorChannel object with configuration parameters

  Returns:
  pulse width in microseconds  
  */

  // Optional direction inversion
  if (motor.direction_inverted) {
    controlNorm = -controlNorm;
  }

  // Clamp to [-1, 1]
  if (controlNorm > 1.0f) controlNorm = 1.0f;
  if (controlNorm < -1.0f) controlNorm = -1.0f;

  int pulse = motor.neutral_us + (int)(controlNorm * motor.speed_range_us);

  // Respect servo limits

  if (motor.enabled == false) {
    pulse = motor.neutral_us; // if motor disabled, send neutral signal
  }
  else if (motor.enabled == true) {
    int pulse = motor.neutral_us + (int)(controlNorm * motor.speed_range_us);
  }
  else {
    // invalid enabled flag, set to neutral
    pulse = motor.neutral_us;
  }

  // if outside min/max, clamp to min/max
  if (pulse < motor.min_us) pulse = motor.min_us;
  if (pulse > motor.max_us) pulse = motor.max_us;

  // If within deadband, set to min_delta_us away from neutral
  if (pulse > (motor.neutral_us - motor.min_delta_us) &&
      pulse < (motor.neutral_us + motor.min_delta_us)) {
      if (pulse > motor.neutral_us- motor.min_delta_us) pulse = motor.neutral_us- motor.min_delta_us; 
      else if (pulse < motor.neutral_us + motor.min_delta_us) pulse = motor.neutral_us + motor.min_delta_us;
      else pulse = motor.neutral_us;
  }


  return pulse;
}

bool readImuQuaternion(Adafruit_BNO055 &imu, Quaternion &out)
{
    // Adafruit's quaternion type (from the library)
    imu::Quaternion q = imu.getQuat();

    out.w = q.w();
    out.x = q.x();
    out.y = q.y();
    out.z = q.z();
    return true;
}



void handleSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("r")) {
    // zeroIMU();
    desiredPitch = 0.0f;
    desiredRoll  = 0.0f;
    Serial.println("IMU re-zeroed, desired pitch = 0 deg");
  }
  else if (line.equalsIgnoreCase("s")) {
    // Toggle servo enabled/disabled
    servoL.enabled = !servoL.enabled;
    servoR.enabled = !servoR.enabled;

    if (servoL.enabled && servoR.enabled) {
      // Re-enable control
      servoL.pidIntegral = 0.0f;
      servoL.lastError   = 0.0f;
      servoR.pidIntegral = 0.0f;
      servoR.lastError   = 0.0f;

      Serial.println("Servo ENABLED (closed-loop control).");

    } else {

      // Turn "off" control: stop servo and reset PID terms
      servoL.pidIntegral = 0.0f;
      servoL.lastError   = 0.0f;

      servoR.pidIntegral = 0.0f;
      servoR.lastError   = 0.0f;

      commandServo(0.0f, servoR);  // neutral pulse
      commandServo(0.0f, servoL);  // neutral pulse
      
      Serial.println("Servo DISABLED (neutral, control off).");
    }
  } 
  else if (line.equalsIgnoreCase("m")) {
    // Toggle main motor enabled/disabled
    mainMotor.enabled = !mainMotor.enabled;

    if (mainMotor.enabled) {
      // Re-enable control
      mainMotor.pidIntegral = 0.0f;
      mainMotor.lastError   = 0.0f;
      
      Serial.println("Main motor ENABLED (closed-loop control).");
    } else {

      // Turn "off" control: stop servo and reset PID terms
      mainMotor.pidIntegral = 0.0f;
      mainMotor.lastError   = 0.0f;

      commandServo(0.0f, mainMotor);  // neutral pulse

      Serial.println("Main motor DISABLED (neutral, control off).");
    }
    }
    else {
    // Check if input is two numbers separated by a comma
    int commaIndex = line.indexOf(',');
    if (commaIndex != -1) {
      String firstPart = line.substring(0, commaIndex);
      String secondPart = line.substring(commaIndex + 1);
      if (firstPart.length() > 0 && secondPart.length() > 0 && 
        firstPart.toFloat() != 0.0f && secondPart.toFloat() != 0.0f) {
      float value1 = firstPart.toFloat();
      float value2 = secondPart.toFloat();
      Serial.print("Received values: ");
      Serial.print(value1);
      Serial.print(", ");
      Serial.println(value2);
      } else {
      Serial.println("Invalid input. Both parts must be valid numbers.");
      }
    } else {
      Serial.println("Invalid input. Please provide two numbers separated by a comma.");
    }
    }
  }



void setup() {



}
