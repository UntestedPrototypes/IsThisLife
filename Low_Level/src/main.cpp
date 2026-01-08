#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ESP32Servo.h>
#include <angles.h>
#include <vectors.h>
#include <IMUaxisCalibration.h>
#include <IMU.h>

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
  Vec3 pid;              // PID coefficients (Kp, Ki, Kd)
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
         bool direction_inverted_ = false,
         Vec3 pid_ = Vec3(0.0f, 0.0f, 0.0f)
    ) : pin(pin_),
      min_us(min_us_),
      max_us(max_us_),
      neutral_us(neutral_us_),
      speed_range_us(speed_range_us_),
      min_delta_us(min_delta_us_),
      direction_inverted(direction_inverted_),
      pid(pid_) {
      };
};


Vec3 zeroAngle;
// ----------------- Configuration -----------------
// I2C pins 
#define SDA_PIN 21
#define SCL_PIN 22
#define IMU1_rst 16
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
motorChannel mainMotor(MAIN_MOTOR_PIN, 1000, 2000, 1500, 500, 0, false, Vec3(Kp, Ki, Kd)); // ESC connected to pin 27
motorChannel servoL(SERVO_L_PIN, 500, 2500, 1500, 500,75, false, Vec3(Kp, Ki, Kd)); // TD-8125MG 360° Left
motorChannel servoR(SERVO_R_PIN, 500, 2500, 1500, 500,75, true, Vec3(Kp, Ki, Kd));    // TD-8125MG 360° Right

// BNO055 IMU configuration
IMU imu1("IMU1", 0x28, 16);




// ----------------- Global variables -----------------
// Latest angles (degrees)
volatile float phi_deg   = 0.0f;  // roll
volatile float theta_deg = 0.0f;  // pitch
volatile float psi_deg   = 0.0f;  // yaw/heading


// ----------------- Functions -----------------


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

void handlePID(motorChannel &motor, float targetAngleDeg, float currentAngleDeg, float dtSec=1/100.0f) {
  /*
  Update motor PID control based on target and current angles

  Inputs:
  motor: reference to motorChannel object
  targetAngleDeg: desired angle in degrees
  currentAngleDeg: current angle in degrees
  dtSec: time step in seconds
  */

  float error = targetAngleDeg - currentAngleDeg;

  // Wrap error to [-180, 180]
  error = atan2f(sinf(error * M_PI / 180.0f), cosf(error * M_PI / 180.0f)) * 180.0f / M_PI;

  // Proportional term
  float P = motor.pid.x * error;

  // Integral term
  if (fabsf(error) < INTEGRAL_ZONE_DEG) {
    motor.pidIntegral += error * dtSec;
    // Clamp integral
    if (motor.pidIntegral > INTEGRAL_LIMIT) motor.pidIntegral = INTEGRAL_LIMIT;
    if (motor.pidIntegral < -INTEGRAL_LIMIT) motor.pidIntegral = -INTEGRAL_LIMIT;
  } else {
    motor.pidIntegral = 0.0f; // reset integral outside zone
  }
  float I = motor.pid.y * motor.pidIntegral;

  // Derivative term
  float derivative = (error - motor.lastError) / dtSec;
  float D = motor.pid.z * derivative;

  // Compute total output
  float output = P + I + D;

  // Update last error
  motor.lastError = error;

  // Command motor
  int pulseUs = commandServo(output, motor); 
}



void handleSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("r")) {
    // IMU_reset();
  }
}



void setup()
{

  Serial.begin(115200);
  delay(1000);
  

  printf("Serial started\n");
  // ---------------- I2C ----------------
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(50000);
  delay(1000);
  printf("I2C started\n");
  // ---------------- IMU1 startup ----------------
  while (!imu1.begin()) {
        Serial.println("IMU init failed");
        delay(5000);
  }
  delay(500);
  imu1.startCalibration();

  while (!imu1.isCalibrated()) {
    imu1.update();
    delay(10);
  }





  Serial.println("IMU1 ready.");

  // ---------------- Init control structs ----------------
  mainMotor.enabled     = false;
  mainMotor.pidIntegral = 0.0f;
  mainMotor.lastError   = 0.0f;

  servoL.enabled        = false;
  servoL.pidIntegral    = 0.0f;
  servoL.lastError      = 0.0f;

  servoR.enabled        = false;
  servoR.pidIntegral    = 0.0f;
  servoR.lastError      = 0.0f;



 if (imu1.isCalibrated()) {
        Vec3 rod  = imu1.getRodAxisIMU();
        Vec3 plane = imu1.getPlaneRefIMU();

        Serial.println("=== IMU CALIBRATION RESULT ===");

        Serial.print("rodAxisIMU   = [ ");
        Serial.print(rod.x, 6); Serial.print(", ");
        Serial.print(rod.y, 6); Serial.print(", ");
        Serial.print(rod.z, 6); Serial.println(" ]");

        Serial.print("planeRefIMU  = [ ");
        Serial.print(plane.x, 6); Serial.print(", ");
        Serial.print(plane.y, 6); Serial.print(", ");
        Serial.print(plane.z, 6); Serial.println(" ]");

        float dot = vecDot(rod, plane);
        Serial.print("dot(rod, plane) = ");
        Serial.println(dot, 6);

        Serial.println("==============================");

        delay(5000);
    }
}


void loop() {
   

handleSerial();
static uint32_t lastReadUs  = 0;
static uint32_t lastPrintUs = 0;

const uint32_t READ_PERIOD_US  = 10000;  // 100 Hz
const uint32_t PRINT_PERIOD_US = 1000000;  // 1 Hz

uint32_t nowUs = micros();

/* ---------- IMU update ---------- */
if ((uint32_t)(nowUs - lastReadUs) >= READ_PERIOD_US) {
    lastReadUs += READ_PERIOD_US;
    imu1.update();
}

/* ---------- Compute + PRINT angles ---------- */
if ((uint32_t)(nowUs - lastPrintUs) >= PRINT_PERIOD_US) {
    lastPrintUs += PRINT_PERIOD_US;

    if (!imu1.isCalibrated()) {
        Serial.println("IMU not calibrated");
        return;
    }

    Quaternion q = imu1.getQuaternion();
    Vec3 rod_b   = imu1.getRodAxisIMU();
    Vec3 plane_b = imu1.getPlaneRefIMU();

    // --- rotate body axes into world frame ---
    Vec3 rod_w   = quatRotate(q, rod_b);
    Vec3 plane_w = quatRotate(q, plane_b);

    // θ: tilt from vertical (BNO055 Z axis points DOWN)
    float theta = acosf(-rod_w.z);

    // φ: azimuth of rod projection (BNO055 convention)
    float phi = atan2f(rod_w.x, rod_w.y);

    // ψ: spin about rod axis (use BNO055 vertical)
    Vec3 z_w(0.0f, 0.0f, -1.0f);


    Vec3 ref_w =
        vecNormalize(z_w - vecDot(z_w, rod_w) * rod_w);

    Vec3 plane_proj =
        vecNormalize(plane_w - vecDot(plane_w, rod_w) * rod_w);

    float psi = atan2f(
        vecDot(rod_w, cross(ref_w, plane_proj)),
        vecDot(ref_w, plane_proj)
    );

    // --- PRINT ---
    Serial.print("phi=");
    Serial.print(phi * 180.0f / M_PI, 3);
    Serial.print(", theta=");
    Serial.print(theta * 180.0f / M_PI, 3);
    Serial.print(", psi=");
    Serial.println(psi * 180.0f / M_PI, 3);
}


}

