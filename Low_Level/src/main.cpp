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
motorChannel mainMotor(MAIN_MOTOR_PIN, 1000, 2000, 1500, 500, 0, false); // ESC connected to pin 27
motorChannel servoL(SERVO_L_PIN, 500, 2500, 1500, 500,75, false); // TD-8120MG 360° Left
motorChannel servoR(SERVO_R_PIN, 500, 2500, 1500, 500,75, true);    // TD-8120MG 360° Right

// BNO055 IMU configuration
IMU imu1(1, 0x28, 16);

// Adafruit_BNO055 IMU1 = Adafruit_BNO055(55, 0x28);
// IMUaxisCalibration IMU1Cal;

// Adafruit_BNO055 IMU2 = Adafruit_BNO055(55, 0x29); 
// IMUaxisCalibration IMU2Cal;


// ----------------- Global variables -----------------
// Latest angles (degrees)
volatile float phi_deg   = 0.0f;  // roll
volatile float theta_deg = 0.0f;  // pitch
volatile float psi_deg   = 0.0f;  // yaw/heading


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



void IMU_reset() {
  Vec3 sum{};
  for (int i=0; i<50; i++) {
    imu::Vector<3> e = IMU1.getVector(Adafruit_BNO055::VECTOR_EULER);
    sum = e + sum;
  }
  zeroAngle = sum/50;

}  


void handleSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("r")) {
    IMU_reset();
  }
}



void setup()
{
  Serial.begin(115200);
  delay(1000);
  


  // ---------------- I2C ----------------
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(50);

  // ---------------- IMU1 startup ----------------
  if (!imu1.begin()) {
        Serial.println("IMU init failed");
        while (1);
  }


//   pinMode(IMU1_rst, OUTPUT);

//   digitalWrite(IMU1_rst, LOW);   // assert reset
//   delay(20);                     // >10 ms

//   digitalWrite(IMU1_rst, HIGH);  // release reset
//   delay(700);                    // allow full boot

//   if (!IMU1.begin())
//   {
//     Serial.println("ERROR: BNO055 IMU1 not detected (0x28).");
//     while (true) { delay(10); }
//   }

//   delay(50);
//   IMU1.setExtCrystalUse(true);
//   delay(10);

//   IMU1Cal.begin();

// while (IMU1Cal.calibrating()) {

//     imu::Vector<3> g = IMU1.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

//     Vec3 gyro = {
//         g.x(), g.y(), g.z()
//     };

//     IMU1Cal.update(gyro);
//     delay(10);
// }


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

  Serial.println("test1.");

  // ---------------- BNO055 begin ----------------

}


void loop() {
  handleSerial();
  static uint32_t lastReadUs  = 0;
  static uint32_t lastPrintUs = 0;

  const uint32_t READ_PERIOD_US  = 100;    // requested (10 kHz read schedule)
  const uint32_t PRINT_PERIOD_US = 10000;  // 100 Hz print (serial-safe)

  uint32_t nowUs = micros();

  // Read IMU1 every 100 us
  if ((uint32_t)(nowUs - lastReadUs) >= READ_PERIOD_US) {
    lastReadUs += READ_PERIOD_US;

    imu::Vector<3> e = IMU1.getVector(Adafruit_BNO055::VECTOR_EULER);
    e = e - zeroAngle;

    // Adafruit convention: x=heading(yaw=psi), y=roll(phi), z=pitch(theta)
    float psi   = wrapAngle(e.x());
    float phi   = wrapAngle(e.y());
    float theta = wrapAngle(e.z());

    psi_deg   = psi;
    phi_deg   = phi;
    theta_deg = theta;
  }

  // Print at a lower rate so Serial doesn't break timing
  if ((uint32_t)(nowUs - lastPrintUs) >= PRINT_PERIOD_US) {
    lastPrintUs += PRINT_PERIOD_US;

    Serial.print("phi=");
    Serial.print(phi_deg, 3);
    Serial.print(", theta=");
    Serial.print(theta_deg, 3);
    Serial.print(", psi=");
    Serial.println(psi_deg, 3);
  }

}

