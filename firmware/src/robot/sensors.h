#ifdef ROLE_ROBOT
#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_INA219.h>

// --- Explicit Error Flags ---
enum SensorErrorFlags {
    ERROR_NONE              = 0x00,       // 00000000 - All good
    ERROR_BATT_UNDERVOLTAGE = (1 << 0),   // 00000001 - Battery < 10V
    ERROR_BATT_OVERVOLTAGE  = (1 << 1),   // 00000010 - Battery > 13V
    ERROR_TEMP_OVERHEAT     = (1 << 2),   // 00000100 - Temp >= 40C
    ERROR_SENSOR_OFFLINE    = 0xFF        // 11111111 - I2C/Sensor Failure
};

// Initialize all hardware sensors (IMUs and INA219)
bool initSensors();

// System Status and Diagnostics
uint16_t readBattery();      // Returns battery voltage in mV
int16_t readTemp();          // Returns system temperature in Celsius
uint8_t getErrorFlags();     // Returns bitmask of SensorErrorFlags

// --- Raw Orientation Data ---
// These provide the raw 4D rotation data to avoid Gimbal Lock
void readMainIMUQuaternion(float* qw, float* qx, float* qy, float* qz);
void readSecondaryIMUQuaternion(float* qw, float* qx, float* qy, float* qz);

// --- Processed Orientation Data ---

/**
 * Calculates the orientation of the sphere's main internal axis relative to the world.
 * Returns angles in degrees.
 */
void getMainAxisOrientation(float* roll, float* pitch, float* yaw);

/**
 * Calculates the internal state of the pendulum.
 * This removes the sphere's rotation from the secondary axis to give the 
 * "Mechanical" angle of the weights.
 * Use these for your +/- 45 degree safety limits.
 */
void getFullPendulumOrientation(float* internalRoll, float* internalPitch);

/**
 * Legacy support: Returns the filtered pitch/roll angle for simple control loops.
 */
float readSecondaryIMUAngle();

// --- Math Utilities ---

/**
 * Combines two quaternions (q_out = q1 * q2).
 * Used for relative orientation and coordinate frame transformations.
 */
void multiplyQuaternions(float w1, float x1, float y1, float z1,
                         float w2, float x2, float y2, float z2,
                         float* rw, float* rx, float* ry, float* rz);

/**
 * Converts Quaternion components to standard Euler angles (Degrees).
 */
void quaternionToEuler(float qw, float qx, float qy, float qz, 
                        float* roll, float* pitch, float* yaw);

#endif // SENSORS_H
#endif // ROLE_ROBOT