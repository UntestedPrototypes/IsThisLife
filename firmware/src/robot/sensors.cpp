#ifdef ROLE_ROBOT

#include "sensors.h"
#include "robot_config.h"
#include <Wire.h>

// Sensor Objects
Adafruit_INA219 ina219;
Adafruit_BNO055 imuSecondary = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 imuMain = Adafruit_BNO055(55, 0x29);

bool sensorsReady = false;

bool initSensors() {
    Serial.println("DEBUG: Initializing Sensors...");
    
    // Init I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000); 
    
    // Init Voltage Sensor
    if (!ina219.begin()) {
        Serial.println("ERROR: Failed to find INA219 chip");
    }

    // Init IMUs
    if (!imuMain.begin()) {
        Serial.println("ERROR: Failed to initialize Main IMU (0x29)");
        return false;
    }
    if (!imuSecondary.begin()) {
        Serial.println("ERROR: Failed to initialize Secondary IMU (0x28)");
        return false;
    }
    
    // Use external crystal for better accuracy
    imuSecondary.setExtCrystalUse(true);
    imuMain.setExtCrystalUse(true);
    
    // Set to IMU mode (Gyro + Accel fusion, no magnetometer to avoid motor interference)
    // IMUPLUS mode = 9-DOF without magnetometer (gyro + accel + gravity vector)
    imuSecondary.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);
    imuMain.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);
    //Serial.println("DEBUG: Both IMUs set to IMU mode (gyro+accel, no magnetometer)");
    
    sensorsReady = true;
    Serial.println("DEBUG: Sensors initialized successfully");

    return sensorsReady;
}

uint16_t readBattery() { 
    // Get bus voltage (V) and convert to mV
    float busvoltage = ina219.getBusVoltage_V();
    return (uint16_t)(busvoltage * 1000);
}

int16_t readTemp() { 
    // Use the Secondary IMU temperature as a proxy for the system/motor temp
    // The BNO055 returns an int8_t (signed byte)
    if (!sensorsReady) return 0;
    return (int16_t)imuSecondary.getTemp();
}

uint8_t getErrorFlags() { 
    static uint32_t lastCheckTime = 0;
    static bool lowBatteryFlag = false;
    static bool highBatteryFlag = false;
    static bool highTempFlag = false;
    
    uint8_t flags = 0;
    if (!sensorsReady) flags |= 0x01; // Sensor init failed
    
    // Battery and temperature checks - only every 1000ms to avoid blocking I2C reads every loop
    uint32_t now = millis();
    if (now - lastCheckTime >= 1000) {
        lastCheckTime = now;
        uint16_t battery = readBattery();
        int16_t temp = readTemp();
        
        // Battery checks
        if (battery < 14800) {
            lowBatteryFlag = true;
        } else {
            lowBatteryFlag = false;
        }
        
        if (battery > 17000) {
            highBatteryFlag = true;
        } else {
            highBatteryFlag = false;
        }
        
        // Temperature check (40°C)
        if (temp > 40) {
            highTempFlag = true;
        } else {
            highTempFlag = false;
        }
    }
    
    if (lowBatteryFlag) flags |= 0x02;  // Low Battery (<14.8V)
    if (highBatteryFlag) flags |= 0x04; // High Battery (>17.0V)
    if (highTempFlag) flags |= 0x08;    // High Temperature (>40°C)
    
    return flags; 
}

// Get quaternion orientation from main IMU (robust, no gimbal lock)
// Returns raw quaternion: w, x, y, z components
// Can be used to safely calculate any rotation without axis flipping
void readMainIMUQuaternion(float* qw, float* qx, float* qy, float* qz) {
    if (!sensorsReady) {
        *qw = 1.0f; *qx = 0.0f; *qy = 0.0f; *qz = 0.0f;
        return;
    }
    
    imu::Quaternion quat = imuMain.getQuat();
    *qw = quat.w();
    *qx = quat.x();
    *qy = quat.y();
    *qz = quat.z();
    
    // static uint32_t lastDebug = 0;
    // uint32_t now = millis();
    // if (now - lastDebug > 500) {
    //     lastDebug = now;
    //     Serial.printf("SENSOR DEBUG - imuMain Quaternion: w=%.4f x=%.4f y=%.4f z=%.4f\n",
    //                   *qw, *qx, *qy, *qz);
    // }
}

// Get quaternion orientation from secondary IMU (robust, no gimbal lock)
void readSecondaryIMUQuaternion(float* qw, float* qx, float* qy, float* qz) {
    if (!sensorsReady) {
        *qw = 1.0f; *qx = 0.0f; *qy = 0.0f; *qz = 0.0f;
        return;
    }
    
    imu::Quaternion quat = imuSecondary.getQuat();
    *qw = quat.w();
    *qx = quat.x();
    *qy = quat.y();
    *qz = quat.z();
}

// Convert quaternion to roll, pitch, yaw (safer than direct Euler angles)
// This handles gimbal lock cases better than getVector(VECTOR_EULER)
void quaternionToEuler(float qw, float qx, float qy, float qz, float* roll, float* pitch, float* yaw) {
    // Roll (rotation around forward axis)
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    *roll = atan2(sinr_cosp, cosr_cosp) * 180.0f / M_PI;
    
    // Pitch (rotation around side axis)
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1.0f) {
        *pitch = copysign(90.0f, sinp);  // Use 90 if out of range
    } else {
        *pitch = asin(sinp) * 180.0f / M_PI;
    }
    
    // Yaw (rotation around vertical axis)
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    *yaw = atan2(siny_cosp, cosy_cosp) * 180.0f / M_PI;
}

// Read rotation around motor's roll axis (accounts for full 3D tumbling)
// Tracks RELATIVE rotation from initial position, only around the motor's axis
float readMainIMUMotorAxisRotation() {
    if (!sensorsReady) return 0.0f;
    
    float qw, qx, qy, qz;
    readMainIMUQuaternion(&qw, &qx, &qy, &qz);
    
    // Store reference quaternion on first call (represents motor at zero position)
    static float refQw = 1.0f, refQx = 0.0f, refQy = 0.0f, refQz = 0.0f;
    static bool firstRead = true;
    
    if (firstRead) {
        refQw = qw;
        refQx = qx;
        refQy = qy;
        refQz = qz;
        firstRead = false;
        return 0.0f;
    }
    
    // Compute relative quaternion: current * conjugate(reference)
    // This gives us the rotation FROM the reference state TO the current state
    float refQx_conj = -refQx;
    float refQy_conj = -refQy;
    float refQz_conj = -refQz;
    
    // Quaternion multiplication: q1 * q2
    float relQw = qw * refQw - qx * refQx_conj - qy * refQy_conj - qz * refQz_conj;
    float relQx = qw * refQx_conj + qx * refQw + qy * refQz_conj - qz * refQy_conj;
    float relQy = qw * refQy_conj - qx * refQz_conj + qy * refQw + qz * refQx_conj;
    float relQz = qw * refQz_conj + qx * refQy_conj - qy * refQx_conj + qz * refQw;
    
    // Extract roll (X-axis rotation) from the RELATIVE quaternion
    // Use full Euler conversion to properly isolate rotation around X-axis
    // This avoids interference from pitch/yaw rotations
    float relRoll, relPitch, relYaw;
    quaternionToEuler(relQw, relQx, relQy, relQz, &relRoll, &relPitch, &relYaw);
    float motorAxisRotation = relRoll;
    
    // Clamp to reasonable range
    if (motorAxisRotation > 180.0f) motorAxisRotation -= 360.0f;
    if (motorAxisRotation < -180.0f) motorAxisRotation += 360.0f;
    
    static uint32_t lastDebug = 0;
    uint32_t now = millis();
    if (now - lastDebug > 50) {
        lastDebug = now;
        Serial.printf("SENSOR DEBUG - imuMain Motor Axis Rotation: %.2f° (relative to init)\n", motorAxisRotation);
    }
    
    return motorAxisRotation;
}

// Read rotation around secondary motor's pitch axis (accounts for full 3D tumbling)
// Tracks RELATIVE rotation from initial position, only around the motor's axis
float readSecondaryIMUMotorAxisRotation() {
    if (!sensorsReady) return 0.0f;
    
    float qw, qx, qy, qz;
    readSecondaryIMUQuaternion(&qw, &qx, &qy, &qz);
    
    // Store reference quaternion on first call (represents motor at zero position)
    static float refQw = 1.0f, refQx = 0.0f, refQy = 0.0f, refQz = 0.0f;
    static bool firstRead = true;
    
    if (firstRead) {
        refQw = qw;
        refQx = qx;
        refQy = qy;
        refQz = qz;
        firstRead = false;
        return 0.0f;
    }
    
    // Compute relative quaternion: current * conjugate(reference)
    // This gives us the rotation FROM the reference state TO the current state
    float refQx_conj = -refQx;
    float refQy_conj = -refQy;
    float refQz_conj = -refQz;
    
    // Quaternion multiplication: q1 * q2
    float relQw = qw * refQw - qx * refQx_conj - qy * refQy_conj - qz * refQz_conj;
    float relQx = qw * refQx_conj + qx * refQw + qy * refQz_conj - qz * refQy_conj;
    float relQy = qw * refQy_conj - qx * refQz_conj + qy * refQw + qz * refQx_conj;
    float relQz = qw * refQz_conj + qx * refQy_conj - qy * refQx_conj + qz * refQw;
    
    // Extract pitch (Y-axis rotation) from the RELATIVE quaternion
    // Use stable atan2-based Euler angles - avoids gimbal lock and yaw interference
    // pitch = atan2(2(w*y - z*x), 1 - 2(y^2 + z^2))
    float motorAxisRotation = atan2(2.0f * (relQw * relQy - relQz * relQx),
                                     1.0f - 2.0f * (relQy * relQy + relQz * relQz)) * 180.0f / M_PI;
    
    // Clamp to reasonable range
    if (motorAxisRotation > 180.0f) motorAxisRotation -= 360.0f;
    if (motorAxisRotation < -180.0f) motorAxisRotation += 360.0f;
    
    static uint32_t lastDebug = 0;
    uint32_t now = millis();
    if (now - lastDebug > 50) {
        lastDebug = now;
        Serial.printf("SENSOR DEBUG - imuSecondary Motor Axis Rotation: %.2f° (relative to init)\n", motorAxisRotation);
    }
    
    return motorAxisRotation;
}

// Read pitch angle from main IMU (imuMain) in degrees
float readMainIMUAngle() {
    if (!sensorsReady) return 0.0f;
    
    float qw, qx, qy, qz;
    readMainIMUQuaternion(&qw, &qx, &qy, &qz);
    
    float roll, pitch, yaw;
    quaternionToEuler(qw, qx, qy, qz, &roll, &pitch, &yaw);
    
    static uint32_t lastDebug = 0;
    uint32_t now = millis();
    if (now - lastDebug > 50) {
        lastDebug = now;
        Serial.printf("SENSOR DEBUG - imuMain (Main - 0x29): Pitch=%.2f° | Roll=%.2f° | Yaw=%.2f°\n",
                      pitch, roll, yaw);
    }
    
    return pitch;
}

// Read pitch angle from secondary IMU (imuSecondary) in degrees
float readSecondaryIMUAngle() {
    if (!sensorsReady) return 0.0f;
    
    float qw, qx, qy, qz;
    readSecondaryIMUQuaternion(&qw, &qx, &qy, &qz);
    
    float roll, pitch, yaw;
    quaternionToEuler(qw, qx, qy, qz, &roll, &pitch, &yaw);
    
    // Low-pass filter to smooth noisy secondary IMU readings
    static float filteredPitch = 0.0f;
    static bool firstRead = true;
    const float FILTER_ALPHA = 0.3f;  // Higher = more filtering
    
    if (firstRead) {
        filteredPitch = pitch;
        firstRead = false;
    } else {
        filteredPitch = (FILTER_ALPHA * pitch) + ((1.0f - FILTER_ALPHA) * filteredPitch);
    }
    
    static uint32_t lastDebug = 0;
    uint32_t now = millis();
    if (now - lastDebug > 500) {
        lastDebug = now;
        Serial.printf("SENSOR DEBUG - imuSecondary (Secondary - 0x28): Pitch=%.2f° (filtered: %.2f°) | Roll=%.2f° | Yaw=%.2f°\n",
                      pitch, filteredPitch, roll, yaw);
    }
    
    return filteredPitch;
}
#endif // ROLE_ROBOT