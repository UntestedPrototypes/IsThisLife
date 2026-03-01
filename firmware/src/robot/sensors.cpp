#ifdef ROLE_ROBOT

#include "sensors.h"
#include "robot_config.h"
#include <Wire.h>

// Sensor Objects
Adafruit_INA219 ina219;
Adafruit_BNO055 imuSecondary = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 imuMain = Adafruit_BNO055(55, 0x29);

bool sensorsReady = false;

// --- Helper Math Functions ---

// Performs quaternion multiplication: r = q1 * q2
void multiplyQuaternions(float w1, float x1, float y1, float z1,
                         float w2, float x2, float y2, float z2,
                         float* rw, float* rx, float* ry, float* rz) {
    *rw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    *rx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    *ry = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    *rz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
}

// Standard conversion from Quaternion to Euler angles
void quaternionToEuler(float qw, float qx, float qy, float qz, float* roll, float* pitch, float* yaw) {
    // Roll
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    *roll = atan2(sinr_cosp, cosr_cosp) * 180.0f / M_PI;

    // Pitch
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1.0f) {
        *pitch = copysign(90.0f, sinp);
    } else {
        *pitch = asin(sinp) * 180.0f / M_PI;
    }

    // Yaw
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    *yaw = atan2(siny_cosp, cosy_cosp) * 180.0f / M_PI;
}

// --- Sensor Initialization and Housekeeping ---

bool initSensors() {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000); 
    
    if (!ina219.begin()) Serial.println("ERROR: INA219 fail");

    if (!imuMain.begin() || !imuSecondary.begin()) {
        Serial.println("ERROR: IMU initialization failed");
        return false;
    }
    
    imuSecondary.setExtCrystalUse(true);
    imuMain.setExtCrystalUse(true);
    
    imuSecondary.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);
    imuMain.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);
    
    sensorsReady = true;
    return true;
}

// --- System Status and Diagnostics (Thread-Safe) ---

uint16_t readBattery() {
    if (!sensorsReady) return 0;
    uint16_t voltage_mv = 0;
    
    // Safety lock for I2C Bus
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float busVoltage = ina219.getBusVoltage_V();
        voltage_mv = (uint16_t)(busVoltage * 1000.0f);
        xSemaphoreGive(i2cMutex);
    }
    return voltage_mv;
}

int16_t readTemp() {
    if (!sensorsReady) return 0;
    int16_t temp = 0;
    
    // Safety lock for I2C Bus
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        temp = imuMain.getTemp();
        xSemaphoreGive(i2cMutex);
    }
    return temp;
}

uint8_t getErrorFlags() {
    if (!sensorsReady) return ERROR_SENSOR_OFFLINE;
    uint8_t flags = ERROR_NONE;
    
    uint16_t batt = readBattery();
    int16_t temp = readTemp();
    
    // 1. Battery Under-Voltage (drops below 10V / 10000mV)
    if (batt > 0 && batt < 10000) { 
        flags |= ERROR_BATT_UNDERVOLTAGE;
    }
    
    // 2. Battery Over-Voltage (spikes above 13V / 13000mV)
    if (batt > 13000) { 
        flags |= ERROR_BATT_OVERVOLTAGE;
    }

    // 3. Over-Temperature (>= 40 Celsius)
    if (temp >= 40) {
        flags |= ERROR_TEMP_OVERHEAT;
    }
    
    return flags;
}

// --- Orientation Methods (Thread-Safe) ---

void readMainIMUQuaternion(float* qw, float* qx, float* qy, float* qz) {
    if (!sensorsReady) { *qw=1; *qx=0; *qy=0; *qz=0; return; }
    
    // Safety lock for I2C Bus
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        imu::Quaternion quat = imuMain.getQuat();
        *qw = quat.w(); *qx = quat.x(); *qy = quat.y(); *qz = quat.z();
        xSemaphoreGive(i2cMutex);
    } else {
        // Mutex timeout fallback
        *qw=1; *qx=0; *qy=0; *qz=0;
    }
}

void readSecondaryIMUQuaternion(float* qw, float* qx, float* qy, float* qz) {
    if (!sensorsReady) { *qw=1; *qx=0; *qy=0; *qz=0; return; }
    
    // Safety lock for I2C Bus
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        imu::Quaternion quat = imuSecondary.getQuat();
        *qw = quat.w(); *qx = quat.x(); *qy = quat.y(); *qz = quat.z();
        xSemaphoreGive(i2cMutex);
    } else {
        // Mutex timeout fallback
        *qw=1; *qx=0; *qy=0; *qz=0;
    }
}

/**
 * Returns the World orientation of the Main Axis (the sphere's frame).
 * Useful for knowing the ball's absolute tilt/heading in the room.
 */
void getMainAxisOrientation(float* roll, float* pitch, float* yaw) {
    float qw, qx, qy, qz;
    readMainIMUQuaternion(&qw, &qx, &qy, &qz);
    quaternionToEuler(qw, qx, qy, qz, roll, pitch, yaw);
}

/**
 * Returns the Relative orientation of the Pendulum.
 * This "subtracts" the rotation of the main axis from the secondary axis.
 * Use these angles for your +/- 45 degree control limits.
 */
void getFullPendulumOrientation(float* internalRoll, float* internalPitch) {
    if (!sensorsReady) return;

    float mw, mx, my, mz;
    readMainIMUQuaternion(&mw, &mx, &my, &mz);
    
    float sw, sx, sy, sz;
    readSecondaryIMUQuaternion(&sw, &sx, &sy, &sz);

    // 1. Calculate Conjugate of Main (q_inv)
    float mw_inv = mw;
    float mx_inv = -mx;
    float my_inv = -my;
    float mz_inv = -mz;

    // 2. Relative Quaternion (q_rel = q_main_inv * q_secondary)
    float rw, rx, ry, rz;
    multiplyQuaternions(mw_inv, mx_inv, my_inv, mz_inv, sw, sx, sy, sz, &rw, &rx, &ry, &rz);

    // 3. Convert relative rotation to Euler angles
    float dummyYaw;
    quaternionToEuler(rw, rx, ry, rz, internalRoll, internalPitch, &dummyYaw);
}

// Updated legacy method to use the new relative logic
float readSecondaryIMUAngle() {
    float relRoll, relPitch;
    getFullPendulumOrientation(&relRoll, &relPitch);
    
    // Choose relRoll or relPitch based on physical mounting orientation
    static float filtered = 0.0f;
    filtered = (relRoll * 0.3f) + (filtered * 0.7f);
    return filtered;
}

#endif // ROLE_ROBOT