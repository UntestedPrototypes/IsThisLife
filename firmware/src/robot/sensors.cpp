#ifdef ROLE_ROBOT

#include "sensors.h"
#include "robot_config.h"
#include "robot_preferences.h"
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
    // Pitch (Y-axis) - This is the balancing angle
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1.0f) {
        *pitch = copysign(90.0f, sinp);
    } else {
        *pitch = asin(sinp) * 180.0f / M_PI;
    }

    // Roll (X-axis) - Side-to-side tilt
    *roll = atan2(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * 180.0f / M_PI;

    // Yaw (Z-axis) - Turning/Heading
    *yaw = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * 180.0f / M_PI;
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

void waitForIMUCalibration() {
    uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
    bool ledState = false; // Keep track of the LED state
    
    // Set up the LED pin as an output
    pinMode(LED_PIN, OUTPUT);
    
    Serial.println("Waiting for ALL sensors to fully calibrate (Level 3).");
    Serial.println("Perform calibration movements (figure-8, tilting, resting)...");

    // Loop continuously as long as ANY sensor is less than 3
    while (gyro < 3 || accel < 3 || mag < 3) {
        
        // Fetch the calibration statuses safely
        getSecondaryIMUCalibration(&sys, &gyro, &accel, &mag);
        
        Serial.printf("Cal Status -> Sys: %u, Gyro: %u, Accel: %u, Mag: %u\n", sys, gyro, accel, mag);
        
        // Toggle the LED
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        
        // Wait 100ms before checking again (this also sets the flash rate to ~5Hz)
        delay(100); 
    }
    
    // Calibration complete! Turn the LED on solid (or off, your choice)
    digitalWrite(LED_PIN, HIGH); 
    
    Serial.println("All sensors are fully calibrated! Safe to tare and run.");
}

void runGravityAlignmentCalibration() {
    Serial.println("\n--- IMU MOUNTING OFFSET CALIBRATION ---");

    // STEP 1: UPRIGHT (Balance Point)
    Serial.println("\nSTEP 1: Hold the robot perfectly UPRIGHT (balancing position).");
    Serial.println("Send ANY character when ready...");
    while(!Serial.available()) { delay(10); }
    while(Serial.available()) { Serial.read(); delay(2); }

    float upright_x = 0, upright_y = 0, upright_z = 0;
    for(int i = 0; i < 100; i++) {
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            imu::Vector<3> vec = imuSecondary.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
            upright_x += vec.x(); upright_y += vec.y(); upright_z += vec.z();
            xSemaphoreGive(i2cMutex);
        }
        delay(10);
    }
    imu::Vector<3> g_upright(upright_x/100.0f, upright_y/100.0f, upright_z/100.0f);

    // STEP 2: NOSE (Forward Direction)
    Serial.println("\nSTEP 2: Tilt the robot 90-degrees FORWARD (Nose to the floor).");
    Serial.println("Send ANY character when ready...");
    while(!Serial.available()) { delay(10); }
    while(Serial.available()) { Serial.read(); delay(2); }

    float nose_x = 0, nose_y = 0, nose_z = 0;
    for(int i = 0; i < 100; i++) {
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            imu::Vector<3> vec = imuSecondary.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
            nose_x += vec.x(); nose_y += vec.y(); nose_z += vec.z();
            xSemaphoreGive(i2cMutex);
        }
        delay(10);
    }
    imu::Vector<3> g_nose(nose_x/100.0f, nose_y/100.0f, nose_z/100.0f);

    // --- COORDINATE MAPPING ---
    // 1. Robot UP is the opposite of the upright gravity vector.
    // We map this to the Z-axis to avoid gimbal lock while balancing.
    imu::Vector<3> robot_Z = g_upright * -1.0f; 
    robot_Z.normalize();

    // 2. Robot FORWARD is the nose gravity vector.
    // We map this to the X-axis.
    imu::Vector<3> robot_X = g_nose;
    robot_X.normalize();

    // 3. Robot RIGHT (Y) = UP (Z) x FORWARD (X)
    imu::Vector<3> robot_Y = robot_Z.cross(robot_X);
    robot_Y.normalize();

    // 4. Re-calculate X to ensure perfect 90-degree orthogonality
    robot_X = robot_Y.cross(robot_Z);
    robot_X.normalize();

    // --- Construct Rotation Matrix (Sensor -> Robot) ---
    float m00 = robot_X.x(), m01 = robot_Y.x(), m02 = robot_Z.x();
    float m10 = robot_X.y(), m11 = robot_Y.y(), m12 = robot_Z.y();
    float m20 = robot_X.z(), m21 = robot_Y.z(), m22 = robot_Z.z();

    // Matrix to Quaternion Conversion
    float tr = m00 + m11 + m22;
    float qw, qx, qy, qz;

    if (tr > 0) {
        float S = sqrt(tr + 1.0f) * 2.0f;
        qw = 0.25f * S;
        qx = (m21 - m12) / S;
        qy = (m02 - m20) / S;
        qz = (m10 - m01) / S;
    } else if ((m00 > m11) && (m00 > m22)) {
        float S = sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        qw = (m21 - m12) / S;
        qx = 0.25f * S;
        qy = (m01 + m10) / S;
        qz = (m02 + m20) / S;
    } else if (m11 > m22) {
        float S = sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        qw = (m02 - m20) / S;
        qx = (m01 + m10) / S;
        qy = 0.25f * S;
        qz = (m12 + m21) / S;
    } else {
        float S = sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        qw = (m10 - m01) / S;
        qx = (m02 + m20) / S;
        qy = (m12 + m21) / S;
        qz = 0.25f * S;
    }

    // Normalize final quaternion
    float mag = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    qw /= mag; qx /= mag; qy /= mag; qz /= mag;

    // Save and apply!
    saveIMUOffsets(qw, qx, qy, qz);
    
    Serial.println("\n==========================================");
    Serial.println(" CALIBRATION SUCCESSFUL & SAVED TO FLASH! ");
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
    if (batt > 0 && batt < 14800) { 
        flags |= ERROR_BATT_UNDERVOLTAGE;
    }
    
    // 2. Battery Over-Voltage (spikes above 13V / 13000mV)
    if (batt > 17000) { 
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
    
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        imu::Quaternion quat = imuSecondary.getQuat();
        
        // Order: Sensor Reading * Calibration Offset
        multiplyQuaternions(quat.w(), quat.x(), quat.y(), quat.z(), 
                            robotSettings.imu_off_w, robotSettings.imu_off_x, 
                            robotSettings.imu_off_y, robotSettings.imu_off_z, 
                            qw, qx, qy, qz);
        
        xSemaphoreGive(i2cMutex);
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

void printSecondaryIMUAngles() {
    float roll, pitch, yaw;
    uint8_t sys, gyro, accel, mag;

    // 1. Get the angles (using the zeroed function we made earlier)
    getZeroedSecondaryIMUAngles(&roll, &pitch, &yaw);

    // 2. Get the calibration status safely
    getSecondaryIMUCalibration(&sys, &gyro, &accel, &mag);

    // 3. Print everything to the Serial Monitor
    Serial.printf("Secondary IMU -> Roll: %6.2f, Pitch: %6.2f, Yaw: %6.2f | Cal (0-3) -> Sys: %u, Gyro: %u, Accel: %u, Mag: %u\n", 
                  roll, pitch, yaw, sys, gyro, accel, mag);
}


void getSecondaryIMUCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
    if (!sensorsReady) { 
        *sys = 0; *gyro = 0; *accel = 0; *mag = 0; 
        return; 
    }
    
    // Safety lock for I2C Bus
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        imuSecondary.getCalibration(sys, gyro, accel, mag);
        xSemaphoreGive(i2cMutex);
    } else {
        // Mutex timeout fallback
        *sys = 0; *gyro = 0; *accel = 0; *mag = 0;
    }
}
// Store the "zero" reference quaternion (defaults to identity quaternion)
float zero_qw = 1.0f, zero_qx = 0.0f, zero_qy = 0.0f, zero_qz = 0.0f;

/**
 * Call this function when the robot is physically sitting in the exact 
 * position you want to consider "Zero" (e.g., during a setup phase or button press).
 */
void tareSecondaryIMU() {
    // Capture the current orientation and save it as our zero reference
    readSecondaryIMUQuaternion(&zero_qw, &zero_qx, &zero_qy, &zero_qz);
    Serial.println("Secondary IMU Tared/Zeroed!");
}

/**
 * Gets the Euler angles relative to the tared "zero" position.
 */
void getZeroedSecondaryIMUAngles(float* roll, float* pitch, float* yaw) {
    // Get current absolute reading
    float curr_qw, curr_qx, curr_qy, curr_qz;
    readSecondaryIMUQuaternion(&curr_qw, &curr_qx, &curr_qy, &curr_qz);

    // 1. Calculate the conjugate (inverse) of our zero reference
    float inv_qw = zero_qw;
    float inv_qx = -zero_qx;
    float inv_qy = -zero_qy;
    float inv_qz = -zero_qz;

    // 2. Multiply the inverse zero reference by the current reading
    // Math: q_relative = q_zero_inverse * q_current
    float rel_w, rel_x, rel_y, rel_z;
    multiplyQuaternions(inv_qw, inv_qx, inv_qy, inv_qz, 
                        curr_qw, curr_qx, curr_qy, curr_qz, 
                        &rel_w, &rel_x, &rel_y, &rel_z);

    // 3. Convert the relative quaternion to Euler angles
    quaternionToEuler(rel_w, rel_x, rel_y, rel_z, roll, pitch, yaw);
}

#endif // ROLE_ROBOT