#ifdef ROLE_ROBOT

#include "imu_handler.h"
#include "../config/robot_config.h"
#include "../config/robot_preferences.h"
#include "../math/quaternion_math.h"
#include "sensors.h" // For sensorsReady
#include <Adafruit_BNO055.h>

Adafruit_BNO055 imuSecondary = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 imuMain = Adafruit_BNO055(55, 0x29);

// Store the "zero" reference quaternion (defaults to identity quaternion)
float zero_qw = 1.0f, zero_qx = 0.0f, zero_qy = 0.0f, zero_qz = 0.0f;

bool initIMU() {
    if (!imuMain.begin() || !imuSecondary.begin()) {
        Serial.println("ERROR: IMU initialization failed");
        return false;
    }
    
    imuSecondary.setExtCrystalUse(true);
    imuMain.setExtCrystalUse(true);
    
    imuSecondary.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);
    imuMain.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);
    
    return true;
}

int16_t getIMUTemp() {
    if (!sensorsReady) return 0;
    int16_t temp = 0;
    
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        temp = imuMain.getTemp();
        xSemaphoreGive(i2cMutex);
    }
    return temp;
}

void waitForIMUCalibration() {
    uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
    bool ledState = false; 
    
    pinMode(LED_PIN, OUTPUT);
    
    Serial.println("Waiting for ALL sensors to fully calibrate (Level 3).");
    Serial.println("Perform calibration movements (figure-8, tilting, resting)...");

    while (gyro < 3 || accel < 3 || mag < 3) {
        getSecondaryIMUCalibration(&sys, &gyro, &accel, &mag);
        Serial.printf("Cal Status -> Sys: %u, Gyro: %u, Accel: %u, Mag: %u\n", sys, gyro, accel, mag);
        
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        delay(100); 
    }
    
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
    imu::Vector<3> robot_Z = g_upright * -1.0f; 
    robot_Z.normalize();

    imu::Vector<3> robot_X = g_nose;
    robot_X.normalize();

    imu::Vector<3> robot_Y = robot_Z.cross(robot_X);
    robot_Y.normalize();

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

    float mag = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    qw /= mag; qx /= mag; qy /= mag; qz /= mag;

    saveIMUOffsets(qw, qx, qy, qz);
    
    Serial.println("\n==========================================");
    Serial.println(" CALIBRATION SUCCESSFUL & SAVED TO FLASH! ");
}

void readMainIMUQuaternion(float* qw, float* qx, float* qy, float* qz) {
    if (!sensorsReady) { *qw=1; *qx=0; *qy=0; *qz=0; return; }
    
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        imu::Quaternion quat = imuMain.getQuat();
        *qw = quat.w(); *qx = quat.x(); *qy = quat.y(); *qz = quat.z();
        xSemaphoreGive(i2cMutex);
    } else {
        *qw=1; *qx=0; *qy=0; *qz=0;
    }
}

void readSecondaryIMUQuaternion(float* qw, float* qx, float* qy, float* qz) {
    if (!sensorsReady) { *qw=1; *qx=0; *qy=0; *qz=0; return; }
    
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        imu::Quaternion quat = imuSecondary.getQuat();
        multiplyQuaternions(quat.w(), quat.x(), quat.y(), quat.z(), 
                            robotSettings.imu_off_w, robotSettings.imu_off_x, 
                            robotSettings.imu_off_y, robotSettings.imu_off_z, 
                            qw, qx, qy, qz);
        xSemaphoreGive(i2cMutex);
    }
}

void getMainAxisOrientation(float* roll, float* pitch, float* yaw) {
    float qw, qx, qy, qz;
    readMainIMUQuaternion(&qw, &qx, &qy, &qz);
    quaternionToEuler(qw, qx, qy, qz, roll, pitch, yaw);
}

void getFullPendulumOrientation(float* internalRoll, float* internalPitch) {
    if (!sensorsReady) return;

    float mw, mx, my, mz;
    readMainIMUQuaternion(&mw, &mx, &my, &mz);
    
    float sw, sx, sy, sz;
    readSecondaryIMUQuaternion(&sw, &sx, &sy, &sz);

    float mw_inv = mw;
    float mx_inv = -mx;
    float my_inv = -my;
    float mz_inv = -mz;

    float rw, rx, ry, rz;
    multiplyQuaternions(mw_inv, mx_inv, my_inv, mz_inv, sw, sx, sy, sz, &rw, &rx, &ry, &rz);

    float dummyYaw;
    quaternionToEuler(rw, rx, ry, rz, internalRoll, internalPitch, &dummyYaw);
}

float readSecondaryIMUAngle() {
    float relRoll, relPitch;
    getFullPendulumOrientation(&relRoll, &relPitch);
    
    static float filtered = 0.0f;
    filtered = (relRoll * 0.3f) + (filtered * 0.7f);
    return filtered;
}

void printSecondaryIMUAngles() {
    float roll, pitch, yaw;
    uint8_t sys, gyro, accel, mag;

    getZeroedSecondaryIMUAngles(&roll, &pitch, &yaw);
    getSecondaryIMUCalibration(&sys, &gyro, &accel, &mag);

    Serial.printf("Secondary IMU -> Roll: %6.2f, Pitch: %6.2f, Yaw: %6.2f | Cal (0-3) -> Sys: %u, Gyro: %u, Accel: %u, Mag: %u\n", 
                  roll, pitch, yaw, sys, gyro, accel, mag);
}

void getSecondaryIMUCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
    if (!sensorsReady) { 
        *sys = 0; *gyro = 0; *accel = 0; *mag = 0; 
        return; 
    }
    
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        imuSecondary.getCalibration(sys, gyro, accel, mag);
        xSemaphoreGive(i2cMutex);
    } else {
        *sys = 0; *gyro = 0; *accel = 0; *mag = 0;
    }
}

void tareSecondaryIMU() {
    readSecondaryIMUQuaternion(&zero_qw, &zero_qx, &zero_qy, &zero_qz);
    Serial.println("Secondary IMU Tared/Zeroed!");
}

void getZeroedSecondaryIMUAngles(float* roll, float* pitch, float* yaw) {
    float curr_qw, curr_qx, curr_qy, curr_qz;
    readSecondaryIMUQuaternion(&curr_qw, &curr_qx, &curr_qy, &curr_qz);

    float inv_qw = zero_qw;
    float inv_qx = -zero_qx;
    float inv_qy = -zero_qy;
    float inv_qz = -zero_qz;

    float rel_w, rel_x, rel_y, rel_z;
    multiplyQuaternions(inv_qw, inv_qx, inv_qy, inv_qz, 
                        curr_qw, curr_qx, curr_qy, curr_qz, 
                        &rel_w, &rel_x, &rel_y, &rel_z);

    quaternionToEuler(rel_w, rel_x, rel_y, rel_z, roll, pitch, yaw);
}

#endif // ROLE_ROBOT