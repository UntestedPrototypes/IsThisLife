#ifdef ROLE_ROBOT

#include "imu_handler.h"
#include "../config/robot_config.h"
#include "../config/robot_preferences.h"
#include "../math/quaternion_math.h"
#include "sensors.h" // For sensorsReady
#include <Adafruit_BNO055.h>

Adafruit_BNO055 imuSecondary = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 imuMain = Adafruit_BNO055(55, 0x29);

static float acc_x = 0, acc_y = 0, acc_z = 0;
static int sample_count = 0;
static imu::Vector<3> g_upright;
static imu::Vector<3> g_nose;

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

bool isIMUCalibrated() {
    uint8_t mg, ma, mm, sg, sa, sm;
    getIMUCalibrationState(&mg, &ma, &mm, &sg, &sa, &sm);
    return (mg >= 3 && ma >= 3 && mm >= 3 && sg >= 3 && sa >= 3 && sm >= 3);
}

void resetOffsetAccumulator() {
    acc_x = 0; acc_y = 0; acc_z = 0;
    sample_count = 0;
}

// Called repeatedly by the Sequence Engine. Returns true when 100 samples are collected.
bool accumulateOffsetSample() {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        imu::Vector<3> vec = imuSecondary.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
        acc_x += vec.x(); 
        acc_y += vec.y(); 
        acc_z += vec.z();
        xSemaphoreGive(i2cMutex);
        sample_count++;
    }
    return (sample_count >= 100);
}

void saveUprightVector() {
    g_upright = imu::Vector<3>(acc_x / 100.0f, acc_y / 100.0f, acc_z / 100.0f);
}

void calculateAndSaveOffsets() {
    g_nose = imu::Vector<3>(acc_x / 100.0f, acc_y / 100.0f, acc_z / 100.0f);
    
    // --- COORDINATE MAPPING ---
    imu::Vector<3> robot_Z = g_upright; 
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
        qw = 0.25f * S; qx = (m21 - m12) / S; qy = (m02 - m20) / S; qz = (m10 - m01) / S;
    } else if ((m00 > m11) && (m00 > m22)) {
        float S = sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        qw = (m21 - m12) / S; qx = 0.25f * S; qy = (m01 + m10) / S; qz = (m02 + m20) / S;
    } else if (m11 > m22) {
        float S = sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        qw = (m02 - m20) / S; qx = (m01 + m10) / S; qy = 0.25f * S; qz = (m12 + m21) / S;
    } else {
        float S = sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        qw = (m10 - m01) / S; qx = (m02 + m20) / S; qy = (m12 + m21) / S; qz = 0.25f * S;
    }

    float mag = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    qw /= mag; qx /= mag; qy /= mag; qz /= mag;

    saveIMUOffsets(qw, qx, qy, qz);
    Serial.println("\n--- IMU CALIBRATION SUCCESSFUL & SAVED TO FLASH ---");
}

void readMainIMU(float* roll, float* pitch, float* yaw) {
    float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
    
    if (sensorsReady) {
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            imu::Quaternion quat = imuMain.getQuat();
            qw = quat.w(); qx = quat.x(); qy = quat.y(); qz = quat.z();
            xSemaphoreGive(i2cMutex);
        }
    }
    
    // Convert directly to zeroed Euler angles
    quaternionToEuler(qw, qx, qy, qz, roll, pitch, yaw);
}

void readSecondaryIMU(float* roll, float* pitch, float* yaw) {
    float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
    
    if (sensorsReady) {
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            imu::Quaternion quat = imuSecondary.getQuat();
            
            // Multiply by the flashed mounting offset to "zero" the pendulum IMU
            multiplyQuaternions(quat.w(), quat.x(), quat.y(), quat.z(), 
                                robotSettings.imu_off_w, robotSettings.imu_off_x, 
                                robotSettings.imu_off_y, robotSettings.imu_off_z, 
                                &qw, &qx, &qy, &qz);
                                
            xSemaphoreGive(i2cMutex);
        }
    }
    
    // Convert offset-corrected quaternion to zeroed Euler angles
    quaternionToEuler(qw, qx, qy, qz, roll, pitch, yaw);
}

void printIMU() {
    float mRoll, mPitch, mYaw;
    float sRoll, sPitch, sYaw;
    uint8_t mg, ma, mm, sg, sa, sm;
    
    getIMUCalibrationState(&mg, &ma, &mm, &sg, &sa, &sm);
    readMainIMU(&mRoll, &mPitch, &mYaw);
    readSecondaryIMU(&sRoll, &sPitch, &sYaw);

    Serial.printf("Main [R:%6.2f P:%6.2f Y:%6.2f] Cal(G:%u A:%u M:%u) | Sec [R:%6.2f P:%6.2f Y:%6.2f] Cal(G:%u A:%u M:%u)\n", 
                  mRoll, mPitch, mYaw, mg, ma, mm, sRoll, sPitch, sYaw, sg, sa, sm);
}

void getIMUCalibrationState(uint8_t* mGyro, uint8_t* mAccel, uint8_t* mMag, 
                            uint8_t* sGyro, uint8_t* sAccel, uint8_t* sMag) {
    if (!sensorsReady) {
        *mGyro = 0; *mAccel = 0; *mMag = 0;
        *sGyro = 0; *sAccel = 0; *sMag = 0;
        return;
    }
    
    uint8_t mSys = 0, sSys = 0; // We must provide a bucket for Sys, but we discard it
    
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        imuMain.getCalibration(&mSys, mGyro, mAccel, mMag);
        imuSecondary.getCalibration(&sSys, sGyro, sAccel, sMag);
        xSemaphoreGive(i2cMutex);
    } else {
        *mGyro = 0; *mAccel = 0; *mMag = 0;
        *sGyro = 0; *sAccel = 0; *sMag = 0;
    }
}

#endif // ROLE_ROBOT