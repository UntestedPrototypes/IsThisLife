#ifdef ROLE_ROBOT

#include "imu_handler.h"
#include "../config/robot_config.h"
#include "../config/robot_preferences.h"
#include "../math/quaternion_math.h"
#include "sensors.h" // For sensorsReady
#include <Adafruit_BNO055.h>

Adafruit_BNO055 imuSecondary = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 imuMain = Adafruit_BNO055(55, 0x29);

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
    uint8_t mSys, mGyro, mAccel, mMag;
    uint8_t sSys, sGyro, sAccel, sMag;
    bool ledState = false; 
        
    Serial.println("Waiting for BOTH IMUs to fully calibrate (Level 3).");
    Serial.println("Move the robot and pendulums until all values reach 3.");

    while (true) {
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            imuMain.getCalibration(&mSys, &mGyro, &mAccel, &mMag);
            imuSecondary.getCalibration(&sSys, &sGyro, &sAccel, &sMag);
            xSemaphoreGive(i2cMutex);
        }

        Serial.printf("Main Cal -> G:%u A:%u M:%u | Sec Cal -> G:%u A:%u M:%u\n", 
                      mGyro, mAccel, mMag, sGyro, sAccel, sMag);

        // Break loop only if all hardware sensors (Gyro, Accel, Mag) on both chips are 3
        if (mGyro >= 3 && mAccel >= 3 && mMag >= 3 && 
            sGyro >= 3 && sAccel >= 3 && sMag >= 3) {
            break;
        }
        
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        delay(100); 
    }
    
    digitalWrite(LED_PIN, HIGH); 
    Serial.println("Both IMUs are fully calibrated!");
}

void CalibrateIMUOffset() {
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
    uint8_t sys, gyro, accel, mag;
    
    // Get the lowest calibration states for each sensor type
    getIMUCalibrationState(&sys, &gyro, &accel, &mag);

    readMainIMU(&mRoll, &mPitch, &mYaw);
    readSecondaryIMU(&sRoll, &sPitch, &sYaw);

    Serial.printf("Main [R:%6.2f P:%6.2f Y:%6.2f] | Sec [R:%6.2f P:%6.2f Y:%6.2f] | Lowest Cal -> Sys:%u Gyro:%u Accel:%u Mag:%u\n", 
                  mRoll, mPitch, mYaw, sRoll, sPitch, sYaw, sys, gyro, accel, mag);
}

void getIMUCalibrationState(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
    if (!sensorsReady) {
        *sys = 0; *gyro = 0; *accel = 0; *mag = 0;
        return;
    }
    
    uint8_t mSys = 0, mGyro = 0, mAccel = 0, mMag = 0;
    uint8_t sSys = 0, sGyro = 0, sAccel = 0, sMag = 0;
    
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        imuMain.getCalibration(&mSys, &mGyro, &mAccel, &mMag);
        imuSecondary.getCalibration(&sSys, &sGyro, &sAccel, &sMag);
        xSemaphoreGive(i2cMutex);
    } else {
        *sys = 0; *gyro = 0; *accel = 0; *mag = 0;
        return; 
    }

    // Compare and return the lowest value for each specific hardware sensor
    *sys   = (mSys < sSys) ? mSys : sSys;
    *gyro  = (mGyro < sGyro) ? mGyro : sGyro;
    *accel = (mAccel < sAccel) ? mAccel : sAccel;
    *mag   = (mMag < sMag) ? mMag : sMag;
}

#endif // ROLE_ROBOT