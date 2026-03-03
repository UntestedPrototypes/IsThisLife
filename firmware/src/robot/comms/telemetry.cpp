#ifdef ROLE_ROBOT

#include "telemetry.h"
#include "../config/robot_config.h"
#include "../config/robot_preferences.h"
#include "../control/safety.h"
#include "../logic/confirmation.h"
#include "../logic/sequence.h"
#include "../sensors/sensors.h"
#include "packets.h"
#include <Arduino.h>
#include <esp_now.h>

// Control packet counter
uint8_t controlPacketCount = 0;

void sendTelemetry(uint8_t type, uint32_t hb, uint16_t latency_ms) {
    TelemetryPacket Tele{};
    Tele.robot_id = robotSettings.robot_id;
    Tele.type = PACKET_TELEMETRY; // Set packet type to telemetry
    Tele.heartbeat = hb;

    Tele.acked_type = type;
    
    uint8_t op_state = STATUS_NORMAL;
    
    // 1. Determine Logical Operational State
    if (waitingForConfirmation) {
        op_state = STATUS_WAITING_CONFIRM;
    } else if (sequenceActive) {
        op_state = STATUS_RUNNING_SEQUENCE;
    } else if (isCalibrationRequired()) {
        op_state = STATUS_CALIBRATION_REQUIRED;
    } else {
        op_state = STATUS_NORMAL;
    }
    
    // 2. Apply E-STOP Bitmask safely over the top
    if (isEstopActive()) {
        Tele.status = op_state | STATUS_FLAG_ESTOP;
    } else {
        Tele.status = op_state;
    }
    
    // 2. Apply E-STOP Bitmask safely over the top
    if (isEstopActive()) {
        Tele.status = op_state | STATUS_FLAG_ESTOP;
    } else {
        Tele.status = op_state;
    }
    
    Tele.battery_mv = readBattery();
    Tele.motor_temp = readTemp();
    Tele.error_flags = getErrorFlags();
    Tele.latency_ms = 808; // Placeholder, can be calculated if needed

    uint8_t mg, ma, mm, sg, sa, sm;
    getIMUCalibrationState(&mg, &ma, &mm, &sg, &sa, &sm);
    
    // Layout: [Unused:4] [sMag:2] [sAccel:2] [sGyro:2] [mMag:2] [mAccel:2] [mGyro:2]
    Tele.imu_calibration = ((sm & 0x03) << 10) | 
                           ((sa & 0x03) << 8)  | 
                           ((sg & 0x03) << 6)  | 
                           ((mm & 0x03) << 4)  | 
                           ((ma & 0x03) << 2)  | 
                           (mg & 0x03);
    
    // Fetch and attach IMU Data
    float mRoll, mPitch, mYaw;
    readMainIMU(&mRoll, &mPitch, &mYaw);
    
    float sRoll, sPitch, sYaw; // Added pYaw to catch the 3rd parameter
    readSecondaryIMU(&sRoll, &sPitch, &sYaw);
    
    Tele.main_roll = mRoll;
    Tele.main_pitch = mPitch;
    Tele.pend_roll = sRoll;
    Tele.pend_pitch = sPitch;

    esp_err_t res = esp_now_send(robotSettings.controller_mac, (uint8_t*)&Tele, sizeof(Tele));
    
    Serial.printf("Sent Telemetry: Type=%d, Status=0x%02X, Battery=%dmV, MotorTemp=%dC, Errors=0x%02X, Latency=%dms, IMU=[%.2f, %.2f, %.2f], Secondary IMU=[%.2f, %.2f, %.2f], ESP_NOW Result=%d\n",
        Tele.acked_type,
        Tele.status,
        Tele.battery_mv,
        Tele.motor_temp,
        Tele.error_flags,
        Tele.latency_ms,
        Tele.main_roll,
        Tele.main_pitch,
        mYaw,
        Tele.pend_roll,
        Tele.pend_pitch,
        sYaw,
        res
    );
}
#endif // ROLE_ROBOT