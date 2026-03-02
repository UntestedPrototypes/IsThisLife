#ifdef ROLE_ROBOT

#include "telemetry.h"
#include "robot_config.h"
#include "robot_preferences.h"
#include "safety.h"
#include "confirmation.h"
#include "sequence.h"
#include "sensors.h"
#include "packets.h"
#include <Arduino.h>
#include <esp_now.h>

// Control packet counter
uint8_t controlPacketCount = 0;

void sendAckTelemetry(uint8_t type, uint32_t hb, uint16_t latency_ms) {
    AckTelemetryPacket ack{};
    ack.robot_id = robotSettings.robot_id;
    ack.acked_type = type;
    ack.heartbeat = hb;
    
    // Update status based on current state (priority order)
    if (estopActive) {
        ack.status = STATUS_ESTOP;
    } else if (waitingForConfirmation) {
        ack.status = STATUS_WAITING_CONFIRM;
    } else if (sequenceActive) {
        ack.status = STATUS_RUNNING_SEQUENCE;
    } else {
        ack.status = STATUS_OK;
    }
    
    ack.battery_mv = readBattery();
    ack.motor_temp = readTemp();
    ack.error_flags = getErrorFlags();
    ack.latency_ms = 808; // Placeholder, can be calculated if needed

    
    // Fetch and attach IMU Data
    float mRoll, mPitch, mYaw;
    getMainAxisOrientation(&mRoll, &mPitch, &mYaw);
    
    float pRoll, pPitch;
    getFullPendulumOrientation(&pRoll, &pPitch);
    
    ack.main_roll = mRoll;
    ack.main_pitch = mPitch;
    ack.pend_roll = pRoll;
    ack.pend_pitch = pPitch;

    esp_err_t res = esp_now_send(robotSettings.controller_mac, (uint8_t*)&ack, sizeof(ack));
    //Serial.printf("DEBUG: Telemetry sent - type=%d heartbeat=%u status=%d result=%d\n", type, hb, ack.status, res);
}
#endif // ROLE_ROBOT