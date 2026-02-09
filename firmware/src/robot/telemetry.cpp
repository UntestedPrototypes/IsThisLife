#ifdef ROLE_ROBOT

#include "telemetry.h"
#include "robot_config.h"
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
    ack.robot_id = ROBOT_ID;
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
    ack.motor_temp = readMotorTemp();
    ack.error_flags = getErrorFlags();
    ack.latency_ms = 808; // Placeholder, can be calculated if needed

    esp_err_t res = esp_now_send(controllerMac, (uint8_t*)&ack, sizeof(ack));
    //Serial.printf("DEBUG: Telemetry sent - type=%d heartbeat=%u status=%d result=%d\n", type, hb, ack.status, res);
}
#endif // ROLE_ROBOT