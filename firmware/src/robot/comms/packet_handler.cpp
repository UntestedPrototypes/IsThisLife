#ifdef ROLE_ROBOT

#include "packet_handler.h"
#include "../config/robot_config.h"
#include "../config/robot_preferences.h"
#include "packets.h"
#include "telemetry.h"
#include "../control/safety.h"
#include "../control/motors.h"
#include "heartbeat.h"
#include "../logic/confirmation.h"
#include "../logic/sequence.h"
#include <Arduino.h>
#include <string.h>

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len < 1 || len > 250) return;
    
    uint8_t pkt_type = data[0];

    // ==========================================
    // FAST PATH: E-STOP ONLY
    // ==========================================
    if (pkt_type == PACKET_ESTOP) {
        // DO NOT use stateMutex here! This is an ISR.
        // Booleans are atomic on 32-bit systems, so this is safe.
        estopActive = true;
        motorsEnabled = false;

        // WAKE UP MOTOR TASK INSTANTLY
        if (controlTaskHandle != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xTaskNotifyFromISR(controlTaskHandle, 1, eSetBits, &xHigherPriorityTaskWoken);
            if(xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
        }
    }

    // ==========================================
    // SLOW PATH: Send ALL packets to Queue
    // ==========================================
    RxPacket rxPkt;
    memcpy(rxPkt.mac, mac, 6);
    memcpy(rxPkt.data, data, len);
    rxPkt.len = len;
    
    // Non-blocking send from the callback context to the SystemTask queue
    xQueueSendFromISR(rxQueue, &rxPkt, NULL);
}

void processPacket(const uint8_t *mac, const uint8_t *data, int len) {
    uint8_t pkt_type = data[0];
    
    if (pkt_type == PACKET_CONFIRM && len >= sizeof(ConfirmPacket)) {
        ConfirmPacket confirm{};
        memcpy(&confirm, data, sizeof(confirm));
        if (confirm.robot_id == robotSettings.robot_id) {
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            handleConfirmation(confirm.step_id, confirm.approved);
            xSemaphoreGive(stateMutex);
        }
        return;
    }
    
    if (pkt_type == PACKET_START_SEQUENCE && len >= sizeof(StartSequencePacket)) {
        StartSequencePacket seq{};
        memcpy(&seq, data, sizeof(seq));
        if (seq.robot_id == robotSettings.robot_id) {
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            startSequence(seq.sequence_id);
            xSemaphoreGive(stateMutex);
        }
        return;
    }
    
    if (len < sizeof(ControlPacket)) return;
    
    ControlPacket pkt{};
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.robot_id != 0 && pkt.robot_id != robotSettings.robot_id) return;

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    
    uint32_t now = millis();
    recordHeartbeat(now);

    switch(pkt.type) {
        case PACKET_DISCOVER:
            Serial.println("DEBUG: DISCOVER received");
            sendTelemetry(pkt.type, pkt.heartbeat, pkt.timestamp_ms);
            break;
            
        case PACKET_ESTOP:
            // Hardware was already stopped by Fast Path. 
            // Gracefully cancel sequences and send the ACK.
            Serial.println("DEBUG: E-STOP received");
            cancelConfirmation();
            stopSequence();
            sendTelemetry(pkt.type, pkt.heartbeat, pkt.timestamp_ms);
            break;
            
        case PACKET_ESTOP_CLEAR:
            Serial.println("DEBUG: E-STOP cleared / ARM received");
            clearEstop();
            sendTelemetry(pkt.type, pkt.heartbeat, pkt.timestamp_ms);
            break;
            
        case PACKET_CONTROL:
            //Serial.printf("DEBUG: CONTROL packet received - vx=%f vy=%f omega=%f\n", pkt.vx, pkt.vy, pkt.omega);
            controlPacketCount++;
            if (controlPacketCount >= robotSettings.telemetry_interval) {
                sendTelemetry(pkt.type, pkt.heartbeat, pkt.timestamp_ms);
                controlPacketCount = 0;
            }

            // Normal control update logic (ADDED !isCalibrationRequired())
            if (!isSequenceActive() && !waitingForConfirmation && !isEstopActive() && heartbeatValid() && !isCalibrationRequired()) {
                motorsEnabled = true;
                setTargetVelocities(pkt.vx, pkt.vy, pkt.omega);
            }
            break;
    }
    
    xSemaphoreGive(stateMutex);
}
#endif // ROLE_ROBOT