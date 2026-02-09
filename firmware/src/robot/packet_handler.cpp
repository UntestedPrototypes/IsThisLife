#ifdef ROLE_ROBOT

#include "packet_handler.h"
#include "robot_config.h"
#include "packets.h"
#include "telemetry.h"
#include "safety.h"
#include "motors.h"
#include "heartbeat.h"
#include "confirmation.h"
#include "sequence.h"
#include <Arduino.h>
#include <string.h>

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len < 1) return;
    
    uint8_t pkt_type = data[0];
    
    // ========== Handle CONFIRM packet ==========
    if (pkt_type == PACKET_CONFIRM && len >= sizeof(ConfirmPacket)) {
        ConfirmPacket confirm{};
        memcpy(&confirm, data, sizeof(confirm));
        if (confirm.robot_id == ROBOT_ID) {
            handleConfirmation(confirm.step_id, confirm.approved);
        }
        return;
    }
    
    // ========== Handle START_SEQUENCE packet ==========
    if (pkt_type == PACKET_START_SEQUENCE && len >= sizeof(StartSequencePacket)) {
        StartSequencePacket seq{};
        memcpy(&seq, data, sizeof(seq));
        if (seq.robot_id == ROBOT_ID) {
            startSequence(seq.sequence_id);
        }
        return;
    }
    
    // Handle regular control packets
    if (len < sizeof(ControlPacket)) {
        Serial.printf("DEBUG: Received packet too small (%d bytes)\n", len);
        return;
    }
    
    ControlPacket pkt{};
    memcpy(&pkt, data, sizeof(pkt));

    if (pkt.robot_id != 0 && pkt.robot_id != ROBOT_ID) return;

    uint32_t now = millis();
    recordHeartbeat(now);

    switch(pkt.type) {
        case PACKET_DISCOVER:
            Serial.println("DEBUG: DISCOVER received");
            sendAckTelemetry(pkt.type, pkt.heartbeat, pkt.timestamp_ms);
            break;
            
        case PACKET_ESTOP:
            Serial.println("DEBUG: E-STOP received");
            activateEstop();
            cancelConfirmation();
            stopSequence();
            sendAckTelemetry(pkt.type, pkt.heartbeat, pkt.timestamp_ms);
            break;
            
        case PACKET_ESTOP_CLEAR:
            Serial.println("DEBUG: E-STOP cleared / ARM received");
            clearEstop();
            sendAckTelemetry(pkt.type, pkt.heartbeat, pkt.timestamp_ms);
            break;
            
        case PACKET_CONTROL:
            controlPacketCount++;
            
            if (controlPacketCount >= TELEMETRY_INTERVAL) {
                sendAckTelemetry(pkt.type, pkt.heartbeat, pkt.timestamp_ms);
                controlPacketCount = 0;
            }

            if (sequenceActive || waitingForConfirmation) {
                motorsEnabled = false;
                stopMotors();
                if (sequenceActive) {
                    Serial.println("DEBUG: Sequence active, motors disabled");
                } else {
                    Serial.println("DEBUG: Waiting for confirmation, motors disabled");
                }
            } else if (!estopActive) {
                if (heartbeatValid()) {
                    motorsEnabled = true;
                    setMotors(pkt.vx, pkt.vy, pkt.omega);
                    Serial.print("DEBUG: CONTROL ACTIVE");
                } else {
                    motorsEnabled = false;
                    stopMotors();
                    Serial.print("DEBUG: Heartbeat invalid, stopping motors");
                }
            } else {
                Serial.print("DEBUG: E-STOP active ");
            }
            // Updated format specifiers for uint16_t (%u)
            Serial.printf(" - type=%d robot_id=%d vx=%u vy=%u omega=%u hb=%u\n", 
                          pkt.type, pkt.robot_id, pkt.vx, pkt.vy, pkt.omega, pkt.heartbeat);
            break;
            
        default:
            Serial.printf("DEBUG: Unknown packet type %d\n", pkt.type);
            break;
    }
}
#endif // ROLE_ROBOT