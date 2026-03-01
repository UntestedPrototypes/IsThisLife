#ifdef ROLE_CONTROLLER
#include "robot_commands.h"
#include "controller_config.h"
#include "packets.h"
#include <Arduino.h>
#include <esp_now.h>

// Helper to handle ESP-NOW sends
void dispatchPacket(uint8_t robot_id, void* pkt, size_t size) {
    if (robot_id == 0) { // Broadcast
        for (int i = 0; i < NUM_ROBOTS; i++) {
            if (esp_now_is_peer_exist(robotMacs[i])) {
                esp_now_send(robotMacs[i], (uint8_t*)pkt, size);
            }
        }
    } else if (robot_id <= NUM_ROBOTS) {
        if (esp_now_is_peer_exist(robotMacs[robot_id-1])) {
            esp_now_send(robotMacs[robot_id-1], (uint8_t*)pkt, size);
        }
    }
}

void sendControlCommand(uint8_t robot_id, uint16_t vx, uint16_t vy, uint16_t omega) {
    ControlPacket pkt{};
    pkt.type = PACKET_CONTROL;
    pkt.priority = 0; // Default priority
    pkt.robot_id = robot_id;
    pkt.heartbeat = heartbeatCounter;
    pkt.vx = vx;
    pkt.vy = vy;
    pkt.omega = omega;
    pkt.timestamp_ms = millis();

    dispatchPacket(robot_id, &pkt, sizeof(pkt));
    heartbeatCounter = (heartbeatCounter + 1) & 0xFF;
}

void sendArmRobot(uint8_t robot_id) {
    if (robot_id < 1 || robot_id > NUM_ROBOTS) return;
    ControlPacket pkt{};
    pkt.type = PACKET_ESTOP_CLEAR;
    pkt.robot_id = robot_id;
    pkt.heartbeat = heartbeatCounter;

    pkt.vx = RC_NEUTRAL;
    pkt.vy = RC_NEUTRAL;
    pkt.omega = RC_NEUTRAL;

    pkt.timestamp_ms = millis();    
    dispatchPacket(robot_id, &pkt, sizeof(pkt));
    heartbeatCounter = (heartbeatCounter + 1) & 0xFF;
}

void sendEstopRobot(uint8_t robot_id) {
    if (robot_id < 1 || robot_id > NUM_ROBOTS) return;
    ControlPacket pkt{};
    pkt.type = PACKET_ESTOP;
    pkt.robot_id = robot_id;
    pkt.heartbeat = heartbeatCounter;

    pkt.vx = RC_NEUTRAL;
    pkt.vy = RC_NEUTRAL;
    pkt.omega = RC_NEUTRAL;

    pkt.timestamp_ms = millis();
    dispatchPacket(robot_id, &pkt, sizeof(pkt));
    heartbeatCounter = (heartbeatCounter + 1) & 0xFF;
}

void sendConfirmation(uint8_t robot_id, uint8_t step_id, bool approved) {
    ConfirmPacket pkt{};
    pkt.type = PACKET_CONFIRM;
    pkt.robot_id = robot_id;
    pkt.heartbeat = heartbeatCounter;

    pkt.step_id = step_id;
    pkt.approved = approved;

    dispatchPacket(robot_id, &pkt, sizeof(pkt));
    heartbeatCounter = (heartbeatCounter + 1) & 0xFF;
}

void sendStartSequence(uint8_t robot_id, uint8_t sequence_id) {
    StartSequencePacket pkt{};
    pkt.type = PACKET_START_SEQUENCE;
    pkt.robot_id = robot_id;
    pkt.heartbeat = heartbeatCounter;
    pkt.sequence_id = sequence_id;

    dispatchPacket(robot_id, &pkt, sizeof(pkt));
    heartbeatCounter = (heartbeatCounter + 1) & 0xFF;
}

void sendDiscover() {
    ControlPacket pkt{};
    pkt.type = PACKET_DISCOVER;
    pkt.heartbeat = heartbeatCounter;

    dispatchPacket(0, &pkt, sizeof(pkt));
    heartbeatCounter = (heartbeatCounter + 1) & 0xFF;
}
#endif // ROLE_CONTROLLER