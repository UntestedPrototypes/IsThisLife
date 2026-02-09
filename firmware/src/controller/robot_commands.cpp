#ifdef ROLE_CONTROLLER
#include "robot_commands.h"
#include "controller_config.h"
#include "packets.h"
#include <Arduino.h>
#include <esp_now.h>

void sendHeartbeat() {
    ControlPacket pkt{};
    pkt.type = PACKET_CONTROL;
    pkt.robot_id = 0; // broadcast to all robots
    pkt.vx = pkt.vy = pkt.omega = 0;
    pkt.heartbeat = heartbeatCounter;

    for (int i = 0; i < NUM_ROBOTS; i++) {
        if (esp_now_is_peer_exist(robotMacs[i])) {
            esp_now_send(robotMacs[i], (uint8_t*)&pkt, sizeof(pkt));
        }
    }

    heartbeatCounter = (heartbeatCounter + 1) & 0xFF;
}

void sendArmRobot(uint8_t robot_id) {
    if (robot_id < 1 || robot_id > NUM_ROBOTS) return;

    ControlPacket pkt{};
    pkt.type = PACKET_ESTOP_CLEAR;
    pkt.robot_id = robot_id;
    pkt.heartbeat = heartbeatCounter;
    pkt.vx = pkt.vy = pkt.omega = 0;
    
    esp_now_send(robotMacs[robot_id-1], (uint8_t*)&pkt, sizeof(pkt));
    Serial.printf("Sent ARM command to Robot %d\n", robot_id);
    heartbeatCounter = (heartbeatCounter + 1) & 0xFF;
}

void sendEstopRobot(uint8_t robot_id) {
    if (robot_id < 1 || robot_id > NUM_ROBOTS) return;

    ControlPacket pkt{};
    pkt.type = PACKET_ESTOP;
    pkt.robot_id = robot_id;
    pkt.heartbeat = heartbeatCounter;
    pkt.vx = pkt.vy = pkt.omega = 0;
    
    esp_now_send(robotMacs[robot_id-1], (uint8_t*)&pkt, sizeof(pkt));
    Serial.printf("Sent E-STOP to Robot %d\n", robot_id);
    heartbeatCounter = (heartbeatCounter + 1) & 0xFF;
}

void sendConfirmation(uint8_t robot_id, uint8_t step_id, bool approved) {
    if (robot_id < 1 || robot_id > NUM_ROBOTS) return;

    ConfirmPacket pkt{};
    pkt.type = PACKET_CONFIRM;
    pkt.robot_id = robot_id;
    pkt.heartbeat = heartbeatCounter;
    pkt.step_id = step_id;
    pkt.approved = approved;
    
    esp_now_send(robotMacs[robot_id-1], (uint8_t*)&pkt, sizeof(pkt));
    Serial.printf("Sent CONFIRM to Robot %d: step=%d approved=%d\n", robot_id, step_id, approved);
    heartbeatCounter = (heartbeatCounter + 1) & 0xFF;
}

void sendStartSequence(uint8_t robot_id, uint8_t sequence_id) {
    if (robot_id < 1 || robot_id > NUM_ROBOTS) return;

    StartSequencePacket pkt{};
    pkt.type = PACKET_START_SEQUENCE;
    pkt.robot_id = robot_id;
    pkt.heartbeat = heartbeatCounter;
    pkt.sequence_id = sequence_id;
    
    esp_now_send(robotMacs[robot_id-1], (uint8_t*)&pkt, sizeof(pkt));
    Serial.printf("Sent START_SEQUENCE to Robot %d: sequence_id=%d\n", robot_id, sequence_id);
    heartbeatCounter = (heartbeatCounter + 1) & 0xFF;
}

void sendControlCommand(uint8_t robot_id, int8_t vx, int8_t vy, int8_t omega) {
    ControlPacket pkt{};
    pkt.type = PACKET_CONTROL;
    pkt.robot_id = robot_id;
    pkt.vx = vx;
    pkt.vy = vy;
    pkt.omega = omega;
    pkt.heartbeat = heartbeatCounter;
    pkt.timestamp_ms = millis() & 0xFFFF;

    if (robot_id == 0) { // broadcast
        for (int i = 0; i < NUM_ROBOTS; i++) {
            if (esp_now_is_peer_exist(robotMacs[i])) {
                esp_now_send(robotMacs[i], (uint8_t*)&pkt, sizeof(pkt));
            }
        }
    } else if (robot_id <= NUM_ROBOTS) {
        esp_now_send(robotMacs[robot_id-1], (uint8_t*)&pkt, sizeof(pkt));
    }
    
    heartbeatCounter = (heartbeatCounter + 1) & 0xFF;
}

void sendDiscover() {
    ControlPacket pkt{};
    pkt.type = PACKET_DISCOVER;
    pkt.robot_id = 0;
    
    for (int i = 0; i < NUM_ROBOTS; i++) {
        if (esp_now_is_peer_exist(robotMacs[i])) {
            esp_now_send(robotMacs[i], (uint8_t*)&pkt, sizeof(pkt));
        }
    }
    
    heartbeatCounter = (heartbeatCounter + 1) & 0xFF;
}
#endif // ROLE_CONTROLLER
