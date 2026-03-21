#ifdef ROLE_CONTROLLER
#ifndef ROBOT_COMMANDS_H
#define ROBOT_COMMANDS_H

#include <Arduino.h>
#include <stdint.h>

void dispatchPacket(uint8_t robot_id, void* pkt, size_t size);

void sendHeartbeat();
void sendArmRobot(uint8_t robot_id);
void sendEstopRobot(uint8_t robot_id);
void sendConfirmation(uint8_t robot_id, uint8_t step_id, bool approved);
void sendStartSequence(uint8_t robot_id, uint8_t sequence_id);
void sendControlCommand(uint8_t robot_id, uint8_t mode, uint16_t vx, uint16_t vy, uint16_t omega);

void sendSetSetting(uint8_t robot_id, const char* key, float value);
void sendGetSetting(uint8_t robot_id, const char* key); // <--- Added Get command

void sendDiscover();

#endif // ROBOT_COMMANDS_H
#endif // ROLE_CONTROLLER