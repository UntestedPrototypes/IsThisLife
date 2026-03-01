#ifdef ROLE_CONTROLLER
#ifndef ROBOT_COMMANDS_H
#define ROBOT_COMMANDS_H

#include <Arduino.h>
#include <stdint.h>

void dispatchPacket(uint8_t robot_id, void* pkt, size_t size);
// Functions to send commands to robots
void sendHeartbeat();
void sendArmRobot(uint8_t robot_id);
void sendEstopRobot(uint8_t robot_id);
void sendConfirmation(uint8_t robot_id, uint8_t step_id, bool approved);
void sendStartSequence(uint8_t robot_id, uint8_t sequence_id);

// Updated for RC signals (1000-2000)
void sendControlCommand(uint8_t robot_id, uint16_t vx, uint16_t vy, uint16_t omega);

void sendDiscover();

#endif // ROBOT_COMMANDS_H
#endif // ROLE_CONTROLLER