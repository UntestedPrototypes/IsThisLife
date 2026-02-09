#ifdef ROLE_CONTROLLER
#ifndef ROBOT_COMMANDS_H
#define ROBOT_COMMANDS_H

#include <stdint.h>

// Functions to send commands to robots
void sendHeartbeat();
void sendArmRobot(uint8_t robot_id);
void sendEstopRobot(uint8_t robot_id);
void sendConfirmation(uint8_t robot_id, uint8_t step_id, bool approved);
void sendStartSequence(uint8_t robot_id, uint8_t sequence_id);
void sendControlCommand(uint8_t robot_id, int8_t vx, int8_t vy, int8_t omega);
void sendDiscover();

#endif // ROBOT_COMMANDS_H
#endif // ROLE_CONTROLLER
