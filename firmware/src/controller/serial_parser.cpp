#ifdef ROLE_CONTROLLER
#include "serial_parser.h"
#include "controller_config.h"
#include "packets.h"
#include "robot_commands.h"
#include "python_comm.h"
#include <Arduino.h>

void readSerialCommands() {
    while (Serial.available() >= 2) {
        uint8_t cmd_type = Serial.read();
        uint8_t robot_id = Serial.read();

        switch(cmd_type) {
            case PACKET_CONTROL: {
                if (Serial.available() >= 3) {
                    int8_t vx = Serial.read();
                    int8_t vy = Serial.read();
                    int8_t omega = Serial.read();
                    sendControlCommand(robot_id, vx, vy, omega);
                }
                break;
            }
            
            case PACKET_ESTOP_CLEAR: {
                sendArmRobot(robot_id);
                Serial.printf("DEBUG: Sent ARM command to Robot %d\n", robot_id);
                break;
            }
            
            case PACKET_ESTOP: {
                sendEstopRobot(robot_id);
                break;
            }
            
            case PACKET_DISCOVER: {
                sendDiscover();
                break;
            }
            
            case PACKET_CONFIRM: {
                if (Serial.available() >= 2) {
                    uint8_t step_id = Serial.read();
                    uint8_t approved = Serial.read();  // 0=deny, 1=approve
                    sendConfirmation(robot_id, step_id, approved != 0);
                }
                break;
            }
            
            case PACKET_START_SEQUENCE: {
                if (Serial.available() >= 1) {
                    uint8_t sequence_id = Serial.read();
                    sendStartSequence(robot_id, sequence_id);
                }
                break;
            }
            
            default: {
                Serial.printf("Unknown serial command: %d\n", cmd_type);
                return;
            }
        }
        
        updatePythonConnection();
    }
}
#endif // ROLE_CONTROLLER
