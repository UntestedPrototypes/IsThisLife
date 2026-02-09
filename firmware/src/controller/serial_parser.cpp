#ifdef ROLE_CONTROLLER
#include "serial_parser.h"
#include "controller_config.h"
#include "packets.h"
#include "robot_commands.h"
#include "python_comm.h"
#include <Arduino.h>

void readSerialCommands() {
    while (Serial.available() >= 2) {
        uint8_t cmd_type = Serial.peek(); // Peek first to check availability of payload
        
        // Simple check: we need at least 2 bytes to read type + ID
        // But for CONTROL, we need 2 + 6 = 8 bytes total
        if (cmd_type == PACKET_CONTROL && Serial.available() < 8) {
            return; // Wait for more data
        }

        // Consume Type and ID
        cmd_type = Serial.read(); 
        uint8_t robot_id = Serial.read();

        switch(cmd_type) {
            case PACKET_CONTROL: {
                // Read 3 * uint16_t (6 bytes)
                uint8_t buf[6];
                Serial.readBytes(buf, 6);
                
                // Reassemble (assuming Little Endian from Python)
                uint16_t vx = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
                uint16_t vy = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
                uint16_t omega = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
                
                sendControlCommand(robot_id, vx, vy, omega);
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
                // Unknown command, consume and warn
                Serial.printf("Unknown serial command: %d\n", cmd_type);
                break;
            }
        }
        
        updatePythonConnection();
    }
}
#endif // ROLE_CONTROLLER