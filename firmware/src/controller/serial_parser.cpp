#ifdef ROLE_CONTROLLER
#include "serial_parser.h"
#include "controller_config.h"
#include "packets.h"
#include "robot_commands.h"
#include "python_comm.h"
#include <Arduino.h>

void readSerialCommands() {
    // Process all available full packets in the buffer
    while (Serial.available() > 0) {
        uint8_t cmd_type = Serial.peek();
        size_t expected_len = 0;

        // Determine expected length based on Python packet_sender.py protocol
        switch(cmd_type) {
            case PACKET_CONTROL:        expected_len = 8; break; // Type(1)+ID(1)+3*U16(6)
            case PACKET_ESTOP:          expected_len = 2; break; // Type(1)+ID(1)
            case PACKET_ESTOP_CLEAR:    expected_len = 2; break; 
            case PACKET_DISCOVER:       expected_len = 2; break;
            case PACKET_CONFIRM:        expected_len = 4; break; // Type(1)+ID(1)+Step(1)+Apprv(1)
            case PACKET_START_SEQUENCE: expected_len = 3; break; // Type(1)+ID(1)+Seq(1)
            default:
                // Unknown byte: discard to find next valid header
                Serial.read(); 
                continue;
        }

        // If the full packet hasn't arrived yet, stop and wait
        if (Serial.available() < expected_len) {
            return; 
        }

        // Now safe to consume Type and ID
        cmd_type = Serial.read(); 
        uint8_t robot_id = Serial.read();

        switch(cmd_type) {
            case PACKET_CONTROL: {
                uint8_t buf[6];
                Serial.readBytes(buf, 6);
                
                // Reassemble Little Endian from Python
                uint16_t vx = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
                uint16_t vy = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
                uint16_t omega = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
                
                sendControlCommand(robot_id, vx, vy, omega);
                break;
            }
            
            case PACKET_ESTOP_CLEAR: {
                sendArmRobot(robot_id);
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
                uint8_t step_id = Serial.read();
                uint8_t approved = Serial.read(); 
                sendConfirmation(robot_id, step_id, approved != 0);
                break;
            }
            
            case PACKET_START_SEQUENCE: {
                uint8_t sequence_id = Serial.read();
                sendStartSequence(robot_id, sequence_id);
                break;
            }
        }
        
        updatePythonConnection();
    }
}
#endif // ROLE_CONTROLLER