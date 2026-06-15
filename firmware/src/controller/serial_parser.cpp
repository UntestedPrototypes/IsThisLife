#ifdef ROLE_CONTROLLER
#include "serial_parser.h"
#include "controller_config.h"
#include "packets.h"
#include "robot_commands.h"
#include "python_comm.h"
#include <Arduino.h>

void readSerialCommands() {
    if (Serial.available() > 0 && Serial.available() < 2) {
        delay(2);
        if(Serial.available() < 2) {
             while(Serial.available()) Serial.read(); // Flush garbage
             return; 
        }
    }
    
    while (Serial.available() >= 2) {
        uint8_t cmd_type = Serial.peek();
        size_t expected_len = 0;

        switch(cmd_type) {
            case PACKET_CONTROL:        expected_len = 9; break; 
            case PACKET_ESTOP:          expected_len = 2; break; 
            case PACKET_ESTOP_CLEAR:    expected_len = 2; break; 
            case PACKET_DISCOVER:       expected_len = 2; break;
            case PACKET_CONFIRM:        expected_len = 4; break; 
            case PACKET_START_SEQUENCE: expected_len = 3; break; 
            case PACKET_SET_SETTING:    expected_len = 22; break; // Type(1) + ID(1) + Key(16) + Float(4)
            case PACKET_GET_SETTING:    expected_len = 18; break; // Type(1) + ID(1) + Key(16)
            default:
                Serial.read(); 
                continue;
        }

        if (Serial.available() < expected_len) {
            return; 
        }

        cmd_type = Serial.read(); 
        uint8_t robot_id = Serial.read();

        switch(cmd_type) {
            case PACKET_CONTROL: {
                uint8_t mode = Serial.read(); 
                uint8_t buf[6];
                Serial.readBytes(buf, 6);
                
                uint16_t vx = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
                uint16_t vy = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
                uint16_t omega = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
                sendControlCommand(robot_id, mode, vx, vy, omega);
                break;
            }
            case PACKET_ESTOP_CLEAR:
                sendArmRobot(robot_id);
                break;
            case PACKET_ESTOP:
                sendEstopRobot(robot_id);
                break;
            case PACKET_DISCOVER:
                sendDiscover();
                break;
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
            case PACKET_SET_SETTING: {
                char key[17];
                Serial.readBytes(key, 16);
                key[16] = '\0';
                float value;
                Serial.readBytes((char*)&value, sizeof(float));
                sendSetSetting(robot_id, key, value);
                break;
            }
            // --- NEW: Handle Get Setting ---
            case PACKET_GET_SETTING: {
                char key[17];
                Serial.readBytes(key, 16);
                key[16] = '\0';
                sendGetSetting(robot_id, key);
                break;
            }
        }
        updatePythonConnection();
    }
}
#endif // ROLE_CONTROLLER