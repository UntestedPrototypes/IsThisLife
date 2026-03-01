#ifdef ROLE_CONTROLLER
#include "python_comm.h"
#include "controller_config.h"
#include "robot_commands.h"
#include <Arduino.h>

// Python connection state
bool python_connected = false;
uint32_t lastPythonComm = 0;

void forwardTelemetryToPython(const AckTelemetryPacket& ack) {
    uint8_t header[2] = {0xAA, 0x55};
    Serial.write(header, 2); // Send sync bytes
    Serial.write((uint8_t*)&ack, sizeof(AckTelemetryPacket)); // Send binary payload
    Serial.flush(); 
}

void forwardConfirmRequestToPython(const RequestConfirmPacket& req) {
    uint8_t header[2] = {0xFF, 0xAA}; // Different header for requests
    Serial.write(header, 2);
    Serial.write((uint8_t*)&req, sizeof(RequestConfirmPacket));
    Serial.flush();
}

void updatePythonConnection() {
    lastPythonComm = millis();
    python_connected = true;
}

bool isPythonConnected() {
    return python_connected;
}

void checkPythonTimeout() {
    if (millis() - lastPythonComm > PYTHON_TIMEOUT_MS) {
        if (python_connected) {
            Serial.println("Python disconnected! Sending E-STOP to all robots.");
            for (int i = 1; i <= NUM_ROBOTS; i++) {
                sendEstopRobot(i);
            }
            python_connected = false;
        }
    }
}
#endif // ROLE_CONTROLLER
