#ifdef ROLE_CONTROLLER
#include "python_comm.h"
#include "controller_config.h"
#include "robot_commands.h"
#include <Arduino.h>

// Python connection state
bool python_connected = false;
uint32_t lastPythonComm = 0;

void forwardTelemetryToPython(const TelemetryPacket& pkt) {
    uint8_t header[2] = {0xAA, 0x55};
    Serial.write(header, 2);
    Serial.write((const uint8_t*)&pkt, sizeof(pkt));
}

void forwardConfirmRequestToPython(const RequestConfirmPacket& req) {
    uint8_t header[2] = {0xFF, 0xAA};
    Serial.write(header, 2);
    Serial.write((const uint8_t*)&req, sizeof(req));
}

void forwardSettingResponseToPython(const SettingResponsePacket& pkt) {
    uint8_t header[2] = {0xCC, 0x33}; // Unique sync header for Setting Responses
    Serial.write(header, 2);
    Serial.write((const uint8_t*)&pkt, sizeof(pkt));
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
