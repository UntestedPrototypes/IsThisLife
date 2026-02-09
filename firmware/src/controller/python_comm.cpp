#ifdef ROLE_CONTROLLER
#include "python_comm.h"
#include "controller_config.h"
#include "robot_commands.h"
#include <Arduino.h>

// Python connection state
bool python_connected = false;
uint32_t lastPythonComm = 0;

void forwardTelemetryToPython(const AckTelemetryPacket &ack) {
    // Send telemetry as text line
    Serial.printf(
        "ID=%d HB=%u STATUS=%d BATT=%u TEMP=%d ERR=0x%02X RTT=%u\n",
        ack.robot_id,
        ack.heartbeat,
        ack.status,
        ack.battery_mv,
        ack.motor_temp,
        ack.error_flags,
        ack.latency_ms
    );
}

void forwardConfirmRequestToPython(const RequestConfirmPacket &req) {
    // Send confirmation request as text line
    Serial.printf(
        "CONFIRM_REQ ID=%d STEP=%d MSG=%s\n",
        req.robot_id,
        req.step_id,
        req.message
    );
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
