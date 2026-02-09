#ifdef ROLE_ROBOT

#include "confirmation.h"
#include "robot_config.h"
#include "safety.h"
#include "motors.h"
#include "packets.h"
#include <Arduino.h>
#include <esp_now.h>

// Confirmation state
bool waitingForConfirmation = false;
uint8_t currentStepId = 0;
uint32_t confirmRequestTime = 0;

void requestConfirmation(uint8_t step_id, const char* message) {
    if (waitingForConfirmation) {
        Serial.println("DEBUG: Already waiting for confirmation, ignoring new request");
        return;
    }
    
    RequestConfirmPacket req{};
    req.type = PACKET_REQUEST_CONFIRM;
    req.robot_id = ROBOT_ID;
    req.heartbeat = 0; // Could use a separate counter
    req.step_id = step_id;
    strncpy(req.message, message, sizeof(req.message) - 1);
    req.message[sizeof(req.message) - 1] = '\0';  // Ensure null termination

    esp_err_t res = esp_now_send(controllerMac, (uint8_t*)&req, sizeof(req));
    
    if (res == ESP_OK) {
        waitingForConfirmation = true;
        currentStepId = step_id;
        confirmRequestTime = millis();
        motorsEnabled = false;  // Disable motors while waiting
        stopMotors();
        Serial.printf("DEBUG: Requested confirmation for step %d: %s\n", step_id, message);
    } else {
        Serial.printf("DEBUG: Failed to send confirmation request, error=%d\n", res);
    }
}

void handleConfirmation(uint8_t step_id, bool approved) {
    if (!waitingForConfirmation || step_id != currentStepId) {
        return;
    }
    
    Serial.printf("DEBUG: Received confirmation for step %d, approved=%d\n", step_id, approved);
    
    if (approved) {
        Serial.printf("DEBUG: Step %d approved, proceeding...\n", currentStepId);
        // TODO: Add your actual calibration logic here based on currentStepId
    } else {
        Serial.printf("DEBUG: Step %d denied, canceling\n", currentStepId);
    }
    
    waitingForConfirmation = false;
    currentStepId = 0;
}

void checkConfirmationTimeout() {
    if (waitingForConfirmation) {
        uint32_t now = millis();
        if (now - confirmRequestTime > CONFIRM_TIMEOUT_MS) {
            Serial.println("DEBUG: Confirmation request timed out");
            waitingForConfirmation = false;
            currentStepId = 0;
        }
    }
}

void cancelConfirmation() {
    if (waitingForConfirmation) {
        Serial.println("DEBUG: Pending confirmation canceled");
        waitingForConfirmation = false;
        currentStepId = 0;
    }
}
#endif // ROLE_ROBOT