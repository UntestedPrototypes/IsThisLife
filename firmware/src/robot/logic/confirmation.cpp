#ifdef ROLE_ROBOT

#include "confirmation.h"
#include "../config/robot_config.h"
#include "../config/robot_preferences.h"
#include "../utils/debug.h"
#include "../control/safety.h"
#include "../control/motors.h"
#include "packets.h"
#include <Arduino.h>
#include <esp_now.h>

// Confirmation state
bool waitingForConfirmation = false;
uint8_t currentStepId = 0;
uint32_t confirmRequestTime = 0;

void requestConfirmation(uint8_t step_id, const char* message) {
    if (waitingForConfirmation) {
        DEBUG_PRINTLN("DEBUG: Already waiting for confirmation, ignoring new request");
        return;
    }
    
    RequestConfirmPacket req{};
    req.type = PACKET_REQUEST_CONFIRM;
    req.robot_id = robotSettings.robot_id;
    req.heartbeat = 0; // Could use a separate counter
    req.step_id = step_id;
    strncpy(req.message, message, sizeof(req.message) - 1);
    req.message[sizeof(req.message) - 1] = '\0';  // Ensure null termination

    esp_err_t res = esp_now_send(robotSettings.controller_mac, (uint8_t*)&req, sizeof(req));
    
    if (res == ESP_OK) {
        waitingForConfirmation = true;
        currentStepId = step_id;
        confirmRequestTime = millis();
        motorsEnabled = false;  // Disable motors while waiting
        stopMotors();
        DEBUG_PRINTF("DEBUG: Requested confirmation for step %d: %s\n", step_id, message);
    } else {
        DEBUG_PRINTF("DEBUG: Failed to send confirmation request, error=%d\n", res);
    }
}

void handleConfirmation(uint8_t step_id, bool approved) {
    if (!waitingForConfirmation || step_id != currentStepId) {
        return;
    }
    
    DEBUG_PRINTF("DEBUG: Received confirmation for step %d, approved=%d\n", step_id, approved);
    
    if (approved) {
        DEBUG_PRINTF("DEBUG: Step %d approved, proceeding...\n", currentStepId);
        // TODO: Add your actual calibration logic here based on currentStepId
    } else {
        DEBUG_PRINTF("DEBUG: Step %d denied, canceling\n", currentStepId);
    }
    
    waitingForConfirmation = false;
    currentStepId = 0;
}

void checkConfirmationTimeout() {
    if (waitingForConfirmation) {
        uint32_t now = millis();
        if (now - confirmRequestTime > robotSettings.confirm_timeout_ms) {
            DEBUG_PRINTLN("DEBUG: Confirmation request timed out");
            waitingForConfirmation = false;
            currentStepId = 0;
        }
    }
}

void cancelConfirmation() {
    if (waitingForConfirmation) {
        DEBUG_PRINTLN("DEBUG: Pending confirmation canceled");
        waitingForConfirmation = false;
        currentStepId = 0;
    }
}
#endif // ROLE_ROBOT