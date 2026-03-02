#ifdef ROLE_ROBOT

#include "sequence.h"
#include "confirmation.h"
#include "../comms/telemetry.h"
#include "../control/motors.h"
#include "../sensors/sensors.h"
#include "../control/safety.h"
#include "packets.h"
#include <Arduino.h>

// Sequence state
bool sequenceActive = false;
uint8_t currentSequenceStep = 0;
uint8_t sequenceId = 0;
uint32_t sequenceStepStartTime = 0;

void startSequence(uint8_t sequence_id) {
    if (!isEstopActive()) {
        Serial.printf("DEBUG: Starting sequence ID %d\n", sequence_id);
        sequenceActive = true;
        sequenceId = sequence_id;
        currentSequenceStep = 0;
        sequenceStepStartTime = millis();
        motorsEnabled = false;  // Disable motors during sequence
        stopMotors();
    } else {
        Serial.println("DEBUG: Cannot start sequence - E-STOP active");
    }
}

void stopSequence() {
    if (sequenceActive) {
        Serial.println("DEBUG: Sequence canceled");
        sequenceActive = false;
        currentSequenceStep = 0;
    }
}

void runSequenceStep() {
    if (!sequenceActive) return;
    
    uint32_t elapsed = millis() - sequenceStepStartTime;
    
    // ========== SEQUENCE ROUTER ==========
    // Add your custom sequences here!
    
    switch(sequenceId) {
        // ===== CALIBRATION SEQUENCES =====
        case 0: {  // SEQUENCE_CALIBRATION_FULL
            switch(currentSequenceStep) {
                case 0:
                    Serial.println("SEQ: Full calibration started - Initializing");
                    sendAckTelemetry(PACKET_CONTROL, 0, 0);
                    sequenceStepStartTime = millis();
                    requestConfirmation(1, "Start full calibration? This will disable the robot until complete.");
                    currentSequenceStep++;
                    break;
                
                case 1:
                    if (waitingForConfirmation) return;

                    if (elapsed > 2000) {
                        Serial.println("SEQ: Calibrating gyro");
                        // TODO: Add actual gyro calibration code here
                        sequenceStepStartTime = millis();
                        currentSequenceStep++;
                    }
                    break;
                
                case 2:
                    if (elapsed > 3000) {
                        Serial.println("SEQ: Testing motors");
                        // TODO: Add actual motor test code here
                        sequenceStepStartTime = millis();
                        currentSequenceStep++;
                    }
                    break;
                
                case 3:
                    if (elapsed > 2000) {
                        Serial.println("SEQ: Full calibration complete");
                        sequenceActive = false;
                        currentSequenceStep = 0;
                        sendAckTelemetry(PACKET_CONTROL, 0, 0);
                    }
                    break;
            }
            break;
        }
        
        case 1: {  // SEQUENCE_CALIBRATION_GYRO
            switch(currentSequenceStep) {
                case 0:
                    Serial.println("SEQ: Gyro calibration started");
                    sendAckTelemetry(PACKET_CONTROL, 0, 0);
                    sequenceStepStartTime = millis();
                    currentSequenceStep++;
                    break;
                
                case 1:
                    if (elapsed > 3000) {
                        Serial.println("SEQ: Gyro calibration complete");
                        // TODO: Add actual gyro calibration code here
                        sequenceActive = false;
                        currentSequenceStep = 0;
                        sendAckTelemetry(PACKET_CONTROL, 0, 0);
                    }
                    break;
            }
            break;
        }
        
        case 2: {  // SEQUENCE_CALIBRATION_MOTORS
            switch(currentSequenceStep) {
                case 0:
                    Serial.println("SEQ: Motor test started");
                    sendAckTelemetry(PACKET_CONTROL, 0, 0);
                    sequenceStepStartTime = millis();
                    currentSequenceStep++;
                    break;
                
                case 1:
                    if (elapsed > 2000) {
                        Serial.println("SEQ: Motor test complete");
                        // TODO: Add actual motor test code here
                        sequenceActive = false;
                        currentSequenceStep = 0;
                        sendAckTelemetry(PACKET_CONTROL, 0, 0);
                    }
                    break;
            }
            break;
        }
        
        // ===== DEMO SEQUENCES =====
        case 3: {  // SEQUENCE_DEMO_DANCE
            switch(currentSequenceStep) {
                case 0:
                    Serial.println("SEQ: Dance demo started - Move 1");
                    // TODO: setMotors(50, 0, 0);  // Forward
                    sequenceStepStartTime = millis();
                    currentSequenceStep++;
                    break;
                
                case 1:
                    if (elapsed > 1000) {
                        Serial.println("SEQ: Dance move 2");
                        // TODO: setMotors(0, 50, 0);  // Strafe right
                        sequenceStepStartTime = millis();
                        currentSequenceStep++;
                    }
                    break;
                
                case 2:
                    if (elapsed > 1000) {
                        Serial.println("SEQ: Dance move 3");
                        // TODO: setMotors(0, 0, 50);  // Spin
                        sequenceStepStartTime = millis();
                        currentSequenceStep++;
                    }
                    break;
                
                case 3:
                    if (elapsed > 1000) {
                        Serial.println("SEQ: Dance complete");
                        stopMotors();
                        sequenceActive = false;
                        currentSequenceStep = 0;
                    }
                    break;
            }
            break;
        }
        
        case 4: {  // SEQUENCE_SENSOR_TEST
            switch(currentSequenceStep) {
                case 0:
                    Serial.println("SEQ: Sensor test started");
                    Serial.printf("Battery: %d mV\n", readBattery());
                    Serial.printf("Temp: %d C\n", readTemp());
                    sequenceStepStartTime = millis();
                    currentSequenceStep++;
                    break;
                
                case 1:
                    if (elapsed > 2000) {
                        Serial.println("SEQ: Sensor test complete");
                        sequenceActive = false;
                        currentSequenceStep = 0;
                    }
                    break;
            }
            break;
        }
        
        case 5: {  // SEQUENCE_PATH_FOLLOW
            switch(currentSequenceStep) {
                case 0:
                    Serial.println("SEQ: Path following started - Point 1");
                    // TODO: Navigate to waypoint 1
                    sequenceStepStartTime = millis();
                    currentSequenceStep++;
                    break;
                
                case 1:
                    if (elapsed > 3000) {  // Simulate reaching waypoint
                        Serial.println("SEQ: Moving to Point 2");
                        // TODO: Navigate to waypoint 2
                        sequenceStepStartTime = millis();
                        currentSequenceStep++;
                    }
                    break;
                
                case 2:
                    if (elapsed > 3000) {
                        Serial.println("SEQ: Path following complete");
                        stopMotors();
                        sequenceActive = false;
                        currentSequenceStep = 0;
                    }
                    break;
            }
            break;
        }
        
        // ===== ADD YOUR CUSTOM SEQUENCES HERE =====
        default:
            Serial.printf("SEQ: Unknown sequence ID %d\n", sequenceId);
            sequenceActive = false;
            currentSequenceStep = 0;
            break;
    }
}
#endif // ROLE_ROBOT
