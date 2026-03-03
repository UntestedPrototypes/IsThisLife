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

bool isSequenceActive() {
    return sequenceActive;
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
    
    // Optional elapsed time tracker (useful for other time-based sequences)
    uint32_t elapsed = millis() - sequenceStepStartTime;
    
    switch(sequenceId) {
        
        // =========================================================
        // SEQUENCE 0: FULL IMU OFFSET CALIBRATION
        // =========================================================
        case SEQUENCE_CALIBRATION_FULL: {  
            switch(currentSequenceStep) {
                case 0:
                    Serial.println("SEQ: Full IMU Calibration Started.");
                    // Request confirmation. The sequence pauses until approved.
                    requestConfirmation(1, "Hold robot perfectly UPRIGHT. Approve when ready.");
                    currentSequenceStep++;
                    break;
                
                case 1:
                    if (waitingForConfirmation) return; 
                    
                    Serial.println("SEQ: Gathering UPRIGHT samples (approx 2 sec)...");
                    resetOffsetAccumulator();
                    currentSequenceStep++;
                    break;
                
                case 2:
                    // Since SystemTask runs at 50Hz, sampling 100 times takes ~2 seconds.
                    if (accumulateOffsetSample()) {
                        saveUprightVector();
                        Serial.println("SEQ: Upright vector saved.");
                        requestConfirmation(2, "Tilt robot 90-deg FORWARD (Nose to floor). Approve when ready.");
                        currentSequenceStep++;
                    }
                    break;
                
                case 3:
                    if (waitingForConfirmation) return; 
                    
                    Serial.println("SEQ: Gathering NOSE samples (approx 2 sec)...");
                    resetOffsetAccumulator();
                    currentSequenceStep++;
                    break;
                    
                case 4:
                    if (accumulateOffsetSample()) {
                        calculateAndSaveOffsets();
                        
                        // Tell the safety engine that we are fully calibrated and allowed to drive!
                        setCalibrationRequired(false);
                        
                        Serial.println("SEQ: Full calibration complete! Robot Unlocked.");
                        sequenceActive = false;
                        currentSequenceStep = 0;
                        sendTelemetry(PACKET_CONTROL, 0, 0); // Force UI update
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