#ifdef ROLE_TESTING

#include <Arduino.h>
#include <SCServo.h>
#include "Servo_ST3215.h"

// Pins for Waveshare ESP32 Servo Driver Board
#define S_RXD 16
#define S_TXD 17

// Initialize joint with Servo ID 1 and ID 2
Servo_ST3215 servo(1, 2);

void handleSerialCommands() {
    if (Serial.available() > 0) {
        char cmd = Serial.read(); 
        
        if (cmd == 'V' || cmd == 'v') {
            int val = Serial.parseInt(); 
            Serial.printf("Command: Set Velocity to %d\n", val);
            servo.setVelocity(val);
        } 
        else if (cmd == 'S' || cmd == 's') {
            Serial.println("Command: Stop Motors");
            servo.stop();
        }
        else if (cmd == 'D' || cmd == 'd') {
            Serial.println("Command: Disable Motors (Manual Mode)");
            servo.disableMotors();
        }
        else if (cmd == 'E' || cmd == 'e') {
            Serial.println("Command: Enable Motors");
            servo.enableMotors();
        }
        else if (cmd == '[') {
            servo.setMinLimitToCurrentPosition();
            Serial.printf("New MIN Limit set at: %ld\n", servo.getPosition(1));
        }
        else if (cmd == ']') {
            servo.setMaxLimitToCurrentPosition();
            Serial.printf("New MAX Limit set at: %ld\n", servo.getPosition(1));
        }
        else if (cmd == 'X' || cmd == 'x') { 
            Serial.println("Command: EMERGENCY STOP");
            servo.disableMotors();
        }
        else if (cmd == 'Z' || cmd == 'z') {
            Serial.println("Command: Setting Zero Point");
            servo.resetPositionToZero();
        }
        else if (cmd == 'L' || cmd == 'l') {
            long lim = Serial.parseInt();
            servo.setOuterLimits(-lim, lim);
            Serial.printf("New Outer Limits set: Min = %ld, Max = %ld\n", -lim, lim);
        }
        else {
            Serial.printf("Unknown command: %c\n", cmd);
        }
    }
}

void roleSetup() {
    Serial.begin(115200);
    delay(2000); // Allow time for initialization

    // Initialize servos on Serial1
    if (!servo.begin(Serial1, S_RXD, S_TXD)) {
        Serial.println("Failed to initialize servos!");
        return;
    }

    servo.setReverseSecond(false);
    servo.setLoadThreshold(600); // Stop if load > 60%
    
    // Set symmetric outer limits (e.g., +/- 2 rotations from start)
    long limitRange = 8192 * 2; 
    servo.setOuterLimits(-limitRange, limitRange);    
    
    servo.resetPositionToZero();          // Set current position as zero
    
    Serial.println("System Ready. Use V[val], S (Stop), E (E-Stop), or R (Reset).");
}

void roleLoop() {
    // 1. Mandatory background loop
    // Actively tracks encoder wraps, checks limits, and monitors load
    servo.update(); 

    // 2. Process incoming Serial commands
    handleSerialCommands();

    // 3. Print status periodically
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 200 && servo.safetyCheck()) {
        // Fetch and print the current continuous encoder position
        long currentPos_1 = servo.getPosition(1);
        long currentPos_2 = servo.getPosition(2);
        //Serial.printf("Continuous Position: %ld, %ld\n", currentPos_1, currentPos_2);
        
        lastUpdate = millis();
    }
}

#endif // ROLE_TESTING