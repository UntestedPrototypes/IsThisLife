#ifdef ROLE_ID_CHANGER
#include <SCServo.h>

// Pins for Waveshare ESP32 Servo Driver Board
#define S_RXD 16
#define S_TXD 17

SMS_STS st;
int activeIDs[254]; 
int foundCount = 0;

int findNewMotorID(int excludedID = -1) {
    for (int i = 1; i < 254; i++) {
        if (i == excludedID) continue;
        delay(5); 
        if (st.Ping(i) != -1) return i;
    }
    return -1;
}

void scanBus() {
    foundCount = 0;
    Serial.println("\n--- Performing Full Bus Scan (1-253) ---");
    for (int i = 1; i < 254; i++) {
        delay(5);
        if (st.Ping(i) != -1) {
            activeIDs[foundCount] = i;
            foundCount++;
        }
    }
    Serial.printf("Scan Complete. Found %d motor(s).\n", foundCount);
}

void manualChange() {
    Serial.println("\n--- MANUAL ID CHANGE MODE ---");
    Serial.print("Current ID: ");
    while(!Serial.available());
    int oldID = Serial.parseInt();
    Serial.readString(); 
    
    Serial.print("New ID: ");
    while(!Serial.available());
    int newID = Serial.parseInt();
    Serial.readString();

    st.unLockEprom(oldID);
    st.writeByte(oldID, SMS_STS_ID, newID);
    st.LockEprom(newID);
    Serial.printf("ID Update Command Sent: %d -> %d.\n", oldID, newID);
}

void runWiggleTest(int id) {
    Serial.printf("\n--- WIGGLING ID: %d ---\n", id);
    Serial.println(">>> Type 's' and Enter to STOP.");

    // Safety Force: Enable Torque and Set Wheel Mode
    st.EnableTorque(id, 1);  // Ensure motor is powered
    st.WheelMode(id);        // Force into velocity mode
    delay(50);               // Small delay for servo to process mode change

    bool keepWiggling = true;
    while (keepWiggling) {
        st.WriteSpe(id, 400, 50);
        delay(600);
        st.WriteSpe(id, -400, 50);
        delay(600);
        int encoder = st.ReadPos(id);
        int voltage = st.ReadVoltage(id);
        int temp = st.ReadTemper(id);
        Serial.printf("Status Check - ID %d: Encoder: %d, Voltage: %.1fV, Temp: %dC\n", id, encoder, voltage/10.0, temp);

        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim();
            if (input.equalsIgnoreCase("s")) {
                keepWiggling = false;
                st.WriteSpe(id, 0, 0);
                Serial.printf("Motor %d Stopped.\n", id);
            }
        }
    }
}

void roleSetup() {
    Serial.begin(115200);

    // --- NEW: REPEATING START MESSAGE ---
    unsigned long lastMsg = 0;
    while (Serial.available() <= 0) {
        if (millis() - lastMsg > 2000) {
            Serial.println("\n[WAITING] Serial Monitor detected.");
            Serial.println(">>> SEND ANY CHARACTER TO START SETUP SCRIPT <<<");
            lastMsg = millis();
        }
        delay(10);
    }
    Serial.readString(); // Clear the buffer

    // Initialize Servo Serial
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
    st.pSerial = &Serial1;
    delay(500);

    Serial.println("\n========================================");
    Serial.println("   ST3215 DYNAMIC ID CONFIGURATOR");
    Serial.println("========================================");
    Serial.printf ("  ESP32 RX PIN: %d (Servo TX)\n", S_RXD);
    Serial.printf ("  ESP32 TX PIN: %d (Servo RX)\n", S_TXD);
    Serial.println("  BAUD RATE:    1,000,000");
    Serial.println("========================================");
    
    

    // --- STEP 1: FIND MOTOR A & PARK AT 253 ---
    int motorA_ID = -1;
    while (motorA_ID == -1) {
        Serial.println("\n[STEP 1] Connect ONLY the FIRST motor.");
        Serial.println(">>> Type 'y' and press Enter to scan...");
        while (!Serial.available());
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.equalsIgnoreCase("y")) {
            motorA_ID = findNewMotorID();
            if (motorA_ID == -1) Serial.println("ERROR: No motor detected. Check power/wiring.");
        }
    }

    Serial.printf("Motor A found at ID %d. Parking at temporary ID 253...\n", motorA_ID);
    st.unLockEprom(motorA_ID);
    st.writeByte(motorA_ID, SMS_STS_ID, 253);
    st.LockEprom(253);
    delay(500);

    // --- STEP 2: FIND MOTOR B & SET TO 1 ---
    int motorB_ID = -1;
    while (motorB_ID == -1) {
        Serial.println("\n[STEP 2] Connect the SECOND motor (Motor A is at ID 253).");
        Serial.println(">>> Type 'y' to scan...");
        while (!Serial.available());
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.equalsIgnoreCase("y")) {
            motorB_ID = findNewMotorID(253); 
            if (motorB_ID == -1) Serial.println("ERROR: Second motor not detected.");
        }
    }

    Serial.printf("Motor B found at ID %d. Setting to ID 1...\n", motorB_ID);
    st.unLockEprom(motorB_ID);
    st.writeByte(motorB_ID, SMS_STS_ID, 1);
    st.LockEprom(1);
    delay(500);

    // --- STEP 3: MOVE MOTOR A TO FINAL ID 2 ---
    Serial.println("\n[STEP 3] Finalizing IDs...");
    if (st.Ping(253) != -1) {
        Serial.println("Moving Motor A from 253 to final ID 2.");
        st.unLockEprom(253);
        st.writeByte(253, SMS_STS_ID, 2);
        st.LockEprom(2);
        delay(500);
    } else {
        Serial.println("CRITICAL ERROR: Parked motor (253) not responding!");
    }

    // --- STEP 4: VERIFY & WIGGLE ---
    scanBus();
    Serial.println("\nStarting Physical Identification...");
    for (int i = 0; i < foundCount; i++) {
        runWiggleTest(activeIDs[i]);
    }
    Serial.println("\nSetup Complete.");
}

void roleLoop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.equalsIgnoreCase("custom")) manualChange();
        if (input.equalsIgnoreCase("wiggle")) { 
            scanBus(); 
            for(int i=0; i<foundCount; i++) runWiggleTest(activeIDs[i]); 
        }
    }
}
#endif // ROLE_ID_CHANGER