#ifdef ROLE_ID_CHANGER
#include <Arduino.h>
#include <SCServo.h>

// Pins for Waveshare ESP32 Servo Driver Board
#define S_RXD 16
#define S_TXD 17

SMS_STS st;

// Configuration for Discovery and Parking
#define MAX_MOTORS 30
#define PARK_START_ID 253

struct MotorMap {
    int originalID;
    int parkedID;
    int finalID;
};

MotorMap motors[MAX_MOTORS];
int foundCount = 0;
bool isConfigured = false;

// --- UTILITY FUNCTIONS ---

void flushSerial() {
    while (Serial.available() > 0) {
        Serial.read();
    }
}

String getUserInput() {
    while (!Serial.available()) { delay(10); }
    String input = Serial.readStringUntil('\n');
    input.trim();
    return input;
}

void changeMotorID(int oldID, int newID) {
    st.unLockEprom(oldID);
    st.writeByte(oldID, SMS_STS_ID, newID);
    st.LockEprom(newID);
    delay(100); // Give EEPROM time to write
}

bool isKnownParkedID(int id) {
    for (int i = 0; i < foundCount; i++) {
        if (motors[i].parkedID == id) {
            return true;
        }
    }
    return false;
}

// --- CORE LOGIC ---

void scanAndParkMotors() {
    foundCount = 0;
    Serial.println("\n--- INITIATING DISCOVERY & PARKING SEQUENCE ---");
    Serial.println("Scanning full range 1-253. Parked motors go to 253 downwards.");
    
    while (foundCount < MAX_MOTORS) {
        int newlyFoundID = -1;
        
        // Scan the full valid ID range
        for (int i = 1; i < 254; i++) {
            // CRITICAL: Skip over the IDs where we have already parked motors
            if (isKnownParkedID(i)) {
                continue; 
            }

            delay(2);
            if (st.Ping(i) != -1) {
                newlyFoundID = i;
                break; // Stop at the first unparked motor we find
            }
        }
        
        if (newlyFoundID == -1) {
            // No more unparked motors found anywhere on the bus
            break;
        }
        
        // We found a motor. Record it and park it.
        int targetParkID = PARK_START_ID - foundCount;
        
        motors[foundCount].originalID = newlyFoundID;
        motors[foundCount].parkedID = targetParkID;
        motors[foundCount].finalID = foundCount + 1; // Default proposal (1, 2, 3...)
        
        Serial.printf("Found motor at ID %d. Parking at ID %d...\n", newlyFoundID, targetParkID);
        changeMotorID(newlyFoundID, targetParkID);
        
        foundCount++;
    }
    
    Serial.printf("\nDiscovery complete. %d motor(s) parked.\n", foundCount);
}

void processIDMapping() {
    if (foundCount == 0) {
        Serial.println("No motors found to map!");
        return;
    }

    bool mappingDone = false;
    while (!mappingDone) {
        Serial.println("\n--- MAPPING OVERVIEW ---");
        Serial.println("Index | Original ID | Currently Parked | Proposed Final ID");
        Serial.println("----------------------------------------------------------");
        for (int i = 0; i < foundCount; i++) {
            Serial.printf("  %02d  |     %03d     |       %03d        |       %03d\n", 
                i+1, motors[i].originalID, motors[i].parkedID, motors[i].finalID);
        }
        
        Serial.println("\nOptions:");
        Serial.println("  [A] Accept proposed placement");
        Serial.println("  [C] Custom placement (assign manually)");
        Serial.println("  [R] Revert/Cancel (Move all back to original IDs)");
        Serial.print("Select an option: ");
        
        String choice = getUserInput();
        choice.toUpperCase();
        
        if (choice == "A") {
            Serial.println("\nApplying final IDs...");
            for (int i = 0; i < foundCount; i++) {
                changeMotorID(motors[i].parkedID, motors[i].finalID);
                Serial.printf("Moved %d -> %d\n", motors[i].parkedID, motors[i].finalID);
            }
            mappingDone = true;
            isConfigured = true;
            Serial.println("Success! IDs updated.");
            
        } else if (choice == "C") {
            Serial.println("\n--- CUSTOM PLACEMENT ---");
            for (int i = 0; i < foundCount; i++) {
                Serial.printf("Enter new Final ID for motor parked at %d (Original: %d): ", 
                    motors[i].parkedID, motors[i].originalID);
                String newIdStr = getUserInput();
                int customID = newIdStr.toInt();
                if (customID > 0 && customID < 254) {
                    motors[i].finalID = customID;
                } else {
                    Serial.println("Invalid ID. Keeping proposed ID.");
                }
            }
            // Loops back to show the new table
            
        } else if (choice == "R") {
            Serial.println("\nReverting to original IDs...");
            for (int i = 0; i < foundCount; i++) {
                changeMotorID(motors[i].parkedID, motors[i].originalID);
                Serial.printf("Moved %d -> %d\n", motors[i].parkedID, motors[i].originalID);
            }
            mappingDone = true;
            isConfigured = false;
            foundCount = 0;
            Serial.println("Revert complete. Setup cancelled.");
        } else {
            Serial.println("Invalid choice. Please try again.");
        }
    }
}

// --- MANUAL ID UTILITY ---

void directManualIDChange() {
    Serial.println("\n--- DIRECT MANUAL ID CHANGE ---");
    Serial.println("WARNING: Ensure only the motors you want to interact with are powered.");
    
    Serial.print("Enter the CURRENT ID of the motor (1-253): ");
    int oldID = getUserInput().toInt();

    if (oldID < 1 || oldID > 253) {
        Serial.println("Invalid ID. Aborting.");
        return;
    }

    // Verify the motor actually exists
    if (st.Ping(oldID) == -1) {
        Serial.printf("Error: No motor responding at ID %d. Aborting.\n", oldID);
        return;
    }
    Serial.printf("Motor found at ID %d.\n", oldID);

    Serial.print("Enter the NEW ID for this motor (1-253): ");
    int newID = getUserInput().toInt();

    if (newID < 1 || newID > 253 || newID == oldID) {
        Serial.println("Invalid or identical ID. Aborting.");
        return;
    }

    // Collision check: See if target ID is already occupied
    if (st.Ping(newID) != -1) {
        Serial.printf("CRITICAL WARNING: Another motor is already occupying ID %d!\n", newID);
        Serial.print("If you proceed, BOTH motors will share the same ID. Proceed? (Y/N): ");
        String confirm = getUserInput();
        confirm.toUpperCase();
        if (confirm != "Y") {
            Serial.println("Aborting.");
            return;
        }
    }

    Serial.printf("Executing: ID %d -> %d...\n", oldID, newID);
    changeMotorID(oldID, newID);

    // Verify it moved
    delay(50);
    if (st.Ping(newID) != -1) {
        Serial.println("Success! Motor ID changed.");
    } else {
        Serial.println("Warning: Command sent, but motor is not responding at the new ID.");
    }
}

// --- QUICK SCAN & RENAME ---

void quickScanAndRename() {
    Serial.println("\n--- QUICK SCAN & RENAME FIRST MOTOR ---");
    Serial.println("Note: If multiple motors share the same ID, plug in ONLY ONE new motor at a time.");
    Serial.println("Scanning bus for the first responsive motor...");

    int foundID = -1;
    for (int i = 1; i < 254; i++) {
        if (st.Ping(i) != -1) {
            foundID = i;
            break; // Stop at the very first motor found
        }
        delay(2);
    }

    if (foundID == -1) {
        Serial.println("No motors found on the bus! Check wiring and power.");
        return;
    }

    Serial.printf("\n>>> Motor found at ID: %d <<<\n", foundID);
    Serial.print("Enter the NEW ID to assign (1-253), or 0 to cancel: ");
    
    int newID = getUserInput().toInt();

    if (newID == 0) {
        Serial.println("Cancelled by user.");
        return;
    }

    if (newID < 1 || newID > 253) {
        Serial.println("Invalid ID. Must be between 1 and 253. Aborting.");
        return;
    }

    if (newID == foundID) {
        Serial.println("Motor is already at this ID. Aborting.");
        return;
    }

    // Quick collision check
    if (st.Ping(newID) != -1) {
        Serial.printf("CRITICAL WARNING: Another motor is already occupying ID %d!\n", newID);
        Serial.println("Aborting to prevent ID conflicts.");
        return;
    }

    Serial.printf("Executing: ID %d -> %d...\n", foundID, newID);
    changeMotorID(foundID, newID);

    // Verify it moved
    delay(50);
    if (st.Ping(newID) != -1) {
        Serial.printf("Success! Motor is now ID %d.\n", newID);
    } else {
        Serial.println("Warning: Command sent, but motor is not responding at the new ID.");
    }
}

// --- SCAN BUS ONLY ---

void scanBusOnly() {
    Serial.println("\n--- SCANNING BUS FOR ALL MOTORS ---");
    Serial.println("Pinging IDs 1 to 253... Please wait.");
    int motorsFound = 0;

    for (int i = 1; i < 254; i++) {
        if (st.Ping(i) != -1) {
            Serial.printf(" -> Motor responding at ID: %d\n", i);
            motorsFound++;
        }
        delay(2); // Keep the bus stable
    }

    if (motorsFound == 0) {
        Serial.println("\nScan complete. No motors found.");
    } else {
        Serial.printf("\nScan complete. Total motors found: %d\n", motorsFound);
    }
}

// --- TESTING FUNCTIONS ---

void performSmallWiggle(int id) {
    Serial.printf("\nWiggling ID: %d...\n", id);
    
    // Make sure we are in position mode (0)
    st.unLockEprom(id);
    st.writeByte(id, SMS_STS_MODE, 0); 
    st.LockEprom(id);
    delay(50);
    
    // Read current position to anchor the wiggle
    int startPos = st.ReadPos(id);
    if (startPos < 0) {
        Serial.println("Failed to read position. Check connection.");
        return;
    }
    
    // Wiggle +/- 150 steps
    int offset = 150; 
    
    st.WritePosEx(id, startPos + offset, 1500, 50);
    delay(800);
    st.WritePosEx(id, startPos - offset, 1500, 50);
    delay(800);
    st.WritePosEx(id, startPos, 1500, 50);
    delay(800);
    
    Serial.println("Wiggle complete.");
}

void performTorqueTest(int id) {
    Serial.printf("\nTesting Torque on ID: %d...\n", id);
    st.EnableTorque(id, 1);
    Serial.println(">>> Torque is ON. Try to manually move the motor horn (it should resist).");
    Serial.println(">>> Press ENTER to turn torque OFF and finish.");
    
    getUserInput(); // Wait for user to press enter
    
    st.EnableTorque(id, 0);
    Serial.println("Torque is OFF.");
}

void testMenu() {
    if (!isConfigured || foundCount == 0) {
        Serial.println("\nNo active configuration found. Testing requires scanning first,");
        Serial.println("or you can test manually via other means.");
        return;
    }

    Serial.println("\n--- MOTOR TESTING ---");
    for (int i = 0; i < foundCount; i++) {
        int currentID = motors[i].finalID;
        if (st.Ping(currentID) == -1) {
            Serial.printf("Motor ID %d not responding, skipping...\n", currentID);
            continue;
        }

        bool testComplete = false;
        while (!testComplete) {
            Serial.printf("\nTesting Motor ID: %d\n", currentID);
            Serial.println("  [W] Small Wiggle Test (Position Mode)");
            Serial.println("  [T] Torque Resistance Test");
            Serial.println("  [S] Skip to next motor");
            Serial.print("Choice: ");
            
            String choice = getUserInput();
            choice.toUpperCase();
            
            if (choice == "W") {
                performSmallWiggle(currentID);
            } else if (choice == "T") {
                performTorqueTest(currentID);
            } else if (choice == "S") {
                Serial.println("Skipping...");
                testComplete = true;
            } else {
                Serial.println("Invalid choice.");
            }
        }
    }
    Serial.println("\nTesting sequence complete.");
}

// --- MENU & SETUP ---

void printMainMenu() {
    Serial.println("\n========================================");
    Serial.println("          MAIN MENU");
    Serial.println("========================================");
    Serial.println(" 1. Quick Scan & Rename First Found Motor");
    Serial.println(" 2. Scan Bus & Dynamically Assign IDs");
    Serial.println(" 3. Scan Bus Only (Read-Only)");
    Serial.println(" 4. View Current Mapped IDs");
    Serial.println(" 5. Test Motors (Wiggle / Torque)");
    Serial.println(" 6. Direct Manual ID Change");
    Serial.println(" 7. Reboot / Exit");
    Serial.println("========================================");
    Serial.print("Select an option: ");
}

void roleSetup() {
    Serial.begin(115200);

    unsigned long lastMsg = 0;
    while (Serial.available() <= 0) {
        if (millis() - lastMsg > 2000) {
            Serial.println("\n[WAITING] Serial Monitor detected.");
            Serial.println(">>> SEND ANY CHARACTER TO START <<<");
            lastMsg = millis();
        }
        delay(10);
    }
    flushSerial(); 

    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
    st.pSerial = &Serial1;
    delay(500);

    Serial.println("\n========================================");
    Serial.println("  ST3215 DYNAMIC ID CONFIGURATOR V3");
    Serial.println("========================================");
    
    printMainMenu();
}

void roleLoop() {
    if (Serial.available() > 0) {
        String input = getUserInput();
        
        if (input == "1") {
            quickScanAndRename();
            printMainMenu();
        } 
        else if (input == "2") {
            scanAndParkMotors();
            processIDMapping();
            printMainMenu();
        }
        else if (input == "3") {
            scanBusOnly();
            printMainMenu();
        }
        else if (input == "4") {
            if (!isConfigured) {
                Serial.println("\nNo active session mapping found.");
            } else {
                Serial.println("\n--- CURRENT ACTIVE IDs ---");
                for (int i = 0; i < foundCount; i++) {
                    Serial.printf("Motor %d: ID %d\n", i+1, motors[i].finalID);
                }
            }
            printMainMenu();
        }
        else if (input == "5") {
            testMenu();
            printMainMenu();
        }
        else if (input == "6") {
            directManualIDChange();
            printMainMenu();
        }
        else if (input == "7") {
            Serial.println("Rebooting...");
            delay(1000);
            ESP.restart();
        } 
        else {
            Serial.println("Invalid choice. Try again.");
            printMainMenu();
        }
    }
}
#endif // ROLE_ID_CHANGER