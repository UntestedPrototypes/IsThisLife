#if defined(ROLE_TESTING) || defined(ROLE_ROBOT)
#include "Servo_ST3215.h"

// Static task handle for the servo update task
static TaskHandle_t servoTaskHandle = NULL;
static Servo_ST3215* servoInstance = NULL;

// RTOS Task - runs on Core 1
void servoUpdateTask(void* parameter) {
    Servo_ST3215* servo = (Servo_ST3215*)parameter;
    
    while (1) {
        // Run servo update every 5ms on core 1
        servo->update();
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

Servo_ST3215::Servo_ST3215(int servoID1, int servoID2) 
    : id1(servoID1), id2(servoID2), accel(200), 
      torqueThreshold(800), rawTruePos1(0), rawTruePos2(0),
      lastRaw1(0), lastRaw2(0), zeroOffset1(0), zeroOffset2(0), 
      currentVelCommand(0), minLimit(-999999), maxLimit(999999), 
      reverse2(true) {}

bool Servo_ST3215::begin(HardwareSerial& serialPort, int rx, int tx) {
    serialPort.begin(1000000, SERIAL_8N1, rx, tx); 
    st.pSerial = &serialPort; 
    delay(500);

    if(st.Ping(id1) == -1) {
        Serial.println("Error: Servo ID 1 not responding!");
        return false; 
    }
    if(st.Ping(id2) == -1) {
        Serial.println("Error: Servo ID 2 not responding!");
        return false; 
    }

    // Set to Wheel Mode (Speed Control) permanently
    st.setMode(id1, SMS_STS_MODE_WHEEL);
    st.setMode(id2, SMS_STS_MODE_WHEEL);
    delay(10); 

    
    // Initialize position tracking values from the 0-4095 range
    lastRaw1 = st.ReadPos(id1);
    lastRaw2 = st.ReadPos(id2);
    
    // Graceful fallback if the initial read fails (-1)
    if (lastRaw1 < 0) lastRaw1 = 0;
    if (lastRaw2 < 0) lastRaw2 = 0;
    
    rawTruePos1 = lastRaw1;
    rawTruePos2 = lastRaw2;
    resetPositionToZero(); // Set current position as zero reference
    
    // Store instance and create FreeRTOS task on Core 1
    servoInstance = this;
    xTaskCreatePinnedToCore(
        servoUpdateTask,      // Task function
        "ServoUpdate",        // Task name
        2048,                 // Stack size
        this,                 // Parameter (this instance)
        1,                    // Priority
        &servoTaskHandle,     // Task handle
        1                     // Core 1
    );

    return true;
}

// Monitors the 0-4095 encoder. If the jump is massive (e.g. 4095 -> 0), 
// it registers as a forward lap. If it's 0 -> 4095, it's a reverse lap.
void Servo_ST3215::trackWraps(int servoNum, int newRaw) {
    if (newRaw < 0) return; // Ignore failed reads

    if (servoNum == 1) {
        int delta = newRaw - lastRaw1;
        if (delta > 1024) delta -= 4096;      // Wrapped backward
        else if (delta < -1024) delta += 4096; // Wrapped forward
        rawTruePos1 += delta;
        lastRaw1 = newRaw;
    } else {
        int delta = newRaw - lastRaw2;
        if (delta > 1024) delta -= 4096;
        else if (delta < -1024) delta += 4096;
        rawTruePos2 += delta;
        lastRaw2 = newRaw;
    }
}

bool Servo_ST3215::safetyCheck() {
    for (int id : {id1, id2}) {
        // FeedBack returns -1 if the servo does not respond (disconnected)
        if (st.FeedBack(id) == -1) {
            Serial.printf("CRITICAL ERROR: Motor with ID %d not detected!\n", id);
            disableMotors();
            return false;
        }

        int load = st.ReadLoad(-1);
        if (abs(load) > torqueThreshold) {
            Serial.printf("SAFETY: Load threshold exceeded on ID%d: %d\n", id, load);
            disableMotors();
            return false;
        }
    }

    return true; // Only returns true if both motors respond and loads are safe
}

void Servo_ST3215::update() {
    // 1. Update position tracking
    if (st.FeedBack(id1) != -1) trackWraps(1, st.ReadPos(-1));
    if (st.FeedBack(id2) != -1) trackWraps(2, st.ReadPos(-1));

    // 2. Deceleration and Hard Stop (STEPPED - only write when factor changes significantly)
    if (currentVelCommand != 0) {
        long Pos1 = getPosition(id1);
        float speedFactor = 1.0;

        // --- A. Deceleration logic for overshooting ---
        if (currentVelCommand > 0 && Pos1 > (maxLimit - slowdownThreshold)) {
            speedFactor = (float)(maxLimit - Pos1) / slowdownThreshold;
        }
        else if (currentVelCommand < 0 && Pos1 < (minLimit + slowdownThreshold)) {
            speedFactor = (float)(Pos1 - minLimit) / slowdownThreshold;
        }

        // --- B. Only write if speedFactor changed significantly (or is zero) ---
        static float lastSpeedFactor = 1.0;
        const float FACTOR_THRESHOLD = 0.1;  // Only write if difference >= 10%
        
        if (speedFactor <= 0 || abs(speedFactor - lastSpeedFactor) >= FACTOR_THRESHOLD) {
            int baseSpeed = (int)(currentVelCommand * speedFactor);
            int s1 = baseSpeed;
            int s2 = (reverse2 ? -baseSpeed : baseSpeed);
            st.WriteSpe(id1, (s16)s1, (u8)accel);
            st.WriteSpe(id2, (s16)s2, (u8)accel);
            lastSpeedFactor = speedFactor;
            
            if (speedFactor <= 0) {
                currentVelCommand = 0;
                Serial.println("Target limit reached. Motors halted.");
                return;
            }
        }
    }

    // 3. Sync Correction and Motor Commands
    if (currentVelCommand != 0) {
        long Pos1 = getPosition(id1);
        long pos2 = getPosition(id2);
        float speedFactor = 1.0;

        // Recalculate speedFactor for sync logic
        if (currentVelCommand > 0 && Pos1 > (maxLimit - slowdownThreshold)) {
            speedFactor = (float)(maxLimit - Pos1) / slowdownThreshold;
        }
        else if (currentVelCommand < 0 && Pos1 < (minLimit + slowdownThreshold)) {
            speedFactor = (float)(Pos1 - minLimit) / slowdownThreshold;
        }

        // --- C. Sync Correction Logic ---
        long drift = Pos1 - pos2;
        int syncCorrection = (int)(drift * 0.2); // Proportional gain K_sync = 0.2

        // --- D. Calculate Final Speeds ---
        int baseSpeed = (int)(currentVelCommand * speedFactor);
        
        // Apply sync correction
        int s1 = baseSpeed - syncCorrection;
        int s2_val = baseSpeed + syncCorrection;
        int s2 = reverse2 ? -s2_val : s2_val;

        // --- E. Send Commands ---
        st.WriteSpe(id1, (s16)s1, (u8)accel);
        st.WriteSpe(id2, (s16)s2, (u8)accel);
    }
}

void Servo_ST3215::setVelocity(int targetVelocity) {
    long currentPos = getPosition(id1);
    float speedFactor = 1.0;

    // Deceleration logic for Max Limit
    if (targetVelocity > 0 && currentPos > (maxLimit - slowdownThreshold)) {
        long distanceToLimit = maxLimit - currentPos;
        speedFactor = (float)distanceToLimit / slowdownThreshold;
    }
    // Deceleration logic for Min Limit
    else if (targetVelocity < 0 && currentPos < (minLimit + slowdownThreshold)) {
        long distanceToLimit = currentPos - minLimit;
        speedFactor = (float)distanceToLimit / slowdownThreshold;
    }

    // Clamp factor between 0.0 and 1.0
    if (speedFactor < 0) speedFactor = 0;
    if (speedFactor > 1.0) speedFactor = 1.0;

    currentVelCommand = targetVelocity;
    
    // Apply the scaling factor to the final speed sent to servos
    int finalSpeed = (int)(targetVelocity * speedFactor);

    int s1 = finalSpeed;
    int s2 = reverse2 ? -finalSpeed : finalSpeed;
    st.WriteSpe(id1, (s16)s1, (u8)accel);
    st.WriteSpe(id2, (s16)s2, (u8)accel);
}

void Servo_ST3215::stop() {
    setVelocity(0);
}

void Servo_ST3215::disableMotors() {
    st.EnableTorque(id1, 0); 
    st.EnableTorque(id2, 0); 
    currentVelCommand = 0; // Clear velocity so it doesn't try to move when re-enabled
}

bool Servo_ST3215::enableMotors() {
    for (int id : {id1, id2}) {
        if (st.FeedBack(id) == -1) {
            Serial.printf("ERROR: Cannot enable motor ID%d because it is not responding!\n", id);
            return false;
        }
        st.setMode(id, SMS_STS_MODE_WHEEL);
        st.EnableTorque(id, 1);
    }

    return true;
}

void Servo_ST3215::setOuterLimits(long minLim, long maxLim) {
    minLimit = minLim;
    maxLimit = maxLim;
}

void Servo_ST3215::setMinLimitToCurrentPosition() {
    minLimit = getPosition(id1);
}

void Servo_ST3215::setMaxLimitToCurrentPosition() {
    maxLimit = getPosition(id1);
    
    // Safety check in case they were set backwards
    if (minLimit > maxLimit) {
        long temp = minLimit;
        minLimit = maxLimit;
        maxLimit = temp;
    }
}

long Servo_ST3215::getPosition(int id) {
    // Return absolute continuous position, applying the zero offset and reverse logic
    if (id == id1) {
        return rawTruePos1 - zeroOffset1;
    } else if (id == id2) {
        return reverse2 ? -(rawTruePos2 - zeroOffset2) : (rawTruePos2 - zeroOffset2);
    }
    return 0;
}

void Servo_ST3215::resetPositionToZero() {
    // 1. Read the current physical 0-4095 angle to prevent a 'jump' in the next update()
    int currentRaw1 = st.ReadPos(id1);
    int currentRaw2 = st.ReadPos(id2);
    
    // 2. Update lastRaw so the next trackWraps() starts from this physical point
    if (currentRaw1 >= 0) lastRaw1 = currentRaw1;
    if (currentRaw2 >= 0) lastRaw2 = currentRaw2;

    // 3. Reset the global/member continuous counters to exactly 0
    // This makes the current physical spot the absolute zero point
    rawTruePos1 = 0;
    rawTruePos2 = 0;

    // 4. Reset user offsets to 0 so getPosition() returns the rawTruePos directly
    zeroOffset1 = 0;
    zeroOffset2 = 0;
    
    Serial.println("Position Reset: Current location is now absolute 0.");
}

void Servo_ST3215::setLoadThreshold(int threshold) { torqueThreshold = threshold; }
void Servo_ST3215::setAcceleration(int a) { accel = a; }
void Servo_ST3215::setReverseSecond(bool rev) { reverse2 = rev; }
#endif // ROLE_TESTING