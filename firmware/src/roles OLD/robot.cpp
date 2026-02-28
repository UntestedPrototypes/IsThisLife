#ifdef ROLE_ROBOT_OLD

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "packets.h"

#define ROBOT_ID 1
#define CHANNEL 1 // Wifi channel (must match to controller channel)
uint8_t controllerMac[6] = {0x28,0x05,0xA5,0x6F,0x3D,0xC0}; // MAC Adress of controller

// Heartbeat monitoring
#define WINDOW_SIZE 10
#define MIN_VALID 1
#define WINDOW_TIME_MS 500
#define HEARTBEAT_LOSS_TIMEOUT_MS 200  // debounce period
uint32_t lastValidHeartbeatTime = 0;
uint32_t hbTimes[WINDOW_SIZE];
uint8_t hbCount = 0;

// Packets and Telemetry
uint8_t controlPacketCount = 0; // Counts received CONTROL packets
#define TELEMETRY_INTERVAL 5       // Send telemetry every 5 packets


// Safety
bool estopActive = true;  // Start in E-STOP
bool motorsEnabled = false;

// ========== Sequence Execution State ==========
bool sequenceActive = false;
uint8_t currentSequenceStep = 0;
uint8_t sequenceId = 0;
uint32_t sequenceStepStartTime = 0;

// ========== Confirmation state (for optional confirmations during sequences) ==========
bool waitingForConfirmation = false;
uint8_t currentStepId = 0;
uint32_t confirmRequestTime = 0;
#define CONFIRM_TIMEOUT_MS 30000  // 30 second timeout


// -------------------- Sensors --------------------
uint16_t readBattery() { return 7400; }
int16_t readTemp() { return 35; }
uint8_t getErrorFlags() { return 0; }

// -------------------- Motors --------------------
void setMotors(int8_t vx, int8_t vy, int8_t omega) {
  //Serial.printf("DEBUG: Motors set - vx=%d vy=%d omega=%d\n", vx, vy, omega);
}
void stopMotors() { 
  //Serial.println("DEBUG: Motors stopped"); 
}

// ========== NEW: Request confirmation from controller ==========
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

// -------------------- Telemetry --------------------
void sendAckTelemetry(uint8_t type, uint32_t hb, uint16_t latency_ms) {
    AckTelemetryPacket ack{};
    ack.robot_id = ROBOT_ID;
    ack.acked_type = type;
    ack.heartbeat = hb;
    
    // Update status based on current state (priority order)
    if (estopActive) {
        ack.status = STATUS_ESTOP;
    } else if (waitingForConfirmation) {
        ack.status = STATUS_WAITING_CONFIRM;
    } else if (sequenceActive) {
        ack.status = STATUS_RUNNING_SEQUENCE;
    } else {
        ack.status = STATUS_OK;
    }
    
    ack.battery_mv = readBattery();
    ack.motor_temp = readTemp();
    ack.error_flags = getErrorFlags();
    ack.latency_ms = 808; // Placeholder, can be calculated if needed

    esp_err_t res = esp_now_send(controllerMac, (uint8_t*)&ack, sizeof(ack));
    //Serial.printf("DEBUG: Telemetry sent - type=%d heartbeat=%u status=%d result=%d\n", type, hb, ack.status, res);
}

// ========== Sequence Execution Engine ==========
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
                    requestConfirmation(1, "Start full calibration? This will disable the robot until complete.");  // Request confirmation before starting
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

// ========== NEW: Handle confirmation timeout ==========
void checkConfirmationTimeout() {
    if (!waitingForConfirmation) return;
    
    uint32_t elapsed = millis() - confirmRequestTime;
    if (elapsed >= CONFIRM_TIMEOUT_MS) {
        Serial.printf("DEBUG: Confirmation timeout for step %d, canceling\n", currentStepId);
        waitingForConfirmation = false;
        currentStepId = 0;
        // Optionally trigger E-STOP or just stay in current state
        // estopActive = true;  // Uncomment if you want to E-STOP on timeout
    }
}



// -------------------- Heartbeat --------------------
bool heartbeatValid() {
    uint32_t now = millis();
    uint8_t samples = min(hbCount, (uint8_t)WINDOW_SIZE);
    bool foundRecent = false;

    // Check if at least one heartbeat is within WINDOW_TIME_MS
    for (uint8_t i = 0; i < samples; i++) {
        uint8_t idx = (hbCount - 1 - i) % WINDOW_SIZE; // most recent first
        if ((int32_t)(now - hbTimes[idx]) <= WINDOW_TIME_MS) {
            foundRecent = true;
            break;
        }
    }

    if (foundRecent) {
        lastValidHeartbeatTime = now;  // reset timer
        return true;
    }

    // If no recent heartbeat, only return false if the timeout has elapsed
    if ((int32_t)(now - lastValidHeartbeatTime) >= HEARTBEAT_LOSS_TIMEOUT_MS) {
      Serial.println("DEBUG: Heartbeat lost");  
      return false;
    }

    // still within debounce period
    return true;
}

// -------------------- ESP-NOW Receive --------------------
void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len < 1) return;  // Need at least the type byte
    
    uint8_t pkt_type = data[0];
    
    // ========== NEW: Handle CONFIRM packet ==========
    if (pkt_type == PACKET_CONFIRM && len >= sizeof(ConfirmPacket)) {
        ConfirmPacket confirm{};
        memcpy(&confirm, data, sizeof(confirm));
        
        if (confirm.robot_id == ROBOT_ID && waitingForConfirmation && confirm.step_id == currentStepId) {
            Serial.printf("DEBUG: Received confirmation for step %d, approved=%d\n", confirm.step_id, confirm.approved);
            
            if (confirm.approved) {
                // Confirmation approved - proceed with calibration step
                Serial.printf("DEBUG: Step %d approved, proceeding...\n", currentStepId);
                // TODO: Add your actual calibration logic here based on currentStepId
                
            } else {
                // Confirmation denied - cancel
                Serial.printf("DEBUG: Step %d denied, canceling\n", currentStepId);
            }
            
            waitingForConfirmation = false;
            currentStepId = 0;
        }
        return;
    }
    
    // ========== NEW: Handle START_SEQUENCE packet ==========
    if (pkt_type == PACKET_START_SEQUENCE && len >= sizeof(StartSequencePacket)) {
        StartSequencePacket seq{};
        memcpy(&seq, data, sizeof(seq));
        
        if (seq.robot_id == ROBOT_ID) {
            if (!estopActive) {
                Serial.printf("DEBUG: Starting sequence ID %d\n", seq.sequence_id);
                sequenceActive = true;
                sequenceId = seq.sequence_id;
                currentSequenceStep = 0;
                sequenceStepStartTime = millis();
                motorsEnabled = false;  // Disable motors during sequence
                stopMotors();
            } else {
                Serial.println("DEBUG: Cannot start sequence - E-STOP active");
            }
        }
        return;
    }
    
    // Handle regular control packets
    if (len < sizeof(ControlPacket)) {
        Serial.printf("DEBUG: Received packet too small (%d bytes)\n", len);
        return;
    }
    
    ControlPacket pkt{};
    memcpy(&pkt, data, sizeof(pkt));

    //Serial.printf("DEBUG: Packet received - type=%d robot_id=%d vx=%d vy=%d omega=%d hb=%u\n", pkt.type, pkt.robot_id, pkt.vx, pkt.vy, pkt.omega, pkt.heartbeat);

    if (pkt.robot_id != 0 && pkt.robot_id != ROBOT_ID) return;

    uint32_t now = millis();
    hbTimes[hbCount % WINDOW_SIZE] = now;
    hbCount++;

    switch(pkt.type) {
        case PACKET_DISCOVER:
            Serial.print("DEBUG: DISCOVER received");
            sendAckTelemetry(pkt.type, pkt.heartbeat, pkt.timestamp_ms);
            break;
        case PACKET_ESTOP:
            Serial.println("DEBUG: E-STOP received");
            estopActive = true;
            motorsEnabled = false;
            stopMotors();
            // Cancel any pending confirmation or sequence
            if (waitingForConfirmation) {
                Serial.println("DEBUG: E-STOP canceled pending confirmation");
                waitingForConfirmation = false;
                currentStepId = 0;
            }
            if (sequenceActive) {
                Serial.println("DEBUG: E-STOP canceled sequence");
                sequenceActive = false;
                currentSequenceStep = 0;
            }
            sendAckTelemetry(pkt.type, pkt.heartbeat, pkt.timestamp_ms);
            break;
        case PACKET_ESTOP_CLEAR:
            Serial.print("DEBUG: E-STOP cleared / ARM received");
            estopActive = false;
            sendAckTelemetry(pkt.type, pkt.heartbeat, pkt.timestamp_ms);
            break;
        case PACKET_CONTROL:
          controlPacketCount++; // Increment counter

          if (controlPacketCount >= TELEMETRY_INTERVAL) {
              sendAckTelemetry(pkt.type, pkt.heartbeat, pkt.timestamp_ms);
              controlPacketCount = 0; // Reset counter
          }

          // ========== Block motor commands while running sequence or waiting for confirmation ==========
          if (sequenceActive || waitingForConfirmation) {
              motorsEnabled = false;
              stopMotors();
              if (sequenceActive) {
                  Serial.print("DEBUG: Sequence active, motors disabled");
              } else {
                  Serial.print("DEBUG: Waiting for confirmation, motors disabled");
              }
          } else if (!estopActive) {
              if (heartbeatValid()) {
                  motorsEnabled = true;
                  setMotors(pkt.vx, pkt.vy, pkt.omega);
                  Serial.print("DEBUG: CONTROL ACTIVE");
              } else {
                  motorsEnabled = false;
                  stopMotors();
                  Serial.print("DEBUG: Heartbeat invalid, stopping motors");
              }
          } else {
              Serial.print("DEBUG: E-STOP active ");
          }
          Serial.printf(" - type=%d robot_id=%d vx=%d vy=%d omega=%d hb=%u\n", pkt.type, pkt.robot_id, pkt.vx, pkt.vy, pkt.omega, pkt.heartbeat);
          break;
        default:
            Serial.printf("DEBUG: Unknown packet type %d\n", pkt.type);
            break;
    }
}

// -------------------- Setup --------------------
void roleSetup() {
    Serial.begin(115200);
    Serial.println("DEBUG: Robot setup starting...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("DEBUG: Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(onReceive);
    delay(100);

    // Print own MAC address
    Serial.print("Robot MAC address: ");
    Serial.println(WiFi.macAddress());

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, controllerMac, 6);
    peer.channel = CHANNEL;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("DEBUG: Failed to add controller as peer");
    } else {
        Serial.println("DEBUG: Controller added as peer");
    }

    Serial.println("DEBUG: Robot setup complete");
    delay(1000);
    Serial.println("DEBUG: Starting in E-STOP");
}

// -------------------- Loop --------------------
void roleLoop() {
    uint32_t now = millis();

    // ========== Run sequence steps if active ==========
    if (sequenceActive) {
        runSequenceStep();
    }

    // ========== Check for confirmation timeout ==========
    checkConfirmationTimeout();

    // Check heartbeat validity
    bool hb_ok = heartbeatValid();

    if (!hb_ok && !estopActive) {
        // Trigger E-STOP due to lost heartbeat
        Serial.println("DEBUG: E-STOP triggered due to lost heartbeat");
        estopActive = true;
        motorsEnabled = false;
        stopMotors();
        
        // Cancel any pending confirmation
        if (waitingForConfirmation) {
            Serial.println("DEBUG: Lost heartbeat canceled pending confirmation");
            waitingForConfirmation = false;
            currentStepId = 0;
        }

        // Optionally send telemetry
        sendAckTelemetry(PACKET_ESTOP, 0, 0);
    }

    // Keep motors stopped if E-STOP is active or running sequence
    if ((estopActive || sequenceActive || waitingForConfirmation) && motorsEnabled) {
        motorsEnabled = false;
        stopMotors();
    }
    
    // ========== EXAMPLE: Request confirmation (uncomment to test) ==========
    // This is where you'd call requestConfirmation() when you need it
    // Example: After 10 seconds, request confirmation for gyro calibration
    // static bool requested = false;
    // if (!requested && millis() > 10000) {
    //     requestConfirmation(1, "Calibrate gyro?");
    //     requested = true;
    // }
}

#endif