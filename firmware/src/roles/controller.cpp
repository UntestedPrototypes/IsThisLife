// CONTROLLER
#ifdef ROLE_CONTROLLER

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "packets.h"

#define NUM_ROBOTS 1
#define CHANNEL 1

#define START_BYTE 0xAA
#define END_BYTE   0x55

uint8_t robotMacs[NUM_ROBOTS][6] = {
    {0x88, 0x57, 0x21, 0x6A, 0x09, 0x9C}
};

uint8_t heartbeatCounter = 0;
#define MAX_HEARTBEAT 256
#define HEARTBEAT_INTERVAL_MS 50
uint32_t heartbeatSentTime[NUM_ROBOTS][MAX_HEARTBEAT]; // store millis() when heartbeat sent


struct RobotTelemetry {
  uint32_t lastAckHeartbeat;
  uint32_t roundTripLatency; // RTT in ms
  uint8_t status;
  uint16_t batteryMv;
  int16_t motorTemp;
  uint8_t errorFlags;
};
RobotTelemetry robots[NUM_ROBOTS];

bool python_connected = false;
uint32_t lastPythonComm = 0;
uint32_t lastHeartbeatTime = 0;
#define PYTHON_TIMEOUT_MS 500

// -------------------- Helpers --------------------
void printRobotTelemetry(int idx) {
    RobotTelemetry &r = robots[idx];
    Serial.printf(
        "Robot %d: HB=%u lastSeen=%u status=%d battery=%u mv temp=%d err=0x%02X latency=%u ms\n",
        idx + 1,
        r.lastAckHeartbeat,
        r.roundTripLatency,
        r.status,
        r.batteryMv,
        r.motorTemp,
        r.errorFlags
    );
}

// -------------------- Forward Telemetry to Python --------------------
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


// -------------------- Robot → Controller --------------------
void onRobotReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len < sizeof(AckTelemetryPacket)) return;

    AckTelemetryPacket ack{};
    memcpy(&ack, data, sizeof(ack));

    int idx = ack.robot_id - 1;
    if (idx >= 0 && idx < NUM_ROBOTS) {
        RobotTelemetry &r = robots[idx];
        r.lastAckHeartbeat = ack.heartbeat;
        r.status = ack.status;
        r.batteryMv = ack.battery_mv;
        r.motorTemp = ack.motor_temp;
        r.errorFlags = ack.error_flags;

        // Fill ack structure so Python can see latency
        ack.latency_ms = 404; // Placeholder, can be calculated if needed

        forwardTelemetryToPython(ack);
    }
}


// -------------------- Send Heartbeat --------------------
void sendHeartbeat() {
    ControlPacket pkt{};
    pkt.type = PACKET_CONTROL;
    pkt.robot_id = 0; // broadcast to all robots
    pkt.vx = pkt.vy = pkt.omega = 0;
    pkt.heartbeat = heartbeatCounter;

    for (int i = 0; i < NUM_ROBOTS; i++) {
        if (esp_now_is_peer_exist(robotMacs[i])) {
            esp_now_send(robotMacs[i], (uint8_t*)&pkt, sizeof(pkt));
        }
    }

    heartbeatCounter = (heartbeatCounter + 1) & 0xFF;
}


// -------------------- Arm / E-STOP --------------------
void sendArmRobot(uint8_t robot_id) {
    if (robot_id < 1 || robot_id > NUM_ROBOTS) return;

    ControlPacket pkt{};
    pkt.type = PACKET_ESTOP_CLEAR;
    pkt.robot_id = robot_id;
    pkt.heartbeat = heartbeatCounter;
    
    pkt.vx = pkt.vy = pkt.omega = 0;
    
    esp_now_send(robotMacs[robot_id-1], (uint8_t*)&pkt, sizeof(pkt));
    Serial.printf("Sent ARM command to Robot %d\n", robot_id);
    heartbeatCounter = (heartbeatCounter + 1) & 0xFF;
}

void sendEstopRobot(uint8_t robot_id) {
    if (robot_id < 1 || robot_id > NUM_ROBOTS) return;

    ControlPacket pkt{};
    pkt.type = PACKET_ESTOP;
    pkt.robot_id = robot_id;
    pkt.heartbeat = heartbeatCounter;

    pkt.vx = pkt.vy = pkt.omega = 0;
    esp_now_send(robotMacs[robot_id-1], (uint8_t*)&pkt, sizeof(pkt));
    Serial.printf("Sent E-STOP to Robot %d\n", robot_id);
    heartbeatCounter = (heartbeatCounter + 1) & 0xFF;
}

// -------------------- Serial Commands --------------------
void readSerialCommands() {
    while (Serial.available() >= 2) {
        uint8_t cmd_type = Serial.read();
        uint8_t robot_id = Serial.read();

        switch(cmd_type) {
            case PACKET_CONTROL: {
                if (Serial.available() >= 3) {
                    int8_t vx = Serial.read();
                    int8_t vy = Serial.read();
                    int8_t omega = Serial.read();

                    ControlPacket pkt{};  // <-- safe now, inside {}
                    pkt.type = PACKET_CONTROL;
                    pkt.robot_id = robot_id;
                    pkt.vx = vx;
                    pkt.vy = vy;
                    pkt.omega = omega;
                    pkt.heartbeat = heartbeatCounter;
                    pkt.timestamp_ms = millis() & 0xFFFF;

                    if (robot_id == 0) { // broadcast
                        for (int i = 0; i < NUM_ROBOTS; i++)
                            if (esp_now_is_peer_exist(robotMacs[i]))
                                esp_now_send(robotMacs[i], (uint8_t*)&pkt, sizeof(pkt));
                    } else if (robot_id <= NUM_ROBOTS) {
                        esp_now_send(robotMacs[robot_id-1], (uint8_t*)&pkt, sizeof(pkt));
                    }
                }
                break;
            }
            case PACKET_ESTOP_CLEAR: {
                sendArmRobot(robot_id);
                Serial.printf("DEBUG: Sent ARM command to Robot %d\n", robot_id);
                break;
            }
            case PACKET_ESTOP: {
                sendEstopRobot(robot_id);
                break;
            }
            case PACKET_DISCOVER: {
                ControlPacket pkt{};
                pkt.type = PACKET_DISCOVER;
                pkt.robot_id = 0;
                for (int i = 0; i < NUM_ROBOTS; i++)
                    if (esp_now_is_peer_exist(robotMacs[i]))
                        esp_now_send(robotMacs[i], (uint8_t*)&pkt, sizeof(pkt));
                break;
            }
            default: {
                Serial.printf("Unknown serial command: %d\n", cmd_type);
                return;
            }
        }
        lastPythonComm = millis();
        python_connected = true;
    }
}

void connect_to_peer(uint8_t robot_id) {
    if (robot_id < 1 || robot_id > NUM_ROBOTS) {
        Serial.printf("Invalid robot_id %d\n", robot_id);
        return;
    }

    uint8_t* mac = robotMacs[robot_id - 1];

    // Check if peer already exists
    if (esp_now_is_peer_exist(mac)) {
        Serial.printf("Robot %d already connected as peer.\n", robot_id);
        return;
    }

    // Setup peer info
    esp_now_peer_info_t peerInfo{};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = CHANNEL;
    peerInfo.encrypt = false;

    // Try to add peer
    esp_err_t res = esp_now_add_peer(&peerInfo);
    if (res == ESP_OK) {
        Serial.printf("Robot %d successfully added as peer.\n", robot_id);
    } else if (res == ESP_ERR_ESPNOW_EXIST) {
        Serial.printf("Robot %d already exists as peer (race condition).\n", robot_id);
    } else if (res == ESP_ERR_ESPNOW_NOT_INIT) {
        Serial.println("ESP-NOW not initialized. Call esp_now_init() first.");
    } else if (res == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid arguments when adding peer.");
    } else if (res == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full, cannot add more peers.");
    } else {
        Serial.printf("Failed to add Robot %d, error=%d\n", robot_id, res);
    }
}


// -------------------- Setup --------------------
void roleSetup() {
    Serial.begin(115200);
    delay(500);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) return;
    esp_now_register_recv_cb(onRobotReceive);
    delay(100);

    // Print own MAC address
    Serial.print("Controller MAC address: ");
    Serial.println(WiFi.macAddress());

    // Try and connect to each robot as a peer
    for (uint8_t i = 1; i <= NUM_ROBOTS; i++) {
      connect_to_peer(i);
    }

    Serial.println("Controller setup complete, robots in E-STOP");
}

// -------------------- Loop --------------------
void roleLoop() {
    

    // Read commands from Python
    readSerialCommands();

    // If Python disconnected for too long, send E-STOP to all robots
    if (millis() - lastPythonComm > PYTHON_TIMEOUT_MS) {
        Serial.println("Python disconnected! Sending E-STOP to all robots.");
        if (python_connected == false) {   
          for (int i = 1; i <= NUM_ROBOTS; i++)
              sendEstopRobot(i);
              return;
        }
        python_connected = false;
    }

    // Send heartbeat only if python is connected (This is not pretty but works for now)
    if (millis() - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatTime = millis();
        sendHeartbeat();
    }
}

#endif
