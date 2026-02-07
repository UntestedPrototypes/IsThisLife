#ifdef ROLE_ROBOT

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "packets.h"

#define ROBOT_ID 1
#define CHANNEL 1
#define WINDOW_SIZE 10
#define MIN_VALID 1
#define WINDOW_TIME_MS 500
#define HEARTBEAT_LOSS_TIMEOUT_MS 200  // debounce period
uint32_t lastValidHeartbeatTime = 0;

uint8_t controlPacketCount = 0; // Counts received CONTROL packets
#define TELEMETRY_INTERVAL 5       // Send telemetry every 5 packets


uint32_t hbTimes[WINDOW_SIZE];
uint8_t hbCount = 0;

bool estopActive = true;  // Start in E-STOP
bool motorsEnabled = false;

uint8_t controllerMac[6] = {0x28,0x05,0xA5,0x6F,0x3D,0xC0};

// -------------------- Sensors --------------------
uint16_t readBattery() { return 7400; }
int16_t readMotorTemp() { return 35; }
uint8_t getErrorFlags() { return 0; }

// -------------------- Motors --------------------
void setMotors(int8_t vx, int8_t vy, int8_t omega) {
  //Serial.printf("DEBUG: Motors set - vx=%d vy=%d omega=%d\n", vx, vy, omega);
}
void stopMotors() { 
  //Serial.println("DEBUG: Motors stopped"); 
}

// -------------------- Telemetry --------------------
void sendAckTelemetry(uint8_t type, uint32_t hb, uint16_t latency_ms) {
    AckTelemetryPacket ack{};
    ack.robot_id = ROBOT_ID;
    ack.acked_type = type;
    ack.heartbeat = hb;
    ack.status = estopActive ? STATUS_ESTOP : STATUS_OK;
    ack.battery_mv = readBattery();
    ack.motor_temp = readMotorTemp();
    ack.error_flags = getErrorFlags();
    ack.latency_ms = 808; // Placeholder, can be calculated if needed

    esp_err_t res = esp_now_send(controllerMac, (uint8_t*)&ack, sizeof(ack));
    //Serial.printf("DEBUG: Telemetry sent - type=%d heartbeat=%u status=%d result=%d\n", type, hb, ack.status, res);
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
            Serial.print("DEBUG: E-STOP received");
            estopActive = true;
            motorsEnabled = false;
            stopMotors();
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

          if (!estopActive) {
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
          break;
        default:
            Serial.printf("DEBUG: Unknown packet type %d\n", pkt.type);
            break;
    }
    Serial.printf(" - type=%d robot_id=%d vx=%d vy=%d omega=%d hb=%u\n", pkt.type, pkt.robot_id, pkt.vx, pkt.vy, pkt.omega, pkt.heartbeat);
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
    Serial.print("Controller MAC address: ");
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

    // Check heartbeat validity
    bool hb_ok = heartbeatValid();

    if (!hb_ok && !estopActive) {
        // Trigger E-STOP due to lost heartbeat
        Serial.println("DEBUG: E-STOP triggered due to lost heartbeat");
        estopActive = true;
        motorsEnabled = false;
        stopMotors();

        // Optionally send telemetry
        sendAckTelemetry(PACKET_ESTOP, 0, 0);
    }

    // Keep motors stopped if E-STOP is active
    if (estopActive && motorsEnabled) {
        motorsEnabled = false;
        stopMotors();
    }
}

#endif
