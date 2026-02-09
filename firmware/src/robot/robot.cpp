#ifdef ROLE_ROBOT

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "packets.h"
#include "robot_config.h"
#include "heartbeat.h"
#include "safety.h"
#include "motors.h"
#include "sensors.h"
#include "telemetry.h"
#include "confirmation.h"
#include "sequence.h"
#include "packet_handler.h"

// Controller MAC Address (defined in robot_config.h)
uint8_t controllerMac[6] = {0x28, 0x05, 0xA5, 0x6F, 0x3D, 0xC0};

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

    // Run sequence steps if active
    if (sequenceActive) {
        runSequenceStep();
    }

    // Check for confirmation timeout
    checkConfirmationTimeout();

    // Check heartbeat validity
    bool hb_ok = heartbeatValid();

    if (!hb_ok && !estopActive) {
        // Trigger E-STOP due to lost heartbeat
        Serial.println("DEBUG: E-STOP triggered due to lost heartbeat");
        activateEstop();
        cancelConfirmation();
        
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
