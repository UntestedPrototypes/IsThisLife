#ifdef ROLE_CONTROLLER
#include "peer_management.h"
#include "controller_config.h"
#include <Arduino.h>
#include <esp_now.h>
#include <string.h>

void connectToPeer(uint8_t robot_id) {
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

void connectToAllPeers() {
    for (uint8_t i = 1; i <= NUM_ROBOTS; i++) {
        connectToPeer(i);
    }
}
#endif // ROLE_CONTROLLER
