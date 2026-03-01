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

// Controller MAC Address
uint8_t controllerMac[6] = {0xB0, 0xCB, 0xD8, 0xC1, 0x6B, 0xE0};

// RTOS Objects
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t stateMutex;
TaskHandle_t controlTaskHandle = NULL;
QueueHandle_t rxQueue;

// Task Prototypes
void systemTask(void *pvParameters);
void controlTask(void *pvParameters);

// -------------------- Setup --------------------
void roleSetup() {
    Serial.begin(115200);
    Serial.println("DEBUG: Robot setup starting...");

    // 1. Initialize FreeRTOS Mutexes & Queues
    i2cMutex = xSemaphoreCreateMutex();
    stateMutex = xSemaphoreCreateMutex();
    rxQueue = xQueueCreate(10, sizeof(RxPacket));

    // 2. Wifi and ESP-NOW Initialization
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("DEBUG: Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(onReceive);
    
    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, controllerMac, 6);
    peer.channel = CHANNEL;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    // 3. Initialize Hardware
    if (!initSensors()) Serial.println("DEBUG: Sensor init failed!");
    if (!initMotors()) Serial.println("DEBUG: Motor init failed!");

    // 4. Spawn FreeRTOS Tasks
    xTaskCreatePinnedToCore(systemTask, "SystemTask", 8192, NULL, 2, NULL, 0); // Core 0
    xTaskCreatePinnedToCore(controlTask, "ControlTask", 8192, NULL, 3, &controlTaskHandle, 1); // Core 1

    Serial.println("DEBUG: Setup complete. Tasks running.");
    delay(1000);
}

// -------------------- Tasks --------------------

void controlTask(void *pvParameters) {
    uint32_t notificationValue;

    while(true) {
        // Wait up to 10ms for a wake-up notification (Fast-Path E-STOP)
        // If 10ms passes, it naturally unblocks to run the 100Hz motor loop.
        xTaskNotifyWait(0x00, ULONG_MAX, &notificationValue, pdMS_TO_TICKS(10));

        // 1. Safely Read IMUs (Core 1 exclusive)
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            // float roll, pitch, yaw;
            // getMainAxisOrientation(&roll, &pitch, &yaw);
            // readSecondaryIMUMotorAxisRotation();
            xSemaphoreGive(i2cMutex);
        }

        // 2. Safely Get Target States
        bool enabled = false;
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            enabled = motorsEnabled && !estopActive && !sequenceActive && !waitingForConfirmation;
            xSemaphoreGive(stateMutex);
        }

        // 3. Command Hardware strictly on Core 1
        if (enabled) {
            executeMotorCommands(); // Apply variables to hardware
        } else {
            stopMotors();           // Execute safely without Serial collision
        }
        
        updateMotorLoop();
    }
}

void systemTask(void *pvParameters) {
    RxPacket pkt;
    while(true) {
        // 1. Process all incoming ESP-NOW packets from the queue
        while (xQueueReceive(rxQueue, &pkt, 0) == pdTRUE) {
            processPacket(pkt.mac, pkt.data, pkt.len);
        }

        // 2. Manage State & Safety
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            bool hb_ok = heartbeatValid();
            uint8_t errors = 0;

            // Safely read hardware flags without interrupting I2C flow on Core 1
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                errors = getErrorFlags();
                xSemaphoreGive(i2cMutex);
            }

            if ((errors != 0 || !hb_ok) && !estopActive) {
                if (errors != 0) Serial.printf("CRITICAL ERROR: 0x%02X\n", errors);
                if (!hb_ok) Serial.println("DEBUG: E-STOP triggered due to lost heartbeat");
                
                estopActive = true;
                motorsEnabled = false;
                cancelConfirmation();
                if (errors != 0) stopSequence();
                
                // Wake up Core 1 immediately so it processes the new E-STOP state!
                if (controlTaskHandle != NULL) {
                    xTaskNotify(controlTaskHandle, 1, eSetBits);
                }
                sendAckTelemetry(PACKET_ESTOP, 0, 0); 
            }

            if (sequenceActive) runSequenceStep();
            checkConfirmationTimeout();

            xSemaphoreGive(stateMutex);
        }

        // Run system loop at 50Hz
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// -------------------- Loop --------------------
void roleLoop() {
    vTaskDelete(NULL); // FreeRTOS manages execution now
}
#endif