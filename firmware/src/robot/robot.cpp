#ifdef ROLE_ROBOT

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "packets.h"
#include "config/robot_config.h"
#include "config/robot_preferences.h"
#include "comms/heartbeat.h"
#include "control/safety.h"
#include "control/motors.h"
#include "sensors/sensors.h"
#include "comms/telemetry.h"
#include "logic/confirmation.h"
#include "logic/sequence.h"
#include "comms/packet_handler.h"
#include "io/led_manager.h"
#include "io/serial_cli.h"
#include "utils/debug.h"

// RTOS Objects
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t stateMutex;
TaskHandle_t controlTaskHandle = NULL;
QueueHandle_t rxQueue;

// Task Prototypes
void systemTask(void *pvParameters);
void controlTask(void *pvParameters);
void cliTask(void *pvParameters);

// -------------------- Setup --------------------
void roleSetup() {
    Serial.begin(115200);
    initLed();

    loadAllPreferences();
    Serial.println("Robot setup starting...");

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
        Serial.println("ERROR: Initializing ESP-NOW failed");
        return;
    }
    esp_now_register_recv_cb(onReceive);
    
    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, robotSettings.controller_mac, 6);
    peer.channel = CHANNEL;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    // 3. Initialize Hardware
    if (!initSensors()) Serial.println("ERROR: Sensor init failed!");
    if (!initMotors()) Serial.println("ERROR: Motor init failed!");
    
    // 4. Spawn FreeRTOS Tasks
    xTaskCreatePinnedToCore(systemTask, "SystemTask", 8192, NULL, 2, NULL, 0); // Core 0
    xTaskCreatePinnedToCore(controlTask, "ControlTask", 8192, NULL, 3, &controlTaskHandle, 1); // Core 1
    xTaskCreatePinnedToCore(cliTask, "CLITask", 4096, NULL, 1, NULL, 0);
    DEBUG_PRINTLN("DEBUG: Setup complete. Entering CALIBRATION REQUIRED state. Tasks running.");
    delay(1000);
}

// -------------------- Tasks --------------------
void cliTask(void *pvParameters) {
    while(true) {
        handleSerialCommands();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void controlTask(void *pvParameters) {
    uint32_t notificationValue;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // Exactly 100Hz

    bool enabled = false;

    while(true) {
        // Calculate exactly how much time is left in our 10ms window
        TickType_t xNow = xTaskGetTickCount();
        TickType_t xTimeToWait = 0;
        if ((xNow - xLastWakeTime) < xFrequency) {
            xTimeToWait = xFrequency - (xNow - xLastWakeTime);
        }

        // Wait for E-STOP notification OR until the precise 10ms window expires
        xTaskNotifyWait(0x00, ULONG_MAX, &notificationValue, xTimeToWait);
        
        // Reset the window timer to right NOW
        xLastWakeTime = xTaskGetTickCount(); 

        // 1. Safely Read IMUs
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            // IMU Reads go here...
            xSemaphoreGive(i2cMutex);
        }

        // 2. Safely Get Target States
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            // ADD !isCalibrationRequired() to the enable condition
            enabled = motorsEnabled && !estopActive && !sequenceActive && !waitingForConfirmation && !isCalibrationRequired();
            xSemaphoreGive(stateMutex);
        }

        // 3. Command Hardware
        if (enabled) {
            executeMotorCommands();
        } else {
            stopMotors(); 
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

        printIMU();

        // 2. Manage State & Safety
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {

            if (isCalibrationRequired()) {
                if (isIMUCalibrated()) {
                    DEBUG_PRINTLN("DEBUG: IMUs are fully calibrated! Calibration state cleared.");
                    setCalibrationRequired(false);
                }
            }

            bool hb_ok = heartbeatValid();
            uint8_t errors = 0;

            // Safely read hardware flags without interrupting I2C flow on Core 1
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                errors = getErrorFlags();
                xSemaphoreGive(i2cMutex);
            }

            if ((errors != 0 || !hb_ok) && !estopActive) {
                if (errors != 0) Serial.printf("CRITICAL ERROR: 0x%02X\n", errors);
                if (!hb_ok) DEBUG_PRINTLN("DEBUG: E-STOP triggered due to lost heartbeat");
                
                estopActive = true;
                motorsEnabled = false;
                cancelConfirmation();
                if (errors != 0) stopSequence();
                
                // Wake up Core 1 immediately so it processes the new E-STOP state!
                if (controlTaskHandle != NULL) {
                    xTaskNotify(controlTaskHandle, 1, eSetBits);
                }
                sendTelemetry(PACKET_ESTOP, 0, 0); 
            }

            if (sequenceActive) runSequenceStep();
            checkConfirmationTimeout();

            // Set the LED mode based on priority
            if (errors != 0) {
                setLedMode(LED_PATTERN_3_SHORT); // E.g., 3 short flashes = Hardware Error
            } else if (isCalibrationRequired()) {
                setLedMode(LED_BLINK_FAST);      // Rapid blinking = Calibrating
            } else if (waitingForConfirmation) {
                setLedMode(LED_PATTERN_MIX);     // Mix = Waiting for user input
            } else if (sequenceActive) {
                setLedMode(LED_BLINK_SLOW);      // Slow pulse = Running automated task
            } else if (estopActive) {
                setLedMode(LED_PATTERN_3_LONG);  // E.g., 3 long flashes = E-STOP Active
            } else {
                setLedMode(LED_SOLID);           // Solid = Normal, armed, and ready to drive
            }

            xSemaphoreGive(stateMutex);
        }

        updateLed();

        // Run system loop at 50Hz
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// -------------------- Loop --------------------
void roleLoop() {
    vTaskDelete(NULL); // FreeRTOS manages execution now
}
#endif