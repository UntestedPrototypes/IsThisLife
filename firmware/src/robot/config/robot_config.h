#ifdef ROLE_ROBOT

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// --- Default Robot Identification ---
#define DEFAULT_ROBOT_ID 3

// (Optional) Define a default MAC here if you want a fallback, or rely on the extern array
#define DEFAULT_MAC_0 0xB0
#define DEFAULT_MAC_1 0xCB
#define DEFAULT_MAC_2 0xD8
#define DEFAULT_MAC_3 0xC1
#define DEFAULT_MAC_4 0x6B
#define DEFAULT_MAC_5 0xE0

// --- Default Timings ---
#define DEFAULT_HEARTBEAT_LOSS_TIMEOUT_MS 500
#define DEFAULT_TELEMETRY_INTERVAL 5
#define DEFAULT_CONFIRM_TIMEOUT_MS 10000

#define CHANNEL 1 
#define LED_PIN 2


// --- RTOS Sync Objects ---
extern SemaphoreHandle_t i2cMutex;
extern SemaphoreHandle_t stateMutex;
extern QueueHandle_t rxQueue;
extern TaskHandle_t controlTaskHandle;

// Heartbeat monitoring constants
#define WINDOW_SIZE 10
#define MIN_VALID 1
#define WINDOW_TIME_MS 500


// --- PIN DEFINITIONS ---
#define MAIN_MOTOR_PIN 27

// Serial Bus Pins for Waveshare ST3215
#define SERVO_RX_PIN 16
#define SERVO_TX_PIN 17

// I2C Pins
#define SDA_PIN 21
#define SCL_PIN 22

#endif // ROBOT_CONFIG_H
#endif // ROLE_ROBOT