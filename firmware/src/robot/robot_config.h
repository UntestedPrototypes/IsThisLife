#ifdef ROLE_ROBOT

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

// --- Robot Identification ---
#ifndef ROBOT_ID
    #define ROBOT_ID 1 
#endif

#define CHANNEL 1 

// Controller MAC Address
extern uint8_t controllerMac[6];

// Heartbeat monitoring constants
#define WINDOW_SIZE 10
#define MIN_VALID 1
#define WINDOW_TIME_MS 500
#define HEARTBEAT_LOSS_TIMEOUT_MS 500

// Telemetry constants
#define TELEMETRY_INTERVAL 5

// Confirmation timeout
#define CONFIRM_TIMEOUT_MS 30000

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