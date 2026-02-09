#ifdef ROLE_CONTROLLER
#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

#include <stdint.h>

// Controller configuration
#define NUM_ROBOTS 1
#define CHANNEL 1

// Serial protocol markers
#define START_BYTE 0xAA
#define END_BYTE   0x55

// RC Control Constants
#define RC_MIN 1000
#define RC_NEUTRAL 1500
#define RC_MAX 2000

// Heartbeat configuration
#define MAX_HEARTBEAT 256
#define HEARTBEAT_INTERVAL_MS 50

// Python connection timeout
#define PYTHON_TIMEOUT_MS 500

// Robot MAC addresses
extern uint8_t robotMacs[NUM_ROBOTS][6];

// Heartbeat counter
extern uint8_t heartbeatCounter;

// Heartbeat timing tracking
extern uint32_t heartbeatSentTime[NUM_ROBOTS][MAX_HEARTBEAT];

#endif // CONTROLLER_CONFIG_H
#endif // ROLE_CONTROLLER