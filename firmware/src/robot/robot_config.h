#ifdef ROLE_ROBOT

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

// Robot identification
// Allow platformio.ini to override this with -DROBOT_ID=X
#ifndef ROBOT_ID
    #define ROBOT_ID 99 // Default ID if not set in platformio.ini
#endif
#define CHANNEL 1 // Wifi channel (must match to controller channel)

// Controller MAC Address
extern uint8_t controllerMac[6];

// Heartbeat monitoring constants
#define WINDOW_SIZE 10
#define MIN_VALID 1
#define WINDOW_TIME_MS 500
#define HEARTBEAT_LOSS_TIMEOUT_MS 500  // debounce period

// Telemetry constants
#define TELEMETRY_INTERVAL 5  // Send telemetry every 5 packets

// Confirmation timeout
#define CONFIRM_TIMEOUT_MS 30000  // 30 second timeout

// --- PIN DEFINITIONS ---
#define MAIN_MOTOR_PIN 27
#define SERVO_L_PIN 25
#define SERVO_R_PIN 33
#define SDA_PIN 21
#define SCL_PIN 22

#endif // ROBOT_CONFIG_H
#endif // ROLE_ROBOT