#ifdef ROLE_CONTROLLER
#ifndef ESPNOW_HANDLER_H
#define ESPNOW_HANDLER_H

#include <stdint.h>

// ESP-NOW callback function
void onRobotReceive(const uint8_t *mac, const uint8_t *data, int len);

#endif // ESPNOW_HANDLER_H
#endif // ROLE_CONTROLLER
