#ifdef ROLE_ROBOT

#ifndef PACKET_HANDLER_H
#define PACKET_HANDLER_H

#include <stdint.h>

// ESP-NOW callback function
void onReceive(const uint8_t *mac, const uint8_t *data, int len);

#endif // PACKET_HANDLER_H
#endif // ROLE_ROBOT