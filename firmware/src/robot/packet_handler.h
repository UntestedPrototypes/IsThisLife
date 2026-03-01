#ifdef ROLE_ROBOT

#ifndef PACKET_HANDLER_H
#define PACKET_HANDLER_H

#include <stdint.h>

typedef struct {
    uint8_t mac[6];
    uint8_t data[250];
    int len;
} RxPacket;

// ESP-NOW callback function
void onReceive(const uint8_t *mac, const uint8_t *data, int len);

// FreeRTOS queue processor
void processPacket(const uint8_t *mac, const uint8_t *data, int len);

#endif // PACKET_HANDLER_H
#endif // ROLE_ROBOT