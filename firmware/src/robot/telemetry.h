#ifdef ROLE_ROBOT

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

// Control packet counter
extern uint8_t controlPacketCount;

// Functions
void sendAckTelemetry(uint8_t type, uint32_t hb, uint16_t latency_ms);

#endif // TELEMETRY_H
#endif // ROLE_ROBOT