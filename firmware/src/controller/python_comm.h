#ifdef ROLE_CONTROLLER
#ifndef PYTHON_COMM_H
#define PYTHON_COMM_H

#include <stdint.h>
#include "packets.h"

// Python connection state
extern bool python_connected;
extern uint32_t lastPythonComm;

// Functions
void forwardTelemetryToPython(const TelemetryPacket &ack);
void forwardConfirmRequestToPython(const RequestConfirmPacket &req);
void updatePythonConnection();
bool isPythonConnected();
void checkPythonTimeout();

#endif // PYTHON_COMM_H
#endif // ROLE_CONTROLLER
