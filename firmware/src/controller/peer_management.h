#ifdef ROLE_CONTROLLER
#ifndef PEER_MANAGEMENT_H
#define PEER_MANAGEMENT_H

#include <stdint.h>

// Functions
void connectToPeer(uint8_t robot_id);
void connectToAllPeers();

#endif // PEER_MANAGEMENT_H
#endif // ROLE_CONTROLLER
