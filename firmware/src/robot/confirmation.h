#ifdef ROLE_ROBOT

#ifndef CONFIRMATION_H
#define CONFIRMATION_H

#include <stdint.h>

// Confirmation state
extern bool waitingForConfirmation;
extern uint8_t currentStepId;
extern uint32_t confirmRequestTime;

// Functions
void requestConfirmation(uint8_t step_id, const char* message);
void handleConfirmation(uint8_t step_id, bool approved);
void checkConfirmationTimeout();
void cancelConfirmation();

#endif // CONFIRMATION_H
#endif // ROLE_ROBOT