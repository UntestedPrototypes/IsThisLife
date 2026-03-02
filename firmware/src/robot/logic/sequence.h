#ifdef ROLE_ROBOT

#ifndef SEQUENCE_H
#define SEQUENCE_H

#include <stdint.h>

// Sequence state
extern bool sequenceActive;
extern uint8_t currentSequenceStep;
extern uint8_t sequenceId;
extern uint32_t sequenceStepStartTime;

// Functions
void startSequence(uint8_t sequence_id);
void stopSequence();
void runSequenceStep();

#endif // SEQUENCE_H
#endif // ROLE_ROBOT