#ifdef ROLE_ROBOT

#include "safety.h"
#include "motors.h"

// Safety state
volatile bool estopActive = true;  // Start in E-STOP
volatile bool motorsEnabled = false;

void activateEstop() {
    estopActive = true;
    motorsEnabled = false;
    //stopMotors();
}

void clearEstop() {
    estopActive = false;
}

bool isEstopActive() {
    return estopActive;
}

bool areMotorsEnabled() {
    return motorsEnabled;
}
#endif // ROLE_ROBOT
