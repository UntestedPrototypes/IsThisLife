#ifdef ROLE_ROBOT

#include "safety.h"
#include "../sensors/system_monitor.h"
#include "motors.h"

// Safety state
volatile bool estopActive = true;  // Start in E-STOP
volatile bool motorsEnabled = false;
volatile bool calibrationRequired = true; // Start in calibration required state

bool isEstopActive() {
    return estopActive;
}

void activateEstop() {
    estopActive = true;
    motorsEnabled = false;
    //stopMotors();
}

bool clearEstop() {
    if (!isCalibrationRequired() && getErrorFlags() == ERROR_NONE) {
        estopActive = false;
        return true;
    }

    return false;
}

bool areMotorsEnabled() {
    return motorsEnabled;
}

bool isCalibrationRequired() {
    return calibrationRequired;
}

void setCalibrationRequired(bool required) {
    calibrationRequired = required;
}
#endif // ROLE_ROBOT
