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
}

bool clearEstop() {
    // Check if calibration is required
    if (isCalibrationRequired()) {
        return false;
    }

    // Check for critical hardware error flags
    uint32_t errorFlags = getErrorFlags();
    if (errorFlags == ERROR_BATT_OVERVOLTAGE ||
        errorFlags == ERROR_BATT_UNDERVOLTAGE ||
        errorFlags == ERROR_TEMP_OVERHEAT ||
        errorFlags == ERROR_SENSOR_OFFLINE) {
        return false;
    }

    // All checks passed - clear estop
    clearErrorFlags();
    estopActive = false;
    return true;
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
