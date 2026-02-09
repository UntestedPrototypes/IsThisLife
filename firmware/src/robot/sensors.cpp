#ifdef ROLE_ROBOT

#include "sensors.h"

uint16_t readBattery() { 
    return 7400; 
}

int16_t readMotorTemp() { 
    return 35; 
}

uint8_t getErrorFlags() { 
    return 0; 
}
#endif // ROLE_ROBOT