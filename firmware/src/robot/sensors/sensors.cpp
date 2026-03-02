#ifdef ROLE_ROBOT

#include "sensors.h"
#include "../config/robot_config.h"
#include <Wire.h>

bool sensorsReady = false;

bool initSensors() {
    // 1. Initialize the shared I2C bus
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000); 
    
    // 2. Initialize the dedicated subsystems
    bool diagOk = initSystemMonitor();
    bool imuOk = initIMU();
    
    // 3. System is ready only if both subsystems loaded
    sensorsReady = diagOk && imuOk;
    
    return sensorsReady;
}

#endif // ROLE_ROBOT