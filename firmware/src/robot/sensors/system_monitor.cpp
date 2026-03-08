#ifdef ROLE_ROBOT

#include "system_monitor.h"
#include "../config/robot_config.h"
#include "sensors.h"
#include "imu_handler.h"
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

bool initSystemMonitor() {
    if (!ina219.begin()) {
        Serial.println("ERROR: INA219 fail");
        return false;
    }
    return true;
}

uint16_t readBattery() {
    if (!sensorsReady) return 0;
    uint16_t voltage_mv = 0;
    
    // Safety lock for I2C Bus
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float busVoltage = ina219.getBusVoltage_V();
        voltage_mv = (uint16_t)(busVoltage * 1000.0f);
        xSemaphoreGive(i2cMutex);
    }
    return voltage_mv;
}

int16_t readTemp() {
    // Temperature is read from the main IMU
    return getIMUTemp();
}

static uint8_t latchedErrors = ERROR_NONE; // Persistent software error state

void raiseErrorFlag(uint8_t flag) {
    latchedErrors |= flag; // Bitwise OR to add the new error
}

void clearErrorFlags() {
    latchedErrors = ERROR_NONE; // Reset all latched errors
}

uint8_t getErrorFlags() {
    if (!sensorsReady) return ERROR_SENSOR_OFFLINE;
    
    // Start with any software errors already raised
    uint8_t flags = latchedErrors;
    
    uint16_t batt = readBattery();
    int16_t temp = readTemp();
    
    // Hardware Checks (Battery/Temp)
    if (batt > 0 && batt < 14800) flags |= ERROR_BATT_UNDERVOLTAGE;
    if (batt > 17000)             flags |= ERROR_BATT_OVERVOLTAGE;
    if (temp >= 40)               flags |= ERROR_TEMP_OVERHEAT;
    
    return flags;
}

#endif // ROLE_ROBOT