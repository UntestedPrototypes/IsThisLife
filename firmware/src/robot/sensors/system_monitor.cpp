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

uint8_t getErrorFlags() {
    if (!sensorsReady) return ERROR_SENSOR_OFFLINE;
    uint8_t flags = ERROR_NONE;
    
    uint16_t batt = readBattery();
    int16_t temp = readTemp();
    
    // 1. Battery Under-Voltage (drops below 10V / 10000mV)
    if (batt > 0 && batt < 14800) { 
        flags |= ERROR_BATT_UNDERVOLTAGE;
    }
    
    // 2. Battery Over-Voltage (spikes above 13V / 13000mV)
    if (batt > 17000) { 
        flags |= ERROR_BATT_OVERVOLTAGE;
    }

    // 3. Over-Temperature (>= 40 Celsius)
    if (temp >= 40) {
        flags |= ERROR_TEMP_OVERHEAT;
    }
    
    return flags;
}

#endif // ROLE_ROBOT