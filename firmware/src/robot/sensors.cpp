#ifdef ROLE_ROBOT

#include "sensors.h"
#include "robot_config.h"
#include <Wire.h>

// Sensor Objects
Adafruit_INA219 ina219;
Adafruit_BNO055 imuL = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 imuR = Adafruit_BNO055(55, 0x29);

bool sensorsReady = false;

bool initSensors() {
    Serial.println("DEBUG: Initializing Sensors...");
    
    // Init I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000); 
    
    // Init Voltage Sensor
    if (!ina219.begin()) {
        Serial.println("ERROR: Failed to find INA219 chip");
    }

    // Init IMUs
    bool imuL_ok = imuL.begin();
    bool imuR_ok = imuR.begin();

    if (!imuL_ok) Serial.println("ERROR: Failed to find IMU Left (0x28)");
    if (!imuR_ok) Serial.println("ERROR: Failed to find IMU Right (0x29)");
    
    if (imuL_ok && imuR_ok) {
        // Use external crystal for better accuracy
        imuL.setExtCrystalUse(true);
        imuR.setExtCrystalUse(true);
        
        // Set to NDOF mode (Fusion mode)
        imuL.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_NDOF);
        imuR.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_NDOF);
        
        sensorsReady = true;
        Serial.println("DEBUG: Sensors initialized successfully");
    }

    return sensorsReady;
}

uint16_t readBattery() { 
    // Get bus voltage (V) and convert to mV
    float busvoltage = ina219.getBusVoltage_V();
    return (uint16_t)(busvoltage * 1000);
}

int16_t readMotorTemp() { 
    // Use the Left IMU temperature as a proxy for the system/motor temp
    // The BNO055 returns an int8_t (signed byte)
    if (!sensorsReady) return 0;
    return (int16_t)imuL.getTemp();
}

uint8_t getErrorFlags() { 
    uint8_t flags = 0;
    if (!sensorsReady) flags |= 0x01; // Sensor init failed
    
    // Example: Low voltage check
    if (readBattery() < 14800) flags |= 0x02; // Low Battery (<14.8V)
    
    return flags; 
}
#endif // ROLE_ROBOT