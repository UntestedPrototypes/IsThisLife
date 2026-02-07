#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <cmath>
#include <Adafruit_MCP9808.h>
#include <Adafruit_INA219.h>

#include "angles.h"
#include "vectors.h"
#include "IMUaxisCalibration.h"
#include "IMU.h"
#include "communication.h"
#include "motors.h"


Adafruit_MCP9808 mcp;
Adafruit_INA219 ina219;

// ================= GLOBALS =================
uint8_t ESP32_ID = 2; // Used for ID '1' or raw 1
float x = 0.0f;
float y = 0.0f;
bool button_pressed = false;
bool emergency_stop;
extern uint32_t lastValidUdpTime;
const float radToDeg = 57.2958f;

float temp  = NAN;
float vBatt = NAN;
bool mcp_ok = false;
bool ina_ok = false;
// ================= CONFIGURATION =================
#define SDA_PIN 21
#define SCL_PIN 22

#define MAIN_MOTOR_PIN 27
#define SERVO_L_PIN 25
#define SERVO_R_PIN 33

// Motor Data Structures
Vec3 pid = Vec3(0.01f, 0.01f, 0.0f); 
MotorChannel mainMotor(MAIN_MOTOR_PIN, 1000, 2000, 1500, 150, 0,  false, pid, 40.0f);
MotorChannel servoL(SERVO_L_PIN, 500, 2500, 1500, 500, 75, false, pid, 15.0f);
MotorChannel servoR(SERVO_R_PIN, 500, 2500, 1500, 500, 75, true, pid, 15.0f);


// IMU setup
IMU imuL("left",  0x28);
IMU imuR("right", 0x29);

void apply_XY(float x, float y) {
    // Map x and y to motor commands
    mainMotor.command(x);
    servoL.command(y);
    servoR.command(y);
}

void killMotors() {
    // Emergency stop - cut all motor commands
    apply_XY(0.0f, 0.0f);
    sendToLaptop("EMERGENCY STOP: Motors killed");
}

// ================= SETUP =================
void setup() {
    // Start Comms
    wiFiInit();
    delay(500);

    mainMotor.begin();
    servoL.begin();
    servoR.begin();
    // I2C Setup
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000); 
    delay(1000);

    mcp_ok  = mcp.begin();
    ina_ok  = ina219.begin();

    checkI2CDevicesAndReport();

    // Start IMUs
    while (!imuL.begin() || !imuR.begin()) {
        sendToLaptop("IMU init failed - checking again...");
        delay(2000);
    }

    sendToLaptop("System Online. Ready for Direct Control.");
}

// ================= MAIN LOOP =================
void loop() {
    wiFiWatchdog();
    parseUdp(); // Updates x and y
    
    // Check for emergency stop or connection loss
    if (emergency_stop) {
        killMotors();
        sendToLaptop("WAITING FOR RESET...");
        // Stay in kill state until connection restored or system reset
        while (emergency_stop) {
            wiFiWatchdog();
            parseUdp();
            apply_XY(0.0f, 0.0f);
            delay(10);
        }
        sendToLaptop("Emergency stop cleared. Resuming operation.");
        return;
    }
    
    // Check for connection loss (2 second timeout)
    if (lastValidUdpTime > 0 && checkConnectionTimeout(500)) {
        killMotors();
        sendToLaptop("CONTROLLER CONNECTION LOST - EMERGENCY STOP!");
        emergency_stop = true;  // Force emergency stop
        return;
    }
    
    if (mcp_ok)  temp  = mcp.readTempC();
    if (ina_ok)  vBatt = ina219.getBusVoltage_V();
    static uint32_t lastUpdate = 0;

    // Check for temperature/battery warnings
    if ((isnan(temp) || temp  > 65.0f) || isnan(vBatt) || vBatt < 14.8f ) {
        if (isnan(temp) || temp  > 65.0f) {sendToLaptop("Warning: High Temperature!");}
        if (isnan(vBatt) || vBatt < 14.8f){sendToLaptop("Warning: Low Battery Voltage!"); }
        apply_XY(0.0f, 0.0f);
    }
    else if (millis() - lastUpdate > 10) {
        lastUpdate = millis();
        
        // Update IMU for telemetry only
        imuL.update();
        imuR.update();

        // Apply direct motor control based on controller input
        apply_XY(x, y);
    }

    // 5. Telemetry (2-second interval)
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 2000) {
        lastLog = millis();

        String telemetry = "X:" + String(x, 2) + 
                   " | Y:" + String(y, 2) + 
                    " | Temp:" + String(temp, 1) +
                    " | Vbatt:" + String(vBatt, 2) +
                    " | EmergencyStop:" + String(emergency_stop ? "ON" : "OFF");
                           
        
        sendToLaptop(telemetry);
    }
    delay(50);
}