#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP9808.h>
#include <Adafruit_INA219.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ================= I2C PINS =================
#define SDA_PIN 21
#define SCL_PIN 22

// ================= SENSOR OBJECTS =================
Adafruit_MCP9808 mcp = Adafruit_MCP9808();
Adafruit_INA219  ina219;

Adafruit_BNO055 imuL = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 imuR = Adafruit_BNO055(56, 0x29);

// ================= STATUS FLAGS =================
bool mcp_ok  = false;
bool ina_ok  = false;
bool imuL_ok = false;
bool imuR_ok = false;

// ================= SETUP =================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n================ SENSOR SELF TEST ================");

    // I2C Init
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    delay(500);

    // ---------- MCP9808 ----------
    mcp_ok = mcp.begin();
    Serial.print("MCP9808 Temperature Sensor: ");
    Serial.println(mcp_ok ? "PASS" : "FAIL");

    // ---------- INA219 ----------
    ina_ok = ina219.begin();
    Serial.print("INA219 Voltage Sensor:     ");
    Serial.println(ina_ok ? "PASS" : "FAIL");

    // ---------- IMU LEFT ----------
    imuL_ok = imuL.begin();
    Serial.print("IMU LEFT (0x28):           ");
    Serial.println(imuL_ok ? "PASS" : "FAIL");

    // ---------- IMU RIGHT ----------
    imuR_ok = imuR.begin();
    Serial.print("IMU RIGHT (0x29):          ");
    Serial.println(imuR_ok ? "PASS" : "FAIL");

    Serial.println("=================================================");
    Serial.println("Starting live sensor output...\n");

    delay(1000);
}

// ================= MAIN LOOP =================
void loop() {

    // ---------- Temperature ----------
    if (mcp_ok) {
        float tempC = mcp.readTempC();
        Serial.print("Temp [C]: ");
        Serial.print(tempC, 2);
    } else {
        Serial.print("Temp [C]: ---");
    }

    Serial.print(" | ");

    // ---------- Battery Voltage ----------
    if (ina_ok) {
        float vBatt = ina219.getBusVoltage_V();
        Serial.print("Vbatt [V]: ");
        Serial.print(vBatt, 2);
    } else {
        Serial.print("Vbatt [V]: ---");
    }

    Serial.print(" | ");

    // ---------- IMU LEFT ----------
    if (imuL_ok) {
        imu::Vector<3> eulerL = imuL.getVector(Adafruit_BNO055::VECTOR_EULER);
        Serial.print("IMU L (R,P,Y): ");
        Serial.print(eulerL.x(), 1); Serial.print(", ");
        Serial.print(eulerL.y(), 1); Serial.print(", ");
        Serial.print(eulerL.z(), 1);
    } else {
        Serial.print("IMU L: ---");
    }

    Serial.print(" | ");

    // ---------- IMU RIGHT ----------
    if (imuR_ok) {
        imu::Vector<3> eulerR = imuR.getVector(Adafruit_BNO055::VECTOR_EULER);
        Serial.print("IMU R (R,P,Y): ");
        Serial.print(eulerR.x(), 1); Serial.print(", ");
        Serial.print(eulerR.y(), 1); Serial.print(", ");
        Serial.print(eulerR.z(), 1);
    } else {
        Serial.print("IMU R: ---");
    }

    Serial.println();
    delay(500);
}
