#include "robot_preferences.h"
#include "robot_config.h"
#include <Preferences.h>

RobotSettings robotSettings;
Preferences prefs;

const char* PREF_NAMESPACE = "robot_mem";

void loadAllPreferences() {
    Serial.println("Loading saved preferences from Flash...");
    prefs.begin(PREF_NAMESPACE, true); 

    // --- Load IMU (Default to perfect 90-degree math) ---
    robotSettings.imu_off_w = prefs.getFloat("imu_w", 0.7071f);
    robotSettings.imu_off_x = prefs.getFloat("imu_x", 0.0f);
    robotSettings.imu_off_y = prefs.getFloat("imu_y", 0.7071f);
    robotSettings.imu_off_z = prefs.getFloat("imu_z", 0.0f);

    // --- Load Network Settings (Fallback to robot_config.h) ---
    robotSettings.robot_id = prefs.getUChar("robot_id", DEFAULT_ROBOT_ID); 
    
    size_t macLen = prefs.getBytesLength("ctrl_mac");
    if (macLen == 6) {
        prefs.getBytes("ctrl_mac", robotSettings.controller_mac, 6);
    } else {
        // Fallback to the default MAC if memory is empty
        robotSettings.controller_mac[0] = DEFAULT_MAC_0;
        robotSettings.controller_mac[1] = DEFAULT_MAC_1;
        robotSettings.controller_mac[2] = DEFAULT_MAC_2;
        robotSettings.controller_mac[3] = DEFAULT_MAC_3;
        robotSettings.controller_mac[4] = DEFAULT_MAC_4;
        robotSettings.controller_mac[5] = DEFAULT_MAC_5;
        
        /* NOTE: If you still have the "extern uint8_t controllerMac[6];" array 
           defined in robot.cpp, you could alternatively do:
           memcpy(robotSettings.controller_mac, controllerMac, 6); */
    }

    // --- Load Timing Settings (Fallback to robot_config.h) ---
    robotSettings.heartbeat_loss_timeout_ms = prefs.getUInt("hb_timeout", DEFAULT_HEARTBEAT_LOSS_TIMEOUT_MS);
    robotSettings.telemetry_interval = prefs.getUInt("tlm_int", DEFAULT_TELEMETRY_INTERVAL);
    robotSettings.confirm_timeout_ms = prefs.getUInt("cnf_timeout", DEFAULT_CONFIRM_TIMEOUT_MS);

    prefs.end(); 

    // Debug Print
    Serial.printf("  -> Robot ID: %u\n", robotSettings.robot_id);
    Serial.printf("  -> Controller MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  robotSettings.controller_mac[0], robotSettings.controller_mac[1], 
                  robotSettings.controller_mac[2], robotSettings.controller_mac[3], 
                  robotSettings.controller_mac[4], robotSettings.controller_mac[5]);
    Serial.printf("  -> HB Timeout: %u ms\n", robotSettings.heartbeat_loss_timeout_ms);
    Serial.printf("  -> TLM Interval: %u ms\n", robotSettings.telemetry_interval);
    Serial.printf("  -> CNF Timeout: %u ms\n", robotSettings.confirm_timeout_ms);
}

void saveIMUOffsets(float qw, float qx, float qy, float qz) {
    Preferences prefs;
    prefs.begin("robot_mem", false); 
    prefs.putFloat("imu_w", qw);
    prefs.putFloat("imu_x", qx);
    prefs.putFloat("imu_y", qy);
    prefs.putFloat("imu_z", qz);
    prefs.end();

    // Update the live global struct
    robotSettings.imu_off_w = qw;
    robotSettings.imu_off_x = qx;
    robotSettings.imu_off_y = qy;
    robotSettings.imu_off_z = qz;
}

void saveNetworkSettings(uint8_t* mac, uint8_t id) {
    prefs.begin(PREF_NAMESPACE, false); 
    prefs.putBytes("ctrl_mac", mac, 6);
    prefs.putUChar("robotSettings.robot_id", id);
    prefs.end();

    // Update live variables
    memcpy(robotSettings.controller_mac, mac, 6);
    robotSettings.robot_id = id;

    Serial.println("SUCCESS: Network settings saved to NVS Flash!");
}

void saveTimingSettings(uint32_t heartbeat, uint32_t telemetry, uint32_t confirm) {
    prefs.begin(PREF_NAMESPACE, false); 
    prefs.putUInt("hb_timeout", heartbeat);
    prefs.putUInt("tlm_int", telemetry);
    prefs.putUInt("cnf_timeout", confirm);
    prefs.end();

    // Update live variables
    robotSettings.heartbeat_loss_timeout_ms = heartbeat;
    robotSettings.telemetry_interval = telemetry;
    robotSettings.confirm_timeout_ms = confirm;

    Serial.println("SUCCESS: Timing settings saved to NVS Flash!");
}