#ifdef ROLE_ROBOT

#include "serial_cli.h"
#include "../config/robot_preferences.h"
#include "../utils/debug.h"
#include <Arduino.h>

void handleSerialCommands() {
    if (!Serial.available()) return;
    
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // --- NEW: Catch empty input (pressing Enter) ---
    if (input.length() == 0) {
        if (dbg_paused) {
            resumeDebug();
        }
        return;
    }

    // Split input into command and arguments
    int spaceIndex = input.indexOf(' ');
    String cmd = input;
    String args = "";
    if (spaceIndex != -1) {
        cmd = input.substring(0, spaceIndex);
        args = input.substring(spaceIndex + 1);
        args.trim();
    }
    cmd.toUpperCase();

    if (cmd == "VIEW" || cmd == "HELP") {
        
        // Pause background debug prints indefinitely
        pauseDebug(); 
        
        Serial.println("\n=== Current Robot Settings ===");
        Serial.printf("Robot ID: %u\n", robotSettings.robot_id);
        Serial.printf("Controller MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
            robotSettings.controller_mac[0], robotSettings.controller_mac[1],
            robotSettings.controller_mac[2], robotSettings.controller_mac[3],
            robotSettings.controller_mac[4], robotSettings.controller_mac[5]);
        Serial.printf("Heartbeat Timeout: %u ms\n", robotSettings.heartbeat_loss_timeout_ms);
        Serial.printf("Telemetry Interval: %u pkts\n", robotSettings.telemetry_interval);
        Serial.printf("Confirm Timeout: %u ms\n", robotSettings.confirm_timeout_ms);
        Serial.printf("IMU Offsets (W,X,Y,Z): %.4f, %.4f, %.4f, %.4f\n",
            robotSettings.imu_off_w, robotSettings.imu_off_x, 
            robotSettings.imu_off_y, robotSettings.imu_off_z);
            
        Serial.printf("Debug General: %s\n", dbg_general ? "ON" : "OFF");
        Serial.printf("Debug IMU: %s\n", dbg_imu ? "ON" : "OFF");
        Serial.printf("Debug Packets: %s\n", dbg_packets ? "ON" : "OFF");
        Serial.println("==============================");
        
        Serial.println("Commands:");
        Serial.println("  VIEW                - Show settings");
        Serial.println("  SET_ID <id>         - Set Robot ID (e.g. SET_ID 5)");
        Serial.println("  SET_MAC <mac>       - Set Controller MAC (e.g. SET_MAC B0:CB:D8:C1:6B:E0)");
        Serial.println("  SET_HB <ms>         - Set Heartbeat timeout");
        Serial.println("  SET_TLM <pkts>      - Set Telemetry interval");
        Serial.println("  SET_CNF <ms>        - Set Confirm timeout");
        Serial.println("  SET_DBG_GEN <ON/OFF>- Toggle general debug messages");
        Serial.println("  SET_DBG_IMU <ON/OFF>- Toggle high-frequency IMU stream");
        Serial.println("  SET_DBG_PKT <ON/OFF>- Toggle incoming packet stream");
        
        // --- NEW: Exit Prompt ---
        Serial.println("\n*** Press [ENTER] to exit menu and resume logs ***\n");
    }
    else if (cmd == "SET_ID") {
        uint8_t new_id = args.toInt();
        saveNetworkSettings(robotSettings.controller_mac, new_id);
    }
    else if (cmd == "SET_MAC") {
        uint8_t mac[6];
        int parsed = sscanf(args.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
            &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
        if (parsed == 6) {
            saveNetworkSettings(mac, robotSettings.robot_id);
        } else {
            Serial.println("ERROR: Invalid MAC format. Use AA:BB:CC:DD:EE:FF");
        }
    }
    else if (cmd == "SET_HB") {
        uint32_t val = args.toInt();
        saveTimingSettings(val, robotSettings.telemetry_interval, robotSettings.confirm_timeout_ms);
    }
    else if (cmd == "SET_TLM") {
        uint32_t val = args.toInt();
        saveTimingSettings(robotSettings.heartbeat_loss_timeout_ms, val, robotSettings.confirm_timeout_ms);
    }
    else if (cmd == "SET_CNF") {
        uint32_t val = args.toInt();
        saveTimingSettings(robotSettings.heartbeat_loss_timeout_ms, robotSettings.telemetry_interval, val);
    }
    else if (cmd == "SET_DBG_GEN") {
        bool state = (args == "ON");
        saveDebugSettings(state, dbg_imu, dbg_packets);
    }
    else if (cmd == "SET_DBG_IMU") {
        bool state = (args == "ON");
        saveDebugSettings(dbg_general, state, dbg_packets);
    }
    else if (cmd == "SET_DBG_PKT") {
        bool state = (args == "ON");
        saveDebugSettings(dbg_general, dbg_imu, state);
    }
    else {
        Serial.println("Unknown command. Type HELP for a list of commands.");
    }
}

#endif // ROLE_ROBOT