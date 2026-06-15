#ifdef ROLE_ROBOT

#include "serial_cli.h"
#include "../config/robot_preferences.h"
#include "../utils/debug.h"
#include "../control/safety.h" 
#include <Arduino.h>
#include <WiFi.h>

void handleSerialCommands() {
    if (!Serial.available()) return;
    
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() == 0) {
        if (dbg_paused) resumeDebug();
        return;
    }

    int spaceIndex = input.indexOf(' ');
    String cmd = input;
    String args = "";
    if (spaceIndex != -1) {
        cmd = input.substring(0, spaceIndex);
        args = input.substring(spaceIndex + 1);
        args.trim();
    }
    cmd.toUpperCase();
    args.toUpperCase();

    if (cmd == "VIEW" || cmd == "HELP") {
        pauseDebug(); 
        
        Serial.println("\n=== Current Robot Settings ===");
        Serial.printf("Robot ID: %u\n", robotSettings.robot_id);
        Serial.printf("Robot MAC: %s\n", WiFi.macAddress().c_str());
        Serial.printf("Controller MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
            robotSettings.controller_mac[0], robotSettings.controller_mac[1],
            robotSettings.controller_mac[2], robotSettings.controller_mac[3],
            robotSettings.controller_mac[4], robotSettings.controller_mac[5]);
        Serial.printf("Heartbeat Timeout: %u ms\n", robotSettings.heartbeat_loss_timeout_ms);
        Serial.printf("Telemetry Interval: %u pkts\n", robotSettings.telemetry_interval);
        Serial.printf("Confirm Timeout: %u ms\n", robotSettings.confirm_timeout_ms);
        Serial.printf("Encoder Limits: [%d, %d]\n", 
            robotSettings.encoder_limit_min, robotSettings.encoder_limit_max);
        Serial.printf("IMU Offsets (W,X,Y,Z): %.4f, %.4f, %.4f, %.4f\n",
            robotSettings.imu_off_w, robotSettings.imu_off_x, 
            robotSettings.imu_off_y, robotSettings.imu_off_z);

        Serial.printf("Pitch PID [P:%.3f I:%.3f D:%.3f] Dir:%.1f\n", 
                  robotSettings.kp_pitch, robotSettings.ki_pitch, robotSettings.kd_pitch, robotSettings.pitch_dir);
        Serial.printf("Roll  PID [P:%.3f I:%.3f D:%.3f] Dir:%.1f\n", 
                  robotSettings.kp_roll, robotSettings.ki_roll, robotSettings.kd_roll, robotSettings.roll_dir);
        Serial.printf("OPitch PID [P:%.3f I:%.3f D:%.3f]\n",
                  robotSettings.kp_outer_pitch, robotSettings.ki_outer_pitch, robotSettings.kd_outer_pitch);
        Serial.printf("ORoll PID [P:%.3f I:%.3f D:%.3f]\n",
                  robotSettings.kp_outer_roll, robotSettings.ki_outer_roll, robotSettings.kd_outer_roll);
        Serial.printf("Filters   [D_Alpha:%.3f Out_Alpha:%.3f Deadband:%.3f]\n", 
                  robotSettings.d_alpha, robotSettings.out_alpha, robotSettings.deadband);
        Serial.printf("Debug General: %s\n", dbg_general ? "ON" : "OFF");
        Serial.printf("Debug IMU: %s\n", dbg_imu ? "ON" : "OFF");
        Serial.printf("Debug Pkt RX: %s\n", dbg_pkt_rx ? "ON" : "OFF"); 
        Serial.printf("Debug Pkt TX: %s\n", dbg_pkt_tx ? "ON" : "OFF"); 
        Serial.printf("Calibration Req: %s\n", isCalibrationRequired() ? "YES" : "NO (Dev Mode)");
        Serial.println("==============================");
        
        Serial.println("Commands:");
        Serial.println("  VIEW                - Show settings");
        Serial.println("  SET_ID <id>         - Set Robot ID");
        Serial.println("  SET_MAC <mac>       - Set Controller MAC");
        Serial.println("  SET_HB <ms>         - Set Heartbeat timeout");
        Serial.println("  SET_TLM <pkts>      - Set Telemetry interval");

        Serial.println("  SET_CNF <ms>        - Set Confirm timeout");
        Serial.println("  SET_ENC <min> <max> - Set Encoder limits");

        Serial.println("  SET_PID_PITCH  <P> <I> <D> <DIR> - Set Pitch constants");
        Serial.println("  SET_PID_ROLL   <P> <I> <D> <DIR> - Set Roll constants");
        Serial.println("  SET_PID_OPITCH <P> <I> <D>       - Set Outer Pitch constants");
        Serial.println("  SET_PID_OROLL  <P> <I> <D>       - Set Outer Roll constants");

        Serial.println("  SET_FILTER <D_ALPHA> <OUT_ALPHA> <DEADBAND> - Set Derivative, Output smoothing (0-1), and Error Deadband");
        
        Serial.println("  SET_DBG_GEN <ON/OFF>- Toggle general debug messages");
        Serial.println("  SET_DBG_IMU <ON/OFF>- Toggle high-frequency IMU stream");
        Serial.println("  SET_DBG_RX <ON/OFF> - Toggle incoming packet stream");
        Serial.println("  SET_DBG_TX <ON/OFF> - Toggle outgoing telemetry stream");
        Serial.println("  DEV_MODE <ON/OFF>   - Toggle Dev Mode (skips IMU calibration)");

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
    else if (cmd == "SET_ENC") {
        int32_t min_val = 0, max_val = 0;
        if (sscanf(args.c_str(), "%d %d", &min_val, &max_val) == 2) {
            saveEncoderLimits(min_val, max_val);
        } else {
            Serial.println("ERROR: Invalid format. Use: SET_ENC <min> <max>");
        }
    }
    else if (cmd == "SET_PID_PITCH") {
        float p, i, d, dir;
        int count = sscanf(args.c_str(), "%f %f %f %f", &p, &i, &d, &dir);
        if (count == 4) {
            savePidSettings(p, i, d, 
                           robotSettings.kp_roll, robotSettings.ki_roll, robotSettings.kd_roll, 
                           robotSettings.kp_outer_pitch, robotSettings.ki_outer_pitch, robotSettings.kd_outer_pitch,
                           robotSettings.kp_outer_roll, robotSettings.ki_outer_roll, robotSettings.kd_outer_roll,
                           dir, robotSettings.roll_dir);
            Serial.printf("SUCCESS: Pitch PID updated to P:%.4f I:%.4f D:%.4f Dir:%.1f\n", p, i, d, dir);
        } else {
            Serial.println("ERROR: Invalid format. Use: SET_PID_PITCH <P> <I> <D> <DIR>");
        }
    }
    else if (cmd == "SET_PID_ROLL") {
        float p, i, d, dir;
        int count = sscanf(args.c_str(), "%f %f %f %f", &p, &i, &d, &dir);
        if (count == 4) {
            savePidSettings(robotSettings.kp_pitch, robotSettings.ki_pitch, robotSettings.kd_pitch,
                           p, i, d, 
                           robotSettings.kp_outer_pitch, robotSettings.ki_outer_pitch, robotSettings.kd_outer_pitch,
                           robotSettings.kp_outer_roll, robotSettings.ki_outer_roll, robotSettings.kd_outer_roll,
                           robotSettings.pitch_dir, dir);
            Serial.printf("SUCCESS: Roll PID updated to P:%.4f I:%.4f D:%.4f Dir:%.1f\n", p, i, d, dir);
        } else {
            Serial.println("ERROR: Invalid format. Use: SET_PID_ROLL <P> <I> <D> <DIR>");
        }
    }
    else if (cmd == "SET_PID_OPITCH") {
        float p, i, d;
        int count = sscanf(args.c_str(), "%f %f %f", &p, &i, &d);
        if (count == 3) {
            savePidSettings(robotSettings.kp_pitch, robotSettings.ki_pitch, robotSettings.kd_pitch,
                           robotSettings.kp_roll, robotSettings.ki_roll, robotSettings.kd_roll,
                           p, i, d, 
                           robotSettings.kp_outer_roll, robotSettings.ki_outer_roll, robotSettings.kd_outer_roll,
                           robotSettings.pitch_dir, robotSettings.roll_dir);
            Serial.printf("SUCCESS: Outer Pitch PID updated to P:%.4f I:%.4f D:%.4f\n", p, i, d);
        } else {
            Serial.println("ERROR: Invalid format. Use: SET_PID_OPITCH <P> <I> <D>");
        }
    }
    else if (cmd == "SET_PID_OROLL") {
        float p, i, d;
        int count = sscanf(args.c_str(), "%f %f %f", &p, &i, &d);
        if (count == 3) {
            savePidSettings(robotSettings.kp_pitch, robotSettings.ki_pitch, robotSettings.kd_pitch,
                           robotSettings.kp_roll, robotSettings.ki_roll, robotSettings.kd_roll,
                           robotSettings.kp_outer_pitch, robotSettings.ki_outer_pitch, robotSettings.kd_outer_pitch,
                           p, i, d, 
                           robotSettings.pitch_dir, robotSettings.roll_dir);
            Serial.printf("SUCCESS: Outer Roll PID updated to P:%.4f I:%.4f D:%.4f\n", p, i, d);
        } else {
            Serial.println("ERROR: Invalid format. Use: SET_PID_OROLL <P> <I> <D>");
        }
    }
   else if (cmd == "SET_FILTER") {
        float d_alpha, out_alpha, deadband;
        if (sscanf(args.c_str(), "%f %f %f", &d_alpha, &out_alpha, &deadband) == 3) {
            d_alpha = constrain(d_alpha, 0.0f, 1.0f);
            out_alpha = constrain(out_alpha, 0.0f, 1.0f);
            if (deadband < 0.0f) deadband = 0.0f;
            
            saveFilterSettings(d_alpha, out_alpha, deadband);
            Serial.printf("SUCCESS: Filters updated to D_ALPHA:%.3f OUT_ALPHA:%.3f DEADBAND:%.3f\n", d_alpha, out_alpha, deadband);
        } else {
            Serial.println("ERROR: Invalid format. Use: SET_FILTER <D_ALPHA> <OUT_ALPHA> <DEADBAND>");
        }
    }
    else if (cmd == "SET_DBG_GEN") {
        bool state = (args == "ON");
        saveDebugSettings(state, dbg_imu, dbg_pkt_rx, dbg_pkt_tx);
    }
    else if (cmd == "SET_DBG_IMU") {
        bool state = (args == "ON");
        saveDebugSettings(dbg_general, state, dbg_pkt_rx, dbg_pkt_tx);
    }
    else if (cmd == "SET_DBG_RX") {
        bool state = (args == "ON");
        saveDebugSettings(dbg_general, dbg_imu, state, dbg_pkt_tx);
    }
    else if (cmd == "SET_DBG_TX") {
        bool state = (args == "ON");
        saveDebugSettings(dbg_general, dbg_imu, dbg_pkt_rx, state);
    }
    else if (cmd == "DEV_MODE") {
        if (args == "ON") {
            setCalibrationRequired(false);
            Serial.println("WARNING: Dev Mode ENABLED. IMU calibration requirement bypassed!");
        } else if (args == "OFF") {
            setCalibrationRequired(true);
            Serial.println("Dev Mode DISABLED. IMU calibration required.");
        } else {
            Serial.println("ERROR: Invalid format. Use: DEV_MODE <ON/OFF>");
        }
    }
    else {
        Serial.println("Unknown command. Type HELP for a list of commands.");
    }
}
#endif // ROLE_ROBOT