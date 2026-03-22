#ifdef ROLE_ROBOT
#include "robot_preferences.h"
#include "robot_config.h"
#include "../utils/debug.h"
#include "../control/motors.h"
#include <Preferences.h>

RobotSettings robotSettings;
Preferences prefs;

const char* PREF_NAMESPACE = "robot_mem";

void loadAllPreferences() {
    Serial.println("Loading saved preferences from Flash...");
    prefs.begin(PREF_NAMESPACE, true);

    robotSettings.imu_off_w = prefs.getFloat("imu_w", 0.7071f);
    robotSettings.imu_off_x = prefs.getFloat("imu_x", 0.0f);
    robotSettings.imu_off_y = prefs.getFloat("imu_y", 0.7071f);
    robotSettings.imu_off_z = prefs.getFloat("imu_z", 0.0f);

    robotSettings.robot_id = prefs.getUChar("robot_id", DEFAULT_ROBOT_ID); 
    
    size_t macLen = prefs.getBytesLength("ctrl_mac");
    if (macLen == 6) {
        prefs.getBytes("ctrl_mac", robotSettings.controller_mac, 6);
    } else {
        robotSettings.controller_mac[0] = DEFAULT_MAC_0;
        robotSettings.controller_mac[1] = DEFAULT_MAC_1;
        robotSettings.controller_mac[2] = DEFAULT_MAC_2;
        robotSettings.controller_mac[3] = DEFAULT_MAC_3;
        robotSettings.controller_mac[4] = DEFAULT_MAC_4;
        robotSettings.controller_mac[5] = DEFAULT_MAC_5;
    }

    robotSettings.heartbeat_loss_timeout_ms = prefs.getUInt("hb_timeout", DEFAULT_HEARTBEAT_LOSS_TIMEOUT_MS);
    robotSettings.telemetry_interval = prefs.getUInt("tlm_int", DEFAULT_TELEMETRY_INTERVAL);
    robotSettings.confirm_timeout_ms = prefs.getUInt("cnf_timeout", DEFAULT_CONFIRM_TIMEOUT_MS);

    robotSettings.encoder_limit_min = prefs.getInt("enc_min", -12288);
    robotSettings.encoder_limit_max = prefs.getInt("enc_max", 12288);

    // Load PID values with current hardcoded defaults
    robotSettings.kp_pitch = prefs.getFloat("kp_p", 0.05f);
    robotSettings.ki_pitch = prefs.getFloat("ki_p", 0.001f);
    robotSettings.kd_pitch = prefs.getFloat("kd_p", 0.01f);
    
    robotSettings.kp_roll  = prefs.getFloat("kp_r", 0.05f);
    robotSettings.ki_roll  = prefs.getFloat("ki_r", 0.001f);
    robotSettings.kd_roll  = prefs.getFloat("kd_r", 0.01f);

    robotSettings.pitch_dir = prefs.getFloat("p_dir", 1.0f);
    robotSettings.roll_dir  = prefs.getFloat("r_dir", 1.0f);

    robotSettings.wobble_gain = prefs.getFloat("w_gain", 0.005f);
    robotSettings.wobble_threshold = prefs.getFloat("w_thresh", 15.0f);
    robotSettings.wobble_min_vel = prefs.getFloat("w_mvel", 30.0f);

    robotSettings.d_alpha = prefs.getFloat("d_alpha", 0.0f);
    robotSettings.out_alpha = prefs.getFloat("out_alpha", 0.0f);
    robotSettings.deadband = prefs.getFloat("deadband", 0.0f);

    dbg_general = prefs.getBool("dbg_gen", true);
    dbg_imu     = prefs.getBool("dbg_imu", false);
    dbg_pkt_rx  = prefs.getBool("dbg_prx", false); // <--- Load RX config
    dbg_pkt_tx  = prefs.getBool("dbg_ptx", false); // <--- Load TX config

    
    prefs.end(); 
    
    robotSettings.dev_mode = false;
    
    Serial.printf("  -> Robot ID: %u\n", robotSettings.robot_id);
    Serial.printf("  -> Controller MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  robotSettings.controller_mac[0], robotSettings.controller_mac[1], 
                  robotSettings.controller_mac[2], robotSettings.controller_mac[3], 
                  robotSettings.controller_mac[4], robotSettings.controller_mac[5]);
    Serial.printf("  -> HB Timeout: %u ms\n", robotSettings.heartbeat_loss_timeout_ms);
    Serial.printf("  -> TLM Interval: %u ms\n", robotSettings.telemetry_interval);
    Serial.printf("  -> CNF Timeout: %u ms\n", robotSettings.confirm_timeout_ms);
    Serial.printf("  -> Encoder Limits: [%d, %d]\n", robotSettings.encoder_limit_min, robotSettings.encoder_limit_max);
    Serial.printf("  -> Debug Settings - Gen: %s | IMU: %s | Pkt RX: %s | Pkt TX: %s\n", 
                  dbg_general ? "ON" : "OFF", dbg_imu ? "ON" : "OFF", 
                  dbg_pkt_rx ? "ON" : "OFF", dbg_pkt_tx ? "ON" : "OFF");
}

void saveIMUOffsets(float qw, float qx, float qy, float qz) { /* unchanged */ }
void saveNetworkSettings(uint8_t* mac, uint8_t id) { /* unchanged */ }
void saveTimingSettings(uint32_t heartbeat, uint32_t telemetry, uint32_t confirm) { /* unchanged */ }

void saveDebugSettings(bool gen, bool imu, bool pkt_rx, bool pkt_tx) {
    prefs.begin(PREF_NAMESPACE, false); 
    
    size_t w1 = prefs.putBool("dbg_gen", gen);
    size_t w2 = prefs.putBool("dbg_imu", imu);
    size_t w3 = prefs.putBool("dbg_prx", pkt_rx); // <--- Save RX
    size_t w4 = prefs.putBool("dbg_ptx", pkt_tx); // <--- Save TX
    
    prefs.end();

    dbg_general = gen;
    dbg_imu = imu;
    dbg_pkt_rx = pkt_rx;
    dbg_pkt_tx = pkt_tx;

    if (w1 == 0 || w2 == 0 || w3 == 0 || w4 == 0) {
        Serial.println("CRITICAL ERROR: Failed to write to NVS Flash! Memory might be corrupted or full.");
    } else {
        Serial.printf("SUCCESS: Debug settings saved! Gen:%s | IMU:%s | Pkt RX:%s | Pkt TX:%s\n", 
                      gen ? "ON" : "OFF", imu ? "ON" : "OFF", 
                      pkt_rx ? "ON" : "OFF", pkt_tx ? "ON" : "OFF");
    }
}

void saveEncoderLimits(int32_t min_limit, int32_t max_limit) {
    prefs.begin(PREF_NAMESPACE, false);
    prefs.putInt("enc_min", min_limit);
    prefs.putInt("enc_max", max_limit);
    prefs.end();

    robotSettings.encoder_limit_min = min_limit;
    robotSettings.encoder_limit_max = max_limit;
    setEncoderLimits(min_limit, max_limit);
}

void savePidSettings(float kpp, float kip, float kdp, float kpr, float kir, float kdr, float pdir, float rdir) {
    prefs.begin(PREF_NAMESPACE, false);
    prefs.putFloat("kp_p", kpp); prefs.putFloat("ki_p", kip); prefs.putFloat("kd_p", kdp);
    prefs.putFloat("kp_r", kpr); prefs.putFloat("ki_r", kir); prefs.putFloat("kd_r", kdr);
    prefs.putFloat("p_dir", pdir); prefs.putFloat("r_dir", rdir);
    prefs.end();

    // Update active runtime settings immediately
    robotSettings.kp_pitch = kpp; robotSettings.ki_pitch = kip; robotSettings.kd_pitch = kdp;
    robotSettings.kp_roll = kpr;  robotSettings.ki_roll = kir;  robotSettings.kd_roll = kdr;
    robotSettings.pitch_dir = pdir; robotSettings.roll_dir = rdir;
}

void saveFilterSettings(float d_alpha, float out_alpha, float deadband) {
    prefs.begin(PREF_NAMESPACE, false);
    prefs.putFloat("d_alpha", d_alpha);
    prefs.putFloat("out_alpha", out_alpha);
    prefs.putFloat("deadband", deadband); // <--- ADD THIS LINE
    prefs.end();

    // Update active runtime settings immediately
    robotSettings.d_alpha = d_alpha;
    robotSettings.out_alpha = out_alpha;
    robotSettings.deadband = deadband; // <--- ADD THIS LINE
}

void saveWobbleSettings(float gain, float threshold, float min_vel) {
    prefs.begin(PREF_NAMESPACE, false);
    prefs.putFloat("w_gain", gain);
    prefs.putFloat("w_thresh", threshold);
    prefs.putFloat("w_mvel", min_vel);
    prefs.end();

    // Update active runtime settings immediately
    robotSettings.wobble_gain = gain;
    robotSettings.wobble_threshold = threshold;
    robotSettings.wobble_min_vel = min_vel;
}
#endif // ROLE_ROBOT