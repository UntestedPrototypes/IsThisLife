#ifdef ROLE_ROBOT
#include "packet_handler.h"
#include "../config/robot_config.h"
#include "../config/robot_preferences.h"
#include "../utils/debug.h"
#include "packets.h"
#include "telemetry.h"
#include "../control/safety.h"
#include "../control/pid_controller.h"
#include "heartbeat.h"
#include "../logic/confirmation.h"
#include "../logic/sequence.h"
#include <Arduino.h>
#include <esp_now.h>
#include <string.h>

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len < 1 || len > 250) return;
    uint8_t pkt_type = data[0];

    if (pkt_type == PACKET_ESTOP) {
        estopActive = true;
        motorsEnabled = false;
        if (controlTaskHandle != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xTaskNotifyFromISR(controlTaskHandle, 1, eSetBits, &xHigherPriorityTaskWoken);
            if(xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
        }
    }

    RxPacket rxPkt;
    memcpy(rxPkt.mac, mac, 6);
    memcpy(rxPkt.data, data, len);
    rxPkt.len = len;
    
    xQueueSendFromISR(rxQueue, &rxPkt, NULL);
}

void processPacket(const uint8_t *mac, const uint8_t *data, int len) {
    uint8_t pkt_type = data[0];
    
    // --- CONFIRMATIONS ---
    if (pkt_type == PACKET_CONFIRM && len >= sizeof(ConfirmPacket)) {
        ConfirmPacket confirm{};
        memcpy(&confirm, data, sizeof(confirm));
        if (confirm.robot_id == robotSettings.robot_id) {
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            handleConfirmation(confirm.step_id, confirm.approved);
            xSemaphoreGive(stateMutex);
        }
        return;
    }
    
    // --- SEQUENCES ---
    if (pkt_type == PACKET_START_SEQUENCE && len >= sizeof(StartSequencePacket)) {
        StartSequencePacket seq{};
        memcpy(&seq, data, sizeof(seq));
        if (seq.robot_id == robotSettings.robot_id) {
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            startSequence(seq.sequence_id);
            xSemaphoreGive(stateMutex);
        }
        return;
    }

    // --- SET SETTING ---
    if (pkt_type == PACKET_SET_SETTING && len >= sizeof(SetSettingPacket)) {
        SetSettingPacket setPkt{};
        memcpy(&setPkt, data, sizeof(setPkt));
        if (setPkt.robot_id == robotSettings.robot_id || setPkt.robot_id == 0) {
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            char safeKey[17];
            strncpy(safeKey, setPkt.key, 16);
            safeKey[16] = '\0';
            String key = String(safeKey);
            
            bool save_pid_needed = false;
            bool save_wobble_needed = false;

            if (key == "dev_mode") {
                robotSettings.dev_mode = (setPkt.value == 1.0f);
                setCalibrationRequired(!robotSettings.dev_mode);
            } 
            else if (key == "kp_pitch") { robotSettings.kp_pitch = setPkt.value; save_pid_needed = true; }
            else if (key == "ki_pitch") { robotSettings.ki_pitch = setPkt.value; save_pid_needed = true; }
            else if (key == "kd_pitch") { robotSettings.kd_pitch = setPkt.value; save_pid_needed = true; }

            else if (key == "kp_roll")  { robotSettings.kp_roll = setPkt.value; save_pid_needed = true; }
            else if (key == "ki_roll")  { robotSettings.ki_roll = setPkt.value; save_pid_needed = true; }
            else if (key == "kd_roll")  { robotSettings.kd_roll = setPkt.value; save_pid_needed = true; }

            else if (key == "kp_opitch") { robotSettings.kp_outer_pitch = setPkt.value; save_pid_needed = true; }
            else if (key == "ki_opitch") { robotSettings.ki_outer_pitch = setPkt.value; save_pid_needed = true; }
            else if (key == "kd_opitch") { robotSettings.kd_outer_pitch = setPkt.value; save_pid_needed = true; }

            else if (key == "kp_oroll") { robotSettings.kp_outer_roll = setPkt.value; save_pid_needed = true; }
            else if (key == "ki_oroll") { robotSettings.ki_outer_roll = setPkt.value; save_pid_needed = true; }
            else if (key == "kd_oroll") { robotSettings.kd_outer_roll = setPkt.value; save_pid_needed = true; }

            else if (key == "pitch_dir"){ robotSettings.pitch_dir = setPkt.value; save_pid_needed = true; }
            else if (key == "roll_dir") { robotSettings.roll_dir = setPkt.value; save_pid_needed = true; }
            else if (key == "wobble_gain")   { robotSettings.wobble_gain = setPkt.value; save_wobble_needed = true; }
            else if (key == "wobble_thresh") { robotSettings.wobble_threshold = setPkt.value; save_wobble_needed = true; }
            else if (key == "wobble_minvel") { robotSettings.wobble_min_vel = setPkt.value; save_wobble_needed = true; }

            if (save_pid_needed) {
                savePidSettings(robotSettings.kp_pitch, robotSettings.ki_pitch, robotSettings.kd_pitch,
                                robotSettings.kp_roll, robotSettings.ki_roll, robotSettings.kd_roll,
                                robotSettings.kp_outer_pitch, robotSettings.ki_outer_pitch, robotSettings.kd_outer_pitch,
                                robotSettings.kp_outer_roll, robotSettings.ki_outer_roll, robotSettings.kd_outer_roll,
                                robotSettings.pitch_dir, robotSettings.roll_dir);             
            }
            if (save_wobble_needed) {
                saveWobbleSettings(robotSettings.wobble_gain, robotSettings.wobble_threshold, robotSettings.wobble_min_vel);
            }
            xSemaphoreGive(stateMutex);
        }
        return;
    }

    // --- GET SETTING ---
    if (pkt_type == PACKET_GET_SETTING && len >= sizeof(GetSettingPacket)) {
        GetSettingPacket getPkt{};
        memcpy(&getPkt, data, sizeof(getPkt));
        if (getPkt.robot_id == robotSettings.robot_id) {
            char safeKey[17];
            strncpy(safeKey, getPkt.key, 16);
            safeKey[16] = '\0';
            String key = String(safeKey);

            float value = 0.0f;
            bool found = true;

            xSemaphoreTake(stateMutex, portMAX_DELAY);
            if (key == "dev_mode") value = robotSettings.dev_mode ? 1.0f : 0.0f; 
            else if (key == "kp_pitch") value = robotSettings.kp_pitch;
            else if (key == "ki_pitch") value = robotSettings.ki_pitch;
            else if (key == "kd_pitch") value = robotSettings.kd_pitch;

            else if (key == "kp_roll") value = robotSettings.kp_roll;
            else if (key == "ki_roll") value = robotSettings.ki_roll;
            else if (key == "kd_roll") value = robotSettings.kd_roll;

            else if (key == "kp_opitch") value = robotSettings.kp_outer_pitch;
            else if (key == "ki_opitch") value = robotSettings.ki_outer_pitch;
            else if (key == "kd_opitch") value = robotSettings.kd_outer_pitch;

            else if (key == "kp_oroll") value = robotSettings.kp_outer_roll;
            else if (key == "ki_oroll") value = robotSettings.ki_outer_roll;
            else if (key == "kd_oroll") value = robotSettings.kd_outer_roll;

            else if (key == "pitch_dir") value = robotSettings.pitch_dir;
            else if (key == "roll_dir") value = robotSettings.roll_dir;
            
            else if (key == "wobble_gain") value = robotSettings.wobble_gain;
            else if (key == "wobble_thresh") value = robotSettings.wobble_threshold;
            else if (key == "wobble_minvel") value = robotSettings.wobble_min_vel;
            else found = false;
            xSemaphoreGive(stateMutex);

            if (found) {
                SettingResponsePacket resp{};
                resp.type = PACKET_SETTING_RESPONSE;
                resp.robot_id = robotSettings.robot_id;
                resp.heartbeat = getPkt.heartbeat;
                strncpy(resp.key, safeKey, 16);
                resp.value = value;
                esp_now_send(robotSettings.controller_mac, (uint8_t*)&resp, sizeof(resp));
            }
        }
        return;
    }
    
    // --- STANDARD CONTROL PACKET PROCESSING ---
    if (len < sizeof(ControlPacket)) return;
    
    ControlPacket pkt{};
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.robot_id != 0 && pkt.robot_id != robotSettings.robot_id) return;

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    uint32_t now = millis();
    recordHeartbeat(now);

    switch(pkt.type) {
        case PACKET_DISCOVER:
            sendTelemetry(pkt.type, pkt.heartbeat, pkt.timestamp_ms);
            break;
        case PACKET_ESTOP:
            cancelConfirmation();
            stopSequence();
            sendTelemetry(pkt.type, pkt.heartbeat, pkt.timestamp_ms);
            break;
        case PACKET_ESTOP_CLEAR:
            clearEstop();
            sendTelemetry(pkt.type, pkt.heartbeat, pkt.timestamp_ms);
            break;
        case PACKET_CONTROL:
            controlPacketCount++;
            if (controlPacketCount >= robotSettings.telemetry_interval) {
                sendTelemetry(pkt.type, pkt.heartbeat, pkt.timestamp_ms);
                controlPacketCount = 0;
            }
            if (!isSequenceActive() && !waitingForConfirmation && !isEstopActive() && heartbeatValid() && !isCalibrationRequired()) {
                motorsEnabled = true;
                setControlMode(pkt.mode);
                setTargetVelocities(pkt.vx, pkt.vy, pkt.omega);
            }
            break;
    }
    xSemaphoreGive(stateMutex);
}
#endif // ROLE_ROBOT