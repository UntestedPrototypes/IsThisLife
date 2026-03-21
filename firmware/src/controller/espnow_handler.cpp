#ifdef ROLE_CONTROLLER
#include "espnow_handler.h"
#include "packets.h"
#include "robot_telemetry.h"
#include "python_comm.h"
#include <Arduino.h>
#include <string.h>

void onRobotReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len < 1) return;
    
    uint8_t detected_id = 0;
    if (len >= 2) {
        detected_id = data[1];
    }

    if (detected_id > 0) {
        checkNewRobot(detected_id);
    }
    
    uint8_t pkt_type = data[0];
    
    if (pkt_type == PACKET_REQUEST_CONFIRM && len >= sizeof(RequestConfirmPacket)) {
        RequestConfirmPacket req{};
        memcpy(&req, data, sizeof(req));
        forwardConfirmRequestToPython(req);
        return;
    }
    
    if (pkt_type == PACKET_TELEMETRY) {
        if (len < sizeof(TelemetryPacket)) return;

        TelemetryPacket ack{};
        memcpy(&ack, data, sizeof(ack));

        updateRobotTelemetry(ack.robot_id, ack.heartbeat, ack.status, ack.mode, 
                        ack.battery_mv, ack.motor_temp, ack.error_flags,
                        ack.imu_calibration,
                        ack.main_roll, ack.main_pitch, 
                        ack.pend_roll, ack.pend_pitch);

        ack.latency_ms = 404; // Placeholder
        forwardTelemetryToPython(ack);
        return;
    }

    // --- NEW: Route the response back to python ---
    if (pkt_type == PACKET_SETTING_RESPONSE && len >= sizeof(SettingResponsePacket)) {
        SettingResponsePacket resp{};
        memcpy(&resp, data, sizeof(resp));
        forwardSettingResponseToPython(resp);
        return;
    }
}
#endif // ROLE_CONTROLLER