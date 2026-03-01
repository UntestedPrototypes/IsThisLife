#ifdef ROLE_CONTROLLER
#include "espnow_handler.h"
#include "packets.h"
#include "robot_telemetry.h"
#include "python_comm.h"
#include <Arduino.h>
#include <string.h>

void onRobotReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len < 1) return;
    
    // Sniff Robot ID to detect new connections
    uint8_t detected_id = 0;
    
    if (data[0] == PACKET_REQUEST_CONFIRM && len >= 2) {
        detected_id = data[1];
    } else {
        detected_id = data[0];
    }

    if (detected_id > 0) {
        checkNewRobot(detected_id);
    }
    
    uint8_t pkt_type = data[0];
    
    // Handle confirmation request
    if (pkt_type == PACKET_REQUEST_CONFIRM && len >= sizeof(RequestConfirmPacket)) {
        RequestConfirmPacket req{};
        memcpy(&req, data, sizeof(req));
        forwardConfirmRequestToPython(req);
        return;
    }
    
    // Handle telemetry
    if (len < sizeof(AckTelemetryPacket)) return;

    AckTelemetryPacket ack{};
    memcpy(&ack, data, sizeof(ack));

    // Pass the new IMU float values to the storage array
    updateRobotTelemetry(ack.robot_id, ack.heartbeat, ack.status, 
                        ack.battery_mv, ack.motor_temp, ack.error_flags,
                        ack.main_roll, ack.main_pitch, 
                        ack.pend_roll, ack.pend_pitch);

    ack.latency_ms = 404; // Placeholder

    forwardTelemetryToPython(ack);
}
#endif // ROLE_CONTROLLER