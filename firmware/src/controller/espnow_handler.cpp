#ifdef ROLE_CONTROLLER
#include "espnow_handler.h"
#include "packets.h"
#include "robot_telemetry.h"
#include "python_comm.h"
#include <Arduino.h>
#include <string.h>

void onRobotReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len < 1) return;
    
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

    // Update local telemetry tracking
    updateRobotTelemetry(ack.robot_id, ack.heartbeat, ack.status, 
                        ack.battery_mv, ack.motor_temp, ack.error_flags);

    // Fill placeholder latency (can be calculated if needed)
    ack.latency_ms = 404;

    // Forward to Python
    forwardTelemetryToPython(ack);
}
#endif // ROLE_CONTROLLER
