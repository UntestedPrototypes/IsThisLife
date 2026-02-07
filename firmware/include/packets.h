#pragma once

enum PacketType : uint8_t {
    PACKET_CONTROL = 0,
    PACKET_ESTOP = 1,
    PACKET_ESTOP_CLEAR = 2,
    PACKET_DISCOVER = 3
};

#define STATUS_OK    0
#define STATUS_ESTOP 1

struct __attribute__((packed)) ControlPacket {
    uint8_t type;
    uint8_t priority;
    uint8_t robot_id;
    int8_t  vx;
    int8_t  vy;
    int8_t  omega;
    uint32_t heartbeat;
    uint16_t timestamp_ms;  // 16-bit timestamp for latency
};

struct __attribute__((packed)) AckTelemetryPacket {
    uint8_t robot_id;
    uint8_t acked_type;
    uint32_t heartbeat;
    uint8_t status;
    uint16_t battery_mv;
    int16_t motor_temp;
    uint8_t error_flags;
    uint16_t latency_ms;    // round-trip latency
};
