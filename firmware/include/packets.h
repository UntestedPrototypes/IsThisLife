#pragma once

enum PacketType : uint8_t {
    PACKET_CONTROL = 0,
    PACKET_ESTOP = 1,
    PACKET_ESTOP_CLEAR = 2,
    PACKET_DISCOVER = 3,
    PACKET_CONFIRM = 4,
    PACKET_REQUEST_CONFIRM = 5,  // Robot asks for confirmation (optional)
    PACKET_START_SEQUENCE = 6    // Python tells robot to start a sequence
};

#define STATUS_OK    0
#define STATUS_ESTOP 1
#define STATUS_WAITING_CONFIRM 2  // Robot waiting for confirmation
#define STATUS_RUNNING_SEQUENCE 3 // Robot is running a sequence

struct __attribute__((packed)) ControlPacket {
    uint8_t type;
    uint8_t priority;
    uint8_t robot_id;
    uint32_t heartbeat;
    int8_t  vx;
    int8_t  vy;
    int8_t  omega;
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

// New: Robot requests confirmation for a calibration step
struct __attribute__((packed)) RequestConfirmPacket {
    uint8_t type;           // PACKET_REQUEST_CONFIRM
    uint8_t robot_id;
    uint32_t heartbeat;
    uint8_t step_id;        // Which calibration step (0-255)
    char message[32];       // Description: "Calibrating gyro", "Testing motors", etc.
};

// New: Controller sends confirmation
struct __attribute__((packed)) ConfirmPacket {
    uint8_t type;           // PACKET_CONFIRM
    uint8_t robot_id;
    uint32_t heartbeat;
    uint8_t step_id;        // Confirms which step
    bool approved;          // true = proceed, false = cancel
};

// New: Python initiates sequence
struct __attribute__((packed)) StartSequencePacket {
    uint8_t type;           // PACKET_START_SEQUENCE
    uint8_t robot_id;
    uint32_t heartbeat;
    uint8_t sequence_id;    // Which sequence to run (see SEQUENCE_IDs below)
};

// ========== Sequence ID Definitions ==========
// Add your custom sequences here!
#define SEQUENCE_CALIBRATION_FULL   0
#define SEQUENCE_CALIBRATION_GYRO   1
#define SEQUENCE_CALIBRATION_MOTORS 2
#define SEQUENCE_DEMO_DANCE         3
#define SEQUENCE_SENSOR_TEST        4
#define SEQUENCE_PATH_FOLLOW        5
// Add more sequences as needed...