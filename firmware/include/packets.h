#pragma once

#include <stdint.h>

enum PacketType : uint8_t {
    PACKET_CONTROL = 0,
    PACKET_ESTOP = 1,
    PACKET_ESTOP_CLEAR = 2,
    PACKET_DISCOVER = 3,
    PACKET_CONFIRM = 4,
    PACKET_REQUEST_CONFIRM = 5,
    PACKET_START_SEQUENCE = 6,
    PACKET_TELEMETRY = 7
};

// --- Bitmasks for Status Byte ---
#define STATUS_FLAG_ESTOP 0x80 // Bit 7: 1 = E-STOP Active
#define STATUS_STATE_MASK 0x7F // Bits 0-6: Operational State

// --- Operational States ---
#define STATUS_NORMAL 0
#define STATUS_WAITING_CONFIRM 2
#define STATUS_RUNNING_SEQUENCE 3
#define STATUS_CALIBRATION_REQUIRED 4

// ========== Sequence ID Definitions ==========
#define SEQUENCE_CALIBRATION_FULL   0
#define SEQUENCE_CALIBRATION_GYRO   1
#define SEQUENCE_CALIBRATION_MOTORS 2
#define SEQUENCE_DEMO_DANCE         3
#define SEQUENCE_SENSOR_TEST        4
#define SEQUENCE_PATH_FOLLOW        5

// =================================================================
// BASE PACKET HEADER
// =================================================================
// Every packet will inherently start with these 6 bytes.
struct __attribute__((packed)) PacketHeader {
    uint8_t type;           // PacketType enum
    uint8_t robot_id;       // 0 = Broadcast, 1-N = Specific Robot
    uint32_t heartbeat;     // Message counter / timestamp
};

// =================================================================
// DERIVED PACKETS
// =================================================================

// Inherits type, robot_id, and heartbeat from PacketHeader
struct __attribute__((packed)) ControlPacket : public PacketHeader {
    uint8_t priority;
    uint16_t vx;            // 1000-2000 us (Throttle)
    uint16_t vy;            // 1000-2000 us (Strafe/Steer)
    uint16_t omega;         // 1000-2000 us (Rotation)
    uint32_t timestamp_ms;  
};

struct __attribute__((packed)) TelemetryPacket : public PacketHeader {
    uint8_t acked_type;
    uint8_t status;       
    uint16_t battery_mv;  
    int16_t motor_temp;   
    uint8_t error_flags;  
    uint16_t latency_ms;
    uint16_t imu_calibration;
    
    float main_roll;      
    float main_pitch;     
    float pend_roll;      
    float pend_pitch;
};

struct __attribute__((packed)) RequestConfirmPacket : public PacketHeader {
    uint8_t step_id;        
    char message[32];       
};

struct __attribute__((packed)) ConfirmPacket : public PacketHeader {
    uint8_t step_id;        
    bool approved;          
};

struct __attribute__((packed)) StartSequencePacket : public PacketHeader {
    uint8_t sequence_id;    
};