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
    PACKET_TELEMETRY = 7,
    PACKET_SET_SETTING = 8,
    PACKET_GET_SETTING = 9,       // <--- NEW: Request a setting
    PACKET_SETTING_RESPONSE = 10  // <--- NEW: Robot replies with setting
};

// --- Bitmasks for Status Byte ---
#define STATUS_FLAG_ESTOP 0x80
#define STATUS_STATE_MASK 0x7F

#define STATUS_NORMAL 0
#define STATUS_WAITING_CONFIRM 2
#define STATUS_RUNNING_SEQUENCE 3
#define STATUS_CALIBRATION_REQUIRED 4

#define SEQUENCE_CALIBRATION_FULL   0
#define SEQUENCE_CALIBRATION_GYRO   1
#define SEQUENCE_CALIBRATION_MOTORS 2
#define SEQUENCE_DEMO_DANCE         3
#define SEQUENCE_SENSOR_TEST        4
#define SEQUENCE_PATH_FOLLOW        5

struct __attribute__((packed)) PacketHeader {
    uint8_t type;           
    uint8_t robot_id;       
    uint32_t heartbeat;     
};

struct __attribute__((packed)) ControlPacket : public PacketHeader {
    uint8_t mode;           
    uint16_t vx;            
    uint16_t vy;            
    uint16_t omega;         
    uint32_t timestamp_ms;  
};

struct __attribute__((packed)) TelemetryPacket : public PacketHeader {
    uint8_t acked_type;
    uint8_t status;
    uint8_t mode;       
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

struct __attribute__((packed)) SetSettingPacket : public PacketHeader {
    char key[16];           
    float value;            
};

struct __attribute__((packed)) GetSettingPacket : public PacketHeader {
    char key[16];           // Key to request
};

struct __attribute__((packed)) SettingResponsePacket : public PacketHeader {
    char key[16];           // The requested key
    float value;            // The current float value on the robot
};