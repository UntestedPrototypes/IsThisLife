"""
Configuration constants for the robot controller system
"""

# Serial Configuration
BAUD_RATE = 115200

# Serial Protocol
START_BYTE = 0xAA
END_BYTE = 0x55

# Packet Types
PACKET_CONTROL = 0
PACKET_ESTOP = 1
PACKET_ESTOP_CLEAR = 2
PACKET_CONFIRM = 4
PACKET_START_SEQUENCE = 6

# Default Robot ID
DEFAULT_ROBOT_ID = 1

# Sequence ID Definitions (must match robot.cpp)
SEQUENCE_CALIBRATION_FULL = 0
SEQUENCE_CALIBRATION_GYRO = 1
SEQUENCE_CALIBRATION_MOTORS = 2
SEQUENCE_DEMO_DANCE = 3
SEQUENCE_SENSOR_TEST = 4
SEQUENCE_PATH_FOLLOW = 5

# Status codes (must match robot definitions)
STATUS_OK = 0
STATUS_ESTOP = 1
STATUS_WAITING_CONFIRM = 2
STATUS_RUNNING_SEQUENCE = 3

# UI Configuration
WINDOW_WIDTH = 1050
WINDOW_HEIGHT = 550
CONTROL_UPDATE_RATE_MS = 50  # 20 Hz
MAX_ROBOTS = 2

# Joystick learning timeout
LEARNING_TIMEOUT_SECONDS = 5.0
LEARNING_THRESHOLD = 0.3  # Axis movement threshold for detection
