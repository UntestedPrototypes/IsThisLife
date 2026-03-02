# Robot Code Refactoring

This is a highly modular refactoring of the robot firmware, split into focused modules by functionality and organized into logical subdirectories for better maintainability and scalability.

## Directory Structure

### Configuration (`config/`)
- **robot_config.h** - All constants and configuration values (robot ID, channel, MAC address, timing constants, pins).
- **robot_preferences.h / robot_preferences.cpp** - NVS Flash memory saving/loading interface for IMU offsets and networking states.

### Core Modules

#### Communication (`comms/`)
- **heartbeat.h / heartbeat.cpp** - Connection health monitoring and validation logic.
- **telemetry.h / telemetry.cpp** - Telemetry packet generation and status reporting.
- **packet_handler.h / packet_handler.cpp** - ESP-NOW packet reception, queue routing, and fast-path ISR handling.

#### Control & Safety (`control/`)
- **safety.h / safety.cpp** - E-STOP state machine and global motor enable/disable logic.
- **motors.h / motors.cpp** - Motor control interface (main wheel and pendulums), including 50Hz PWM limiting.

#### Hardware Sensors (`sensors/`)
- **sensors.h / sensors.cpp** - Main wrapper that initializes all underlying sensor subsystems.
- **imu_handler.h / imu_handler.cpp** - Dedicated BNO055 reading, calibration logic, and orientation processing.
- **system_monitor.h / system_monitor.cpp** - INA219 battery reading, thermal monitoring, and hardware error state diagnostics.

#### Pure Math (`math/`)
- **quaternion_math.h / quaternion_math.cpp** - 4D quaternion combining and Euler angle conversions (hardware independent).

#### Logic & Advanced Features (`logic/`)
- **confirmation.h / confirmation.cpp** - Two-way user confirmation request and response handling.
- **sequence.h / sequence.cpp** - Automated sequence execution engine containing all predefined calibration and demo routines.

### Main Entry
- **robot.cpp** - Main FreeRTOS setup, task orchestration (`SystemTask` and `ControlTask`), and initialization loop.

## Module Dependencies

The modular design ensures a clean, mostly unidirectional flow of data:

```text
robot.cpp
├── config/robot_config.h
├── config/robot_preferences.h
├── comms/heartbeat.h         → config/robot_config.h
├── control/safety.h          → control/motors.h
├── control/motors.h          → config/robot_config.h
├── sensors/sensors.h         → sensors/system_monitor.h, sensors/imu_handler.h, math/quaternion_math.h
├── comms/telemetry.h         → config/, control/safety.h, logic/, sensors/
├── logic/confirmation.h      → config/, control/safety.h, control/motors.h
├── logic/sequence.h          → comms/telemetry.h, logic/confirmation.h, control/, sensors/
└── comms/packet_handler.h    → (routes to logic, control, comms, etc.)