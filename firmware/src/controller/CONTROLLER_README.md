# Controller Code Refactoring

This is a modular refactoring of the original `controller.cpp` file, split into focused modules by functionality.

## File Structure

### Configuration
- **controller_config.h** - All constants and configuration values (robot count, channel, MAC addresses, timing constants)

### Core Modules

#### Robot Communication
- **robot_commands.h / robot_commands.cpp** - Commands sent to robots (heartbeat, arm, E-STOP, control, confirmation, sequences)
- **espnow_handler.h / espnow_handler.cpp** - ESP-NOW receive callback and packet processing
- **peer_management.h / peer_management.cpp** - ESP-NOW peer connection management

#### Telemetry & Status
- **robot_telemetry.h / robot_telemetry.cpp** - Robot status tracking and telemetry data structures

#### Python Interface
- **python_comm.h / python_comm.cpp** - Python connection monitoring and data forwarding
- **serial_parser.h / serial_parser.cpp** - Serial command parsing from Python

#### Timing & Coordination
- **heartbeat_manager.h / heartbeat_manager.cpp** - Periodic heartbeat transmission management

### Main File
- **controller.cpp** - Main setup and loop functions that orchestrate all modules

## Module Dependencies

```
controller.cpp
├── controller_config.h
├── robot_telemetry.h → controller_config.h
├── python_comm.h → robot_commands.h
├── robot_commands.h → controller_config.h
├── serial_parser.h → controller_config.h, robot_commands.h, python_comm.h
├── espnow_handler.h → robot_telemetry.h, python_comm.h
├── peer_management.h → controller_config.h
└── heartbeat_manager.h → controller_config.h, robot_commands.h
```

## Benefits of This Structure

1. **Modularity** - Each file has a single, clear responsibility
2. **Maintainability** - Changes to one feature don't require touching unrelated code
3. **Testability** - Individual modules can be tested in isolation
4. **Readability** - Much easier to find and understand specific functionality
5. **Reusability** - Modules can be reused in other projects
6. **Scalability** - Easy to add new robots, commands, or features

## How to Use

1. Include all header (.h) and implementation (.cpp) files in your project
2. Make sure `packets.h` is available (referenced but not included in this refactoring)
3. **Define `ROLE_CONTROLLER` before including any files** (e.g., in your build system or at the top of your main file)
4. Compile all .cpp files together
5. The main entry points remain `roleSetup()` and `roleLoop()` in controller.cpp

## Compilation Notes

All controller modules are wrapped with `#ifdef ROLE_CONTROLLER` guards to prevent compilation conflicts when building the robot code. You must define `ROLE_CONTROLLER` before compilation, typically:

- **In PlatformIO:** Add to `platformio.ini`:
  ```ini
  build_flags = -DROLE_CONTROLLER
  ```

- **In Arduino IDE:** Add at the top of your main sketch before any includes:
  ```cpp
  #define ROLE_CONTROLLER
  ```

- **Command line compilation:** Add `-DROLE_CONTROLLER` to compiler flags

## Key Functions by Module

### Robot Commands
- `sendHeartbeat()` - Send heartbeat to all robots
- `sendArmRobot()` - Clear E-STOP for specific robot
- `sendEstopRobot()` - Trigger E-STOP for specific robot
- `sendConfirmation()` - Send confirmation response to robot
- `sendStartSequence()` - Start a sequence on a robot
- `sendControlCommand()` - Send motor control commands
- `sendDiscover()` - Broadcast discovery packet

### Python Communication
- `forwardTelemetryToPython()` - Forward robot telemetry to Python
- `forwardConfirmRequestToPython()` - Forward confirmation requests to Python
- `checkPythonTimeout()` - Monitor Python connection and trigger E-STOP if lost

### Serial Parser
- `readSerialCommands()` - Parse incoming serial commands from Python

### Peer Management
- `connectToPeer()` - Add a specific robot as ESP-NOW peer
- `connectToAllPeers()` - Add all configured robots as peers

### Heartbeat Manager
- `sendPeriodicHeartbeat()` - Manage periodic heartbeat transmission

## Adding More Robots

To add more robots:
1. Change `NUM_ROBOTS` in `controller_config.h`
2. Add MAC addresses to `robotMacs` array in `controller.cpp`

## Serial Protocol

The controller receives commands from Python via serial in the format:
- Byte 0: Command type (PACKET_CONTROL, PACKET_ESTOP, etc.)
- Byte 1: Robot ID (0 for broadcast)
- Bytes 2+: Command-specific parameters

See `serial_parser.cpp` for complete command parsing logic.

## Python Connection Monitoring

The controller monitors Python connectivity:
- Timeout: 500ms (configurable in `controller_config.h`)
- On timeout: Automatically sends E-STOP to all robots
- Connection status tracked via `python_connected` flag
