# Robot Code Refactoring

This is a modular refactoring of the original `robot.cpp` file, split into focused modules by functionality.

## File Structure

### Configuration
- **robot_config.h** - All constants and configuration values (robot ID, channel, MAC address, timing constants)

### Core Modules

#### Safety & Control
- **safety.h / safety.cpp** - E-STOP management and motor enable/disable control
- **motors.h / motors.cpp** - Motor control interface (setMotors, stopMotors)

#### Communication
- **heartbeat.h / heartbeat.cpp** - Heartbeat monitoring and validation logic
- **telemetry.h / telemetry.cpp** - Telemetry packet sending and status reporting
- **packet_handler.h / packet_handler.cpp** - ESP-NOW packet reception and routing

#### Sensors
- **sensors.h / sensors.cpp** - Sensor reading functions (battery, temperature, error flags)

#### Advanced Features
- **confirmation.h / confirmation.cpp** - User confirmation request/response handling
- **sequence.h / sequence.cpp** - Automated sequence execution engine with all sequence definitions

### Main File
- **robot.cpp** - Main setup and loop functions that orchestrate all modules

## Module Dependencies

```
robot.cpp
├── robot_config.h
├── heartbeat.h → robot_config.h
├── safety.h → motors.h
├── motors.h
├── sensors.h
├── telemetry.h → robot_config.h, safety.h, confirmation.h, sequence.h, sensors.h
├── confirmation.h → robot_config.h, safety.h, motors.h
├── sequence.h → confirmation.h, telemetry.h, motors.h, sensors.h, safety.h
└── packet_handler.h → (all modules)
```

## Benefits of This Structure

1. **Modularity** - Each file has a single, clear responsibility
2. **Maintainability** - Changes to one feature don't require touching unrelated code
3. **Testability** - Individual modules can be tested in isolation
4. **Readability** - Much easier to find and understand specific functionality
5. **Reusability** - Modules can be reused in other projects
6. **Scalability** - Easy to add new features without bloating existing files

## How to Use

1. Include all header (.h) and implementation (.cpp) files in your project
2. Make sure `packets.h` is available (referenced but not included in this refactoring)
3. Compile all .cpp files together
4. The main entry points remain `roleSetup()` and `roleLoop()` in robot.cpp

## Adding New Sequences

To add a new sequence:
1. Open `sequence.cpp`
2. Add a new case to the switch statement in `runSequenceStep()`
3. Define your sequence steps with timing and actions

## Adding New Features

To add a new feature:
1. Create new .h and .cpp files for the feature
2. Add necessary extern declarations in the header
3. Include the header in `robot.cpp` and any dependent modules
4. Call your feature from `roleLoop()` or packet handlers as needed
