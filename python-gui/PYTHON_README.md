# Robot Controller Dashboard - Python Refactoring

This is a modular refactoring of the original `main.py` file, split into focused modules by functionality.

## File Structure

### Configuration
- **config.py** - All constants, packet types, sequence IDs, and UI configuration

### Communication
- **serial_comm.py** - Low-level serial port communication
- **packet_sender.py** - High-level functions to send command packets to robots
- **serial_reader.py** - Background thread for reading and parsing serial data
- **telemetry_parser.py** - Parse telemetry and confirmation request messages

### Input & Control
- **joystick_control.py** - Game controller/joystick input handling with learning mode
- **robot_state.py** - State management for multiple robots (E-STOP, ARM, etc.)

### User Interface
- **ui_config_tab.py** - Configuration tab with connection settings and terminals
- **ui_live_tab.py** - Live view tab showing robot telemetry and sequence controls
- **ui_game_tab.py** - Game controller tab with joystick mapping and visualization
- **dashboard.py** - Main dashboard window orchestrating all components

### Main Entry Point
- **main.py** - Simple entry point (~20 lines vs original 559)

## Module Dependencies

```
main.py
└── dashboard.py
    ├── config.py
    ├── robot_state.py
    ├── serial_reader.py
    │   ├── serial_comm.py
    │   └── telemetry_parser.py
    ├── packet_sender.py
    │   └── serial_comm.py
    ├── ui_config_tab.py
    │   └── serial_comm.py
    ├── ui_live_tab.py
    │   ├── config.py
    │   └── packet_sender.py
    └── ui_game_tab.py
        └── joystick_control.py
```

## Benefits of This Structure

1. **Modularity** - Each file has a single, clear responsibility
2. **Maintainability** - Changes to one feature don't require touching unrelated code
3. **Testability** - Individual modules can be tested in isolation
4. **Readability** - Much easier to find and understand specific functionality
5. **Reusability** - Modules can be reused in other projects
6. **Scalability** - Easy to add new robots, sequences, or UI features

## Requirements

```bash
pip install pygame pyserial
```

## How to Use

### Running the Application

```bash
python main.py
```

### Project Structure

All Python files should be in the same directory:

```
project/
├── main.py
├── config.py
├── serial_comm.py
├── packet_sender.py
├── serial_reader.py
├── telemetry_parser.py
├── joystick_control.py
├── robot_state.py
├── ui_config_tab.py
├── ui_live_tab.py
├── ui_game_tab.py
└── dashboard.py
```

## Key Features by Module

### Serial Communication (`serial_comm.py`)
- `connect(port)` - Connect to serial port
- `disconnect()` - Close connection
- `is_connected()` - Check connection status
- `read_available()` - Read available data
- `write(data)` - Send data

### Packet Sender (`packet_sender.py`)
- `send_control(robot_id, vx, vy, omega)` - Send motor commands
- `send_estop(robot_id)` - Emergency stop
- `send_arm(robot_id)` - Clear E-STOP
- `send_confirmation(robot_id, step_id, approved)` - Confirm sequence step
- `send_start_sequence(robot_id, sequence_id)` - Start automated sequence

### Telemetry Parser (`telemetry_parser.py`)
- `parse_telemetry_line(line)` - Parse telemetry into TelemetryData object
- `parse_confirmation_request(line)` - Parse confirmation requests
- `is_debug_line(line)` - Identify debug messages

### Joystick Control (`joystick_control.py`)
- `JoystickMapper` class - Handles input mapping with learning mode
- `initialize_joystick()` - Initialize game controller
- `get_input_value(mapping)` - Read current input value

### Robot State (`robot_state.py`)
- `RobotStateManager` class - Manages state for multiple robots
- Handles E-STOP, ARM states, and edge detection for buttons
- Determines when control commands should be sent

## Configuration

### Adding New Sequences

Edit `config.py` to add new sequence IDs:

```python
SEQUENCE_MY_NEW_SEQUENCE = 6
```

Then add buttons in `ui_live_tab.py` to trigger them.

### Changing Robot Count

Edit `config.py`:

```python
MAX_ROBOTS = 3  # Support 3 robots instead of 2
```

### Adjusting Control Rate

Edit `config.py`:

```python
CONTROL_UPDATE_RATE_MS = 100  # 10 Hz instead of 20 Hz
```

## Architecture Notes

### Threading Model

- **Main Thread**: UI updates and user interaction
- **Background Thread**: Serial reading (in `serial_reader.py`)
- **Thread-Safe Communication**: Callbacks scheduled on main thread via `root.after()`

### State Management

- Robot states (E-STOP, ARM) managed centrally in `RobotStateManager`
- Telemetry data stored in `TelemetryData` objects
- UI widgets updated reactively based on state changes

### Serial Protocol

The application communicates with the ESP32 controller using a simple binary protocol:
- **Control**: `[PACKET_CONTROL, robot_id, vx, vy, omega]`
- **E-STOP**: `[PACKET_ESTOP, robot_id]`
- **ARM**: `[PACKET_ESTOP_CLEAR, robot_id]`
- **Confirmation**: `[PACKET_CONFIRM, robot_id, step_id, approved]`
- **Start Sequence**: `[PACKET_START_SEQUENCE, robot_id, sequence_id]`

### Telemetry Format

Received telemetry is text-based:
```
ID=1 HB=123 STATUS=0 BATT=7400 TEMP=35 ERR=0x00 RTT=10
```

## Troubleshooting

**Problem**: Joystick not detected  
**Solution**: Ensure pygame can detect your controller. Test with `pygame.joystick.get_count()`

**Problem**: Serial connection fails  
**Solution**: Check COM port permissions and ensure no other program is using the port

**Problem**: No telemetry received  
**Solution**: Verify controller is sending data and baud rate matches (115200)

**Problem**: UI freezes  
**Solution**: Check for exceptions in serial reader thread, ensure thread-safe UI updates

## Extending the Application

### Adding a New UI Tab

1. Create new module: `ui_mytab.py`
2. Implement tab class with `get_frame()` method
3. Add to `dashboard.py` in `_create_tabs()`

### Adding New Packet Types

1. Add constant to `config.py`
2. Add send function to `packet_sender.py`
3. Add parser to `telemetry_parser.py` if needed
4. Update UI to trigger new packet

### Custom Joystick Mappings

Modify `joystick_control.py`:

```python
default_mappings = {
    "vx": ("axis", 3),  # Different axis
    "vy": ("axis", 4),
    # ... etc
}
```
