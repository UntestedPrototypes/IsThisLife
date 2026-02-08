# Robot Sequence System - Developer Guide

## Overview
A flexible, general-purpose sequence execution system that allows Python to trigger autonomous sequences on your robot. Perfect for calibration, demos, testing, autonomous behaviors, and more!

## Quick Start

### Python Side (3 lines)
```python
# 1. Define your sequence ID at top of main.py
SEQUENCE_MY_ROUTINE = 6

# 2. Add a button somewhere in the GUI
tk.Button(frame, text="My Routine", 
         command=lambda: send_start_sequence(robot_id, SEQUENCE_MY_ROUTINE))

# 3. Done! (now implement on robot)
```

### Robot Side (in robot.cpp)
```cpp
// 1. Add to packets.h
#define SEQUENCE_MY_ROUTINE 6

// 2. Add case to runSequenceStep() switch statement
case 6: {  // SEQUENCE_MY_ROUTINE
    switch(currentSequenceStep) {
        case 0:
            Serial.println("SEQ: Starting my routine");
            // Your code here
            sequenceStepStartTime = millis();
            currentSequenceStep++;
            break;
        
        case 1:
            if (elapsed > 2000) {  // Wait 2 seconds
                Serial.println("SEQ: Step 2");
                // Your code here
                sequenceActive = false;  // Mark complete
                currentSequenceStep = 0;
            }
            break;
    }
    break;
}
```

## Built-in Sequences

### Calibration Sequences
| ID | Name | Description | Duration |
|----|------|-------------|----------|
| 0 | Full Calibration | Initialize → Gyro → Motors | ~7 sec |
| 1 | Gyro Only | Calibrate gyroscope | ~3 sec |
| 2 | Motors Only | Test motor function | ~2 sec |

### Demo Sequences  
| ID | Name | Description | Duration |
|----|------|-------------|----------|
| 3 | Dance Demo | Forward → Strafe → Spin | ~3 sec |
| 4 | Sensor Test | Read and report all sensors | ~2 sec |
| 5 | Path Follow | Navigate waypoint 1 → 2 | ~6 sec |

## Architecture

### Flow Diagram
```
┌─────────┐                ┌────────────┐                ┌───────┐
│ Python  │──START_SEQ──▶  │ Controller │──forwards──▶   │ Robot │
│   GUI   │                │ (ESP-NOW)  │                │       │
└─────────┘                └────────────┘                └───┬───┘
     ▲                                                        │
     │                                                        │
     └──────────STATUS=3 "RUNNING SEQ"─────────────────────┘
```

### State Machine (Robot)
```
IDLE ──START_SEQUENCE──▶ RUNNING ──complete──▶ IDLE
                            │
                         E-STOP
                            │
                            ▼
                          IDLE
```

## Adding New Sequences

### Step 1: Define Sequence ID (packets.h)
```cpp
// In packets.h, add to sequence definitions:
#define SEQUENCE_MY_NEW_SEQUENCE 10
```

### Step 2: Implement Sequence Logic (robot.cpp)
```cpp
// In runSequenceStep(), add a new case:
case 10: {  // SEQUENCE_MY_NEW_SEQUENCE
    switch(currentSequenceStep) {
        case 0:
            Serial.println("SEQ: Step 0 - Initialize");
            // Your initialization code
            sequenceStepStartTime = millis();
            currentSequenceStep++;
            break;
        
        case 1:
            if (elapsed > 1000) {  // Wait condition
                Serial.println("SEQ: Step 1 - Action");
                // Your action code
                sequenceStepStartTime = millis();
                currentSequenceStep++;
            }
            break;
        
        case 2:
            if (elapsed > 2000) {
                Serial.println("SEQ: Complete");
                sequenceActive = false;
                currentSequenceStep = 0;
                sendAckTelemetry(PACKET_CONTROL, 0, 0);
            }
            break;
    }
    break;
}
```

### Step 3: Add to Python GUI (main.py)
```python
# Define the constant at top
SEQUENCE_MY_NEW_SEQUENCE = 10

# Add button in init_live_tab()
tk.Button(frame, text="My Sequence", 
         command=lambda rid=robot_id: send_start_sequence(rid, SEQUENCE_MY_NEW_SEQUENCE))
```

## Sequence Patterns

### Simple Timer-Based
```cpp
case X: {
    switch(currentSequenceStep) {
        case 0:
            // Start action
            sequenceStepStartTime = millis();
            currentSequenceStep++;
            break;
        
        case 1:
            if (elapsed > 2000) {  // Wait 2 seconds
                sequenceActive = false;  // Done
                currentSequenceStep = 0;
            }
            break;
    }
    break;
}
```

### Condition-Based (wait for sensor)
```cpp
case X: {
    switch(currentSequenceStep) {
        case 0:
            Serial.println("Waiting for sensor...");
            currentSequenceStep++;
            break;
        
        case 1:
            if (readSensor() > threshold) {  // Wait for condition
                Serial.println("Sensor ready!");
                sequenceActive = false;
                currentSequenceStep = 0;
            }
            break;
    }
    break;
}
```

### Multi-Step with Motors
```cpp
case X: {
    switch(currentSequenceStep) {
        case 0:
            Serial.println("Moving forward");
            setMotors(50, 0, 0);
            sequenceStepStartTime = millis();
            currentSequenceStep++;
            break;
        
        case 1:
            if (elapsed > 1000) {
                Serial.println("Turning");
                setMotors(0, 0, 50);
                sequenceStepStartTime = millis();
                currentSequenceStep++;
            }
            break;
        
        case 2:
            if (elapsed > 1000) {
                Serial.println("Stopping");
                stopMotors();
                sequenceActive = false;
                currentSequenceStep = 0;
            }
            break;
    }
    break;
}
```

### With Optional User Confirmation
```cpp
case X: {
    switch(currentSequenceStep) {
        case 0:
            Serial.println("Ready for dangerous step");
            requestConfirmation(1, "Proceed with test?");
            currentSequenceStep++;
            break;
        
        case 1:
            // Wait for confirmation (handled by confirmation system)
            // Robot will pause here until user approves/denies
            if (!waitingForConfirmation) {
                // User responded, continue
                currentSequenceStep++;
            }
            break;
        
        case 2:
            Serial.println("Executing dangerous step");
            // Your code here
            sequenceActive = false;
            currentSequenceStep = 0;
            break;
    }
    break;
}
```

## Features

### Automatic Safety
✅ **Motors disabled** during sequences (except where you explicitly enable them)  
✅ **E-STOP cancels** any running sequence immediately  
✅ **Heartbeat continues** - no connection loss during long sequences  
✅ **Can't start if E-STOPPED** - robot must be armed first  

### State Management
- `sequenceActive` - Is a sequence currently running?
- `sequenceId` - Which sequence is running?
- `currentSequenceStep` - Which step within the sequence?
- `sequenceStepStartTime` - When did current step start?

### Python GUI Features
- **Status display** shows "RUNNING SEQ" (blue) during execution
- **Organized buttons** grouped by category (Calibration, Demo, etc.)
- **Easy to add** new sequence buttons

## Example Use Cases

### Calibration
```cpp
// Sequence 0: Full calibration
Step 0: Initialize sensors (2s)
Step 1: Calibrate gyro - collect samples, compute bias (3s)
Step 2: Test motors - spin each wheel (2s)
Step 3: Complete
```

### Autonomous Demo
```cpp
// Sequence 3: Dance routine
Step 0: Move forward 1 second
Step 1: Strafe right 1 second  
Step 2: Spin in place 1 second
Step 3: Stop and complete
```

### Diagnostic Test
```cpp
// Sequence 4: Sensor test
Step 0: Read battery voltage
Step 1: Read motor temperatures
Step 2: Report all values to serial
Step 3: Complete
```

### Path Following
```cpp
// Sequence 5: Navigate waypoints
Step 0: Move to waypoint 1 (3s)
Step 1: Move to waypoint 2 (3s)
Step 2: Return to origin (3s)
Step 3: Complete
```

## Advanced Features

### Nested Sequences
Call one sequence from another:
```cpp
case 20: {  // MASTER_SEQUENCE
    switch(currentSequenceStep) {
        case 0:
            // Start calibration sub-sequence
            sequenceId = SEQUENCE_CALIBRATION_FULL;
            currentSequenceStep = 0;
            break;
        // Continue after calibration completes...
    }
    break;
}
```

### Parameterized Sequences
Use global variables to pass parameters:
```cpp
// Global
int target_position_x = 0;
int target_position_y = 0;

// In Python, set before starting:
// (You'd need to add a parameter packet type for this)

// In sequence:
case X: {
    switch(currentSequenceStep) {
        case 0:
            Serial.printf("Moving to (%d, %d)\n", target_position_x, target_position_y);
            // Navigate to target_position
            break;
    }
    break;
}
```

### Looping Sequences
Repeat steps indefinitely:
```cpp
case X: {
    switch(currentSequenceStep) {
        case 0:
            Serial.println("Step 1");
            sequenceStepStartTime = millis();
            currentSequenceStep++;
            break;
        
        case 1:
            if (elapsed > 1000) {
                Serial.println("Step 2");
                sequenceStepStartTime = millis();
                currentSequenceStep = 0;  // Loop back!
            }
            break;
    }
    // To stop: Check a condition and set sequenceActive = false
    break;
}
```

## Troubleshooting

**Sequence won't start:**
- ❌ Robot in E-STOP → ARM the robot first
- ❌ Serial disconnected → Check connection
- ❌ Another sequence running → Wait for completion or E-STOP

**Sequence stops mid-way:**
- ❌ E-STOP triggered → Intended behavior
- ❌ Heartbeat lost → Check connection
- ❌ Robot powered off → Check battery

**Motors don't work in sequence:**
- Motors are automatically disabled during sequences
- You must explicitly call `setMotors()` in your sequence steps
- Make sure you're not in E-STOP

**Sequence runs too fast/slow:**
- Adjust `elapsed > TIME_MS` conditions
- Make sure you're calling `sequenceStepStartTime = millis()` when entering each step

## Serial Protocol

### Start Sequence Command
```
Byte 0: PACKET_START_SEQUENCE (6)
Byte 1: robot_id
Byte 2: sequence_id (0-255)
```

### Status Updates
```
ID=1 HB=123 STATUS=3 BATT=7400 TEMP=35 ERR=0x00 RTT=10
        STATUS=3 means "RUNNING_SEQUENCE"
```

## Best Practices

1. **Print debug messages** - Use `Serial.println("SEQ: step description")` liberally
2. **Send telemetry** - Call `sendAckTelemetry()` at start and end of sequence
3. **Handle timeouts** - Don't wait forever, add max elapsed checks
4. **Stop motors** - Always `stopMotors()` when sequence completes
5. **Reset state** - Set `sequenceActive = false` and `currentSequenceStep = 0` when done
6. **Test incrementally** - Start with simple 2-step sequences
7. **Use comments** - Explain what each step does

## File Organization

```
packets.h          - Packet structures and SEQUENCE_ID definitions
robot.cpp          - Sequence execution engine (runSequenceStep)
controller.cpp     - Forwards sequence commands
main.py            - Python GUI with sequence buttons
```

## FAQ

**Q: Can I run multiple sequences simultaneously?**  
A: No, only one sequence at a time. Use multi-step sequences or nested sequences.

**Q: How do I stop a sequence early?**  
A: Press E-STOP, or add a "cancel sequence" button that sends E-STOP.

**Q: Can sequences control motors?**  
A: Yes! Call `setMotors(vx, vy, omega)` in your sequence steps. Motors are disabled by default.

**Q: How long can a sequence run?**  
A: As long as you want! Heartbeat continues running in the background.

**Q: Can I add parameters to sequences?**  
A: Not directly, but you can use global variables or extend the packet structure.

**Q: What's the difference between this and normal control?**  
A: Normal control is reactive (joystick input). Sequences are autonomous (robot executes steps independently).

## Next Steps

1. Try the built-in demo sequences
2. Modify an existing sequence
3. Create your first custom sequence
4. Add it to the Python GUI
5. Share your cool sequences! 🤖
