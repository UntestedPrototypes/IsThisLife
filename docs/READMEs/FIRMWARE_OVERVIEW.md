**THIS DOCUMENT WAS GENERATE BY AI.**

# System Overview: Robot & Controller Firmware

This repository contains the firmware for a highly modular, ESP32-based balanced robot and its dedicated controller. The system uses **ESP-NOW** for low-latency wireless communication and **FreeRTOS** for strict real-time task orchestration.

The architecture is divided into two main roles:

1. **Robot (`ROLE_ROBOT`)**: The firmware running on the physical robot, handling sensor fusion, PID stabilization, motor control, and safety constraints.
2. **Controller (`ROLE_CONTROLLER`)**: A bridge device that interfaces between a Python-based PC application (via Serial) and the swarm of robots (via ESP-NOW).

---

## 1. Robot Firmware Architecture

The robot firmware is heavily modularized to ensure a clean separation of co****ncerns and maintain a highly predictable real-time control loop. It is orchestrated primarily by `robot.cpp`, which splits workloads across the ESP32's two cores (System tasks on Core 0, 100Hz Control/PID loops on Core 1).

### Core Modules:

* **`config/` (Configuration & Memory)**

  * `robot_config.h`: Hardware definitions, pins, MAC addresses, and timing constants.
  * `robot_preferences.h/.cpp`: Non-Volatile Storage (NVS) handling. Saves and loads PID values, IMU calibration offsets, encoder limits, and debug states across reboots.
* **`control/` (Motion & Safety)**

  * `motors.h/.cpp`: Interfaces with hardware—a main PWM-driven motor and Waveshare ST3215 serial servos (acting as pendulums).
  * `pid_controller.h/.cpp`: The core balancing logic. Implements a cascading PID controller for roll (Outer loop for target tilt, Inner loop for execution) and a standard PID loop for pitch.
  * `safety.h/.cpp`: The E-STOP state machine. Enforces hard safety stops in the event of hardware errors or connection drops.
* **`sensors/` (Hardware Data)**

  * `sensors.h/.cpp`: Global initialization wrapper.
  * `imu_handler.h/.cpp`: Interfaces with dual BNO055 IMUs. Handles offset application, calibration, and orientation extraction using quaternions.
  * `system_monitor.h/.cpp`: Monitors battery voltage via an INA219 and reads hardware error flags (undervoltage, overheating).
* **`comms/` (ESP-NOW Networking)**

  * `packet_handler.h/.cpp`: Fast-path ISR handling for incoming ESP-NOW packets and FreeRTOS queue routing.
  * `telemetry.h/.cpp`: Aggregates robot state (IMU, battery, mode, errors) and transmits it back to the controller.
  * `heartbeat.h/.cpp`: Tracks incoming connection health. Triggers E-STOPs if communication is lost.
* **`logic/` (High-Level State)**

  * `sequence.h/.cpp`: Automated routines, such as the step-by-step IMU calibration sequence.
  * `confirmation.h/.cpp`: Allows the robot to pause and request operator confirmation (via the controller) before proceeding with dangerous/calibrating steps.
* **`io/` & `math/`**

  * `io/serial_cli.h/.cpp`: A command-line interface over USB for direct tuning, debugging, and diagnostics.
  * `io/led_manager.h/.cpp`: Drives LED patterns (fast blink, slow pulse, mix) to reflect system states (E-STOP, Calibrating, Normal).
  * `math/quaternion_math.h/.cpp`: Hardware-agnostic math for quaternion combination and Euler angle extraction.

---

## 2. Controller Firmware Architecture

The controller acts as the central hub. It accepts high-level commands over Serial (usually from a Python GUI) and routes them to specific robots via ESP-NOW. It also aggregates telemetry from the robots and forwards it back over Serial.

### Core Modules:

* **`controller_config.h`**

  * Defines the maximum number of robots, ESP-NOW channel, robot MAC addresses, and serial protocol byte markers.
* **Serial ↔ ESP-NOW Bridge (`python_comm` & `serial_parser`)**

  * `serial_parser.cpp`: Listens to the USB serial port, parses byte streams into commands (Control, E-STOP, Set/Get Settings), and executes the respective ESP-NOW broadcasts.
  * `python_comm.cpp`: Wraps incoming ESP-NOW telemetry packets into Serial byte streams for the Python GUI. It also acts as a watchdog, sending an E-STOP to all robots if the Python connection times out.
* **ESP-NOW Network Management (`espnow_handler` & `peer_management`)**

  * `espnow_handler.cpp`: The receive callback. Catches telemetry, setting responses, and confirmation requests from the robots, passing them down the line.
  * `peer_management.cpp`: Handles the dynamic registration of ESP-NOW peers (the robots) using their MAC addresses.
* **Robot State Management (`robot_telemetry` & `robot_commands`)**

  * `robot_telemetry.cpp`: Maintains an internal cache of the last known state (battery, mode, IMU, errors) of all connected robots.
  * `robot_commands.cpp`: A suite of helper functions to format and dispatch specific packets (Arm, E-STOP, Discover, Start Sequence) to targeted robots.

---

## System Data Flow

1. **User Input:** A Python script sends a joystick control packet over Serial to the Controller ESP32.
2. **Translation:** The Controller's `serial_parser` decodes this, and `robot_commands` broadcasts it over ESP-NOW.
3. **Reception:** The Robot's `packet_handler` triggers an interrupt, pushing the packet to an RTOS queue.
4. **Execution:** Core 0 processes the packet. If it's a movement command, it sets the target velocities. Core 1's strictly timed `ControlTask` picks up the new target, reads the `imu_handler`, calculates the `pid_controller` error, and drives the `motors`.
5. **Feedback:** The robot periodically gathers state data and uses `telemetry` to shoot a packet back to the Controller, which `python_comm` forwards to the user's screen.
