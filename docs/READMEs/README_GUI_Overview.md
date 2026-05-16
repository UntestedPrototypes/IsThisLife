**THIS DOCUMENT WAS GENERATE BY AI.**

# System Overview: Python GUI Dashboard

This repository contains the Python-based Ground Control Station (GCS) for the ESP32 balanced robot swarm. Built using **Tkinter** for the UI, **Pygame** for gamepad handling, and **PySerial** for communication, this application serves as the central hub for monitoring, tuning, and driving multiple robots simultaneously.

The application communicates exclusively with the **Controller ESP32** via a USB serial connection, which in turn acts as a bridge to the robots over wireless ESP-NOW.

---

## 1. GUI Architecture & Modules

The application is heavily componentized, separating the underlying communication and state management from the visual UI tabs.

### Core & State Management

* **`main.py`**: The application entry point. Initializes the Tkinter root window and the main `Dashboard` class.
* **`dashboard.py`**: The central orchestrator. It holds the main update loops (serial reading, game controller polling, UI refreshes), routes incoming packets to the correct UI tabs, and manages persistent settings via `config.json`.
* **`robot_state.py`**: Contains `RobotState` and `RobotStateManager`. Acts as the single source of truth for all connected robots, tracking their E-STOP status, arm state, control modes, cruise control speed, and fetched configuration variables.
* **`config.py`**: Global constants, UI dimensions, packet byte-markers, and the dictionary defining which PID/tuning variables are available in the Tuning tab.

### Communications Layer

* **`serial_comm.py`**: A robust wrapper around `pyserial`. Handles connecting, disconnecting, reading the raw byte buffer, and writing bytes.
* **`telemetry_parser.py`**: The binary decoding engine. Scans the incoming byte stream for specific sync headers (`0xAA 0x55` for Telemetry, etc.) and uses Python's `struct` module to unpack the payload into namedtuples (`TelemetryData`, `ConfirmRequest`, `SettingResponse`).
* **`packet_sender.py`**: The binary encoding engine. Formats high-level commands (e.g., `send_control(vx, vy, omega)`, `send_estop()`) into precise byte arrays and sends them down the serial port.

### Input Handling

* **`joystick_control.py`**: Uses `pygame.joystick` (running in a headless dummy video mode) to capture gamepad inputs. Supports hot-plugging, deadzone math, dynamic input remapping (learning mode), and advanced features like Cruise Control and Mode Switching.

### User Interface (Tabs)

The UI is divided into focused `ttk.Notebook` tabs, each handling a specific domain:

* **`ui_config_tab.py`**: Manages the serial connection (COM port, baud rate), features an auto-reconnect toggle, and displays a raw system log for debugging.
* **`ui_live_tab.py`**: The primary operational view. Dynamically creates status panels for active robots showing battery voltage, ping/latency, hardware errors, E-STOP banners, real-time IMU roll/pitch data, and buttons for triggering sequences (e.g., IMU Calibration).
* **`ui_game_tab.py`**: The controller assignment matrix. Allows operators to map specific physical gamepads to specific Robot IDs. Also supports dynamic input binding (click "Bind" and press a button/axis on the controller) and axis scaling (e.g., capping forward speed at 50%).
* **`ui_tuning_tab.py`**: A dynamic parameters dashboard. Automatically generates UI rows based on `config.py`, allowing operators to fetch, edit, and push settings (like PID gains, filter alphas, or Wobble mitigation thresholds) directly to a robot's NVS flash memory in real-time.
* **`ui_plot_tab.py`**: A live graphing environment using **Matplotlib**. Plots the Main and Pendulum IMU Roll/Pitch angles over a scrolling time window, highly useful for tuning PID loops.

---

## 2. System Data Flow

1. **Input Capture:** The `Dashboard` polls `GameControllerTab` at 20Hz (50ms). This tab reads `joystick_control.py` to get the latest deadzone-filtered axis values for all assigned controllers.
2. **Command Dispatch:** The scaled joystick values are passed to `packet_sender.py`, converted into a binary `PACKET_CONTROL` payload, and pushed out via `serial_comm.py` to the ESP32 Controller.
3. **Telemetry Reception:** In the background, `dashboard.py` rapidly polls the serial buffer (every 10ms). Raw bytes are passed to `telemetry_parser.py`.
4. **State Update:** If the parser yields a `TelemetryData` object, `dashboard.py` updates the internal `robot_state.py` cache.
5. **UI Refresh:** The dashboard immediately dispatches this new telemetry object to the `LiveViewTab` (to update text/colors) and the `PlotTab` (to append to the Matplotlib data queues).
