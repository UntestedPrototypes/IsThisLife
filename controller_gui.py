import socket
import pygame
import time
import select
import tkinter as tk
from tkinter import ttk
import threading
from collections import deque
import json
import os

# ================= CONFIG =================
ESP_ID = '1'
LAPTOP_ID = '0'
UDP_PORT = 4210
BROADCAST_IP = "255.255.255.255"
SEND_HZ = 50
DEADZONE = 0.15
CONFIG_FILE = "controller_config.json"
# =========================================

SEND_PERIOD = 1.0 / SEND_HZ
MAX_MESSAGES = 20

# Default input mappings
DEFAULT_MAPPINGS = {
    "x_axis": ("axis", 1),
    "y_axis": ("axis", 0),
    "acknowledge": ("button", 0),
    "toggle_send": ("button", 1),
    "emergency_stop": ("button", 9),
}

class ControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Controller Interface")
        self.root.geometry("800x800")
        self.root.resizable(True, True)
        
        # State
        self.running = True
        self.sending_enabled = True
        self.deadzone = DEADZONE
        self.messages = deque(maxlen=MAX_MESSAGES)
        self.last_send = 0.0
        self.x = 0.0
        self.y = 0.0
        
        # Sensor data
        self.sensor_data = {
            "psi": 0.0,
            "phiL": 0.0,
            "phiR": 0.0,
            "theta": 0.0,
            "estl": 0.0,
            "estr": 0.0,
            "temp": 0.0,
            "vbatt": 0.0,
            "emergencystop": "OFF"
        }
        
        # Input mapping
        self.mappings = self.load_config()
        self.learning_mode = None
        self.learning_timeout = 0
        self.learning_baseline_axes = []
        
        # Socket setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("", UDP_PORT))
        self.sock.setblocking(False)
        
        # Pygame setup
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            self.controller_name = "No Controller"
            self.js = None
        else:
            self.js = pygame.joystick.Joystick(0)
            self.js.init()
            self.controller_name = self.js.get_name()
        
        self.setup_ui()
        
        # Start background thread
        self.thread = threading.Thread(target=self.run_loop, daemon=True)
        self.thread.start()
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
    
    def load_config(self):
        """Load input mappings from config file"""
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, 'r') as f:
                    return json.load(f)
            except:
                pass
        return DEFAULT_MAPPINGS.copy()
    
    def save_config(self):
        """Save input mappings to config file"""
        with open(CONFIG_FILE, 'w') as f:
            json.dump(self.mappings, f, indent=2)
    
    def setup_ui(self):
        """Setup the GUI layout"""
        # Main notebook for tabs
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Control Tab
        control_frame = ttk.Frame(notebook)
        notebook.add(control_frame, text="Controls")
        self.setup_control_tab(control_frame)
        
        # Settings Tab
        settings_frame = ttk.Frame(notebook)
        notebook.add(settings_frame, text="Input Mapping")
        self.setup_settings_tab(settings_frame)
    
    def setup_control_tab(self, parent):
        """Setup main controls tab"""
        main_frame = ttk.Frame(parent, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title
        title = ttk.Label(main_frame, text="Controller Interface", font=("Arial", 16, "bold"))
        title.pack(pady=10)
        
        # Controller status
        status_subframe = ttk.Frame(main_frame)
        status_subframe.pack(fill=tk.X, pady=5)
        ttk.Label(status_subframe, text="Controller:", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        self.controller_label = ttk.Label(status_subframe, text=self.controller_name, foreground="green" if self.js else "red")
        self.controller_label.pack(side=tk.LEFT, padx=10)
        
        # Control frame
        control_frame = ttk.LabelFrame(main_frame, text="Controls", padding="10")
        control_frame.pack(fill=tk.X, pady=10)
        
        # Joystick visualization
        self.canvas = tk.Canvas(control_frame, width=200, height=200, bg="white", relief=tk.SUNKEN, borderwidth=2)
        self.canvas.pack(pady=10)
        self.draw_joystick()
        
        # Values
        values_subframe = ttk.Frame(control_frame)
        values_subframe.pack(fill=tk.X, pady=5)
        ttk.Label(values_subframe, text="X:", font=("Arial", 10)).pack(side=tk.LEFT)
        self.x_label = ttk.Label(values_subframe, text="0.000", font=("Arial", 10, "bold"))
        self.x_label.pack(side=tk.LEFT, padx=10)
        ttk.Label(values_subframe, text="Y:", font=("Arial", 10)).pack(side=tk.LEFT, padx=(20, 0))
        self.y_label = ttk.Label(values_subframe, text="0.000", font=("Arial", 10, "bold"))
        self.y_label.pack(side=tk.LEFT, padx=10)
        
        # Deadzone control
        deadzone_subframe = ttk.Frame(control_frame)
        deadzone_subframe.pack(fill=tk.X, pady=10)
        ttk.Label(deadzone_subframe, text="Deadzone:", font=("Arial", 10)).pack(side=tk.LEFT)
        self.deadzone_scale = ttk.Scale(
            deadzone_subframe, from_=0, to=0.5, orient=tk.HORIZONTAL,
            command=self.update_deadzone
        )
        self.deadzone_scale.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)
        self.deadzone_label = ttk.Label(deadzone_subframe, text=f"{self.deadzone:.2f}", font=("Arial", 9), width=5)
        self.deadzone_label.pack(side=tk.LEFT, padx=5)
        self.deadzone_scale.set(self.deadzone)
        
        # Sensor display frame
        sensor_frame = ttk.LabelFrame(control_frame, text="Sensor Values (Live)", padding="10")
        sensor_frame.pack(fill=tk.X, pady=10)
        
        # Create sensor display grid (3 columns x 2 rows)
        sensors_grid = ttk.Frame(sensor_frame)
        sensors_grid.pack(fill=tk.X)
        sensors_grid.columnconfigure(0, weight=1)
        sensors_grid.columnconfigure(1, weight=1)
        sensors_grid.columnconfigure(2, weight=1)
        
        # Row 1: Angles
        ttk.Label(sensors_grid, text="Psi (°):", font=("Arial", 9)).grid(row=0, column=0, sticky=tk.W, padx=5, pady=3)
        self.psi_label = ttk.Label(sensors_grid, text="0.0", font=("Arial", 9, "bold"), foreground="blue")
        self.psi_label.grid(row=0, column=0, sticky=tk.E, padx=5, pady=3)
        
        ttk.Label(sensors_grid, text="PhiL (°):", font=("Arial", 9)).grid(row=0, column=1, sticky=tk.W, padx=5, pady=3)
        self.phiL_label = ttk.Label(sensors_grid, text="0.0", font=("Arial", 9, "bold"), foreground="blue")
        self.phiL_label.grid(row=0, column=1, sticky=tk.E, padx=5, pady=3)
        
        ttk.Label(sensors_grid, text="PhiR (°):", font=("Arial", 9)).grid(row=0, column=2, sticky=tk.W, padx=5, pady=3)
        self.phiR_label = ttk.Label(sensors_grid, text="0.0", font=("Arial", 9, "bold"), foreground="blue")
        self.phiR_label.grid(row=0, column=2, sticky=tk.E, padx=5, pady=3)
        
        # Row 2: Theta, Temp, Battery
        ttk.Label(sensors_grid, text="Theta (°):", font=("Arial", 9)).grid(row=1, column=0, sticky=tk.W, padx=5, pady=3)
        self.theta_label = ttk.Label(sensors_grid, text="0.0", font=("Arial", 9, "bold"), foreground="blue")
        self.theta_label.grid(row=1, column=0, sticky=tk.E, padx=5, pady=3)
        
        ttk.Label(sensors_grid, text="Temp (°C):", font=("Arial", 9)).grid(row=1, column=1, sticky=tk.W, padx=5, pady=3)
        self.temp_label = ttk.Label(sensors_grid, text="0.0", font=("Arial", 9, "bold"), foreground="green")
        self.temp_label.grid(row=1, column=1, sticky=tk.E, padx=5, pady=3)
        
        ttk.Label(sensors_grid, text="Vbatt (V):", font=("Arial", 9)).grid(row=1, column=2, sticky=tk.W, padx=5, pady=3)
        self.vbatt_label = ttk.Label(sensors_grid, text="0.0", font=("Arial", 9, "bold"), foreground="green")
        self.vbatt_label.grid(row=1, column=2, sticky=tk.E, padx=5, pady=3)
        # Row 3: Estimated angles and Emergency Stop Status
        ttk.Label(sensors_grid, text="EstL (°):", font=("Arial", 9)).grid(row=2, column=0, sticky=tk.W, padx=5, pady=3)
        self.estL_label = ttk.Label(sensors_grid, text="0.0", font=("Arial", 9, "bold"), foreground="purple")
        self.estL_label.grid(row=2, column=0, sticky=tk.E, padx=5, pady=3)

        ttk.Label(sensors_grid, text="EstR (°):", font=("Arial", 9)).grid(row=2, column=1, sticky=tk.W, padx=5, pady=3)
        self.estR_label = ttk.Label(sensors_grid, text="0.0", font=("Arial", 9, "bold"), foreground="purple")
        self.estR_label.grid(row=2, column=1, sticky=tk.E, padx=5, pady=3)

        # Emergency Stop on the right
        ttk.Label(sensors_grid, text="Emergency Stop:", font=("Arial", 9)).grid(row=2, column=2, sticky=tk.W, padx=5, pady=3)
        self.emergencystop_label = ttk.Label(sensors_grid, text="OFF", font=("Arial", 9, "bold"), foreground="green")
        self.emergencystop_label.grid(row=2, column=2, sticky=tk.E, padx=5, pady=3)
        
        # Toggle sending
        self.send_var = tk.BooleanVar(value=True)
        self.send_check = ttk.Checkbutton(
            control_frame, text="Send Commands", variable=self.send_var,
            command=self.toggle_sending
        )
        self.send_check.pack(fill=tk.X, pady=10)
        
        # Acknowledge button
        self.ack_button = ttk.Button(
            control_frame, text="Acknowledge", command=self.send_acknowledge
        )
        self.ack_button.pack(fill=tk.X, pady=5)
        
        # Emergency Stop button
        self.emergency_button = ttk.Button(
            control_frame, text="EMERGENCY STOP", command=self.send_emergency_stop
        )
        self.emergency_button.pack(fill=tk.X, pady=5)
        
        # Reset Emergency Stop button
        self.reset_emergency_button = ttk.Button(
            control_frame, text="Reset Emergency Stop", command=self.reset_emergency_stop
        )
        self.reset_emergency_button.pack(fill=tk.X, pady=5)
        
        # Messages frame
        messages_frame = ttk.LabelFrame(main_frame, text="ESP32 Messages", padding="10")
        messages_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        # Message listbox
        scrollbar = ttk.Scrollbar(messages_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.message_box = tk.Listbox(messages_frame, height=10, yscrollcommand=scrollbar.set)
        self.message_box.pack(fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.message_box.yview)
        
        # Status bar
        status_frame = ttk.Frame(main_frame)
        status_frame.pack(fill=tk.X, pady=5)
        
        self.status_label = ttk.Label(status_frame, text="Status: Running", foreground="green")
        self.status_label.pack(side=tk.LEFT)
        
        self.fps_label = ttk.Label(status_frame, text="FPS: 0")
        self.fps_label.pack(side=tk.RIGHT)
    
    def setup_settings_tab(self, parent):
        """Setup input mapping settings with two columns"""
        main_frame = ttk.Frame(parent, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        title = ttk.Label(main_frame, text="Input Mapping Configuration", font=("Arial", 14, "bold"))
        title.pack(pady=10)
        
        info = ttk.Label(main_frame, text="Click 'Bind' button and then move/press the desired controller input.", 
                        font=("Arial", 10), wraplength=400, justify=tk.LEFT)
        info.pack(pady=10)
        
        # Two-column container
        container = ttk.Frame(main_frame)
        container.pack(fill=tk.BOTH, expand=True, pady=10)
        container.columnconfigure(0, weight=1)
        container.columnconfigure(1, weight=1)
        container.rowconfigure(0, weight=1)
        
        # LEFT COLUMN: Input Bindings
        left_frame = ttk.LabelFrame(container, text="Input Mappings", padding="10")
        left_frame.grid(row=0, column=0, sticky=(tk.N, tk.S, tk.E, tk.W), padx=(0, 5))
        left_frame.columnconfigure(0, weight=1)
        
        self.binding_labels = {}
        
        # X Axis
        self.create_binding_row(left_frame, "x_axis", "X Axis", 0)
        # Y Axis
        self.create_binding_row(left_frame, "y_axis", "Y Axis", 1)
        # Acknowledge
        self.create_binding_row(left_frame, "acknowledge", "Acknowledge Button", 2)
        # Toggle Send
        self.create_binding_row(left_frame, "toggle_send", "Toggle Send Button", 3)
        # Emergency Stop
        self.create_binding_row(left_frame, "emergency_stop", "Emergency Stop Button", 4)
        
        # RIGHT COLUMN: Input Display
        right_frame = ttk.LabelFrame(container, text="Controller Inputs (Live)", padding="10")
        right_frame.grid(row=0, column=1, sticky=(tk.N, tk.S, tk.E, tk.W), padx=(5, 0))
        right_frame.columnconfigure(0, weight=1)
        right_frame.rowconfigure(1, weight=1)
        
        info_right = ttk.Label(right_frame, text="Current input values:", font=("Arial", 9))
        info_right.grid(row=0, column=0, sticky=tk.W, pady=(0, 5))
        
        # Display with scrollbar
        scrollbar = ttk.Scrollbar(right_frame)
        scrollbar.grid(row=1, column=1, sticky=(tk.N, tk.S))
        
        self.input_display = tk.Text(right_frame, height=15, width=35, yscrollcommand=scrollbar.set, 
                                     font=("Courier", 9), relief=tk.SUNKEN, state=tk.DISABLED)
        self.input_display.grid(row=1, column=0, sticky=(tk.N, tk.S, tk.E, tk.W))
        scrollbar.config(command=self.input_display.yview)
        
        # Buttons frame
        buttons_frame = ttk.Frame(main_frame)
        buttons_frame.pack(fill=tk.X, pady=10)
        
        ttk.Button(buttons_frame, text="Save Configuration", command=self.save_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(buttons_frame, text="Reset to Defaults", command=self.reset_defaults).pack(side=tk.LEFT, padx=5)
    
    def create_binding_row(self, parent, bind_key, label_text, row):
        """Create a single input binding row"""
        row_frame = ttk.Frame(parent)
        row_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(row_frame, text=label_text + ":", font=("Arial", 10), width=20).pack(side=tk.LEFT, padx=5)
        
        self.binding_labels[bind_key] = ttk.Label(row_frame, text=self.format_binding(self.mappings[bind_key]), 
                                                  font=("Arial", 10), width=30, relief=tk.SUNKEN)
        self.binding_labels[bind_key].pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        ttk.Button(row_frame, text="Bind", width=10, 
                  command=lambda: self.start_learning_mode(bind_key)).pack(side=tk.LEFT, padx=5)
    
    def format_binding(self, binding):
        """Format binding tuple for display"""
        input_type, input_num = binding
        if input_type == "axis":
            return f"Axis {input_num}"
        elif input_type == "button":
            return f"Button {input_num}"
        else:
            return "Not Set"
    
    def start_learning_mode(self, bind_key):
        """Start learning mode for a specific binding"""
        self.learning_mode = bind_key
        self.learning_timeout = time.time() + 5  # 5 second timeout
        self.learning_axis_stable = 0  # Track axis value consistency
        self.learning_last_axis = None
        
        # Record baseline axis values to detect only changes
        if self.js:
            pygame.event.pump()
            self.learning_baseline_axes = [self.js.get_axis(i) for i in range(self.js.get_numaxes())]
        else:
            self.learning_baseline_axes = []
        
        self.binding_labels[bind_key].config(text="Waiting for input...", foreground="red")
    
    def detect_input(self):
        """Detect controller input and return (type, value) or None"""
        if not self.js:
            return None
        
        pygame.event.pump()
        
        # Check buttons first (higher priority)
        for i in range(self.js.get_numbuttons()):
            if self.js.get_button(i):
                return ("button", i)
        
        # Check axes with high threshold and require change from baseline
        # This prevents always-active axes (like triggers at -1) from being detected
        for i in range(self.js.get_numaxes()):
            val = self.js.get_axis(i)
            
            # Get baseline value (default to 0 if not recorded)
            baseline = self.learning_baseline_axes[i] if i < len(self.learning_baseline_axes) else 0.0
            
            # Calculate change from baseline
            change = abs(val - baseline)
            
            # Require both high absolute value AND significant change from baseline
            # This prevents:
            # - Trigger axes at -1 from always triggering (they're at -1 at baseline)
            # - Small drifts from being detected
            if abs(val) > 0.95 and change > 0.5:
                return ("axis", i)
        
        return None
    
    def draw_joystick(self):
        """Draw the joystick visualization"""
        self.canvas.delete("all")
        
        # Background circle
        self.canvas.create_oval(20, 20, 180, 180, outline="black", width=2)
        self.canvas.create_line(100, 20, 100, 180, fill="gray", dash=(2, 2))
        self.canvas.create_line(20, 100, 180, 100, fill="gray", dash=(2, 2))
        
        # Deadzone circle
        dz_pixel = self.deadzone * 80
        self.canvas.create_oval(100 - dz_pixel, 100 - dz_pixel, 100 + dz_pixel, 100 + dz_pixel,
                               outline="orange", width=1, dash=(2, 2))
        
        # Joystick position
        x_pixel = 100 + (self.x * 80)
        y_pixel = 100 - (self.y * 80)
        
        self.canvas.create_oval(x_pixel - 8, y_pixel - 8, x_pixel + 8, y_pixel + 8,
                               fill="red", outline="darkred", width=2)
        
        # Center dot
        self.canvas.create_oval(98, 98, 102, 102, fill="black")
    
    def update_deadzone(self, value):
        """Update deadzone value"""
        self.deadzone = float(value)
        self.deadzone_label.config(text=f"{self.deadzone:.2f}")
        self.draw_joystick()
    
    def toggle_sending(self):
        """Toggle command sending"""
        self.sending_enabled = self.send_var.get()
    
    def send_acknowledge(self):
        """Send acknowledge button press to ESP32"""
        msg = ESP_ID + "B"
        self.sock.sendto(msg.encode(), (BROADCAST_IP, UDP_PORT))
        self.add_message(f"[{time.strftime('%H:%M:%S')}] >>> Button sent")
    
    def send_emergency_stop(self):
        """Send emergency stop signal to ESP32"""
        msg = ESP_ID + "E"  # Emergency stop signal
        self.sock.sendto(msg.encode(), (BROADCAST_IP, UDP_PORT))
        self.add_message(f"[{time.strftime('%H:%M:%S')}] >>> EMERGENCY STOP SENT")
    
    def reset_emergency_stop(self):
        """Send emergency stop reset signal to ESP32"""
        msg = ESP_ID + "R"  # Emergency stop reset signal
        self.sock.sendto(msg.encode(), (BROADCAST_IP, UDP_PORT))
        self.add_message(f"[{time.strftime('%H:%M:%S')}] >>> Emergency stop reset sent")
    
    def parse_sensor_data(self, msg):
        """Parse telemetry message and extract sensor values"""
        # Remove timestamp if present (format: "[HH:MM:SS] ...")
        try:
            if ']' in msg:
                msg = msg.split(']', 1)[1].strip()
            
            # Remove "ESP32" prefix if present
            if msg.startswith("ESP32"):
                msg = msg[5:].strip()
            
            # Now parse: "Psi:X | PhiL:Y | PhiR:Z | theta:A | Temp:B | Vbatt:C | EmergencyStop:D"
            parts = msg.split("|")
            for part in parts:
                part = part.strip()
                if ':' in part:
                    key, value = part.split(':', 1)
                    key = key.strip().lower()
                    value_str = value.strip()
                    
                    try:
                        if key == "emergencystop":
                            self.sensor_data["emergencystop"] = value_str
                        elif key == "estl":
                            self.sensor_data["estl"] = float(value_str)
                        elif key == "estr":
                            self.sensor_data["estr"] = float(value_str)
                        else:
                            value = float(value_str)
                            if key == "psi":
                                self.sensor_data["psi"] = value
                            elif key == "phil":
                                self.sensor_data["phiL"] = value
                            elif key == "phir":
                                self.sensor_data["phiR"] = value
                            elif key == "theta":
                                self.sensor_data["theta"] = value
                            elif key == "temp":
                                self.sensor_data["temp"] = value
                            elif key == "vbatt":
                                self.sensor_data["vbatt"] = value
                    except ValueError:
                        pass
        except:
            pass
    
    def add_message(self, msg):
        """Add message to the message box"""
        self.messages.append(msg)
        self.message_box.delete(0, tk.END)
        for msg in reversed(self.messages):
            self.message_box.insert(0, msg)
        
        # Try to parse sensor data from message
        if "Psi:" in msg:
            self.parse_sensor_data(msg)
    
    def reset_defaults(self):
        """Reset to default mappings"""
        self.mappings = DEFAULT_MAPPINGS.copy()
        for key in self.binding_labels:
            self.binding_labels[key].config(text=self.format_binding(self.mappings[key]), foreground="black")
        self.save_config()
    
    def get_mapped_input(self, bind_key):
        """Get the current value of a mapped input"""
        if not self.js:
            return 0.0
        
        if bind_key not in self.mappings:
            return 0.0
        
        input_type, input_num = self.mappings[bind_key]
        
        try:
            if input_type == "axis":
                return self.js.get_axis(input_num)
            elif input_type == "button":
                return float(self.js.get_button(input_num))
        except:
            return 0.0
        
        return 0.0
    
    def update_input_display(self):
        """Update the live input display with current axis and button values"""
        try:
            if not self.js:
                self.input_display.config(state=tk.NORMAL)
                self.input_display.delete("1.0", tk.END)
                self.input_display.insert(tk.END, "No controller connected")
                self.input_display.config(state=tk.DISABLED)
                return
            
            display_text = ""
            
            # Show all axes
            num_axes = self.js.get_numaxes()
            if num_axes > 0:
                display_text += "=== AXES ===\n"
                for i in range(num_axes):
                    val = self.js.get_axis(i)
                    # Visual bar representation
                    bar_length = int(abs(val) * 20)
                    bar = "=" * bar_length
                    if val > 0:
                        bar_display = f"+{bar:20s}"
                    else:
                        bar_display = f"-{bar:20s}"
                    display_text += f"Axis[{i}]: {val:+.3f} {bar_display}\n"
            
            # Show buttons
            display_text += "\n=== BUTTONS ===\n"
            num_buttons = self.js.get_numbuttons()
            pressed_buttons = []
            for i in range(num_buttons):
                if self.js.get_button(i):
                    pressed_buttons.append(f"Button[{i}]")
            
            if pressed_buttons:
                display_text += ", ".join(pressed_buttons)
            else:
                display_text += "(none pressed)"
            
            # Update text widget
            self.input_display.config(state=tk.NORMAL)
            self.input_display.delete("1.0", tk.END)
            self.input_display.insert(tk.END, display_text)
            self.input_display.config(state=tk.DISABLED)
        except:
            pass
    
    def run_loop(self):
        """Main control loop (runs in background thread)"""
        frame_count = 0
        fps_time = time.time()
        last_toggle_state = False
        
        while self.running:
            now = time.time()
            
            # Handle learning mode
            if self.learning_mode:
                if now > self.learning_timeout:
                    # Timeout
                    if self.learning_mode in self.binding_labels:
                        self.binding_labels[self.learning_mode].config(
                            text=self.format_binding(self.mappings[self.learning_mode]), 
                            foreground="black"
                        )
                    self.learning_mode = None
                else:
                    detected = self.detect_input()
                    if detected:
                        # For axes, require stability to avoid drift/triggers
                        if detected[0] == "axis":
                            axis_val = abs(self.js.get_axis(detected[1]))
                            # Require axis to be very firmly pressed (>0.95) to confirm
                            if axis_val > 0.95:
                                self.mappings[self.learning_mode] = detected
                                self.binding_labels[self.learning_mode].config(
                                    text=self.format_binding(detected),
                                    foreground="green"
                                )
                                self.add_message(f"[{time.strftime('%H:%M:%S')}] Bound {self.learning_mode} to {self.format_binding(detected)}")
                                self.learning_mode = None
                        else:
                            # Buttons can be registered immediately
                            self.mappings[self.learning_mode] = detected
                            self.binding_labels[self.learning_mode].config(
                                text=self.format_binding(detected),
                                foreground="green"
                            )
                            self.add_message(f"[{time.strftime('%H:%M:%S')}] Bound {self.learning_mode} to {self.format_binding(detected)}")
                            self.learning_mode = None
            
            # Update joystick input using mappings
            if self.js:
                pygame.event.pump()
                self.x = self.get_mapped_input("x_axis")
                self.y = self.get_mapped_input("y_axis")
            
            # Apply deadzone
            if abs(self.x) > self.deadzone or abs(self.y) > self.deadzone:
                sx = -self.x
                sy = -self.y
            else:
                sx = 0.0
                sy = 0.0
            
            # Send data
            if self.sending_enabled and now - self.last_send >= SEND_PERIOD:
                payload = f"{sx:.3f},{sy:.3f}"
                msg = ESP_ID + "X" + payload  # "X" = control command
                self.sock.sendto(msg.encode(), (BROADCAST_IP, UDP_PORT))
                self.last_send = now
            
            # Check toggle send button
            toggle_state = bool(self.get_mapped_input("toggle_send"))
            if toggle_state and not last_toggle_state:
                self.sending_enabled = not self.sending_enabled
                self.send_var.set(self.sending_enabled)
            last_toggle_state = toggle_state
            
            # Check acknowledge button
            ack_state = bool(self.get_mapped_input("acknowledge"))
            if ack_state:
                # Send acknowledge
                msg = ESP_ID + "B"
                self.sock.sendto(msg.encode(), (BROADCAST_IP, UDP_PORT))
                # Add small delay to prevent multiple sends
                time.sleep(0.2)
            
            # Check emergency stop button
            if bool(self.get_mapped_input("emergency_stop")):
                msg = ESP_ID + "E"  # Emergency stop signal
                self.sock.sendto(msg.encode(), (BROADCAST_IP, UDP_PORT))
                self.add_message(f"[{time.strftime('%H:%M:%S')}] >>> EMERGENCY STOP SENT")
                time.sleep(0.5)  # Longer delay to prevent spam
            
            # Receive messages
            ready, _, _ = select.select([self.sock], [], [], 0.001)
            if ready:
                try:
                    data, addr = self.sock.recvfrom(1024)
                    text = data.decode(errors="ignore").strip()
                    if text and text[0] == LAPTOP_ID:
                        esp_msg = text[1:]
                        self.add_message(f"[{time.strftime('%H:%M:%S')}] {esp_msg}")
                except Exception as e:
                    pass
            
            # Update GUI (throttled)
            frame_count += 1
            if frame_count % 2 == 0:  # Update every 2 frames instead of 5 (20ms vs 50ms)
                self.update_display()
                self.update_input_display()
                
                # Calculate FPS
                elapsed = time.time() - fps_time
                if elapsed >= 1.0:
                    fps = frame_count / elapsed
                    self.fps_label.config(text=f"FPS: {fps:.1f}")
                    frame_count = 0
                    fps_time = time.time()
            
            time.sleep(0.01)
    
    def update_display(self):
        """Update the GUI display"""
        try:
            self.x_label.config(text=f"{self.x:+.3f}")
            self.y_label.config(text=f"{self.y:+.3f}")
            self.draw_joystick()
            
            # Update sensor labels
            self.psi_label.config(text=f"{self.sensor_data['psi']:.1f}")
            self.phiL_label.config(text=f"{self.sensor_data['phiL']:.1f}")
            self.phiR_label.config(text=f"{self.sensor_data['phiR']:.1f}")
            self.theta_label.config(text=f"{self.sensor_data['theta']:.1f}")
            # Estimated angles
            try:
                self.estL_label.config(text=f"{self.sensor_data['estl']:.1f}")
                self.estR_label.config(text=f"{self.sensor_data['estr']:.1f}")
            except:
                pass
            
            # Update temperature with color warning if high
            temp = self.sensor_data['temp']
            self.temp_label.config(text=f"{temp:.1f}")
            if temp > 65.0:
                self.temp_label.config(foreground="red")
            else:
                self.temp_label.config(foreground="green")
            
            # Update battery voltage with color warning if low
            vbatt = self.sensor_data['vbatt']
            self.vbatt_label.config(text=f"{vbatt:.2f}")
            if vbatt < 14.8:
                self.vbatt_label.config(foreground="red")
            else:
                self.vbatt_label.config(foreground="green")
            
            # Update emergency stop status
            emergency_status = self.sensor_data['emergencystop']
            self.emergencystop_label.config(text=emergency_status)
            if emergency_status == "ON":
                self.emergencystop_label.config(foreground="red")
            else:
                self.emergencystop_label.config(foreground="green")
        except:
            pass
    
    def on_close(self):
        """Handle window close"""
        self.running = False
        self.save_config()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = ControllerGUI(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("\nShutdown requested by user")
        app.on_close()
