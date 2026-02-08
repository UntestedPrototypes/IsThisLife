import sys
import struct
import serial
import serial.tools.list_ports
import threading
import time
import pygame
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, font

# -------------------- Serial Config --------------------
BAUD_RATE = 115200
ser = None

START_BYTE = 0xAA
END_BYTE = 0x55

PACKET_CONTROL = 0
PACKET_ESTOP = 1
PACKET_ESTOP_CLEAR = 2
PACKET_CONFIRM = 4  # Confirmation packet
PACKET_START_SEQUENCE = 6  # Start sequence
ROBOT_ID = 1  # Default robot to control

# ========== Sequence ID Definitions ==========
# Match these with robot.cpp!
SEQUENCE_CALIBRATION_FULL = 0
SEQUENCE_CALIBRATION_GYRO = 1
SEQUENCE_CALIBRATION_MOTORS = 2
SEQUENCE_DEMO_DANCE = 3
SEQUENCE_SENSOR_TEST = 4
SEQUENCE_PATH_FOLLOW = 5
# Add more sequences as needed...

# -------------------- Pygame Controller --------------------
pygame.init()
pygame.joystick.init()
joystick = None
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Joystick initialized:", joystick.get_name())

# -------------------- Tkinter Dashboard --------------------
class TkDashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Controller Dashboard")
        self.root.geometry("1050x550")
        self.rx_buffer = bytearray()

        # -------------------- Controller Mapping --------------------
        self.mappings = {
            "vx": ("axis", 1),
            "vy": ("axis", 0),
            "omega": ("axis", 2),
            "estop": ("button", 0),
            "arm": ("button", 1)
        }
        self.learning_mode = None
        self.learning_timeout = 0
        self.learning_baseline_axes = []

        # -------------------- Tabs --------------------
        self.tabs = ttk.Notebook(root)
        self.tabs.pack(expand=True, fill=tk.BOTH)

        self.config_tab = ttk.Frame(self.tabs)
        self.live_tab = ttk.Frame(self.tabs)
        self.game_tab = ttk.Frame(self.tabs)

        self.tabs.add(self.config_tab, text="Configuration")
        self.tabs.add(self.live_tab, text="Live View")
        self.tabs.add(self.game_tab, text="Game Controller")

        # Per-robot states
        self.robot_state = {}  # {robot_id: {"E_STOP": False, "ARM": False}}

        self.robot_widgets = {}  # Will hold Live tab labels

        self.init_config_tab()
        self.init_live_tab()
        self.init_game_tab()

        # Start serial read loop in separate thread
        self.running = True
        threading.Thread(target=self.serial_read_loop, daemon=True).start()

        # Start pygame controller update loop
        self.root.after(50, self.update_game_controller)

    # -------------------- CONFIG TAB --------------------
    def init_config_tab(self):
        frame = self.config_tab

        tk.Label(frame, text="Serial Port").pack()
        self.com_dropdown = ttk.Combobox(frame, width=20)
        self.com_dropdown.pack()

        self.refresh_button = tk.Button(frame, text="Refresh COM Ports", command=self.refresh_ports)
        self.refresh_button.pack(pady=2)

        self.connect_button = tk.Button(frame, text="Connect", command=self.connect_controller)
        self.connect_button.pack(pady=2)

        self.status_label = tk.Label(frame, text="Controller: Disconnected", fg="red", font=("Arial", 10, "bold"))
        self.status_label.pack(pady=2)

        # Debug and Telemetry terminals
        terminal_frame = tk.Frame(frame)
        terminal_frame.pack(expand=True, fill=tk.BOTH)

        self.debug_terminal = scrolledtext.ScrolledText(terminal_frame, bg="#111", fg="#9cdcfe", font=("Consolas", 10))
        self.debug_terminal.pack(side=tk.LEFT, expand=True, fill=tk.BOTH)

        self.telemetry_terminal = scrolledtext.ScrolledText(terminal_frame, bg="#111", fg="#4ec9b0", font=("Consolas", 10))
        self.telemetry_terminal.pack(side=tk.LEFT, expand=True, fill=tk.BOTH)

        self.refresh_ports()

    # -------------------- LIVE TAB --------------------
    def init_live_tab(self):
        frame = self.live_tab
        bold_font = font.Font(weight="bold")
        for robot_id in range(1, 3):  # assume 2 robots
            self.robot_state[robot_id] = {"E_STOP": False, "ARM": False}

            group = tk.LabelFrame(frame, text=f"Robot {robot_id}", padx=5, pady=5)
            group.pack(fill=tk.X, padx=5, pady=5)

            labels = {}
            row = 0
            for key in ["HB", "STATUS", "BATT", "TEMP", "ERR", "RTT"]:
                tk.Label(group, text=f"{key}:", width=10, anchor=tk.W).grid(row=row, column=0, sticky=tk.W)
                val_label = tk.Label(group, text="N/A", width=15, anchor=tk.W)
                val_label.grid(row=row, column=1, sticky=tk.W)
                labels[key] = val_label
                row += 1

            # E-STOP status label
            tk.Label(group, text="E-STOP:", width=10, anchor=tk.W).grid(row=row, column=0, sticky=tk.W)
            estop_label = tk.Label(group, text="OFF", width=10, anchor=tk.W, font=bold_font, fg="green")
            estop_label.grid(row=row, column=1, sticky=tk.W)
            labels["E_STOP"] = estop_label
            row += 1

            # E-STOP status label
            tk.Label(group, text="ARM:", width=10, anchor=tk.W).grid(row=row, column=0, sticky=tk.W)
            arm_label = tk.Label(group, text="OFF", width=10, anchor=tk.W, font=bold_font, fg="green")
            arm_label.grid(row=row, column=1, sticky=tk.W)
            labels["ARM"] = arm_label
            row += 1

            # ========== Sequence Buttons ==========
            seq_label = tk.Label(group, text="Sequences:", font=("Arial", 9, "bold"))
            seq_label.grid(row=row, column=0, columnspan=2, pady=(10, 5), sticky=tk.W)
            row += 1
            
            # Calibration sequences
            calib_frame = tk.LabelFrame(group, text="Calibration", padx=5, pady=3)
            calib_frame.grid(row=row, column=0, columnspan=2, pady=2, sticky=tk.EW)
            tk.Button(calib_frame, text="Full", width=8,
                     command=lambda rid=robot_id: send_start_sequence(rid, SEQUENCE_CALIBRATION_FULL)).pack(side=tk.LEFT, padx=2)
            tk.Button(calib_frame, text="Gyro", width=8,
                     command=lambda rid=robot_id: send_start_sequence(rid, SEQUENCE_CALIBRATION_GYRO)).pack(side=tk.LEFT, padx=2)
            tk.Button(calib_frame, text="Motors", width=8,
                     command=lambda rid=robot_id: send_start_sequence(rid, SEQUENCE_CALIBRATION_MOTORS)).pack(side=tk.LEFT, padx=2)
            row += 1
            
            # Demo sequences
            demo_frame = tk.LabelFrame(group, text="Demo", padx=5, pady=3)
            demo_frame.grid(row=row, column=0, columnspan=2, pady=2, sticky=tk.EW)
            tk.Button(demo_frame, text="Dance", width=8,
                     command=lambda rid=robot_id: send_start_sequence(rid, SEQUENCE_DEMO_DANCE)).pack(side=tk.LEFT, padx=2)
            tk.Button(demo_frame, text="Sensors", width=8,
                     command=lambda rid=robot_id: send_start_sequence(rid, SEQUENCE_SENSOR_TEST)).pack(side=tk.LEFT, padx=2)
            tk.Button(demo_frame, text="Path", width=8,
                     command=lambda rid=robot_id: send_start_sequence(rid, SEQUENCE_PATH_FOLLOW)).pack(side=tk.LEFT, padx=2)

            self.robot_widgets[robot_id] = labels

    # -------------------- GAME TAB --------------------
    def init_game_tab(self):
        frame = self.game_tab
        self.game_label = tk.Label(frame, text="Joystick control values will appear here")
        self.game_label.pack(pady=5)

        binding_frame = tk.Frame(frame)
        binding_frame.pack(pady=5)

        # VX binding
        tk.Label(binding_frame, text="VX:").grid(row=0, column=0)
        self.vx_bind_label = tk.Label(binding_frame, text="Axis 1", width=15, relief=tk.SUNKEN)
        self.vx_bind_label.grid(row=0, column=1)
        tk.Button(binding_frame, text="Bind", command=lambda: self.start_learning_mode("vx")).grid(row=0, column=2)

        # VY binding
        tk.Label(binding_frame, text="VY:").grid(row=1, column=0)
        self.vy_bind_label = tk.Label(binding_frame, text="Axis 0", width=15, relief=tk.SUNKEN)
        self.vy_bind_label.grid(row=1, column=1)
        tk.Button(binding_frame, text="Bind", command=lambda: self.start_learning_mode("vy")).grid(row=1, column=2)

        # Omega binding
        tk.Label(binding_frame, text="Omega:").grid(row=2, column=0)
        self.omega_bind_label = tk.Label(binding_frame, text="Axis 2", width=15, relief=tk.SUNKEN)
        self.omega_bind_label.grid(row=2, column=1)
        tk.Button(binding_frame, text="Bind", command=lambda: self.start_learning_mode("omega")).grid(row=2, column=2)

        # E-STOP binding
        tk.Label(binding_frame, text="E-STOP:").grid(row=3, column=0)
        self.estop_bind_label = tk.Label(binding_frame, text="Button 0", width=15, relief=tk.SUNKEN)
        self.estop_bind_label.grid(row=3, column=1)
        tk.Button(binding_frame, text="Bind", command=lambda: self.start_learning_mode("estop")).grid(row=3, column=2)

        # ARM binding
        tk.Label(binding_frame, text="ARM:").grid(row=4, column=0)
        self.arm_bind_label = tk.Label(binding_frame, text="Button 1", width=15, relief=tk.SUNKEN)
        self.arm_bind_label.grid(row=4, column=1)
        tk.Button(binding_frame, text="Bind", command=lambda: self.start_learning_mode("arm")).grid(row=4, column=2)

        self.live_values_label = tk.Label(frame, text="Waiting for inputs...")
        self.live_values_label.pack(pady=5)

        self.joy_canvas = tk.Canvas(frame, width=300, height=300, bg="white")
        self.joy_canvas.pack()
        self.joy_canvas.create_oval(50, 50, 250, 250, outline="gray", width=2)
        self.joy_thumb = self.joy_canvas.create_oval(145, 145, 155, 155, fill="blue")

    def start_learning_mode(self, key):
        if not joystick:
            messagebox.showwarning("No Joystick", "No joystick detected!")
            return
        self.learning_mode = key
        self.learning_timeout = time.time() + 5.0
        self.learning_baseline_axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
        self.game_label.config(text=f"Learning {key.upper()}... Move input now!")

    def check_learning_mode(self):
        if not self.learning_mode or time.time() > self.learning_timeout:
            if self.learning_mode:
                self.game_label.config(text="Timeout, try again.")
                self.learning_mode = None
            return

        key = self.learning_mode
        for i in range(joystick.get_numaxes()):
            val = joystick.get_axis(i)
            if abs(val - self.learning_baseline_axes[i]) > 0.3:
                self.mappings[key] = ("axis", i)
                self.game_label.config(text=f"{key.upper()} mapped to Axis {i}")
                label_map = {
                    "vx": self.vx_bind_label,
                    "vy": self.vy_bind_label,
                    "omega": self.omega_bind_label,
                    "estop": self.estop_bind_label,
                    "arm": self.arm_bind_label
                }
                label_map[key].config(text=f"Axis {i}")
                self.learning_mode = None
                return

        for i in range(joystick.get_numbuttons()):
            if joystick.get_button(i):
                self.mappings[key] = ("button", i)
                self.game_label.config(text=f"{key.upper()} mapped to Button {i}")
                label_map = {
                    "vx": self.vx_bind_label,
                    "vy": self.vy_bind_label,
                    "omega": self.omega_bind_label,
                    "estop": self.estop_bind_label,
                    "arm": self.arm_bind_label
                }
                label_map[key].config(text=f"Button {i}")
                self.learning_mode = None
                return

    def update_game_controller(self):
        if not joystick:
            self.root.after(50, self.update_game_controller)
            return

        pygame.event.pump()
        self.check_learning_mode()

        def get_input(key):
            t, idx = self.mappings[key]
            if t == "axis":
                return joystick.get_axis(idx)
            return joystick.get_button(idx)

        vx = get_input("vx")
        vy = -get_input("vy")
        omega = get_input("omega")

        robot_id = 1
        # Send joystick values to Robot 1 only; other robots get periodic zero packets
        if self.robot_state[robot_id]["ARM"] == 1:  # Only send control if ARM is active
            send_control(int(vx * 127), int(vy * 127), int(omega * 127))
        else:
            send_control(0, 0, 0)  # Send zero velocities if not armed

        # Map joystick buttons to Robot 1
        estop_val = get_input("estop")
        arm_val = get_input("arm")
        prev_arm_val = self.robot_state[robot_id].get("_prev_arm_input", False)

        # Update states if different
        if estop_val != self.robot_state[robot_id]["E_STOP"]:
            send_estop()
        
        # Only act on the press (edge-trigger), not on every loop
        if arm_val and not prev_arm_val: 
            # Flip the current state
            self.robot_state[robot_id]["ARM"] = not self.robot_state[robot_id]["ARM"]
            send_arm()  # send the new state to the robot
        
        self.robot_state[robot_id]["_prev_arm_input"] = arm_val

        # Update live values label
        self.live_values_label.config(
            text=f"VX={int(vx*127)} | VY={int(vy*127)} | Ω={int(omega*127)} | "
                 f"E-STOP={'ON' if estop_val else 'OFF'} | "
                 f"ARM={'ON' if arm_val else 'OFF'}"
        )

        self.update_joystick_canvas(vx, vy)
        self.root.after(50, self.update_game_controller)

    def update_joystick_canvas(self, vx, vy):
        cx, cy = 150, 150
        radius = 100
        y = cx + int(vx * radius)
        x = cy + int(-vy * radius)
        self.joy_canvas.coords(self.joy_thumb, x-5, y-5, x+5, y+5)

    # ========== NEW: Show confirmation dialog ==========
    def show_confirmation_dialog(self, robot_id, step_id, message):
        """Show a dialog asking user to confirm a robot calibration step"""
        def on_approve():
            send_confirmation(robot_id, step_id, True)
            dialog.destroy()
        
        def on_deny():
            send_confirmation(robot_id, step_id, False)
            dialog.destroy()
        
        # Create a modal dialog
        dialog = tk.Toplevel(self.root)
        dialog.title(f"Robot {robot_id} - Confirmation Required")
        dialog.geometry("400x150")
        dialog.transient(self.root)
        dialog.grab_set()
        
        tk.Label(dialog, text=f"Robot {robot_id} requesting confirmation:", font=("Arial", 10, "bold")).pack(pady=10)
        tk.Label(dialog, text=message, font=("Arial", 12)).pack(pady=5)
        
        button_frame = tk.Frame(dialog)
        button_frame.pack(pady=10)
        
        tk.Button(button_frame, text="✓ Approve", bg="green", fg="white", width=12, command=on_approve).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="✗ Deny", bg="red", fg="white", width=12, command=on_deny).pack(side=tk.LEFT, padx=5)
        
        # Log the request
        self.debug_terminal.insert(tk.END, f">> Confirmation request from Robot {robot_id}: {message}\n")

    # -------------------- Periodic Control Loop for all robots --------------------
    def periodic_control_loop(self):
        if ser and ser.is_open:
            for robot_id in self.robot_state:
                # If Robot 1, send joystick values; else, send zeros
                if robot_id == 1:
                    continue  # Robot 1 handled in update_game_controller
                vx = 0
                vy = 0
                omega = 0
                pkt = bytes([PACKET_CONTROL, robot_id, vx & 0xFF, vy & 0xFF, omega & 0xFF])
                try:
                    ser.write(pkt)
                except Exception as e:
                    self.debug_terminal.insert(tk.END, f">> Control send error: {e}\n")
        self.root.after(50, self.periodic_control_loop)  # 20 Hz

    # -------------------- Serial Helpers --------------------
    def refresh_ports(self):
        self.com_dropdown['values'] = [p.device for p in serial.tools.list_ports.comports()]
        if self.com_dropdown['values']:
            self.com_dropdown.current(0)

    def connect_controller(self):
        global ser
        if ser and ser.is_open:
            ser.close()
            ser = None
            self.status_label.config(text="Controller: Disconnected", fg="red")
            self.debug_terminal.insert(tk.END, ">> Disconnected\n")
            self.connect_button.config(text="Connect")
            return

        port = self.com_dropdown.get()
        try:
            ser = serial.Serial(port, BAUD_RATE, timeout=0.01)
            self.rx_buffer.clear()
            self.status_label.config(text=f"Controller: Connected ({port})", fg="green")
            self.debug_terminal.insert(tk.END, f">> Connected on {port}\n")
            self.connect_button.config(text="Disconnect")
            
            # Start periodic control packets
            self.periodic_control_loop()
        except Exception as e:
            self.debug_terminal.insert(tk.END, f">> Connection failed: {e}\n")

    # -------------------- Serial Read --------------------
    def serial_read_loop(self):
        while self.running:
            if ser and ser.is_open:
                try:
                    data = ser.read(ser.in_waiting)
                    if data:
                        self.rx_buffer.extend(data)
                        while b"\n" in self.rx_buffer:
                            newline_idx = self.rx_buffer.find(b"\n")
                            line = self.rx_buffer[:newline_idx]
                            self.rx_buffer = self.rx_buffer[newline_idx + 1:]
                            try:
                                text = line.decode("utf-8", errors="replace").strip()
                                if not text:
                                    continue
                                
                                # ========== NEW: Handle confirmation request ==========
                                if text.startswith("CONFIRM_REQ"):
                                    parts = {}
                                    for p in text.split():
                                        if "=" in p:
                                            key, val = p.split("=", 1)
                                            parts[key] = val
                                    
                                    robot_id = int(parts.get("ID", 0))
                                    step_id = int(parts.get("STEP", 0))
                                    message = parts.get("MSG", "Unknown step")
                                    
                                    # Show confirmation dialog on main thread
                                    self.root.after(0, lambda: self.show_confirmation_dialog(robot_id, step_id, message))
                                
                                elif text.startswith("DEBUG:"):
                                    self.debug_terminal.insert(tk.END, text + "\n")
                                elif text.startswith("Python"):
                                    self.debug_terminal.insert(tk.END, text + "\n")
                                elif text.startswith("ID="):
                                    telemetry = {}
                                    for p in text.split():
                                        key, val = p.split("=")
                                        if key == "ERR":
                                            telemetry[key] = int(val, 16)
                                        else:
                                            telemetry[key] = int(val)
                                    robot_id = telemetry["ID"]
                                    if robot_id in self.robot_widgets:
                                        labels = self.robot_widgets[robot_id]
                                        labels["HB"].config(text=f"{telemetry['HB']:03d}")
                                        
                                        # ========== Show status text ==========
                                        status = telemetry['STATUS']
                                        if status == 3:  # STATUS_RUNNING_SEQUENCE
                                            labels["STATUS"].config(text="RUNNING SEQ", fg="blue")
                                        elif status == 2:  # STATUS_WAITING_CONFIRM
                                            labels["STATUS"].config(text="WAITING", fg="orange")
                                        elif status == 1:  # STATUS_ESTOP
                                            labels["STATUS"].config(text="E-STOP", fg="red")
                                        else:  # STATUS_OK
                                            labels["STATUS"].config(text="OK", fg="green")
                                        
                                        labels["BATT"].config(text=f"{telemetry['BATT']} mV")
                                        labels["TEMP"].config(text=f"{telemetry['TEMP']} C")
                                        labels["ERR"].config(text=f"0x{telemetry['ERR']:02X}")
                                        labels["RTT"].config(text=f"{telemetry['RTT']} ms")
                                        
                                        # update E-STOP status based on telemetry
                                        estop_off= telemetry["STATUS"] != PACKET_ESTOP
                                        labels["E_STOP"].config(
                                            text="ON" if not estop_off else "OFF",
                                            fg="red" if not estop_off else "green"
                                        )

                                        # update ARM status, disable if E-STOP is active
                                        arm_active = self.robot_state[robot_id]["ARM"]
                                        labels["ARM"].config(
                                            text="ARMED" if arm_active else "UNARMED",
                                            fg="red" if arm_active else "green"
                                        )

                            except Exception as e:
                                self.debug_terminal.insert(tk.END, f"Serial parse error: {e}\n")
                except Exception as e:
                    self.debug_terminal.insert(tk.END, f">> Serial error: {e}\n")
            time.sleep(0.02)  # 50 Hz


# -------------------- Serial Send Functions --------------------
def send_control(vx, vy, omega):
    if not ser or not ser.is_open:
        return
    vx_byte = int(max(-127, min(127, vx)))
    vy_byte = int(max(-127, min(127, vy)))
    omega_byte = int(max(-127, min(127, omega)))
    pkt = bytes([PACKET_CONTROL, ROBOT_ID, vx_byte & 0xFF, vy_byte & 0xFF, omega_byte & 0xFF])
    ser.write(pkt)

def send_estop():
    if ser and ser.is_open:
        ser.write(bytes([PACKET_ESTOP, ROBOT_ID]))

def send_arm():
    if ser and ser.is_open:
        ser.write(bytes([PACKET_ESTOP_CLEAR, ROBOT_ID]))

# ========== NEW: Send confirmation response ==========
def send_confirmation(robot_id, step_id, approved):
    """Send confirmation response to robot"""
    if ser and ser.is_open:
        pkt = bytes([PACKET_CONFIRM, robot_id, step_id, 1 if approved else 0])
        ser.write(pkt)
        print(f"Sent confirmation: robot={robot_id} step={step_id} approved={approved}")

# ========== Send start sequence command ==========
def send_start_sequence(robot_id, sequence_id):
    """Send command to start a sequence
    
    Available sequences:
    - SEQUENCE_CALIBRATION_FULL (0): Full calibration
    - SEQUENCE_CALIBRATION_GYRO (1): Gyro calibration only
    - SEQUENCE_CALIBRATION_MOTORS (2): Motor test only
    - SEQUENCE_DEMO_DANCE (3): Demo dance routine
    - SEQUENCE_SENSOR_TEST (4): Test all sensors
    - SEQUENCE_PATH_FOLLOW (5): Follow pre-programmed path
    """
    if ser and ser.is_open:
        pkt = bytes([PACKET_START_SEQUENCE, robot_id, sequence_id])
        ser.write(pkt)
        print(f"Sent start sequence: robot={robot_id} sequence_id={sequence_id}")
    else:
        print("Error: Not connected to controller")

# -------------------- MAIN --------------------
if __name__ == "__main__":
    root = tk.Tk()
    dashboard = TkDashboard(root)
    root.mainloop()