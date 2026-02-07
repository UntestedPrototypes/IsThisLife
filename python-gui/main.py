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
ROBOT_ID = 1  # Default robot to control

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

        self.live_values_label = tk.Label(frame, text="")
        self.live_values_label.pack(pady=5)

        # Joystick visualization canvas
        self.joy_canvas = tk.Canvas(frame, width=300, height=300, bg="#222")
        self.joy_canvas.pack(pady=5)
        self.joy_canvas.create_oval(50, 50, 250, 250, outline="white", width=2)  # Outer boundary
        self.joy_thumb = self.joy_canvas.create_oval(145, 145, 155, 155, fill="cyan")  # Thumb indicator

    # -------------------- Joystick Binding --------------------
    def start_learning_mode(self, action_key):
        if not joystick:
            return
        self.learning_mode = action_key
        self.learning_timeout = time.time() + 5
        self.learning_baseline_axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
        getattr(self, f"{action_key}_bind_label").config(text="Waiting for input...", fg="red")

    def detect_input(self):
        if not joystick:
            return None
        pygame.event.pump()
        for i in range(joystick.get_numbuttons()):
            if joystick.get_button(i):
                return ("button", i)
        for i in range(joystick.get_numaxes()):
            val = joystick.get_axis(i)
            baseline = self.learning_baseline_axes[i] if i < len(self.learning_baseline_axes) else 0.0
            if abs(val) > 0.95 and abs(val - baseline) > 0.5:
                return ("axis", i)
        return None

    def process_learning_mode(self):
        if not self.learning_mode:
            return
        if time.time() > self.learning_timeout:
            getattr(self, f"{self.learning_mode}_bind_label").config(text=f"{self.learning_mode.upper()}", fg="black")
            self.learning_mode = None
            return
        detected = self.detect_input()
        if detected:
            self.mappings[self.learning_mode] = detected
            getattr(self, f"{self.learning_mode}_bind_label").config(text=f"{detected[0].capitalize()} {detected[1]}", fg="green")
            self.learning_mode = None

    # -------------------- Game Controller Loop --------------------
    def update_game_controller(self):
        if not joystick or not ser or not ser.is_open:
            self.root.after(50, self.update_game_controller)
            return

        self.process_learning_mode()
        pygame.event.pump()

        def get_input(key):
            typ, idx = self.mappings[key]
            if typ == "axis":
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
                                if text.startswith("DEBUG:"):
                                    self.debug_terminal.insert(tk.END, text + "\n")
                                if text.startswith("Python"):
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
                                        labels["STATUS"].config(text=f"{telemetry['STATUS']}")
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

                                        # Optional: force the ARM state to False if E-STOP is active
                                        # if not estop_off and self.robot_state[robot_id]["ARM"]:
                                        #     self.robot_state[robot_id]["ARM"] = False

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

# -------------------- MAIN --------------------
if __name__ == "__main__":
    root = tk.Tk()
    dashboard = TkDashboard(root)
    root.mainloop()
