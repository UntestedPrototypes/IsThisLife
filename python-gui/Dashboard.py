import sys
import struct
import serial
import serial.tools.list_ports
from PySide6 import QtWidgets, QtCore
import pygame

# -------------------- Serial Config --------------------
BAUD_RATE = 115200
ser = None

START_BYTE = 0xAA
END_BYTE   = 0x55

# AckTelemetryPacket (must match packets.h exactly!)
# < = little endian
ACK_FMT = "<BBBBBHbBH"
ACK_SIZE = struct.calcsize(ACK_FMT)

# -------------------- Pygame Controller --------------------
pygame.init()
pygame.joystick.init()
joystick = None
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Joystick initialized:", joystick.get_name())

PACKET_CONTROL = 0
PACKET_ESTOP = 1
PACKET_ESTOP_CLEAR = 2
ROBOT_ID = 1  # Default robot to control

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

# -------------------- Dashboard --------------------
class SimpleDashboard(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Controller Dashboard")
        self.resize(950, 520)

        self.rx_buffer = bytearray()

        main_layout = QtWidgets.QVBoxLayout(self)

        # -------------------- Tabs --------------------
        self.tabs = QtWidgets.QTabWidget()
        main_layout.addWidget(self.tabs)

        self.config_tab = QtWidgets.QWidget()
        self.live_tab = QtWidgets.QWidget()
        self.game_tab = QtWidgets.QWidget()  # <-- New Game Controller tab

        self.tabs.addTab(self.config_tab, "Configuration")
        self.tabs.addTab(self.live_tab, "Live View")
        self.tabs.addTab(self.game_tab, "Game Controller")  # Add new tab

        self.init_config_tab()
        self.init_live_tab()
        self.init_game_tab()  # Initialize new tab
        self.refresh_ports()

        # Game controller timer
        self.game_timer = QtCore.QTimer()
        self.game_timer.timeout.connect(self.update_game_controller)
        self.game_timer.start(50)  # 20 Hz update

    # -------------------- CONFIG TAB --------------------
    def init_config_tab(self):
        layout = QtWidgets.QVBoxLayout(self.config_tab)

        self.com_dropdown = QtWidgets.QComboBox()
        self.refresh_button = QtWidgets.QPushButton("Refresh COM Ports")
        self.refresh_button.clicked.connect(self.refresh_ports)

        self.connect_button = QtWidgets.QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_controller)

        self.status_label = QtWidgets.QLabel("Controller: Disconnected")
        self.status_label.setStyleSheet("color:red; font-weight:bold;")

        # ---- Terminals ----
        self.debug_terminal = QtWidgets.QTextEdit()
        self.debug_terminal.setReadOnly(True)
        self.debug_terminal.setStyleSheet(
            "background:#111; color:#9cdcfe; font-family:monospace;"
        )

        self.telemetry_terminal = QtWidgets.QTextEdit()
        self.telemetry_terminal.setReadOnly(True)
        self.telemetry_terminal.setStyleSheet(
            "background:#111; color:#4ec9b0; font-family:monospace;"
        )

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.addWidget(self.debug_terminal)
        splitter.addWidget(self.telemetry_terminal)
        splitter.setSizes([500, 500])

        layout.addWidget(QtWidgets.QLabel("Serial Port"))
        layout.addWidget(self.com_dropdown)
        layout.addWidget(self.refresh_button)
        layout.addWidget(self.connect_button)
        layout.addWidget(self.status_label)
        layout.addWidget(QtWidgets.QLabel("Debug (left)   |   Telemetry (right)"))
        layout.addWidget(splitter)

    # -------------------- LIVE TAB --------------------
    def init_live_tab(self):
        layout = QtWidgets.QVBoxLayout(self.live_tab)

        self.robot_widgets = {}  # key = robot_id, value = QTextEdit

        # For now, assume NUM_ROBOTS = 2
        for robot_id in range(1, 2 + 1):
            group = QtWidgets.QGroupBox(f"Robot {robot_id}")
            group_layout = QtWidgets.QVBoxLayout()
            te = QtWidgets.QTextEdit()
            te.setReadOnly(True)
            te.setStyleSheet("background:#111; color:#4ec9b0; font-family:monospace;")
            group_layout.addWidget(te)
            group.setLayout(group_layout)
            layout.addWidget(group)
            self.robot_widgets[robot_id] = te

        layout.addStretch()

    # -------------------- GAME CONTROLLER TAB --------------------
    def init_game_tab(self):
        layout = QtWidgets.QVBoxLayout(self.game_tab)

        self.game_label = QtWidgets.QLabel("Joystick control values will appear here")
        layout.addWidget(self.game_label)

        # -------------------- Bindings --------------------
        bind_layout = QtWidgets.QFormLayout()
        
        self.vx_bind = QtWidgets.QComboBox()
        self.vy_bind = QtWidgets.QComboBox()
        self.omega_bind = QtWidgets.QComboBox()
        self.estop_btn_bind = QtWidgets.QComboBox()
        self.arm_btn_bind = QtWidgets.QComboBox()      


        # Populate with axes
        self.update_joystick_bind_options()
        self.update_joystick_button_options()

        bind_layout.addRow("VX Axis:", self.vx_bind)
        bind_layout.addRow("VY Axis:", self.vy_bind)
        bind_layout.addRow("Omega Axis:", self.omega_bind)
        layout.addWidget(QtWidgets.QLabel("Button Bindings"))
        bind_btn_layout = QtWidgets.QFormLayout()
        bind_btn_layout.addRow("E-STOP Button:", self.estop_btn_bind)
        bind_btn_layout.addRow("ARM Button:", self.arm_btn_bind)
        
        layout.addLayout(bind_layout)

        # -------------------- Toggle Buttons --------------------
        self.estop_toggle_btn = QtWidgets.QPushButton("E-STOP: OFF")
        self.estop_toggle_btn.setCheckable(True)
        self.estop_toggle_btn.clicked.connect(self.toggle_estop)
        layout.addWidget(self.estop_toggle_btn)

        self.arm_toggle_btn = QtWidgets.QPushButton("ARM: OFF")
        self.arm_toggle_btn.setCheckable(True)
        self.arm_toggle_btn.clicked.connect(self.toggle_arm)
        layout.addWidget(self.arm_toggle_btn)

        self.confirm_btn = QtWidgets.QPushButton("CONFIRM")
        self.confirm_btn.clicked.connect(send_arm)  # one-shot CONFIRM
        layout.addWidget(self.confirm_btn)

        # Live values display
        self.live_values_label = QtWidgets.QLabel("")
        layout.addWidget(self.live_values_label)

        # Initialize toggle states
        self.estop_active = False
        self.arm_active = False

    # -------------------- Toggle Handlers --------------------
    def toggle_estop(self):
        self.estop_active = self.estop_toggle_btn.isChecked()
        if self.estop_active:
            send_estop()
            self.estop_toggle_btn.setText("E-STOP: ON")
            # Automatically disarm the robot if E-STOP is hit
            if self.arm_active:
                self.arm_active = False
                self.arm_toggle_btn.setChecked(False)
                self.arm_toggle_btn.setText("ARM: OFF")
        else:
            send_arm()  # clear E-STOP
            self.estop_toggle_btn.setText("E-STOP: OFF")


    def toggle_arm(self):
        self.arm_active = self.arm_toggle_btn.isChecked()
        if self.arm_active:
            send_arm()
            self.arm_toggle_btn.setText("ARM: ON")
        else:
            # Could optionally send a “disarm” packet if your protocol supports it
            self.arm_toggle_btn.setText("ARM: OFF")

    # -------------------- Update Joystick Bind Options --------------------
    def update_joystick_bind_options(self):
        if not joystick:
            return
        
        axes = [f"Axis {i}" for i in range(joystick.get_numaxes())]

        # Only populate the axis dropdowns
        for combo in [self.vx_bind, self.vy_bind, self.omega_bind]:
            combo.clear()
            combo.addItems(axes)

    def update_joystick_button_options(self):
        if not joystick:
            return
        buttons = [f"Button {i}" for i in range(joystick.get_numbuttons())]
        for combo in [self.estop_btn_bind, self.arm_btn_bind]:
            combo.clear()
            combo.addItems(buttons)


    # -------------------- Updated Game Controller Timer --------------------
    def update_game_controller(self):
        if not joystick or not ser or not ser.is_open:
            return

        pygame.event.pump()

        # Axes control
        vx_axis = self.vx_bind.currentIndex()
        vy_axis = self.vy_bind.currentIndex()
        omega_axis = self.omega_bind.currentIndex()

        vx = int(joystick.get_axis(vx_axis) * 127)
        vy = int(-joystick.get_axis(vy_axis) * 127)
        omega = int(joystick.get_axis(omega_axis) * 127)

        # Send control packet
        send_control(vx, vy, omega)

        # Handle buttons
        estop_btn = self.estop_btn_bind.currentIndex()
        arm_btn = self.arm_btn_bind.currentIndex()

        # E-STOP toggle
        estop_pressed = joystick.get_button(estop_btn)
        if estop_pressed != self.estop_active:
            self.estop_toggle_btn.setChecked(estop_pressed)
            self.toggle_estop()

        # ARM toggle (only if E-STOP is not active)
        arm_pressed = joystick.get_button(arm_btn)
        if not self.estop_active:
            if arm_pressed != self.arm_active:
                self.arm_toggle_btn.setChecked(arm_pressed)
                self.toggle_arm()
        else:
            # ensure ARM is OFF while E-STOP is ON
            if self.arm_active:
                self.arm_active = False
                self.arm_toggle_btn.setChecked(False)
                self.arm_toggle_btn.setText("ARM: OFF")

        # Update live values display
        self.live_values_label.setText(
            f"VX={vx} | VY={vy} | Ω={omega} | "
            f"E-STOP={'ON' if self.estop_active else 'OFF'} | "
            f"ARM={'ON' if self.arm_active else 'OFF'}"
        )


    # -------------------- Serial Helpers --------------------
    def refresh_ports(self):
        self.com_dropdown.clear()
        for p in serial.tools.list_ports.comports():
            self.com_dropdown.addItem(p.device)

    def connect_controller(self):
        global ser

        if ser and ser.is_open:
            ser.close()
            ser = None
            self.status_label.setText("Controller: Disconnected")
            self.status_label.setStyleSheet("color:red; font-weight:bold;")
            self.debug_terminal.append(">> Disconnected")
            self.connect_button.setText("Connect")
            return

        port = self.com_dropdown.currentText()
        try:
            ser = serial.Serial(port, BAUD_RATE, timeout=0.01)
            self.rx_buffer.clear()
            self.status_label.setText(f"Controller: Connected ({port})")
            self.status_label.setStyleSheet("color:green; font-weight:bold;")
            self.debug_terminal.append(f">> Connected on {port}")
            self.connect_button.setText("Disconnect")
        except Exception as e:
            self.debug_terminal.append(f">> Connection failed: {e}")

    # -------------------- SERIAL READ --------------------
    def read_serial(self):
        if not (ser and ser.is_open):
            return

        try:
            data = ser.read(ser.in_waiting)
            if not data:
                return

            self.rx_buffer.extend(data)

            while True:
                newline_idx = self.rx_buffer.find(b"\n")
                if newline_idx == -1:
                    return  # wait for full line

                line = self.rx_buffer[:newline_idx]
                self.rx_buffer = self.rx_buffer[newline_idx + 1:]

                try:
                    text = line.decode("utf-8", errors="replace").strip()
                    if not text:
                        continue

                    # DEBUG lines
                    if text.startswith("DEBUG:"):
                        self.debug_terminal.append(text)
                        continue

                    # TELEMETRY lines (ID=)
                    if text.startswith("ID="):
                        telemetry = {}
                        for p in text.split():
                            key, val = p.split("=")
                            if key == "ERR":
                                telemetry[key] = int(val, 16)
                            else:
                                telemetry[key] = int(val)

                        robot_id = telemetry["ID"]

                        if robot_id in self.robot_widgets:
                            msg = (
                                f"HB {telemetry['HB']:03d} | "
                                f"Status {telemetry['STATUS']} | "
                                f"Batt {telemetry['BATT']} mV | "
                                f"Temp {telemetry['TEMP']} C | "
                                f"Err 0x{telemetry['ERR']:02X} | "
                                f"RTT {telemetry['RTT']} ms"
                            )
                            self.robot_widgets[robot_id].append(msg)

                except Exception as e:
                    self.debug_terminal.append(f"Serial parse error: {e}")

        except Exception as e:
            self.debug_terminal.append(f">> Serial error: {e}")

    # -------------------- Telemetry Decode --------------------
    def handle_telemetry(self, payload: bytes):
        try:
            (
                pkt_type,
                robot_id,
                heartbeat,
                acked_type,
                status,
                battery_mv,
                motor_temp,
                error_flags,
                latency_ms
            ) = struct.unpack(ACK_FMT, payload)

            msg = (
                f"Robot {robot_id} | "
                f"HB {heartbeat:03d} | "
                f"Status {status} | "
                f"Batt {battery_mv} mV | "
                f"Temp {motor_temp} C | "
                f"Err 0x{error_flags:02X} | "
                f"RTT {latency_ms} ms"
            )

            self.telemetry_terminal.append(msg)

        except Exception as e:
            self.debug_terminal.append(f"Telemetry decode error: {e}")


# -------------------- MAIN --------------------
app = QtWidgets.QApplication([])
dashboard = SimpleDashboard()
dashboard.show()

timer = QtCore.QTimer()
timer.timeout.connect(dashboard.read_serial)
timer.start(20)  # fast enough for 50ms telemetry

sys.exit(app.exec())
