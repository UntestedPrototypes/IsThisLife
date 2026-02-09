"""
Live view of robot status and sequence control
"""
import tkinter as tk
from tkinter import ttk
from config import *
import packet_sender

class LiveViewTab:
    """Tab for monitoring live robot telemetry"""
    
    def __init__(self, notebook, robot_state_manager):
        self.frame = ttk.Frame(notebook)
        self.robot_state = robot_state_manager
        
        # Dictionary to store UI widgets for each robot
        self.robot_widgets = {}
        
        self._setup_ui()
    
    def get_frame(self):
        return self.frame
    
    def _setup_ui(self):
        # Scrollable container for robot panels (optional, but good practice)
        canvas = tk.Canvas(self.frame)
        scrollbar = ttk.Scrollbar(self.frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Pack scroll components
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Create a dedicated panel for each robot
        for i in range(1, MAX_ROBOTS + 1):
            self._create_robot_panel(scrollable_frame, i)

    def _create_robot_panel(self, parent, robot_id):
        """Create a combined status and control frame for a specific robot"""
        lf = ttk.LabelFrame(parent, text=f"Robot {robot_id}")
        lf.pack(fill=tk.X, padx=10, pady=5, expand=True)
        
        # --- Top: Big Status Banner (E-STOP / ARMED) ---
        # We use a standard tk.Label here because changing background color 
        # is much easier/reliable than with ttk.Label styles
        status_banner = tk.Label(
            lf, 
            text="DISCONNECTED", 
            font=("Arial", 12, "bold"), 
            bg="#cccccc", 
            fg="black",
            pady=5
        )
        status_banner.pack(fill=tk.X, padx=5, pady=(5, 10))
        
        # --- Middle: Telemetry Grid ---
        stats_frame = ttk.Frame(lf)
        stats_frame.pack(fill=tk.X, padx=5, pady=2)
        
        widgets = {"status_banner": status_banner}
        
        # Helper to create label pairs
        def add_stat(key, label_text, row, col):
            ttk.Label(stats_frame, text=label_text, font=("", 9, "bold")).grid(row=row, column=col*2, sticky="e", padx=(10, 2))
            lbl = ttk.Label(stats_frame, text="--")
            lbl.grid(row=row, column=col*2 + 1, sticky="w", padx=(0, 20))
            widgets[key] = lbl

        # Row 0: Vital Connection Stats
        add_stat("hb",   "Heartbeat:", 0, 0)
        add_stat("batt", "Battery:", 0, 1)
        add_stat("rtt",  "RTT:",     0, 2)
        
        # Row 1: Hardware Stats
        add_stat("temp", "Temp:",    1, 0)
        add_stat("err",  "Error:",   1, 1)
        # Placeholder for future use
        
        self.robot_widgets[robot_id] = widgets
        
        # --- Bottom: Sequence Controls ---
        ttk.Separator(lf, orient='horizontal').pack(fill='x', padx=5, pady=10)
        
        control_frame = ttk.Frame(lf)
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        sequences = [
            ("Full Calib", SEQUENCE_CALIBRATION_FULL),
            ("Gyro Calib", SEQUENCE_CALIBRATION_GYRO),
            ("Motor Test", SEQUENCE_CALIBRATION_MOTORS),
            ("Dance", SEQUENCE_DEMO_DANCE),
            ("Sensors", SEQUENCE_SENSOR_TEST),
            ("Path Follow", SEQUENCE_PATH_FOLLOW),
        ]
        
        # Grid buttons
        for i, (name, seq_id) in enumerate(sequences):
            btn = ttk.Button(
                control_frame,
                text=name,
                command=lambda r=robot_id, s=seq_id: self._start_sequence(r, s)
            )
            # Arrange in 2 rows of 3
            row = i // 3
            col = i % 3
            btn.grid(row=row, column=col, padx=3, pady=2, sticky="ew")
            control_frame.columnconfigure(col, weight=1)
    
    def update_telemetry(self, data):
        """Update telemetry labels for a specific robot"""
        robot_id = int(data.robot_id)
        
        if robot_id not in self.robot_widgets:
            return

        widgets = self.robot_widgets[robot_id]
        banner = widgets["status_banner"]
        
        # --- 1. Status Banner Logic ---
        # Default State
        bg_color = "#cccccc" # Grey
        fg_color = "black"
        text = f"DISCONNECTED ({data.status})"
        
        if data.status == STATUS_ESTOP:
            bg_color = "#ff0000" # Red
            fg_color = "white"
            text = "!!! E-STOP ACTIVE !!!"
        elif data.status == STATUS_WAITING_CONFIRM:
            bg_color = "#ffa500" # Orange
            fg_color = "black"
            text = "WAITING FOR CONFIRMATION"
        elif data.status == STATUS_RUNNING_SEQUENCE:
            bg_color = "#3498db" # Blue
            fg_color = "white"
            text = "RUNNING SEQUENCE"
        elif data.status == STATUS_OK:
            if self.robot_state.is_armed(robot_id):
                bg_color = "#2ecc71" # Green
                fg_color = "white"
                text = "ARMED & READY"
            else:
                bg_color = "#f1c40f" # Yellow
                fg_color = "black"
                text = "DISARMED (Safe)"
        
        banner.configure(text=text, bg=bg_color, fg=fg_color)

        # --- 2. Update Stats ---
        hb = getattr(data, 'heartbeat', getattr(data, 'hb', getattr(data, 'HB', '--')))
        batt_v = getattr(data, 'battery_mv', 0) / 1000.0
        temp = getattr(data, 'temperature_c', getattr(data, 'temp_c', 0))
        err = getattr(data, 'error_flags', getattr(data, 'error_code', 0))
        rtt = getattr(data, 'rtt_ms', 0)

        widgets["batt"].configure(text=f"{batt_v:.1f} V")
        widgets["temp"].configure(text=f"{temp}°C")
        widgets["hb"].configure(text=str(hb))
        widgets["rtt"].configure(text=f"{rtt} ms")
        
        # Highlight Error if non-zero
        err_text = f"0x{err:02X}"
        widgets["err"].configure(
            text=err_text, 
            foreground="red" if err != 0 else "black"
        )

    def _start_sequence(self, robot_id, seq_id):
        """Start sequence on a specific robot"""
        packet_sender.send_start_sequence(robot_id, seq_id)
        print(f"Sent sequence {seq_id} to Robot {robot_id}")