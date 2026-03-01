"""
Live view of robot status, sequence control, and dual-IMU telemetry
"""
import tkinter as tk
from tkinter import ttk, messagebox
from config import *
import packet_sender

class LiveViewTab:
    """Tab for monitoring live robot telemetry including Main and Pendulum IMUs"""
    
    def __init__(self, notebook, robot_state_manager):
        self.frame = ttk.Frame(notebook)
        self.robot_state = robot_state_manager
        
        # Dictionary to store UI widgets for each robot
        self.robot_widgets = {}
        
        self._setup_ui()
        
        # Local update loop (Checks for timeouts)
        self.frame.after(100, self._update_ui_loop)
    
    def get_frame(self):
        return self.frame
    
    def _setup_ui(self):
        # --- 1. Top Bar: Manual Add Robot ---
        top_frame = ttk.Frame(self.frame)
        top_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(top_frame, text="Add Robot ID:").pack(side=tk.LEFT, padx=5)
        self.ent_manual_id = ttk.Entry(top_frame, width=5)
        self.ent_manual_id.pack(side=tk.LEFT, padx=5)
        ttk.Button(top_frame, text="Add & Wake Up", command=self._add_manual_robot).pack(side=tk.LEFT, padx=5)

        # --- 2. Scrollable Area ---
        canvas = tk.Canvas(self.frame)
        scrollbar = ttk.Scrollbar(self.frame, orient="vertical", command=canvas.yview)
        self.scrollable_frame = ttk.Frame(canvas)

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
    def _add_manual_robot(self):
        try:
            r_id = int(self.ent_manual_id.get())
            if r_id < 1: 
                messagebox.showerror("Error", "ID must be positive")
                return
            self.robot_state.get_robot(r_id)
            if r_id not in self.robot_widgets:
                self._create_robot_panel(r_id)
            self.ent_manual_id.delete(0, tk.END)
        except ValueError:
            messagebox.showerror("Error", "Invalid ID")

    def _create_robot_panel(self, robot_id):
        """Create a combined status, IMU telemetry, and control frame"""
        lf = ttk.LabelFrame(self.scrollable_frame, text=f" Robot {robot_id} ")
        lf.pack(fill=tk.X, padx=10, pady=5, expand=True)
        
        # --- Top: Status Banner ---
        status_banner = tk.Label(
            lf, text="WAKING UP...", font=("Arial", 11, "bold"), 
            bg="#cccccc", fg="black", pady=5
        )
        status_banner.pack(fill=tk.X, padx=5, pady=(5, 5))
        
        # --- Middle: Telemetry Grid ---
        stats_frame = ttk.Frame(lf)
        stats_frame.pack(fill=tk.X, padx=5, pady=2)
        
        widgets = {"status_banner": status_banner, "panel": lf}
        
        def add_stat(key, label_text, row, col, width=8):
            ttk.Label(stats_frame, text=label_text, font=("", 9, "bold")).grid(row=row, column=col*2, sticky="e", padx=(10, 2))
            lbl = ttk.Label(stats_frame, text="--", width=width)
            lbl.grid(row=row, column=col*2 + 1, sticky="w", padx=(0, 10))
            widgets[key] = lbl

        # Standard Vitals
        add_stat("hb",   "Heartbeat:", 0, 0)
        add_stat("batt", "Battery:",   0, 1)
        add_stat("rtt",  "RTT:",       0, 2)
        add_stat("temp", "Temp:",      1, 0)
        add_stat("err",  "Error:",     1, 1)

        # --- IMU Data Section ---
        ttk.Separator(lf, orient='horizontal').pack(fill='x', padx=20, pady=5)
        imu_frame = ttk.Frame(lf)
        imu_frame.pack(fill=tk.X, padx=5, pady=5)

        # Labels for Body and Pendulum IMUs
        ttk.Label(imu_frame, text="Main Body:", font=("", 9, "bold")).grid(row=0, column=0, sticky="e", padx=5)
        widgets["imu_main"] = ttk.Label(imu_frame, text="R: 0.0°  P: 0.0°", font=("Consolas", 10))
        widgets["imu_main"].grid(row=0, column=1, sticky="w", padx=5)

        ttk.Label(imu_frame, text="Pendulum:", font=("", 9, "bold")).grid(row=1, column=0, sticky="e", padx=5)
        widgets["imu_pend"] = ttk.Label(imu_frame, text="R: 0.0°  P: 0.0°", font=("Consolas", 10))
        widgets["imu_pend"].grid(row=1, column=1, sticky="w", padx=5)
        
        self.robot_widgets[robot_id] = widgets
        
        # --- Bottom: Sequence Controls ---
        ttk.Separator(lf, orient='horizontal').pack(fill='x', padx=5, pady=10)
        control_frame = ttk.Frame(lf)
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        sequences = [
            ("Calibrate Full", SEQUENCE_CALIBRATION_FULL),
            ("Calib Gyro", SEQUENCE_CALIBRATION_GYRO),
            ("Calib Motors", SEQUENCE_CALIBRATION_MOTORS),
            ("Demo Dance", SEQUENCE_DEMO_DANCE),
            ("Sensor Test", SEQUENCE_SENSOR_TEST),
            ("Path Follow", SEQUENCE_PATH_FOLLOW),
        ]
        
        for i, (name, seq_id) in enumerate(sequences):
            btn = ttk.Button(
                control_frame, text=name,
                command=lambda r=robot_id, s=seq_id: self._start_sequence(r, s)
            )
            btn.grid(row=i // 3, column=i % 3, padx=3, pady=2, sticky="ew")
            control_frame.columnconfigure(i % 3, weight=1)
    
    def update_telemetry(self, data):
        """Populates the UI with new binary data including IMU floats"""
        robot_id = int(data.robot_id)
        
        if robot_id not in self.robot_widgets:
            self._create_robot_panel(robot_id)

        widgets = self.robot_widgets[robot_id]
        banner = widgets["status_banner"]
        
        # 1. Update Banner Based on Status
        bg_color, fg_color, text = "#cccccc", "black", f"CONNECTED ({data.status})"
        
        if data.status == STATUS_ESTOP:
            bg_color, fg_color, text = "#ff0000", "white", "!!! E-STOP ACTIVE !!!"
        elif data.status == STATUS_WAITING_CONFIRM:
            bg_color, fg_color, text = "#ffa500", "black", "WAITING FOR CONFIRMATION"
        elif data.status == STATUS_RUNNING_SEQUENCE:
            bg_color, fg_color, text = "#3498db", "white", "RUNNING SEQUENCE"
        elif data.status == STATUS_OK:
            robot = self.robot_state.get_robot(robot_id)
            if robot.autopilot_active:
                bg_color, fg_color = "#007bff", "white"
                text = f"AUTO-PILOT ({int(robot.autopilot_speed * 100)}%)"
            elif self.robot_state.is_armed(robot_id):
                bg_color, fg_color, text = "#2ecc71", "white", "ARMED & READY"
            else:
                bg_color, fg_color, text = "#f1c40f", "black", "DISARMED (Safe)"
        
        banner.configure(text=text, bg=bg_color, fg=fg_color)

        # 2. Update Standard Stats
        widgets["hb"].configure(text=str(data.heartbeat))
        widgets["batt"].configure(text=f"{data.battery_mv/1000.0:.2f} V")
        widgets["temp"].configure(text=f"{data.motor_temp}°C")
        widgets["rtt"].configure(text=f"{data.latency_ms} ms")
        widgets["err"].configure(text=f"0x{data.error_flags:02X}", 
                                 foreground="red" if data.error_flags != 0 else "black")

        # 3. Update IMU Labels (The 4 new floats)
        widgets["imu_main"].configure(text=f"R: {data.main_roll:>5.1f}°  P: {data.main_pitch:>5.1f}°")
        widgets["imu_pend"].configure(text=f"R: {data.pend_roll:>5.1f}°  P: {data.pend_pitch:>5.1f}°")

    def _update_ui_loop(self):
        """Periodic check for connection timeouts"""
        for r_id, widgets in self.robot_widgets.items():
            robot = self.robot_state.get_robot(r_id)
            banner = widgets["status_banner"]
            
            # Check Connection Timeout (0.5s)
            if not robot.is_connected(timeout_sec=0.5):
                banner.configure(text="CONNECTION LOST", bg="#7f8c8d", fg="white")
                # Grey out the IMU data to show it's stale
                widgets["imu_main"].configure(foreground="#999999")
                widgets["imu_pend"].configure(foreground="#999999")
            else:
                widgets["imu_main"].configure(foreground="black")
                widgets["imu_pend"].configure(foreground="black")

        self.frame.after(100, self._update_ui_loop)

    def _start_sequence(self, robot_id, seq_id):
        packet_sender.send_start_sequence(robot_id, seq_id)
        print(f"Sent sequence {seq_id} to Robot {robot_id}")