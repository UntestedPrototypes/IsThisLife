"""
Live view of robot status and sequence control
"""
import tkinter as tk
from tkinter import ttk, messagebox
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
        
        # Local update loop
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
        """Manually register a robot to start sending it packets"""
        try:
            r_id = int(self.ent_manual_id.get())
            if r_id < 1: 
                messagebox.showerror("Error", "ID must be positive")
                return
                
            # 1. Register in Backend State (Crucial Step: This "Allows" the ID)
            # dashboard.py will now accept packets from this ID
            self.robot_state.get_robot(r_id)
            
            # 2. Create UI Panel immediately
            if r_id not in self.robot_widgets:
                self._create_robot_panel(r_id)
                print(f"Manually added Robot {r_id}. Packets should start streaming.")
            
            self.ent_manual_id.delete(0, tk.END)
            
        except ValueError:
            messagebox.showerror("Error", "Invalid ID")

    def _create_robot_panel(self, robot_id):
        """Create a combined status and control frame for a specific robot"""
        lf = ttk.LabelFrame(self.scrollable_frame, text=f"Robot {robot_id}")
        lf.pack(fill=tk.X, padx=10, pady=5, expand=True)
        
        # --- Top: Status Banner ---
        status_banner = tk.Label(
            lf, 
            text="WAKING UP...", 
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
        
        def add_stat(key, label_text, row, col):
            ttk.Label(stats_frame, text=label_text, font=("", 9, "bold")).grid(row=row, column=col*2, sticky="e", padx=(10, 2))
            lbl = ttk.Label(stats_frame, text="--")
            lbl.grid(row=row, column=col*2 + 1, sticky="w", padx=(0, 20))
            widgets[key] = lbl

        add_stat("hb",   "Heartbeat:", 0, 0)
        add_stat("batt", "Battery:", 0, 1)
        add_stat("rtt",  "RTT:",     0, 2)
        add_stat("temp", "Temp:",    1, 0)
        add_stat("err",  "Error:",   1, 1)
        
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
                control_frame,
                text=name,
                command=lambda r=robot_id, s=seq_id: self._start_sequence(r, s)
            )
            row = i // 3
            col = i % 3
            btn.grid(row=row, column=col, padx=3, pady=2, sticky="ew")
            control_frame.columnconfigure(col, weight=1)
    
    def update_telemetry(self, data):
        """Update telemetry labels. Creates panel if new robot detected."""
        robot_id = int(data.robot_id)
        
        # DYNAMIC CREATION (Only if already allowed by backend/manual add)
        if robot_id not in self.robot_widgets:
            # If dashboard passed it here, it means it's valid. Create UI.
            self._create_robot_panel(robot_id)

        widgets = self.robot_widgets[robot_id]
        banner = widgets["status_banner"]
        
        # Banner Logic
        bg_color = "#cccccc"
        fg_color = "black"
        text = f"CONNECTED ({data.status})"
        
        if data.status == STATUS_ESTOP:
            bg_color = "#ff0000"; fg_color = "white"; text = "!!! E-STOP ACTIVE !!!"
        elif data.status == STATUS_WAITING_CONFIRM:
            bg_color = "#ffa500"; fg_color = "black"; text = "WAITING FOR CONFIRMATION"
        elif data.status == STATUS_RUNNING_SEQUENCE:
            bg_color = "#3498db"; fg_color = "white"; text = "RUNNING SEQUENCE"
        elif data.status == STATUS_OK:
            robot = self.robot_state.get_robot(robot_id)
            if robot.autopilot_active:
                bg_color = "#007bff"; fg_color = "white"
                text = f"AUTO-PILOT ({int(robot.autopilot_speed * 100)}%)"
            elif self.robot_state.is_armed(robot_id):
                bg_color = "#2ecc71"; fg_color = "white"; text = "ARMED & READY"
            else:
                bg_color = "#f1c40f"; fg_color = "black"; text = "DISARMED (Safe)"
        
        banner.configure(text=text, bg=bg_color, fg=fg_color)

        # Update Stats
        hb = getattr(data, 'heartbeat', '--')
        batt_v = getattr(data, 'battery_mv', 0) / 1000.0
        temp = getattr(data, 'temperature_c', 0)
        err = getattr(data, 'error_flags', 0)
        rtt = getattr(data, 'rtt_ms', 0)

        widgets["batt"].configure(text=f"{batt_v:.1f} V")
        widgets["temp"].configure(text=f"{temp}°C")
        widgets["hb"].configure(text=str(hb))
        widgets["rtt"].configure(text=f"{rtt} ms")
        widgets["err"].configure(text=f"0x{err:02X}", foreground="red" if err != 0 else "black")

    def _update_ui_loop(self):
        """Periodic UI update for local state"""
        for r_id, widgets in self.robot_widgets.items():
            robot = self.robot_state.get_robot(r_id)
            banner = widgets["status_banner"]
            
            # --- 1. Check Connection Timeout ---
            if not robot.is_connected(timeout_sec=0.5):
                banner.configure(text="CONNECTION LOST", bg="#7f8c8d", fg="white")
                continue # Skip the rest of the logic if disconnected
            
            # --- 2. Live State Updates (Only runs if connected) ---
            # Update banner color immediately on state change (instant feedback)
            if not robot.estop_active:
                if robot.autopilot_active:
                    banner.configure(text=f"AUTO-PILOT ({int(robot.autopilot_speed * 100)}%)", bg="#007bff", fg="white")
                elif robot.armed:
                    banner.configure(text="ARMED & READY", bg="#2ecc71", fg="white")
                else:
                    # Keep existing text (likely "CONNECTED" or "DISARMED"), change color to yellow/grey
                    # We avoid overwriting "RUNNING SEQUENCE" here blindly
                    pass

        self.frame.after(100, self._update_ui_loop)

    def _start_sequence(self, robot_id, seq_id):
        packet_sender.send_start_sequence(robot_id, seq_id)
        print(f"Sent sequence {seq_id} to Robot {robot_id}")