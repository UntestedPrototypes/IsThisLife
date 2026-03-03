"""
Main dashboard window - handles multi-packet type routing.
"""
import tkinter as tk
from tkinter import ttk, messagebox
import json
import os
import time

from config import *
from robot_state import RobotStateManager
from telemetry_parser import TelemetryParser, TelemetryData, ConfirmRequest
from ui_config_tab import ConfigTab
from ui_live_tab import LiveViewTab
from ui_game_tab import GameControllerTab
import joystick_control
import packet_sender
import serial_comm

AUTO_RECONNECT_INTERVAL_MS = 1000

class Dashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Controller Dashboard")
        self.root.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        
        self.app_config = self._load_config()
        self.robot_state = RobotStateManager()
        self.joy_manager = joystick_control.get_manager()
        self.parser = TelemetryParser()
        
        if "assignments" in self.app_config:
            for r_id_str in self.app_config["assignments"]:
                try:
                    r_id = int(r_id_str)
                    self.robot_state.get_robot(r_id)
                except ValueError: pass

        self._create_status_bar()
        self._create_tabs()
        
        self.root.after(CONTROL_UPDATE_RATE_MS, self._update_game_controller)
        self.root.after(CONTROL_UPDATE_RATE_MS, self._periodic_control_loop)
        self.root.after(AUTO_RECONNECT_INTERVAL_MS, self._auto_reconnect_loop)
        self.root.after(10, self._read_serial_loop) 
        self.root.after(200, self._update_status_bar)

    def _read_serial_loop(self):
        """Routes binary packets to their respective handlers"""
        if serial_comm.is_connected():
            try:
                raw_data = serial_comm.read_all() 
                if raw_data:
                    packets = self.parser.process_bytes(raw_data)
                    for pkt in packets:
                        if isinstance(pkt, TelemetryData):
                            self._handle_telemetry(pkt)
                        elif isinstance(pkt, ConfirmRequest):
                            self._handle_confirmation_request(pkt)
            except Exception as e:
                self.config_tab.log_debug(f"Serial Read Error: {e}")
        
        self.root.after(10, self._read_serial_loop)

    def _handle_telemetry(self, data):
        if not self.robot_state.exists(data.robot_id): return
        self.robot_state.mark_seen(data.robot_id)
        self.root.after(0, lambda: self.live_tab.update_telemetry(data))
        self.root.after(0, lambda: self.game_tab.update_available_robots(
            self.robot_state.get_all_robot_ids()
        ))
        
        # Update the state manager using the raw status code directly
        self.robot_state.set_status(data.robot_id, data.status)

    def _handle_confirmation_request(self, req):
        """Shows a popup dialog for robot confirmation steps"""
        if not self.robot_state.exists(req.robot_id): return
        
        def on_dialog_close(approved):
            packet_sender.send_confirmation(req.robot_id, req.step_id, approved)
            dialog.destroy()

        dialog = tk.Toplevel(self.root)
        dialog.title(f"Robot {req.robot_id} Request")
        tk.Label(dialog, text=f"Robot {req.robot_id} asks:\n{req.message}", padx=20, pady=20).pack()
        
        btn_frame = tk.Frame(dialog)
        btn_frame.pack(pady=10)
        tk.Button(btn_frame, text="Approve", command=lambda: on_dialog_close(True), width=10).pack(side=tk.LEFT, padx=5)
        tk.Button(btn_frame, text="Deny", command=lambda: on_dialog_close(False), width=10).pack(side=tk.LEFT, padx=5)

    def _load_config(self):
        default = {"com_port": "", "baud_rate": BAUD_RATE, "assignments": {}, "auto_reconnect": False}
        if os.path.exists("config.json"):
            try:
                with open("config.json", 'r') as f:
                    return json.load(f)
            except: pass
        return default
    
    def _save_config(self):
        """Saves current configurations to the config.json file"""
        self.app_config["com_port"] = self.config_tab.get_selected_port()
        self.app_config["baud_rate"] = self.config_tab.get_selected_baud()
        self.app_config["auto_reconnect"] = self.config_tab.is_auto_reconnect_enabled()
        self.app_config["assignments"] = self.game_tab.get_assignment_guids()
        try:
            with open("config.json", 'w') as f:
                json.dump(self.app_config, f, indent=4)
            messagebox.showinfo("Success", "Configuration saved successfully!")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save config: {e}")

    def _create_tabs(self):
        self.tabs = ttk.Notebook(self.root)
        self.tabs.pack(expand=True, fill=tk.BOTH)
        
        # Pass the auto_reconnect state to the config tab
        self.config_tab = ConfigTab(
            self.tabs, 
            None, 
            self.app_config.get("com_port"), 
            self.app_config.get("baud_rate", BAUD_RATE),
            self.app_config.get("auto_reconnect", False)
        )
        self.tabs.add(self.config_tab.get_frame(), text="Configuration")
        
        self.live_tab = LiveViewTab(self.tabs, self.robot_state)
        self.tabs.add(self.live_tab.get_frame(), text="Live View")
        
        # Pass the save callback to the game tab so the save button appears
        self.game_tab = GameControllerTab(self.tabs, self._save_config, self.app_config.get("assignments"))
        self.tabs.add(self.game_tab.get_frame(), text="Game Controller")

    def _create_status_bar(self):
        """Builds the UI elements for the bottom status bar"""
        self.status_frame = ttk.Frame(self.root, relief=tk.SUNKEN, padding=(2, 2))
        self.status_frame.pack(side=tk.BOTTOM, fill=tk.X)
        
        self.serial_status_lbl = ttk.Label(self.status_frame, text="Serial: Disconnected", font=("Arial", 9, "bold"))
        self.serial_status_lbl.pack(side=tk.LEFT, padx=10)
        
        ttk.Separator(self.status_frame, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=5)
        
        self.robot_status_container = ttk.Frame(self.status_frame)
        self.robot_status_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

    def _update_status_bar(self):
        """Periodically refreshes the text on the status bar"""
        # 1. Update Serial Status
        if serial_comm.is_connected():
            self.serial_status_lbl.config(text="Serial: CONNECTED", foreground="green")
        else:
            self.serial_status_lbl.config(text="Serial: DISCONNECTED", foreground="red")
        
        # 2. Update Robot + Controller Assignments and their States
        assignments = self.game_tab.get_assignment_names()
        
        for widget in self.robot_status_container.winfo_children():
            widget.destroy()
            
        if not assignments:
            ttk.Label(self.robot_status_container, text="No Robots Assigned").pack(side=tk.LEFT)
        else:
            for idx, (r_id, c_name) in enumerate(assignments.items()):
                if idx > 0:
                    ttk.Label(self.robot_status_container, text="  |  ").pack(side=tk.LEFT, padx=5)
                
                ttk.Label(self.robot_status_container, text=f"R{r_id}: ").pack(side=tk.LEFT)
                robot = self.robot_state.get_robot(r_id)
                
                # Dynamic colored label based on exact telemetry state
                if not robot.is_connected(timeout_sec=0.5):
                    r_state, r_color = "OFFLINE", "gray"
                elif robot.estop_active:
                    r_state, r_color = "E-STOP", "red"
                elif robot.status_code == STATUS_CALIBRATION_REQUIRED:
                    r_state, r_color = "NEEDS CALIB", "purple"
                elif robot.status_code == STATUS_WAITING_CONFIRM:
                    r_state, r_color = "CONFIRM?", "#d35400" # Orange
                elif robot.status_code == STATUS_RUNNING_SEQUENCE:
                    r_state, r_color = "SEQUENCE", "blue"
                elif robot.status_code == STATUS_NORMAL:
                    if robot.armed:
                        r_state, r_color = "ARMED", "green"
                    else:
                        r_state, r_color = "DISARMED", "#f39c12" # Orange
                else:
                    r_state, r_color = "UNKNOWN", "gray"
                
                ttk.Label(self.robot_status_container, text=f"[{r_state}]", foreground=r_color, font=("Arial", 9, "bold")).pack(side=tk.LEFT)
                
                if c_name == "Disconnected":
                    c_text, c_color = "[No Gamepad]", "red"
                else:
                    c_text, c_color = f" [{c_name}]", "blue"
                    
                ttk.Label(self.robot_status_container, text=c_text, foreground=c_color).pack(side=tk.LEFT)
            
        self.root.after(200, self._update_status_bar)

    def _auto_reconnect_loop(self):
        if self.config_tab.is_auto_reconnect_enabled():
            port = self.config_tab.get_selected_port() or self.app_config.get("com_port")
            baud = self.config_tab.get_selected_baud()
            if port and not serial_comm.is_connected():
                if serial_comm.connect(port, baud):
                    self.config_tab.set_connected_state(True)
                    self.config_tab.log_debug(f"Auto-reconnected to {port}")
        self.root.after(AUTO_RECONNECT_INTERVAL_MS, self._auto_reconnect_loop)

    def _update_game_controller(self):
        self.game_tab.check_learning()
        self.game_tab.update_display_values()
        all_commands = self.game_tab.get_active_commands()
        self.controlled_robot_ids = list(all_commands.keys())
        
        for r_id, controls in all_commands.items():
            robot = self.robot_state.get_robot(r_id)
            
            c_id = self.game_tab.assignments.get(r_id, {}).get('id')
            ctrl = self.joy_manager.get_controller(c_id) if c_id is not None else None
            
            # --- Consume Intercepted Events from RobotState ---
            evt = robot.consume_rumble()
            if evt and ctrl:
                if evt == "ESTOP_CLEARED":
                    ctrl.rumble(0.2, 0.2, 300) 
                elif evt == "ESTOP_REJECTED":
                    ctrl.rumble(1.0, 1.0, 600) 
                elif evt == "DISARM":
                    ctrl.rumble(0.8, 0.2, 400) 
            
            # --- E-Stop Handler ---
            if controls["estop"] and not robot.estop_active:
                packet_sender.send_estop(r_id)
                if ctrl: ctrl.rumble(0.8, 0.2, 400) 
                
            # --- Multi-Stage Arm/E-Stop Clear Logic ---
            if self.robot_state.handle_arm_button(r_id, controls["arm"]):
                if robot.estop_active:
                    packet_sender.send_arm(r_id)
                    robot.request_estop_clear() 
                else:
                    if robot.armed:
                        robot.armed = False
                        if ctrl: ctrl.rumble(0.8, 0.2, 400) 
                    elif robot.status_code == STATUS_NORMAL:
                        robot.armed = True
                        if ctrl: ctrl.rumble(0.1, 0.8, 300) 

            # --- CANCEL CRUISE CONTROL ON DISARM/ESTOP ---
            if not robot.armed or robot.estop_active or controls["estop"]:
                if ctrl:
                    ctrl.cancel_cruise()
                # Override the dictionary for this loop so UI and robot update instantly
                controls["cruise_state"] = "DISABLED"
                controls["cruise_val"] = 0.0
                        
            # --- UPDATE CRUISE CONTROL STATE ---
            robot.set_cruise_control(controls.get("cruise_state") == "ENABLED", controls.get("cruise_val", 0.0))
            
            # --- Transmission Handler ---
            if robot.should_send_control():
                packet_sender.send_control(r_id, controls["vx"], controls["vy"], controls["omega"])
            else:
                packet_sender.send_control(r_id, 0.0, 0.0, 0.0)
                
        self.root.after(CONTROL_UPDATE_RATE_MS, self._update_game_controller)

    def _periodic_control_loop(self):
        active = getattr(self, 'controlled_robot_ids', [])
        # Deadlock Fix: Always broadcast to a standard block of robots to force discovery
        for r_id in range(1, 4): 
            if r_id not in active:
                packet_sender.send_control(r_id, 0.0, 0.0, 0.0)
        self.root.after(CONTROL_UPDATE_RATE_MS, self._periodic_control_loop)

    def cleanup(self):
        serial_comm.disconnect()