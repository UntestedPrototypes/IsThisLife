"""
Main dashboard window
"""
import tkinter as tk
from tkinter import ttk, messagebox
import json
import os
import time
from config import *
from robot_state import RobotStateManager
from serial_reader import SerialReader
from ui_config_tab import ConfigTab
from ui_live_tab import LiveViewTab
from ui_game_tab import GameControllerTab
import joystick_control
import packet_sender
import serial_comm

CONFIG_FILE = "config.json"
AUTO_RECONNECT_INTERVAL_MS = 1000  # Try to reconnect every 1 second

class Dashboard:
    """Main robot controller dashboard"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Controller Dashboard")
        self.root.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        
        # Load Global Config
        self.app_config = self._load_config()
        
        # State management
        self.robot_state = RobotStateManager()
        self.joy_manager = joystick_control.get_manager()
        
        # Serial reader
        self.serial_reader = SerialReader(
            on_telemetry=self._handle_telemetry,
            on_confirmation=self._handle_confirmation_request,
            on_debug=self._handle_debug
        )
        
        # Create UI
        self._create_tabs()
        
        # Start background processes
        self.serial_reader.start()
        
        # Start update loops
        self.root.after(CONTROL_UPDATE_RATE_MS, self._update_game_controller)
        self.root.after(CONTROL_UPDATE_RATE_MS, self._periodic_control_loop)
        
        # Start Auto-Reconnect Loop
        self.root.after(AUTO_RECONNECT_INTERVAL_MS, self._auto_reconnect_loop)
    
    def _load_config(self):
        """Load global config file"""
        default_config = {"com_port": "", "baud_rate": BAUD_RATE, "assignments": {}}
        if not os.path.exists(CONFIG_FILE):
            return default_config
        
        try:
            with open(CONFIG_FILE, 'r') as f:
                data = json.load(f)
                # Migration handling
                if "assignments" not in data and "com_port" not in data:
                    return {"com_port": "", "baud_rate": BAUD_RATE, "assignments": data}
                return data
        except Exception as e:
            print(f"Error loading config: {e}")
            return default_config

    def _save_global_config(self):
        """Gather settings from all tabs and save to JSON"""
        # Get Port & Baud
        port = self.config_tab.get_selected_port()
        baud = self.config_tab.get_selected_baud()
        
        # Get Assignments
        assignments = self.game_tab.get_assignment_guids()
        
        data = {
            "com_port": port,
            "baud_rate": baud,
            "assignments": assignments
        }
        
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(data, f, indent=4)
            messagebox.showinfo("Success", f"Configuration saved to {CONFIG_FILE}")
            # Update internal config so auto-reconnect knows the new port
            self.app_config = data
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save config: {e}")

    def _create_tabs(self):
        """Create tab interface"""
        self.tabs = ttk.Notebook(self.root)
        self.tabs.pack(expand=True, fill=tk.BOTH)
        
        # Config tab
        self.config_tab = ConfigTab(
            self.tabs,
            on_connect_callback=self._on_connected,
            initial_port=self.app_config.get("com_port"),
            initial_baud=self.app_config.get("baud_rate", BAUD_RATE)
        )
        self.tabs.add(self.config_tab.get_frame(), text="Configuration")
        
        # Live view tab
        self.live_tab = LiveViewTab(self.tabs, self.robot_state)
        self.tabs.add(self.live_tab.get_frame(), text="Live View")
        
        # Game controller tab
        self.game_tab = GameControllerTab(
            self.tabs,
            save_callback=self._save_global_config,
            initial_assignments=self.app_config.get("assignments")
        )
        self.tabs.add(self.game_tab.get_frame(), text="Game Controller")
    
    def _on_connected(self):
        pass
        
    def _auto_reconnect_loop(self):
        """Check connection status and reconnect if needed"""
        target_port = self.app_config.get("com_port")
        target_baud = self.app_config.get("baud_rate", BAUD_RATE)
        
        # If we have a configured port but are NOT connected
        if target_port and not serial_comm.is_connected():
            # Try to connect (silently, don't spam UI logs unless successful)
            try:
                # We access the low-level connect directly to avoid UI toggle logic
                # However, we must update the UI state if successful
                if serial_comm.connect(target_port, target_baud):
                    print(f"Auto-reconnected to {target_port}")
                    self.config_tab.set_connected_state(True)
                    self.config_tab.log_debug(f"Auto-reconnected to {target_port}")
            except Exception:
                pass # Fail silently and try again later
        
        self.root.after(AUTO_RECONNECT_INTERVAL_MS, self._auto_reconnect_loop)
    
    def _handle_telemetry(self, telemetry_data):
        self.root.after(0, lambda: self.live_tab.update_telemetry(telemetry_data))
        is_estopped = (telemetry_data.status == STATUS_ESTOP)
        self.robot_state.set_estop(telemetry_data.robot_id, is_estopped)
    
    def _handle_confirmation_request(self, conf_req):
        self.root.after(
            0,
            lambda: self._show_confirmation_dialog(
                conf_req.robot_id,
                conf_req.step_id,
                conf_req.message
            )
        )
    
    def _handle_debug(self, message):
        self.root.after(0, lambda: self.config_tab.log_debug(message))
    
    def _show_confirmation_dialog(self, robot_id, step_id, message):
        def on_approve():
            packet_sender.send_confirmation(robot_id, step_id, True)
            dialog.destroy()
        
        def on_deny():
            packet_sender.send_confirmation(robot_id, step_id, False)
            dialog.destroy()
        
        dialog = tk.Toplevel(self.root)
        dialog.title(f"Robot {robot_id} - Confirmation Required")
        dialog.geometry("400x150")
        
        tk.Label(dialog, text=f"Robot {robot_id} requesting confirmation:").pack(pady=10)
        tk.Label(dialog, text=message).pack(pady=5)
        
        btn_frame = tk.Frame(dialog)
        btn_frame.pack(pady=10)
        tk.Button(btn_frame, text="Approve", bg="green", command=on_approve).pack(side=tk.LEFT, padx=5)
        tk.Button(btn_frame, text="Deny", bg="red", command=on_deny).pack(side=tk.LEFT, padx=5)
    
    def _update_game_controller(self):
        self.game_tab.check_learning()
        self.game_tab.update_display_values()
        
        all_commands = self.game_tab.get_active_commands()
        controlled_robot_ids = []

        assigned_robots = self.game_tab.assignments.keys()
        
        for robot_id in assigned_robots:
            if robot_id not in all_commands:
                if not self.robot_state.is_estopped(robot_id):
                    print(f"SAFETY: Controller disconnected for Robot {robot_id} -> E-STOPPING")
                    packet_sender.send_estop(robot_id)
                    self.robot_state.set_estop(robot_id, True)
                    self.game_tab._update_assignment_list()

        for robot_id, controls in all_commands.items():
            controlled_robot_ids.append(robot_id)
            
            if controls["estop"]:
                if not self.robot_state.is_estopped(robot_id):
                    packet_sender.send_estop(robot_id)
                    self.robot_state.set_estop(robot_id, True)
            
            if self.robot_state.handle_arm_button(robot_id, controls["arm"]):
                packet_sender.send_arm(robot_id)
            
            if self.robot_state.should_send_control(robot_id):
                vx = int(controls["vx"] * 127)
                vy = int(controls["vy"] * 127)
                omega = int(controls["omega"] * 127)
                packet_sender.send_control(robot_id, vx, vy, omega)
            else:
                packet_sender.send_control(robot_id, 0, 0, 0)

        self.controlled_robot_ids = controlled_robot_ids
        self.root.after(CONTROL_UPDATE_RATE_MS, self._update_game_controller)
    
    def _periodic_control_loop(self):
        active_joystick_robots = getattr(self, 'controlled_robot_ids', [])
        
        for robot_id in self.robot_state.get_all_robot_ids():
            if robot_id in active_joystick_robots:
                continue
            packet_sender.send_control(robot_id, 0, 0, 0)
        
        self.root.after(CONTROL_UPDATE_RATE_MS, self._periodic_control_loop)
    
    def cleanup(self):
        self.serial_reader.stop()