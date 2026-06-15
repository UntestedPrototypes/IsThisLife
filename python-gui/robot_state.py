"""
Manage state for multiple robots
"""
import time
from config import MAX_ROBOTS

class RobotState:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.status_code = 0
        self.estop_active = False
        self.armed = False
        self._prev_arm_input = False
        self.cruise_active = False
        self.cruise_speed = 0.0
        
        self.control_mode = 0
        self.telemetry_mode = 0
        
        self.last_telemetry_time = time.time()

        self.pending_estop_clear = False
        self.pending_estop_time = 0.0
        self.rumble_event = None

        # --- NEW: Holds the latest fetched variable readings ---
        self.fetched_settings = {}
    
    def mark_seen(self):
        self.last_telemetry_time = time.time()
        
    def is_connected(self, timeout_sec=2.0):
        return (time.time() - self.last_telemetry_time) < timeout_sec

    def toggle_control_mode(self):
        self.control_mode = 1 if self.control_mode == 0 else 0
        return self.control_mode

    def set_status(self, raw_status):
        from config import STATUS_FLAG_ESTOP, STATUS_STATE_MASK, STATUS_NORMAL
        
        new_estop = bool(raw_status & STATUS_FLAG_ESTOP)
        self.status_code = raw_status & STATUS_STATE_MASK
        
        if self.pending_estop_clear:
            if not new_estop:  
                self.rumble_event = "ESTOP_CLEARED"
                self.pending_estop_clear = False
            elif time.time() - self.pending_estop_time > 0.5: 
                self.rumble_event = "ESTOP_REJECTED"
                self.pending_estop_clear = False
                
        if new_estop and self.armed:
            self.rumble_event = "DISARM"

        self.estop_active = new_estop
        
        if self.estop_active:
            self.armed = False
            self.cruise_active = False
            self.cruise_speed = 0.0
            self.control_mode = 0

    def request_estop_clear(self):
        self.pending_estop_clear = True
        self.pending_estop_time = time.time()
        
    def consume_rumble(self):
        evt = self.rumble_event
        self.rumble_event = None
        return evt

    def set_estop(self, active):
        self.estop_active = active
        if active:
            self.armed = False
            self.cruise_active = False
            self.cruise_speed = 0.0
            self.control_mode = 0
    
    def set_cruise_control(self, active, speed):
        if self.estop_active or not self.armed:
            self.cruise_active = False
            self.cruise_speed = 0.0
        else:
            self.cruise_active = active
            self.cruise_speed = speed
    
    def toggle_arm(self):
        if self.estop_active:
            self.estop_active = False
            self.armed = False
            self.cruise_active = False
            self.cruise_speed = 0.0
            return True
        self.armed = not self.armed
        if not self.armed:
            self.cruise_active = False
            self.cruise_speed = 0.0
        return self.armed
    
    def set_arm(self, armed):
        if self.estop_active and armed:
            self.estop_active = False
        self.armed = armed
        if not armed:
            self.cruise_active = False
            self.cruise_speed = 0.0
    
    def should_send_control(self):
        return self.armed and not self.estop_active

    # --- NEW METHOD to store retrieved settings
    def update_setting(self, key, value):
        self.fetched_settings[key] = value


class RobotStateManager:
    def __init__(self):
        self.robots = {}
    
    def exists(self, robot_id):
        return robot_id in self.robots

    def get_robot(self, robot_id):
        if robot_id not in self.robots:
            self.robots[robot_id] = RobotState(robot_id)
        return self.robots[robot_id]
    
    def get_all_robot_ids(self):
        return list(self.robots.keys())

    def mark_seen(self, robot_id):
        if self.exists(robot_id): self.get_robot(robot_id).mark_seen()
        
    def set_status(self, robot_id, status):
        if self.exists(robot_id): self.get_robot(robot_id).set_status(status)
        
    def set_estop(self, robot_id, active):
        if self.exists(robot_id): self.get_robot(robot_id).set_estop(active)
        
    def set_cruise_control(self, robot_id, active, speed):
        if self.exists(robot_id): self.get_robot(robot_id).set_cruise_control(active, speed)
        
    def toggle_arm(self, robot_id):
        return self.get_robot(robot_id).toggle_arm() if self.exists(robot_id) else False
        
    def set_arm(self, robot_id, armed):
        if self.exists(robot_id): self.get_robot(robot_id).set_arm(armed)
        
    def is_armed(self, robot_id):
        return self.get_robot(robot_id).armed if self.exists(robot_id) else False
        
    def is_estopped(self, robot_id):
        return self.get_robot(robot_id).estop_active if self.exists(robot_id) else False
        
    def should_send_control(self, robot_id):
        return self.get_robot(robot_id).should_send_control() if self.exists(robot_id) else False
        
    def handle_arm_button(self, robot_id, button_pressed):
        if not self.exists(robot_id): return False
        robot = self.get_robot(robot_id)
        if button_pressed and not robot._prev_arm_input:
            robot._prev_arm_input = True
            return True
        robot._prev_arm_input = button_pressed
        return False
        
    # --- NEW: Forward Setting to Robot State ---
    def update_fetched_setting(self, robot_id, key, value):
        if self.exists(robot_id):
            self.get_robot(robot_id).update_setting(key, value)