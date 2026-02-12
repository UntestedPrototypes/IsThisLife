"""
Manage state for multiple robots
"""
from config import MAX_ROBOTS

class RobotState:
    """State for a single robot"""
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.estop_active = False
        self.armed = False
        self._prev_arm_input = False
        self.autopilot_active = False
        self.autopilot_speed = 0.0
    
    def set_estop(self, active):
        self.estop_active = active
        if active:
            self.armed = False
            self.autopilot_active = False
            self.autopilot_speed = 0.0
    
    def set_autopilot(self, active, speed):
        if self.estop_active or not self.armed:
            self.autopilot_active = False
            self.autopilot_speed = 0.0
        else:
            self.autopilot_active = active
            self.autopilot_speed = speed
    
    def toggle_arm(self):
        if self.estop_active:
            self.estop_active = False
            self.armed = False
            self.autopilot_active = False
            self.autopilot_speed = 0.0
            return True
        self.armed = not self.armed
        if not self.armed:
            self.autopilot_active = False
            self.autopilot_speed = 0.0
        return self.armed
    
    def set_arm(self, armed):
        if self.estop_active and armed:
            self.estop_active = False
        self.armed = armed
        if not armed:
            self.autopilot_active = False
            self.autopilot_speed = 0.0
    
    def should_send_control(self):
        return self.armed and not self.estop_active

class RobotStateManager:
    """Manage state for multiple robots"""
    def __init__(self):
        self.robots = {}
    
    def exists(self, robot_id):
        """Check if robot exists without creating it"""
        return robot_id in self.robots

    def get_robot(self, robot_id):
        """Get robot state by ID, creating if missing (used by manual add)"""
        if robot_id not in self.robots:
            self.robots[robot_id] = RobotState(robot_id)
        return self.robots[robot_id]
    
    def get_all_robot_ids(self):
        return list(self.robots.keys())

    # Forwarding methods
    def set_estop(self, robot_id, active):
        if self.exists(robot_id): self.get_robot(robot_id).set_estop(active)
    def set_autopilot(self, robot_id, active, speed):
        if self.exists(robot_id): self.get_robot(robot_id).set_autopilot(active, speed)
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
            robot._prev_arm_input = button_pressed
            return robot.toggle_arm()
        robot._prev_arm_input = button_pressed
        return False