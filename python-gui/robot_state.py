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
        self._prev_arm_input = False  # For edge detection
    
    def set_estop(self, active):
        """Set E-STOP state"""
        self.estop_active = active
        if active:
            self.armed = False  # E-STOP disables arm
    
    def toggle_arm(self):
        """
        Toggle armed state or clear E-STOP.
        Returns: True if an ARM packet (ESTOP_CLEAR) should be sent.
        """
        # Case 1: E-STOP is currently active
        # Action: Clear E-STOP flag, but go to DISARMED state first for safety.
        if self.estop_active:
            self.estop_active = False
            self.armed = False  # Require a second press to ARM
            return True         # Return True to trigger 'send_arm' (clears hardware E-STOP)
            
        # Case 2: Normal Operation
        # Action: Toggle between ARMED and DISARMED
        self.armed = not self.armed
        return self.armed # Return True (Send Packet) if Armed, False if Disarmed
    
    def set_arm(self, armed):
        """Set armed state directly"""
        if self.estop_active and armed:
            self.estop_active = False
            
        self.armed = armed
    
    def should_send_control(self):
        """Check if robot should receive control commands"""
        return self.armed and not self.estop_active


class RobotStateManager:
    """Manage state for multiple robots"""
    
    def __init__(self, num_robots=MAX_ROBOTS):
        self.robots = {}
        for i in range(1, num_robots + 1):
            self.robots[i] = RobotState(i)
    
    def get_robot(self, robot_id):
        """Get robot state by ID"""
        if robot_id not in self.robots:
            self.robots[robot_id] = RobotState(robot_id)
        return self.robots[robot_id]
    
    def get_all_robot_ids(self):
        """Get list of all robot IDs"""
        return list(self.robots.keys())

    def set_estop(self, robot_id, active):
        """Set E-STOP for a robot"""
        self.get_robot(robot_id).set_estop(active)
    
    def toggle_arm(self, robot_id):
        """Toggle armed state for a robot"""
        return self.get_robot(robot_id).toggle_arm()
    
    def set_arm(self, robot_id, armed):
        """Set armed state for a robot"""
        self.get_robot(robot_id).set_arm(armed)
    
    def is_armed(self, robot_id):
        """Check if robot is armed"""
        return self.get_robot(robot_id).armed
    
    def is_estopped(self, robot_id):
        """Check if robot is E-STOPped"""
        return self.get_robot(robot_id).estop_active
    
    def should_send_control(self, robot_id):
        """Check if control commands should be sent to robot"""
        return self.get_robot(robot_id).should_send_control()
    
    def handle_arm_button(self, robot_id, button_pressed):
        """
        Handle arm button with edge detection
        
        Args:
            robot_id: Robot ID
            button_pressed: Current button state
        
        Returns:
            True if an ARM/ESTOP_CLEAR packet should be sent
        """
        robot = self.get_robot(robot_id)
        
        # Edge detection - only trigger on press, not release
        if button_pressed and not robot._prev_arm_input:
            robot._prev_arm_input = button_pressed
            return robot.toggle_arm()
        
        robot._prev_arm_input = button_pressed
        return False