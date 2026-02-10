"""
Game controller / joystick input handling with multi-controller support
"""
import pygame
import time
from config import *

# Initialize pygame once globally
pygame.init()
pygame.joystick.init()

# Default controller mappings
DEFAULT_MAPPINGS = {
    "vx": ("axis", 1),
    "vy": ("axis", 0),
    "omega": ("axis", 2),
    "estop": ("button", 0),
    "arm": ("button", 1),
    "autopilot": ("button", 3)
}

AUTOPILOT_HOLD_TIME = 2.0  # Seconds to hold button to ENABLE
CRUISE_ADJUST_RATE = 0.02  # Speed change per tick when pushing stick

class JoystickController:
    """Represents a single connected joystick with its own mappings"""
    
    def __init__(self, pygame_joy_instance):
        self.joy = pygame_joy_instance
        try:
            self.joy.init()
        except:
            pass
        self.id = self.joy.get_instance_id()
        self.name = self.joy.get_name()
        self.guid = self.joy.get_guid()
        self.mappings = DEFAULT_MAPPINGS.copy()
        
        # Learning state
        self.learning_mode = None
        self.learning_timeout = 0
        self.learning_baseline = []
        
        # Auto-Pilot State
        self.autopilot_active = False
        self.cruise_speed = 0.0
        self.autopilot_btn_start = 0
        self.autopilot_btn_prev = False

    def rumble(self, low_freq, high_freq, duration_ms):
        """
        Trigger haptic feedback on the controller
        Args:
            low_freq: Intensity of low frequency motor (0.0 to 1.0)
            high_freq: Intensity of high frequency motor (0.0 to 1.0)
            duration_ms: Duration in milliseconds
        """
        try:
            self.joy.rumble(low_freq, high_freq, int(duration_ms))
        except Exception:
            pass # Gracefully fail if controller doesn't support rumble

    def get_input_value(self, mapping):
        input_type, index = mapping
        try:
            if input_type == "axis":
                if index < self.joy.get_numaxes():
                    return self.joy.get_axis(index)
            elif input_type == "button":
                if index < self.joy.get_numbuttons():
                    return self.joy.get_button(index)
        except pygame.error:
            pass
        return 0

    def get_control_values(self):
        """Get processed control dictionary with Auto-Pilot logic"""
        
        raw_vx = self.get_input_value(self.mappings["vx"])
        raw_vy = -self.get_input_value(self.mappings["vy"]) # Inverted Y
        raw_omega = self.get_input_value(self.mappings["omega"])
        
        # --- Auto-Pilot Button Logic ---
        btn_val = self.get_input_value(self.mappings["autopilot"])
        is_pressed = (btn_val > 0.5)
        
        if is_pressed and not self.autopilot_btn_prev:
            # Just pressed
            if self.autopilot_active:
                # INSTANT DISABLE
                self.autopilot_active = False
                self.cruise_speed = 0.0
                print("Auto-Pilot Disabled (Instant)")
                self.rumble(0.6, 0.0, 300) # Low thud for OFF
            else:
                # Start timer for enable
                self.autopilot_btn_start = time.time()
        
        if is_pressed and self.autopilot_btn_prev:
            # Holding...
            if not self.autopilot_active:
                # Check for enable duration
                duration = time.time() - self.autopilot_btn_start
                if 0 < (duration - AUTOPILOT_HOLD_TIME) < 0.1: 
                    self.autopilot_active = True
                    self.cruise_speed = 0.0
                    self.autopilot_btn_start = time.time() - (AUTOPILOT_HOLD_TIME + 1.0)
                    print("Auto-Pilot Enabled (Hold Complete)")
                    self.rumble(0.2, 0.6, 400) # High buzz for ON

        self.autopilot_btn_prev = is_pressed

        # --- Calculate Final Outputs ---
        final_vx = raw_vx
        final_vy = raw_vy
        
        if self.autopilot_active:
            if abs(raw_vx) > 0.1:
                self.cruise_speed -= (raw_vx * CRUISE_ADJUST_RATE)
            self.cruise_speed = max(-1.0, min(1.0, self.cruise_speed))
            
            final_vx = self.cruise_speed
            final_vy = 0.0 
        else:
            self.cruise_speed = 0.0

        return {
            "vx": final_vx,
            "vy": final_vy,
            "omega": raw_omega,
            "estop": self.get_input_value(self.mappings["estop"]),
            "arm": self.get_input_value(self.mappings["arm"]),
            "autopilot": btn_val, 
            "autopilot_on": self.autopilot_active,
            "autopilot_val": self.cruise_speed
        }

    # ... (Learning methods unchanged) ...
    def start_learning(self, key):
        self.learning_mode = key
        self.learning_timeout = time.time() + LEARNING_TIMEOUT_SECONDS
        self.learning_baseline = self._get_all_axis_values()
        return True

    def check_learning(self):
        if not self.learning_mode:
            return False, None, False
        if time.time() > self.learning_timeout:
            self.learning_mode = None
            return False, None, True
        detected = self._detect_input_change()
        if detected:
            input_type, index = detected
            self.mappings[self.learning_mode] = (input_type, index)
            self.learning_mode = None
            return True, (input_type, index), False
        return False, None, False

    def _get_all_axis_values(self):
        try:
            return [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
        except:
            return []

    def _detect_input_change(self):
        try:
            for i in range(self.joy.get_numaxes()):
                if len(self.learning_baseline) > i:
                    if abs(self.joy.get_axis(i) - self.learning_baseline[i]) > LEARNING_THRESHOLD:
                        return ("axis", i)
            for i in range(self.joy.get_numbuttons()):
                if self.joy.get_button(i):
                    return ("button", i)
        except:
            pass
        return None
        
    def get_mapping_text(self, key):
        if key in self.mappings:
            input_type, index = self.mappings[key]
            return f"{'Axis' if input_type == 'axis' else 'Btn'} {index}"
        return "--"

class ControllerManager:
    # ... (No changes to Manager needed) ...
    def __init__(self):
        self.controllers = {}
        self.scan_devices()

    def scan_devices(self):
        count = pygame.joystick.get_count()
        for i in range(count):
            try:
                joy = pygame.joystick.Joystick(i)
                if not joy.get_init():
                    joy.init()
                jid = joy.get_instance_id()
                if jid not in self.controllers:
                    print(f"Found controller: {joy.get_name()} (ID: {jid})")
                    self.controllers[jid] = JoystickController(joy)
            except Exception as e:
                print(f"Error scanning joystick {i}: {e}")
        return list(self.controllers.values())

    def update(self):
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                self._handle_add(event.device_index)
            elif event.type == pygame.JOYDEVICEREMOVED:
                self._handle_remove(event.instance_id)

    def _handle_add(self, index):
        try:
            joy = pygame.joystick.Joystick(index)
            joy.init()
            jid = joy.get_instance_id()
            print(f"Hotplug Connected: {joy.get_name()} (ID: {jid})")
            self.controllers[jid] = JoystickController(joy)
        except Exception as e:
            print(f"Hotplug Error: {e}")

    def _handle_remove(self, instance_id):
        if instance_id in self.controllers:
            print(f"Hotplug Disconnected: {instance_id}")
            del self.controllers[instance_id]

    def get_controller(self, jid):
        return self.controllers.get(jid)
    
    def get_all(self):
        return list(self.controllers.values())

    def find_id_by_guid(self, target_guid):
        for jid, ctrl in self.controllers.items():
            if ctrl.guid == target_guid:
                return jid
        return None

manager = ControllerManager()
def get_manager():
    return manager