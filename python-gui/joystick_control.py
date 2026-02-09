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
    "arm": ("button", 1)
}

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

    def get_input_value(self, mapping):
        """Get value for a specific mapping (axis/button)"""
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
        """Get processed control dictionary"""
        return {
            "vx": self.get_input_value(self.mappings["vx"]),
            "vy": -self.get_input_value(self.mappings["vy"]),  # Inverted Y
            "omega": self.get_input_value(self.mappings["omega"]),
            "estop": self.get_input_value(self.mappings["estop"]),
            "arm": self.get_input_value(self.mappings["arm"])
        }

    def start_learning(self, key):
        self.learning_mode = key
        self.learning_timeout = time.time() + LEARNING_TIMEOUT_SECONDS
        self.learning_baseline = self._get_all_axis_values()
        return True

    def check_learning(self):
        if not self.learning_mode:
            return False, None, False

        # Timeout
        if time.time() > self.learning_timeout:
            self.learning_mode = None
            return False, None, True

        # Detect change
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
            # Check axes
            for i in range(self.joy.get_numaxes()):
                if len(self.learning_baseline) > i:
                    if abs(self.joy.get_axis(i) - self.learning_baseline[i]) > LEARNING_THRESHOLD:
                        return ("axis", i)
            # Check buttons
            for i in range(self.joy.get_numbuttons()):
                if self.joy.get_button(i):
                    return ("button", i)
        except:
            pass
        return None
        
    def get_mapping_text(self, key):
        input_type, index = self.mappings[key]
        return f"{'Axis' if input_type == 'axis' else 'Btn'} {index}"


class ControllerManager:
    """Manages detection and retrieval of controllers"""
    
    def __init__(self):
        self.controllers = {} # Map instance_id -> JoystickController
        # Initial scan
        self.scan_devices()

    def scan_devices(self):
        """Full scan of connected devices"""
        # Note: We rely on hotplug events mostly, but this is good for startup
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
        """Pump events and handle hotplugging"""
        # We use event.get() to catch connection events
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
        """Find a controller ID that matches the given GUID"""
        for jid, ctrl in self.controllers.items():
            if ctrl.guid == target_guid:
                return jid
        return None

# Global singleton
manager = ControllerManager()

def get_manager():
    return manager