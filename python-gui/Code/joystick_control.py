"""
Game controller / joystick input handling with multi-controller support
"""
import pygame
import time
from config import *

# Initialize pygame once globally
pygame.init()
pygame.joystick.init()

DEFAULT_MAPPINGS = {
    "vx": ("axis", 1),       # Left Stick Up/Down
    "vy": ("axis", 2),       # Right Stick Left/Right
    "omega": ("none", 0),    # Unbound (Rotation disabled)
    "estop": ("button", 0),  # A Button / Cross
    "arm": ("button", 1),    # B Button / Circle
    "autopilot": ("button", 3) # Y Button / Triangle
}

AUTOPILOT_HOLD_TIME = 1.0
CRUISE_ADJUST_RATE = 0.02

class JoystickController:
    """Represents a single connected joystick"""
    
    def __init__(self, pygame_joy_instance):
        self.joy = pygame_joy_instance
        try:
            self.joy.init()
        except: pass
        self.id = self.joy.get_instance_id()
        self.name = self.joy.get_name()
        self.guid = self.joy.get_guid()
        self.mappings = DEFAULT_MAPPINGS.copy()
        
        # Internal States
        self.autopilot_active = False
        self.cruise_speed = 0.0
        self.autopilot_btn_start = 0
        self.autopilot_btn_prev = False
        
        # Learning States
        self.learning_mode = None
        self.learning_timeout = 0
        self.learning_baseline = []

    def rumble(self, low, high, duration):
        try: self.joy.rumble(low, high, int(duration))
        except: pass

    def get_input_value(self, mapping):
        input_type, index = mapping
        try:
            if input_type == "axis" and index < self.joy.get_numaxes():
                return self.joy.get_axis(index)
            elif input_type == "button" and index < self.joy.get_numbuttons():
                return self.joy.get_button(index)
        except: pass
        return 0

    def get_control_values(self):
        # Raw Inputs
        raw_vx = self.get_input_value(self.mappings["vx"])
        raw_vy = -self.get_input_value(self.mappings["vy"])
        raw_omega = self.get_input_value(self.mappings["omega"])
        
        # Autopilot Logic
        btn_val = self.get_input_value(self.mappings["autopilot"])
        is_pressed = (btn_val > 0.5)
        
        if is_pressed and not self.autopilot_btn_prev:
            if self.autopilot_active:
                self.autopilot_active = False
                self.cruise_speed = 0.0
                self.rumble(0.6, 0.0, 300)
            else:
                self.autopilot_btn_start = time.time()
        
        if is_pressed and self.autopilot_btn_prev:
            if not self.autopilot_active:
                if (time.time() - self.autopilot_btn_start) > AUTOPILOT_HOLD_TIME:
                    self.autopilot_active = True
                    self.cruise_speed = 0.0
                    self.autopilot_btn_start = time.time() + 999 
                    self.rumble(0.2, 0.6, 400)
        
        self.autopilot_btn_prev = is_pressed
        
        final_vx = raw_vx
        final_vy = raw_vy
        
        if self.autopilot_active:
            if abs(raw_vx) > 0.1: self.cruise_speed -= (raw_vx * CRUISE_ADJUST_RATE)
            self.cruise_speed = max(-1.0, min(1.0, self.cruise_speed))
            final_vx = self.cruise_speed
            #final_vy = 0.0
            
        return {
            "vx": final_vx, "vy": final_vy, "omega": raw_omega,
            "estop": self.get_input_value(self.mappings["estop"]),
            "arm": self.get_input_value(self.mappings["arm"]),
            "autopilot": btn_val,
            "autopilot_on": self.autopilot_active,
            "autopilot_val": self.cruise_speed
        }

    # Learning methods omitted for brevity (same as previous)
    def start_learning(self, key):
        self.learning_mode = key; self.learning_timeout = time.time() + LEARNING_TIMEOUT_SECONDS; self.learning_baseline = self._get_all_axis_values()
        return True
    def check_learning(self):
        if not self.learning_mode: return False, None, False
        if time.time() > self.learning_timeout: self.learning_mode=None; return False,None,True
        detected = self._detect_input_change()
        if detected: self.mappings[self.learning_mode]=detected; self.learning_mode=None; return True,detected,False
        return False,None,False
    def _get_all_axis_values(self):
        try: return [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
        except: return []
    def _detect_input_change(self):
        try:
            for i in range(self.joy.get_numaxes()):
                if len(self.learning_baseline)>i and abs(self.joy.get_axis(i)-self.learning_baseline[i])>LEARNING_THRESHOLD: return ("axis",i)
            for i in range(self.joy.get_numbuttons()):
                if self.joy.get_button(i): return ("button",i)
        except: pass
        return None
    def get_mapping_text(self, k): return f"{'Axis' if self.mappings.get(k,[0])[0]=='axis' else 'Btn'} {self.mappings.get(k,[0,0])[1]}" if k in self.mappings else "--"


class ControllerManager:
    def __init__(self):
        self.controllers = {}
        self.scan_devices()

    def scan_devices(self):
        count = pygame.joystick.get_count()
        for i in range(count):
            try:
                joy = pygame.joystick.Joystick(i)
                if not joy.get_init(): joy.init()
                jid = joy.get_instance_id()
                if jid not in self.controllers:
                    print(f"Found: {joy.get_name()} (ID: {jid}) GUID: {joy.get_guid()}")
                    self.controllers[jid] = JoystickController(joy)
            except: pass
        return list(self.controllers.values())

    def update(self):
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED: self._handle_add(event.device_index)
            elif event.type == pygame.JOYDEVICEREMOVED: self._handle_remove(event.instance_id)

    def _handle_add(self, index):
        try:
            joy = pygame.joystick.Joystick(index)
            joy.init()
            jid = joy.get_instance_id()
            self.controllers[jid] = JoystickController(joy)
            print(f"Hotplug: {joy.get_name()} (ID: {jid})")
        except: pass

    def _handle_remove(self, instance_id):
        if instance_id in self.controllers: del self.controllers[instance_id]

    def get_controller(self, jid): return self.controllers.get(jid)
    def get_all(self): return list(self.controllers.values())

    # --- THE BETTER WAY ---
    def find_id_by_guid(self, target_guid, instance_index=0, exclude_ids=None):
        """
        Find a controller with target_guid.
        If there are multiple, return the one at 'instance_index'.
        """
        if exclude_ids is None: exclude_ids = []
        
        # 1. Find ALL matches
        matches = []
        # Sort by ID to ensure stability (Controller 0 is always first, etc.)
        sorted_ids = sorted(self.controllers.keys())
        
        for jid in sorted_ids:
            ctrl = self.controllers[jid]
            if ctrl.guid == target_guid and jid not in exclude_ids:
                matches.append(jid)
        
        # 2. Pick the Nth match
        if instance_index < len(matches):
            return matches[instance_index]
            
        return None

manager = ControllerManager()
def get_manager(): return manager