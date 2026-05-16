# Opening in VS Code

- Firmware: open `firmware/` as a PlatformIO project
- Python GUI: open `python-gui/` for Python development

# HOW TO INSTALL

- Follow the README.md in 'python-gui' to install all dependencies to run the python script
- Follow the README.md in 'firmware' to install

---

# 🎮 Virtual Game Controller Integration

This guide outlines how to interface a virtual or custom game controller with the Robot Controller Dashboard using an OS-level virtual joystick.

By emulating an Xbox 360 controller at the operating system level, the Python Dashboard will automatically detect it as a standard hot-plugged gamepad, allowing you to script automated sequences or bridge custom inputs.

---

## 🛠️ Prerequisites

We use the `vgamepad` Python library to create the virtual Xbox 360 controller.

Install it via pip:

```bash
pip install vgamepad

import time
import vgamepad as vg

class VirtualRobotController:
    def __init__(self):
        print("Initializing Virtual Xbox 360 Controller...")
        self.gamepad = vg.VX360Gamepad()
        # Give the OS a moment to register the new virtual hardware
        time.sleep(1.0) 
        print("Virtual Controller Ready! You can now assign it in the Dashboard.")

    def update(self):
        """Pushes the current state to the virtual gamepad."""
        self.gamepad.update()

    def set_movement(self, vx=0.0, vy=0.0, omega=0.0):
        """
        Sets the joystick axes for movement.
        Values must be floats between -1.0 and 1.0.
        """
        def to_int(val):
            return int(max(-1.0, min(1.0, val)) * 32767)

        # Left joystick: X controls Omega (Rotation), Y controls VX (Forward/Back)
        self.gamepad.left_joystick(x_value=to_int(omega), y_value=to_int(vx))
  
        # Right joystick: X controls VY (Strafe)
        self.gamepad.right_joystick(x_value=to_int(vy), y_value=0)
        self.update()

    def press_button(self, button_name, duration=0.1):
        """Presses a specific button mapped to the robot's functions."""
        buttons = self._get_button_map()
        if button_name in buttons:
            self.gamepad.press_button(button=buttons[button_name])
            self.update()
            time.sleep(duration)
            self.gamepad.release_button(button=buttons[button_name])
            self.update()
        else:
            print(f"Unknown button: {button_name}")

    def hold_button(self, button_name):
        """Holds a button down (e.g., for Cruise Control)."""
        buttons = self._get_button_map()
        if button_name in buttons:
            self.gamepad.press_button(button=buttons[button_name])
            self.update()

    def release_button(self, button_name):
        """Releases a held button."""
        buttons = self._get_button_map()
        if button_name in buttons:
            self.gamepad.release_button(button=buttons[button_name])
            self.update()
    
    def _get_button_map(self):
        return {
            "ESTOP": vg.XUSB_BUTTON.XUSB_GAMEPAD_A,
            "ARM": vg.XUSB_BUTTON.XUSB_GAMEPAD_B,
            "CRUISE": vg.XUSB_BUTTON.XUSB_GAMEPAD_Y,
            "MODE_SWITCH": vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER
        }

# ==========================================
# Example Usage / Test Sequence
# ==========================================
if __name__ == "__main__":
    controller = VirtualRobotController()
  
    print("Arming robot...")
    controller.press_button("ARM")
    time.sleep(1)

    print("Driving Forward...")
    controller.set_movement(vx=0.5, vy=0.0, omega=0.0)
    time.sleep(3)

    print("Stopping...")
    controller.set_movement(vx=0.0, vy=0.0, omega=0.0)
    time.sleep(1)
  
    print("Disarming...")
    controller.press_button("ARM")



How to Connect to the Dashboard

Launch the Dashboard: Start your main Robot Controller UI (python main.py).

Run the Virtual Controller: Open a new terminal window and run python virtual_controller.py.

Scan Devices: In the Dashboard, navigate to the Game Controller tab and click "↻ Scan Controllers".

Assign the Robot: Select your Target Robot ID, choose the newly appeared "XInput Controller" (or "Xbox 360 Controller") from the dropdown, and click "Apply Settings to Robot".

Your Python script is now remotely piloting the robot!
```


| **Function**                      | **Virtual Axis** | **Physical Stick Equivalent**   |
| :---------------------------------- | ------------------ | --------------------------------- |
| **Forward/Backward (**$v_x$**)**  | `Axis 1`         | Left Stick Y-Axis (Up/Down)     |
| **Strafe Left/Right (**$v_y$**)** | `Axis 2`         | Right Stick X-Axis (Left/Right) |


| **Function**       | **Virtual Button** | **Dashboard Behavior**                               |
| -------------------- | -------------------- | ------------------------------------------------------ |
| **E-STOP**         | `A Button`         | Halts and disarms immediately.                       |
| **Arm/Disarm**     | `B Button`         | Toggles armed state / clears pending E-Stop.         |
| **Cruise Control** | `Y Button`         | Requires a**1-second hold** to engage.               |
| **Mode Switch**    | `Left Bumper`      | Toggles between Stabilized and Direct control modes. |
