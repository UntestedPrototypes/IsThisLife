"""
Parse telemetry data from robots
"""
from config import *


class TelemetryData:
    """Container for robot telemetry data"""
    def __init__(self):
        self.robot_id = 0
        self.heartbeat = 0
        self.status = STATUS_OK
        self.battery_mv = 0
        self.temperature_c = 0
        self.error_flags = 0
        self.rtt_ms = 0
    
    def update_from_dict(self, data):
        """Update telemetry from parsed dictionary"""
        self.robot_id = data.get("ID", 0)
        self.heartbeat = data.get("HB", 0)
        self.status = data.get("STATUS", STATUS_OK)
        self.battery_mv = data.get("BATT", 0)
        self.temperature_c = data.get("TEMP", 0)
        self.error_flags = data.get("ERR", 0)
        self.rtt_ms = data.get("RTT", 0)
    
    def get_status_text(self):
        """Get human-readable status text"""
        status_map = {
            STATUS_OK: "OK",
            STATUS_ESTOP: "E-STOP",
            STATUS_WAITING_CONFIRM: "WAITING",
            STATUS_RUNNING_SEQUENCE: "RUNNING SEQ"
        }
        return status_map.get(self.status, "UNKNOWN")
    
    def get_status_color(self):
        """Get color for status display"""
        color_map = {
            STATUS_OK: "green",
            STATUS_ESTOP: "red",
            STATUS_WAITING_CONFIRM: "orange",
            STATUS_RUNNING_SEQUENCE: "blue"
        }
        return color_map.get(self.status, "gray")


class ConfirmationRequest:
    """Container for confirmation request data"""
    def __init__(self, robot_id, step_id, message):
        self.robot_id = robot_id
        self.step_id = step_id
        self.message = message


def parse_telemetry_line(line):
    """
    Parse a telemetry line from the robot
    
    Args:
        line: String like "ID=1 HB=123 STATUS=0 BATT=7400 TEMP=35 ERR=0x00 RTT=10"
    
    Returns:
        TelemetryData object or None if parsing failed
    """
    if not line.startswith("ID="):
        return None
    
    try:
        telemetry = {}
        for part in line.split():
            key, val = part.split("=")
            if key == "ERR":
                telemetry[key] = int(val, 16)  # Hex value
            else:
                telemetry[key] = int(val)
        
        data = TelemetryData()
        data.update_from_dict(telemetry)
        return data
    except Exception:
        return None


def parse_confirmation_request(line):
    """
    Parse a confirmation request line from the robot
    
    Args:
        line: String like "CONFIRM_REQ ID=1 STEP=1 MSG=Start calibration?"
    
    Returns:
        ConfirmationRequest object or None if parsing failed
    """
    if not line.startswith("CONFIRM_REQ"):
        return None
    
    try:
        parts = {}
        tokens = line.split(" ")
        
        for t in tokens:
            if t.startswith("ID="):
                parts["ID"] = t.split("=", 1)[1]
            elif t.startswith("STEP="):
                parts["STEP"] = t.split("=", 1)[1]
            elif t.startswith("MSG="):
                # Everything after MSG= is the message
                parts["MSG"] = line.split("MSG=", 1)[1]
                break
        
        robot_id = int(parts.get("ID", 0))
        step_id = int(parts.get("STEP", 0))
        message = parts.get("MSG", "Unknown step")
        
        return ConfirmationRequest(robot_id, step_id, message)
    except Exception:
        return None


def is_debug_line(line):
    """Check if line is a debug message"""
    return line.startswith("DEBUG:") or line.startswith("Python")
