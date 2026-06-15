"""
Core serial communication wrapper
"""
import serial
import serial.tools.list_ports

# The global serial object
_ser = None

def connect(port, baud):
    global _ser
    try:
        if _ser and _ser.is_open:
            _ser.close()
        _ser = serial.Serial(port, baud, timeout=0.01) # Low timeout for responsiveness
        return True
    except Exception as e:
        print(f"Connection error: {e}")
        return False

def disconnect():
    global _ser
    if _ser and _ser.is_open:
        _ser.close()
    _ser = None

def is_connected():
    return _ser is not None and _ser.is_open

def write(data):
    """Sends raw bytes to the robot"""
    if is_connected():
        try:
            _ser.write(data)
            _ser.flush()
            return True
        except:
            return False
    return False

def read_all():
    """
    Reads every single byte currently waiting in the buffer.
    Crucial for the binary parser to find sync headers.
    """
    if is_connected():
        try:
            if _ser.in_waiting > 0:
                return _ser.read(_ser.in_waiting)
        except:
            return None
    return None

def get_available_ports():
    return [p.device for p in serial.tools.list_ports.comports()]