"""
Low-level serial communication module
"""
import serial

# Global serial instance
_ser = None

def connect(port_name, baud_rate=115200):
    """
    Connect to a serial port
    Args:
        port_name: Name of the port (e.g. COM3)
        baud_rate: Baud rate (default 115200)
    Returns:
        True if successful, False otherwise
    """
    global _ser
    try:
        # Close existing connection if any
        disconnect()
        
        _ser = serial.Serial(port_name, baud_rate, timeout=0.1)
        return True
    except Exception as e:
        print(f"Serial connection error: {e}")
        return False

def disconnect():
    """Close the serial connection"""
    global _ser
    if _ser and _ser.is_open:
        try:
            _ser.close()
        except:
            pass
    _ser = None

def is_connected():
    """Check if serial port is open"""
    return _ser is not None and _ser.is_open

def write(data):
    """Write bytes to serial port"""
    if not is_connected():
        return False
    try:
        _ser.write(data)
        return True
    except Exception as e:
        print(f"Serial write error: {e}")
        return False

def read_available():
    """Read all available bytes"""
    if not is_connected():
        return None
    try:
        if _ser.in_waiting > 0:
            return _ser.read(_ser.in_waiting)
    except Exception as e:
        print(f"Serial read error: {e}")
    return None