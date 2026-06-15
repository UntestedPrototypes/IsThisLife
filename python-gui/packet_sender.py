"""
Functions to send command packets to robots
"""
from config import *
import serial_comm
import struct

def send_control(robot_id, mode, vx, vy, omega):
    if not serial_comm.is_connected(): return False
    
    def map_to_rc(val):
        val_rc = int(1500 + (val * 500))
        return int(max(1000, min(2000, val_rc)))
    
    vx_u16 = map_to_rc(vx)
    vy_u16 = map_to_rc(vy)
    omega_u16 = map_to_rc(omega)
    
    pkt = bytes([
        PACKET_CONTROL, int(robot_id), int(mode),
        vx_u16 & 0xFF, (vx_u16 >> 8) & 0xFF,
        vy_u16 & 0xFF, (vy_u16 >> 8) & 0xFF,
        omega_u16 & 0xFF, (omega_u16 >> 8) & 0xFF
    ])
    return serial_comm.write(pkt)

def send_estop(robot_id):
    if not serial_comm.is_connected(): return False
    return serial_comm.write(bytes([PACKET_ESTOP, int(robot_id)]))

def send_arm(robot_id):
    if not serial_comm.is_connected(): return False
    return serial_comm.write(bytes([PACKET_ESTOP_CLEAR, int(robot_id)]))

def send_confirmation(robot_id, step_id, approved):
    if not serial_comm.is_connected(): return False
    return serial_comm.write(bytes([PACKET_CONFIRM, int(robot_id), int(step_id), 1 if approved else 0]))

def send_start_sequence(robot_id, sequence_id):
    if not serial_comm.is_connected(): return False
    return serial_comm.write(bytes([PACKET_START_SEQUENCE, int(robot_id), int(sequence_id)]))

def send_set_setting(robot_id, key, value):
    """Sends a generic key-value payload to modify robot settings"""
    if not serial_comm.is_connected(): return False
    key_bytes = key.encode('utf-8')[:15].ljust(16, b'\x00')
    pkt = struct.pack('<BB16sf', PACKET_SET_SETTING, int(robot_id), key_bytes, float(value))
    return serial_comm.write(pkt)

def send_get_setting(robot_id, key):
    """Requests a setting's current value from the robot"""
    if not serial_comm.is_connected(): return False
    key_bytes = key.encode('utf-8')[:15].ljust(16, b'\x00')
    pkt = struct.pack('<BB16s', PACKET_GET_SETTING, int(robot_id), key_bytes)
    return serial_comm.write(pkt)