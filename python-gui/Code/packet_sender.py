"""
Functions to send command packets to robots
"""
from config import *
import serial_comm

def send_control(robot_id, vx, vy, omega):
    """
    Send motor control command to robot as RC pulses (1000us - 2000us)
    Format: Little Endian (Low Byte, High Byte)
    Args:
        robot_id: Target robot ID
        vx, vy, omega: -1.0 to 1.0 floats
    """
    if not serial_comm.is_connected():
        return False
    
    def map_to_us(val):
        # Map -1.0..1.0 to 1000..2000 (Center 1500)
        us = 1500 + (val * 500)
        return int(max(1000, min(2000, us)))
    
    vx_us = map_to_us(vx)
    vy_us = map_to_us(vy)
    omega_us = map_to_us(omega)
    
    # Packet Format: [TYPE, ID, VX_L, VX_H, VY_L, VY_H, OMEGA_L, OMEGA_H]
    pkt = bytes([
        PACKET_CONTROL, 
        robot_id, 
        vx_us & 0xFF, (vx_us >> 8) & 0xFF,
        vy_us & 0xFF, (vy_us >> 8) & 0xFF,
        omega_us & 0xFF, (omega_us >> 8) & 0xFF
    ])
    return serial_comm.write(pkt)

def send_estop(robot_id):
    if not serial_comm.is_connected(): return False
    pkt = bytes([PACKET_ESTOP, robot_id])
    return serial_comm.write(pkt)

def send_arm(robot_id):
    if not serial_comm.is_connected(): return False
    pkt = bytes([PACKET_ESTOP_CLEAR, robot_id])
    return serial_comm.write(pkt)

def send_confirmation(robot_id, step_id, approved):
    if not serial_comm.is_connected(): return False
    pkt = bytes([PACKET_CONFIRM, robot_id, step_id, 1 if approved else 0])
    return serial_comm.write(pkt)

def send_start_sequence(robot_id, sequence_id):
    if not serial_comm.is_connected(): return False
    pkt = bytes([PACKET_START_SEQUENCE, robot_id, sequence_id])
    return serial_comm.write(pkt)