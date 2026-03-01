"""
Functions to send command packets to robots
Fixed to use 1000-2000 RC standard mapping.
"""
from config import *
import serial_comm

def send_control(robot_id, vx, vy, omega):
    """
    Send motor control command to robot.
    Maps -1.0..1.0 to 1000..2000 uint16 for RC Standard.
    """
    if not serial_comm.is_connected():
        return False
    
    def map_to_rc(val):
        # Maps -1.0..1.0 to 1000..2000
        # 1500 is neutral
        val_rc = int(1500 + (val * 500))
        return int(max(1000, min(2000, val_rc)))
    
    vx_u16 = map_to_rc(vx)
    vy_u16 = map_to_rc(vy)
    omega_u16 = map_to_rc(omega)
    
    # Packet Format: [TYPE, ID, VX_L, VX_H, VY_L, VY_H, OMEGA_L, OMEGA_H]
    pkt = bytes([
        PACKET_CONTROL, 
        int(robot_id), 
        vx_u16 & 0xFF, (vx_u16 >> 8) & 0xFF,
        vy_u16 & 0xFF, (vy_u16 >> 8) & 0xFF,
        omega_u16 & 0xFF, (omega_u16 >> 8) & 0xFF
    ])
    return serial_comm.write(pkt)

def send_estop(robot_id):
    if not serial_comm.is_connected(): return False
    pkt = bytes([PACKET_ESTOP, int(robot_id)])
    return serial_comm.write(pkt)

def send_arm(robot_id):
    if not serial_comm.is_connected(): return False
    pkt = bytes([PACKET_ESTOP_CLEAR, int(robot_id)])
    return serial_comm.write(pkt)

def send_confirmation(robot_id, step_id, approved):
    if not serial_comm.is_connected(): return False
    # PACKET_CONFIRM expects [Type, ID, Step, Approved]
    pkt = bytes([PACKET_CONFIRM, int(robot_id), int(step_id), 1 if approved else 0])
    return serial_comm.write(pkt)

def send_start_sequence(robot_id, sequence_id):
    if not serial_comm.is_connected(): return False
    pkt = bytes([PACKET_START_SEQUENCE, int(robot_id), int(sequence_id)])
    return serial_comm.write(pkt)