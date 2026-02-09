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
        vx: Forward/backward (-1.0 to 1.0)
        vy: Left/right (-1.0 to 1.0)
        omega: Rotation (-1.0 to 1.0)
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
    # Little Endian: Low Byte first
    pkt = bytes([
        PACKET_CONTROL, 
        robot_id, 
        vx_us & 0xFF, (vx_us >> 8) & 0xFF,
        vy_us & 0xFF, (vy_us >> 8) & 0xFF,
        omega_us & 0xFF, (omega_us >> 8) & 0xFF
    ])
    return serial_comm.write(pkt)


def send_estop(robot_id):
    """
    Send E-STOP command to robot
    
    Args:
        robot_id: Target robot ID
    """
    if not serial_comm.is_connected():
        return False
    
    pkt = bytes([PACKET_ESTOP, robot_id])
    return serial_comm.write(pkt)


def send_arm(robot_id):
    """
    Send ARM (clear E-STOP) command to robot
    
    Args:
        robot_id: Target robot ID
    """
    if not serial_comm.is_connected():
        return False
    
    pkt = bytes([PACKET_ESTOP_CLEAR, robot_id])
    return serial_comm.write(pkt)


def send_confirmation(robot_id, step_id, approved):
    """
    Send confirmation response to robot
    
    Args:
        robot_id: Target robot ID
        step_id: Step ID being confirmed
        approved: True to approve, False to deny
    """
    if not serial_comm.is_connected():
        return False
    
    pkt = bytes([PACKET_CONFIRM, robot_id, step_id, 1 if approved else 0])
    success = serial_comm.write(pkt)
    
    if success:
        print(f"Sent confirmation: robot={robot_id} step={step_id} approved={approved}")
    
    return success


def send_start_sequence(robot_id, sequence_id):
    """
    Send command to start a sequence on the robot
    
    Args:
        robot_id: Target robot ID
        sequence_id: Sequence to run
    """
    if not serial_comm.is_connected():
        print("Error: Not connected to controller")
        return False
    
    pkt = bytes([PACKET_START_SEQUENCE, robot_id, sequence_id])
    success = serial_comm.write(pkt)
    
    if success:
        print(f"Sent start sequence: robot={robot_id} seq={sequence_id}")
    
    return success