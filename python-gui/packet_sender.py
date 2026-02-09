"""
Functions to send command packets to robots
"""
from config import *
import serial_comm


def send_control(robot_id, vx, vy, omega):
    """
    Send motor control command to robot
    
    Args:
        robot_id: Target robot ID
        vx: Forward/backward velocity (-127 to 127)
        vy: Left/right velocity (-127 to 127)
        omega: Rotation velocity (-127 to 127)
    """
    if not serial_comm.is_connected():
        return False
    
    # Clamp values to valid range
    vx_byte = int(max(-127, min(127, vx))) & 0xFF
    vy_byte = int(max(-127, min(127, vy))) & 0xFF
    omega_byte = int(max(-127, min(127, omega))) & 0xFF
    
    pkt = bytes([PACKET_CONTROL, robot_id, vx_byte, vy_byte, omega_byte])
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
        sequence_id: Sequence to run (see config.py for available sequences)
    
    Available sequences:
        SEQUENCE_CALIBRATION_FULL (0): Full calibration
        SEQUENCE_CALIBRATION_GYRO (1): Gyro calibration only
        SEQUENCE_CALIBRATION_MOTORS (2): Motor test only
        SEQUENCE_DEMO_DANCE (3): Demo dance routine
        SEQUENCE_SENSOR_TEST (4): Test all sensors
        SEQUENCE_PATH_FOLLOW (5): Follow pre-programmed path
    """
    if not serial_comm.is_connected():
        print("Error: Not connected to controller")
        return False
    
    pkt = bytes([PACKET_START_SEQUENCE, robot_id, sequence_id])
    success = serial_comm.write(pkt)
    
    if success:
        print(f"Sent start sequence: robot={robot_id} sequence_id={sequence_id}")
    
    return success
