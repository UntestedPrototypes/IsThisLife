"""
Binary Telemetry Parser
Handles Telemetry, Confirmations, and Setting Responses from Serial stream.
"""
import struct
from collections import namedtuple

TelemetryData = namedtuple('TelemetryData', [
    'type', 'robot_id', 'heartbeat', 'acked_type', 'status', 'mode',
    'battery_mv', 'motor_temp', 'error_flags', 'latency_ms',
    'imu_calibration',
    'main_roll', 'main_pitch', 'pend_roll', 'pend_pitch'
])

ConfirmRequest = namedtuple('ConfirmRequest', [
    'type', 'robot_id', 'heartbeat', 'step_id', 'message'
])

# --- NEW: Setting Response Namedtuple ---
SettingResponse = namedtuple('SettingResponse', [
    'type', 'robot_id', 'heartbeat', 'key', 'value'
])

BINARY_FORMAT_TELEMETRY = "<BBIBBBHhBHHffff"
PAYLOAD_SIZE_TELEMETRY = struct.calcsize(BINARY_FORMAT_TELEMETRY)

BINARY_FORMAT_CONFIRM = "<BBIB32s"
PAYLOAD_SIZE_CONFIRM = struct.calcsize(BINARY_FORMAT_CONFIRM)

# --- NEW: Setting Response Struct Format ---
# Header: type(B), robot_id(B), heartbeat(I) -> 6 bytes
# Payload: key(16s), value(f) -> 20 bytes
# Total = 26 bytes -> <BBI16sf
BINARY_FORMAT_SETTING = "<BBI16sf"
PAYLOAD_SIZE_SETTING = struct.calcsize(BINARY_FORMAT_SETTING)

SYNC_HEADER_TELEMETRY = b'\xAA\x55'
SYNC_HEADER_CONFIRM = b'\xFF\xAA'
SYNC_HEADER_SETTING = b'\xCC\x33' # Unique sync assigned in python_comm.cpp

class TelemetryParser:
    def __init__(self):
        self.buffer = bytearray()

    def process_bytes(self, new_bytes):
        self.buffer.extend(new_bytes)
        packets = []

        while len(self.buffer) >= 2:
            t_idx = self.buffer.find(SYNC_HEADER_TELEMETRY)
            c_idx = self.buffer.find(SYNC_HEADER_CONFIRM)
            s_idx = self.buffer.find(SYNC_HEADER_SETTING)

            indices = [i for i in [t_idx, c_idx, s_idx] if i != -1]
            if not indices:
                self.buffer = self.buffer[-1:]
                break
            
            header_idx = min(indices)
            
            if header_idx > 0:
                self.buffer = self.buffer[header_idx:]
                continue

            if self.buffer.startswith(SYNC_HEADER_TELEMETRY):
                if len(self.buffer) < (2 + PAYLOAD_SIZE_TELEMETRY): break
                
                payload = self.buffer[2 : 2 + PAYLOAD_SIZE_TELEMETRY]
                try:
                    unpacked = struct.unpack(BINARY_FORMAT_TELEMETRY, payload)
                    packets.append(TelemetryData(*unpacked))
                except struct.error as e:
                    print(f"Unpack Error (Telemetry): {e}")
                
                self.buffer = self.buffer[2 + PAYLOAD_SIZE_TELEMETRY:]

            elif self.buffer.startswith(SYNC_HEADER_CONFIRM):
                if len(self.buffer) < (2 + PAYLOAD_SIZE_CONFIRM): break
                
                payload = self.buffer[2 : 2 + PAYLOAD_SIZE_CONFIRM]
                try:
                    unpacked = struct.unpack(BINARY_FORMAT_CONFIRM, payload)
                    msg = unpacked[4].split(b'\x00')[0].decode('utf-8', errors='replace')
                    packets.append(ConfirmRequest(unpacked[0], unpacked[1], unpacked[2], unpacked[3], msg))
                except struct.error as e:
                    print(f"Unpack Error (Confirm): {e}")
                
                self.buffer = self.buffer[2 + PAYLOAD_SIZE_CONFIRM:]

            # --- NEW: Process Setting Responses ---
            elif self.buffer.startswith(SYNC_HEADER_SETTING):
                if len(self.buffer) < (2 + PAYLOAD_SIZE_SETTING): break
                
                payload = self.buffer[2 : 2 + PAYLOAD_SIZE_SETTING]
                try:
                    unpacked = struct.unpack(BINARY_FORMAT_SETTING, payload)
                    key_str = unpacked[3].split(b'\x00')[0].decode('utf-8', errors='replace')
                    packets.append(SettingResponse(unpacked[0], unpacked[1], unpacked[2], key_str, unpacked[4]))
                except struct.error as e:
                    print(f"Unpack Error (Setting): {e}")
                
                self.buffer = self.buffer[2 + PAYLOAD_SIZE_SETTING:]

        return packets