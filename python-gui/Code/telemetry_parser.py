"""
Binary Telemetry Parser
Handles 0xAA 0x55 (Telemetry) and 0xFF 0xAA (Confirmation Requests) synchronization.
"""
import struct
from collections import namedtuple

# Namedtuples for clean data access in the UI
TelemetryData = namedtuple('TelemetryData', [
    'robot_id', 'acked_type', 'heartbeat', 'status', 
    'battery_mv', 'motor_temp', 'error_flags', 'latency_ms',
    'main_roll', 'main_pitch', 'pend_roll', 'pend_pitch'
])

ConfirmRequest = namedtuple('ConfirmRequest', [
    'type', 'robot_id', 'heartbeat', 'step_id', 'message'
])

# Format Strings (Little-endian <)
# AckTelemetryPacket: 2*uint8, 1*uint32, 1*uint8, 1*uint16, 1*int16, 1*uint8, 1*uint16, 4*float
# B=uint8, I=uint32, H=uint16, h=int16, f=float
BINARY_FORMAT_TELEMETRY = "<BBIBHhBHffff"
PAYLOAD_SIZE_TELEMETRY = struct.calcsize(BINARY_FORMAT_TELEMETRY)

# RequestConfirmPacket: 2*uint8, 1*uint32, 1*uint8, 32*char
BINARY_FORMAT_CONFIRM = "<BBIB32s"
PAYLOAD_SIZE_CONFIRM = struct.calcsize(BINARY_FORMAT_CONFIRM)

SYNC_HEADER_TELEMETRY = b'\xAA\x55'
SYNC_HEADER_CONFIRM = b'\xFF\xAA'

class TelemetryParser:
    def __init__(self):
        self.buffer = bytearray()

    def process_bytes(self, new_bytes):
        """
        Scans raw serial bytes for headers and returns a list of data objects.
        Automatically aligns to the nearest sync header to avoid garbage data.
        """
        self.buffer.extend(new_bytes)
        packets = []

        while len(self.buffer) >= 2:
            # Look for both possible headers
            t_idx = self.buffer.find(SYNC_HEADER_TELEMETRY)
            c_idx = self.buffer.find(SYNC_HEADER_CONFIRM)

            # Find which header appears first in the buffer
            indices = [i for i in [t_idx, c_idx] if i != -1]
            if not indices:
                # No header found; keep the last byte in case it's the start of a header
                self.buffer = self.buffer[-1:]
                break
            
            header_idx = min(indices)
            
            # Discard any noise/debug text bytes before the first valid header
            if header_idx > 0:
                self.buffer = self.buffer[header_idx:]
                continue

            # Check which packet type we found and if we have enough data for it
            if self.buffer.startswith(SYNC_HEADER_TELEMETRY):
                if len(self.buffer) < (2 + PAYLOAD_SIZE_TELEMETRY):
                    break # Wait for more bytes
                
                payload = self.buffer[2 : 2 + PAYLOAD_SIZE_TELEMETRY]
                try:
                    unpacked = struct.unpack(BINARY_FORMAT_TELEMETRY, payload)
                    packets.append(TelemetryData(*unpacked))
                except struct.error as e:
                    print(f"Parser Unpack Error (Telemetry): {e}")
                
                # Advance buffer past header and payload
                self.buffer = self.buffer[2 + PAYLOAD_SIZE_TELEMETRY:]

            elif self.buffer.startswith(SYNC_HEADER_CONFIRM):
                if len(self.buffer) < (2 + PAYLOAD_SIZE_CONFIRM):
                    break # Wait for more bytes
                
                payload = self.buffer[2 : 2 + PAYLOAD_SIZE_CONFIRM]
                try:
                    unpacked = struct.unpack(BINARY_FORMAT_CONFIRM, payload)
                    # Decode the message string and strip null terminators
                    msg = unpacked[4].split(b'\x00')[0].decode('utf-8', errors='replace')
                    packets.append(ConfirmRequest(unpacked[0], unpacked[1], unpacked[2], unpacked[3], msg))
                except struct.error as e:
                    print(f"Parser Unpack Error (Confirm): {e}")
                
                # Advance buffer past header and payload
                self.buffer = self.buffer[2 + PAYLOAD_SIZE_CONFIRM:]

        return packets