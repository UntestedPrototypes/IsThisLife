"""
Serial reading thread for receiving data from controller
"""
import threading
import time
import serial_comm
import telemetry_parser


class SerialReader:
    """Background thread for reading serial data"""
    
    def __init__(self, on_telemetry=None, on_confirmation=None, on_debug=None):
        self.running = False
        self.rx_buffer = bytearray()
        self.thread = None
        
        # Callbacks
        self.on_telemetry = on_telemetry
        self.on_confirmation = on_confirmation
        self.on_debug = on_debug

        self.parser = telemetry_parser.TelemetryParser()
    
    def start(self):
        """Start the serial reading thread"""
        if self.running:
            return
        
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """Stop the serial reading thread"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
    
    def _read_loop(self):
        """Main reading loop (runs in background thread)"""
        while self.running:
            if not serial_comm.is_connected():
                time.sleep(0.1)
                continue
            
            try:
                # Use the new read_all() from serial_comm.py
                data = serial_comm.read_all()
                if data:
                    # Pass the raw binary directly to the new parser
                    packets = self.parser.process_bytes(data)
                    
                    # Handle the parsed objects
                    for pkt in packets:
                        if isinstance(pkt, telemetry_parser.TelemetryData):
                            if self.on_telemetry:
                                self.on_telemetry(pkt)
                        elif isinstance(pkt, telemetry_parser.ConfirmRequest):
                            if self.on_confirmation:
                                self.on_confirmation(pkt)
            
            except Exception as e:
                if self.on_debug:
                    self.on_debug(f"Serial read error: {e}")
            
            time.sleep(0.02)  # 50 Hz
    
    def _process_line(self, line):
        """Process a received line"""
        # Check for confirmation request
        conf_req = telemetry_parser.parse_confirmation_request(line)
        if conf_req:
            if self.on_debug:
                self.on_debug(line)
            if self.on_confirmation:
                self.on_confirmation(conf_req)
            return
        
        # Check for telemetry
        telemetry = telemetry_parser.parse_telemetry_line(line)
        if telemetry:
            if self.on_telemetry:
                self.on_telemetry(telemetry)
            return
        
        # Check for debug messages
        if telemetry_parser.is_debug_line(line):
            if self.on_debug:
                self.on_debug(line)
            return
        
        # Unknown line type
        if self.on_debug:
            self.on_debug(f"Unknown: {line}")
