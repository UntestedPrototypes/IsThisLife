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
                # Read available data
                data = serial_comm.read_available()
                if data:
                    self.rx_buffer.extend(data)
                    
                    # Process complete lines
                    while b"\n" in self.rx_buffer:
                        newline_idx = self.rx_buffer.find(b"\n")
                        line_bytes = self.rx_buffer[:newline_idx]
                        self.rx_buffer = self.rx_buffer[newline_idx + 1:]
                        
                        # Decode line
                        try:
                            line = line_bytes.decode("utf-8", errors="replace").strip()
                            if line:
                                self._process_line(line)
                        except Exception as e:
                            if self.on_debug:
                                self.on_debug(f"Line decode error: {e}")
            
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
