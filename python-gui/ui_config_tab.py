"""
Configuration tab for serial connection and debugging
"""
import tkinter as tk
from tkinter import ttk
import serial.tools.list_ports
from config import *
import serial_comm

class ConfigTab:
    def __init__(self, notebook, on_connect_callback=None, initial_port=None, initial_baud=None):
        self.frame = ttk.Frame(notebook)
        self.on_connect_callback = on_connect_callback
        
        # UI State
        self.port_var = tk.StringVar()
        if initial_port:
            self.port_var.set(initial_port)
            
        self.baud_var = tk.StringVar(value=str(initial_baud if initial_baud else BAUD_RATE))
        self.is_connected = False
        
        self._setup_ui()
        
    def get_frame(self):
        return self.frame

    def _setup_ui(self):
        # --- Connection Settings ---
        conn_frame = ttk.LabelFrame(self.frame, text="Connection Settings")
        conn_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Port Selection
        ttk.Label(conn_frame, text="Port:").pack(side=tk.LEFT, padx=5)
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15)
        self.port_combo.pack(side=tk.LEFT, padx=5)
        
        # Refresh Ports Button
        ttk.Button(conn_frame, text="↻", width=3, command=self.refresh_ports).pack(side=tk.LEFT, padx=2)
        
        # Baud Rate
        ttk.Label(conn_frame, text="Baud:").pack(side=tk.LEFT, padx=5)
        ttk.Entry(conn_frame, textvariable=self.baud_var, width=8, state="normal").pack(side=tk.LEFT, padx=5)
        
        # Connect Button
        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self._toggle_connection)
        self.btn_connect.pack(side=tk.LEFT, padx=10)
        
        # --- Terminal / Log ---
        log_frame = ttk.LabelFrame(self.frame, text="System Log")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.log_text = tk.Text(log_frame, height=10, state="disabled", font=("Consolas", 9))
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text['yscrollcommand'] = scrollbar.set

        # Initial port scan
        self.refresh_ports()

    def refresh_ports(self):
        """Scan for available serial ports"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        
        # If current selection is invalid, clear it or select first
        if self.port_var.get() not in ports:
            if ports:
                self.port_combo.current(0)
            else:
                self.port_var.set("")

    def _toggle_connection(self):
        if not self.is_connected:
            # Connect
            port = self.port_var.get()
            try:
                baud = int(self.baud_var.get())
                if serial_comm.connect(port, baud):
                    self.set_connected_state(True)
                    self.log_debug(f"Connected to {port} at {baud}")
                    if self.on_connect_callback:
                        self.on_connect_callback()
                else:
                    self.log_debug(f"Failed to connect to {port}")
            except ValueError:
                self.log_debug("Error: Invalid Baud Rate")
            except Exception as e:
                self.log_debug(f"Connection error: {e}")
        else:
            # Disconnect
            serial_comm.disconnect()
            self.set_connected_state(False)
            self.log_debug("Disconnected")

    def set_connected_state(self, connected):
        """Update UI state based on connection"""
        self.is_connected = connected
        if connected:
            self.btn_connect.configure(text="Disconnect")
            self.port_combo.configure(state="disabled")
        else:
            self.btn_connect.configure(text="Connect")
            self.port_combo.configure(state="normal")

    def log_debug(self, message):
        """Append message to log window"""
        self.log_text.configure(state="normal")
        self.log_text.insert("end", message + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def get_selected_port(self):
        return self.port_var.get()
        
    def get_selected_baud(self):
        try:
            return int(self.baud_var.get())
        except:
            return BAUD_RATE