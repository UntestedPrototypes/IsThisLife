"""
Robot Controller Dashboard - Main Entry Point

A GUI application for controlling robots via ESP-NOW communication.
Supports joystick input, telemetry monitoring, and sequence execution.
"""
import tkinter as tk
from dashboard import Dashboard


def main():
    """Main application entry point"""
    root = tk.Tk()
    app = Dashboard(root)
    
    # Handle window close
    def on_closing():
        app.cleanup()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Run the application
    root.mainloop()


if __name__ == "__main__":
    main()
