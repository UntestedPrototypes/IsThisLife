"""
Live plotting tab for IMU Roll and Pitch data.
Requires matplotlib (pip install matplotlib).
"""
import tkinter as tk
from tkinter import ttk
import time
from collections import deque

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Number of historical data points to keep on the graph
HISTORY_LEN = 200  

class PlotTab:
    def __init__(self, notebook, robot_state_manager):
        self.frame = ttk.Frame(notebook)
        self.robot_state = robot_state_manager
        
        # Currently selected robot to plot
        self.current_robot_id = tk.StringVar()
        
        # Store historical data for each robot ID
        # Format: { robot_id: { 't': deque, 'mr': deque, 'mp': deque, 'pr': deque, 'pp': deque } }
        self.data = {}
        
        self._setup_ui()
        self._update_dropdown_loop()
        self._animation_loop()

    def get_frame(self):
        return self.frame

    def _setup_ui(self):
        # --- Top Control Bar ---
        top_frame = ttk.Frame(self.frame)
        top_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(top_frame, text="Select Target Robot:", font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(0, 5))
        self.cb_robots = ttk.Combobox(top_frame, textvariable=self.current_robot_id, width=10, state="readonly")
        self.cb_robots.pack(side=tk.LEFT, padx=5)
        
        # --- Matplotlib Canvas Setup ---
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.fig.patch.set_facecolor('#f0f0f0')
        
        # Top subplot for Roll Axis
        self.ax_roll = self.fig.add_subplot(211)
        self.ax_roll.set_title("Roll Axis")
        self.ax_roll.set_ylabel("Degrees")
        self.ax_roll.grid(True)
        # Main is red, Pendulum is blue
        self.line_mr, = self.ax_roll.plot([], [], label='Main Roll', color='red', linewidth=1.5)
        self.line_pr, = self.ax_roll.plot([], [], label='Pendulum Roll', color='blue', linewidth=1.5)
        self.ax_roll.legend(loc='upper right')
        
        # Bottom subplot for Pitch Axis
        self.ax_pitch = self.fig.add_subplot(212)
        self.ax_pitch.set_title("Pitch Axis")
        self.ax_pitch.set_ylabel("Degrees")
        self.ax_pitch.set_xlabel("Time (s)")
        self.ax_pitch.grid(True)
        # Main is red, Pendulum is blue
        self.line_mp, = self.ax_pitch.plot([], [], label='Main Pitch', color='red', linewidth=1.5)
        self.line_pp, = self.ax_pitch.plot([], [], label='Pendulum Pitch', color='blue', linewidth=1.5)
        self.ax_pitch.legend(loc='upper right')
        
        self.fig.tight_layout(pad=3.0)
        
        # Embed the plot in Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    def update_telemetry(self, data):
        """Called by the main loop whenever new telemetry arrives"""
        r_id = int(data.robot_id)
        
        # Initialize queues if robot is new
        if r_id not in self.data:
            self.data[r_id] = {
                't': deque(maxlen=HISTORY_LEN),
                'mr': deque(maxlen=HISTORY_LEN),
                'mp': deque(maxlen=HISTORY_LEN),
                'pr': deque(maxlen=HISTORY_LEN),
                'pp': deque(maxlen=HISTORY_LEN)
            }
        
        # Append latest data
        d = self.data[r_id]
        d['t'].append(time.time())
        d['mr'].append(data.main_roll)
        d['mp'].append(data.main_pitch)
        d['pr'].append(data.pend_roll)
        d['pp'].append(data.pend_pitch)

    def _update_dropdown_loop(self):
        """Keeps the dropdown list populated with known robots"""
        active_ids = [str(r) for r in self.robot_state.get_all_robot_ids()]
        current_vals = list(self.cb_robots['values'])
        if active_ids != current_vals:
            self.cb_robots['values'] = active_ids
            if active_ids and not self.current_robot_id.get():
                self.cb_robots.current(0)
        
        self.frame.after(500, self._update_dropdown_loop)

    def _animation_loop(self):
        """Redraws the plot at 10 Hz"""
        self._redraw_plot()
        self.frame.after(100, self._animation_loop)

    def _redraw_plot(self):
        """Updates the plot data efficiently"""
        r_id_str = self.current_robot_id.get()
        if not r_id_str: return
        
        r_id = int(r_id_str)
        if r_id not in self.data: return
        
        d = self.data[r_id]
        if not d['t']: return
        
        # Calculate relative time for the X-axis (scrolling effect)
        t_start = d['t'][0]
        t_rel = [t - t_start for t in d['t']]
        
        # Update Roll Plot
        self.line_mr.set_data(t_rel, d['mr'])
        self.line_pr.set_data(t_rel, d['pr'])
        self.ax_roll.relim()
        self.ax_roll.autoscale_view(scalex=True, scaley=True)
        
        # Update Pitch Plot
        self.line_mp.set_data(t_rel, d['mp'])
        self.line_pp.set_data(t_rel, d['pp'])
        self.ax_pitch.relim()
        self.ax_pitch.autoscale_view(scalex=True, scaley=True)
        
        self.canvas.draw()