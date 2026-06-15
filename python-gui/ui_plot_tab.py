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

# Increased to 1000 to allow up to a 100-second time window at 10Hz telemetry
HISTORY_LEN = 1000  

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
        
        # --- Y-Axis Scaling Controls ---
        ttk.Separator(top_frame, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=15)
        
        self.auto_scale_var = tk.BooleanVar(value=True)
        self.chk_auto = ttk.Checkbutton(top_frame, text="Auto-Scale Y", variable=self.auto_scale_var, command=self._toggle_scale_inputs)
        self.chk_auto.pack(side=tk.LEFT, padx=5)
        
        self.y_min_var = tk.StringVar(value="-90")
        self.y_max_var = tk.StringVar(value="90")
        
        self.lbl_min = ttk.Label(top_frame, text="Min:")
        self.lbl_min.pack(side=tk.LEFT, padx=(5, 2))
        self.ent_min = ttk.Entry(top_frame, textvariable=self.y_min_var, width=5, state="disabled")
        self.ent_min.pack(side=tk.LEFT, padx=(0, 5))
        
        self.lbl_max = ttk.Label(top_frame, text="Max:")
        self.lbl_max.pack(side=tk.LEFT, padx=(5, 2))
        self.ent_max = ttk.Entry(top_frame, textvariable=self.y_max_var, width=5, state="disabled")
        self.ent_max.pack(side=tk.LEFT, padx=(0, 5))
        
        # --- NEW: X-Axis Time Window Control ---
        ttk.Separator(top_frame, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=15)
        
        self.time_window_var = tk.StringVar(value="10") # Default to 10 seconds
        self.lbl_time = ttk.Label(top_frame, text="Time Window (s):")
        self.lbl_time.pack(side=tk.LEFT, padx=(5, 2))
        self.ent_time = ttk.Entry(top_frame, textvariable=self.time_window_var, width=5)
        self.ent_time.pack(side=tk.LEFT, padx=(0, 5))

        # --- Matplotlib Canvas Setup ---
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.fig.patch.set_facecolor('#f0f0f0')
        
        # Top subplot for Roll Axis
        self.ax_roll = self.fig.add_subplot(211)
        self.ax_roll.set_title("Roll Axis")
        self.ax_roll.set_ylabel("Degrees")
        self.ax_roll.grid(True)
        self.line_mr, = self.ax_roll.plot([], [], label='Main Roll', color='red', linewidth=1.5)
        self.line_pr, = self.ax_roll.plot([], [], label='Pendulum Roll', color='blue', linewidth=1.5)
        self.ax_roll.legend(loc='upper left')
        
        # Bottom subplot for Pitch Axis
        self.ax_pitch = self.fig.add_subplot(212)
        self.ax_pitch.set_title("Pitch Axis")
        self.ax_pitch.set_ylabel("Degrees")
        self.ax_pitch.set_xlabel("Time (s)")
        self.ax_pitch.grid(True)
        self.line_mp, = self.ax_pitch.plot([], [], label='Main Pitch', color='red', linewidth=1.5)
        self.line_pp, = self.ax_pitch.plot([], [], label='Pendulum Pitch', color='blue', linewidth=1.5)
        self.ax_pitch.legend(loc='upper left')
        
        self.fig.tight_layout(pad=3.0)
        
        # Embed the plot in Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    def _toggle_scale_inputs(self):
        """Enable or disable manual scale entries based on the checkbox."""
        state = "disabled" if self.auto_scale_var.get() else "normal"
        self.ent_min.configure(state=state)
        self.ent_max.configure(state=state)

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
        
        # Calculate relative time
        t_start = d['t'][0]
        t_rel = [t - t_start for t in d['t']]
        t_latest = t_rel[-1]
        
        # Update line data
        self.line_mr.set_data(t_rel, d['mr'])
        self.line_pr.set_data(t_rel, d['pr'])
        self.line_mp.set_data(t_rel, d['mp'])
        self.line_pp.set_data(t_rel, d['pp'])

        # --- Handle X-Axis (Time Window) ---
        try:
            window = float(self.time_window_var.get())
            if window <= 0: window = 10.0 # Prevent zero or negative windows
        except ValueError:
            window = 10.0 # Fallback if user types non-numbers

        # Pin the left edge to 0 until the graph fills up the requested window size
        x_min = max(0, t_latest - window)
        x_max = max(window, t_latest)
        
        self.ax_roll.set_xlim(x_min, x_max)
        self.ax_pitch.set_xlim(x_min, x_max)
        
        # --- Handle Y-Axis Scaling ---
        if self.auto_scale_var.get():
            # Auto scale only the Y-Axis (Scalex=False avoids overriding our manual time window)
            self.ax_roll.relim()
            self.ax_roll.autoscale_view(scalex=False, scaley=True)
            self.ax_pitch.relim()
            self.ax_pitch.autoscale_view(scalex=False, scaley=True)
        else:
            try:
                # Apply user-defined Y limits
                y_min = float(self.y_min_var.get())
                y_max = float(self.y_max_var.get())
                self.ax_roll.set_ylim(y_min, y_max)
                self.ax_pitch.set_ylim(y_min, y_max)
            except ValueError:
                # Fallback to auto if the user types something invalid
                self.ax_roll.relim()
                self.ax_roll.autoscale_view(scalex=False, scaley=True)
                self.ax_pitch.relim()
                self.ax_pitch.autoscale_view(scalex=False, scaley=True)
        
        self.canvas.draw()