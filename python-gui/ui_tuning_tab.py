"""
Dedicated tab for viewing and tuning PID and robot variables.
Loads required variables dynamically based on config groups.
"""
import tkinter as tk
from tkinter import ttk, messagebox
import packet_sender
import platform

class TuningTab:
    def __init__(self, notebook, robot_state_manager, settings_data):
        self.frame = ttk.Frame(notebook)
        self.robot_state = robot_state_manager
        
        # Handle both legacy flat lists and the new grouped dictionaries
        if isinstance(settings_data, list):
            self.settings_groups = {"Uncategorized Variables": settings_data}
        elif isinstance(settings_data, dict):
            self.settings_groups = settings_data
        else:
            self.settings_groups = {"Error": []}

        # Create a flat list of all active keys for the 'Fetch All' button
        self.settings_list = []
        for group, keys in self.settings_groups.items():
            for key in keys:
                if key not in self.settings_list:
                    self.settings_list.append(key)
        
        self.current_robot_id = tk.StringVar()
        self.setting_widgets = {}
        
        self._setup_ui()
        self.frame.after(200, self._update_loop)

    def get_frame(self): return self.frame

    def _setup_ui(self):
        # Top Control Bar
        top_frame = ttk.Frame(self.frame)
        top_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(top_frame, text="Select Target Robot:", font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(0, 5))
        self.cb_robots = ttk.Combobox(top_frame, textvariable=self.current_robot_id, width=10, state="readonly")
        self.cb_robots.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(top_frame, text="Fetch All Loaded", command=self._get_all).pack(side=tk.LEFT, padx=20)
        
        # Main Settings Container
        container = ttk.LabelFrame(self.frame, text=" Active Configuration Variables ")
        container.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        canvas = tk.Canvas(container)
        scrollbar = ttk.Scrollbar(container, orient="vertical", command=canvas.yview)
        self.scrollable_frame = ttk.Frame(canvas)
        
        self.scrollable_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True, padx=5, pady=5)
        scrollbar.pack(side="right", fill="y")
        
        # --- NEW: Scroll Wheel Binding Logic ---
        def _on_mousewheel(event):
            # Windows/Mac use event.delta
            if platform.system() == "Windows":
                canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
            elif platform.system() == "Darwin": # macOS
                canvas.yview_scroll(int(-1 * event.delta), "units")
            else: # Linux uses event.num
                if event.num == 4:
                    canvas.yview_scroll(-1, "units")
                elif event.num == 5:
                    canvas.yview_scroll(1, "units")

        def _bound_to_mousewheel(event):
            canvas.bind_all("<MouseWheel>", _on_mousewheel)
            canvas.bind_all("<Button-4>", _on_mousewheel)
            canvas.bind_all("<Button-5>", _on_mousewheel)

        def _unbound_to_mousewheel(event):
            canvas.unbind_all("<MouseWheel>")
            canvas.unbind_all("<Button-4>")
            canvas.unbind_all("<Button-5>")

        # Only bind mouse wheel when hovering over the canvas
        canvas.bind('<Enter>', _bound_to_mousewheel)
        canvas.bind('<Leave>', _unbound_to_mousewheel)
        # ---------------------------------------
        
        # Headers
        headers = ["Variable Key", "Current Value", "Action", "New Value", "Action"]
        for col, text in enumerate(headers):
            ttk.Label(self.scrollable_frame, text=text, font=("Arial", 9, "bold")).grid(row=0, column=col, padx=15, pady=(5, 5), sticky="w")
            
        ttk.Separator(self.scrollable_frame, orient='horizontal').grid(row=1, column=0, columnspan=5, sticky='ew', pady=(0, 5))

        # Dynamically generate rows grouped by config dictionary
        row = 2
        for group_name, keys in self.settings_groups.items():
            if not keys:
                continue
                
            # Group Header Section
            ttk.Label(self.scrollable_frame, text=group_name, font=("Arial", 10, "bold"), foreground="#2c3e50").grid(row=row, column=0, columnspan=5, sticky="w", padx=5, pady=(15, 5))
            row += 1
            
            for key in keys:
                self._create_row(row, key)
                row += 1

    def _create_row(self, row, key):
        """Helper to create standard rows for setting fields"""
        ttk.Label(self.scrollable_frame, text=key, font=("Consolas", 10)).grid(row=row, column=0, padx=15, pady=2, sticky="w")
        
        lbl_val = ttk.Label(self.scrollable_frame, text="Not Loaded", foreground="gray", width=12, anchor="center", background="#e8e8e8")
        lbl_val.grid(row=row, column=1, padx=15, pady=2)
        
        ttk.Button(self.scrollable_frame, text="Get", width=8, 
                   command=lambda k=key: self._get_setting(k)).grid(row=row, column=2, padx=15, pady=2)
        
        entry_var = tk.StringVar(value="0.0")
        ttk.Entry(self.scrollable_frame, textvariable=entry_var, width=12).grid(row=row, column=3, padx=15, pady=2)
        
        ttk.Button(self.scrollable_frame, text="Set & Push", width=12, 
                   command=lambda k=key, v=entry_var: self._set_setting(k, v.get())).grid(row=row, column=4, padx=15, pady=2)
        
        self.setting_widgets[key] = {
            "val_lbl": lbl_val,
            "entry_var": entry_var
        }

    def _get_setting(self, key):
        r_id = self.current_robot_id.get()
        if not r_id:
            messagebox.showwarning("Warning", "Please select a robot first.")
            return
            
        r_id = int(r_id)
        self.robot_state.get_robot(r_id).fetched_settings[key] = "Fetching..."
        packet_sender.send_get_setting(r_id, key)

    def _get_all(self):
        r_id = self.current_robot_id.get()
        if not r_id:
            messagebox.showwarning("Warning", "Please select a robot first.")
            return
            
        r_id = int(r_id)
        for key in self.settings_list:
            self.robot_state.get_robot(r_id).fetched_settings[key] = "Fetching..."
            packet_sender.send_get_setting(r_id, key)

    def _set_setting(self, key, value_str):
        r_id = self.current_robot_id.get()
        if not r_id:
            messagebox.showwarning("Warning", "Please select a robot first.")
            return
            
        try:
            val = float(value_str)
            packet_sender.send_set_setting(int(r_id), key, val)
            self.frame.after(100, lambda: self._get_setting(key))
        except ValueError:
            messagebox.showerror("Error", f"Invalid numeric value for {key}.")

    def _update_loop(self):
        active_ids = [str(r) for r in self.robot_state.get_all_robot_ids()]
        current_vals = list(self.cb_robots['values'])
        if active_ids != current_vals:
            self.cb_robots['values'] = active_ids
            if active_ids and not self.current_robot_id.get():
                self.cb_robots.current(0)
                
        r_id_str = self.current_robot_id.get()
        if r_id_str:
            robot = self.robot_state.get_robot(int(r_id_str))
            for key, widgets in self.setting_widgets.items():
                lbl = widgets["val_lbl"]
                if key in robot.fetched_settings:
                    val = robot.fetched_settings[key]
                    if isinstance(val, float):
                        lbl.configure(text=f"{val:.4f}", foreground="blue")
                    else:
                        lbl.configure(text=str(val), foreground="#d35400")
                else:
                    lbl.configure(text="Not Loaded", foreground="gray")
        else:
            for widgets in self.setting_widgets.values():
                widgets["val_lbl"].configure(text="Not Loaded", foreground="gray")

        self.frame.after(150, self._update_loop)