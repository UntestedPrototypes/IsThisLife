"""
Game controller tab with multi-robot assignment and mapping
"""
import tkinter as tk
from tkinter import ttk, messagebox
import joystick_control
from config import *

class GameControllerTab:
    def __init__(self, notebook, save_callback=None, initial_assignments=None):
        self.frame = ttk.Frame(notebook)
        self.manager = joystick_control.get_manager()
        self.save_callback = save_callback
        self.assignments = {}
        self._last_connection_states = {}
        self.selected_controller_id = None
        self.learning_button_map = {} 
        self._setup_ui()
        if initial_assignments: self._load_initial_assignments(initial_assignments)
        self.refresh_controllers()
        self._on_robot_select(None)

    def get_frame(self): return self.frame
    
    def _setup_ui(self):
        global_frame = ttk.Frame(self.frame); global_frame.pack(fill=tk.X, padx=5, pady=5)
        ttk.Button(global_frame, text="↻ Scan Controllers", command=self.refresh_controllers).pack(side=tk.LEFT, padx=5)
        if self.save_callback: ttk.Button(global_frame, text="💾 Save All Configs", command=self.save_callback).pack(side=tk.RIGHT, padx=5)

        list_frame = ttk.LabelFrame(self.frame, text="Active Robot Assignments")
        list_frame.pack(fill=tk.X, padx=5, pady=5)
        self.tree = ttk.Treeview(list_frame, columns=("Robot", "Controller", "ID", "Limits"), show="headings", height=4)
        self.tree.heading("Robot", text="Robot ID"); self.tree.heading("Controller", text="Controller Name"); self.tree.heading("ID", text="ID"); self.tree.heading("Limits", text="Config")
        self.tree.column("Robot", width=60); self.tree.column("Controller", width=200); self.tree.column("ID", width=40); self.tree.column("Limits", width=180)
        self.tree.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)
        self.tree.bind("<<TreeviewSelect>>", self._on_tree_select)
        
        btn_frame = tk.Frame(list_frame); btn_frame.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Button(btn_frame, text="Delete\nAssignment", command=self._unassign_selected).pack(padx=5, pady=20)

        edit_frame = ttk.LabelFrame(self.frame, text="Edit Configuration")
        edit_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        sel_frame = ttk.Frame(edit_frame); sel_frame.pack(fill=tk.X, padx=5, pady=10)
        
        # Robot Selection
        ttk.Label(sel_frame, text="Target Robot:").pack(side=tk.LEFT, padx=5)
        self.robot_var = tk.StringVar() # Changed from IntVar to StringVar so it can be blank
        self.robot_combo = ttk.Combobox(sel_frame, textvariable=self.robot_var, width=5)
        self.robot_combo.pack(side=tk.LEFT, padx=5)
        self.robot_combo.bind("<<ComboboxSelected>>", self._on_robot_select)

        # Controller Selection
        ttk.Label(sel_frame, text="Link Controller:").pack(side=tk.LEFT, padx=(20, 5))
        self.controller_combo = ttk.Combobox(sel_frame, state="readonly", width=30); self.controller_combo.pack(side=tk.LEFT, padx=5); self.controller_combo.bind("<<ComboboxSelected>>", self._on_controller_select)

        # Global Deadzone
        ttk.Label(sel_frame, text="Deadzone %:").pack(side=tk.LEFT, padx=(20, 5))
        self.deadzone_var = tk.IntVar(value=10)
        self.deadzone_sb = ttk.Spinbox(sel_frame, from_=0, to=50, increment=5, width=5, textvariable=self.deadzone_var)
        self.deadzone_sb.pack(side=tk.LEFT, padx=5)

        map_frame = ttk.Frame(edit_frame); map_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        keys = ["vx", "vy", "omega", "estop", "arm", "cruise_control", "cruise_state"]
        labels = ["Forward/Back", "Left/Right", "Rotate", "E-STOP", "ARM/Disarm", "Cruise-Control Btn", "Cruise-Control State"]
        ttk.Label(map_frame, text="Function", font=("", 9, "bold")).grid(row=0, column=0, padx=5, sticky="w")
        ttk.Label(map_frame, text="Value", font=("", 9, "bold")).grid(row=0, column=1, padx=5)
        ttk.Label(map_frame, text="Mapped To", font=("", 9, "bold")).grid(row=0, column=2, padx=5)
        ttk.Label(map_frame, text="Action", font=("", 9, "bold")).grid(row=0, column=3, padx=5)
        ttk.Label(map_frame, text="Max Speed %", font=("", 9, "bold")).grid(row=0, column=4, padx=5)
        
        for i, (key, label) in enumerate(zip(keys, labels)):
            row = i + 1
            ttk.Label(map_frame, text=label).grid(row=row, column=0, padx=10, pady=8, sticky="w")
            lbl_val = ttk.Label(map_frame, text="0.00", width=8); lbl_val.grid(row=row, column=1, padx=5)
            lbl_map = ttk.Label(map_frame, text="--", width=15); lbl_map.grid(row=row, column=2, padx=5)
            
            btn = ttk.Button(map_frame, text="Bind Input", command=lambda k=key: self._start_learning(k))
            
            # Hide the bind button and mapping text for the state display
            if key == "cruise_state":
                lbl_map.configure(text="N/A")
                btn.grid_remove() 
            else:
                btn.grid(row=row, column=3, padx=5)
                
            sb = None
            if key in ["vx", "vy", "omega"]: sb = ttk.Spinbox(map_frame, from_=10, to=100, increment=10, width=5); sb.set(100); sb.grid(row=row, column=4, padx=5)
            self.learning_button_map[key] = {"val_label": lbl_val, "map_label": lbl_map, "btn": btn, "scale_sb": sb}
            
        btn_apply = ttk.Button(edit_frame, text="Apply Settings to Robot", command=self._assign_controller); btn_apply.pack(fill=tk.X, padx=20, pady=(10, 10))

    def update_available_robots(self, robot_ids):
        current = self.robot_combo.get()
        assigned = list(self.assignments.keys())
        all_ids = sorted(list(set(robot_ids + assigned))) # Removed hardcoded + [1]
        
        current_values = list(self.robot_combo['values'])
        try:
            current_values = [int(x) for x in current_values]
        except:
            pass
            
        if current_values != all_ids:
            self.robot_combo['values'] = all_ids
            if not all_ids:
                self.robot_combo.set("")
            elif not current or current not in [str(x) for x in all_ids]: 
                self.robot_combo.current(0)
                self._on_robot_select(None)

    def _load_initial_assignments(self, saved_data):
        default_scales = {'vx': 100, 'vy': 100, 'omega': 100}
        for r_id_str, val in saved_data.items():
            try:
                r_id = int(r_id_str)
                idx = val.get('instance_index', 0) if isinstance(val, dict) else 0
                
                if isinstance(val, dict):
                    self.assignments[r_id] = {
                        'id': None, 'guid': val.get('guid', ''), 
                        'instance_index': idx, 
                        'name': "Searching...", 
                        'scales': val.get('scales', default_scales.copy()),
                        'deadzone': val.get('deadzone', 10)
                    }
                else:
                    self.assignments[r_id] = {
                        'id': None, 'guid': str(val), 
                        'instance_index': 0, 
                        'name': "Searching...", 
                        'scales': default_scales.copy(),
                        'deadzone': 10
                    }
            except ValueError: pass

    def get_assignment_guids(self):
        data = {}
        for r_id, info in self.assignments.items():
            data[str(r_id)] = {
                'guid': info['guid'],
                'instance_index': info['instance_index'],
                'scales': info['scales'],
                'deadzone': info['deadzone']
            }
        return data

    def _assign_controller(self):
        try:
            r_val = self.robot_var.get()
            if not r_val:
                messagebox.showerror("Error", "Please enter a valid Target Robot ID")
                return
            r_id = int(r_val)
            
            c_name = self.controller_combo.get()
            if not c_name and r_id in self.assignments:
                ctrl_id = self.assignments[r_id]['id']
                ctrl_guid = self.assignments[r_id]['guid']
                ctrl_name = self.assignments[r_id]['name']
                idx = self.assignments[r_id]['instance_index']
            elif c_name in self.controller_map:
                ctrl = self.controller_map[c_name]
                ctrl_id = ctrl.id
                ctrl_guid = ctrl.guid
                ctrl_name = ctrl.name
                match_count = 0
                sorted_ids = sorted(self.manager.controllers.keys())
                for jid in sorted_ids:
                    c = self.manager.controllers[jid]
                    if c.guid == ctrl_guid:
                        if c.id == ctrl_id:
                            idx = match_count
                            break
                        match_count += 1
            else:
                messagebox.showerror("Error", "Select a valid controller"); return
            
            scales = {}
            for key in ["vx", "vy", "omega"]:
                try: scales[key] = max(0, min(100, int(self.learning_button_map[key]["scale_sb"].get())))
                except: scales[key] = 100
                
            dz = 10
            try: dz = max(0, min(50, int(self.deadzone_var.get())))
            except: pass
                
            self.assignments[r_id] = {
                'id': ctrl_id, 'guid': ctrl_guid, 
                'instance_index': idx, 
                'name': ctrl_name, 'scales': scales,
                'deadzone': dz
            }
            self._update_assignment_list()
        except Exception as e: messagebox.showerror("Error", str(e))

    def refresh_controllers(self):
        controllers = self.manager.get_all()
        values = [f"{c.name} (ID: {c.id})" for c in controllers]
        self.controller_map = {f"{c.name} (ID: {c.id})": c for c in controllers}
        self.controller_combo['values'] = values
        self._update_assignment_list()
        if values and self.controller_combo.get() not in values: self.controller_combo.current(0); self._on_controller_select(None)
        elif not values: self.controller_combo.set(""); self.selected_controller_id = None

    def _on_controller_select(self, event):
        name = self.controller_combo.get()
        if name in self.controller_map: self.selected_controller_id = self.controller_map[name].id; self._update_mapping_display()

    def _unassign_selected(self):
        selected = self.tree.selection()
        if selected:
            try:
                r_id = int(str(self.tree.item(selected)['values'][0]).replace("Robot ", ""))
                if r_id in self.assignments: del self.assignments[r_id]; self._update_assignment_list(); self._on_robot_select(None)
            except: pass

    def _start_learning(self, key):
        if self.selected_controller_id is None: return
        ctrl = self.manager.get_controller(self.selected_controller_id);
        if ctrl: ctrl.start_learning(key); self.learning_button_map[key]["btn"].configure(text="Press...", state="disabled")

    def check_learning(self):
        if self.selected_controller_id is None: return
        ctrl = self.manager.get_controller(self.selected_controller_id)
        if not ctrl: return
        success, result, timeout = ctrl.check_learning()
        for key, widgets in self.learning_button_map.items():
            if key == "cruise_state": continue # Skip state parameter
            if str(widgets["btn"]['state']) == 'disabled':
                if success or timeout: widgets["btn"].configure(text="Bind Input", state="normal"); 
                if success: self._update_mapping_display()
                break

    def _update_mapping_display(self):
        if self.selected_controller_id is None: return
        ctrl = self.manager.get_controller(self.selected_controller_id)
        if ctrl: 
            for key, widgets in self.learning_button_map.items(): 
                if key == "cruise_state":
                    widgets["map_label"].configure(text="N/A")
                    continue
                try:
                    widgets["map_label"].configure(text=ctrl.get_mapping_text(key))
                except Exception:
                    widgets["map_label"].configure(text="--")

    def update_display_values(self):
        if self.selected_controller_id is None: return
        ctrl = self.manager.get_controller(self.selected_controller_id)
        if ctrl:
            try: dz = max(0, min(50, int(self.deadzone_var.get()))) / 100.0
            except: dz = 0.10
            
            vals = ctrl.get_control_values(deadzone=dz)
            for key, val in vals.items():
                if key in self.learning_button_map: 
                    # Format correctly depending on if it's a number or a state string
                    if isinstance(val, str):
                        self.learning_button_map[key]["val_label"].configure(text=val)
                    else:
                        self.learning_button_map[key]["val_label"].configure(text=f"{val:.2f}")

    def _on_robot_select(self, event):
        try:
            r_val = self.robot_var.get()
            if not r_val: return
            r_id = int(r_val)
            
            if r_id in self.assignments:
                scales = self.assignments[r_id]['scales']
                for key, val in scales.items():
                    if key in self.learning_button_map and self.learning_button_map[key]["scale_sb"]: self.learning_button_map[key]["scale_sb"].set(val)
                
                self.deadzone_var.set(self.assignments[r_id].get('deadzone', 10))
                
                guid = self.assignments[r_id]['guid']
                found = False
                for name, ctrl in self.controller_map.items():
                    if ctrl.guid == guid:
                        self.controller_combo.set(name); self._on_controller_select(None); found = True; break
                if not found: self.controller_combo.set("")
            else:
                for key in ["vx", "vy", "omega"]:
                    if self.learning_button_map[key]["scale_sb"]: self.learning_button_map[key]["scale_sb"].set(100)
                self.deadzone_var.set(10)
                self.controller_combo.set("") # Clear controller box if no assignment exists
        except: pass

    def _on_tree_select(self, event):
        selected_item = self.tree.selection()
        if not selected_item: return
        try:
            r_id = int(str(self.tree.item(selected_item)['values'][0]).replace("Robot ", ""))
            self.robot_var.set(str(r_id))
            self._on_robot_select(None)
        except ValueError: pass

    def _update_assignment_list(self):
        for item in self.tree.get_children(): self.tree.delete(item)
        for r_id, info in self.assignments.items():
            c_id = info['id']
            # FIX: Force evaluation to strict Boolean to prevent infinite UI redraw loops
            is_connected = bool(c_id is not None and self.manager.get_controller(c_id) is not None)
            
            self._last_connection_states[r_id] = is_connected
            display_name = info['name'] if is_connected else f"{info.get('name', 'Unknown')} (Disconnected)"
            sc = info['scales']
            dz = info.get('deadzone', 10)
            limits_str = f"Spd: {sc['vx']}% | DZ: {dz}%"
            self.tree.insert("", "end", values=(f"Robot {r_id}", display_name, c_id if c_id is not None else "-", limits_str))

    def get_assignment_names(self):
        names = {}
        for r_id, info in self.assignments.items():
            c_id = info['id']
            if c_id is not None and self.manager.get_controller(c_id): names[r_id] = f"{info['name']} ({c_id})"
            else: names[r_id] = "Disconnected"
        return names

    def get_active_commands(self):
        self.manager.update()
        commands = {}
        ui_needs_update = False
        
        for r_id, info in self.assignments.items():
            c_id = info['id']
            ctrl = self.manager.get_controller(c_id)
            is_connected = bool(ctrl is not None)

            if self._last_connection_states.get(r_id) != is_connected: ui_needs_update = True
            
            if is_connected:
                dz = info.get('deadzone', 10) / 100.0
                raw_cmd = ctrl.get_control_values(deadzone=dz)
                
                scales = info['scales']
                scaled_cmd = raw_cmd.copy()
                scaled_cmd['vx'] *= (scales['vx'] / 100.0)
                scaled_cmd['vy'] *= (scales['vy'] / 100.0)
                scaled_cmd['omega'] *= (scales['omega'] / 100.0)
                commands[r_id] = scaled_cmd
                
                if info['name'] != ctrl.name: info['name'] = ctrl.name; ui_needs_update = True
            else:
                idx = info.get('instance_index', 0)
                new_id = self.manager.find_id_by_guid(info['guid'], instance_index=idx)
                
                if new_id is not None:
                    print(f"Auto-reconnected Robot {r_id} to Controller {new_id} (Idx {idx})")
                    info['id'] = new_id
                    ui_needs_update = True
        
        if ui_needs_update: self._update_assignment_list()
        return commands