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
        
        # Assignment Storage
        self.assignments = {}
        self._last_connection_states = {}
        
        # UI State
        self.selected_controller_id = None
        self.learning_button_map = {} 
        
        self._setup_ui()
        
        if initial_assignments:
            self._load_initial_assignments(initial_assignments)
            
        self.refresh_controllers()
        
        # Select Robot 1 by default
        self._on_robot_select(None)

    def get_frame(self):
        return self.frame

    def _setup_ui(self):
        # =================================================================
        # SECTION 1: GLOBAL ACTIONS (Scan & Save)
        # =================================================================
        global_frame = ttk.Frame(self.frame)
        global_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(global_frame, text="↻ Scan Controllers", command=self.refresh_controllers).pack(side=tk.LEFT, padx=5)
        
        if self.save_callback:
            ttk.Button(global_frame, text="💾 Save All Configs", command=self.save_callback).pack(side=tk.RIGHT, padx=5)

        # =================================================================
        # SECTION 2: ACTIVE ASSIGNMENTS LIST (Master)
        # =================================================================
        list_frame = ttk.LabelFrame(self.frame, text="Active Robot Assignments")
        list_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Treeview
        self.tree = ttk.Treeview(list_frame, columns=("Robot", "Controller", "ID", "Limits"), show="headings", height=4)
        self.tree.heading("Robot", text="Robot ID")
        self.tree.heading("Controller", text="Controller Name")
        self.tree.heading("ID", text="ID")
        self.tree.heading("Limits", text="Max Speed %")
        
        self.tree.column("Robot", width=60)
        self.tree.column("Controller", width=200)
        self.tree.column("ID", width=40)
        self.tree.column("Limits", width=120)
        
        self.tree.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)
        self.tree.bind("<<TreeviewSelect>>", self._on_tree_select)
        
        # Unassign Button (Right of tree)
        btn_frame = tk.Frame(list_frame)
        btn_frame.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Button(btn_frame, text="Delete\nAssignment", command=self._unassign_selected).pack(padx=5, pady=20)

        # =================================================================
        # SECTION 3: EDITOR (Detail) - Setup, Bindings & Limits
        # =================================================================
        edit_frame = ttk.LabelFrame(self.frame, text="Edit Configuration")
        edit_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # --- 3A: Selection Controls ---
        sel_frame = ttk.Frame(edit_frame)
        sel_frame.pack(fill=tk.X, padx=5, pady=10)
        
        # Robot ID
        ttk.Label(sel_frame, text="Target Robot:").pack(side=tk.LEFT, padx=5)
        self.robot_var = tk.IntVar(value=1)
        self.robot_combo = ttk.Combobox(sel_frame, textvariable=self.robot_var, width=5, state="readonly")
        self.robot_combo['values'] = [i for i in range(1, MAX_ROBOTS + 1)]
        self.robot_combo.pack(side=tk.LEFT, padx=5)
        self.robot_combo.bind("<<ComboboxSelected>>", self._on_robot_select)
        
        # Controller
        ttk.Label(sel_frame, text="Link Controller:").pack(side=tk.LEFT, padx=(20, 5))
        self.controller_combo = ttk.Combobox(sel_frame, state="readonly", width=35)
        self.controller_combo.pack(side=tk.LEFT, padx=5)
        self.controller_combo.bind("<<ComboboxSelected>>", self._on_controller_select)

        # --- 3B: Mappings & Limits Grid ---
        map_frame = ttk.Frame(edit_frame)
        map_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        keys = ["vx", "vy", "omega", "estop", "arm"]
        labels = ["Forward/Back", "Left/Right", "Rotate", "E-STOP", "ARM/Disarm"]
        
        # Headers
        ttk.Label(map_frame, text="Function", font=("", 9, "bold")).grid(row=0, column=0, padx=5, sticky="w")
        ttk.Label(map_frame, text="Value", font=("", 9, "bold")).grid(row=0, column=1, padx=5)
        ttk.Label(map_frame, text="Mapped To", font=("", 9, "bold")).grid(row=0, column=2, padx=5)
        ttk.Label(map_frame, text="Action", font=("", 9, "bold")).grid(row=0, column=3, padx=5)
        ttk.Label(map_frame, text="Max Speed %", font=("", 9, "bold")).grid(row=0, column=4, padx=5)
        
        for i, (key, label) in enumerate(zip(keys, labels)):
            row = i + 1
            ttk.Label(map_frame, text=label).grid(row=row, column=0, padx=10, pady=8, sticky="w")
            
            # Value readout
            lbl_val = ttk.Label(map_frame, text="0.00", width=8)
            lbl_val.grid(row=row, column=1, padx=5)
            
            # Current mapping text
            lbl_map = ttk.Label(map_frame, text="--", width=15)
            lbl_map.grid(row=row, column=2, padx=5)
            
            # Bind button
            btn = ttk.Button(map_frame, text="Bind Input", 
                           command=lambda k=key: self._start_learning(k))
            btn.grid(row=row, column=3, padx=5)
            
            # Scale Spinbox
            sb = None
            if key in ["vx", "vy", "omega"]:
                sb = ttk.Spinbox(map_frame, from_=10, to=100, increment=10, width=5)
                sb.set(100)
                sb.grid(row=row, column=4, padx=5)
            
            self.learning_button_map[key] = {
                "val_label": lbl_val,
                "map_label": lbl_map,
                "btn": btn,
                "scale_sb": sb
            }
            
        # --- 3C: Apply Button (Normal style) ---
        # Separator removed
        
        btn_apply = ttk.Button(
            edit_frame, 
            text="Apply Settings to Robot", 
            command=self._assign_controller
        )
        btn_apply.pack(fill=tk.X, padx=20, pady=(10, 10))

    def _load_initial_assignments(self, saved_data):
        default_scales = {'vx': 100, 'vy': 100, 'omega': 100}
        for r_id_str, val in saved_data.items():
            try:
                r_id = int(r_id_str)
                if isinstance(val, dict):
                    self.assignments[r_id] = {
                        'id': None, 
                        'guid': val.get('guid', ''),
                        'name': "Searching...",
                        'scales': val.get('scales', default_scales.copy())
                    }
                else:
                    self.assignments[r_id] = {
                        'id': None, 
                        'guid': str(val),
                        'name': "Searching...",
                        'scales': default_scales.copy()
                    }
            except ValueError:
                pass

    def get_assignment_guids(self):
        data = {}
        for r_id, info in self.assignments.items():
            data[str(r_id)] = {
                'guid': info['guid'],
                'scales': info['scales']
            }
        return data

    def _on_robot_select(self, event):
        """Load settings for the selected robot into the UI"""
        try:
            r_id = self.robot_var.get()
            if r_id in self.assignments:
                scales = self.assignments[r_id]['scales']
                for key, val in scales.items():
                    if key in self.learning_button_map:
                        self.learning_button_map[key]["scale_sb"].set(val)
                
                # Sync Dropdown
                guid = self.assignments[r_id]['guid']
                found = False
                for name, ctrl in self.controller_map.items():
                    if ctrl.guid == guid:
                        self.controller_combo.set(name)
                        self._on_controller_select(None)
                        found = True
                        break
                if not found:
                    self.controller_combo.set("")
            else:
                # Defaults
                for key in ["vx", "vy", "omega"]:
                    self.learning_button_map[key]["scale_sb"].set(100)
        except:
            pass

    def _on_tree_select(self, event):
        """Clicking tree loads that robot into editor"""
        selected_item = self.tree.selection()
        if not selected_item:
            return
        item_vals = self.tree.item(selected_item)['values']
        try:
            r_id_str = str(item_vals[0]).replace("Robot ", "")
            r_id = int(r_id_str)
            self.robot_var.set(r_id)
            self._on_robot_select(None)
        except ValueError:
            pass

    def _assign_controller(self):
        """Apply the settings from the Editor to the Assignment Logic"""
        try:
            r_id = self.robot_var.get()
            c_name = self.controller_combo.get()
            
            # Determine Controller Info
            if not c_name and r_id in self.assignments:
                # Keeping existing controller, just updating limits
                ctrl_id = self.assignments[r_id]['id']
                ctrl_guid = self.assignments[r_id]['guid']
                ctrl_name = self.assignments[r_id]['name']
            elif c_name in self.controller_map:
                # New controller selected
                ctrl = self.controller_map[c_name]
                ctrl_id = ctrl.id
                ctrl_guid = ctrl.guid
                ctrl_name = ctrl.name
            else:
                messagebox.showerror("Error", "Select a valid controller to assign")
                return
            
            # Read scales from UI
            scales = {}
            for key in ["vx", "vy", "omega"]:
                try:
                    val = int(self.learning_button_map[key]["scale_sb"].get())
                    scales[key] = max(0, min(100, val))
                except:
                    scales[key] = 100

            # Remove if controller used elsewhere
            for existing_r, info in list(self.assignments.items()):
                if info['id'] == ctrl_id and existing_r != r_id and ctrl_id is not None:
                    del self.assignments[existing_r]
            
            self.assignments[r_id] = {
                'id': ctrl_id,
                'guid': ctrl_guid,
                'name': ctrl_name,
                'scales': scales
            }
            self._update_assignment_list()
            
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _update_assignment_list(self):
        for item in self.tree.get_children():
            self.tree.delete(item)
        
        for r_id, info in self.assignments.items():
            c_id = info['id']
            is_connected = False
            if c_id is not None and self.manager.get_controller(c_id):
                is_connected = True
                
            self._last_connection_states[r_id] = is_connected

            display_name = info['name'] if is_connected else f"{info.get('name', 'Unknown')} (Disconnected)"
            
            sc = info['scales']
            limits_str = f"F:{sc['vx']}% L:{sc['vy']}% R:{sc['omega']}%"
            
            self.tree.insert("", "end", values=(f"Robot {r_id}", display_name, c_id if c_id is not None else "-", limits_str))

    def refresh_controllers(self):
        controllers = self.manager.get_all()
        values = []
        self.controller_map = {} 
        for c in controllers:
            name = f"{c.name} (ID: {c.id})"
            values.append(name)
            self.controller_map[name] = c
        self.controller_combo['values'] = values
        self._update_assignment_list()
        
        if values:
            if not self.controller_combo.get() in values:
                self.controller_combo.current(0)
                self._on_controller_select(None)
        else:
            self.controller_combo.set("")
            self.selected_controller_id = None

    def _on_controller_select(self, event):
        name = self.controller_combo.get()
        if name in self.controller_map:
            self.selected_controller_id = self.controller_map[name].id
            self._update_mapping_display()

    def _unassign_selected(self):
        selected_item = self.tree.selection()
        if not selected_item:
            return
        item_vals = self.tree.item(selected_item)['values']
        try:
            r_id_str = str(item_vals[0]).replace("Robot ", "")
            r_id = int(r_id_str)
            if r_id in self.assignments:
                del self.assignments[r_id]
                self._update_assignment_list()
                self._on_robot_select(None) 
        except ValueError:
            pass

    def _start_learning(self, key):
        if self.selected_controller_id is None:
            return
        ctrl = self.manager.get_controller(self.selected_controller_id)
        if ctrl:
            ctrl.start_learning(key)
            self.learning_button_map[key]["btn"].configure(text="Press...", state="disabled")

    def check_learning(self):
        if self.selected_controller_id is None:
            return
        ctrl = self.manager.get_controller(self.selected_controller_id)
        if not ctrl:
            return
        success, result, timeout = ctrl.check_learning()
        for key, widgets in self.learning_button_map.items():
            if str(widgets["btn"]['state']) == 'disabled':
                if success or timeout:
                    widgets["btn"].configure(text="Bind Input", state="normal")
                    if success:
                        self._update_mapping_display()
                break

    def _update_mapping_display(self):
        if self.selected_controller_id is None:
            return
        ctrl = self.manager.get_controller(self.selected_controller_id)
        if ctrl:
            for key, widgets in self.learning_button_map.items():
                widgets["map_label"].configure(text=ctrl.get_mapping_text(key))

    def update_display_values(self):
        if self.selected_controller_id is None:
            return
        ctrl = self.manager.get_controller(self.selected_controller_id)
        if ctrl:
            vals = ctrl.get_control_values()
            for key, val in vals.items():
                self.learning_button_map[key]["val_label"].configure(text=f"{val:.2f}")

    def get_active_commands(self):
        self.manager.update()
        commands = {}
        ui_needs_update = False

        for r_id, info in self.assignments.items():
            c_id = info['id']
            ctrl = self.manager.get_controller(c_id)
            is_connected = (ctrl is not None)

            if self._last_connection_states.get(r_id) != is_connected:
                ui_needs_update = True
            
            if is_connected:
                raw_cmd = ctrl.get_control_values()
                scales = info['scales']
                scaled_cmd = raw_cmd.copy()
                scaled_cmd['vx']    = raw_cmd['vx']    * (scales['vx'] / 100.0)
                scaled_cmd['vy']    = raw_cmd['vy']    * (scales['vy'] / 100.0)
                scaled_cmd['omega'] = raw_cmd['omega'] * (scales['omega'] / 100.0)
                
                commands[r_id] = scaled_cmd
                
                if info['name'] != ctrl.name:
                    info['name'] = ctrl.name
                    ui_needs_update = True
            else:
                new_id = self.manager.find_id_by_guid(info['guid'])
                if new_id is not None:
                    print(f"Auto-reconnected Robot {r_id} to ID {new_id}")
                    info['id'] = new_id
                    ui_needs_update = True
        
        if ui_needs_update:
            self._update_assignment_list()
                
        return commands
    
    def get_assignment_names(self):
        names = {}
        for r_id, info in self.assignments.items():
            c_id = info['id']
            if c_id is not None and self.manager.get_controller(c_id):
                names[r_id] = f"{info['name']} ({c_id})"
            else:
                names[r_id] = "Disconnected"
        return names