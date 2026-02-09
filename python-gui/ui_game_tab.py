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
        
        # Assignment Storage:
        # { robot_id: {'id': instance_id, 'guid': guid_string, 'name': name} }
        self.assignments = {} 
        
        # Load initial assignments passed from Dashboard
        if initial_assignments:
            self._load_initial_assignments(initial_assignments)
        
        # UI State
        self.selected_controller_id = None
        self.learning_button_map = {} 
        
        self._setup_ui()
        self.refresh_controllers()

    def get_frame(self):
        return self.frame

    def _setup_ui(self):
        # --- Top: Assignment Section ---
        assign_frame = ttk.LabelFrame(self.frame, text="Controller Assignment")
        assign_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Robot Selector
        ttk.Label(assign_frame, text="Robot ID:").pack(side=tk.LEFT, padx=5)
        self.robot_var = tk.IntVar(value=1)
        self.robot_combo = ttk.Combobox(assign_frame, textvariable=self.robot_var, width=5)
        self.robot_combo['values'] = [i for i in range(1, MAX_ROBOTS + 1)]
        self.robot_combo.pack(side=tk.LEFT, padx=5)
        
        # Controller Selector
        ttk.Label(assign_frame, text="Controller:").pack(side=tk.LEFT, padx=5)
        self.controller_combo = ttk.Combobox(assign_frame, state="readonly", width=30)
        self.controller_combo.pack(side=tk.LEFT, padx=5)
        self.controller_combo.bind("<<ComboboxSelected>>", self._on_controller_select)
        
        # Buttons
        ttk.Button(assign_frame, text="Assign", command=self._assign_controller).pack(side=tk.LEFT, padx=5)
        ttk.Button(assign_frame, text="↻ Scan", command=self.refresh_controllers).pack(side=tk.LEFT, padx=5)
        
        # Save Button (Calls the global save callback)
        if self.save_callback:
            ttk.Button(assign_frame, text="💾 Save Config", command=self.save_callback).pack(side=tk.RIGHT, padx=10)

        # --- Middle: Active Assignments List ---
        list_frame = ttk.LabelFrame(self.frame, text="Active Assignments")
        list_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.tree = ttk.Treeview(list_frame, columns=("Robot", "Controller", "ID", "GUID"), show="headings", height=4)
        self.tree.heading("Robot", text="Robot ID")
        self.tree.heading("Controller", text="Controller Name")
        self.tree.heading("ID", text="ID")
        self.tree.heading("GUID", text="GUID ( partial )")
        
        self.tree.column("Robot", width=60)
        self.tree.column("Controller", width=200)
        self.tree.column("ID", width=40)
        self.tree.column("GUID", width=100)
        
        self.tree.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)
        
        # Unassign Button
        btn_frame = tk.Frame(list_frame)
        btn_frame.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Button(btn_frame, text="Unassign\nSelected", command=self._unassign_selected).pack(padx=5, pady=20)

        # --- Bottom: Mapping & Visuals ---
        map_frame = ttk.LabelFrame(self.frame, text="Button Mapping (Select Controller Above)")
        map_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        keys = ["vx", "vy", "omega", "estop", "arm"]
        labels = ["Forward/Back", "Left/Right", "Rotate", "E-STOP", "ARM/Disarm"]
        
        for i, (key, label) in enumerate(zip(keys, labels)):
            ttk.Label(map_frame, text=label).grid(row=i, column=0, padx=10, pady=5, sticky="w")
            
            lbl_val = ttk.Label(map_frame, text="0.00", width=8)
            lbl_val.grid(row=i, column=1, padx=5)
            
            lbl_map = ttk.Label(map_frame, text="--", width=15)
            lbl_map.grid(row=i, column=2, padx=5)
            
            # Button text changed to "Bind"
            btn = ttk.Button(map_frame, text="Bind", 
                           command=lambda k=key: self._start_learning(k))
            btn.grid(row=i, column=3, padx=5)
            
            self.learning_button_map[key] = {
                "val_label": lbl_val,
                "map_label": lbl_map,
                "btn": btn
            }

    def _load_initial_assignments(self, saved_guids):
        """Load assignments passed from config"""
        for r_id_str, guid in saved_guids.items():
            try:
                self.assignments[int(r_id_str)] = {
                    'id': None, 
                    'guid': guid,
                    'name': "Searching..."
                }
            except ValueError:
                pass

    def get_assignment_guids(self):
        """Return dict of {robot_id: guid} for saving"""
        data = {}
        for r_id, info in self.assignments.items():
            data[str(r_id)] = info['guid']
        return data

    def refresh_controllers(self):
        """Scan for controllers and update dropdown"""
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
            self.controller_combo.set("No controllers found")
            self.selected_controller_id = None

    def _on_controller_select(self, event):
        name = self.controller_combo.get()
        if name in self.controller_map:
            self.selected_controller_id = self.controller_map[name].id
            self._update_mapping_display()

    def _assign_controller(self):
        try:
            r_id = self.robot_var.get()
            c_name = self.controller_combo.get()
            
            if c_name not in self.controller_map:
                messagebox.showerror("Error", "Select a valid controller")
                return

            ctrl = self.controller_map[c_name]
            
            for existing_r, info in list(self.assignments.items()):
                if info['id'] == ctrl.id and existing_r != r_id:
                    del self.assignments[existing_r]
            
            self.assignments[r_id] = {
                'id': ctrl.id,
                'guid': ctrl.guid,
                'name': ctrl.name
            }
            self._update_assignment_list()
            
        except Exception as e:
            messagebox.showerror("Error", str(e))

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
        except ValueError:
            pass

    def _update_assignment_list(self):
        for item in self.tree.get_children():
            self.tree.delete(item)
        
        for r_id, info in self.assignments.items():
            c_id = info['id']
            is_connected = False
            if c_id is not None and self.manager.get_controller(c_id):
                is_connected = True
                
            display_name = info['name'] if is_connected else f"{info.get('name', 'Unknown')} (Disconnected)"
            guid_short = str(info['guid'])[:8] + "..."
            
            self.tree.insert("", "end", values=(f"Robot {r_id}", display_name, c_id if c_id is not None else "-", guid_short))

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
                    # Reset text to "Bind" instead of "Learn"
                    widgets["btn"].configure(text="Bind", state="normal")
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
        for r_id, info in self.assignments.items():
            c_id = info['id']
            ctrl = self.manager.get_controller(c_id)
            
            if ctrl:
                commands[r_id] = ctrl.get_control_values()
                if info['name'] != ctrl.name:
                    info['name'] = ctrl.name
                    self._update_assignment_list()
            else:
                new_id = self.manager.find_id_by_guid(info['guid'])
                if new_id is not None:
                    print(f"Auto-reconnected Robot {r_id} to ID {new_id}")
                    info['id'] = new_id
                    new_ctrl = self.manager.get_controller(new_id)
                    commands[r_id] = new_ctrl.get_control_values()
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