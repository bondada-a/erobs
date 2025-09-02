#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, messagebox, filedialog, scrolledtext
import json
import threading
import time
import sys
import os
import subprocess
import tempfile

# Import local modules
try:
    from pose_editor import PoseManager
    from poses_manager import PosesManager
except ImportError:
    print("Warning: Local modules not found. Running with limited functionality.")
    # Fallback if modules not found
    PoseManager = None
    PosesManager = None

class MTCGUIClient:
    """Working MTC GUI Client that communicates with the actual MTC action server"""
    
    def __init__(self):
        self.name = 'mtc_gui_client'
        self.logger = type('Logger', (), {'info': print, 'error': print, 'warn': print})()
        
        # GUI state
        self.current_goal_handle = None
        self.execution_thread = None
        self.stop_execution = False
        self.temp_json_file = None
        
        # Create GUI
        self.setup_gui()
        
        # Load default configuration
        self.load_default_config()

    def setup_gui(self):
        """Setup the main GUI window"""
        self.root = tk.Tk()
        self.root.title("MTC Action Client GUI - Working Version")
        self.root.geometry("1000x800")
        
        # Configure grid weights
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=1)
        
        self.create_menu()
        self.create_robot_config_frame()
        self.create_task_editor_frame()
        self.create_execution_frame()
        self.create_status_frame()

    def create_menu(self):
        """Create menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)
        
        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Load JSON", command=self.load_json_file)
        file_menu.add_command(label="Save JSON", command=self.save_json_file)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)
        
        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)

    def create_robot_config_frame(self):
        """Create robot configuration frame"""
        config_frame = ttk.LabelFrame(self.root, text="Robot Configuration", padding="10")
        config_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=5)
        config_frame.grid_columnconfigure(1, weight=1)
        
        # Robot IP
        ttk.Label(config_frame, text="Robot IP:").grid(row=0, column=0, sticky="w", padx=(0, 10))
        self.robot_ip_var = tk.StringVar(value="192.168.1.101")
        self.robot_ip_entry = ttk.Entry(config_frame, textvariable=self.robot_ip_var, width=20)
        self.robot_ip_entry.grid(row=0, column=1, sticky="ew", padx=(0, 10))
        
        # Start Gripper
        ttk.Label(config_frame, text="Start Gripper:").grid(row=0, column=2, sticky="w", padx=(20, 10))
        self.start_gripper_var = tk.StringVar(value="epick")
        gripper_combo = ttk.Combobox(config_frame, textvariable=self.start_gripper_var, 
                                    values=["epick", "hande", "none"], width=10)
        gripper_combo.grid(row=0, column=3, sticky="w")
        
        # Test Connection Button
        ttk.Button(config_frame, text="Test MTC Server", 
                  command=self.test_mtc_server).grid(row=0, column=4, padx=(20, 0))
        
        # Manage Poses Button
        if PosesManager:
            ttk.Button(config_frame, text="Manage Poses", 
                      command=self.manage_poses).grid(row=0, column=5, padx=(20, 0))

    def create_task_editor_frame(self):
        """Create task sequence editor frame"""
        editor_frame = ttk.LabelFrame(self.root, text="Task Sequence Editor", padding="10")
        editor_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)
        editor_frame.grid_columnconfigure(0, weight=1)
        editor_frame.grid_rowconfigure(1, weight=1)
        
        # Toolbar
        toolbar = ttk.Frame(editor_frame)
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        
        ttk.Button(toolbar, text="Add MoveTo", command=lambda: self.add_task_step("moveto")).pack(side="left", padx=(0, 5))
        ttk.Button(toolbar, text="Add Pick&Place", command=lambda: self.add_task_step("pick_and_place")).pack(side="left", padx=(0, 5))
        ttk.Button(toolbar, text="Add Tool Exchange", command=lambda: self.add_task_step("tool_exchange")).pack(side="left", padx=(0, 5))
        ttk.Button(toolbar, text="Add End Effector", command=lambda: self.add_task_step("end_effector")).pack(side="left", padx=(0, 5))
        ttk.Button(toolbar, text="Remove Step", command=self.remove_task_step).pack(side="left", padx=(20, 0))
        
        # Task sequence tree
        self.task_tree = ttk.Treeview(editor_frame, columns=("Action", "Details"), show="tree headings")
        self.task_tree.heading("#0", text="Step")
        self.task_tree.heading("Action", text="Action")
        self.task_tree.heading("Details", text="Details")
        self.task_tree.column("#0", width=60)
        self.task_tree.column("Action", width=150)
        self.task_tree.column("Details", width=300)
        self.task_tree.grid(row=1, column=0, sticky="nsew")
        
        # Scrollbar for task tree
        task_scrollbar = ttk.Scrollbar(editor_frame, orient="vertical", command=self.task_tree.yview)
        task_scrollbar.grid(row=1, column=1, sticky="ns")
        self.task_tree.configure(yscrollcommand=task_scrollbar.set)
        
        # Bind double-click to edit
        self.task_tree.bind("<Double-1>", self.edit_task_step)

    def create_execution_frame(self):
        """Create execution control frame"""
        exec_frame = ttk.LabelFrame(self.root, text="Execution Control", padding="10")
        exec_frame.grid(row=2, column=0, sticky="ew", padx=10, pady=5)
        exec_frame.grid_columnconfigure(1, weight=1)
        
        # Execute button
        self.execute_btn = ttk.Button(exec_frame, text="Execute Task", command=self.execute_task)
        self.execute_btn.grid(row=0, column=0, padx=(0, 10))
        
        # Stop button
        self.stop_btn = ttk.Button(exec_frame, text="Stop Execution", command=self.stop_task, state="disabled")
        self.stop_btn.grid(row=0, column=1, padx=(0, 10))
        
        # Progress bar
        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(exec_frame, variable=self.progress_var, maximum=100)
        self.progress_bar.grid(row=0, column=2, sticky="ew", padx=(10, 0))

    def create_status_frame(self):
        """Create status display frame"""
        status_frame = ttk.LabelFrame(self.root, text="Status & Logs", padding="10")
        status_frame.grid(row=3, column=0, sticky="nsew", padx=10, pady=5)
        status_frame.grid_columnconfigure(0, weight=1)
        status_frame.grid_rowconfigure(0, weight=1)
        
        # Status text area
        self.status_text = scrolledtext.ScrolledText(status_frame, height=8, wrap=tk.WORD)
        self.status_text.grid(row=0, column=0, sticky="nsew")
        
        # Clear button
        ttk.Button(status_frame, text="Clear Logs", command=self.clear_logs).grid(row=1, column=0, pady=(5, 0))

    def load_default_config(self):
        """Load default task configuration"""
        default_config = {
            "start_gripper": "epick",
            "poses": {
                "home": [0.0, -90.0, -90.0, -90.0, 90.0, 0.0],
                "pickup_approach": [45.0, -90.0, -90.0, -90.0, 90.0, -90.0],
                "pickup": [45.0, -90.0, -90.0, -90.0, 90.0, 0.0],
                "place_approach": [-45.0, -90.0, -90.0, -90.0, 90.0, 0.0],
                "place": [-45.0, -90.0, -90.0, -90.0, 90.0, 0.0]
            },
            "sequence": []
        }
        
        self.current_config = default_config
        self.update_task_tree()
    
    def manage_poses(self):
        """Open poses management dialog"""
        if not PosesManager:
            messagebox.showwarning("Warning", "Poses manager not available")
            return
        
        try:
            manager = PosesManager(self.root, self.current_config.get("poses", {}))
            result = manager.show()
            
            if result is not None:
                self.current_config["poses"] = result
                self.log_message(f"Updated poses configuration ({len(result)} poses)")
                
        except Exception as e:
            messagebox.showerror("Error", f"Failed to open poses manager: {str(e)}")

    def add_task_step(self, action_type):
        """Add a new task step"""
        step_id = len(self.current_config["sequence"]) + 1
        
        if action_type == "moveto":
            step = {
                "action": "moveto",
                "target_type": "pose",
                "target": "home",
                "planning_type": "joint",
                "arm_group": "ur_arm"
            }
        elif action_type == "pick_and_place":
            step = {
                "action": "pick_and_place",
                "gripper": "epick",
                "pickup_poses": ["pickup_approach", "pickup"],
                "place_poses": ["place_approach", "place"]
            }
        elif action_type == "tool_exchange":
            step = {
                "action": "tool_exchange",
                "operation": "load",
                "gripper": "hande",
                "dock_number": 3,
                "poses": ["load_approach"]
            }
        elif action_type == "end_effector":
            step = {
                "action": "end_effector",
                "end_effector_type": "epick",
                "end_effector_action": "vacuum_on"
            }
        
        self.current_config["sequence"].append(step)
        self.update_task_tree()
        self.log_message(f"Added {action_type} step")

    def remove_task_step(self):
        """Remove selected task step"""
        selection = self.task_tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a step to remove")
            return
        
        item = selection[0]
        step_index = int(item) - 1
        
        if 0 <= step_index < len(self.current_config["sequence"]):
            removed_step = self.current_config["sequence"].pop(step_index)
            self.update_task_tree()
            self.log_message(f"Removed step: {removed_step['action']}")

    def edit_task_step(self, event):
        """Edit selected task step"""
        selection = self.task_tree.selection()
        if not selection:
            return
        
        item = selection[0]
        step_index = int(item) - 1
        
        if 0 <= step_index < len(self.current_config["sequence"]):
            self.edit_step_dialog(step_index)

    def edit_step_dialog(self, step_index):
        """Open dialog to edit a task step"""
        step = self.current_config["sequence"][step_index]
        
        dialog = tk.Toplevel(self.root)
        dialog.title(f"Edit Step {step_index + 1}")
        dialog.geometry("500x400")
        dialog.transient(self.root)
        try:
            dialog.grab_set()
        except:
            pass  # Ignore grab errors
        
        # Create form fields based on step type
        if step["action"] == "moveto":
            self.create_moveto_edit_form(dialog, step, step_index)
        elif step["action"] == "pick_and_place":
            self.create_pickplace_edit_form(dialog, step, step_index)
        elif step["action"] == "tool_exchange":
            self.create_toolexchange_edit_form(dialog, step, step_index)
        elif step["action"] == "end_effector":
            self.create_end_effector_edit_form(dialog, step, step_index)

    def create_moveto_edit_form(self, dialog, step, step_index):
        """Create edit form for moveto steps"""
        ttk.Label(dialog, text="MoveTo Configuration", font=("Arial", 12, "bold")).pack(pady=10)
        
        # Target pose
        ttk.Label(dialog, text="Target Pose:").pack(anchor="w", padx=20)
        target_var = tk.StringVar(value=step.get("target", "home"))
        target_entry = ttk.Entry(dialog, textvariable=target_var, width=30)
        target_entry.pack(padx=20, pady=(0, 10))
        
        # Planning type
        ttk.Label(dialog, text="Planning Type:").pack(anchor="w", padx=20)
        planning_var = tk.StringVar(value=step.get("planning_type", "joint"))
        planning_combo = ttk.Combobox(dialog, textvariable=planning_var, 
                                    values=["joint", "cartesian"], width=30)
        planning_combo.pack(padx=20, pady=(0, 10))
        
        # Arm group
        ttk.Label(dialog, text="Arm Group:").pack(anchor="w", padx=20)
        arm_group_var = tk.StringVar(value=step.get("arm_group", "ur_arm"))
        arm_group_entry = ttk.Entry(dialog, textvariable=arm_group_var, width=30)
        arm_group_entry.pack(padx=20, pady=(0, 20))
        
        def save_changes():
            step["target"] = target_var.get()
            step["planning_type"] = planning_var.get()
            step["arm_group"] = arm_group_var.get()
            self.update_task_tree()
            dialog.destroy()
            self.log_message(f"Updated step {step_index + 1}")
        
        ttk.Button(dialog, text="Save", command=save_changes).pack(pady=10)

    def create_pickplace_edit_form(self, dialog, step, step_index):
        """Create edit form for pick and place steps"""
        ttk.Label(dialog, text="Pick & Place Configuration", font=("Arial", 12, "bold")).pack(pady=10)
        
        # Gripper
        ttk.Label(dialog, text="Gripper:").pack(anchor="w", padx=20)
        gripper_var = tk.StringVar(value=step.get("gripper", "epick"))
        gripper_combo = ttk.Combobox(dialog, textvariable=gripper_var, 
                                    values=["epick", "hande"], width=30)
        gripper_combo.pack(padx=20, pady=(0, 10))
        
        # Pickup poses
        ttk.Label(dialog, text="Pickup Poses (comma-separated):").pack(anchor="w", padx=20)
        pickup_var = tk.StringVar(value=", ".join(step.get("pickup_poses", [])))
        pickup_entry = ttk.Entry(dialog, textvariable=pickup_var, width=30)
        pickup_entry.pack(padx=20, pady=(0, 10))
        
        # Place poses
        ttk.Label(dialog, text="Place Poses (comma-separated):").pack(anchor="w", padx=20)
        place_var = tk.StringVar(value=", ".join(step.get("place_poses", [])))
        place_entry = ttk.Entry(dialog, textvariable=place_var, width=30)
        place_entry.pack(padx=20, pady=(0, 20))
        
        def save_changes():
            step["gripper"] = gripper_var.get()
            step["pickup_poses"] = [p.strip() for p in pickup_var.get().split(",")]
            step["place_poses"] = [p.strip() for p in place_var.get().split(",")]
            self.update_task_tree()
            dialog.destroy()
            self.log_message(f"Updated step {step_index + 1}")
        
        ttk.Button(dialog, text="Save", command=save_changes).pack(pady=10)

    def create_toolexchange_edit_form(self, dialog, step, step_index):
        """Create edit form for tool exchange steps"""
        ttk.Label(dialog, text="Tool Exchange Configuration", font=("Arial", 12, "bold")).pack(pady=10)
        
        # Operation
        ttk.Label(dialog, text="Operation:").pack(anchor="w", padx=20)
        operation_var = tk.StringVar(value=step.get("operation", "load"))
        operation_combo = ttk.Combobox(dialog, textvariable=operation_var, 
                                     values=["load", "dock"], width=30)
        operation_combo.pack(padx=20, pady=(0, 10))
        
        # Gripper
        ttk.Label(dialog, text="Gripper:").pack(anchor="w", padx=20)
        gripper_var = tk.StringVar(value=step.get("gripper", "hande"))
        gripper_combo = ttk.Combobox(dialog, textvariable=gripper_var, 
                                    values=["epick", "hande"], width=30)
        gripper_combo.pack(padx=20, pady=(0, 10))
        
        # Dock number
        ttk.Label(dialog, text="Dock Number:").pack(anchor="w", padx=20)
        dock_var = tk.StringVar(value=str(step.get("dock_number", 3)))
        dock_entry = ttk.Entry(dialog, textvariable=dock_var, width=30)
        dock_entry.pack(padx=20, pady=(0, 20))
        
        def save_changes():
            step["operation"] = operation_var.get()
            step["gripper"] = gripper_var.get()
            step["dock_number"] = int(dock_var.get())
            self.update_task_tree()
            dialog.destroy()
            self.log_message(f"Updated step {step_index + 1}")
        
        ttk.Button(dialog, text="Save", command=save_changes).pack(pady=10)

    def create_end_effector_edit_form(self, dialog, step, step_index):
        """Create edit form for end effector steps"""
        ttk.Label(dialog, text="End Effector Configuration", font=("Arial", 12, "bold")).pack(pady=10)
        
        # End effector type
        ttk.Label(dialog, text="End Effector Type:").pack(anchor="w", padx=20)
        type_var = tk.StringVar(value=step.get("end_effector_type", "epick"))
        type_combo = ttk.Combobox(dialog, textvariable=type_var, 
                                 values=["epick", "hande"], width=30)
        type_combo.pack(padx=20, pady=(0, 10))
        
        # Action
        ttk.Label(dialog, text="Action:").pack(anchor="w", padx=20)
        action_var = tk.StringVar(value=step.get("end_effector_action", "vacuum_on"))
        action_combo = ttk.Combobox(dialog, textvariable=action_var, 
                                  values=["vacuum_on", "vacuum_off", "open", "close"], width=30)
        action_combo.pack(padx=20, pady=(0, 20))
        
        def save_changes():
            step["end_effector_type"] = type_var.get()
            step["end_effector_action"] = action_var.get()
            self.update_task_tree()
            dialog.destroy()
            self.log_message(f"Updated step {step_index + 1}")
        
        ttk.Button(dialog, text="Save", command=save_changes).pack(pady=10)

    def update_task_tree(self):
        """Update the task sequence tree display"""
        # Clear existing items
        for item in self.task_tree.get_children():
            self.task_tree.delete(item)
        
        # Add current sequence
        for i, step in enumerate(self.current_config["sequence"]):
            step_id = str(i + 1)
            action = step.get("action", "unknown")
            
            # Create details string
            details = ""
            if action == "moveto":
                details = f"Move to {step.get('target', 'unknown')}"
            elif action == "pick_and_place":
                details = f"Pick & Place with {step.get('gripper', 'unknown')}"
            elif action == "tool_exchange":
                details = f"{step.get('operation', 'unknown')} {step.get('gripper', 'unknown')}"
            elif action == "end_effector":
                details = f"{step.get('end_effector_type', 'unknown')} {step.get('end_effector_action', 'unknown')}"
            
            self.task_tree.insert("", "end", iid=step_id, text=step_id, 
                                values=(action, details))

    def execute_task(self):
        """Execute the current task configuration using the MTC action client"""
        if not self.current_config["sequence"]:
            messagebox.showwarning("Warning", "No task sequence defined")
            return
        
        # Validate configuration
        validation_result = self.validate_configuration()
        if not validation_result["valid"]:
            messagebox.showerror("Configuration Error", validation_result["message"])
            return
        
        # Update configuration with current GUI values
        self.current_config["start_gripper"] = self.start_gripper_var.get()
        
        # Start execution in background thread
        self.execution_thread = threading.Thread(target=self._execute_task_thread, daemon=True)
        self.execution_thread.start()
        
        # Update GUI state
        self.execute_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        self.progress_var.set(0)

    def validate_configuration(self):
        """Validate the current configuration"""
        # Check if poses exist
        poses = self.current_config.get("poses", {})
        sequence = self.current_config.get("sequence", [])
        
        # Collect all referenced pose names
        referenced_poses = set()
        for step in sequence:
            if step["action"] == "moveto":
                target = step.get("target")
                if target:
                    referenced_poses.add(target)
            elif step["action"] == "pick_and_place":
                pickup_poses = step.get("pickup_poses", [])
                place_poses = step.get("place_poses", [])
                referenced_poses.update(pickup_poses)
                referenced_poses.update(place_poses)
            elif step["action"] == "tool_exchange":
                step_poses = step.get("poses", [])
                referenced_poses.update(step_poses)
        
        # Check if all referenced poses exist
        missing_poses = referenced_poses - set(poses.keys())
        if missing_poses:
            return {
                "valid": False,
                "message": f"Missing poses: {', '.join(missing_poses)}"
            }
        
        return {"valid": True, "message": "Configuration is valid"}

    def _execute_task_thread(self):
        """Execute task using the MTC action client executable"""
        try:
            self.log_message("Starting MTC task execution...")
            
            # Create temporary JSON file
            with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
                json.dump(self.current_config, f, indent=2)
                self.temp_json_file = f.name
            
            self.log_message(f"Created temporary configuration file: {self.temp_json_file}")
            
            # Find the MTC action client executable
            mtc_client_path = None
            possible_paths = [
                "/home/aditya/work/github_ws/erobs/install/mtc_pipeline/lib/mtc_pipeline/mtc_action_client_example",
                "/home/aditya/work/github_ws/erobs/build/mtc_pipeline/mtc_action_client_example"
            ]
            
            for path in possible_paths:
                if os.path.exists(path):
                    mtc_client_path = path
                    break
            
            if not mtc_client_path:
                self.log_message("ERROR: Could not find MTC action client executable")
                return
            
            self.log_message(f"Using MTC client: {mtc_client_path}")
            
            # Execute the MTC action client
            cmd = [mtc_client_path, self.temp_json_file, self.robot_ip_var.get()]
            self.log_message(f"Executing: {' '.join(cmd)}")
            
            # Run the command and capture output
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            # Monitor the process
            while process.poll() is None:
                if self.stop_execution:
                    self.log_message("Stopping execution...")
                    process.terminate()
                    break
                
                # Read output
                output = process.stdout.readline()
                if output:
                    self.log_message(f"MTC: {output.strip()}")
                
                time.sleep(0.1)
            
            # Get final result
            return_code = process.poll()
            if return_code == 0:
                self.log_message("✓ Task completed successfully!")
            else:
                stderr_output = process.stderr.read()
                self.log_message(f"✗ Task failed with return code {return_code}")
                if stderr_output:
                    self.log_message(f"Error: {stderr_output}")
                
        except Exception as e:
            self.log_message(f"ERROR: {str(e)}")
        finally:
            # Clean up temporary file
            if self.temp_json_file and os.path.exists(self.temp_json_file):
                try:
                    os.unlink(self.temp_json_file)
                    self.log_message("Cleaned up temporary file")
                except:
                    pass
            
            # Reset GUI state
            self.root.after(0, self._reset_execution_state)

    def _reset_execution_state(self):
        """Reset execution GUI state"""
        self.execute_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.progress_var.set(0)
        self.stop_execution = False

    def stop_task(self):
        """Stop current task execution"""
        self.stop_execution = True
        self.log_message("Stopping task execution...")

    def test_mtc_server(self):
        """Test if the MTC action server is available"""
        def test_thread():
            try:
                self.log_message("Testing MTC action server availability...")
                
                # Check if the MTC action server executable exists
                mtc_server_path = "/home/aditya/work/github_ws/erobs/install/mtc_pipeline/lib/mtc_pipeline/mtc_orchestrator_action_server"
                
                if os.path.exists(mtc_server_path):
                    self.log_message("✓ MTC action server executable found")
                    
                    # Try to check if it's running
                    try:
                        result = subprocess.run(['pgrep', '-f', 'mtc_orchestrator_action_server'], 
                                             capture_output=True, text=True)
                        if result.returncode == 0:
                            self.log_message("✓ MTC action server is running")
                        else:
                            self.log_message("⚠ MTC action server is not running")
                            self.log_message("You may need to start it with: ros2 run mtc_pipeline mtc_orchestrator_action_server")
                    except Exception as e:
                        self.log_message(f"⚠ Could not check if server is running: {e}")
                else:
                    self.log_message("✗ MTC action server executable not found")
                    self.log_message("Make sure the package is built: colcon build --packages-select mtc_pipeline")
                    
            except Exception as e:
                self.log_message(f"✗ Test failed: {str(e)}")
        
        threading.Thread(target=test_thread, daemon=True).start()

    def load_json_file(self):
        """Load configuration from JSON file"""
        file_path = filedialog.askopenfilename(
            title="Load JSON Configuration",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if file_path:
            try:
                with open(file_path, 'r') as f:
                    self.current_config = json.load(f)
                
                # Update GUI with loaded configuration
                if "start_gripper" in self.current_config:
                    self.start_gripper_var.set(self.current_config["start_gripper"])
                
                self.update_task_tree()
                self.log_message(f"Loaded configuration from {file_path}")
                
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load file: {str(e)}")

    def save_json_file(self):
        """Save current configuration to JSON file"""
        file_path = filedialog.asksaveasfilename(
            title="Save JSON Configuration",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if file_path:
            try:
                # Update configuration with current GUI values
                self.current_config["start_gripper"] = self.start_gripper_var.get()
                
                with open(file_path, 'w') as f:
                    json.dump(self.current_config, f, indent=2)
                
                self.log_message(f"Configuration saved to {file_path}")
                
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save file: {str(e)}")

    def log_message(self, message):
        """Add message to status log"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        self.root.after(0, lambda: self.status_text.insert(tk.END, log_entry))
        self.root.after(0, lambda: self.status_text.see(tk.END))

    def clear_logs(self):
        """Clear status log"""
        self.status_text.delete(1.0, tk.END)

    def show_about(self):
        """Show about dialog"""
        messagebox.showinfo("About", 
                          "MTC Action Client GUI - Working Version\n\n"
                          "A graphical interface for the MoveIt Task Constructor (MTC) action client.\n"
                          "This version communicates with the actual MTC action server.\n"
                          "Allows you to create, edit, and execute robot task sequences.\n\n"
                          "Version: 1.0 (Working)")

    def run(self):
        """Start the GUI main loop"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            pass

def main():
    """Main function"""
    try:
        gui_client = MTCGUIClient()
        gui_client.run()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()
