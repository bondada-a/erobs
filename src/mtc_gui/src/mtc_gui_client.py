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
import numpy as np
from PIL import Image, ImageTk, ImageDraw, ImageFont
import cv2

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image as RosImage
    from apriltag_msgs.msg import AprilTagDetectionArray
    from std_srvs.srv import Trigger
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    print("Warning: ROS2 or cv_bridge not available. Camera view will be disabled.")
    ROS2_AVAILABLE = False

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

        # Camera state
        self.current_image = None
        self.current_detections = None
        self.camera_label = None
        self.bridge = CvBridge() if ROS2_AVAILABLE else None
        self.ros_node = None
        self.ros_spin_thread = None

        # Initialize ROS2 if available
        if ROS2_AVAILABLE:
            self.init_ros2()

        # Create GUI
        self.setup_gui()

        # Load default configuration
        self.load_default_config()

    def setup_gui(self):
        """Setup the main GUI window"""
        self.root = tk.Tk()
        self.root.title("MTC Action Client GUI with Camera View")
        self.root.geometry("1400x800")

        # Configure grid weights - 2 columns now
        self.root.grid_columnconfigure(0, weight=2)  # Left side - task editor (2x weight)
        self.root.grid_columnconfigure(1, weight=1)  # Right side - camera view
        self.root.grid_rowconfigure(1, weight=1)

        self.create_menu()
        self.create_robot_config_frame()
        self.create_task_editor_frame()
        self.create_execution_frame()
        self.create_status_frame()

        # Add camera view on the right side
        if ROS2_AVAILABLE:
            self.create_camera_panel()

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
                                    values=["epick", "hande", "pipettor", "none"], width=10)
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
        ttk.Button(toolbar, text="Add Vision MoveTo", command=lambda: self.add_task_step("vision_moveto")).pack(side="left", padx=(0, 5))
        ttk.Button(toolbar, text="Add Pipettor", command=lambda: self.add_task_step("pipettor")).pack(side="left", padx=(0, 5))
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
            "tasks": []
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
        step_id = len(self.current_config["tasks"]) + 1

        if action_type == "moveto":
            step = {
                "task_type": "moveto",
                "target": "moveit_home",
                "planning_type": "joint"
            }
        elif action_type == "pick_and_place":
            step = {
                "task_type": "pick_and_place",
                "gripper": "epick",
                "pick_approach": "pickup_approach",
                "pick_target": "pickup",
                "place_approach": "place_approach",
                "place_target": "place"
            }
        elif action_type == "tool_exchange":
            step = {
                "task_type": "tool_exchange",
                "operation": "load",
                "gripper": "hande",
                "dock_number": 3,
                "approach_pose": "load_approach"
            }
        elif action_type == "end_effector":
            step = {
                "task_type": "end_effector",
                "end_effector_type": "epick",
                "end_effector_action": "vacuum_on"
            }
        elif action_type == "vision_moveto":
            step = {
                "task_type": "vision_moveto",
                "tag_id": 0,
                "timeout": 10.0
            }
        elif action_type == "pipettor":
            step = {
                "task_type": "pipettor",
                "operation": "SUCK",
                "volume_pct": 0.5
            }

        self.current_config["tasks"].append(step)
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

        if 0 <= step_index < len(self.current_config["tasks"]):
            removed_step = self.current_config["tasks"].pop(step_index)
            self.update_task_tree()
            self.log_message(f"Removed step: {removed_step['task_type']}")

    def edit_task_step(self, event):
        """Edit selected task step"""
        selection = self.task_tree.selection()
        if not selection:
            return

        item = selection[0]
        step_index = int(item) - 1

        if 0 <= step_index < len(self.current_config["tasks"]):
            self.edit_step_dialog(step_index)

    def edit_step_dialog(self, step_index):
        """Open dialog to edit a task step"""
        step = self.current_config["tasks"][step_index]

        dialog = tk.Toplevel(self.root)
        dialog.title(f"Edit Step {step_index + 1}")

        # Larger dialog for pipettor due to LED controls
        if step["task_type"] == "pipettor":
            dialog.geometry("550x900")
        else:
            dialog.geometry("500x600")

        dialog.transient(self.root)
        try:
            dialog.grab_set()
        except:
            pass  # Ignore grab errors

        # Create form fields based on step type
        if step["task_type"] == "moveto":
            self.create_moveto_edit_form(dialog, step, step_index)
        elif step["task_type"] == "pick_and_place":
            self.create_pickplace_edit_form(dialog, step, step_index)
        elif step["task_type"] == "tool_exchange":
            self.create_toolexchange_edit_form(dialog, step, step_index)
        elif step["task_type"] == "end_effector":
            self.create_end_effector_edit_form(dialog, step, step_index)
        elif step["task_type"] == "vision_moveto":
            self.create_vision_moveto_edit_form(dialog, step, step_index)
        elif step["task_type"] == "pipettor":
            self.create_pipettor_edit_form(dialog, step, step_index)

    def create_moveto_edit_form(self, dialog, step, step_index):
        """Create edit form for moveto steps"""
        ttk.Label(dialog, text="MoveTo Configuration", font=("Arial", 12, "bold")).pack(pady=10)

        # Target pose/state
        ttk.Label(dialog, text="Target:").pack(anchor="w", padx=20)
        target_var = tk.StringVar(value=step.get("target", "moveit_home"))
        target_entry = ttk.Entry(dialog, textvariable=target_var, width=30)
        target_entry.pack(padx=20, pady=(0, 10))

        # Add help text
        help_text = ttk.Label(dialog, text="Named states: moveit_home, hande_open, hande_closed\nPoses: use names from pose manager",
                            font=("Arial", 8), foreground="gray")
        help_text.pack(padx=20, pady=(0, 10))

        # Planning type
        ttk.Label(dialog, text="Planning Type:").pack(anchor="w", padx=20)
        planning_var = tk.StringVar(value=step.get("planning_type", "joint"))
        planning_combo = ttk.Combobox(dialog, textvariable=planning_var,
                                    values=["joint", "cartesian"], width=30)
        planning_combo.pack(padx=20, pady=(0, 20))

        # Separator for optional relative move
        ttk.Separator(dialog, orient='horizontal').pack(fill='x', padx=20, pady=10)

        ttk.Label(dialog, text="Relative Move (Optional)", font=("Arial", 10, "bold")).pack(pady=5)

        # Direction
        ttk.Label(dialog, text="Direction:").pack(anchor="w", padx=20)
        direction_var = tk.StringVar(value=step.get("direction", ""))
        direction_combo = ttk.Combobox(dialog, textvariable=direction_var,
                                     values=["", "forward", "backward", "left", "right", "up", "down"], width=30)
        direction_combo.pack(padx=20, pady=(0, 10))

        # Distance
        ttk.Label(dialog, text="Distance (meters):").pack(anchor="w", padx=20)
        distance_var = tk.StringVar(value=str(step.get("distance", "")))
        distance_entry = ttk.Entry(dialog, textvariable=distance_var, width=30)
        distance_entry.pack(padx=20, pady=(0, 20))

        ttk.Label(dialog, text="Note: If direction is set, target is ignored",
                font=("Arial", 8), foreground="gray").pack(padx=20, pady=(0, 10))

        def save_changes():
            step["target"] = target_var.get()
            step["planning_type"] = planning_var.get()

            # Handle optional relative move
            if direction_var.get():
                step["direction"] = direction_var.get()
                try:
                    step["distance"] = float(distance_var.get()) if distance_var.get() else 0.0
                except ValueError:
                    step["distance"] = 0.0
            else:
                # Remove relative move fields if direction is empty
                step.pop("direction", None)
                step.pop("distance", None)

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

        # Pick approach pose
        ttk.Label(dialog, text="Pick Approach Pose:").pack(anchor="w", padx=20)
        pick_approach_var = tk.StringVar(value=step.get("pick_approach", ""))
        pick_approach_entry = ttk.Entry(dialog, textvariable=pick_approach_var, width=30)
        pick_approach_entry.pack(padx=20, pady=(0, 10))

        # Pick target pose
        ttk.Label(dialog, text="Pick Target Pose:").pack(anchor="w", padx=20)
        pick_target_var = tk.StringVar(value=step.get("pick_target", ""))
        pick_target_entry = ttk.Entry(dialog, textvariable=pick_target_var, width=30)
        pick_target_entry.pack(padx=20, pady=(0, 10))

        # Place approach pose
        ttk.Label(dialog, text="Place Approach Pose:").pack(anchor="w", padx=20)
        place_approach_var = tk.StringVar(value=step.get("place_approach", ""))
        place_approach_entry = ttk.Entry(dialog, textvariable=place_approach_var, width=30)
        place_approach_entry.pack(padx=20, pady=(0, 10))

        # Place target pose
        ttk.Label(dialog, text="Place Target Pose:").pack(anchor="w", padx=20)
        place_target_var = tk.StringVar(value=step.get("place_target", ""))
        place_target_entry = ttk.Entry(dialog, textvariable=place_target_var, width=30)
        place_target_entry.pack(padx=20, pady=(0, 20))

        def save_changes():
            step["gripper"] = gripper_var.get()
            step["pick_approach"] = pick_approach_var.get()
            step["pick_target"] = pick_target_var.get()
            step["place_approach"] = place_approach_var.get()
            step["place_target"] = place_target_var.get()
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
                                    values=["epick", "hande", "none"], width=30)
        gripper_combo.pack(padx=20, pady=(0, 10))

        # Dock number
        ttk.Label(dialog, text="Dock Number:").pack(anchor="w", padx=20)
        dock_var = tk.StringVar(value=str(step.get("dock_number", 3)))
        dock_entry = ttk.Entry(dialog, textvariable=dock_var, width=30)
        dock_entry.pack(padx=20, pady=(0, 10))

        # Approach pose
        ttk.Label(dialog, text="Approach Pose:").pack(anchor="w", padx=20)
        approach_var = tk.StringVar(value=step.get("approach_pose", ""))
        approach_entry = ttk.Entry(dialog, textvariable=approach_var, width=30)
        approach_entry.pack(padx=20, pady=(0, 20))

        def save_changes():
            step["operation"] = operation_var.get()
            step["gripper"] = gripper_var.get()
            step["dock_number"] = int(dock_var.get())
            step["approach_pose"] = approach_var.get()
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

    def create_vision_moveto_edit_form(self, dialog, step, step_index):
        """Create edit form for vision moveto steps"""
        ttk.Label(dialog, text="Vision MoveTo Configuration", font=("Arial", 12, "bold")).pack(pady=10)

        # Add description
        description = ttk.Label(dialog,
                               text="Detect AprilTag and move gripper to tag location",
                               font=("Arial", 9),
                               foreground="gray")
        description.pack(padx=20, pady=(0, 20))

        # Tag ID
        ttk.Label(dialog, text="AprilTag ID:").pack(anchor="w", padx=20)
        tag_id_var = tk.StringVar(value=str(step.get("tag_id", 0)))
        tag_id_entry = ttk.Entry(dialog, textvariable=tag_id_var, width=30)
        tag_id_entry.pack(padx=20, pady=(0, 10))

        ttk.Label(dialog, text="The ID number of the AprilTag to detect",
                 font=("Arial", 8), foreground="gray").pack(padx=20, pady=(0, 10))

        # Timeout
        ttk.Label(dialog, text="Timeout (seconds):").pack(anchor="w", padx=20)
        timeout_var = tk.StringVar(value=str(step.get("timeout", 10.0)))
        timeout_entry = ttk.Entry(dialog, textvariable=timeout_var, width=30)
        timeout_entry.pack(padx=20, pady=(0, 10))

        ttk.Label(dialog, text="Maximum time to wait for tag detection",
                 font=("Arial", 8), foreground="gray").pack(padx=20, pady=(0, 20))

        # Information box
        info_frame = ttk.LabelFrame(dialog, text="Info", padding="10")
        info_frame.pack(padx=20, pady=(10, 20), fill="x")

        info_text = ("Vision MoveTo will:\n"
                    "1. Continuously capture images from Zivid camera\n"
                    "2. Detect the specified AprilTag\n"
                    "3. Move gripper to the detected tag position\n"
                    "4. Use Cartesian planning for straight-line motion")
        ttk.Label(info_frame, text=info_text, justify="left",
                 font=("Arial", 8)).pack()

        def save_changes():
            try:
                step["tag_id"] = int(tag_id_var.get())
                step["timeout"] = float(timeout_var.get())
                self.update_task_tree()
                dialog.destroy()
                self.log_message(f"Updated step {step_index + 1}")
            except ValueError:
                messagebox.showerror("Invalid Input",
                                   "Tag ID must be an integer and timeout must be a number")

        ttk.Button(dialog, text="Save", command=save_changes).pack(pady=10)

    def create_pipettor_edit_form(self, dialog, step, step_index):
        """Create edit form for pipettor steps"""
        ttk.Label(dialog, text="Pipettor Configuration", font=("Arial", 12, "bold")).pack(pady=10)

        # Add description
        description = ttk.Label(dialog,
                               text="Control pipettor operations: aspirate, dispense, eject tip, and LED color",
                               font=("Arial", 9),
                               foreground="gray")
        description.pack(padx=20, pady=(0, 20))

        # Operation
        ttk.Label(dialog, text="Operation:").pack(anchor="w", padx=20)
        operation_var = tk.StringVar(value=step.get("operation", "SUCK"))
        operation_combo = ttk.Combobox(dialog, textvariable=operation_var,
                                     values=["SUCK", "EXPEL", "EJECT_TIP", "SET_LED"],
                                     width=30, state="readonly")
        operation_combo.pack(padx=20, pady=(0, 10))

        # Volume percentage
        ttk.Label(dialog, text="Volume Percentage (0.0 - 1.0):").pack(anchor="w", padx=20)
        volume_var = tk.StringVar(value=str(step.get("volume_pct", 0.5)))
        volume_entry = ttk.Entry(dialog, textvariable=volume_var, width=30)
        volume_entry.pack(padx=20, pady=(0, 5))

        # Volume slider for easier control
        volume_slider_var = tk.DoubleVar(value=step.get("volume_pct", 0.5))
        volume_slider = ttk.Scale(dialog, from_=0.0, to=1.0, orient="horizontal",
                                 variable=volume_slider_var, length=300)
        volume_slider.pack(padx=20, pady=(0, 10))

        # Sync slider with entry
        def update_volume_entry(val):
            volume_var.set(f"{float(val):.2f}")

        volume_slider.config(command=update_volume_entry)

        ttk.Label(dialog, text="Used for SUCK and EXPEL operations (0.0 = empty, 1.0 = full)",
                 font=("Arial", 8), foreground="gray").pack(padx=20, pady=(0, 20))

        # LED Color section
        ttk.Separator(dialog, orient='horizontal').pack(fill='x', padx=20, pady=10)
        ttk.Label(dialog, text="LED Color (for SET_LED operation)", font=("Arial", 10, "bold")).pack(pady=5)

        # Get current LED color or use defaults
        led_color = step.get("led_color", {"r": 0.0, "g": 1.0, "b": 0.0, "a": 1.0})

        # Red slider
        ttk.Label(dialog, text="Red (0.0 - 1.0):").pack(anchor="w", padx=20)
        red_var = tk.DoubleVar(value=led_color.get("r", 0.0))
        red_slider = ttk.Scale(dialog, from_=0.0, to=1.0, orient="horizontal",
                              variable=red_var, length=300)
        red_slider.pack(padx=20, pady=(0, 5))

        # Green slider
        ttk.Label(dialog, text="Green (0.0 - 1.0):").pack(anchor="w", padx=20)
        green_var = tk.DoubleVar(value=led_color.get("g", 1.0))
        green_slider = ttk.Scale(dialog, from_=0.0, to=1.0, orient="horizontal",
                                variable=green_var, length=300)
        green_slider.pack(padx=20, pady=(0, 5))

        # Blue slider
        ttk.Label(dialog, text="Blue (0.0 - 1.0):").pack(anchor="w", padx=20)
        blue_var = tk.DoubleVar(value=led_color.get("b", 0.0))
        blue_slider = ttk.Scale(dialog, from_=0.0, to=1.0, orient="horizontal",
                               variable=blue_var, length=300)
        blue_slider.pack(padx=20, pady=(0, 5))

        # Alpha slider
        ttk.Label(dialog, text="Alpha/Brightness (0.0 - 1.0):").pack(anchor="w", padx=20)
        alpha_var = tk.DoubleVar(value=led_color.get("a", 1.0))
        alpha_slider = ttk.Scale(dialog, from_=0.0, to=1.0, orient="horizontal",
                                variable=alpha_var, length=300)
        alpha_slider.pack(padx=20, pady=(0, 10))

        # Color preview
        color_preview = tk.Canvas(dialog, width=100, height=30, bg="white")
        color_preview.pack(padx=20, pady=(5, 10))

        def update_color_preview(*args):
            r = int(red_var.get() * 255)
            g = int(green_var.get() * 255)
            b = int(blue_var.get() * 255)
            color_hex = f"#{r:02x}{g:02x}{b:02x}"
            color_preview.config(bg=color_hex)

        # Bind sliders to preview update
        red_var.trace('w', update_color_preview)
        green_var.trace('w', update_color_preview)
        blue_var.trace('w', update_color_preview)

        # Initial preview
        update_color_preview()

        # Preset color buttons
        preset_frame = ttk.LabelFrame(dialog, text="Color Presets", padding="5")
        preset_frame.pack(padx=20, pady=(10, 20), fill="x")

        def set_preset_color(r, g, b):
            red_var.set(r)
            green_var.set(g)
            blue_var.set(b)
            alpha_var.set(1.0)

        ttk.Button(preset_frame, text="Red",
                  command=lambda: set_preset_color(1.0, 0.0, 0.0)).pack(side="left", padx=2)
        ttk.Button(preset_frame, text="Green",
                  command=lambda: set_preset_color(0.0, 1.0, 0.0)).pack(side="left", padx=2)
        ttk.Button(preset_frame, text="Blue",
                  command=lambda: set_preset_color(0.0, 0.0, 1.0)).pack(side="left", padx=2)
        ttk.Button(preset_frame, text="Yellow",
                  command=lambda: set_preset_color(1.0, 1.0, 0.0)).pack(side="left", padx=2)
        ttk.Button(preset_frame, text="Purple",
                  command=lambda: set_preset_color(0.5, 0.0, 0.5)).pack(side="left", padx=2)
        ttk.Button(preset_frame, text="White",
                  command=lambda: set_preset_color(1.0, 1.0, 1.0)).pack(side="left", padx=2)
        ttk.Button(preset_frame, text="Off",
                  command=lambda: set_preset_color(0.0, 0.0, 0.0)).pack(side="left", padx=2)

        # Info box
        info_frame = ttk.LabelFrame(dialog, text="Operation Info", padding="10")
        info_frame.pack(padx=20, pady=(10, 20), fill="x")

        info_text = ("SUCK: Aspirate liquid (uses volume_pct)\n"
                    "EXPEL: Dispense liquid (uses volume_pct)\n"
                    "EJECT_TIP: Eject pipette tip (volume_pct ignored)\n"
                    "SET_LED: Change LED color (uses led_color, volume_pct ignored)")
        ttk.Label(info_frame, text=info_text, justify="left",
                 font=("Arial", 8)).pack()

        def save_changes():
            try:
                step["operation"] = operation_var.get()
                step["volume_pct"] = float(volume_var.get())

                # Always include led_color for consistency
                step["led_color"] = {
                    "r": float(red_var.get()),
                    "g": float(green_var.get()),
                    "b": float(blue_var.get()),
                    "a": float(alpha_var.get())
                }

                self.update_task_tree()
                dialog.destroy()
                self.log_message(f"Updated pipettor step {step_index + 1}: {operation_var.get()}")
            except ValueError:
                messagebox.showerror("Invalid Input",
                                   "Volume percentage must be a number between 0.0 and 1.0")

        ttk.Button(dialog, text="Save", command=save_changes).pack(pady=10)

    def update_task_tree(self):
        """Update the task sequence tree display"""
        # Clear existing items
        for item in self.task_tree.get_children():
            self.task_tree.delete(item)

        # Add current tasks
        for i, step in enumerate(self.current_config["tasks"]):
            step_id = str(i + 1)
            action = step.get("task_type", "unknown")

            # Create details string
            details = ""
            if action == "moveto":
                target = step.get('target', 'unknown')
                direction = step.get('direction', '')
                if direction:
                    distance = step.get('distance', 0)
                    details = f"Relative move {direction} {distance}m"
                else:
                    details = f"Move to {target}"
            elif action == "pick_and_place":
                pick_target = step.get('pick_target', '?')
                place_target = step.get('place_target', '?')
                gripper = step.get('gripper', 'unknown')
                details = f"Pick from {pick_target} to {place_target} ({gripper})"
            elif action == "tool_exchange":
                operation = step.get('operation', '?')
                gripper = step.get('gripper', '?')
                dock_number = step.get('dock_number', '?')
                details = f"{operation} {gripper} at dock {dock_number}"
            elif action == "end_effector":
                ee_type = step.get('end_effector_type', 'unknown')
                ee_action = step.get('end_effector_action', 'unknown')
                details = f"{ee_type} {ee_action}"
            elif action == "vision_moveto":
                tag_id = step.get('tag_id', 0)
                timeout = step.get('timeout', 10.0)
                details = f"Detect tag {tag_id} (timeout: {timeout}s)"
            elif action == "pipettor":
                operation = step.get('operation', 'SUCK')
                volume_pct = step.get('volume_pct', 0.0)
                if operation in ["SUCK", "EXPEL"]:
                    details = f"{operation} at {volume_pct*100:.0f}% volume"
                elif operation == "SET_LED":
                    led_color = step.get('led_color', {})
                    r = led_color.get('r', 0.0)
                    g = led_color.get('g', 0.0)
                    b = led_color.get('b', 0.0)
                    details = f"SET_LED (R:{r:.1f}, G:{g:.1f}, B:{b:.1f})"
                else:
                    details = f"{operation}"

            self.task_tree.insert("", "end", iid=step_id, text=step_id,
                                values=(action, details))

    def execute_task(self):
        """Execute the current task configuration using the MTC action client"""
        if not self.current_config["tasks"]:
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
        tasks = self.current_config.get("tasks", [])

        # Define known named states (from SRDF)
        known_named_states = {
            "moveit_home", "hande_open", "hande_closed"
        }

        # Collect all referenced pose names
        referenced_poses = set()
        for step in tasks:
            if step["task_type"] == "moveto":
                target = step.get("target")
                direction = step.get("direction")
                if target and not direction:
                    # Only check if it's not a named state and not a relative move
                    if target not in known_named_states:
                        referenced_poses.add(target)
            elif step["task_type"] == "pick_and_place":
                # Add all 4 explicit pose fields
                pick_approach = step.get("pick_approach")
                pick_target = step.get("pick_target")
                place_approach = step.get("place_approach")
                place_target = step.get("place_target")
                if pick_approach:
                    referenced_poses.add(pick_approach)
                if pick_target:
                    referenced_poses.add(pick_target)
                if place_approach:
                    referenced_poses.add(place_approach)
                if place_target:
                    referenced_poses.add(place_target)
            elif step["task_type"] == "tool_exchange":
                # Use singular approach_pose
                approach_pose = step.get("approach_pose")
                if approach_pose:
                    referenced_poses.add(approach_pose)
        
        # Check if all referenced poses exist (excluding named states)
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

    def create_camera_panel(self):
        """Create camera view panel on the right side"""
        camera_frame = ttk.LabelFrame(self.root, text="Camera View", padding="10")
        camera_frame.grid(row=0, column=1, rowspan=4, sticky="nsew", padx=(0, 10), pady=5)
        camera_frame.grid_rowconfigure(1, weight=1)
        camera_frame.grid_columnconfigure(0, weight=1)

        # Control buttons
        button_frame = ttk.Frame(camera_frame)
        button_frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))

        ttk.Button(button_frame, text="Capture Image",
                  command=self.trigger_capture).pack(side="left", padx=(0, 5))

        self.camera_status_label = ttk.Label(button_frame, text="Waiting for camera...",
                                             foreground="gray")
        self.camera_status_label.pack(side="left", padx=(10, 0))

        # Image display area
        self.camera_label = tk.Label(camera_frame, bg="black", text="No camera feed",
                                     fg="white", font=("Arial", 14))
        self.camera_label.grid(row=1, column=0, sticky="nsew")

        # Detection info
        self.detection_info = scrolledtext.ScrolledText(camera_frame, height=6, wrap=tk.WORD)
        self.detection_info.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        self.detection_info.insert(tk.END, "No detections yet\n")

    def init_ros2(self):
        """Initialize ROS2 node and subscriptions"""
        try:
            if not rclpy.ok():
                rclpy.init()

            self.ros_node = rclpy.create_node('mtc_gui_camera_client')

            # Subscribe to camera image
            self.image_sub = self.ros_node.create_subscription(
                RosImage,
                '/color/image_color',
                self.image_callback,
                10
            )

            # Subscribe to AprilTag detections
            self.detection_sub = self.ros_node.create_subscription(
                AprilTagDetectionArray,
                '/detections',
                self.detection_callback,
                10
            )

            # Create capture service client
            self.capture_client = self.ros_node.create_client(Trigger, '/capture_2d')

            # Start ROS2 spinning in background thread
            self.ros_spin_thread = threading.Thread(target=self.spin_ros, daemon=True)
            self.ros_spin_thread.start()

            print("ROS2 node initialized for camera view")

        except Exception as e:
            print(f"Failed to initialize ROS2: {e}")

    def spin_ros(self):
        """Spin ROS2 node in background thread"""
        while rclpy.ok():
            try:
                rclpy.spin_once(self.ros_node, timeout_sec=0.1)
            except Exception as e:
                print(f"ROS2 spin error: {e}")
                break

    def image_callback(self, msg):
        """Handle incoming camera images"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_image = cv_image

            # Update GUI in main thread
            self.root.after(0, self.update_camera_display)

        except Exception as e:
            print(f"Image callback error: {e}")

    def detection_callback(self, msg):
        """Handle AprilTag detections"""
        self.current_detections = msg

        # Update detection info in GUI
        self.root.after(0, self.update_detection_info)

    def update_camera_display(self):
        """Update camera image display with AprilTag overlays"""
        if self.current_image is None or self.camera_label is None:
            return

        try:
            # Clone image for drawing
            display_image = self.current_image.copy()

            # Draw AprilTag overlays if we have detections
            if self.current_detections and len(self.current_detections.detections) > 0:
                for detection in self.current_detections.detections:
                    # Get corners
                    corners = detection.corners

                    if len(corners) == 4:
                        # Draw bounding box
                        pts = np.array([[int(c.x), int(c.y)] for c in corners], np.int32)
                        pts = pts.reshape((-1, 1, 2))
                        cv2.polylines(display_image, [pts], True, (0, 255, 0), 3)

                        # Draw tag ID
                        center_x = int(sum(c.x for c in corners) / 4)
                        center_y = int(sum(c.y for c in corners) / 4)

                        # Add background rectangle for text
                        text = f"ID: {detection.id}"
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        font_scale = 1.5
                        thickness = 3
                        (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, thickness)

                        cv2.rectangle(display_image,
                                    (center_x - 10, center_y - text_height - 10),
                                    (center_x + text_width + 10, center_y + 10),
                                    (0, 255, 0), -1)

                        cv2.putText(display_image, text, (center_x, center_y),
                                   font, font_scale, (0, 0, 0), thickness)

                self.camera_status_label.config(
                    text=f"{len(self.current_detections.detections)} tag(s) detected",
                    foreground="green"
                )
            else:
                self.camera_status_label.config(
                    text="No tags detected",
                    foreground="orange"
                )

            # Resize image to fit display (maintain aspect ratio)
            height, width = display_image.shape[:2]
            max_width = 600
            max_height = 600

            scale = min(max_width / width, max_height / height)
            new_width = int(width * scale)
            new_height = int(height * scale)

            display_image = cv2.resize(display_image, (new_width, new_height))

            # Convert to PIL Image then to PhotoImage
            display_image_rgb = cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(display_image_rgb)
            photo = ImageTk.PhotoImage(image=pil_image)

            # Update label
            self.camera_label.config(image=photo, text="")
            self.camera_label.image = photo  # Keep reference

        except Exception as e:
            print(f"Display update error: {e}")

    def update_detection_info(self):
        """Update detection information display"""
        if self.detection_info is None:
            return

        try:
            self.detection_info.delete(1.0, tk.END)

            if self.current_detections and len(self.current_detections.detections) > 0:
                self.detection_info.insert(tk.END, f"Detected {len(self.current_detections.detections)} tag(s):\n\n")

                for detection in self.current_detections.detections:
                    self.detection_info.insert(tk.END,
                        f"Tag ID: {detection.id}\n"
                        f"Family: {detection.family}\n"
                        f"Hamming: {detection.hamming}\n"
                        f"Decision Margin: {detection.decision_margin:.2f}\n"
                        f"---\n"
                    )
            else:
                self.detection_info.insert(tk.END, "No tags detected\n")

        except Exception as e:
            print(f"Detection info update error: {e}")

    def trigger_capture(self):
        """Manually trigger Zivid camera capture"""
        if not self.capture_client:
            messagebox.showwarning("Camera", "Capture service not available")
            return

        def capture_thread():
            try:
                self.log_message("Triggering camera capture...")

                if not self.capture_client.wait_for_service(timeout_sec=2.0):
                    self.log_message("Capture service not available")
                    return

                request = Trigger.Request()
                future = self.capture_client.call_async(request)

                # Wait for result
                timeout = 10.0
                start_time = time.time()
                while not future.done():
                    if time.time() - start_time > timeout:
                        self.log_message("Capture timeout")
                        return
                    time.sleep(0.1)

                response = future.result()
                if response.success:
                    self.log_message("✓ Camera capture successful")
                else:
                    self.log_message(f"✗ Capture failed: {response.message}")

            except Exception as e:
                self.log_message(f"Capture error: {e}")

        threading.Thread(target=capture_thread, daemon=True).start()

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
