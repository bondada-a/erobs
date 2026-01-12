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
    from rclpy.action import ActionClient
    from action_msgs.msg import GoalStatus
    from sensor_msgs.msg import Image as RosImage
    from std_srvs.srv import Trigger
    from zivid_interfaces.srv import CaptureAndDetectMarkers
    from cv_bridge import CvBridge
    from beambot_interfaces.action import MTCExecution
    ROS2_AVAILABLE = True
except ImportError as e:
    print(f"Warning: ROS2 or required packages not available: {e}")
    print("Camera view and task execution will be disabled.")
    ROS2_AVAILABLE = False

# Import local modules
try:
    from .pose_editor import PoseManager
    from .poses_manager import PosesManager
    from .save_current_pose_dialog import SaveCurrentPoseDialog
except ImportError:
    print("Warning: Local modules not found. Running with limited functionality.")
    # Fallback if modules not found
    PoseManager = None
    PosesManager = None
    SaveCurrentPoseDialog = None

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
        self.current_json_file = None  # Track currently loaded JSON file

        # Action client state
        self.mtc_action_client = None

        # Camera state
        self.current_image = None
        self.current_detections = []  # List of Zivid MarkerShape objects
        self.camera_label = None
        self.bridge = CvBridge() if ROS2_AVAILABLE else None
        self.ros_node = None
        self.ros_spin_thread = None

        # Robot state
        self.current_robot_pose = None  # Current robot joint positions in degrees

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
        self.root.title("MTC GUI Client (beambot)")
        self.root.geometry("1920x1080")

        # Configure grid weights - 2 columns now
        self.root.grid_columnconfigure(0, weight=3)  # Left side - task editor (3x weight for more space)
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
        self.robot_ip_entry = ttk.Entry(config_frame, textvariable=self.robot_ip_var, width=30)
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

        # Save Current Pose Button
        if ROS2_AVAILABLE:
            ttk.Button(config_frame, text="Save Current Pose",
                      command=self.save_current_pose).grid(row=0, column=6, padx=(20, 0))

    def create_task_editor_frame(self):
        """Create task sequence editor frame"""
        editor_frame = ttk.LabelFrame(self.root, text="Task Sequence Editor", padding="10")
        editor_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)
        editor_frame.grid_columnconfigure(0, weight=1)
        editor_frame.grid_rowconfigure(1, weight=1)
        
        # Toolbar - organize in two rows to prevent overflow
        toolbar = ttk.Frame(editor_frame)
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 10))

        # First row of buttons
        toolbar_row1 = ttk.Frame(toolbar)
        toolbar_row1.pack(fill="x", pady=(0, 5))
        ttk.Button(toolbar_row1, text="Add MoveTo", command=lambda: self.add_task_step("moveto")).pack(side="left", padx=(0, 5))
        ttk.Button(toolbar_row1, text="Add Pick&Place", command=lambda: self.add_task_step("pick_and_place")).pack(side="left", padx=(0, 5))
        ttk.Button(toolbar_row1, text="Add Tool Exchange", command=lambda: self.add_task_step("tool_exchange")).pack(side="left", padx=(0, 5))
        ttk.Button(toolbar_row1, text="Add End Effector", command=lambda: self.add_task_step("end_effector")).pack(side="left", padx=(0, 5))

        # Second row of buttons
        toolbar_row2 = ttk.Frame(toolbar)
        toolbar_row2.pack(fill="x")
        ttk.Button(toolbar_row2, text="Add Vision MoveTo", command=lambda: self.add_task_step("vision_moveto")).pack(side="left", padx=(0, 5))
        ttk.Button(toolbar_row2, text="Add Pipettor", command=lambda: self.add_task_step("pipettor")).pack(side="left", padx=(0, 5))
        ttk.Button(toolbar_row2, text="Remove Step", command=self.remove_task_step).pack(side="left", padx=(20, 0))
        ttk.Button(toolbar_row2, text="Clear All", command=self.clear_all_tasks).pack(side="left", padx=(5, 0))
        ttk.Button(toolbar_row2, text="↑ Move Up", command=self.move_task_up).pack(side="left", padx=(20, 0))
        ttk.Button(toolbar_row2, text="↓ Move Down", command=self.move_task_down).pack(side="left", padx=(5, 0))
        
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
        exec_frame.grid_columnconfigure(4, weight=1)  # Progress bar column expands

        # Execute button
        self.execute_btn = ttk.Button(exec_frame, text="Execute Task", command=self.execute_task)
        self.execute_btn.grid(row=0, column=0, padx=(0, 10))

        # Stop button
        self.stop_btn = ttk.Button(exec_frame, text="Stop Execution", command=self.stop_task, state="disabled")
        self.stop_btn.grid(row=0, column=1, padx=(0, 10))

        # Pause button
        self.pause_btn = ttk.Button(exec_frame, text="⏸ Pause", command=self.pause_task, state="disabled")
        self.pause_btn.grid(row=0, column=2, padx=(0, 5))

        # Resume button
        self.resume_btn = ttk.Button(exec_frame, text="▶ Resume", command=self.resume_task, state="disabled")
        self.resume_btn.grid(row=0, column=3, padx=(0, 10))

        # Progress bar
        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(exec_frame, variable=self.progress_var, maximum=100)
        self.progress_bar.grid(row=0, column=4, sticky="ew", padx=(10, 0))

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

    def save_current_pose(self):
        """Open dialog to save the current robot pose"""
        if not ROS2_AVAILABLE:
            messagebox.showwarning("Warning", "ROS2 not available")
            return

        if not SaveCurrentPoseDialog:
            messagebox.showwarning("Warning", "Save Current Pose dialog not available")
            return

        if self.current_robot_pose is None:
            messagebox.showwarning("Warning",
                                 "No robot pose available. Make sure the robot is connected and publishing joint states.")
            return

        try:
            dialog = SaveCurrentPoseDialog(self.root, self.current_robot_pose,
                                          self.current_config, self.current_json_file)
            result = dialog.show()

            if result:
                action = result["action"]
                pose_name = result["pose_name"]
                pose_values = result["pose_values"]

                if action == "add_to_config":
                    # Add to current configuration
                    self.current_config["poses"][pose_name] = pose_values
                    self.log_message(f"Added pose '{pose_name}' to current configuration")

                elif action == "save_to_current":
                    # Save to currently loaded JSON file
                    if self.current_json_file:
                        self.current_config["poses"][pose_name] = pose_values
                        self.current_config["start_gripper"] = self.start_gripper_var.get()

                        with open(self.current_json_file, 'w') as f:
                            json.dump(self.current_config, f, indent=2)

                        self.log_message(f"Saved pose '{pose_name}' to {self.current_json_file}")
                    else:
                        messagebox.showerror("Error", "No JSON file currently loaded")

                elif action == "save_to_new":
                    # Save to new JSON file
                    file_path = result.get("file_path")
                    if file_path:
                        # Update or create poses dictionary
                        if "poses" not in self.current_config:
                            self.current_config["poses"] = {}
                        self.current_config["poses"][pose_name] = pose_values

                        # Save to file
                        with open(file_path, 'w') as f:
                            json.dump(self.current_config, f, indent=2)

                        self.current_json_file = file_path
                        self.log_message(f"Saved pose '{pose_name}' to new file: {file_path}")

        except Exception as e:
            messagebox.showerror("Error", f"Failed to save pose: {str(e)}")

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
                "detection_type": "marker",  # "marker" or "circle"
                "tag_id": 0,
                "timeout": 10.0,
                "z_offset": 0.0,  # 0 = use gripper default
                "marker_dictionary": "aruco4x4_50"  # Default ArUco dictionary
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
        """Remove all selected task steps (supports multi-select with Ctrl+Click or Shift+Click)"""
        selection = self.task_tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select step(s) to remove")
            return

        # Convert 1-based item IDs to 0-based indices, sorted descending
        # (must delete from highest index first to avoid shifting issues)
        indices = sorted([int(item) - 1 for item in selection], reverse=True)

        # Remove each selected task
        removed_tasks = []
        for step_index in indices:
            if 0 <= step_index < len(self.current_config["tasks"]):
                removed_step = self.current_config["tasks"].pop(step_index)
                removed_tasks.append(removed_step['task_type'])

        if removed_tasks:
            self.update_task_tree()
            # Log message (reverse list to show in original order)
            removed_tasks.reverse()
            if len(removed_tasks) == 1:
                self.log_message(f"Removed step: {removed_tasks[0]}")
            else:
                self.log_message(f"Removed {len(removed_tasks)} steps: {', '.join(removed_tasks)}")

    def clear_all_tasks(self):
        """Clear all tasks from the sequence with confirmation"""
        if not self.current_config["tasks"]:
            messagebox.showinfo("Info", "Task list is already empty")
            return

        # Ask for confirmation
        task_count = len(self.current_config["tasks"])
        if messagebox.askyesno("Confirm Clear All",
                               f"Are you sure you want to remove all {task_count} task(s)?"):
            self.current_config["tasks"] = []
            self.update_task_tree()
            self.log_message(f"Cleared all {task_count} tasks")

    def move_task_up(self):
        """Move selected task up in the sequence"""
        selection = self.task_tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a step to move")
            return

        if len(selection) > 1:
            messagebox.showwarning("Warning", "Please select only one step to move")
            return

        # Convert 1-based item ID to 0-based index
        step_index = int(selection[0]) - 1
        tasks = self.current_config["tasks"]

        # Check bounds
        if step_index <= 0:
            messagebox.showinfo("Info", "Cannot move the first step up")
            return

        # Swap with previous item
        tasks[step_index], tasks[step_index - 1] = tasks[step_index - 1], tasks[step_index]
        self.update_task_tree()

        # Re-select the moved item (now at new position)
        new_item_id = str(step_index)  # Was step_index+1, now step_index (1-based)
        self.task_tree.selection_set(new_item_id)
        self.task_tree.focus(new_item_id)
        self.log_message(f"Moved step {step_index + 1} up to position {step_index}")

    def move_task_down(self):
        """Move selected task down in the sequence"""
        selection = self.task_tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a step to move")
            return

        if len(selection) > 1:
            messagebox.showwarning("Warning", "Please select only one step to move")
            return

        # Convert 1-based item ID to 0-based index
        step_index = int(selection[0]) - 1
        tasks = self.current_config["tasks"]

        # Check bounds
        if step_index >= len(tasks) - 1:
            messagebox.showinfo("Info", "Cannot move the last step down")
            return

        # Swap with next item
        tasks[step_index], tasks[step_index + 1] = tasks[step_index + 1], tasks[step_index]
        self.update_task_tree()

        # Re-select the moved item (now at new position)
        new_item_id = str(step_index + 2)  # Was step_index+1, now step_index+2 (1-based)
        self.task_tree.selection_set(new_item_id)
        self.task_tree.focus(new_item_id)
        self.log_message(f"Moved step {step_index + 1} down to position {step_index + 2}")

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
        elif step["task_type"] == "vision_moveto":
            dialog.geometry("500x700")  # Taller for detection type options
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
                                    values=["epick", "hande", "pipettor", "none"], width=30)
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
                                  values=["hande_open", "hande_closed", "vacuum_on", "vacuum_off"], width=30)
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
                               text="Detect object using Zivid camera and move gripper to detected location",
                               font=("Arial", 9),
                               foreground="gray")
        description.pack(padx=20, pady=(0, 20))

        # Detection Type (NEW)
        ttk.Label(dialog, text="Detection Type:").pack(anchor="w", padx=20)
        detection_type_var = tk.StringVar(value=step.get("detection_type", "marker"))
        detection_type_combo = ttk.Combobox(dialog, textvariable=detection_type_var,
                                           values=["marker", "circle", "contour"],
                                           width=27, state="readonly")
        detection_type_combo.pack(padx=20, pady=(0, 5))

        ttk.Label(dialog, text="marker = ArUco tag, circle = Hough circles, contour = any shape by edges",
                 font=("Arial", 8), foreground="gray").pack(padx=20, pady=(0, 15))

        # Frame for marker-specific options
        marker_frame = ttk.LabelFrame(dialog, text="ArUco Marker Options", padding="10")
        marker_frame.pack(padx=20, pady=(0, 10), fill="x")

        # Marker ID
        ttk.Label(marker_frame, text="Marker ID:").pack(anchor="w")
        tag_id_var = tk.StringVar(value=str(step.get("tag_id", 0)))
        tag_id_entry = ttk.Entry(marker_frame, textvariable=tag_id_var, width=25)
        tag_id_entry.pack(pady=(0, 5))

        # Marker Dictionary
        ttk.Label(marker_frame, text="Dictionary:").pack(anchor="w")
        marker_dict_var = tk.StringVar(value=step.get("marker_dictionary", "aruco4x4_50"))
        marker_dict_combo = ttk.Combobox(marker_frame, textvariable=marker_dict_var,
                                        values=["aruco4x4_50", "aruco4x4_100", "aruco4x4_250",
                                               "aruco5x5_50", "aruco5x5_100", "aruco5x5_250",
                                               "aruco6x6_50", "aruco6x6_100", "aruco6x6_250",
                                               "aruco7x7_50", "aruco7x7_100", "aruco7x7_250"],
                                        width=23, state="readonly")
        marker_dict_combo.pack(pady=(0, 5))

        # Frame for contour-specific options
        contour_frame = ttk.LabelFrame(dialog, text="Contour Detection Options", padding="10")
        contour_frame.pack(padx=20, pady=(0, 10), fill="x")

        # Sample Index
        ttk.Label(contour_frame, text="Sample Number (1-indexed):").pack(anchor="w")
        sample_index_var = tk.StringVar(value=str(step.get("sample_index", 1)))
        sample_index_entry = ttk.Entry(contour_frame, textvariable=sample_index_var, width=25)
        sample_index_entry.pack(pady=(0, 5))

        ttk.Label(contour_frame, text="Objects sorted left-to-right, top-to-bottom (reading order)",
                 font=("Arial", 8), foreground="gray").pack(pady=(0, 5))

        # Function to enable/disable options based on detection type
        def update_detection_options(*args):
            detection_type = detection_type_var.get()
            # Marker options
            if detection_type == "marker":
                tag_id_entry.config(state="normal")
                marker_dict_combo.config(state="readonly")
            else:
                for child in marker_frame.winfo_children():
                    if isinstance(child, (ttk.Entry, ttk.Combobox)):
                        child.config(state="disabled")
            # Contour options
            if detection_type == "contour":
                sample_index_entry.config(state="normal")
            else:
                sample_index_entry.config(state="disabled")

        detection_type_var.trace('w', update_detection_options)
        update_detection_options()  # Initial state

        # Common options frame
        common_frame = ttk.LabelFrame(dialog, text="Common Options", padding="10")
        common_frame.pack(padx=20, pady=(0, 10), fill="x")

        # Z Offset (NEW)
        ttk.Label(common_frame, text="Z Offset (meters):").pack(anchor="w")
        z_offset_var = tk.StringVar(value=str(step.get("z_offset", 0.0)))
        z_offset_entry = ttk.Entry(common_frame, textvariable=z_offset_var, width=25)
        z_offset_entry.pack(pady=(0, 5))

        ttk.Label(common_frame, text="0 = use gripper default, positive = higher above object",
                 font=("Arial", 8), foreground="gray").pack(pady=(0, 10))

        # Timeout
        ttk.Label(common_frame, text="Timeout (seconds):").pack(anchor="w")
        timeout_var = tk.StringVar(value=str(step.get("timeout", 10.0)))
        timeout_entry = ttk.Entry(common_frame, textvariable=timeout_var, width=25)
        timeout_entry.pack(pady=(0, 5))

        # Information box
        info_frame = ttk.LabelFrame(dialog, text="Info", padding="10")
        info_frame.pack(padx=20, pady=(10, 20), fill="x")

        info_text = ("Vision MoveTo will:\n"
                    "1. Capture 3D point cloud using Zivid camera\n"
                    "2. Detect object (marker, circle, or contour)\n"
                    "3. Transform pose to robot base frame\n"
                    "4. Move gripper to detected position\n\n"
                    "• Circle: Hough Transform for wafers/discs\n"
                    "• Contour: Edge detection for any shape")
        ttk.Label(info_frame, text=info_text, justify="left",
                 font=("Arial", 8)).pack()

        def save_changes():
            try:
                step["detection_type"] = detection_type_var.get()
                step["tag_id"] = int(tag_id_var.get())
                step["sample_index"] = int(sample_index_var.get())
                step["timeout"] = float(timeout_var.get())
                step["z_offset"] = float(z_offset_var.get())
                step["marker_dictionary"] = marker_dict_var.get()
                self.update_task_tree()
                dialog.destroy()
                self.log_message(f"Updated step {step_index + 1}")
            except ValueError:
                messagebox.showerror("Invalid Input",
                                   "Marker ID and Sample Index must be integers, timeout and z_offset must be numbers")

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
                detection_type = step.get('detection_type', 'marker')
                tag_id = step.get('tag_id', 0)
                timeout = step.get('timeout', 10.0)
                z_offset = step.get('z_offset', 0.0)
                sample_index = step.get('sample_index', 1)
                if detection_type == "circle":
                    details = f"Detect circle/wafer"
                    if z_offset != 0.0:
                        details += f" (z_offset: {z_offset}m)"
                elif detection_type == "contour":
                    details = f"Detect contour → Sample #{sample_index}"
                    if z_offset != 0.0:
                        details += f" (z_offset: {z_offset}m)"
                else:
                    marker_dict = step.get('marker_dictionary', 'aruco4x4_50')
                    details = f"Detect ArUco {tag_id} ({marker_dict})"
                details += f" timeout: {timeout}s"
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
        self.pause_btn.config(state="normal")
        self.resume_btn.config(state="disabled")
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
        """Execute task using the MTC action client (direct ROS2 ActionClient)"""
        try:
            self.log_message("Starting MTC task execution...")

            # Check if action client is available
            if not ROS2_AVAILABLE or self.mtc_action_client is None:
                self.log_message("ERROR: ROS2 or MTC action client not available")
                return

            # Wait for action server
            self.log_message("Waiting for beambot_execution action server...")
            if not self.mtc_action_client.wait_for_server(timeout_sec=10.0):
                self.log_message("ERROR: Action server 'beambot_execution' not available!")
                self.log_message("Make sure beambot is running: ros2 launch beambot beambot_bringup.launch.py")
                return

            self.log_message("Action server available, sending goal...")

            # Create goal with JSON configuration
            goal = MTCExecution.Goal()
            goal.full_json = json.dumps(self.current_config)

            # Send goal asynchronously
            send_future = self.mtc_action_client.send_goal_async(
                goal,
                feedback_callback=self._action_feedback_callback
            )

            # Wait for goal to be accepted (with stop check)
            while not send_future.done():
                if self.stop_execution:
                    self.log_message("Execution stopped before goal was accepted")
                    return
                time.sleep(0.1)

            self.current_goal_handle = send_future.result()

            if not self.current_goal_handle.accepted:
                self.log_message("ERROR: Goal was rejected by the action server")
                return

            self.log_message("Goal accepted, executing task sequence...")

            # Wait for result (with stop check)
            result_future = self.current_goal_handle.get_result_async()

            while not result_future.done():
                if self.stop_execution:
                    self.log_message("Cancelling task execution...")
                    cancel_future = self.current_goal_handle.cancel_goal_async()
                    # Wait for cancel to complete
                    timeout = 5.0
                    start = time.time()
                    while not cancel_future.done() and (time.time() - start) < timeout:
                        time.sleep(0.1)
                    self.log_message("Task cancelled")
                    return
                time.sleep(0.1)

            # Process result
            result = result_future.result()
            status = result.status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.log_message(
                    f"✓ Task completed successfully! "
                    f"Steps: {result.result.completed_steps}/{result.result.total_steps}"
                )
                self.root.after(0, lambda: self.progress_var.set(100))
            elif status == GoalStatus.STATUS_CANCELED:
                self.log_message(f"⊗ Task was cancelled: {result.result.error_message}")
            else:
                self.log_message(
                    f"✗ Task failed: {result.result.error_message} "
                    f"(completed {result.result.completed_steps}/{result.result.total_steps} steps)"
                )

        except Exception as e:
            self.log_message(f"ERROR: {str(e)}")
            import traceback
            self.log_message(f"Traceback: {traceback.format_exc()}")
        finally:
            self.current_goal_handle = None
            # Reset GUI state
            self.root.after(0, self._reset_execution_state)

    def _action_feedback_callback(self, feedback_msg):
        """Handle action feedback from the MTC orchestrator"""
        fb = feedback_msg.feedback
        # Update progress bar
        self.root.after(0, lambda: self.progress_var.set(fb.progress_percentage))
        # Log feedback
        self.log_message(
            f"[{fb.progress_percentage:.0f}%] Step {fb.current_step}: "
            f"{fb.current_action} | Gripper: {fb.current_gripper} | {fb.status_message}"
        )

    def _reset_execution_state(self):
        """Reset execution GUI state"""
        self.execute_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.pause_btn.config(state="disabled")
        self.resume_btn.config(state="disabled")
        self.progress_var.set(0)
        self.stop_execution = False

    def stop_task(self):
        """Stop current task execution"""
        self.stop_execution = True
        self.log_message("Stopping task execution...")

    def pause_task(self):
        """Pause current task execution"""
        if not ROS2_AVAILABLE or self.pause_client is None:
            self.log_message("✗ Pause service not available")
            return

        def call_pause():
            try:
                if not self.pause_client.wait_for_service(timeout_sec=2.0):
                    self.log_message("✗ Pause service not available (timeout)")
                    return

                request = Trigger.Request()
                future = self.pause_client.call_async(request)

                # Wait for result
                start_time = time.time()
                while not future.done():
                    if time.time() - start_time > 5.0:
                        self.log_message("✗ Pause request timed out")
                        return
                    time.sleep(0.01)

                result = future.result()
                if result.success:
                    self.log_message(f"⏸ {result.message}")
                    # Update button states
                    self.root.after(0, lambda: self.pause_btn.configure(state="disabled"))
                    self.root.after(0, lambda: self.resume_btn.configure(state="normal"))
                else:
                    self.log_message(f"✗ Pause failed: {result.message}")

            except Exception as e:
                self.log_message(f"✗ Pause error: {e}")

        threading.Thread(target=call_pause, daemon=True).start()

    def resume_task(self):
        """Resume paused task execution"""
        if not ROS2_AVAILABLE or self.resume_client is None:
            self.log_message("✗ Resume service not available")
            return

        def call_resume():
            try:
                if not self.resume_client.wait_for_service(timeout_sec=2.0):
                    self.log_message("✗ Resume service not available (timeout)")
                    return

                request = Trigger.Request()
                future = self.resume_client.call_async(request)

                # Wait for result
                start_time = time.time()
                while not future.done():
                    if time.time() - start_time > 5.0:
                        self.log_message("✗ Resume request timed out")
                        return
                    time.sleep(0.01)

                result = future.result()
                if result.success:
                    self.log_message(f"▶ {result.message}")
                    # Update button states
                    self.root.after(0, lambda: self.pause_btn.configure(state="normal"))
                    self.root.after(0, lambda: self.resume_btn.configure(state="disabled"))
                else:
                    self.log_message(f"✗ Resume failed: {result.message}")

            except Exception as e:
                self.log_message(f"✗ Resume error: {e}")

        threading.Thread(target=call_resume, daemon=True).start()

    def test_mtc_server(self):
        """Test if the MTC action server is available"""
        def test_thread():
            try:
                self.log_message("Testing MTC action server availability...")

                # Check if ROS2 is available
                if not ROS2_AVAILABLE:
                    self.log_message("✗ ROS2 not available - please source your ROS2 workspace")
                    return

                # Check if action client exists
                if self.mtc_action_client is None:
                    self.log_message("✗ Action client not initialized")
                    self.log_message("This usually means ROS2 or beambot package is not available")
                    return

                self.log_message("✓ ROS2 available and action client initialized")

                # Check if beambot package is available
                try:
                    result = subprocess.run(
                        ['ros2', 'pkg', 'prefix', 'beambot'],
                        capture_output=True,
                        text=True,
                        timeout=5
                    )

                    if result.returncode == 0:
                        pkg_prefix = result.stdout.strip()
                        self.log_message(f"✓ beambot package found at: {pkg_prefix}")
                    else:
                        self.log_message("⚠ beambot package not found")
                        self.log_message("Make sure the package is built: colcon build --packages-select beambot")
                except Exception as e:
                    self.log_message(f"⚠ Could not check for beambot package: {e}")

                # Check if action server is available
                self.log_message("Checking if 'beambot_execution' action server is available...")
                if self.mtc_action_client.wait_for_server(timeout_sec=3.0):
                    self.log_message("✓ MTC action server (beambot_execution) is running!")
                else:
                    self.log_message("⚠ MTC action server is not available")
                    self.log_message("Start it with: ros2 launch beambot beambot_bringup.launch.py")

                # List available action servers for debugging
                try:
                    result = subprocess.run(
                        ['ros2', 'action', 'list'],
                        capture_output=True,
                        text=True,
                        timeout=5
                    )
                    if result.returncode == 0:
                        actions = result.stdout.strip()
                        if actions:
                            self.log_message(f"Available action servers:\n{actions}")
                        else:
                            self.log_message("No action servers currently running")
                except Exception as e:
                    self.log_message(f"⚠ Could not list action servers: {e}")

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

                # Track the current JSON file path
                self.current_json_file = file_path

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
                          "MTC Action Client GUI\n\n"
                          "A graphical interface for the MoveIt Task Constructor (MTC) pipeline.\n"
                          "Communicates directly with beambot orchestrator via ROS2 ActionClient.\n\n"
                          "Features:\n"
                          "- Visual task sequence editor\n"
                          "- Pose management\n"
                          "- Live camera view with ArUco detection\n"
                          "- Real-time execution feedback\n\n"
                          "Backend: beambot (Python MTC implementation)\n"
                          "Action server: beambot_execution\n\n"
                          "Version: 2.0")

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

        ttk.Button(button_frame, text="Detect Markers",
                  command=self.trigger_marker_detection).pack(side="left", padx=(0, 5))

        ttk.Button(button_frame, text="Detect Contours",
                  command=self.trigger_contour_detection).pack(side="left", padx=(0, 5))

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

            # Subscribe to joint states for pose capture
            from sensor_msgs.msg import JointState
            self.joint_state_sub = self.ros_node.create_subscription(
                JointState,
                '/joint_states',
                self.joint_state_callback,
                10
            )

            # Create Zivid service clients
            self.capture_client = self.ros_node.create_client(Trigger, '/capture_2d')
            self.marker_detection_client = self.ros_node.create_client(
                CaptureAndDetectMarkers,
                '/capture_and_detect_markers'
            )

            # Create MTC action client for beambot orchestrator
            self.mtc_action_client = ActionClient(
                self.ros_node,
                MTCExecution,
                'beambot_execution'
            )

            # Create pause/resume service clients
            self.pause_client = self.ros_node.create_client(Trigger, 'beambot/pause')
            self.resume_client = self.ros_node.create_client(Trigger, 'beambot/resume')

            # Detection is now manual via button (no periodic timer)

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

    def joint_state_callback(self, msg):
        """Handle incoming joint state messages"""
        try:
            import math
            # Map joint names to positions
            joint_dict = dict(zip(msg.name, msg.position))

            # UR robot joint order for JSON: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
            joint_order = [
                'shoulder_pan_joint',
                'shoulder_lift_joint',
                'elbow_joint',
                'wrist_1_joint',
                'wrist_2_joint',
                'wrist_3_joint'
            ]

            # Convert radians to degrees and round to 2 decimal places
            if all(j in joint_dict for j in joint_order):
                pose_deg = [round(math.degrees(joint_dict[j]), 2) for j in joint_order]
                self.current_robot_pose = pose_deg

        except Exception as e:
            print(f"Joint state callback error: {e}")

    def trigger_marker_detection(self):
        """Manually trigger ArUco marker detection from Zivid (called by button)"""
        if not self.marker_detection_client:
            print("Marker detection client not available")
            return

        if not self.marker_detection_client.service_is_ready():
            self.camera_status_label.config(
                text="Zivid service not ready",
                foreground="orange"
            )
            return

        try:
            # Update status
            self.camera_status_label.config(
                text="Detecting markers...",
                foreground="blue"
            )

            # Request detection for all marker IDs in aruco4x4_50 dictionary (0-49)
            request = CaptureAndDetectMarkers.Request()
            request.marker_ids = list(range(50))  # Detect IDs 0-49 (full dictionary)
            request.marker_dictionary = "aruco4x4_50"  # Default dictionary

            # Send async request
            future = self.marker_detection_client.call_async(request)
            future.add_done_callback(self.detection_response_callback)

        except Exception as e:
            print(f"Marker detection request error: {e}")
            self.camera_status_label.config(
                text=f"Detection error: {e}",
                foreground="red"
            )

    def detection_response_callback(self, future):
        """Handle Zivid marker detection response"""
        try:
            response = future.result()
            if response.success:
                # Store detected markers (cached for overlay on live stream)
                self.current_detections = response.detection_result.detected_markers

                # Update GUI in main thread
                self.root.after(0, self.update_detection_info)
                self.root.after(0, self.update_camera_display)
            else:
                print(f"Marker detection failed: {response.message}")
                self.current_detections = []
                self.root.after(0, lambda: self.camera_status_label.config(
                    text=f"Detection failed: {response.message}",
                    foreground="red"
                ))

        except Exception as e:
            print(f"Detection response error: {e}")
            self.current_detections = []
            self.root.after(0, lambda: self.camera_status_label.config(
                text=f"Detection error",
                foreground="red"
            ))

    def trigger_contour_detection(self):
        """Capture image and run contour detection with labeled visualization.

        This performs local contour detection on the current camera image,
        detecting objects by edge analysis and labeling them in reading order
        (left-to-right, top-to-bottom).
        """
        if self.current_image is None:
            self.camera_status_label.config(
                text="No image - capture first",
                foreground="orange"
            )
            messagebox.showwarning("No Image", "Please capture an image first using 'Capture Image' button")
            return

        try:
            self.camera_status_label.config(
                text="Detecting contours...",
                foreground="blue"
            )
            self.root.update()

            # Import contour detection from camera module
            from beambot.camera.zivid import ContourDetectionParams, _detect_contours_in_image

            # Run contour detection on current image
            params = ContourDetectionParams(
                min_area=500,
                max_area=50000,
                blur_kernel=5,
                canny_low=50,
                canny_high=150,
                row_tolerance=50
            )

            # Convert BGR to RGB for processing
            rgb_image = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2RGB)
            detected_contours = _detect_contours_in_image(rgb_image, params)

            if not detected_contours:
                self.camera_status_label.config(
                    text="No contours detected",
                    foreground="orange"
                )
                self.detection_info.delete('1.0', tk.END)
                self.detection_info.insert(tk.END, "No contours detected.\n\n")
                self.detection_info.insert(tk.END, "Try adjusting:\n")
                self.detection_info.insert(tk.END, "• Lighting conditions\n")
                self.detection_info.insert(tk.END, "• Object contrast with background\n")
                return

            # Draw contour visualizations with sample numbers
            display_image = self.current_image.copy()

            for i, (cx, cy, area, _) in enumerate(detected_contours):
                sample_num = i + 1

                # Draw a circle at the centroid
                cv2.circle(display_image, (cx, cy), 15, (0, 255, 0), 3)
                cv2.circle(display_image, (cx, cy), 5, (0, 255, 0), -1)

                # Draw sample number label with background
                label = f"#{sample_num}"
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 1.2
                thickness = 3
                (text_width, text_height), baseline = cv2.getTextSize(label, font, font_scale, thickness)

                # Position label above the point
                label_x = cx - text_width // 2
                label_y = cy - 25

                # Background rectangle
                cv2.rectangle(display_image,
                            (label_x - 5, label_y - text_height - 5),
                            (label_x + text_width + 5, label_y + baseline + 5),
                            (0, 200, 0), -1)

                # Text
                cv2.putText(display_image, label, (label_x, label_y),
                           font, font_scale, (0, 0, 0), thickness)

            # Update display with contour visualization
            self.current_image = display_image
            self.update_camera_display()

            # Update status
            self.camera_status_label.config(
                text=f"Detected {len(detected_contours)} contour(s)",
                foreground="green"
            )

            # Update detection info panel
            self.detection_info.delete('1.0', tk.END)
            self.detection_info.insert(tk.END, f"Contour Detection Results:\n")
            self.detection_info.insert(tk.END, f"Found {len(detected_contours)} objects (reading order)\n\n")

            for i, (cx, cy, area, _) in enumerate(detected_contours):
                sample_num = i + 1
                self.detection_info.insert(tk.END, f"Sample #{sample_num}: pixel({cx}, {cy}), area={area}px²\n")

        except ImportError as e:
            self.camera_status_label.config(
                text="Camera module not available",
                foreground="red"
            )
            messagebox.showerror("Import Error", f"Could not import contour detection: {e}")
        except Exception as e:
            self.camera_status_label.config(
                text=f"Detection error",
                foreground="red"
            )
            print(f"Contour detection error: {e}")
            import traceback
            traceback.print_exc()
            messagebox.showerror("Detection Error", f"Contour detection failed: {e}")

    def update_camera_display(self):
        """Update camera image display with ArUco marker overlays"""
        if self.current_image is None or self.camera_label is None:
            return

        try:
            # Clone image for drawing
            display_image = self.current_image.copy()

            # Draw marker overlays if we have detections
            if self.current_detections and len(self.current_detections) > 0:
                for marker in self.current_detections:
                    # Get 2D pixel corners from Zivid detection
                    corners = marker.corners_in_pixel_coordinates

                    if len(corners) == 4:
                        # Draw bounding box (green for detected markers)
                        pts = np.array([[int(c.x), int(c.y)] for c in corners], np.int32)
                        pts = pts.reshape((-1, 1, 2))
                        cv2.polylines(display_image, [pts], True, (0, 255, 0), 3)

                        # Draw tag ID at center
                        center_x = int(sum(c.x for c in corners) / 4)
                        center_y = int(sum(c.y for c in corners) / 4)

                        # Add background rectangle for text
                        text = f"ID: {marker.id}"
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
                    text=f"{len(self.current_detections)} ArUco marker(s) detected",
                    foreground="green"
                )
            else:
                self.camera_status_label.config(
                    text="No markers detected",
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

            if self.current_detections and len(self.current_detections) > 0:
                self.detection_info.insert(tk.END, f"Detected {len(self.current_detections)} ArUco marker(s):\n\n")

                for marker in self.current_detections:
                    # Get 3D position from marker pose
                    pos = marker.pose.position
                    self.detection_info.insert(tk.END,
                        f"Marker ID: {marker.id}\n"
                        f"Position (camera frame):\n"
                        f"  X: {pos.x:.3f} m\n"
                        f"  Y: {pos.y:.3f} m\n"
                        f"  Z: {pos.z:.3f} m\n"
                        f"---\n"
                    )
            else:
                self.detection_info.insert(tk.END, "No markers detected\n")

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
