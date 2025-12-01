#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import json
import os


class SaveCurrentPoseDialog:
    """Dialog for saving the current robot pose"""

    def __init__(self, parent, current_pose, current_config, current_json_file=None):
        self.parent = parent
        self.current_pose = current_pose  # List of 6 joint values in degrees
        self.current_config = current_config
        self.current_json_file = current_json_file
        self.result = None

        self.create_dialog()

    def create_dialog(self):
        """Create the save current pose dialog"""
        self.dialog = tk.Toplevel(self.parent)
        self.dialog.title("Save Current Robot Pose")
        self.dialog.geometry("550x750")  # Increased height to ensure buttons are visible
        self.dialog.minsize(550, 750)  # Set minimum size
        self.dialog.transient(self.parent)
        try:
            self.dialog.grab_set()
        except:
            pass  # Ignore grab errors

        # Configure grid
        self.dialog.grid_columnconfigure(0, weight=1)

        # Title
        title_label = ttk.Label(self.dialog, text="Save Current Robot Pose",
                               font=("Arial", 14, "bold"))
        title_label.pack(pady=(20, 10))

        # Current pose display
        pose_frame = ttk.LabelFrame(self.dialog, text="Current Robot Pose", padding="15")
        pose_frame.pack(fill="x", padx=20, pady=(0, 15))

        # Display joint values
        joint_names = ["Base (Shoulder Pan)", "Shoulder Lift", "Elbow",
                      "Wrist 1", "Wrist 2", "Wrist 3"]

        for i, (name, value) in enumerate(zip(joint_names, self.current_pose)):
            row_frame = ttk.Frame(pose_frame)
            row_frame.pack(fill="x", pady=2)

            ttk.Label(row_frame, text=f"{name}:", width=20,
                     anchor="w").pack(side="left")
            ttk.Label(row_frame, text=f"{value:.2f}°",
                     font=("Arial", 10, "bold"),
                     foreground="blue").pack(side="left", padx=(10, 0))

        # Visual representation (simplified)
        visual_text = self.get_pose_description(self.current_pose)
        ttk.Label(pose_frame, text=f"\nPose Description: {visual_text}",
                 font=("Arial", 9, "italic"),
                 foreground="gray").pack(pady=(10, 0))

        # Pose name input
        name_frame = ttk.LabelFrame(self.dialog, text="Pose Name", padding="15")
        name_frame.pack(fill="x", padx=20, pady=(0, 15))

        ttk.Label(name_frame, text="Enter a name for this pose:").pack(anchor="w", pady=(0, 5))
        self.pose_name_var = tk.StringVar(value="")
        name_entry = ttk.Entry(name_frame, textvariable=self.pose_name_var, width=40)
        name_entry.pack(fill="x")
        name_entry.focus_set()

        # Help text
        ttk.Label(name_frame, text="Examples: pickup_approach, dock_position, home_custom",
                 font=("Arial", 8), foreground="gray").pack(anchor="w", pady=(5, 0))

        # Save options
        options_frame = ttk.LabelFrame(self.dialog, text="Save Options", padding="15")
        options_frame.pack(fill="x", padx=20, pady=(0, 15))

        self.save_action = tk.StringVar(value="add_to_config")

        # Option 1: Add to current configuration
        rb1 = ttk.Radiobutton(options_frame, text="Add to current configuration (in memory)",
                             variable=self.save_action, value="add_to_config")
        rb1.pack(anchor="w", pady=2)
        ttk.Label(options_frame, text="  → Adds pose to current config without saving to disk",
                 font=("Arial", 8), foreground="gray").pack(anchor="w", padx=(20, 0))

        # Option 2: Save to current JSON file
        if self.current_json_file:
            rb2 = ttk.Radiobutton(options_frame,
                                 text=f"Save to current JSON file",
                                 variable=self.save_action, value="save_to_current")
            rb2.pack(anchor="w", pady=(10, 2))

            # Show current file path
            file_name = os.path.basename(self.current_json_file)
            ttk.Label(options_frame,
                     text=f"  → Updates: {file_name}",
                     font=("Arial", 8), foreground="gray").pack(anchor="w", padx=(20, 0))
        else:
            ttk.Label(options_frame, text="Save to current JSON file (no file loaded)",
                     foreground="gray", state="disabled").pack(anchor="w", pady=(10, 2))

        # Option 3: Save to new JSON file
        rb3 = ttk.Radiobutton(options_frame, text="Save to new JSON file",
                             variable=self.save_action, value="save_to_new")
        rb3.pack(anchor="w", pady=(10, 2))
        ttk.Label(options_frame, text="  → Creates a new JSON file with this pose",
                 font=("Arial", 8), foreground="gray").pack(anchor="w", padx=(20, 0))

        # Buttons - Make them more prominent and always visible at bottom
        button_frame = ttk.Frame(self.dialog)
        button_frame.pack(fill="x", padx=20, pady=(15, 25), side="bottom")

        save_btn = ttk.Button(button_frame, text="Save Pose",
                             command=self.save_clicked)
        save_btn.pack(side="right", padx=(5, 0), ipadx=10, ipady=5)

        cancel_btn = ttk.Button(button_frame, text="Cancel",
                               command=self.cancel_clicked)
        cancel_btn.pack(side="right", ipadx=10, ipady=5)

        # Center dialog
        self.dialog.update_idletasks()
        x = (self.dialog.winfo_screenwidth() // 2) - (self.dialog.winfo_width() // 2)
        y = (self.dialog.winfo_screenheight() // 2) - (self.dialog.winfo_height() // 2)
        self.dialog.geometry(f"+{x}+{y}")

    def get_pose_description(self, pose_values):
        """Get a human-readable description of the pose"""
        if not pose_values or len(pose_values) != 6:
            return "Invalid pose"

        base, shoulder, elbow, wrist1, wrist2, wrist3 = pose_values

        # Check for common pose patterns
        if abs(base) < 5 and abs(shoulder + 90) < 5 and abs(elbow + 90) < 5:
            if abs(wrist1 + 90) < 5 and abs(wrist2 - 90) < 5 and abs(wrist3) < 5:
                return "Home-like position"

        if abs(shoulder + 90) < 5 and abs(elbow + 90) < 5:
            if base > 30:
                return "Right side position"
            elif base < -30:
                return "Left side position"

        if abs(shoulder + 45) < 5:
            return "Upper/raised position"
        elif abs(shoulder + 135) < 5:
            return "Lower/lowered position"

        return "Custom position"

    def save_clicked(self):
        """Handle Save button click"""
        try:
            # Validate pose name
            pose_name = self.pose_name_var.get().strip()
            if not pose_name:
                messagebox.showerror("Error", "Please enter a name for the pose")
                return

            # Check if pose already exists
            if pose_name in self.current_config.get("poses", {}):
                if not messagebox.askyesno("Pose Exists",
                                         f"Pose '{pose_name}' already exists. Overwrite?"):
                    return

            action = self.save_action.get()

            # Handle save to new file action
            if action == "save_to_new":
                file_path = filedialog.asksaveasfilename(
                    title="Save Pose to New JSON File",
                    defaultextension=".json",
                    filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                    initialfile=f"{pose_name}.json"
                )

                if not file_path:
                    return  # User cancelled

                self.result = {
                    "action": action,
                    "pose_name": pose_name,
                    "pose_values": self.current_pose,
                    "file_path": file_path
                }
            else:
                self.result = {
                    "action": action,
                    "pose_name": pose_name,
                    "pose_values": self.current_pose
                }

            self.dialog.destroy()

        except Exception as e:
            messagebox.showerror("Error", f"Failed to save pose: {str(e)}")

    def cancel_clicked(self):
        """Handle Cancel button click"""
        self.result = None
        self.dialog.destroy()

    def show(self):
        """Show the dialog and return result"""
        self.dialog.wait_window()
        return self.result


def main():
    """Test the save current pose dialog"""
    root = tk.Tk()
    root.withdraw()  # Hide main window

    # Test data
    test_pose = [45.0, -90.0, -90.0, -90.0, 90.0, 0.0]
    test_config = {
        "start_gripper": "epick",
        "poses": {
            "home": [0.0, -90.0, -90.0, -90.0, 90.0, 0.0]
        },
        "tasks": []
    }
    test_file = "/tmp/test_config.json"

    dialog = SaveCurrentPoseDialog(root, test_pose, test_config, test_file)
    result = dialog.show()

    if result:
        print("Save result:")
        print(f"  Action: {result['action']}")
        print(f"  Pose name: {result['pose_name']}")
        print(f"  Pose values: {result['pose_values']}")
        if 'file_path' in result:
            print(f"  File path: {result['file_path']}")

    root.destroy()


if __name__ == '__main__':
    main()
