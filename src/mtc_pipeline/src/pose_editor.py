#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, messagebox
import json

class PoseEditor:
    """Dialog for editing robot poses"""
    
    def __init__(self, parent, pose_name="", pose_values=None):
        self.parent = parent
        self.pose_name = pose_name
        self.pose_values = pose_values or [0.0, -90.0, -90.0, -90.0, 90.0, 0.0]
        self.result = None
        
        self.create_dialog()
    
    def create_dialog(self):
        """Create the pose editor dialog"""
        self.dialog = tk.Toplevel(self.parent)
        self.dialog.title(f"Edit Pose: {self.pose_name}" if self.pose_name else "Edit Pose")
        self.dialog.geometry("400x500")
        self.dialog.transient(self.parent)
        try:
            self.dialog.grab_set()
        except:
            pass  # Ignore grab errors
        
        # Pose name
        name_frame = ttk.Frame(self.dialog)
        name_frame.pack(fill="x", padx=20, pady=(20, 10))
        
        ttk.Label(name_frame, text="Pose Name:").pack(side="left")
        self.name_var = tk.StringVar(value=self.pose_name)
        name_entry = ttk.Entry(name_frame, textvariable=self.name_var, width=20)
        name_entry.pack(side="left", padx=(10, 0))
        
        # Joint values
        joints_frame = ttk.LabelFrame(self.dialog, text="Joint Values (degrees)", padding="20")
        joints_frame.pack(fill="x", padx=20, pady=10)
        
        self.joint_vars = []
        joint_names = ["Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"]
        
        for i, (name, value) in enumerate(zip(joint_names, self.pose_values)):
            joint_frame = ttk.Frame(joints_frame)
            joint_frame.pack(fill="x", pady=2)
            
            ttk.Label(joint_frame, text=f"{name}:").pack(side="left")
            
            var = tk.DoubleVar(value=value)
            self.joint_vars.append(var)
            
            entry = ttk.Entry(joint_frame, textvariable=var, width=15)
            entry.pack(side="left", padx=(10, 0))
            
            # Add increment/decrement buttons
            ttk.Button(joint_frame, text="-", width=3, 
                      command=lambda v=var: self.adjust_joint(v, -1)).pack(side="left", padx=(5, 0))
            ttk.Button(joint_frame, text="+", width=3, 
                      command=lambda v=var: self.adjust_joint(v, 1)).pack(side="left", padx=(2, 0))
        
        # Preset poses
        presets_frame = ttk.LabelFrame(self.dialog, text="Preset Poses", padding="20")
        presets_frame.pack(fill="x", padx=20, pady=10)
        
        preset_buttons = ttk.Frame(presets_frame)
        preset_buttons.pack(fill="x")
        
        presets = {
            "Home": [0.0, -90.0, -90.0, -90.0, 90.0, 0.0],
            "Pick": [45.0, -90.0, -90.0, -90.0, 90.0, 0.0],
            "Place": [-45.0, -90.0, -90.0, -90.0, 90.0, 0.0],
            "Up": [0.0, -45.0, -90.0, -90.0, 90.0, 0.0],
            "Down": [0.0, -135.0, -90.0, -90.0, 90.0, 0.0]
        }
        
        for i, (name, values) in enumerate(presets.items()):
            btn = ttk.Button(preset_buttons, text=name, width=8,
                           command=lambda v=values: self.load_preset(v))
            btn.grid(row=i//3, column=i%3, padx=2, pady=2)
        
        # Buttons
        button_frame = ttk.Frame(self.dialog)
        button_frame.pack(fill="x", padx=20, pady=20)
        
        ttk.Button(button_frame, text="OK", command=self.ok_clicked).pack(side="right", padx=(5, 0))
        ttk.Button(button_frame, text="Cancel", command=self.cancel_clicked).pack(side="right")
        
        # Center dialog on parent
        self.dialog.update_idletasks()
        x = (self.dialog.winfo_screenwidth() // 2) - (self.dialog.winfo_width() // 2)
        y = (self.dialog.winfo_screenheight() // 2) - (self.dialog.winfo_height() // 2)
        self.dialog.geometry(f"+{x}+{y}")
        
        # Focus on first entry
        name_entry.focus_set()
    
    def adjust_joint(self, var, delta):
        """Adjust joint value by delta"""
        current = var.get()
        var.set(round(current + delta, 2))
    
    def load_preset(self, values):
        """Load preset pose values"""
        for i, value in enumerate(values):
            if i < len(self.joint_vars):
                self.joint_vars[i].set(value)
    
    def ok_clicked(self):
        """Handle OK button click"""
        try:
            # Validate pose name
            pose_name = self.name_var.get().strip()
            if not pose_name:
                messagebox.showerror("Error", "Pose name cannot be empty")
                return
            
            # Get joint values
            pose_values = [var.get() for var in self.joint_vars]
            
            # Validate values
            for i, value in enumerate(pose_values):
                if not isinstance(value, (int, float)):
                    messagebox.showerror("Error", f"Invalid value for joint {i+1}")
                    return
            
            self.result = {
                "name": pose_name,
                "values": pose_values
            }
            
            self.dialog.destroy()
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save pose: {str(e)}")
    
    def cancel_clicked(self):
        """Handle Cancel button click"""
        self.dialog.destroy()
    
    def show(self):
        """Show the dialog and return result"""
        self.dialog.wait_window()
        return self.result

class PoseManager:
    """Manager for handling pose operations"""
    
    @staticmethod
    def edit_pose(parent, pose_name="", pose_values=None):
        """Open pose editor dialog"""
        editor = PoseEditor(parent, pose_name, pose_values)
        return editor.show()
    
    @staticmethod
    def format_pose_for_display(pose_values):
        """Format pose values for display"""
        if not pose_values:
            return "No pose"
        
        formatted = []
        for value in pose_values:
            if isinstance(value, (int, float)):
                formatted.append(f"{value:.2f}")
            else:
                formatted.append(str(value))
        
        return f"[{', '.join(formatted)}]"
    
    @staticmethod
    def validate_pose_values(pose_values):
        """Validate pose values"""
        if not isinstance(pose_values, list):
            return False, "Pose values must be a list"
        
        if len(pose_values) != 6:
            return False, "Pose must have exactly 6 joint values"
        
        for i, value in enumerate(pose_values):
            if not isinstance(value, (int, float)):
                return False, f"Joint {i+1} value must be a number"
        
        return True, "Valid pose"
