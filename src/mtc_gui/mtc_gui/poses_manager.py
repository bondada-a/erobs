#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, messagebox
from .pose_editor import PoseManager
import json

class PosesManager:
    """Dialog for managing robot poses"""
    
    def __init__(self, parent, poses_dict=None):
        self.parent = parent
        self.poses_dict = poses_dict or {}
        self.result = None
        
        self.create_dialog()
    
    def create_dialog(self):
        """Create the poses manager dialog"""
        self.dialog = tk.Toplevel(self.parent)
        self.dialog.title("Manage Poses")
        self.dialog.geometry("600x500")
        self.dialog.transient(self.parent)
        self.dialog.grab_set()
        
        # Configure grid
        self.dialog.grid_columnconfigure(0, weight=1)
        self.dialog.grid_rowconfigure(1, weight=1)
        
        # Title
        title_label = ttk.Label(self.dialog, text="Robot Poses Management", 
                               font=("Arial", 14, "bold"))
        title_label.grid(row=0, column=0, pady=(20, 10))
        
        # Buttons frame
        buttons_frame = ttk.Frame(self.dialog)
        buttons_frame.grid(row=1, column=0, sticky="ew", padx=20, pady=(0, 10))
        buttons_frame.grid_columnconfigure(1, weight=1)
        
        ttk.Button(buttons_frame, text="Add New Pose", 
                  command=self.add_new_pose).pack(side="left", padx=(0, 10))
        ttk.Button(buttons_frame, text="Edit Selected", 
                  command=self.edit_selected_pose).pack(side="left", padx=(0, 10))
        ttk.Button(buttons_frame, text="Delete Selected", 
                  command=self.delete_selected_pose).pack(side="left", padx=(0, 10))
        ttk.Button(buttons_frame, text="Import Poses", 
                  command=self.import_poses).pack(side="left", padx=(20, 0))
        
        # Poses list
        list_frame = ttk.LabelFrame(self.dialog, text="Available Poses", padding="10")
        list_frame.grid(row=2, column=0, sticky="nsew", padx=20, pady=(0, 10))
        list_frame.grid_columnconfigure(0, weight=1)
        list_frame.grid_rowconfigure(0, weight=1)
        
        # Create treeview for poses
        columns = ("Pose Name", "Joint Values", "Description")
        self.poses_tree = ttk.Treeview(list_frame, columns=columns, show="tree headings", height=15)
        
        # Configure columns
        self.poses_tree.heading("#0", text="#")
        self.poses_tree.heading("Pose Name", text="Pose Name")
        self.poses_tree.heading("Joint Values", text="Joint Values")
        self.poses_tree.heading("Description", text="Description")
        
        self.poses_tree.column("#0", width=40)
        self.poses_tree.column("Pose Name", width=150)
        self.poses_tree.column("Joint Values", width=200)
        self.poses_tree.column("Description", width=150)
        
        # Scrollbar
        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=self.poses_tree.yview)
        self.poses_tree.configure(yscrollcommand=scrollbar.set)
        
        # Grid layout
        self.poses_tree.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        
        # Bind double-click to edit
        self.poses_tree.bind("<Double-1>", self.edit_selected_pose)
        
        # Bottom buttons
        bottom_frame = ttk.Frame(self.dialog)
        bottom_frame.grid(row=3, column=0, pady=20)
        
        ttk.Button(bottom_frame, text="OK", command=self.ok_clicked).pack(side="right", padx=(5, 0))
        ttk.Button(bottom_frame, text="Cancel", command=self.cancel_clicked).pack(side="right")
        
        # Load existing poses
        self.refresh_poses_list()
        
        # Center dialog
        self.dialog.update_idletasks()
        x = (self.dialog.winfo_screenwidth() // 2) - (self.dialog.winfo_width() // 2)
        y = (self.dialog.winfo_screenheight() // 2) - (self.dialog.winfo_height() // 2)
        self.dialog.geometry(f"+{x}+{y}")
    
    def refresh_poses_list(self):
        """Refresh the poses list display"""
        # Clear existing items
        for item in self.poses_tree.get_children():
            self.poses_tree.delete(item)
        
        # Add poses
        for i, (name, values) in enumerate(self.poses_dict.items()):
            item_id = str(i + 1)
            
            # Format joint values for display
            if isinstance(values, list) and len(values) == 6:
                joint_str = f"[{', '.join([f'{v:.2f}' for v in values])}]"
            else:
                joint_str = str(values)
            
            # Create description
            description = self.get_pose_description(name, values)
            
            self.poses_tree.insert("", "end", iid=item_id, text=item_id,
                                 values=(name, joint_str, description))
    
    def get_pose_description(self, name, values):
        """Get a human-readable description of the pose"""
        if not isinstance(values, list) or len(values) != 6:
            return "Invalid pose"
        
        # Check for common pose patterns
        base, shoulder, elbow, wrist1, wrist2, wrist3 = values
        
        if abs(base) < 5 and abs(shoulder + 90) < 5 and abs(elbow + 90) < 5:
            if abs(wrist1 + 90) < 5 and abs(wrist2 - 90) < 5 and abs(wrist3) < 5:
                return "Home position"
        
        if abs(shoulder + 90) < 5 and abs(elbow + 90) < 5:
            if base > 30:
                return "Right side position"
            elif base < -30:
                return "Left side position"
        
        if abs(shoulder + 45) < 5:
            return "Upper position"
        elif abs(shoulder + 135) < 5:
            return "Lower position"
        
        return "Custom position"
    
    def add_new_pose(self):
        """Add a new pose"""
        result = PoseManager.edit_pose(self.dialog)
        if result:
            pose_name = result["name"]
            pose_values = result["values"]
            
            if pose_name in self.poses_dict:
                if not messagebox.askyesno("Pose Exists", 
                                         f"Pose '{pose_name}' already exists. Overwrite?"):
                    return
            
            self.poses_dict[pose_name] = pose_values
            self.refresh_poses_list()
    
    def edit_selected_pose(self, event=None):
        """Edit the selected pose"""
        selection = self.poses_tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a pose to edit")
            return
        
        item = selection[0]
        pose_index = int(item) - 1
        pose_names = list(self.poses_dict.keys())
        
        if 0 <= pose_index < len(pose_names):
            pose_name = pose_names[pose_index]
            pose_values = self.poses_dict[pose_name]
            
            result = PoseManager.edit_pose(self.dialog, pose_name, pose_values)
            if result:
                new_name = result["name"]
                new_values = result["values"]
                
                # Remove old entry if name changed
                if new_name != pose_name:
                    del self.poses_dict[pose_name]
                
                self.poses_dict[new_name] = new_values
                self.refresh_poses_list()
    
    def delete_selected_pose(self):
        """Delete the selected pose"""
        selection = self.poses_tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a pose to delete")
            return
        
        item = selection[0]
        pose_index = int(item) - 1
        pose_names = list(self.poses_dict.keys())
        
        if 0 <= pose_index < len(pose_names):
            pose_name = pose_names[pose_index]
            
            if messagebox.askyesno("Confirm Delete", 
                                 f"Are you sure you want to delete pose '{pose_name}'?"):
                del self.poses_dict[pose_name]
                self.refresh_poses_list()
    
    def import_poses(self):
        """Import poses from JSON file"""
        from tkinter import filedialog
        
        file_path = filedialog.askopenfilename(
            title="Import Poses from JSON",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if file_path:
            try:
                with open(file_path, 'r') as f:
                    data = json.load(f)
                
                if isinstance(data, dict) and "poses" in data:
                    poses = data["poses"]
                    if isinstance(poses, dict):
                        # Merge poses
                        for name, values in poses.items():
                            if isinstance(values, list) and len(values) == 6:
                                self.poses_dict[name] = values
                        
                        self.refresh_poses_list()
                        messagebox.showinfo("Success", f"Imported {len(poses)} poses")
                    else:
                        messagebox.showerror("Error", "Invalid poses format in file")
                else:
                    messagebox.showerror("Error", "File does not contain valid poses data")
                    
            except Exception as e:
                messagebox.showerror("Error", f"Failed to import poses: {str(e)}")
    
    def ok_clicked(self):
        """Handle OK button click"""
        self.result = self.poses_dict.copy()
        self.dialog.destroy()
    
    def cancel_clicked(self):
        """Handle Cancel button click"""
        self.dialog.destroy()
    
    def show(self):
        """Show the dialog and return result"""
        self.dialog.wait_window()
        return self.result

def main():
    """Test the poses manager"""
    root = tk.Tk()
    root.withdraw()  # Hide main window
    
    # Test data
    test_poses = {
        "home": [0.0, -90.0, -90.0, -90.0, 90.0, 0.0],
        "pickup_approach": [45.0, -90.0, -90.0, -90.0, 90.0, 0.0],
        "pickup": [45.0, -90.0, -90.0, -90.0, 90.0, 0.0]
    }
    
    manager = PosesManager(root, test_poses)
    result = manager.show()
    
    if result:
        print("Updated poses:")
        for name, values in result.items():
            print(f"  {name}: {values}")
    
    root.destroy()

if __name__ == '__main__':
    main()
