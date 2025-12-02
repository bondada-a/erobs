#!/usr/bin/env python3
"""
Task Builder - Simple helper to build MTC tasks using named locations

Uses your existing pattern:
- Locations defined in task_sequences/triple_test_no_tasks.json
- Tasks built programmatically using those location names
"""

import json
import tempfile
from pathlib import Path


class TaskBuilder:
    """Build MTC task JSONs using your existing locations"""

    def __init__(self, locations_file='task_sequences/triple_test_no_tasks.json'):
        """
        Initialize with locations file

        Args:
            locations_file: Path to JSON with poses (relative to workspace root)
        """
        # Auto-detect workspace root
        try:
            if Path("/root/ws/erobs").exists():
                self.workspace = Path("/root/ws/erobs")
            else:
                self.workspace = Path.home() / "work/github_ws/erobs"
        except PermissionError:
            self.workspace = Path.home() / "work/github_ws/erobs"

        # Load locations
        loc_path = self.workspace / locations_file
        with open(loc_path) as f:
            data = json.load(f)

        self.poses = data['poses']
        self.start_gripper = data.get('start_gripper', 'hande')

        print(f"✅ Loaded {len(self.poses)} locations from {loc_path.name}")

    def list_locations(self):
        """Print all available locations"""
        print(f"\n📍 Available Locations ({len(self.poses)}):")
        for name in sorted(self.poses.keys()):
            pose = self.poses[name]
            print(f"  • {name:30s}  {pose}")

    def build_task(self, tasks, gripper=None):
        """
        Build a complete task JSON with your locations

        Args:
            tasks: List of task dictionaries (same format as triple_test.json)
            gripper: Starting gripper (default: from locations file)

        Returns:
            Path to temporary JSON file

        Example:
            tasks = [
                {"task_type": "moveto", "target": "pickup_approach", "planning_type": "joint"},
                {"task_type": "end_effector", "end_effector_type": "hande",
                 "end_effector_action": "hande_closed"}
            ]
            json_path = builder.build_task(tasks)
        """
        task_dict = {
            "start_gripper": gripper or self.start_gripper,
            "poses": self.poses,
            "tasks": tasks
        }

        # Write to temp file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json',
                                          delete=False, dir='/tmp') as f:
            json.dump(task_dict, f, indent=2)
            return f.name

    # ========== CONVENIENCE METHODS ==========

    def move_to(self, location, planning_type='joint', gripper=None):
        """
        Simple move to named location

        Args:
            location: Name from your locations (e.g., 'pickup_approach')
            planning_type: 'joint' or 'cartesian'
            gripper: Starting gripper (optional)

        Returns:
            Path to JSON file for use with simple_mtc_bluesky

        Example:
            json_file = builder.move_to('pickup_approach')
            yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': '...'})
        """
        if location not in self.poses:
            raise ValueError(f"Location '{location}' not found. Use list_locations() to see all.")

        tasks = [{
            "task_type": "moveto",
            "target": location,
            "planning_type": planning_type
        }]

        return self.build_task(tasks, gripper)

    def pick_sequence(self, approach, grasp, retreat, gripper='hande'):
        """
        Build a pick sequence using 3 locations

        Args:
            approach: Location name for approach pose
            grasp: Location name for grasp pose
            retreat: Location name for retreat pose
            gripper: Which gripper to use

        Returns:
            Path to JSON file

        Example:
            json_file = builder.pick_sequence('pickup_approach', 'pickup', 'post_pickup_camera_safety')
        """
        tasks = [
            {"task_type": "end_effector", "end_effector_type": gripper,
             "end_effector_action": f"{gripper}_open"},
            {"task_type": "moveto", "target": approach, "planning_type": "joint"},
            {"task_type": "moveto", "target": grasp, "planning_type": "cartesian"},
            {"task_type": "end_effector", "end_effector_type": gripper,
             "end_effector_action": f"{gripper}_closed"},
            {"task_type": "moveto", "target": retreat, "planning_type": "cartesian"}
        ]

        return self.build_task(tasks, gripper)

    def place_sequence(self, approach, place, retreat, gripper='hande'):
        """
        Build a place sequence using 3 locations

        Example:
            json_file = builder.place_sequence('place_approach', 'place', 'post_pickup_camera_safety')
        """
        tasks = [
            {"task_type": "moveto", "target": approach, "planning_type": "joint"},
            {"task_type": "moveto", "target": place, "planning_type": "cartesian"},
            {"task_type": "end_effector", "end_effector_type": gripper,
             "end_effector_action": f"{gripper}_open"},
            {"task_type": "moveto", "target": retreat, "planning_type": "cartesian"}
        ]

        return self.build_task(tasks, gripper)

    def tool_change(self, old_gripper, new_gripper, dock_approach='dock_approach',
                    load_approach='load_approach'):
        """
        Build tool exchange sequence

        Args:
            old_gripper: Current gripper ('hande', 'epick', 'pipettor')
            new_gripper: Target gripper
            dock_approach: Approach pose for docking
            load_approach: Approach pose for loading

        Returns:
            Path to JSON file
        """
        dock_numbers = {'hande': 2, 'epick': 3, 'pipettor': 4}

        tasks = [
            {"task_type": "tool_exchange", "operation": "dock",
             "gripper": old_gripper, "dock_number": dock_numbers[old_gripper],
             "approach_pose": dock_approach},
            {"task_type": "tool_exchange", "operation": "load",
             "gripper": new_gripper, "dock_number": dock_numbers[new_gripper],
             "approach_pose": load_approach}
        ]

        return self.build_task(tasks, old_gripper)

    # ========== GRIPPER CONTROL HELPERS ==========

    def gripper_open(self, gripper='hande'):
        """
        Open gripper

        Args:
            gripper: 'hande', 'epick', or 'pipettor'

        Returns:
            Path to JSON file

        Example:
            json_file = builder.gripper_open('hande')
            RE(bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP}))
        """
        action_map = {
            'hande': 'hande_open',
            'epick': 'vacuum_off',
            'pipettor': 'pipettor_idle'
        }

        tasks = [{
            "task_type": "end_effector",
            "end_effector_type": gripper,
            "end_effector_action": action_map[gripper]
        }]

        return self.build_task(tasks, gripper)

    def gripper_close(self, gripper='hande'):
        """
        Close gripper

        Args:
            gripper: 'hande', 'epick', or 'pipettor'

        Returns:
            Path to JSON file

        Example:
            json_file = builder.gripper_close('hande')
            RE(bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP}))
        """
        action_map = {
            'hande': 'hande_closed',
            'epick': 'vacuum_on',
            'pipettor': 'pipettor_aspirate'
        }

        tasks = [{
            "task_type": "end_effector",
            "end_effector_type": gripper,
            "end_effector_action": action_map[gripper]
        }]

        return self.build_task(tasks, gripper)

    def vacuum_on(self):
        """EPick vacuum ON - shortcut for gripper_close('epick')"""
        return self.gripper_close('epick')

    def vacuum_off(self):
        """EPick vacuum OFF - shortcut for gripper_open('epick')"""
        return self.gripper_open('epick')

    # ========== RELATIVE MOVEMENT HELPERS ==========

    def move_up(self, distance, gripper=None):
        """
        Move up (relative to current position)

        Args:
            distance: Distance in meters (e.g., 0.05 = 5cm)
            gripper: Current gripper (optional)

        Example:
            json_file = builder.move_up(0.05)  # Move up 5cm
            RE(bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP}))
        """
        tasks = [{
            "task_type": "moveto",
            "direction": "up",
            "distance": distance,
            "planning_type": "cartesian"
        }]
        return self.build_task(tasks, gripper or self.start_gripper)

    def move_down(self, distance, gripper=None):
        """Move down (relative to current position)"""
        tasks = [{
            "task_type": "moveto",
            "direction": "down",
            "distance": distance,
            "planning_type": "cartesian"
        }]
        return self.build_task(tasks, gripper or self.start_gripper)

    def move_forward(self, distance, gripper=None):
        """Move forward (relative to current position)"""
        tasks = [{
            "task_type": "moveto",
            "direction": "forward",
            "distance": distance,
            "planning_type": "cartesian"
        }]
        return self.build_task(tasks, gripper or self.start_gripper)

    def move_backward(self, distance, gripper=None):
        """Move backward (relative to current position)"""
        tasks = [{
            "task_type": "moveto",
            "direction": "backward",
            "distance": distance,
            "planning_type": "cartesian"
        }]
        return self.build_task(tasks, gripper or self.start_gripper)

    def move_left(self, distance, gripper=None):
        """Move left (relative to current position)"""
        tasks = [{
            "task_type": "moveto",
            "direction": "left",
            "distance": distance,
            "planning_type": "cartesian"
        }]
        return self.build_task(tasks, gripper or self.start_gripper)

    def move_right(self, distance, gripper=None):
        """Move right (relative to current position)"""
        tasks = [{
            "task_type": "moveto",
            "direction": "right",
            "distance": distance,
            "planning_type": "cartesian"
        }]
        return self.build_task(tasks, gripper or self.start_gripper)

    # ========== PIPETTOR HELPERS ==========

    def pipettor_suck(self, volume_pct=0.8):
        """
        Pipettor aspirate/suck

        Args:
            volume_pct: Volume percentage (0.0 to 1.0)

        Example:
            json_file = builder.pipettor_suck(0.8)  # Suck 80%
            RE(bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP}))
        """
        tasks = [{
            "task_type": "pipettor",
            "operation": "SUCK",
            "volume_pct": volume_pct
        }]
        return self.build_task(tasks, gripper='pipettor')

    def pipettor_eject(self):
        """
        Pipettor eject tip

        Example:
            json_file = builder.pipettor_eject()
            RE(bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP}))
        """
        tasks = [{
            "task_type": "pipettor",
            "operation": "EJECT_TIP",
            "volume_pct": 0.0
        }]
        return self.build_task(tasks, gripper='pipettor')


# ========== EXAMPLE USAGE ==========

if __name__ == '__main__':
    # Create builder
    builder = TaskBuilder()

    # Show available locations
    builder.list_locations()

    print("\n" + "="*60)
    print("Example 1: Simple move")
    print("="*60)
    json_file = builder.move_to('pickup_approach')
    print(f"✅ Created: {json_file}")
    print(f"   Use: yield from bps.abs_set(mtc, {{'json_file': '{json_file}', 'robot_ip': '...'}})")

    print("\n" + "="*60)
    print("Example 2: Pick sequence")
    print("="*60)
    json_file = builder.pick_sequence(
        approach='pickup_approach',
        grasp='pickup',
        retreat='post_pickup_camera_safety',
        gripper='hande'
    )
    print(f"✅ Created: {json_file}")

    print("\n" + "="*60)
    print("Example 3: Tool change")
    print("="*60)
    json_file = builder.tool_change('hande', 'epick')
    print(f"✅ Created: {json_file}")

    print("\n" + "="*60)
    print("Example 4: Custom task sequence")
    print("="*60)
    tasks = [
        {"task_type": "moveto", "target": "pickup_approach", "planning_type": "joint"},
        {"task_type": "moveto", "target": "pickup", "planning_type": "cartesian"},
        {"task_type": "end_effector", "end_effector_type": "hande",
         "end_effector_action": "hande_closed"}
    ]
    json_file = builder.build_task(tasks)
    print(f"✅ Created: {json_file}")
