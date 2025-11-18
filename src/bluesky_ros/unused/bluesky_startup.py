#!/usr/bin/env python3
"""
Bluesky Startup Script for UR5e Robot Control
==============================================

This script initializes the Bluesky environment with robot control.
After running, you'll be in an IPython shell with everything ready to use.

Usage:
    ipython -i bluesky_startup.py

Then in IPython:
    >>> RE(mv(robot, "beamline_test.json"))
    >>> RE(scan(...))
"""

import os
import sys
from pathlib import Path

# Setup paths
WORKSPACE_ROOT = Path("/home/aditya/work/github_ws/erobs")
TASK_DIR = WORKSPACE_ROOT  # Where JSON files are stored

# Initialize ROS 2
import rclpy
print("Initializing ROS 2...")
rclpy.init()

# Import Bluesky components
from bluesky import RunEngine
from bluesky.utils import install_nb_kicker
import bluesky.plans as bp
import bluesky.plan_stubs as bps
from bluesky.callbacks import LiveTable
from bluesky.callbacks.best_effort import BestEffortCallback

# Import robot device
from mtc_ophyd_device import MTCExecutionDevice

print("\n" + "="*60)
print("  Bluesky Environment for UR5e Robot Control")
print("="*60)

# Create RunEngine
print("\nCreating RunEngine...")
RE = RunEngine({})

# Install best effort callback (nice output)
bec = BestEffortCallback()
RE.subscribe(bec)

# Enable notebook mode if in Jupyter
try:
    install_nb_kicker()
    print("✓ Jupyter notebook mode enabled")
except:
    pass

# Get robot IP from environment or use default
ROBOT_IP = os.environ.get('ROBOT_IP', '10.68.82.41')
print(f"\nRobot IP: {ROBOT_IP}")

# Create robot device
print("Creating robot device...")
robot = MTCExecutionDevice(
    name="ur5e_robot",
    robot_ip=ROBOT_IP
)
print("✓ Robot device created")

# Helper function to make task paths easier
def task(filename):
    """Convert filename to full path

    Examples:
        task("beamline_test.json") -> /home/aditya/.../beamline_test.json
        task("complete_sequence.json") -> /home/aditya/.../complete_sequence.json
    """
    if os.path.isabs(filename):
        return filename
    return str(TASK_DIR / filename)

# Convenience function for moving robot
def mv(device, task_file):
    """Move robot to execute a task

    Usage:
        RE(mv(robot, "beamline_test.json"))
        RE(mv(robot, task("complete_sequence.json")))
    """
    # Convert filename to full path if needed
    if isinstance(task_file, str) and not task_file.endswith('.json'):
        task_file = task_file + '.json'

    task_path = task(task_file)

    if not os.path.exists(task_path):
        raise FileNotFoundError(f"Task file not found: {task_path}")

    return bps.abs_set(device, task_path, wait=True)

# List available tasks
print("\nAvailable task files:")
json_files = sorted(TASK_DIR.glob("*.json"))
for i, f in enumerate(json_files[:10], 1):  # Show first 10
    print(f"  {i}. {f.name}")
if len(json_files) > 10:
    print(f"  ... and {len(json_files) - 10} more")

print("\n" + "="*60)
print("  Ready! Available commands:")
print("="*60)
print("""
Devices:
    robot          - UR5e robot (MTCExecutionDevice)
    RE             - RunEngine for executing plans

Helper Functions:
    task(name)     - Get full path to task file
    mv(dev, file)  - Move device to execute task

Bluesky Plans:
    bp.count([det])            - Take readings
    bp.scan(...)               - Scanning plans
    bps.abs_set(dev, val)      - Set device value
    bps.mv(dev, val)           - Move device (alias)
    bps.sleep(seconds)         - Sleep in plan

Quick Examples:
    # Execute single task
    RE(mv(robot, "beamline_test.json"))

    # Execute with shorthand
    RE(mv(robot, "beamline_test"))  # .json added automatically

    # Multiple tasks in sequence
    RE(mv(robot, "task1.json"))
    RE(mv(robot, "task2.json"))

    # Custom plan
    def my_plan():
        yield from mv(robot, "approach.json")
        yield from bps.sleep(2)
        yield from mv(robot, "grasp.json")
    RE(my_plan())

Environment Variables:
    ROBOT_IP={ROBOT_IP}

Shutdown:
    exit()         - Exit and cleanup ROS 2
""")

# Register cleanup
import atexit

def cleanup():
    print("\nShutting down ROS 2...")
    try:
        rclpy.shutdown()
        print("✓ Cleanup complete")
    except:
        pass

atexit.register(cleanup)

print("="*60)
print()
