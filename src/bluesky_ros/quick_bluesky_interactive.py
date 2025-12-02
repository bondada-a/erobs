#!/usr/bin/env python3
"""
Quick Bluesky Interactive Setup
Matches your previous workflow for easy interactive use
"""

import os
import sys

# Auto-detect workspace
if os.path.exists("/root/ws/erobs"):
    WORKSPACE = "/root/ws/erobs"
else:
    WORKSPACE = os.path.expanduser("~/work/github_ws/erobs")

print("=" * 60)
print("Bluesky Interactive Setup")
print("=" * 60)
print()

# Initialize ROS 2
print("Initializing ROS 2...")
import rclpy
rclpy.init()
print("✓ ROS 2 initialized")

# Import Bluesky components
print("Importing Bluesky components...")
from bluesky import RunEngine
import bluesky.plan_stubs as bps

# Import MTCExecutionDevice (handle both local and Docker paths)
try:
    from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice
    print("✓ Imported from bluesky_ros.mtc_ophyd_device")
except ImportError:
    from mtc_ophyd_device import MTCExecutionDevice
    print("✓ Imported from mtc_ophyd_device")

print()

# Create RunEngine
print("Creating RunEngine...")
RE = RunEngine({})
print("✓ RunEngine created")
print()

# Get robot IP from environment or use default
ROBOT_IP = os.environ.get("ROBOT_IP", "10.68.82.41")

# Create robot device
print(f"Creating robot device (IP: {ROBOT_IP})...")
robot = MTCExecutionDevice(name="ur5e_robot", robot_ip=ROBOT_IP)
print("✓ Robot device created")
print()

print("=" * 60)
print("Setup Complete! Available variables:")
print("=" * 60)
print(f"  WORKSPACE = '{WORKSPACE}'")
print(f"  ROBOT_IP  = '{ROBOT_IP}'")
print(f"  RE        = RunEngine()")
print(f"  robot     = MTCExecutionDevice(name='ur5e_robot', robot_ip='{ROBOT_IP}')")
print(f"  bps       = bluesky.plan_stubs")
print()
print("Example Usage:")
print("  # Blocking execution (wait for completion)")
print("  RE(bps.abs_set(robot, 'task_sequences/complete_sequence.json', wait=True))")
print()
print("  # Non-blocking execution (returns immediately)")
print("  RE(bps.abs_set(robot, 'tool_exchange_test.json', wait=False))")
print()
print("  # Multiple tasks")
print("  def multi_task(tasks):")
print("      for task in tasks:")
print("          yield from bps.abs_set(robot, task, wait=True)")
print("  RE(multi_task(['task1.json', 'task2.json']))")
print()
print("To exit: rclpy.shutdown() or Ctrl+D")
print("=" * 60)
print()

# Enter interactive mode
import code
import readline
import rlcompleter

# Enable tab completion
readline.parse_and_bind("tab: complete")

# Prepare namespace for interactive session
namespace = {
    'WORKSPACE': WORKSPACE,
    'ROBOT_IP': ROBOT_IP,
    'RE': RE,
    'robot': robot,
    'rclpy': rclpy,
    'bps': bps,
    'MTCExecutionDevice': MTCExecutionDevice,
    'RunEngine': RunEngine,
    'os': os,
    'sys': sys,
}

# Start interactive console
try:
    code.interact(local=namespace, banner="")
except SystemExit:
    pass
finally:
    print("\nCleaning up...")
    try:
        rclpy.shutdown()
        print("✓ ROS 2 shutdown complete")
    except:
        pass
    print("Goodbye!")
