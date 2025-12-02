"""
Load Robot TaskBuilder into existing bsui session

Usage from bsui IPython shell:
    %run /path/to/erobs/src/bluesky_ros/load_robot.py

Or:
    exec(open('/path/to/erobs/src/bluesky_ros/load_robot.py').read())
"""

import sys
import os
from pathlib import Path

# Auto-detect workspace
workspace = None
try:
    if Path("/root/ws/erobs").exists():
        workspace = Path("/root/ws/erobs")
except PermissionError:
    pass

if workspace is None:
    if Path.home().joinpath("work/github_ws/erobs").exists():
        workspace = Path.home() / "work/github_ws/erobs"
    else:
        print("❌ Error: Cannot find erobs workspace")
        print("   Looking for: ~/work/github_ws/erobs or /root/ws/erobs")
        sys.exit(1)

# Add to path
sys.path.insert(0, str(workspace / "src"))

# Import
try:
    import rclpy
    from bluesky_ros.simple_mtc_bluesky import MTCDevice
    from bluesky_ros.task_builder import TaskBuilder

    # Initialize ROS if not already initialized
    try:
        rclpy.init()
    except:
        pass  # Already initialized

    # Create devices
    mtc = MTCDevice("robot")
    builder = TaskBuilder()

    # Set default robot IP (users can change this)
    ROBOT_IP = os.environ.get('ROBOT_IP', '10.69.26.90')

    print("\n" + "="*60)
    print("✅ Robot Control Loaded!")
    print("="*60)
    print(f"📍 Loaded {len(builder.poses)} locations")
    print(f"🤖 Robot IP: {ROBOT_IP} (change with: ROBOT_IP = 'your_ip')")
    print()
    print("Available objects:")
    print("  • builder  - TaskBuilder instance")
    print("  • mtc      - MTC Device")
    print("  • ROBOT_IP - Robot IP address")
    print()
    print("Quick commands:")
    print("  builder.list_locations()  # Show all locations")
    print()
    print("Example usage:")
    print("  json_file = builder.move_to('pickup_approach')")
    print("  RE(bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP}))")
    print("="*60 + "\n")

except Exception as e:
    print(f"❌ Error loading robot control: {e}")
    import traceback
    traceback.print_exc()
