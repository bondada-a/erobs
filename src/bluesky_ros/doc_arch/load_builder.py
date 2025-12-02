"""
Load TaskBuilder into existing bsui session

This ONLY loads the builder - nothing else.
You use your existing RE, mtc, etc from bsui.

Usage from bsui:
    %run /path/to/erobs/src/bluesky_ros/load_builder.py

Then use:
    json_file = builder.move_to('pickup_approach')
    RE(bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': 'YOUR_IP'}))
"""

import sys
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
        raise RuntimeError("Workspace not found")

# Add to path
sys.path.insert(0, str(workspace / "src"))

# Import TaskBuilder
from bluesky_ros.task_builder import TaskBuilder

# Create builder
builder = TaskBuilder()

print(f"✅ TaskBuilder loaded with {len(builder.poses)} locations")
print(f"   Use: builder.move_to(), builder.pick_sequence(), etc.")
print(f"   Available: builder.list_locations()")
