#!/usr/bin/env python3
"""
Interactive Bluesky Shell with TaskBuilder

Start this, then use TaskBuilder commands interactively!
"""

import sys
sys.path.insert(0, 'src')

import rclpy
import bluesky.plan_stubs as bps
from bluesky import RunEngine
from bluesky_ros.simple_mtc_bluesky import MTCDevice
from bluesky_ros.task_builder import TaskBuilder

# Initialize
print("🚀 Starting Interactive Bluesky Shell...")
rclpy.init()

# Create devices
RE = RunEngine({})
mtc = MTCDevice("robot")
builder = TaskBuilder()

# Robot IP - CHANGE THIS!
ROBOT_IP = '10.69.26.90'

print("\n" + "="*60)
print("✅ Ready! You can now use:")
print("="*60)
print("  RE        - RunEngine")
print("  mtc       - MTC Device")
print("  builder   - TaskBuilder")
print("  ROBOT_IP  - Robot IP address")
print()
print("📍 Available locations:", builder.list_locations())
print()
print("Example commands:")
print("  json_file = builder.move_to('pickup_approach')")
print("  RE(bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP}))")
print()
print("Or create a plan:")
print("  def test():")
print("      json_file = builder.move_to('pickup_approach')")
print("      yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})")
print("  RE(test())")
print("="*60)

# Start IPython shell
try:
    from IPython import embed
    embed(colors='neutral')
except ImportError:
    import code
    code.interact(local=locals())
finally:
    rclpy.shutdown()
    print("\n✅ Shutdown complete")
