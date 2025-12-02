#!/usr/bin/env python3
"""
SIMPLEST POSSIBLE EXAMPLE - TaskBuilder Usage

Just copy this and modify for your needs!
"""

import sys
sys.path.insert(0, 'src')

import rclpy
import bluesky.plan_stubs as bps
from bluesky import RunEngine
from bluesky_ros.simple_mtc_bluesky import MTCDevice
from bluesky_ros.task_builder import TaskBuilder

# ========== SETUP (Same for all scripts) ==========
rclpy.init()
RE = RunEngine({})
mtc = MTCDevice("robot")
builder = TaskBuilder()

ROBOT_IP = '10.69.26.90'  # ← Change to your robot IP


# ========== YOUR EXPERIMENT HERE ==========

def my_experiment():
    """Your Bluesky plan"""

    # Example 1: Move to a location
    print("Moving to pickup_approach...")
    json_file = builder.move_to('pickup_approach')
    yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})

    print("✅ Done!")


# ========== RUN IT ==========
if __name__ == '__main__':
    try:
        RE(my_experiment())
    except KeyboardInterrupt:
        print("\n⚠️  Stopped by user")
    finally:
        rclpy.shutdown()
