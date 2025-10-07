#!/usr/bin/env python3
"""Simple example of MTC + Bluesky integration"""

import sys
from pathlib import Path

# Add src directory to Python path
src_path = Path(__file__).parent.parent
sys.path.insert(0, str(src_path))

import rclpy
import bluesky.plan_stubs as bps
from bluesky import RunEngine
from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice


def single_task_plan(mtc_device, json_file):
    """Execute a single MTC task"""
    print(f"Executing task from: {json_file}")
    yield from bps.abs_set(mtc_device, json_file, wait=True)
    print("Task complete")



def main():
    # Initialize ROS2
    rclpy.init()

    try:
        # Create MTC device
        mtc = MTCExecutionDevice(
            name="mtc_executor",
            robot_ip="10.68.82.41"
        )

        # Create Bluesky RunEngine
        RE = RunEngine({})

        # Example 1: Single task
        RE(single_task_plan(mtc, "/root/ws/erobs/beamline_test.json"))

        # Example 2: Multiple tasks
        # tasks = ["/path/to/task1.json", "/path/to/task2.json"]
        # RE(multi_task_plan(mtc, tasks))


    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
