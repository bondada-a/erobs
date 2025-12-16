#!/usr/bin/env python3
"""Simple MTC-Bluesky Integration

This script demonstrates how to use beambot with Bluesky
by using the beambot_client.py action client.

Backend: beambot (Python MTC implementation)
"""

import argparse
import json
import os
import sys
import subprocess
import bluesky.plan_stubs as bps
import rclpy
from bluesky import RunEngine
from ophyd.status import Status


class MTCDevice:
    """Simple MTC device that uses beambot_client.py action client"""

    def __init__(self, name="mtc_device"):
        self.name = name
        self.get_logger = lambda: print

    def set(self, task_params):
        """Execute MTC task using beambot_client.py

        Args:
            task_params: Dict with 'json_file' key (robot_ip no longer needed)
        """
        json_file_path = task_params['json_file']

        # Create a Status object
        status = Status()

        try:
            # Use beambot_client.py (only takes json file path)
            cmd = ['ros2', 'run', 'beambot', 'beambot_client.py', json_file_path]

            print(f"Executing MTC task from: {json_file_path}")

            result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)

            if result.returncode == 0:
                print("✓ MTC task completed successfully")
                status.set_finished()
            else:
                print(f"✗ MTC task failed: {result.stderr}")
                status.set_exception(Exception(f"MTC task failed: {result.stderr}"))

        except subprocess.TimeoutExpired:
            print("✗ MTC task timed out")
            status.set_exception(Exception("MTC task timed out"))
        except Exception as e:
            print(f"✗ Error running MTC task: {e}")
            status.set_exception(e)

        return status


def simple_mtc_plan(mtc_device, json_files):
    """Simple plan to execute MTC tasks

    Args:
        mtc_device: The MTC device
        json_files: List of JSON file paths
    """

    for i, json_file in enumerate(json_files):
        print(f"Processing task {i+1}/{len(json_files)}")
        task_params = {'json_file': json_file}
        yield from bps.abs_set(mtc_device, task_params)
        yield from bps.wait()
        print(f"Task {i+1} completed")


def main():
    """Main function"""
    print("=== Simple MTC-Bluesky Integration (beambot) ===")

    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description='Execute MTC tasks using Bluesky with beambot backend',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use default JSON file
  python3 simple_mtc_bluesky.py

  # Use specific JSON file
  python3 simple_mtc_bluesky.py beamline_test.json

  # Multiple files
  python3 simple_mtc_bluesky.py test1.json test2.json

Note: Robot IP is now configured in beambot beamline config, not passed as argument.
        """
    )
    parser.add_argument(
        'json_files',
        nargs='*',
        help='JSON file(s) to execute (default: complete_sequence.json)'
    )

    args = parser.parse_args()

    # Initialize ROS2
    rclpy.init()

    try:
        # Create MTC device
        mtc_device = MTCDevice("mtc_device")

        # Create RunEngine
        RE = RunEngine({})

        # Workspace path detection (priority order):
        # 1. EROBS_WORKSPACE environment variable
        # 2. Auto-detect from script location
        # 3. Search for colcon workspace markers
        WORKSPACE_ROOT = os.environ.get('EROBS_WORKSPACE')

        if not WORKSPACE_ROOT:
            # Auto-detect from script location (src/bluesky_ros/ -> workspace root)
            script_dir = os.path.dirname(os.path.abspath(__file__))
            candidate = os.path.dirname(os.path.dirname(script_dir))

            # Verify it's a valid workspace (has install/setup.bash or src/ directory)
            if os.path.exists(os.path.join(candidate, 'install', 'setup.bash')) or \
               os.path.exists(os.path.join(candidate, 'src')):
                WORKSPACE_ROOT = candidate

        if not WORKSPACE_ROOT:
            print("⚠ Could not detect workspace. Set EROBS_WORKSPACE environment variable.")
            print("  Example: export EROBS_WORKSPACE=/path/to/your/workspace")
            return

        # Process JSON file arguments
        if args.json_files:
            json_files = []
            for json_file in args.json_files:
                if os.path.isabs(json_file):
                    json_files.append(json_file)
                else:
                    json_files.append(os.path.join(WORKSPACE_ROOT, json_file))
        else:
            # Default to complete_sequence.json in cms/tasks/
            default_path = os.path.join(WORKSPACE_ROOT, "src/cms/tasks/complete_sequence.json")
            if os.path.exists(default_path):
                json_files = [default_path]
            else:
                print(f"⚠ Default task file not found: {default_path}")
                print("Please specify a JSON file path")
                return

        print(f"Workspace: {WORKSPACE_ROOT}")
        print(f"JSON files: {json_files}")
        print("Executing MTC tasks...")
        RE(simple_mtc_plan(mtc_device, json_files))
        print("✓ All tasks completed")

    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        rclpy.shutdown()
        print("✓ Done!")


if __name__ == "__main__":
    main()
