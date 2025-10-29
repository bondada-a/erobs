#!/usr/bin/env python3
"""Simple MTC-Bluesky Integration

This script demonstrates how to use your MTC pipeline with Bluesky
by using the existing C++ action client.
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
    """Simple MTC device that uses the existing C++ action client"""
    
    def __init__(self, name="mtc_device"):
        self.name = name
        self.get_logger = lambda: print  
    
    def set(self, task_params):
        """Execute MTC task using the existing C++ client with a JSON file

        Args:
            task_params: Dict with 'json_file' and 'robot_ip' keys
        """
        json_file_path = task_params['json_file']
        robot_ip = task_params['robot_ip']

        # Create a Status object
        status = Status()

        try:
            # Use existing C++ client with the provided JSON file
            cmd = ['ros2', 'run', 'mtc_pipeline', 'mtc_action_client_example',
                   json_file_path, robot_ip, '300']
            
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


def simple_mtc_plan(mtc_device, json_files, robot_ip):
    """Simple plan to execute MTC tasks

    Args:
        mtc_device: The MTC device
        json_files: List of JSON file paths
        robot_ip: IP address of the robot
    """

    for i, json_file in enumerate(json_files):
        print(f"Processing task {i+1}/{len(json_files)}")
        task_params = {'json_file': json_file, 'robot_ip': robot_ip}
        yield from bps.abs_set(mtc_device, task_params)
        yield from bps.wait()
        print(f"Task {i+1} completed")


def main():
    """Main function"""
    print("=== Simple MTC-Bluesky Integration ===")

    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description='Execute MTC tasks using Bluesky',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use default JSON file and env ROBOT_IP
  python3 simple_mtc_bluesky.py

  # Use specific JSON file
  python3 simple_mtc_bluesky.py beamline_test.json

  # Override robot IP
  python3 simple_mtc_bluesky.py --robot-ip 192.168.1.101

  # Multiple files with custom IP
  python3 simple_mtc_bluesky.py test1.json test2.json --robot-ip 10.0.0.5
        """
    )
    parser.add_argument(
        'json_files',
        nargs='*',
        help='JSON file(s) to execute (default: complete_sequence.json)'
    )
    parser.add_argument(
        '--robot-ip',
        default=os.environ.get('ROBOT_IP', '10.69.26.90'),
        help='Robot IP address (default: env ROBOT_IP or 10.69.26.90)'
    )

    args = parser.parse_args()

    # Initialize ROS2
    rclpy.init()

    try:
        # Create MTC device
        mtc_device = MTCDevice("mtc_device")

        # Create RunEngine
        RE = RunEngine({})

        # Docker workspace path
        WORKSPACE_ROOT = "/root/ws/erobs"

        # Process JSON file arguments
        if args.json_files:
            json_files = []
            for json_file in args.json_files:
                if os.path.isabs(json_file):
                    json_files.append(json_file)
                else:
                    json_files.append(os.path.join(WORKSPACE_ROOT, json_file))
        else:
            # Default to complete_sequence.json
            json_files = [os.path.join(WORKSPACE_ROOT, "complete_sequence.json")]

        print(f"Robot IP: {args.robot_ip}")
        print(f"JSON files: {json_files}")
        print("Executing MTC tasks...")
        RE(simple_mtc_plan(mtc_device, json_files, args.robot_ip))
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
