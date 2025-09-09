#!/usr/bin/env python3
"""Simple MTC-Bluesky Integration

This script demonstrates how to use your MTC pipeline with Bluesky
by using the existing C++ action client.
"""

import json
import subprocess
import bluesky.plan_stubs as bps
import rclpy
from bluesky import RunEngine
from ophyd.status import Status


class MTCDevice:
    """Simple MTC device that uses the existing C++ action client"""
    
    def __init__(self, name="mtc_device"):
        self.name = name
        self.get_logger = lambda: print  # Simple logger
    
    def set(self, json_file_path):
        """Execute MTC task using the existing C++ client with a JSON file"""
        # Create a Status object
        status = Status()
        
        try:
            # Use existing C++ client with the provided JSON file
            cmd = ['ros2', 'run', 'mtc_pipeline', 'mtc_action_client_example', 
                   json_file_path, '192.168.56.101', '300']
            
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
    """Simple plan to execute MTC tasks"""
    
    for i, json_file in enumerate(json_files):
        print(f"Processing task {i+1}/{len(json_files)}")
        yield from bps.abs_set(mtc_device, json_file)
        yield from bps.wait()
        print(f"Task {i+1} completed")


def main():
    """Main function"""
    print("=== Simple MTC-Bluesky Integration ===")
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create MTC device
        mtc_device = MTCDevice("mtc_device")
        
        # Create RunEngine
        RE = RunEngine({})
        
        # Use existing JSON files
        json_files = [
            "/home/aditya/work/github_ws/erobs/actions_test.json"
        ]
        
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
