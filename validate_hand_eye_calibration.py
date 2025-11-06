#!/usr/bin/env python3
"""
Validation script for Zivid hand-eye calibration in ROS 2.

This script provides instructions and commands to validate the hand-eye calibration
after applying the calibrated transform to the URDF.
"""

import subprocess
import time
from pathlib import Path


def print_header(title):
    """Print formatted section header."""
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)


def run_command(cmd, description):
    """Run a shell command and display the output."""
    print(f"\n{description}")
    print(f"Command: {cmd}")
    print("-" * 40)
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if result.returncode == 0:
            print("✓ Success")
            if result.stdout:
                print(result.stdout[:500])  # Limit output
        else:
            print("✗ Failed")
            if result.stderr:
                print(result.stderr[:500])
        return result.returncode == 0
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def main():
    """Main validation workflow."""
    print_header("HAND-EYE CALIBRATION VALIDATION")

    print("""
This script will guide you through validating your hand-eye calibration.
Make sure your robot system is running before proceeding.
    """)

    input("Press Enter to start validation...")

    # Step 1: Source workspace
    print_header("Step 1: Source ROS 2 Workspace")
    print("""
Please run in your terminal:
source /opt/ros/humble/setup.bash
source /home/aditya/work/github_ws/erobs/install/setup.bash
    """)
    input("Press Enter after sourcing the workspace...")

    # Step 2: Check TF tree
    print_header("Step 2: Verify TF Tree")
    print("""
The calibrated transform should appear in the TF tree.
Run this command in a new terminal to save the TF tree:

ros2 run tf2_tools view_frames

This will create a PDF file 'frames.pdf' showing the TF tree.
Look for the connection: flange → zivid_optical_frame
    """)

    # Step 3: Launch robot system
    print_header("Step 3: Launch Robot System (if not running)")
    print("""
If your robot system is not already running, launch it:

ros2 launch ur5e_moveit_configs robot_bringup.launch.py

This will load the updated URDF with calibrated transforms.
    """)
    input("Press Enter when robot system is running...")

    # Step 4: Test with static transform publisher
    print_header("Step 4: Test Static Transform Publisher")
    print("""
To compare with a test frame, run this command:

ros2 run tf2_ros static_transform_publisher \\
  --x -0.05435 --y -0.10490 --z -0.19139 \\
  --qx -0.01493 --qy 0.02668 --qz -0.00867 --qw 0.99949 \\
  --frame-id flange --child-frame-id zivid_optical_frame_test

Then check the transform:
ros2 run tf2_ros tf2_echo flange zivid_optical_frame
ros2 run tf2_ros tf2_echo flange zivid_optical_frame_test

Both should show similar transforms.
    """)

    # Step 5: ArUco marker detection test
    print_header("Step 5: ArUco Marker Detection Test")
    print("""
VALIDATION TEST PROCEDURE:

1. Place an ArUco marker at a known position in the workspace
2. Ensure the marker is visible to the Zivid camera
3. Call the capture and detect service:

   ros2 service call /capture_and_detect_markers std_srvs/srv/Trigger

4. Check the detected marker pose in the base_link frame
5. Move the robot TCP to the detected position (be careful!)
6. Verify the robot reaches the marker accurately

EXPECTED ACCURACY:
- With good calibration: ±1-2mm position accuracy
- With poor calibration: >5mm position error

TROUBLESHOOTING:
- If marker is not detected: Check camera exposure settings
- If position is off: Verify calibration values in URDF
- If TF fails: Check all frames are connected in TF tree
    """)

    # Step 6: Manual verification with RViz
    print_header("Step 6: Visual Verification in RViz")
    print("""
Launch RViz to visualize the transforms:

ros2 run rviz2 rviz2

In RViz:
1. Add Robot Model display
2. Add TF display
3. Add PointCloud2 display (topic: /zivid/points)
4. Set Fixed Frame to "base_link"
5. Enable display of frame names in TF
6. Verify zivid_optical_frame is correctly positioned

The camera frustum should align with the actual camera position.
    """)

    # Step 7: Python test script
    print_header("Step 7: Transform Verification Script")
    print("""
Here's a Python script to verify transforms programmatically:
    """)

    test_script = '''
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

class TransformVerifier(Node):
    def __init__(self):
        super().__init__('transform_verifier')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.check_transform)

    def check_transform(self):
        try:
            # Get transform from flange to optical frame
            transform = self.tf_buffer.lookup_transform(
                'flange', 'zivid_optical_frame', rclpy.time.Time())

            trans = transform.transform.translation
            rot = transform.transform.rotation

            self.get_logger().info(f'Transform flange → zivid_optical_frame:')
            self.get_logger().info(f'  Translation: [{trans.x:.4f}, {trans.y:.4f}, {trans.z:.4f}] m')
            self.get_logger().info(f'  Rotation: [{rot.x:.4f}, {rot.y:.4f}, {rot.z:.4f}, {rot.w:.4f}]')

            # Expected values from calibration
            expected_trans = [-0.05435, -0.10490, -0.19139]
            diff = [abs(trans.x - expected_trans[0]),
                   abs(trans.y - expected_trans[1]),
                   abs(trans.z - expected_trans[2])]

            if max(diff) < 0.001:  # 1mm tolerance
                self.get_logger().info('✓ Transform matches calibration!')
            else:
                self.get_logger().warn(f'⚠ Transform differs from calibration by {max(diff)*1000:.1f}mm')

        except Exception as e:
            self.get_logger().error(f'Failed to get transform: {e}')

def main():
    rclpy.init()
    node = TransformVerifier()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

    print(test_script)

    # Save test script
    test_file = Path("/home/aditya/work/github_ws/erobs/test_transform.py")
    test_file.write_text(test_script)
    print(f"\nTest script saved to: {test_file}")

    print_header("Validation Complete")
    print("""
SUMMARY:
1. ✓ Calibration values converted to URDF format
2. ✓ URDF files backed up with timestamps
3. ✓ zivid_camera_mount.xacro updated with calibrated transform
4. ✓ Test launch file created for static TF publisher
5. ✓ Workspace rebuilt successfully
6. ✓ Validation procedures documented

NEXT STEPS:
1. Launch your robot system with the updated URDF
2. Verify TF tree shows correct transforms
3. Test with ArUco marker detection
4. Fine-tune if necessary

CALIBRATION VALUES APPLIED:
- Translation: [-54.35, -104.90, -191.39] mm
- Rotation: [-1.74°, 3.04°, -1.04°] (Euler XYZ)
- Transform: flange → zivid_optical_frame

If accuracy is not satisfactory:
- Verify calibration board was detected correctly in all poses
- Check for mechanical play in camera mount
- Ensure robot was at exact recorded poses during calibration
- Consider recalibrating with more poses or better coverage
    """)


if __name__ == "__main__":
    main()