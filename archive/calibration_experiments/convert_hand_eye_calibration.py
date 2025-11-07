#!/usr/bin/env python3
"""
Convert Zivid hand-eye calibration matrix to URDF and ROS 2 compatible formats.

This script takes the hand-eye calibration result from the Zivid calibration process
and converts it to various formats needed for ROS 2 integration.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml


def load_calibration_matrix(filepath):
    """Load calibration matrix from YAML file."""
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)

    # Extract the 4x4 matrix from the FloatMatrix data
    matrix_data = data['FloatMatrix']['Data']
    matrix = np.array(matrix_data)
    return matrix


def analyze_calibration(matrix):
    """Analyze the calibration matrix and convert to useful formats."""
    print("=" * 60)
    print("HAND-EYE CALIBRATION ANALYSIS")
    print("=" * 60)

    print("\n1. Original Calibration Matrix (millimeters):")
    print(matrix)

    # Extract translation (convert mm to m)
    trans_mm = matrix[:3, 3]
    trans_m = trans_mm / 1000.0

    print(f"\n2. Translation:")
    print(f"   Millimeters: x={trans_mm[0]:.2f}, y={trans_mm[1]:.2f}, z={trans_mm[2]:.2f}")
    print(f"   Meters:      x={trans_m[0]:.5f}, y={trans_m[1]:.5f}, z={trans_m[2]:.5f}")

    # Extract rotation matrix
    rot_matrix = matrix[:3, :3]
    rot = R.from_matrix(rot_matrix)

    # Convert to different representations
    rpy_rad = rot.as_euler('xyz', degrees=False)
    rpy_deg = rot.as_euler('xyz', degrees=True)
    quat = rot.as_quat()  # [x, y, z, w]

    print(f"\n3. Rotation:")
    print(f"   RPY (degrees): roll={rpy_deg[0]:.3f}°, pitch={rpy_deg[1]:.3f}°, yaw={rpy_deg[2]:.3f}°")
    print(f"   RPY (radians): roll={rpy_rad[0]:.5f}, pitch={rpy_rad[1]:.5f}, yaw={rpy_rad[2]:.5f}")
    print(f"   Quaternion:    x={quat[0]:.5f}, y={quat[1]:.5f}, z={quat[2]:.5f}, w={quat[3]:.5f}")

    print("\n" + "=" * 60)
    print("URDF INTEGRATION OPTIONS")
    print("=" * 60)

    print("\n4. URDF Joint Format (for xacro file):")
    print("   Option A - Direct calibrated joint (RECOMMENDED):")
    print(f'   <joint name="flange_to_zivid_optical_calibrated" type="fixed">')
    print(f'     <parent link="flange"/>')
    print(f'     <child link="zivid_optical_frame"/>')
    print(f'     <origin xyz="{trans_m[0]:.5f} {trans_m[1]:.5f} {trans_m[2]:.5f}"')
    print(f'             rpy="{rpy_rad[0]:.5f} {rpy_rad[1]:.5f} {rpy_rad[2]:.5f}"/>')
    print(f'   </joint>')

    print("\n   Option B - Update existing mount_to_camera_joint:")
    print("   Replace line 56 in zivid_camera_mount.xacro:")
    print("   OLD: <origin xyz=\"0.025 0.062 -0.049\" rpy=\"0 -1.5708 -1.5708\"/>")
    print(f"   NEW: <origin xyz=\"{trans_m[0]:.5f} {trans_m[1]:.5f} {trans_m[2]:.5f}\"")
    print(f"                rpy=\"{rpy_rad[0]:.5f} {rpy_rad[1]:.5f} {rpy_rad[2]:.5f}\"/>")

    print("\n5. Static Transform Publisher (for testing):")
    print("   Launch file node configuration:")
    print("   Node(")
    print("     package='tf2_ros',")
    print("     executable='static_transform_publisher',")
    print("     arguments=[")
    print(f"       '--x', '{trans_m[0]:.5f}',")
    print(f"       '--y', '{trans_m[1]:.5f}',")
    print(f"       '--z', '{trans_m[2]:.5f}',")
    print(f"       '--qx', '{quat[0]:.5f}',")
    print(f"       '--qy', '{quat[1]:.5f}',")
    print(f"       '--qz', '{quat[2]:.5f}',")
    print(f"       '--qw', '{quat[3]:.5f}',")
    print("       '--frame-id', 'flange',")
    print("       '--child-frame-id', 'zivid_optical_frame_calibrated'")
    print("     ]")
    print("   )")

    print("\n6. Command line test (run this to test immediately):")
    print(f"   ros2 run tf2_ros static_transform_publisher \\")
    print(f"     --x {trans_m[0]:.5f} --y {trans_m[1]:.5f} --z {trans_m[2]:.5f} \\")
    print(f"     --qx {quat[0]:.5f} --qy {quat[1]:.5f} --qz {quat[2]:.5f} --qw {quat[3]:.5f} \\")
    print(f"     --frame-id flange --child-frame-id zivid_optical_frame_calibrated")

    # Calculate inverse transform (camera to flange)
    matrix_inv = np.linalg.inv(matrix)
    trans_inv_m = matrix_inv[:3, 3] / 1000.0
    rot_inv = R.from_matrix(matrix_inv[:3, :3])
    rpy_inv_rad = rot_inv.as_euler('xyz', degrees=False)

    print("\n7. Inverse Transform (zivid_optical_frame to flange):")
    print(f"   Translation (m): x={trans_inv_m[0]:.5f}, y={trans_inv_m[1]:.5f}, z={trans_inv_m[2]:.5f}")
    print(f"   RPY (radians):   roll={rpy_inv_rad[0]:.5f}, pitch={rpy_inv_rad[1]:.5f}, yaw={rpy_inv_rad[2]:.5f}")

    return {
        'translation_m': trans_m,
        'rpy_rad': rpy_rad,
        'rpy_deg': rpy_deg,
        'quaternion': quat,
        'matrix': matrix
    }


def main():
    # Your calibration matrix (hardcoded for quick reference)
    your_calibration = np.array([
        [0.9984258, 0.01653194, 0.0535967, -54.35176],
        [-0.01812541, 0.9994039, 0.02938225, -104.9027],
        [-0.05307901, -0.03030745, 0.9981303, -191.3902],
        [0, 0, 0, 1]
    ])

    print("Using hardcoded calibration matrix from hand_eye_transform.yaml")
    results = analyze_calibration(your_calibration)

    # Try to load from file if it exists
    try:
        import os
        yaml_path = "/home/aditya/work/github_ws/erobs/src/vision/zivid-python-samples/source/applications/advanced/hand_eye_calibration/hand_eye_transform.yaml"
        if os.path.exists(yaml_path):
            print("\n" + "=" * 60)
            print("VERIFYING AGAINST YAML FILE")
            print("=" * 60)
            loaded_matrix = load_calibration_matrix(yaml_path)
            if np.allclose(loaded_matrix, your_calibration):
                print("✓ Hardcoded matrix matches YAML file")
            else:
                print("⚠ Warning: Matrices differ!")
                print("YAML file matrix:")
                print(loaded_matrix)
    except Exception as e:
        print(f"Could not verify against YAML file: {e}")

    print("\n" + "=" * 60)
    print("NEXT STEPS:")
    print("=" * 60)
    print("1. Test with static transform publisher (command in section 6)")
    print("2. Backup your xacro files before modification")
    print("3. Apply URDF changes from section 4")
    print("4. Rebuild workspace: colcon build --packages-select ur5e_robot_description")
    print("5. Verify with: ros2 run tf2_tools view_frames")


if __name__ == "__main__":
    main()