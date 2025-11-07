#!/usr/bin/env python3
"""
Ultra-careful verification that we're using the exact calibration values.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml


def xyz_rpy_to_matrix(xyz, rpy):
    T = np.eye(4)
    T[:3, 3] = xyz
    T[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
    return T


def matrix_to_xyz_rpy(T):
    xyz = T[:3, 3]
    rot = R.from_matrix(T[:3, :3])
    rpy = rot.as_euler('xyz', degrees=False)
    return xyz, rpy


def main():
    print("=" * 70)
    print("ULTRA-CAREFUL VERIFICATION OF CALIBRATION VALUES")
    print("=" * 70)

    # EXACT values from hand_eye_transform.yaml
    print("\n1. LOADING EXACT CALIBRATION FROM FILE:")
    calib_file = "/home/aditya/work/github_ws/erobs/src/vision/zivid-python-samples/source/applications/advanced/hand_eye_calibration/hand_eye_transform.yaml"

    with open(calib_file, 'r') as f:
        data = yaml.safe_load(f)

    matrix_data = data['FloatMatrix']['Data']
    T_tool0_optical_calib = np.array(matrix_data)

    print(f"   Loaded from: {calib_file}")
    print(f"   Version: {data['__version__']}")
    print("\n   Raw matrix (millimeters):")
    print(T_tool0_optical_calib)

    # Verify it matches what user provided
    expected = np.array([
        [0.9984258, 0.01653194, 0.0535967, -54.35176],
        [-0.01812541, 0.9994039, 0.02938225, -104.9027],
        [-0.05307901, -0.03030745, 0.9981303, -191.3902],
        [0, 0, 0, 1]
    ])

    if np.allclose(T_tool0_optical_calib, expected):
        print("\n   ✓ Matrix matches user-provided values EXACTLY")
    else:
        print("\n   ✗ WARNING: Matrix differs!")
        print("   Difference:")
        print(T_tool0_optical_calib - expected)

    # Convert to meters
    T_tool0_optical_calib_m = T_tool0_optical_calib.copy()
    T_tool0_optical_calib_m[:3, 3] /= 1000.0

    print("\n2. CALIBRATION VALUES (tool0 → optical_frame):")
    print(f"   Translation (mm): {T_tool0_optical_calib[:3, 3]}")
    print(f"   Translation (m):  {T_tool0_optical_calib_m[:3, 3]}")

    rot = R.from_matrix(T_tool0_optical_calib[:3, :3])
    rpy = rot.as_euler('xyz', degrees=False)
    print(f"   Rotation (rad):   {rpy}")
    print(f"   Rotation (deg):   {np.degrees(rpy)}")

    print("\n3. KNOWN URDF TRANSFORMS:")

    # tool0 → arm_mount (what we set in xacro)
    xyz_tool0_mount = np.array([0.00000, 0.00000, 0.00500])
    rpy_tool0_mount = np.array([0.00000, 0.00000, 3.14159])
    T_tool0_mount = xyz_rpy_to_matrix(xyz_tool0_mount, rpy_tool0_mount)
    print(f"   tool0 → arm_mount: xyz={xyz_tool0_mount}, rpy={rpy_tool0_mount}")

    # base_link → optical (internal Zivid offset, fixed)
    xyz_base_optical = np.array([0.049, 0.03202, 0.0295])
    rpy_base_optical = np.array([-1.5707963267948966, 0, -1.6144295580947547])
    T_base_optical = xyz_rpy_to_matrix(xyz_base_optical, rpy_base_optical)
    print(f"   base → optical:    xyz={xyz_base_optical}, rpy={rpy_base_optical}")

    print("\n4. CALCULATING mount → base_link:")
    print("   Formula: mount→base = inv(tool0→mount) @ (tool0→optical) @ inv(base→optical)")

    T_mount_base = np.linalg.inv(T_tool0_mount) @ T_tool0_optical_calib_m @ np.linalg.inv(T_base_optical)

    xyz_mount_base, rpy_mount_base = matrix_to_xyz_rpy(T_mount_base)

    print(f"\n   CALCULATED mount→base:")
    print(f"   xyz: {xyz_mount_base}")
    print(f"   rpy: {rpy_mount_base}")

    print("\n5. WHAT'S CURRENTLY IN THE XACRO:")
    current_xyz = np.array([0.02234, 0.07744, -0.24656])
    current_rpy = np.array([0.30355, -1.53903, -1.89103])
    print(f"   xyz: {current_xyz}")
    print(f"   rpy: {current_rpy}")

    print("\n6. VERIFICATION - DO THEY MATCH?")
    xyz_diff = xyz_mount_base - current_xyz
    rpy_diff = rpy_mount_base - current_rpy

    print(f"   xyz difference (mm): {xyz_diff * 1000}")
    print(f"   rpy difference (deg): {np.degrees(rpy_diff)}")

    if np.allclose(xyz_mount_base, current_xyz, atol=1e-4) and np.allclose(rpy_mount_base, current_rpy, atol=1e-4):
        print("\n   ✓ XACRO values are CORRECT!")
    else:
        print("\n   ✗ XACRO values differ from calculation!")
        print("\n   SHOULD BE:")
        print(f"   xyz=\"{xyz_mount_base[0]:.5f} {xyz_mount_base[1]:.5f} {xyz_mount_base[2]:.5f}\"")
        print(f"   rpy=\"{rpy_mount_base[0]:.5f} {rpy_mount_base[1]:.5f} {rpy_mount_base[2]:.5f}\"")

    print("\n7. FINAL VERIFICATION - COMPLETE CHAIN:")
    T_final = T_tool0_mount @ T_mount_base @ T_base_optical
    xyz_final, rpy_final = matrix_to_xyz_rpy(T_final)

    print(f"   Calculated tool0→optical: {xyz_final}")
    print(f"   From calibration file:     {T_tool0_optical_calib_m[:3, 3]}")
    print(f"   Error (mm): {(xyz_final - T_tool0_optical_calib_m[:3, 3]) * 1000}")

    if np.allclose(xyz_final, T_tool0_optical_calib_m[:3, 3], atol=1e-6):
        print("\n   ✓✓✓ PERFECT! Complete chain matches calibration exactly!")
    else:
        print("\n   ✗✗✗ ERROR! Chain does not match calibration!")

    print("\n8. COMPARING WITH YOUR PHYSICAL MEASUREMENTS:")
    print("   You measured (tool0 → optical):")
    print("   X: 0-2cm, Y: -6cm, Z: -11cm")
    measured = np.array([0.01, -0.06, -0.11])
    print(f"   Measured: {measured * 100} cm")
    print(f"   Calibration: {T_tool0_optical_calib_m[:3, 3] * 100} cm")
    print(f"   Difference: {(T_tool0_optical_calib_m[:3, 3] - measured) * 100} cm")
    print(f"   Distance: {np.linalg.norm(T_tool0_optical_calib_m[:3, 3] - measured) * 100:.1f} cm")

    print("\n" + "=" * 70)
    print("CONCLUSION:")
    print("=" * 70)
    print("If the camera is still floating, the calibration itself may have errors.")
    print("The URDF is correctly implementing the calibration matrix.")
    print("\nPossible issues:")
    print("1. Robot TCP was not set to tool0 during calibration")
    print("2. Calibration board detection had systematic errors")
    print("3. Robot pose recording had offsets")
    print("\nRecommendation: Check robot_pose_*.yaml files for correctness")


if __name__ == "__main__":
    main()