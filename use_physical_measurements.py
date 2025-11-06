#!/usr/bin/env python3
"""
Calculate URDF values based on physical measurements instead of calibration.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


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
    print("=" * 60)
    print("USING PHYSICAL MEASUREMENTS")
    print("=" * 60)

    # User's physical measurements (flange → optical_frame)
    measured_xyz_m = np.array([0.01, -0.06, -0.11])  # Use middle of 0-2cm range for X
    print("\n1. YOUR PHYSICAL MEASUREMENTS (flange → optical_frame):")
    print(f"   X: {measured_xyz_m[0]*100:.1f}cm (small offset)")
    print(f"   Y: {measured_xyz_m[1]*100:.1f}cm (backward)")
    print(f"   Z: {measured_xyz_m[2]*100:.1f}cm (below)")

    # Keep the rotation from calibration (small refinement angles)
    measured_rpy = np.array([-0.03035, 0.05310, -0.01815])  # From calibration
    print(f"\n   Rotation (from calibration): {np.degrees(measured_rpy)} degrees")

    # Build measured flange→optical transform
    T_flange_optical_measured = xyz_rpy_to_matrix(measured_xyz_m, measured_rpy)

    # Known transforms
    T_flange_mount = xyz_rpy_to_matrix(
        np.array([0.005, 0, 0]),
        np.array([-1.5708, 0, -1.5708])
    )

    T_base_optical = xyz_rpy_to_matrix(
        np.array([0.049, 0.03202, 0.0295]),
        np.array([-1.5707963267948966, 0, -1.6144295580947547])
    )

    # Calculate mount→base
    T_mount_base = np.linalg.inv(T_flange_mount) @ T_flange_optical_measured @ np.linalg.inv(T_base_optical)

    xyz_mount_base, rpy_mount_base = matrix_to_xyz_rpy(T_mount_base)

    print("\n2. CALCULATED mount_to_camera_joint (based on measurements):")
    print(f"   xyz=\"{xyz_mount_base[0]:.5f} {xyz_mount_base[1]:.5f} {xyz_mount_base[2]:.5f}\"")
    print(f"   rpy=\"{rpy_mount_base[0]:.5f} {rpy_mount_base[1]:.5f} {rpy_mount_base[2]:.5f}\"")

    # Compare with uncalibrated
    uncalib_xyz = np.array([0.025, 0.062, -0.049])
    print("\n3. COMPARISON:")
    print(f"   Uncalibrated:     xyz=\"{uncalib_xyz[0]:.5f} {uncalib_xyz[1]:.5f} {uncalib_xyz[2]:.5f}\"")
    print(f"   From measurement: xyz=\"{xyz_mount_base[0]:.5f} {xyz_mount_base[1]:.5f} {xyz_mount_base[2]:.5f}\"")
    print(f"   Difference: {(xyz_mount_base - uncalib_xyz)*1000:.1f} mm")

    # Compare with calibration values
    calib_xyz = np.array([0.07744, 0.24156, -0.02734])
    print(f"\n   From calibration: xyz=\"{calib_xyz[0]:.5f} {calib_xyz[1]:.5f} {calib_xyz[2]:.5f}\"")
    print(f"   Difference from calibration: {(xyz_mount_base - calib_xyz)*1000:.1f} mm")

    print("\n4. WHY IS THE CALIBRATION WRONG?")
    print("   Possible reasons:")
    print("   a) Robot poses during calibration had incorrect TCP offset")
    print("   b) Calibration board was detected at wrong distance")
    print("   c) Robot poses in robot_pose_*.yaml files were incorrect")
    print("   d) Coordinate frame mismatch during calibration")

    print("\n5. RECOMMENDATION:")
    print("   Use the values based on physical measurement above")
    print("   OR re-do the calibration ensuring:")
    print("   - Robot TCP is set to flange (no tool offset)")
    print("   - Calibration board is properly detected")
    print("   - Robot poses are accurate")


if __name__ == "__main__":
    main()