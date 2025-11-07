#!/usr/bin/env python3
"""
Debug the camera position issue - check if transform needs to be inverted.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


def main():
    print("=" * 60)
    print("DEBUGGING CAMERA POSITION")
    print("=" * 60)

    # Original calibration: flange → zivid_optical_frame
    flange_T_optical = np.array([
        [0.9984258, 0.01653194, 0.0535967, -54.35176],
        [-0.01812541, 0.9994039, 0.02938225, -104.9027],
        [-0.05307901, -0.03030745, 0.9981303, -191.3902],
        [0, 0, 0, 1]
    ])

    # Convert to meters
    flange_T_optical_m = flange_T_optical.copy()
    flange_T_optical_m[:3, 3] /= 1000.0

    print("\n1. Original calibration (flange → optical):")
    print(f"   Translation: {flange_T_optical_m[:3, 3]}")

    # The current values in xacro (arm_mount → base_link)
    print("\n2. Current xacro values:")
    print("   xyz='0.07744 0.24156 -0.02734'")
    print("   rpy='-1.58733 -0.00999 -1.60095'")

    # Check if we need the inverse
    print("\n3. Let's check the inverse transform:")
    optical_T_flange = np.linalg.inv(flange_T_optical_m)
    trans_inv = optical_T_flange[:3, 3]
    rot_inv = R.from_matrix(optical_T_flange[:3, :3])
    rpy_inv = rot_inv.as_euler('xyz', degrees=False)

    print(f"   Inverse translation: {trans_inv}")
    print(f"   Inverse RPY: {rpy_inv}")

    # Recheck our original uncalibrated values
    print("\n4. Original UNCALIBRATED values from backup:")
    print("   xyz='0.025 0.062 -0.049'")
    print("   rpy='0 -1.5708 -1.5708'")

    # The difference in position
    print("\n5. Positional difference (calibrated vs uncalibrated):")
    calib_pos = np.array([0.07744, 0.24156, -0.02734])
    uncalib_pos = np.array([0.025, 0.062, -0.049])
    diff = calib_pos - uncalib_pos
    print(f"   Difference: {diff} meters")
    print(f"   Distance: {np.linalg.norm(diff):.3f} meters")

    print("\n" + "=" * 60)
    print("POSSIBLE SOLUTIONS:")
    print("=" * 60)

    # Try simpler approach - just replace with calibrated values directly
    print("\n1. Direct mount_to_camera_joint (simplest):")
    print("   Keep original structure but with calibrated translation/rotation")

    # Account for the arm mount offset
    arm_mount_offset = np.array([0.005, 0, 0])  # From parent_link xyz
    arm_mount_rpy = np.array([-1.5708, 0, -1.5708])  # From parent_link rpy

    print("\n2. Try using smaller adjustments from uncalibrated:")
    # Maybe the calibration is a refinement, not absolute position
    refined_xyz = uncalib_pos + flange_T_optical_m[:3, 3] * 0.1  # Scale down
    print(f"   xyz='{refined_xyz[0]:.5f} {refined_xyz[1]:.5f} {refined_xyz[2]:.5f}'")

    print("\n3. Check if Y-axis is flipped (robot vs camera convention):")
    flipped_trans = flange_T_optical_m[:3, 3].copy()
    flipped_trans[1] = -flipped_trans[1]  # Flip Y
    print(f"   With Y flipped: {flipped_trans}")


if __name__ == "__main__":
    main()