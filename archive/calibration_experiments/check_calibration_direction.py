#!/usr/bin/env python3
"""
Check if the calibration direction is correct and if there's a coordinate system issue.
The 90-degree rotation differences are suspicious!
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


def main():
    print("=" * 60)
    print("CHECKING CALIBRATION DIRECTION & COORDINATE SYSTEMS")
    print("=" * 60)

    # Your calibration matrix
    hand_eye_matrix = np.array([
        [0.9984258, 0.01653194, 0.0535967, -54.35176],
        [-0.01812541, 0.9994039, 0.02938225, -104.9027],
        [-0.05307901, -0.03030745, 0.9981303, -191.3902],
        [0, 0, 0, 1]
    ])

    print("\n1. CHECKING ZIVID DOCUMENTATION:")
    print("   From Zivid hand-eye calibration docs:")
    print("   - Eye-in-hand: Transform is from END-EFFECTOR to CAMERA")
    print("   - This should be: flange → camera_optical_frame")

    print("\n2. CHECKING YOUR CALIBRATION SETTINGS:")
    print("   You used: Euler XYZ, Degrees, EXTRINSIC")
    print("   Robot poses were in: robot base frame")
    print("   Calibration board poses were in: camera frame")

    print("\n3. SUSPICIOUS ROTATION DIFFERENCES:")
    print("   The calculated values show ~90° rotation changes")
    print("   This often indicates:")
    print("   a) Wrong coordinate convention (ROS vs robot)")
    print("   b) Transform applied in wrong direction")
    print("   c) Different rotation sequence (XYZ vs ZYX)")

    # Let's check if we should use the INVERSE
    print("\n4. TESTING INVERSE TRANSFORM:")

    # Get inverse
    hand_eye_inv = np.linalg.inv(hand_eye_matrix)
    trans_inv_mm = hand_eye_inv[:3, 3]
    trans_inv_m = trans_inv_mm / 1000.0
    rot_inv = R.from_matrix(hand_eye_inv[:3, :3])
    rpy_inv = rot_inv.as_euler('xyz', degrees=False)

    print(f"   Inverse translation (m): {trans_inv_m}")
    print(f"   Inverse RPY (rad): {rpy_inv}")
    print(f"   Inverse RPY (deg): {np.degrees(rpy_inv)}")

    # Check with different rotation sequences
    print("\n5. TESTING DIFFERENT ROTATION SEQUENCES:")

    rot_matrix = hand_eye_matrix[:3, :3]
    rot = R.from_matrix(rot_matrix)

    sequences = ['xyz', 'xzy', 'yxz', 'yzx', 'zxy', 'zyx']
    for seq in sequences:
        rpy = rot.as_euler(seq, degrees=True)
        print(f"   {seq.upper()}: {rpy}")

    print("\n6. CHECKING ROBOT VS ROS CONVENTIONS:")
    print("   UR robots use: Z-Y-X intrinsic (roll-pitch-yaw)")
    print("   ROS typically uses: X-Y-Z extrinsic")
    print("   Zivid calibration: Uses robot convention")

    # Try ZYX interpretation
    print("\n7. REINTERPRETING WITH ZYX (ROBOT CONVENTION):")
    # If the calibration gave us ZYX intrinsic, convert to XYZ extrinsic for ROS

    # The rotation matrix is still valid, but interpretation changes
    rpy_zyx = rot.as_euler('ZYX', degrees=False)  # Capital letters = intrinsic
    print(f"   As ZYX intrinsic (rad): {rpy_zyx}")
    print(f"   As ZYX intrinsic (deg): {np.degrees(rpy_zyx)}")

    # For ROS, we need XYZ extrinsic, which equals ZYX intrinsic reversed
    rpy_for_ros = rot.as_euler('xyz', degrees=False)  # lowercase = extrinsic
    print(f"   For ROS xyz extrinsic (rad): {rpy_for_ros}")

    print("\n8. MOST LIKELY ISSUE:")
    print("   The large 188mm position difference suggests either:")
    print("   a) The camera mount has been mechanically adjusted since calibration")
    print("   b) The calibration board detection had systematic errors")
    print("   c) Wrong units somewhere (but we checked - all in mm→m)")

    print("\n9. RECOMMENDED FIX:")
    print("   Try the INVERSE transform since camera might be floating")
    print("   because we're applying transform in wrong direction:")

    # Calculate mount_to_camera with INVERSE calibration
    joint1_xyz = np.array([0.005, 0, 0])
    joint1_rpy = np.array([-1.5708, 0, -1.5708])
    joint3_xyz = np.array([0.049, 0.03202, 0.0295])
    joint3_rpy = np.array([-1.5707963267948966, 0, -1.6144295580947547])

    def xyz_rpy_to_matrix(xyz, rpy):
        T = np.eye(4)
        T[:3, 3] = xyz
        T[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
        return T

    T_flange_mount = xyz_rpy_to_matrix(joint1_xyz, joint1_rpy)
    T_base_optical = xyz_rpy_to_matrix(joint3_xyz, joint3_rpy)

    # Use INVERSE of calibration
    T_flange_optical_inv = hand_eye_inv.copy()
    T_flange_optical_inv[:3, 3] /= 1000.0  # Convert to meters

    # This assumes calibration was actually optical→flange
    T_optical_flange = T_flange_optical_inv
    T_flange_optical_corrected = np.linalg.inv(T_optical_flange)

    T_mount_base = np.linalg.inv(T_flange_mount) @ T_flange_optical_corrected @ np.linalg.inv(T_base_optical)

    xyz_corrected = T_mount_base[:3, 3]
    rot_corrected = R.from_matrix(T_mount_base[:3, :3])
    rpy_corrected = rot_corrected.as_euler('xyz', degrees=False)

    print(f"\n   With INVERSE: xyz=\"{xyz_corrected[0]:.5f} {xyz_corrected[1]:.5f} {xyz_corrected[2]:.5f}\"")
    print(f"                 rpy=\"{rpy_corrected[0]:.5f} {rpy_corrected[1]:.5f} {rpy_corrected[2]:.5f}\"")


if __name__ == "__main__":
    main()