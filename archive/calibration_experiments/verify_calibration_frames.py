#!/usr/bin/env python3
"""
Verify that we're applying the calibration to the correct frames.
Check the actual frame hierarchy and calibration reference.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml


def main():
    print("=" * 60)
    print("FRAME HIERARCHY VERIFICATION")
    print("=" * 60)

    # Load the actual calibration file to verify
    calib_file = "/home/aditya/work/github_ws/erobs/src/vision/zivid-python-samples/source/applications/advanced/hand_eye_calibration/hand_eye_transform.yaml"

    print("\n1. CALIBRATION DATA FROM FILE:")
    print(f"   File: hand_eye_transform.yaml")

    # Your calibration matrix
    hand_eye_matrix = np.array([
        [0.9984258, 0.01653194, 0.0535967, -54.35176],
        [-0.01812541, 0.9994039, 0.02938225, -104.9027],
        [-0.05307901, -0.03030745, 0.9981303, -191.3902],
        [0, 0, 0, 1]
    ])

    print("\n   Matrix (in MILLIMETERS):")
    print(hand_eye_matrix)

    # Convert translation to meters
    trans_m = hand_eye_matrix[:3, 3] / 1000.0
    rot_matrix = hand_eye_matrix[:3, :3]
    rot = R.from_matrix(rot_matrix)
    rpy_rad = rot.as_euler('xyz', degrees=False)
    rpy_deg = rot.as_euler('xyz', degrees=True)

    print(f"\n   Translation (meters): {trans_m}")
    print(f"   RPY (radians): {rpy_rad}")
    print(f"   RPY (degrees): {rpy_deg}")

    print("\n2. FRAME HIERARCHY IN URDF:")
    print("   flange")
    print("     ├── zivid_arm_mount (via zivid_arm_mount_joint)")
    print("     │     └── zivid_base_link (via mount_to_camera_joint) <- WE MODIFY THIS")
    print("     │           └── zivid_optical_frame (via zivid_uncalibrated_optical_joint)")
    print("     └── [gripper or tool]")

    print("\n3. WHAT THE CALIBRATION REPRESENTS:")
    print("   According to Zivid docs for eye-in-hand:")
    print("   - The matrix is: end-effector → camera")
    print("   - In our case: flange → zivid_optical_frame")
    print("   - This is the COMPLETE transform from flange to optical frame")

    print("\n4. CURRENT URDF JOINTS:")

    # Joint 1: flange → zivid_arm_mount
    joint1_xyz = np.array([0.005, 0, 0])  # from ur_with_zivid_hande.xacro line 112
    joint1_rpy = np.array([-1.5708, 0, -1.5708])  # from line 113

    print("\n   a) zivid_arm_mount_joint (flange → zivid_arm_mount):")
    print(f"      xyz: {joint1_xyz}")
    print(f"      rpy: {joint1_rpy}")

    # Joint 2: zivid_arm_mount → zivid_base_link (THIS IS WHAT WE MODIFY)
    joint2_xyz_uncalib = np.array([0.025, 0.062, -0.049])
    joint2_rpy_uncalib = np.array([0, -1.5708, -1.5708])

    print("\n   b) mount_to_camera_joint (zivid_arm_mount → zivid_base_link):")
    print(f"      UNCALIBRATED xyz: {joint2_xyz_uncalib}")
    print(f"      UNCALIBRATED rpy: {joint2_rpy_uncalib}")

    # Joint 3: zivid_base_link → zivid_optical_frame (internal, fixed)
    joint3_xyz = np.array([0.049, 0.03202, 0.0295])
    joint3_rpy = np.array([-1.5707963267948966, 0, -1.6144295580947547])

    print("\n   c) zivid_uncalibrated_optical_joint (zivid_base_link → zivid_optical_frame):")
    print(f"      xyz: {joint3_xyz}")
    print(f"      rpy: {joint3_rpy}")

    print("\n5. CALCULATING CORRECT mount_to_camera_joint:")
    print("   We have: flange → optical (calibrated)")
    print("   We need: mount → base_link")
    print("   Formula: mount → base = inverse(flange → mount) @ (flange → optical) @ inverse(base → optical)")

    # Build transformation matrices
    def xyz_rpy_to_matrix(xyz, rpy):
        T = np.eye(4)
        T[:3, 3] = xyz
        T[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
        return T

    # Create all transforms
    T_flange_mount = xyz_rpy_to_matrix(joint1_xyz, joint1_rpy)
    T_base_optical = xyz_rpy_to_matrix(joint3_xyz, joint3_rpy)
    T_flange_optical_calib = hand_eye_matrix.copy()
    T_flange_optical_calib[:3, 3] /= 1000.0  # Convert to meters

    # Calculate: mount → base = inv(flange → mount) @ (flange → optical) @ inv(base → optical)
    T_mount_base_calib = np.linalg.inv(T_flange_mount) @ T_flange_optical_calib @ np.linalg.inv(T_base_optical)

    # Extract xyz and rpy
    xyz_calib = T_mount_base_calib[:3, 3]
    rot_calib = R.from_matrix(T_mount_base_calib[:3, :3])
    rpy_calib = rot_calib.as_euler('xyz', degrees=False)

    print("\n6. CALIBRATED mount_to_camera_joint VALUES:")
    print(f"   xyz=\"{xyz_calib[0]:.5f} {xyz_calib[1]:.5f} {xyz_calib[2]:.5f}\"")
    print(f"   rpy=\"{rpy_calib[0]:.5f} {rpy_calib[1]:.5f} {rpy_calib[2]:.5f}\"")

    print("\n7. DIFFERENCE from uncalibrated:")
    diff_xyz = xyz_calib - joint2_xyz_uncalib
    diff_rpy = rpy_calib - joint2_rpy_uncalib
    print(f"   Position difference: {np.linalg.norm(diff_xyz)*1000:.1f} mm")
    print(f"   xyz diff: {diff_xyz*1000} mm")
    print(f"   rpy diff: {np.degrees(diff_rpy)} degrees")

    print("\n8. SANITY CHECK:")
    print("   Let's verify the complete chain gives us the calibration:")
    T_check = T_flange_mount @ T_mount_base_calib @ T_base_optical
    check_trans = T_check[:3, 3]
    calib_trans = T_flange_optical_calib[:3, 3]
    print(f"   Calculated flange→optical: {check_trans}")
    print(f"   Original calibration:       {calib_trans}")
    print(f"   Match: {np.allclose(check_trans, calib_trans, atol=1e-5)}")


if __name__ == "__main__":
    main()