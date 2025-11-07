#!/usr/bin/env python3
"""
Recalculate the calibration assuming it was done with tool0 as reference,
not flange.
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
    print("RECALCULATING WITH tool0 INSTEAD OF flange")
    print("=" * 60)

    # Your calibration: tool0 → optical_frame (in millimeters)
    T_tool0_optical_calib = np.array([
        [0.9984258, 0.01653194, 0.0535967, -54.35176],
        [-0.01812541, 0.9994039, 0.02938225, -104.9027],
        [-0.05307901, -0.03030745, 0.9981303, -191.3902],
        [0, 0, 0, 1]
    ])
    # Convert to meters
    T_tool0_optical_calib[:3, 3] /= 1000.0

    print("\n1. CALIBRATION (assuming tool0 → optical_frame):")
    xyz_calib, rpy_calib = matrix_to_xyz_rpy(T_tool0_optical_calib)
    print(f"   xyz: {xyz_calib}")
    print(f"   rpy (rad): {rpy_calib}")
    print(f"   rpy (deg): {np.degrees(rpy_calib)}")

    print("\n2. KNOWN TRANSFORMS:")

    # flange → tool0 (from URDF line 608)
    xyz_flange_tool0 = np.array([0, 0, 0])
    rpy_flange_tool0 = np.array([1.5707963267948966, 0, 1.5707963267948966])  # 90° roll, 90° yaw
    T_flange_tool0 = xyz_rpy_to_matrix(xyz_flange_tool0, rpy_flange_tool0)
    print(f"   flange → tool0: xyz={xyz_flange_tool0}, rpy={rpy_flange_tool0}")
    print(f"                   rpy (deg): {np.degrees(rpy_flange_tool0)}")

    # tool0 → arm_mount (we need to calculate this for the xacro)
    # Current xacro has: flange → arm_mount with xyz="0.005 0 0" rpy="-1.5708 0 -1.5708"
    xyz_flange_mount_old = np.array([0.005, 0, 0])
    rpy_flange_mount_old = np.array([-1.5708, 0, -1.5708])
    T_flange_mount_old = xyz_rpy_to_matrix(xyz_flange_mount_old, rpy_flange_mount_old)

    # Calculate tool0 → arm_mount
    T_tool0_mount = np.linalg.inv(T_flange_tool0) @ T_flange_mount_old
    xyz_tool0_mount, rpy_tool0_mount = matrix_to_xyz_rpy(T_tool0_mount)
    print(f"\n   tool0 → arm_mount: xyz={xyz_tool0_mount}, rpy={rpy_tool0_mount}")

    # base_link → optical_frame (internal camera offset)
    xyz_base_optical = np.array([0.049, 0.03202, 0.0295])
    rpy_base_optical = np.array([-1.5707963267948966, 0, -1.6144295580947547])
    T_base_optical = xyz_rpy_to_matrix(xyz_base_optical, rpy_base_optical)
    print(f"   base_link → optical: xyz={xyz_base_optical}, rpy={rpy_base_optical}")

    print("\n3. CALCULATING mount → base_link (for tool0 parent):")
    print("   We need: tool0→mount→base→optical = tool0→optical(calibrated)")
    print("   Therefore: mount→base = inv(tool0→mount) @ (tool0→optical) @ inv(base→optical)")

    # Calculate the required mount → base transform
    T_mount_base_calib = np.linalg.inv(T_tool0_mount) @ T_tool0_optical_calib @ np.linalg.inv(T_base_optical)

    xyz_mount_base, rpy_mount_base = matrix_to_xyz_rpy(T_mount_base_calib)

    print(f"\n4. RESULT for mount_to_camera_joint (with tool0 parent):")
    print(f"   xyz=\"{xyz_mount_base[0]:.5f} {xyz_mount_base[1]:.5f} {xyz_mount_base[2]:.5f}\"")
    print(f"   rpy=\"{rpy_mount_base[0]:.5f} {rpy_mount_base[1]:.5f} {rpy_mount_base[2]:.5f}\"")

    print(f"\n5. FOR THE XACRO (parent_link change):")
    print(f"   Change: parent_link=\"$(arg tf_prefix)flange\"")
    print(f"   To:     parent_link=\"$(arg tf_prefix)tool0\"")
    print(f"\n   And adjust xyz/rpy at line 112-113:")
    print(f"   xyz=\"{xyz_tool0_mount[0]:.5f} {xyz_tool0_mount[1]:.5f} {xyz_tool0_mount[2]:.5f}\"")
    print(f"   rpy=\"{rpy_tool0_mount[0]:.5f} {rpy_tool0_mount[1]:.5f} {rpy_tool0_mount[2]:.5f}\"")

    # Verify the calculation
    print("\n6. VERIFICATION:")
    T_verify = T_tool0_mount @ T_mount_base_calib @ T_base_optical
    xyz_verify, rpy_verify = matrix_to_xyz_rpy(T_verify)
    print(f"   Calculated tool0→optical: {xyz_verify}")
    print(f"   Should match calibration:  {xyz_calib}")
    print(f"   Error (mm): {(xyz_verify - xyz_calib) * 1000}")

    # Compare with uncalibrated
    print("\n7. COMPARISON WITH PREVIOUS (flange-based):")
    xyz_uncalib = np.array([0.025, 0.062, -0.049])
    print(f"   Previous (flange): xyz={xyz_uncalib}")
    print(f"   New (tool0):       xyz={xyz_mount_base}")
    print(f"   Difference: {(xyz_mount_base - xyz_uncalib) * 1000} mm")


if __name__ == "__main__":
    main()