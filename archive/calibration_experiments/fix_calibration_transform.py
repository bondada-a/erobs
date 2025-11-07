#!/usr/bin/env python3
"""
Fix the calibration transform to account for internal camera offset.

The hand-eye calibration gives us: flange → zivid_optical_frame
But we need: flange → zivid_base_link (since optical_frame is already defined internally)
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


def main():
    # Your calibration: flange → zivid_optical_frame
    flange_T_optical = np.array([
        [0.9984258, 0.01653194, 0.0535967, -54.35176],
        [-0.01812541, 0.9994039, 0.02938225, -104.9027],
        [-0.05307901, -0.03030745, 0.9981303, -191.3902],
        [0, 0, 0, 1]
    ])

    # Internal offset from zivid_camera.xacro: zivid_base_link → zivid_optical_frame
    # From line 666: rpy="-1.5707963267948966 0 -1.6144295580947547" xyz="0.049 0.03202 0.0295"
    # Note: These are in meters already
    internal_trans = np.array([0.049, 0.03202, 0.0295])
    internal_rpy = np.array([-1.5707963267948966, 0, -1.6144295580947547])

    # Create transformation matrix for internal offset
    rot_internal = R.from_euler('xyz', internal_rpy)
    base_T_optical = np.eye(4)
    base_T_optical[:3, :3] = rot_internal.as_matrix()
    base_T_optical[:3, 3] = internal_trans

    print("Internal transform (zivid_base_link → zivid_optical_frame):")
    print(base_T_optical)

    # To get flange → base_link, we need to remove the internal offset:
    # flange_T_base = flange_T_optical @ inv(base_T_optical)

    # Convert calibration to meters first
    flange_T_optical_m = flange_T_optical.copy()
    flange_T_optical_m[:3, 3] /= 1000.0  # mm to m

    # Calculate the corrected transform
    base_T_optical_inv = np.linalg.inv(base_T_optical)
    flange_T_base = flange_T_optical_m @ base_T_optical_inv

    print("\n" + "=" * 60)
    print("CORRECTED TRANSFORM: flange → zivid_base_link")
    print("=" * 60)

    # Extract translation and rotation
    trans = flange_T_base[:3, 3]
    rot = R.from_matrix(flange_T_base[:3, :3])
    rpy = rot.as_euler('xyz', degrees=False)

    print(f"\nTranslation (m): x={trans[0]:.5f}, y={trans[1]:.5f}, z={trans[2]:.5f}")
    print(f"RPY (radians):   roll={rpy[0]:.5f}, pitch={rpy[1]:.5f}, yaw={rpy[2]:.5f}")

    print("\n" + "=" * 60)
    print("XACRO UPDATE REQUIRED")
    print("=" * 60)

    print("\nReplace the joint in zivid_camera_mount.xacro with:")
    print("""
    <!-- CALIBRATED TRANSFORM: flange → zivid_base_link -->
    <!-- Accounts for internal optical frame offset -->
    <joint name="mount_to_camera_joint" type="fixed">
      <parent link="zivid_arm_mount"/>
      <child link="zivid_base_link"/>
      <origin xyz="{:.5f} {:.5f} {:.5f}"
              rpy="{:.5f} {:.5f} {:.5f}"/>
    </joint>
    """.format(trans[0], trans[1], trans[2], rpy[0], rpy[1], rpy[2]))

    # Also need to account for arm mount offset
    # The arm mount is connected to flange with xyz="0.005 0 0" rpy="-1.5708 0 -1.5708"
    arm_mount_trans = np.array([0.005, 0, 0])
    arm_mount_rpy = np.array([-1.5708, 0, -1.5708])

    rot_arm_mount = R.from_euler('xyz', arm_mount_rpy)
    flange_T_mount = np.eye(4)
    flange_T_mount[:3, :3] = rot_arm_mount.as_matrix()
    flange_T_mount[:3, 3] = arm_mount_trans

    # mount_T_base = inv(flange_T_mount) @ flange_T_base
    mount_T_base = np.linalg.inv(flange_T_mount) @ flange_T_base

    trans_mount = mount_T_base[:3, 3]
    rot_mount = R.from_matrix(mount_T_base[:3, :3])
    rpy_mount = rot_mount.as_euler('xyz', degrees=False)

    print("\nOR, if keeping the arm_mount intermediate link:")
    print("""
    <!-- CALIBRATED TRANSFORM: arm_mount → zivid_base_link -->
    <joint name="mount_to_camera_joint" type="fixed">
      <parent link="zivid_arm_mount"/>
      <child link="zivid_base_link"/>
      <origin xyz="{:.5f} {:.5f} {:.5f}"
              rpy="{:.5f} {:.5f} {:.5f}"/>
    </joint>
    """.format(trans_mount[0], trans_mount[1], trans_mount[2],
               rpy_mount[0], rpy_mount[1], rpy_mount[2]))


if __name__ == "__main__":
    main()