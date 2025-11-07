#!/usr/bin/env python3
"""
Calculate the exact mount_to_camera transform needed.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


def xyz_rpy_to_matrix(xyz, rpy):
    """Convert xyz and rpy to 4x4 transformation matrix."""
    T = np.eye(4)
    T[:3, 3] = xyz
    T[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
    return T


def matrix_to_xyz_rpy(T):
    """Extract xyz and rpy from transformation matrix."""
    xyz = T[:3, 3]
    rot = R.from_matrix(T[:3, :3])
    rpy = rot.as_euler('xyz', degrees=False)
    return xyz, rpy


def main():
    print("=" * 60)
    print("CALCULATING EXACT MOUNT_TO_CAMERA TRANSFORM")
    print("=" * 60)

    # Your calibration: flange → optical_frame (in millimeters)
    T_flange_optical_calib = np.array([
        [0.9984258, 0.01653194, 0.0535967, -54.35176],
        [-0.01812541, 0.9994039, 0.02938225, -104.9027],
        [-0.05307901, -0.03030745, 0.9981303, -191.3902],
        [0, 0, 0, 1]
    ])
    # Convert to meters
    T_flange_optical_calib[:3, 3] /= 1000.0

    print("\n1. CALIBRATION (flange → optical_frame):")
    xyz_calib, rpy_calib = matrix_to_xyz_rpy(T_flange_optical_calib)
    print(f"   xyz: {xyz_calib}")
    print(f"   rpy: {rpy_calib}")

    # Known transforms from URDF
    print("\n2. KNOWN TRANSFORMS:")

    # flange → arm_mount (from ur_with_zivid_hande.xacro line 112-113)
    xyz_flange_mount = np.array([0.005, 0, 0])
    rpy_flange_mount = np.array([-1.5708, 0, -1.5708])
    T_flange_mount = xyz_rpy_to_matrix(xyz_flange_mount, rpy_flange_mount)
    print(f"   flange → arm_mount: xyz={xyz_flange_mount}, rpy={rpy_flange_mount}")

    # base_link → optical_frame (internal camera offset from zivid_camera.xacro)
    xyz_base_optical = np.array([0.049, 0.03202, 0.0295])
    rpy_base_optical = np.array([-1.5707963267948966, 0, -1.6144295580947547])
    T_base_optical = xyz_rpy_to_matrix(xyz_base_optical, rpy_base_optical)
    print(f"   base_link → optical: xyz={xyz_base_optical}, rpy={rpy_base_optical}")

    print("\n3. CALCULATING mount → base_link:")
    print("   We need: flange→mount→base→optical = flange→optical(calibrated)")
    print("   Therefore: mount→base = inv(flange→mount) * (flange→optical) * inv(base→optical)")

    # Calculate the required mount → base transform
    T_mount_base = np.linalg.inv(T_flange_mount) @ T_flange_optical_calib @ np.linalg.inv(T_base_optical)

    xyz_mount_base, rpy_mount_base = matrix_to_xyz_rpy(T_mount_base)

    print(f"\n4. RESULT for mount_to_camera_joint:")
    print(f"   xyz=\"{xyz_mount_base[0]:.5f} {xyz_mount_base[1]:.5f} {xyz_mount_base[2]:.5f}\"")
    print(f"   rpy=\"{rpy_mount_base[0]:.5f} {rpy_mount_base[1]:.5f} {rpy_mount_base[2]:.5f}\"")

    # Verify the calculation
    print("\n5. VERIFICATION:")
    T_verify = T_flange_mount @ T_mount_base @ T_base_optical
    xyz_verify, rpy_verify = matrix_to_xyz_rpy(T_verify)
    print(f"   Calculated flange→optical: {xyz_verify}")
    print(f"   Should match calibration:  {xyz_calib}")
    print(f"   Error (mm): {(xyz_verify - xyz_calib) * 1000}")

    # Compare with uncalibrated values
    print("\n6. COMPARISON WITH UNCALIBRATED:")
    xyz_uncalib = np.array([0.025, 0.062, -0.049])
    rpy_uncalib = np.array([0, -1.5708, -1.5708])
    print(f"   Uncalibrated: xyz={xyz_uncalib}, rpy={rpy_uncalib}")
    print(f"   Difference in position (mm): {(xyz_mount_base - xyz_uncalib) * 1000}")
    print(f"   Distance moved: {np.linalg.norm(xyz_mount_base - xyz_uncalib) * 1000:.1f} mm")


if __name__ == "__main__":
    main()