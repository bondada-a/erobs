#!/usr/bin/env python3
"""
Check the new calibration matrix values.
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
    print("=" * 70)
    print("CHECKING NEW CALIBRATION MATRIX")
    print("=" * 70)

    # OLD calibration
    T_old = np.array([
        [0.9984258, 0.01653194, 0.0535967, -54.35176],
        [-0.01812541, 0.9994039, 0.02938225, -104.9027],
        [-0.05307901, -0.03030745, 0.9981303, -191.3902],
        [0, 0, 0, 1]
    ])

    # NEW calibration
    T_new = np.array([
        [0.997725,  0.027249,  0.061669, -59.955345],
        [-0.028968,  0.999211,  0.027162, -103.841217],
        [-0.060880, -0.028886,  0.997727,  9.612831],
        [0.000000,  0.000000,  0.000000,  1.000000]
    ])

    print("\n1. OLD CALIBRATION (currently in use):")
    print(f"   Translation (mm): {T_old[:3, 3]}")
    print(f"   Translation (m):  {T_old[:3, 3] / 1000}")

    print("\n2. NEW CALIBRATION:")
    print(f"   Translation (mm): {T_new[:3, 3]}")
    print(f"   Translation (m):  {T_new[:3, 3] / 1000}")

    print("\n3. KEY DIFFERENCE:")
    diff = T_new[:3, 3] - T_old[:3, 3]
    print(f"   ΔX: {diff[0]:+.1f} mm")
    print(f"   ΔY: {diff[1]:+.1f} mm")
    print(f"   ΔZ: {diff[2]:+.1f} mm ← HUGE CHANGE!")
    print(f"   Total: {np.linalg.norm(diff):.1f} mm")

    print("\n4. COMPARING NEW CALIBRATION TO YOUR MEASUREMENTS:")
    print("   You measured: X=1cm, Y=-6cm, Z=-11cm")
    measured = np.array([0.01, -0.06, -0.11])
    new_calib_m = T_new[:3, 3] / 1000
    print(f"   New calibration: X={new_calib_m[0]*100:.1f}cm, Y={new_calib_m[1]*100:.1f}cm, Z={new_calib_m[2]*100:.1f}cm")

    diff_from_measured = new_calib_m - measured
    print(f"   Difference: X={diff_from_measured[0]*100:+.1f}cm, Y={diff_from_measured[1]*100:+.1f}cm, Z={diff_from_measured[2]*100:+.1f}cm")
    print(f"   Distance from measured: {np.linalg.norm(diff_from_measured)*100:.1f}cm")

    print("\n5. CALCULATING NEW URDF VALUES:")

    # Convert to meters
    T_new_m = T_new.copy()
    T_new_m[:3, 3] /= 1000.0

    # Known transforms
    T_tool0_mount = xyz_rpy_to_matrix(
        np.array([0.00000, 0.00000, 0.00500]),
        np.array([0.00000, 0.00000, 3.14159])
    )

    T_base_optical = xyz_rpy_to_matrix(
        np.array([0.049, 0.03202, 0.0295]),
        np.array([-1.5707963267948966, 0, -1.6144295580947547])
    )

    # Calculate mount→base
    T_mount_base = np.linalg.inv(T_tool0_mount) @ T_new_m @ np.linalg.inv(T_base_optical)

    xyz_mount_base, rpy_mount_base = matrix_to_xyz_rpy(T_mount_base)

    print(f"\n   NEW mount_to_camera_joint values:")
    print(f"   xyz=\"{xyz_mount_base[0]:.5f} {xyz_mount_base[1]:.5f} {xyz_mount_base[2]:.5f}\"")
    print(f"   rpy=\"{rpy_mount_base[0]:.5f} {rpy_mount_base[1]:.5f} {rpy_mount_base[2]:.5f}\"")

    print("\n6. CURRENT XACRO VALUES:")
    current_xyz = np.array([0.02234, 0.07744, -0.24656])
    print(f"   xyz=\"{current_xyz[0]:.5f} {current_xyz[1]:.5f} {current_xyz[2]:.5f}\"")

    print("\n7. DIFFERENCE:")
    diff_xyz = xyz_mount_base - current_xyz
    print(f"   Δxyz: {diff_xyz * 1000} mm")
    print(f"   Most significant: ΔZ = {diff_xyz[2]*1000:.0f}mm!")

    # Verify
    print("\n8. VERIFICATION:")
    T_verify = T_tool0_mount @ T_mount_base @ T_base_optical
    xyz_verify = T_verify[:3, 3]
    print(f"   Calculated tool0→optical: {xyz_verify}")
    print(f"   New calibration:          {T_new_m[:3, 3]}")
    print(f"   Match: {np.allclose(xyz_verify, T_new_m[:3, 3], atol=1e-6)}")


if __name__ == "__main__":
    main()