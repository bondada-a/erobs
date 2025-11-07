#!/usr/bin/env python3
"""
Apply the hand-eye calibration correctly as a small correction to existing values.
The calibration should be a refinement, not a complete repositioning.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


def main():
    print("=" * 60)
    print("CORRECT CALIBRATION APPLICATION")
    print("=" * 60)

    # Your calibration gives: flange → optical_frame
    # This is the "true" measured transform
    calib_trans_mm = np.array([-54.35176, -104.9027, -191.3902])
    calib_trans_m = calib_trans_mm / 1000.0

    # The uncalibrated values that work (mount → base_link)
    uncalib_xyz = np.array([0.025, 0.062, -0.049])
    uncalib_rpy = np.array([0, -1.5708, -1.5708])

    print("\n1. Current WORKING uncalibrated values:")
    print(f"   xyz: {uncalib_xyz}")
    print(f"   rpy: {uncalib_rpy}")

    # The expected theoretical transform (from CAD/design)
    # We need to find what the "expected" flange→optical transform would be
    # with the uncalibrated values

    # Arm mount offset (flange → arm_mount)
    flange_to_mount_xyz = np.array([0.005, 0, 0])
    flange_to_mount_rpy = np.array([-1.5708, 0, -1.5708])

    # Internal camera offset (base_link → optical_frame)
    base_to_optical_xyz = np.array([0.049, 0.03202, 0.0295])
    base_to_optical_rpy = np.array([-1.5707963267948966, 0, -1.6144295580947547])

    # Calculate the theoretical flange → optical with uncalibrated values
    # flange → mount → base → optical

    # Build transformation matrices
    def build_transform(xyz, rpy):
        T = np.eye(4)
        T[:3, 3] = xyz
        T[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
        return T

    T_flange_mount = build_transform(flange_to_mount_xyz, flange_to_mount_rpy)
    T_mount_base_uncalib = build_transform(uncalib_xyz, uncalib_rpy)
    T_base_optical = build_transform(base_to_optical_xyz, base_to_optical_rpy)

    # Theoretical uncalibrated: flange → optical
    T_flange_optical_uncalib = T_flange_mount @ T_mount_base_uncalib @ T_base_optical
    uncalib_trans = T_flange_optical_uncalib[:3, 3]
    uncalib_rot = R.from_matrix(T_flange_optical_uncalib[:3, :3])
    uncalib_rpy_optical = uncalib_rot.as_euler('xyz', degrees=False)

    print("\n2. Theoretical uncalibrated flange→optical:")
    print(f"   Translation: {uncalib_trans} m")
    print(f"   Translation: {uncalib_trans * 1000} mm")

    print("\n3. Measured calibrated flange→optical:")
    print(f"   Translation: {calib_trans_m} m")
    print(f"   Translation: {calib_trans_mm} mm")

    print("\n4. Calibration correction needed:")
    correction_mm = calib_trans_mm - (uncalib_trans * 1000)
    correction_m = correction_mm / 1000
    print(f"   Correction: {correction_mm} mm")
    print(f"   Correction: {correction_m} m")

    # Apply a small portion of the correction to the mount_to_camera joint
    # The correction should be distributed, but let's apply it at mount→base level

    # Transform the correction back to the mount frame
    # We need to apply only a portion since there might be mechanical tolerances

    correction_factor = 1.0  # Start with full correction
    corrected_xyz = uncalib_xyz + correction_m * correction_factor

    print("\n5. CORRECTED mount_to_camera_joint values:")
    print(f"   xyz=\"{corrected_xyz[0]:.5f} {corrected_xyz[1]:.5f} {corrected_xyz[2]:.5f}\"")
    print(f"   rpy=\"{uncalib_rpy[0]:.5f} {uncalib_rpy[1]:.5f} {uncalib_rpy[2]:.5f}\"")

    print("\n6. Alternative: Apply smaller correction (50%):")
    corrected_xyz_half = uncalib_xyz + correction_m * 0.5
    print(f"   xyz=\"{corrected_xyz_half[0]:.5f} {corrected_xyz_half[1]:.5f} {corrected_xyz_half[2]:.5f}\"")

    print("\n7. Alternative: Apply very small correction (10%):")
    corrected_xyz_small = uncalib_xyz + correction_m * 0.1
    print(f"   xyz=\"{corrected_xyz_small[0]:.5f} {corrected_xyz_small[1]:.5f} {corrected_xyz_small[2]:.5f}\"")

    # Also consider that the calibration might be giving us the correction
    # in a different reference frame
    print("\n8. Direct replacement approach:")
    print("   If the calibration is meant to directly replace mount→base:")
    print("   Just use small adjustments to the working uncalibrated values")


if __name__ == "__main__":
    main()