#!/usr/bin/env python3
"""
Compare uncalibrated vs physical measurements vs calibration.
"""

import numpy as np


def main():
    print("=" * 60)
    print("COMPARING ALL THREE APPROACHES")
    print("=" * 60)

    # Original uncalibrated values
    uncalib_xyz = np.array([0.025, 0.062, -0.049])
    uncalib_rpy = np.array([0, -1.5708, -1.5708])

    # Based on physical measurements
    physical_xyz = np.array([0.03253, 0.16017, 0.03701])
    physical_rpy = np.array([-1.58732, -0.00999, -1.60094])

    # From hand-eye calibration
    calib_xyz = np.array([0.07744, 0.24156, -0.02734])
    calib_rpy = np.array([-1.58733, -0.00999, -1.60095])

    print("\n1. UNCALIBRATED (original URDF):")
    print(f"   xyz: {uncalib_xyz} m")
    print(f"   rpy: {uncalib_rpy} rad")

    print("\n2. PHYSICAL MEASUREMENTS:")
    print(f"   xyz: {physical_xyz} m")
    print(f"   rpy: {physical_rpy} rad")

    print("\n3. HAND-EYE CALIBRATION:")
    print(f"   xyz: {calib_xyz} m")
    print(f"   rpy: {calib_rpy} rad")

    print("\n" + "=" * 60)
    print("DIFFERENCES FROM UNCALIBRATED")
    print("=" * 60)

    # Difference: Physical - Uncalibrated
    diff_physical = physical_xyz - uncalib_xyz
    print("\nPhysical Measurement vs Uncalibrated:")
    print(f"   ΔX: {diff_physical[0]*1000:+.1f} mm ({diff_physical[0]*100:+.1f} cm)")
    print(f"   ΔY: {diff_physical[1]*1000:+.1f} mm ({diff_physical[1]*100:+.1f} cm)")
    print(f"   ΔZ: {diff_physical[2]*1000:+.1f} mm ({diff_physical[2]*100:+.1f} cm)")
    print(f"   Total distance: {np.linalg.norm(diff_physical)*1000:.1f} mm ({np.linalg.norm(diff_physical)*100:.1f} cm)")

    # Difference: Calibration - Uncalibrated
    diff_calib = calib_xyz - uncalib_xyz
    print("\nCalibration vs Uncalibrated:")
    print(f"   ΔX: {diff_calib[0]*1000:+.1f} mm ({diff_calib[0]*100:+.1f} cm)")
    print(f"   ΔY: {diff_calib[1]*1000:+.1f} mm ({diff_calib[1]*100:+.1f} cm)")
    print(f"   ΔZ: {diff_calib[2]*1000:+.1f} mm ({diff_calib[2]*100:+.1f} cm)")
    print(f"   Total distance: {np.linalg.norm(diff_calib)*1000:.1f} mm ({np.linalg.norm(diff_calib)*100:.1f} cm)")

    # Difference: Calibration - Physical
    diff_calib_physical = calib_xyz - physical_xyz
    print("\nCalibration vs Physical Measurement:")
    print(f"   ΔX: {diff_calib_physical[0]*1000:+.1f} mm")
    print(f"   ΔY: {diff_calib_physical[1]*1000:+.1f} mm")
    print(f"   ΔZ: {diff_calib_physical[2]*1000:+.1f} mm")
    print(f"   Total distance: {np.linalg.norm(diff_calib_physical)*1000:.1f} mm")

    print("\n" + "=" * 60)
    print("ROTATION DIFFERENCES")
    print("=" * 60)

    diff_rpy_physical = physical_rpy - uncalib_rpy
    diff_rpy_calib = calib_rpy - uncalib_rpy

    print("\nPhysical vs Uncalibrated (degrees):")
    print(f"   Δroll:  {np.degrees(diff_rpy_physical[0]):+.1f}°")
    print(f"   Δpitch: {np.degrees(diff_rpy_physical[1]):+.1f}°")
    print(f"   Δyaw:   {np.degrees(diff_rpy_physical[2]):+.1f}°")

    print("\nCalibration vs Uncalibrated (degrees):")
    print(f"   Δroll:  {np.degrees(diff_rpy_calib[0]):+.1f}°")
    print(f"   Δpitch: {np.degrees(diff_rpy_calib[1]):+.1f}°")
    print(f"   Δyaw:   {np.degrees(diff_rpy_calib[2]):+.1f}°")

    print("\n" + "=" * 60)
    print("RECOMMENDATION")
    print("=" * 60)
    print("\nThe physical measurements differ from uncalibrated by:")
    print(f"   ~{np.linalg.norm(diff_physical)*100:.0f}cm total")
    print("\nThe calibration differs from uncalibrated by:")
    print(f"   ~{np.linalg.norm(diff_calib)*100:.0f}cm total")
    print("\nThe calibration differs from physical measurement by:")
    print(f"   ~{np.linalg.norm(diff_calib_physical)*100:.0f}cm total")
    print("\nSince calibration disagrees significantly with your physical")
    print("measurements, there was likely an error in the calibration process.")


if __name__ == "__main__":
    main()