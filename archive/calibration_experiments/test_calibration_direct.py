#!/usr/bin/env python3
"""
Test if calibration gives flange → base_link directly
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

# Your hand-eye calibration result (convert mm to meters)
H_calib = np.array([
    [0.9977247, 0.02724888, 0.06166882, -59.95535/1000],
    [-0.02896834, 0.9992113, 0.02716185, -103.8412/1000],
    [-0.06088005, -0.02888649, 0.997727, 9.612831/1000],
    [0, 0, 0, 1]
])

# Extract translation
translation = H_calib[:3, 3]
x, y, z = translation

# Extract rotation
rotation_matrix = H_calib[:3, :3]
rot = R.from_matrix(rotation_matrix)
rpy = rot.as_euler('xyz', degrees=False)
roll, pitch, yaw = rpy

print("=" * 70)
print("HYPOTHESIS: Calibration gives flange → base_link DIRECTLY")
print("=" * 70)
print(f"\nTranslation (meters):")
print(f"  x = {x:.6f}")
print(f"  y = {y:.6f}")
print(f"  z = {z:.6f}")
print(f"\nRotation (radians):")
print(f"  roll  = {roll:.6f}")
print(f"  pitch = {pitch:.6f}")
print(f"  yaw   = {yaw:.6f}")
print(f"\nRotation (degrees):")
print(f"  roll  = {np.degrees(roll):.3f}°")
print(f"  pitch = {np.degrees(pitch):.3f}°")
print(f"  yaw   = {np.degrees(yaw):.3f}°")

print("\n" + "=" * 70)
print("If this were flange → base_link, use this in URDF:")
print("=" * 70)
print(f'<origin xyz="{x:.6f} {y:.6f} {z:.6f}" rpy="{roll:.6f} {pitch:.6f} {yaw:.6f}"/>')

print("\n" + "=" * 70)
print("Does the rotation look reasonable?")
print("=" * 70)
print("For a camera mounted on the robot flange:")
print("- Should it be nearly aligned with flange? (roll/pitch/yaw all small ~0-10°)")
print("- Or rotated ~90° in some axis?")
print("\nYour values suggest:")
if abs(np.degrees(roll)) < 10 and abs(np.degrees(pitch)) < 10 and abs(np.degrees(yaw)) < 10:
    print("  → Nearly aligned! Calibration might be flange → base_link")
else:
    print(f"  → Significant rotation present")
print("\n")
