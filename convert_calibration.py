#!/usr/bin/env python3
"""Convert Zivid calibration transform to URDF format"""
import numpy as np
from scipy.spatial.transform import Rotation

# Transform matrix from Zivid calibration (flange -> camera)
transform = np.array([
    [0.99881,  -0.00657,   0.04839, -56.59406],
    [0.00529,   0.99963,   0.02655, -104.87761],
    [-0.04855, -0.02627,   0.99848,   7.90229],
    [0.0,       0.0,       0.0,       1.0]
])

# Extract translation (convert mm to meters)
translation = transform[:3, 3] / 1000.0

# Extract rotation matrix
rotation_matrix = transform[:3, :3]

# Convert rotation matrix to RPY (in radians)
r = Rotation.from_matrix(rotation_matrix)
rpy_rad = r.as_euler('xyz', degrees=False)
rpy_deg = r.as_euler('xyz', degrees=True)

print("="*60)
print("  Calibration Results - URDF Format")
print("="*60)
print(f"\nTranslation (meters):")
print(f"  xyz=\"{translation[0]:.6f} {translation[1]:.6f} {translation[2]:.6f}\"")
print(f"\nRotation (radians):")
print(f"  rpy=\"{rpy_rad[0]:.6f} {rpy_rad[1]:.6f} {rpy_rad[2]:.6f}\"")
print(f"\nRotation (degrees for reference):")
print(f"  roll={rpy_deg[0]:.3f}°, pitch={rpy_deg[1]:.3f}°, yaw={rpy_deg[2]:.3f}°")
print("\n" + "="*60)
print("  Update zivid_camera_mount.xacro:")
print("="*60)
print(f"\nReplace the line:")
print(f"  <origin xyz=\"0.025 0 -0.105\" rpy=\"-1.5708 0 -1.5708\"/>")
print(f"\nWith:")
print(f"  <origin xyz=\"{translation[0]:.6f} {translation[1]:.6f} {translation[2]:.6f}\" rpy=\"{rpy_rad[0]:.6f} {rpy_rad[1]:.6f} {rpy_rad[2]:.6f}\"/>")
print("="*60)
