#!/usr/bin/env python3
"""
Verify we're interpreting the calibration correctly.
Check if it's flange->optical or optical->flange.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


def main():
    print("=" * 60)
    print("VERIFYING CALIBRATION INTERPRETATION")
    print("=" * 60)

    # Your calibration matrix from hand_eye_transform.yaml
    hand_eye_matrix = np.array([
        [0.9984258, 0.01653194, 0.0535967, -54.35176],
        [-0.01812541, 0.9994039, 0.02938225, -104.9027],
        [-0.05307901, -0.03030745, 0.9981303, -191.3902],
        [0, 0, 0, 1]
    ])

    print("\n1. WHAT DOES THE CALIBRATION REPRESENT?")
    print("   According to Zivid docs for eye-in-hand calibration:")
    print("   - The transform is: robot end-effector → camera")
    print("   - In ROS terms: flange → optical_frame")
    print("   - Direction: FROM flange TO optical_frame")

    trans_mm = hand_eye_matrix[:3, 3]
    print(f"\n   Translation (mm): {trans_mm}")
    print("   This means: optical frame is located at this position")
    print("   relative to the flange coordinate system")

    print("\n2. SANITY CHECK THE VALUES:")
    print(f"   X: {trans_mm[0]:.1f}mm - Camera is ~54mm to the left of flange")
    print(f"   Y: {trans_mm[1]:.1f}mm - Camera is ~105mm backward from flange")
    print(f"   Z: {trans_mm[2]:.1f}mm - Camera is ~191mm below flange")
    print("\n   These values seem reasonable for a camera mounted on the side")

    print("\n3. THE PROBLEM WITH CURRENT APPROACH:")
    print("   We have an existing URDF chain:")
    print("   flange → arm_mount → base_link → optical_frame")
    print("            (0.005,0,0)  (0.025,0.062,-0.049)  (0.049,0.032,0.029)")

    # Calculate current uncalibrated transform
    uncalib_chain = np.array([
        0.005 + 0.025 + 0.049,
        0 + 0.062 + 0.032,
        0 + (-0.049) + 0.029
    ]) * 1000  # Convert to mm for comparison

    print(f"\n   Current uncalibrated total offset (mm): {uncalib_chain}")
    print(f"   Calibrated offset (mm): {trans_mm}")
    print(f"   Difference (mm): {trans_mm - uncalib_chain}")

    print("\n4. THE KEY QUESTION:")
    print("   Is the large difference (100-200mm) realistic?")
    print("   - If camera was physically moved: YES")
    print("   - If same physical setup: NO - suggests interpretation error")

    print("\n5. CHECKING IF WE SHOULD USE INVERSE:")
    hand_eye_inv = np.linalg.inv(hand_eye_matrix)
    trans_inv_mm = hand_eye_inv[:3, 3]
    print(f"\n   Inverse translation (mm): {trans_inv_mm}")
    print("   This would mean optical frame is the reference")

    print("\n6. LET'S CHECK THE ROTATION:")
    rot = R.from_matrix(hand_eye_matrix[:3, :3])
    rpy_deg = rot.as_euler('xyz', degrees=True)
    print(f"   Rotation (degrees): {rpy_deg}")
    print("   Small rotations (~1-3°) - this is typical calibration refinement")

    print("\n7. RECOMMENDED APPROACH:")
    print("   Since rotations are small (refinement-level),")
    print("   but translations are large (100+ mm),")
    print("   This suggests:")
    print("   a) Camera has physically moved since 'uncalibrated' values were set")
    print("   b) OR we're applying transform incorrectly")

    print("\n8. TEST: What if calibration is RELATIVE correction?")
    print("   Some calibration tools output the CORRECTION to apply,")
    print("   not the absolute transform.")

    # If it's a correction, we'd add it to existing
    corrected_trans = uncalib_chain + trans_mm
    print(f"   If additive correction: {corrected_trans} mm")
    print("   This gives huge values - unlikely correct")

    print("\n9. SIMPLEST TEST:")
    print("   Apply calibration directly as mount_to_camera transform")
    print("   (replacing the uncalibrated values entirely)")

    # We need to account for the arm mount offset
    # The calibration is flange->optical, but we need mount->base
    # So we need to remove both the arm_mount offset and optical offset

    print("\n   Instead of complex math, try this simple approach:")
    print("   - Keep arm_mount at (0,0,0) relative to flange")
    print("   - Put calibration at mount_to_camera")
    print("   - Remove the optical offset")


if __name__ == "__main__":
    main()