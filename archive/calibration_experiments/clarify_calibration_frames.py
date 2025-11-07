#!/usr/bin/env python3
"""
Clarify exactly what frames the calibration represents.
"""

import numpy as np


def main():
    print("=" * 60)
    print("CLARIFYING CALIBRATION FRAMES")
    print("=" * 60)

    print("\n1. YOUR HAND-EYE CALIBRATION REPRESENTS:")
    print("   FROM: flange (robot end-effector)")
    print("   TO:   zivid_optical_frame (camera optical center)")
    print("\n   This is the transform that positions the camera's")
    print("   optical center relative to the robot's flange.")

    print("\n2. THE CALIBRATION VALUES:")
    print("   Translation: [-54.35, -104.90, -191.39] mm")
    print("   This means the camera's optical center is:")
    print("   - 54mm to the LEFT of the flange (-X)")
    print("   - 105mm BACKWARD from the flange (-Y)")
    print("   - 191mm BELOW the flange (-Z)")

    print("\n3. WHAT IS 'OPTICAL FRAME'?")
    print("   The optical frame is the camera's viewpoint, where:")
    print("   - Z-axis points forward (into the scene)")
    print("   - X-axis points right in the image")
    print("   - Y-axis points down in the image")
    print("   - Origin is at the camera's optical center (inside the camera)")

    print("\n4. WHAT IS 'BASE_LINK'?")
    print("   The zivid_base_link is the camera body reference frame,")
    print("   typically at the mounting surface or base of the camera.")

    print("\n5. THE INTERNAL OFFSET:")
    print("   From zivid_base_link to zivid_optical_frame:")
    print("   Translation: [49.0, 32.02, 29.5] mm")
    print("   This is the physical offset from the camera mount")
    print("   to the optical center inside the camera.")

    print("\n6. YOUR URDF CHAIN:")
    print("   flange")
    print("     ↓ (5mm forward)")
    print("   zivid_arm_mount (mounting bracket)")
    print("     ↓ (we calculate this)")
    print("   zivid_base_link (camera body/mount point)")
    print("     ↓ (49mm internal offset)")
    print("   zivid_optical_frame (optical center)")

    print("\n7. WHAT WE CALCULATED:")
    print("   Your calibration gives: flange → optical_frame")
    print("   We calculated: arm_mount → base_link")
    print("   Such that the complete chain matches your calibration.")

    print("\n8. IS 188mm MOVEMENT REASONABLE?")
    print("   The camera moved 188mm from the uncalibrated position.")
    print("   This could mean:")
    print("   a) The original URDF had incorrect/placeholder values")
    print("   b) The camera mount was adjusted after URDF was created")
    print("   c) There's an issue with the calibration")

    print("\n9. TO VERIFY PHYSICALLY:")
    print("   Measure from the robot flange to the camera lens:")
    print("   - Should be roughly 191mm vertical distance")
    print("   - Should be roughly 105mm horizontal offset")
    print("   - Camera should be on the side/below the flange")


if __name__ == "__main__":
    main()