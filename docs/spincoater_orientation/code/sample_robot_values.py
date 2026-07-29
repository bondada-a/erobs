#!/usr/bin/env python3
"""Print the spincoater-sample YOLO values consumed by the robot.

Usage: python3 sample_robot_values.py <image> [k-offset-deg]
"""
import sys
from pathlib import Path

import cv2
import numpy as np

repo = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(repo / "src" / "beambot"))

from beambot.detection.spincoater import detect_spincoater_sample
from beambot.pipeline.motion_target import snap_j6


if len(sys.argv) not in (2, 3):
    raise SystemExit(f"Usage: {sys.argv[0]} <image> [k-offset-deg]")

source = Path(sys.argv[1])
image = cv2.imread(str(source))
if image is None:
    raise SystemExit(f"FAIL: cannot read {source}")

result = detect_spincoater_sample(image)
if result is None:
    raise SystemExit("FAIL: no sample detected")

k_offset = float(sys.argv[2]) if len(sys.argv) == 3 else 0.0
angle = result["angle_mod90"]
correction = snap_j6(0.0, angle, k_offset)
output = source.with_name(f"{source.stem}_orientation{source.suffix}")
box = cv2.boxPoints(
    (
        result["center_px"],
        (result["width"], result["height"]),
        result["angle_raw"],
    )
).astype(np.int32)
label = (
    f"sample={angle:.1f} deg mod90 | J6={correction:+.1f} deg | "
    f"conf={result['confidence']:.2f}"
)

cv2.drawContours(image, [box], 0, (0, 255, 0), 4)
cv2.circle(image, result["center_px"], 8, (0, 0, 255), -1)
cv2.putText(
    image, label, (40, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 6
)
cv2.putText(
    image, label, (40, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3
)
if not cv2.imwrite(str(output), image):
    raise SystemExit(f"FAIL: cannot write {output}")

print(f"Sample center:       {result['center_px']} px")
print(f"Sample angle mod 90: {angle:.2f} deg")
print(f"Raw rectangle angle: {result['angle_raw']:.2f} deg")
print(f"J6 correction:       {correction:+.2f} deg (K={k_offset:g})")
print(f"Confidence:          {result['confidence']:.3f}")
print(f"Aspect:              {result['aspect']:.3f}")
print(f"Mask points:         {result['mask_points']}")
print(f"Overlay saved:       {output}")
