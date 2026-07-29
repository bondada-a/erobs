#!/usr/bin/env python3
"""Detect the empty spincoater pocket and save an annotated image.

Usage: python3 annotate_pocket.py <input-image> [output-image]
"""
import sys
from pathlib import Path

import cv2
import numpy as np

from detect_pocket import detect, locate_chuck


def main():
    if len(sys.argv) not in (2, 3):
        raise SystemExit(f"Usage: {sys.argv[0]} <input-image> [output-image]")

    source = Path(sys.argv[1])
    output = (
        Path(sys.argv[2])
        if len(sys.argv) == 3
        else source.with_name(f"{source.stem}_detection{source.suffix}")
    )
    image = cv2.imread(str(source))
    if image is None:
        raise SystemExit(f"FAIL: cannot read {source}")

    location = locate_chuck(image)
    if location is None:
        raise SystemExit("FAIL: no red chuck field detected")

    cx, cy, half, _ = location
    result = detect(image, cx, cy, half)
    if result is None:
        raise SystemExit("FAIL: no valid pocket detected")

    angle = result["angle"]
    j6_delta = (angle + 45) % 90 - 45
    box = cv2.boxPoints(
        (
            (result["cx"], result["cy"]),
            (result["w"], result["h"]),
            result["raw_angle"],
        )
    ).astype(np.int32)
    center = (round(result["cx"]), round(result["cy"]))
    label = f"pocket={angle:.1f} deg mod90 | nearest J6={j6_delta:+.1f} deg"

    cv2.drawContours(image, [box], 0, (0, 255, 0), 4)
    cv2.circle(image, center, 8, (0, 0, 255), -1)
    cv2.putText(
        image, label, (40, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 6
    )
    cv2.putText(
        image, label, (40, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3
    )

    if not cv2.imwrite(str(output), image):
        raise SystemExit(f"FAIL: cannot write {output}")

    equivalents = ", ".join(f"{angle + 90 * i:.2f}" for i in range(4))
    print(f"Pocket center:       ({result['cx']:.1f}, {result['cy']:.1f}) px")
    print(f"Pocket angle mod 90: {angle:.2f} deg")
    print(f"Equivalent angles:   {equivalents} deg")
    print(f"Nearest J6 delta:    {j6_delta:+.2f} deg (K=0)")
    print(f"Aspect / solidity:   {result['asp']:.3f} / {result['sol']:.3f}")
    print(f"Overlay saved:       {output}")


if __name__ == "__main__":
    main()
