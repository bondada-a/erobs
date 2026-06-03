"""Spincoater pocket detection for orientation-aware placement.

Detects the machined pocket in a red-painted spincoater chuck using 2D
flash-lit imagery. The chuck face is painted red (leaving the pocket bare),
so the pocket appears as a bright bare-metal square inside a red field.

Pipeline:
  1. HSV dual-range red mask (handles hue wraparound at 0/179)
  2. Locate chuck as the largest red blob
  3. Restrict to red field, isolate bright bare-metal (high V, low S)
  4. minAreaRect → center, angle (mod 90° for 4-fold symmetry)

Requirements:
  - 2D flash-lit capture (/capture_2d) — NOT the 3D projector capture
  - Chuck centered in frame (flash falloff causes failure off-axis)
  - Opaque red paint with saturation ~200+ for reliable masking
"""

import cv2
import numpy as np


def _red_mask(hsv: np.ndarray) -> np.ndarray:
    """Dual-range red HSV mask handling the hue wraparound at 0/179."""
    m1 = cv2.inRange(hsv, (0, 60, 40), (14, 255, 255))
    m2 = cv2.inRange(hsv, (166, 60, 40), (179, 255, 255))
    return cv2.bitwise_or(m1, m2)


def _locate_chuck(image: np.ndarray) -> tuple[int, int, int] | None:
    """Find the chuck center as the centroid of the largest red blob.

    Returns:
        (cx, cy, half_roi_size) or None if no red field found.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    red = _red_mask(hsv)
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, np.ones((25, 25), np.uint8))
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    cnts, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    r = int(np.sqrt(cv2.contourArea(c) / np.pi))
    return cx, cy, max(120, int(r * 1.4))


def detect_spincoater_pocket(
    image: np.ndarray,
    min_area: int = 1200,
    max_aspect: float = 1.25,
    min_solidity: float = 0.85,
) -> dict | None:
    """Detect the empty pocket in a red-painted spincoater chuck.

    Args:
        image: BGR image from a 2D flash-lit capture (/capture_2d).
        min_area: Minimum contour area in pixels to consider.
        max_aspect: Maximum aspect ratio for a valid square pocket.
        min_solidity: Minimum solidity (contour_area / convex_hull_area).

    Returns:
        Dict with keys:
          - center_px: (x, y) pixel coordinates of pocket center
          - angle_mod90: pocket rotation in degrees [0, 90), mod 90 for
            4-fold symmetry, measured from image horizontal
          - width: fitted rectangle width in pixels
          - height: fitted rectangle height in pixels
          - aspect: aspect ratio (>= 1.0)
          - solidity: contour solidity
          - area: contour area in pixels
        Returns None if detection fails.
    """
    loc = _locate_chuck(image)
    if loc is None:
        return None
    cx, cy, half = loc

    x0 = max(0, cx - half)
    y0 = max(0, cy - half)
    roi = image[y0:y0 + 2 * half, x0:x0 + 2 * half]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Red field mask within ROI
    red = _red_mask(hsv)
    red_closed = cv2.morphologyEx(red, cv2.MORPH_CLOSE, np.ones((21, 21), np.uint8))
    cnts, _ = cv2.findContours(
        red_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not cnts:
        return None

    # Largest red blob = the chuck field
    field = max(cnts, key=cv2.contourArea)
    field_mask = np.zeros_like(red)
    cv2.drawContours(field_mask, [field], -1, 255, -1)

    # Bright bare-metal: high value, low saturation (inside the red field)
    bright = cv2.inRange(hsv, (0, 0, 150), (179, 90, 255))
    bright = cv2.bitwise_and(bright, field_mask)
    bright = cv2.morphologyEx(bright, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    bright = cv2.morphologyEx(bright, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

    cnts, _ = cv2.findContours(
        bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    for c in sorted(cnts, key=cv2.contourArea, reverse=True):
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        (rx, ry), (rw, rh), ang = cv2.minAreaRect(c)
        aspect = max(rw, rh) / (min(rw, rh) + 1e-6)
        solidity = area / (cv2.contourArea(cv2.convexHull(c)) + 1e-6)
        if aspect < max_aspect and solidity > min_solidity:
            return {
                "center_px": (int(rx + x0), int(ry + y0)),
                "angle_mod90": ang % 90,
                "width": rw,
                "height": rh,
                "aspect": round(aspect, 3),
                "solidity": round(solidity, 3),
                "area": int(area),
            }
    return None
