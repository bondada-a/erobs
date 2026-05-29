#!/usr/bin/env python3
"""Wafer-present detector for the red-filled spincoater chuck (RE-PICKUP, WIP).

The empty-pocket detector (detect_pocket.py) finds a BRIGHT bare-metal square.
When a silicon wafer sits in the pocket, the target is instead a DARK grey
mirror square. This script detects the wafer as the largest non-red region
inside the red chuck field.

STATUS: validated on the IDEAL wafer position (aspect 1.03, sol 0.96) but FAILS
on non-ideal/rotated positions when a specular glint on the wafer bridges to a
screw-head reflection, or thin red coverage merges in (see images 16/17 and
README §8 Q2).

RECOMMENDED next iteration (not yet implemented): instead of "largest non-red
blob", detect the pocket as the INTERIOR HOLE of the red field:
  1. red mask -> close -> the red field is an annulus/disc with a hole.
  2. fill the field's external contour (cv2.drawContours filled).
  3. pocket = filled_field AND NOT red  (the hole, wherever the wafer is).
  4. minAreaRect on that.
This keys off the stable RED BOUNDARY, not the wafer's unpredictable mirror
appearance, so glints on the wafer no longer corrupt the fit.

Usage: detect_wafer.py <image> [cx cy half]
"""
import sys, cv2, numpy as np

def red_mask(hsv):
    m1 = cv2.inRange(hsv, (0, 60, 40), (14, 255, 255))
    m2 = cv2.inRange(hsv, (166, 60, 40), (179, 255, 255))
    return cv2.bitwise_or(m1, m2)

def locate_chuck(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red = red_mask(hsv)
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, np.ones((25, 25), np.uint8))
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    cnts, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
    r = int(np.sqrt(cv2.contourArea(c) / np.pi))
    return cx, cy, max(120, int(r * 1.4))

def detect_wafer_v1_nonred(img, cx, cy, half):
    """Largest non-red region inside field. Works for ideal, fragile otherwise."""
    x0, y0 = max(0, cx - half), max(0, cy - half)
    roi = img[y0:y0 + 2 * half, x0:x0 + 2 * half]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    red = red_mask(hsv)
    redc = cv2.morphologyEx(red, cv2.MORPH_CLOSE, np.ones((21, 21), np.uint8))
    cnts, _ = cv2.findContours(redc, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    field = max(cnts, key=cv2.contourArea)
    fmask = np.zeros_like(red)
    cv2.drawContours(fmask, [field], -1, 255, -1)
    notred = cv2.bitwise_and(fmask, cv2.bitwise_not(redc))
    notred = cv2.morphologyEx(notred, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
    notred = cv2.morphologyEx(notred, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(notred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in sorted(cnts, key=cv2.contourArea, reverse=True):
        a = cv2.contourArea(c)
        if a < 1500:
            continue
        (rx, ry), (rw, rh), ang = cv2.minAreaRect(c)
        asp = max(rw, rh) / (min(rw, rh) + 1e-6)
        sol = a / (cv2.contourArea(cv2.convexHull(c)) + 1e-6)
        return dict(cx=rx + x0, cy=ry + y0, angle=ang % 90, w=rw, h=rh,
                    asp=asp, sol=sol, area=a)
    return None

def detect_pocket_v2_hole(img, cx, cy, half):
    """RECOMMENDED: pocket = interior hole of the red field (glint-immune).

    Works whether the pocket is empty or holds a wafer — it finds the gap in the
    red, not the wafer itself. UNTESTED on hardware; implement & validate next.
    """
    x0, y0 = max(0, cx - half), max(0, cy - half)
    roi = img[y0:y0 + 2 * half, x0:x0 + 2 * half]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    red = red_mask(hsv)
    redc = cv2.morphologyEx(red, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    cnts, _ = cv2.findContours(redc, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    field = max(cnts, key=cv2.contourArea)
    filled = np.zeros_like(red)
    cv2.drawContours(filled, [field], -1, 255, -1)   # solid disc, hole included
    hole = cv2.bitwise_and(filled, cv2.bitwise_not(redc))
    hole = cv2.morphologyEx(hole, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    hole = cv2.morphologyEx(hole, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    cnts, _ = cv2.findContours(hole, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in sorted(cnts, key=cv2.contourArea, reverse=True):
        a = cv2.contourArea(c)
        if a < 1500:
            continue
        (rx, ry), (rw, rh), ang = cv2.minAreaRect(c)
        asp = max(rw, rh) / (min(rw, rh) + 1e-6)
        sol = a / (cv2.contourArea(cv2.convexHull(c)) + 1e-6)
        if best is None:
            best = dict(cx=rx + x0, cy=ry + y0, angle=ang % 90, w=rw, h=rh,
                        asp=asp, sol=sol, area=a)
    return best

if __name__ == "__main__":
    img = cv2.imread(sys.argv[1])
    if len(sys.argv) >= 5:
        cx, cy, half = int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
    else:
        loc = locate_chuck(img)
        if loc is None:
            print("FAIL: no red field"); sys.exit(1)
        cx, cy, half = loc
    for name, fn in [("v1_nonred", detect_wafer_v1_nonred),
                     ("v2_hole", detect_pocket_v2_hole)]:
        r = fn(img, cx, cy, half)
        if r is None:
            print(f"{name}: FAIL")
        else:
            print(f"{name}: center=({r['cx']:.0f},{r['cy']:.0f}) "
                  f"angle_mod90={r['angle']:.1f} asp={r['asp']:.2f} "
                  f"sol={r['sol']:.2f} area={r['area']:.0f}")
