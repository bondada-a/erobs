#!/usr/bin/env python3
"""Test script for ArUco-guided sample contour detection.

Two modes:
  --mode draw   : Show GUI, let user draw ROI rectangle. Computes offset/size relative to tag.
  --mode detect : Use predefined ROI params to detect sample contour.

Usage:
    python3 test_sample_detection.py --mode draw --tag 0
    python3 test_sample_detection.py --mode detect --tag 0
"""

import argparse
import sys

import cv2
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.widgets import RectangleSelector
import numpy as np


# --- Detection Parameters (set after draw mode calibration) ---
ROI_OFFSET_X_MM = 26.2   # ROI center offset from tag in marker +X (mm)
ROI_OFFSET_Y_MM = 0.5    # ROI center offset from tag in marker +Y (mm)
ROI_WIDTH_MM = 28.4       # ROI width in mm
ROI_HEIGHT_MM = 31.6      # ROI height in mm
EDGE_INSET_MM = 4.0       # How far inward from the edge toward center (mm)
SAMPLE_MIN_AREA = 100
SAMPLE_MAX_AREA = 15000
MAX_ASPECT_RATIO = 3.0
BLUR_KERNEL = 5
CANNY_LOW = 50
CANNY_HIGH = 150


def detect_aruco(image, target_id):
    """Detect ArUco markers and return corners for target_id."""
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = detector.detectMarkers(gray)
    except AttributeError:
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters_create()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is None:
        return None, []

    all_ids = ids.flatten().tolist()
    target_corners = None
    for i, mid in enumerate(ids.flatten()):
        if int(mid) == target_id:
            target_corners = corners_list[i][0]
            break

    return target_corners, all_ids


def get_marker_info(corners):
    """Get pixel scale and orientation from marker corners."""
    side_lengths = [np.linalg.norm(corners[(i+1)%4] - corners[i]) for i in range(4)]
    avg_side_px = np.mean(side_lengths)
    marker_size_mm = 20.0  # 4x4_50 printed at 100%
    px_per_mm = avg_side_px / marker_size_mm

    # Marker axes in pixel space
    top_left, top_right, bottom_right, bottom_left = corners
    marker_x = (top_right - top_left)  # marker +X direction
    marker_x = marker_x / np.linalg.norm(marker_x)
    marker_y = (bottom_left - top_left)  # marker +Y direction
    marker_y = marker_y / np.linalg.norm(marker_y)

    center = corners.mean(axis=0)
    return center, px_per_mm, marker_x, marker_y


# ---- DRAW MODE ----

roi_result = {}

def draw_roi_mode(image, tag_corners, tag_id):
    """Show matplotlib GUI for user to draw ROI rectangle."""
    center, px_per_mm, marker_x, marker_y = get_marker_info(tag_corners)

    # Convert BGR to RGB for matplotlib
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    fig, ax = plt.subplots(1, 1, figsize=(14, 11))
    ax.imshow(rgb)

    # Draw tag outline
    tag_poly = plt.Polygon(tag_corners, fill=False, edgecolor='blue', linewidth=2)
    ax.add_patch(tag_poly)
    ax.plot(*center, 'bo', markersize=5)
    ax.text(center[0] - 20, center[1] - 25, f"Tag {tag_id}",
            color='blue', fontsize=10, fontweight='bold')

    ax.set_title("Click and drag to draw ROI around the sample. Close window when done.",
                 fontsize=12)

    def on_select(eclick, erelease):
        roi_result['x1'] = int(min(eclick.xdata, erelease.xdata))
        roi_result['y1'] = int(min(eclick.ydata, erelease.ydata))
        roi_result['x2'] = int(max(eclick.xdata, erelease.xdata))
        roi_result['y2'] = int(max(eclick.ydata, erelease.ydata))
        w_mm = abs(erelease.xdata - eclick.xdata) / px_per_mm
        h_mm = abs(erelease.ydata - eclick.ydata) / px_per_mm
        ax.set_title(f"ROI: {w_mm:.1f} x {h_mm:.1f} mm. Close window when done.", fontsize=12)
        fig.canvas.draw_idle()

    selector = RectangleSelector(ax, on_select, interactive=True,
                                  button=[1], useblit=True,
                                  props=dict(facecolor='green', alpha=0.2, edgecolor='green', linewidth=2))

    plt.tight_layout()
    plt.show()

    if 'x1' not in roi_result:
        print("No ROI drawn.")
        sys.exit(1)

    x1, y1 = roi_result['x1'], roi_result['y1']
    x2, y2 = roi_result['x2'], roi_result['y2']

    # Compute ROI center relative to tag in marker frame (mm)
    roi_center_px = np.array([(x1 + x2) / 2, (y1 + y2) / 2])
    offset_px = roi_center_px - center
    offset_x_mm = np.dot(offset_px, marker_x) / px_per_mm
    offset_y_mm = np.dot(offset_px, marker_y) / px_per_mm
    width_mm = (x2 - x1) / px_per_mm
    height_mm = (y2 - y1) / px_per_mm

    print(f"\n{'='*50}")
    print(f"ROI relative to tag {tag_id}:")
    print(f"  Offset X (marker +X): {offset_x_mm:.1f} mm")
    print(f"  Offset Y (marker +Y): {offset_y_mm:.1f} mm")
    print(f"  Width:  {width_mm:.1f} mm")
    print(f"  Height: {height_mm:.1f} mm")
    print(f"  Scale:  {px_per_mm:.2f} px/mm")
    print(f"\nCopy these values into the script:")
    print(f"  ROI_OFFSET_X_MM = {offset_x_mm:.1f}")
    print(f"  ROI_OFFSET_Y_MM = {offset_y_mm:.1f}")
    print(f"  ROI_WIDTH_MM = {width_mm:.1f}")
    print(f"  ROI_HEIGHT_MM = {height_mm:.1f}")
    print(f"{'='*50}")

    # Run detection in drawn ROI
    roi = image[y1:y2, x1:x2]
    cv2.imwrite("/tmp/sample_roi.jpg", roi)
    print(f"\nSaved ROI: /tmp/sample_roi.jpg")

    roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    sample_contour = find_sample_contour(roi_gray)

    if sample_contour is not None:
        rect = cv2.minAreaRect(sample_contour)
        print(f"\nSample found in ROI:")
        print(f"  Size (px): {rect[1][0]:.1f} x {rect[1][1]:.1f}")
        print(f"  Size (mm): {rect[1][0]/px_per_mm:.1f} x {rect[1][1]/px_per_mm:.1f}")
        print(f"  Angle: {rect[2]:.1f}°")
        print(f"  Area: {cv2.contourArea(sample_contour):.0f} px²")

        annotated = image.copy()
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.polylines(annotated, [tag_corners.astype(int)], True, (255, 0, 0), 2)
        rect_corners = cv2.boxPoints(rect) + np.array([x1, y1])
        cv2.drawContours(annotated, [rect_corners.astype(int)], 0, (0, 255, 0), 2)
        pickup = (int(rect[0][0] + x1), int(rect[0][1] + y1))
        cv2.circle(annotated, pickup, 8, (0, 0, 255), -1)
        cv2.circle(annotated, pickup, 12, (0, 0, 255), 2)
        cv2.imwrite("/tmp/sample_detection_result.jpg", annotated)
        print(f"Saved annotated: /tmp/sample_detection_result.jpg")
    else:
        print("\nNo sample contour found in drawn ROI.")


# ---- DETECT MODE ----

def detect_mode(image, tag_corners, tag_id, strategy):
    """Use predefined ROI params to detect sample."""
    center, px_per_mm, marker_x, marker_y = get_marker_info(tag_corners)

    # Compute ROI center in pixel space
    roi_center = (center
                  + marker_x * (ROI_OFFSET_X_MM * px_per_mm)
                  + marker_y * (ROI_OFFSET_Y_MM * px_per_mm))

    half_w = (ROI_WIDTH_MM * px_per_mm) / 2
    half_h = (ROI_HEIGHT_MM * px_per_mm) / 2

    h, w = image.shape[:2]
    roi_x1 = max(0, int(roi_center[0] - half_w))
    roi_y1 = max(0, int(roi_center[1] - half_h))
    roi_x2 = min(w, int(roi_center[0] + half_w))
    roi_y2 = min(h, int(roi_center[1] + half_h))

    roi = image[roi_y1:roi_y2, roi_x1:roi_x2]
    roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    print(f"ROI: ({roi_x1},{roi_y1})-({roi_x2},{roi_y2}) = "
          f"{roi_x2-roi_x1}x{roi_y2-roi_y1}px")

    sample_contour = find_sample_contour(roi_gray)

    if sample_contour is None:
        print("ERROR: No sample found")
        return

    rect = cv2.minAreaRect(sample_contour)
    rect_center_full = (int(rect[0][0] + roi_x1), int(rect[0][1] + roi_y1))
    rect_corners_full = cv2.boxPoints(rect) + np.array([roi_x1, roi_y1])

    print(f"Sample: {rect[1][0]/px_per_mm:.1f}x{rect[1][1]/px_per_mm:.1f}mm, "
          f"angle={rect[2]:.1f}°")

    edge_inset_px = EDGE_INSET_MM * px_per_mm
    pickup = compute_pickup_point(
        rect_center_full, rect_corners_full, center, strategy, edge_inset_px
    )
    print(f"Pickup ({strategy}): ({pickup[0]}, {pickup[1]})")

    # Compute pickup offset from tag center in marker frame (mm)
    offset_px = np.array(pickup, dtype=float) - center
    offset_marker_x_mm = np.dot(offset_px, marker_x) / px_per_mm
    offset_marker_y_mm = np.dot(offset_px, marker_y) / px_per_mm
    print(f"\nFor vision_moveto:")
    print(f"  marker_offset_x: {offset_marker_x_mm / 1000:.4f} m ({offset_marker_x_mm:.1f} mm)")
    print(f"  marker_offset_y: {offset_marker_y_mm / 1000:.4f} m ({offset_marker_y_mm:.1f} mm)")

    # Save annotated image
    annotated = image.copy()
    cv2.rectangle(annotated, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 255, 255), 2)
    cv2.polylines(annotated, [tag_corners.astype(int)], True, (255, 0, 0), 2)
    cv2.circle(annotated, tuple(center.astype(int)), 5, (255, 0, 0), -1)
    cv2.drawContours(annotated, [rect_corners_full.astype(int)], 0, (0, 255, 0), 2)
    # Mark all corners small
    for c in rect_corners_full:
        cv2.circle(annotated, tuple(c.astype(int)), 3, (0, 255, 0), -1)
    # Mark center as small cyan dot
    cv2.circle(annotated, rect_center_full, 5, (255, 255, 0), -1)
    # Pickup point as red
    cv2.circle(annotated, pickup, 8, (0, 0, 255), -1)
    cv2.circle(annotated, pickup, 12, (0, 0, 255), 2)
    cv2.putText(annotated, strategy, (pickup[0] + 15, pickup[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.imwrite("/tmp/sample_detection_result.jpg", annotated)
    print(f"Saved: /tmp/sample_detection_result.jpg")


def compute_pickup_point(rect_center, rect_corners, tag_center_px, strategy="center",
                         edge_inset_px=0):
    """Compute pickup point based on strategy.

    Args:
        rect_center: (cx, cy) of fitted rectangle in full image pixels
        rect_corners: 4 corner points of fitted rectangle in full image pixels
        tag_center_px: (tx, ty) tag center in full image pixels
        strategy: "center", "nearest_edge", "farthest_edge",
                  "nearest_corner", "farthest_corner"
        edge_inset_px: pixels to move inward from edge toward center
    """
    if strategy == "center":
        return rect_center

    tag_pt = np.array(tag_center_px)
    center_pt = np.array(rect_center, dtype=float)

    if "corner" in strategy:
        distances = [np.linalg.norm(np.array(c) - tag_pt) for c in rect_corners]
        idx = np.argmax(distances) if "farthest" in strategy else np.argmin(distances)
        pt = rect_corners[idx].astype(float)
        if edge_inset_px > 0:
            # Move toward center
            toward_center = center_pt - pt
            norm = np.linalg.norm(toward_center)
            if norm > 0:
                pt = pt + toward_center / norm * edge_inset_px
        return tuple(pt.astype(int))

    if "edge" in strategy:
        edge_midpoints = []
        for i in range(4):
            p1 = rect_corners[i]
            p2 = rect_corners[(i + 1) % 4]
            edge_midpoints.append((p1 + p2) / 2)
        distances = [np.linalg.norm(m - tag_pt) for m in edge_midpoints]
        idx = np.argmax(distances) if "farthest" in strategy else np.argmin(distances)
        pt = edge_midpoints[idx].astype(float)
        if edge_inset_px > 0:
            # Move toward center
            toward_center = center_pt - pt
            norm = np.linalg.norm(toward_center)
            if norm > 0:
                pt = pt + toward_center / norm * edge_inset_px
        return tuple(pt.astype(int))

    return rect_center


# ---- SHARED ----

def find_sample_contour(roi_gray):
    """Find sample contour in ROI."""
    blurred = cv2.GaussianBlur(roi_gray, (BLUR_KERNEL, BLUR_KERNEL), 0)
    edges = cv2.Canny(blurred, CANNY_LOW, CANNY_HIGH)
    kernel = np.ones((3, 3), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=1)
    cv2.imwrite("/tmp/sample_edges.jpg", edges)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    areas = sorted([cv2.contourArea(c) for c in contours], reverse=True)
    print(f"  Contour areas: {[f'{a:.0f}' for a in areas[:10]]}")

    valid = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < SAMPLE_MIN_AREA or area > SAMPLE_MAX_AREA:
            continue
        rect = cv2.minAreaRect(c)
        w, h = rect[1]
        if w == 0 or h == 0:
            continue
        if max(w, h) / min(w, h) > MAX_ASPECT_RATIO:
            continue
        valid.append((area, c))

    if not valid:
        return None

    valid.sort(key=lambda x: x[0], reverse=True)
    return valid[0][1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", default="/tmp/sample_test_live.jpg")
    parser.add_argument("--tag", type=int, default=0)
    parser.add_argument("--mode", default="draw", choices=["draw", "detect"])
    parser.add_argument("--strategy", default="center")
    args = parser.parse_args()

    image = cv2.imread(args.image)
    if image is None:
        print(f"ERROR: Could not load: {args.image}")
        sys.exit(1)
    print(f"Image: {image.shape[1]}x{image.shape[0]}")

    tag_corners, all_ids = detect_aruco(image, args.tag)
    print(f"Tags: {sorted(all_ids)}")

    if tag_corners is None:
        print(f"ERROR: Tag {args.tag} not found")
        sys.exit(1)

    if args.mode == "draw":
        draw_roi_mode(image, tag_corners, args.tag)
    else:
        detect_mode(image, tag_corners, args.tag, args.strategy)


if __name__ == "__main__":
    main()
