"""Spincoater pocket and sample detection for orientation-aware placement.

Two detectors:

1. detect_spincoater_pocket() — detects the empty pocket in the red-painted
   chuck using classical CV (HSV color masking + bright-metal isolation).
   Used BEFORE placing a sample to determine the pocket's orientation.

2. detect_spincoater_sample() — detects a sample wafer sitting on the chuck
   using a YOLO segmentation model. Used AFTER spincoating to find the
   sample's actual position and orientation for re-pickup.

Both require 2D flash-lit capture (/capture_2d) — NOT the 3D projector capture.
Chuck must be centered in the camera frame (flash falloff off-axis).
"""

import logging
import threading
from pathlib import Path

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# YOLO model for sample detection (lazy-loaded)
# Resolve from repo root (non-.py files aren't copied to install/)
def _find_model_path() -> Path:
    """Find the model file by searching from the git repo root."""
    import subprocess
    try:
        repo_root = subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"],
            stderr=subprocess.DEVNULL, text=True
        ).strip()
        return Path(repo_root) / "src" / "beambot" / "models" / "spincoater_sample_seg.pt"
    except (subprocess.CalledProcessError, FileNotFoundError):
        return Path(__file__).parent.parent.parent / "models" / "spincoater_sample_seg.pt"

_SAMPLE_MODEL_PATH = _find_model_path()
_sample_model = None
_sample_model_lock = threading.Lock()


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


def _get_sample_model():
    """Lazy-load the YOLO segmentation model for sample detection.

    Thread-safe: a background warmup thread and a task thread may both call
    this; the lock ensures the model loads exactly once.
    """
    global _sample_model
    if _sample_model is not None:
        return _sample_model

    with _sample_model_lock:
        # Re-check inside the lock (another thread may have loaded it)
        if _sample_model is not None:
            return _sample_model

        try:
            from ultralytics import YOLO
        except ImportError:
            raise RuntimeError(
                "ultralytics not installed. Run: pip install ultralytics"
            )

        if not _SAMPLE_MODEL_PATH.exists():
            raise FileNotFoundError(
                f"Spincoater sample model not found at {_SAMPLE_MODEL_PATH}. "
                "Train with: yolo segment train data=data.yaml model=yolov8n-seg.pt"
            )

        logger.info(f"Loading spincoater sample model: {_SAMPLE_MODEL_PATH}")
        _sample_model = YOLO(str(_SAMPLE_MODEL_PATH))
        return _sample_model


def detect_spincoater_sample(
    image: np.ndarray,
    confidence: float = 0.3,
) -> dict | None:
    """Detect a sample wafer on the spincoater chuck using YOLO segmentation.

    Uses a fine-tuned YOLOv8-seg model to detect the sample. Returns the
    centroid and orientation derived from the segmentation mask's minAreaRect.

    This is used for re-pickup after spincoating: the chuck stops at a random
    angle, the sample may have shifted, and we need its actual position and
    orientation.

    Args:
        image: BGR image from a 2D flash-lit capture (/capture_2d).
        confidence: Minimum detection confidence (0-1).

    Returns:
        Dict with keys:
          - center_px: (x, y) centroid of the segmentation mask
          - angle_mod90: sample rotation in degrees [0, 90), mod 90 for
            4-fold symmetry, measured from image horizontal
          - angle_raw: raw minAreaRect angle (for cases where full 180° needed)
          - width: fitted rectangle width in pixels
          - height: fitted rectangle height in pixels
          - aspect: aspect ratio (>= 1.0)
          - confidence: detection confidence score
          - mask_points: number of polygon points in the segmentation mask
        Returns None if no sample detected.
    """
    model = _get_sample_model()
    results = model(image, conf=confidence, verbose=False)

    for r in results:
        if r.masks is None or len(r.masks) == 0:
            continue

        # Take the highest-confidence detection
        best_idx = r.boxes.conf.argmax()
        mask_xy = r.masks[best_idx].xy[0]
        conf = r.boxes[best_idx].conf[0].item()

        if len(mask_xy) < 4:
            continue

        pts = mask_xy.astype(np.float32)
        (cx, cy), (rw, rh), ang = cv2.minAreaRect(pts)
        aspect = max(rw, rh) / (min(rw, rh) + 1e-6)

        # Centroid from moments (more accurate than rect center)
        M = cv2.moments(pts.astype(np.int32))
        if M["m00"] > 0:
            centroid_x = M["m10"] / M["m00"]
            centroid_y = M["m01"] / M["m00"]
        else:
            centroid_x, centroid_y = cx, cy

        return {
            "center_px": (int(round(centroid_x)), int(round(centroid_y))),
            "angle_mod90": ang % 90,
            "angle_raw": ang,
            "width": rw,
            "height": rh,
            "aspect": round(aspect, 3),
            "confidence": round(conf, 3),
            "mask_points": len(mask_xy),
        }

    return None
