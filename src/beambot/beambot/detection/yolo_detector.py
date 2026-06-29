"""YOLO-based object detection using Ultralytics.

Provides class-aware object detection that is more robust than traditional CV
methods (contour, circle, HSV) for detecting arbitrary objects with varying
contrast, lighting, and backgrounds.

Usage:
    detector = YoloDetector()  # loads default model (yolov8n)
    detections = detector.detect(image, confidence=0.25)
    # Returns: [(class_name, confidence, center_x, center_y, x1, y1, x2, y2), ...]
"""

import logging
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# Default model — YOLOv8 nano for speed. Can be overridden with custom weights.
DEFAULT_MODEL = "yolov8n.pt"

# Where to store downloaded/custom models
MODEL_DIR = Path.home() / ".beambot" / "models"


@dataclass
class YoloDetectionParams:
    """Parameters for YOLO object detection."""
    model_path: str = DEFAULT_MODEL   # Model weights file or Ultralytics model name
    confidence: float = 0.25          # Min confidence threshold (0-1)
    iou_threshold: float = 0.45       # NMS IoU threshold
    classes: list[int] | None = None  # Filter to specific class IDs (None = all)
    max_detections: int = 50          # Max number of detections to return
    device: str = ""                  # "" = auto (CUDA if available), "cpu", "cuda:0"


# Type alias for a single detection result
# (class_name, confidence, center_x, center_y, x1, y1, x2, y2)
YoloDetection = tuple[str, float, int, int, int, int, int, int]


class YoloDetector:
    """YOLO object detector using Ultralytics.

    Loads the model once on first use and caches it for subsequent calls.
    Supports both pre-trained COCO models and custom fine-tuned weights.
    """

    def __init__(self, model_path: str = DEFAULT_MODEL, device: str = ""):
        self._model = None
        self._model_path = model_path
        self._device = device

    def _ensure_model(self):
        """Lazy-load the YOLO model on first detection call."""
        if self._model is not None:
            return

        try:
            from ultralytics import YOLO
        except ImportError:
            raise RuntimeError(
                "ultralytics not installed. Run: pip install ultralytics"
            )

        # Check if it's a path to custom weights or a standard model name
        model_path = Path(self._model_path)
        if not model_path.exists() and not self._model_path.startswith("yolo"):
            # Check in the models directory
            alt_path = MODEL_DIR / self._model_path
            if alt_path.exists():
                model_path = alt_path

        logger.info(f"Loading YOLO model: {model_path}")
        self._model = YOLO(str(model_path))

        # Move to device
        if self._device:
            self._model.to(self._device)

        logger.info(
            f"YOLO model loaded: {self._model.model_name} "
            f"({len(self._model.names)} classes)"
        )

    def detect(
        self,
        image: np.ndarray,
        params: YoloDetectionParams | None = None,
    ) -> list[YoloDetection]:
        """Run YOLO detection on an image.

        Args:
            image: BGR or RGB image (numpy array).
            params: Detection parameters. Uses defaults if None.

        Returns:
            List of (class_name, confidence, cx, cy, x1, y1, x2, y2) tuples,
            sorted by confidence (highest first).
        """
        if params is None:
            params = YoloDetectionParams()

        # Load model if different from current
        if self._model_path != params.model_path:
            self._model = None
            self._model_path = params.model_path
        self._ensure_model()

        # Run inference
        results = self._model(
            image,
            conf=params.confidence,
            iou=params.iou_threshold,
            classes=params.classes,
            max_det=params.max_detections,
            verbose=False,
        )

        detections = []
        for result in results:
            boxes = result.boxes
            if boxes is None or len(boxes) == 0:
                continue

            for box in boxes:
                # Bounding box coordinates (cast to Python int for JSON serialization)
                coords = box.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = int(coords[0]), int(coords[1]), int(coords[2]), int(coords[3])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                # Class and confidence
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = self._model.names[cls_id]

                detections.append((cls_name, conf, cx, cy, x1, y1, x2, y2))

        # Sort by confidence (highest first)
        detections.sort(key=lambda d: d[1], reverse=True)

        return detections

    def annotate(
        self,
        image: np.ndarray,
        detections: list[YoloDetection],
    ) -> np.ndarray:
        """Draw YOLO detection results on an image.

        Args:
            image: BGR image to annotate.
            detections: List of detection tuples from detect().

        Returns:
            Annotated image copy.
        """
        annotated = image.copy()

        for i, (cls_name, conf, cx, cy, x1, y1, x2, y2) in enumerate(detections):
            color = (0, 255, 0)  # Green

            # Bounding box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

            # Label with class name and confidence
            label = f"#{i+1} {cls_name} {conf:.2f}"
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(
                annotated,
                (x1, y1 - label_size[1] - 8),
                (x1 + label_size[0] + 4, y1),
                color, -1,
            )
            cv2.putText(
                annotated, label,
                (x1 + 2, y1 - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2,
            )

            # Center point
            cv2.circle(annotated, (cx, cy), 4, (0, 0, 255), -1)

        return annotated

    @property
    def class_names(self) -> dict:
        """Get the model's class name mapping."""
        self._ensure_model()
        return self._model.names


# Module-level singleton for reuse across calls
_detector: YoloDetector | None = None


def get_detector(model_path: str = DEFAULT_MODEL) -> YoloDetector:
    """Get or create a YOLO detector singleton.

    Reuses the same detector if the model path matches.
    """
    global _detector
    if _detector is None or _detector._model_path != model_path:
        _detector = YoloDetector(model_path=model_path)
    return _detector
