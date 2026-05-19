"""HTTP client for the standalone VLM pointing service.

The VLM service runs as a separate FastAPI process (typically on a remote
GPU box). This client wraps the wire format so callers in the ROS workspace
get a `detect(image_path, prompt) -> {pixel_x, pixel_y, confidence, raw}`
contract that mirrors the existing ArUco / YOLO detectors.

Why HTTP and not in-process? Loading a 7B-parameter VLM into the ROS
node would (a) bloat the orchestrator process by ~14 GB, (b) drag in
torch/transformers into the ROS Python env (a known pain point — see
the erobs-sim-loop skill on Hermes-venv shadowing), and (c) prevent
GPU sharing across multiple ROS hosts. A separate service process is
the clean separation.
"""
from __future__ import annotations

import base64
import logging
from pathlib import Path
from typing import Any, Dict, Optional

try:
    import httpx
except ImportError:  # pragma: no cover - httpx is a hard dep at runtime
    httpx = None  # type: ignore

log = logging.getLogger(__name__)


class VLMDetectorError(RuntimeError):
    """Raised when the VLM service fails or returns an unusable response."""


class VLMDetector:
    """Sync HTTP client for the VLM pointing service.

    Args:
        service_url: Base URL of the VLM service (no trailing slash).
        timeout_seconds: Request timeout (single image inference can take
            several seconds on smaller GPUs; 10s is a sensible default).
    """

    def __init__(
        self,
        service_url: str = "http://localhost:8765",
        timeout_seconds: float = 10.0,
    ):
        if httpx is None:
            raise ImportError(
                "httpx is required for VLMDetector. Install with: pip install httpx"
            )
        self.service_url = service_url.rstrip("/")
        self.timeout = timeout_seconds

    # --- Public API --------------------------------------------------------

    def health(self) -> Dict[str, Any]:
        """Return the VLM service health status."""
        try:
            r = httpx.get(f"{self.service_url}/health", timeout=self.timeout)
            r.raise_for_status()
            return r.json()
        except httpx.HTTPError as e:
            raise VLMDetectorError(f"VLM health check failed: {e}") from e

    def info(self) -> Dict[str, Any]:
        """Return backend metadata."""
        try:
            r = httpx.get(f"{self.service_url}/info", timeout=self.timeout)
            r.raise_for_status()
            return r.json()
        except httpx.HTTPError as e:
            raise VLMDetectorError(f"VLM info request failed: {e}") from e

    def detect(
        self,
        image_path: str,
        prompt: str,
        *,
        image_bytes: Optional[bytes] = None,
    ) -> Dict[str, Any]:
        """Detect a single point in an image.

        Args:
            image_path: Path to image file. Ignored if image_bytes is set.
            prompt: Pointing instruction, e.g. 'Point to the sample on the puck.'
            image_bytes: Optional pre-loaded image bytes (PNG/JPEG).

        Returns:
            Dict with:
              - pixel_x: float, absolute pixel X
              - pixel_y: float, absolute pixel Y
              - confidence: float
              - backend: str, name of active backend
              - raw: dict, backend-specific extra info

        Raises:
            VLMDetectorError on HTTP error, timeout, or malformed response.
            FileNotFoundError if image_path doesn't exist (and no image_bytes).
        """
        if image_bytes is None:
            p = Path(image_path)
            if not p.is_file():
                raise FileNotFoundError(f"Image not found: {image_path}")
            image_bytes = p.read_bytes()

        b64 = base64.b64encode(image_bytes).decode("ascii")
        payload = {"image_b64": b64, "prompt": prompt}

        try:
            r = httpx.post(
                f"{self.service_url}/point",
                json=payload,
                timeout=self.timeout,
            )
        except httpx.TimeoutException as e:
            raise VLMDetectorError(
                f"VLM service timed out after {self.timeout}s: {e}"
            ) from e
        except httpx.HTTPError as e:
            raise VLMDetectorError(f"VLM service request failed: {e}") from e

        if r.status_code != 200:
            raise VLMDetectorError(
                f"VLM service returned HTTP {r.status_code}: {r.text}"
            )

        try:
            data = r.json()
        except ValueError as e:
            raise VLMDetectorError(f"VLM response was not valid JSON: {e}") from e

        # Validate response shape.
        for key in ("x", "y", "confidence", "backend"):
            if key not in data:
                raise VLMDetectorError(
                    f"VLM response missing required field '{key}': {data!r}"
                )

        return {
            "pixel_x": float(data["x"]),
            "pixel_y": float(data["y"]),
            "confidence": float(data["confidence"]),
            "backend": data["backend"],
            "raw": data.get("raw", {}),
        }
