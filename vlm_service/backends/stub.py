"""Deterministic stub backend.

Returns the image center with a fixed confidence. Used for:
- Wire-format tests (CI, FastAPI TestClient).
- Smoke tests on machines without a GPU.
- Default backend when the server starts with no --model flag.

Hashes (image_size, prompt) so the result is deterministic but varies
slightly per call — useful for catching cache bugs in clients.
"""
from __future__ import annotations

import hashlib

from PIL import Image

from .base import PointResult, VLMBackend


class StubBackend(VLMBackend):
    name = "stub"

    def load(self) -> None:
        return None

    def point(self, image: Image.Image, prompt: str) -> PointResult:
        w, h = image.size
        # Deterministic offset based on prompt so tests can assert exact coords.
        digest = hashlib.sha256(prompt.encode("utf-8")).digest()
        dx = (digest[0] / 255.0 - 0.5) * 0.2  # +/- 10% of width
        dy = (digest[1] / 255.0 - 0.5) * 0.2
        x = w * (0.5 + dx)
        y = h * (0.5 + dy)
        return PointResult(
            x=float(x),
            y=float(y),
            confidence=0.99,
            raw={"backend": "stub", "image_size": [w, h], "prompt": prompt},
        )

    def info(self) -> dict:
        return {
            "name": self.name,
            "model_id": None,
            "device": "cpu",
            "deterministic": True,
            "requires_verification": False,
        }
