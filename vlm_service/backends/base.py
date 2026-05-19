"""Abstract base for VLM pointing backends.

A backend takes an image + prompt and returns a single 2D pixel coordinate
(plus optional confidence + raw model output for inspection).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict

from PIL import Image


@dataclass
class PointResult:
    """Result of a single point query."""

    x: float  # absolute pixel x
    y: float  # absolute pixel y
    confidence: float = 1.0
    raw: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "confidence": self.confidence,
            "raw": self.raw,
        }


class VLMBackend(ABC):
    """Abstract VLM pointing backend."""

    name: str = "base"
    requires_verification: bool = False

    @abstractmethod
    def load(self) -> None:
        """Load model weights into memory. Called once at server startup."""

    @abstractmethod
    def point(self, image: Image.Image, prompt: str) -> PointResult:
        """Return a single pixel coordinate that satisfies the prompt."""

    def info(self) -> Dict[str, Any]:
        """Return metadata about this backend (model id, dtype, device, etc.)."""
        return {
            "name": self.name,
            "requires_verification": self.requires_verification,
        }
