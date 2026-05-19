"""Backend registry — maps backend name to class.

Backends are imported lazily so the server can run with `stub` even if
heavy ML deps (transformers, torch) aren't installed.
"""
from __future__ import annotations

from typing import Dict, Type

from .base import VLMBackend


def _load_stub() -> Type[VLMBackend]:
    from .stub import StubBackend
    return StubBackend


def _load_molmoact() -> Type[VLMBackend]:
    from .molmoact import MolmoActBackend
    return MolmoActBackend


def _load_molmoact2() -> Type[VLMBackend]:
    from .molmoact2 import MolmoAct2Backend
    return MolmoAct2Backend


def _load_robobrain25() -> Type[VLMBackend]:
    from .robobrain25 import RoboBrain25Backend
    return RoboBrain25Backend


def _load_robopoint() -> Type[VLMBackend]:
    from .robopoint import RoboPointBackend
    return RoboPointBackend


REGISTRY: Dict[str, callable] = {
    "stub": _load_stub,
    "molmoact": _load_molmoact,
    "molmoact2": _load_molmoact2,
    "robobrain25": _load_robobrain25,
    "robopoint": _load_robopoint,
}


def get_backend(name: str) -> VLMBackend:
    if name not in REGISTRY:
        raise ValueError(
            f"Unknown backend '{name}'. Available: {sorted(REGISTRY.keys())}"
        )
    cls = REGISTRY[name]()
    return cls()


def list_backends() -> list[str]:
    return sorted(REGISTRY.keys())
