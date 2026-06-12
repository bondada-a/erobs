"""NVIDIA Cosmos 3 Reasoner backend — thin OpenAI-compatible HTTP client.

⚠ requires_verification: true ⚠

Unlike every other backend in this package (which loads a HuggingFace model
in-process via transformers), Cosmos 3 Reasoner (`nvidia/Cosmos3-Nano`)
*cannot* be loaded via transformers today — NVIDIA's docs list that path as
"Coming soon". It runs only as its own OpenAI-compatible HTTP server:

  - hosted NVIDIA API  (integrate.api.nvidia.com/v1, API-key auth) — easiest
    first call, no GPU deploy
  - self-hosted NIM container  (nvcr.io/nim/nvidia/cosmos3-reasoner -> :8000)
  - self-hosted vLLM  (vllm-cosmos3 -> :8000)

So this backend is NOT an in-process model — it POSTs an image + a 2D-grounding
prompt to a chat-completions endpoint and parses the returned bounding boxes,
returning the center of the first box as the pixel coordinate.

Config via env vars on the service host (mirrors the XXX_MODEL_ID pattern of
the other backends):

    COSMOS_BASE_URL   default http://localhost:8000/v1
    COSMOS_API_KEY    default "" (no auth; set for the hosted API)
    COSMOS_MODEL      default nvidia/cosmos3-nano-reasoner

UNVERIFIED until the first real run (hence requires_verification=True):
  - exact grounding JSON schema (assumed Qwen-style [{"bbox_2d":[...], ...}])
  - coordinate space (original pixels vs. resized/normalized/0..1000 grid).
    See `_parse_cosmos_boxes` / `_maybe_rescale` and inspect `raw` to calibrate.

The Reasoner uses Qwen3-VL-compatible message conventions and emits a
`<think>...</think>` reasoning block before its answer. The full reasoning is
kept in `raw["text"]` — that is the action chain-of-thought we'll grow into for
later (non-pointing) Cosmos features.
"""
from __future__ import annotations

import base64
import io
import json
import logging
import os
import re
from typing import List, Optional

from PIL import Image

from .base import PointResult, VLMBackend

log = logging.getLogger(__name__)

COSMOS_BASE_URL = os.environ.get("COSMOS_BASE_URL", "http://localhost:8000/v1")
COSMOS_API_KEY = os.environ.get("COSMOS_API_KEY", "")
COSMOS_MODEL = os.environ.get("COSMOS_MODEL", "nvidia/cosmos3-nano-reasoner")

# Appended to the caller's prompt to bias the model toward a parseable,
# absolute-pixel grounding response.
_GROUNDING_SUFFIX = (
    "\nLocate the target and respond with ONLY a JSON list of objects, each "
    'like {"bbox_2d": [x1, y1, x2, y2], "label": "..."}, using absolute pixel '
    "coordinates of this image."
)

_THINK_RE = re.compile(r"<think>.*?</think>", re.DOTALL | re.IGNORECASE)
_FENCE_RE = re.compile(r"```(?:json)?\s*(.*?)```", re.DOTALL)
_BBOX4_RE = re.compile(
    r"\[\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\]"
)
_PAIR_RE = re.compile(r"\[\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\]")


def _strip_reasoning(text: str) -> str:
    """Drop <think>...</think> blocks and unwrap a markdown code fence.

    Cosmos wraps reasoning in <think>...</think> before the answer; the answer
    itself is sometimes inside a ```json ... ``` fence. We take the last fenced
    block (the answer) when present.
    """
    t = _THINK_RE.sub("", text)
    fences = _FENCE_RE.findall(t)
    return fences[-1] if fences else t


def _maybe_rescale(v: float, dim: int, coords: List[float]) -> float:
    """Conservatively map a coordinate into absolute pixels.

    Only acts on *unambiguous* signals, otherwise passes the value through as
    absolute pixels (the documented intent of the grounding prompt). This is
    deliberately cautious — see the coordinate-space risk note in the module
    docstring; we'd rather log raw coords and calibrate than mis-rescale.

        all coords <= 1.5            -> normalized [0, 1]      -> v * dim
        all coords <= 1000, dim>1000 -> Qwen 0..1000 grid      -> v/1000 * dim
        otherwise                    -> assume absolute pixels  -> v
    """
    mx = max((abs(c) for c in coords), default=0.0)
    if mx <= 1.5:
        return v * dim
    if mx <= 1000 and dim > 1000:
        return v / 1000.0 * dim
    return v


def _parse_cosmos_boxes(
    text: str, width: int, height: int
) -> Optional[PointResult]:
    """Parse Cosmos grounding output into the center of the first 2D box.

    Handles, in order:
        [{"bbox_2d": [x1,y1,x2,y2], "label": "..."}, ...]   — Qwen-style JSON
        [x1, y1, x2, y2]                                     — bare box
        [x, y]                                               — single point
    plus <think> reasoning blocks and markdown ```json fences. Returns None if
    nothing parseable is found (the backend then raises ValueError).
    """
    answer = _strip_reasoning(text)
    boxes: List[List[float]] = []
    labels: List[Optional[str]] = []
    fmt = "bbox_2d_json"

    # 1. Qwen-style JSON list of dicts.
    m = re.search(r"\[.*\]", answer, re.DOTALL)
    if m:
        try:
            data = json.loads(m.group(0))
        except (json.JSONDecodeError, ValueError):
            data = None
        if isinstance(data, list) and data and isinstance(data[0], dict):
            for d in data:
                bb = d.get("bbox_2d") or d.get("bbox")
                if isinstance(bb, list) and len(bb) >= 4:
                    boxes.append([float(c) for c in bb[:4]])
                    labels.append(d.get("label"))

    # 2. Bare [x1, y1, x2, y2].
    if not boxes:
        b4 = _BBOX4_RE.search(answer)
        if b4:
            boxes.append([float(b4.group(i)) for i in range(1, 5)])
            fmt = "bare_bbox4"

    # 3. Single point [x, y] -> zero-area box.
    if not boxes:
        p = _PAIR_RE.search(answer)
        if p:
            x, y = float(p.group(1)), float(p.group(2))
            boxes.append([x, y, x, y])
            fmt = "point_xy"

    if not boxes:
        return None

    x1, y1, x2, y2 = boxes[0]
    cx_raw = (x1 + x2) / 2.0
    cy_raw = (y1 + y2) / 2.0
    cx = _maybe_rescale(cx_raw, width, [x1, x2, y1, y2])
    cy = _maybe_rescale(cy_raw, height, [x1, x2, y1, y2])
    cx = max(0.0, min(float(width), cx))
    cy = max(0.0, min(float(height), cy))

    return PointResult(
        x=cx,
        y=cy,
        confidence=1.0,  # Cosmos grounding doesn't emit a confidence score.
        raw={
            "text": text,  # full output incl. <think> — action CoT for later use
            "answer": answer,  # reasoning-stripped answer block
            "reasoning_present": "<think>" in text.lower(),
            "all_boxes_raw": boxes,  # as emitted, pre-rescale
            "labels": labels,
            "selected_index": 0,
            "selected_box_center_raw": [cx_raw, cy_raw],
            "coord_format": fmt,
            "image_size": [width, height],
            "rescaled": (cx != cx_raw or cy != cy_raw),
            "model": COSMOS_MODEL,
        },
    )


class CosmosBackend(VLMBackend):
    """OpenAI-compatible HTTP client for the NVIDIA Cosmos 3 Reasoner."""

    name = "cosmos"
    requires_verification = True

    def __init__(
        self,
        base_url: str = COSMOS_BASE_URL,
        api_key: str = COSMOS_API_KEY,
        model: str = COSMOS_MODEL,
    ):
        self.base_url = base_url.rstrip("/")
        self.api_key = api_key
        self.model = model
        self._client = None

    def _headers(self) -> dict:
        h = {"Content-Type": "application/json"}
        if self.api_key:
            h["Authorization"] = f"Bearer {self.api_key}"
        return h

    def load(self) -> None:
        """Create the HTTP client and do a best-effort reachability probe.

        Unlike weight-loading backends, this never hard-fails at startup if the
        Cosmos endpoint is down — it logs a warning and retries on first
        /point. This keeps `--backend cosmos` startup behavior lazy/forgiving.
        """
        import httpx  # already a hard dep (requirements.txt)

        self._client = httpx.Client(timeout=120.0)
        log.warning(
            "Cosmos backend is an OpenAI-compatible HTTP CLIENT (model=%s @ %s); "
            "no weights loaded in-process. requires_verification=True.",
            self.model,
            self.base_url,
        )
        try:
            r = self._client.get(
                f"{self.base_url}/models", headers=self._headers()
            )
            log.info("Cosmos endpoint reachable: HTTP %s", r.status_code)
        except Exception as e:  # noqa: BLE001 - probe is best-effort
            log.warning(
                "Cosmos endpoint not reachable at load() (%s). "
                "Will retry on first /point.",
                e,
            )

    def point(self, image: Image.Image, prompt: str) -> PointResult:
        if self._client is None:
            raise RuntimeError("Backend not loaded. Call load() first.")

        width, height = image.size
        buf = io.BytesIO()
        image.save(buf, format="PNG")
        data_url = "data:image/png;base64," + base64.b64encode(
            buf.getvalue()
        ).decode("ascii")

        body = {
            "model": self.model,
            "temperature": 0,
            "max_tokens": 1024,  # room for <think> reasoning + answer
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "image_url", "image_url": {"url": data_url}},
                        {"type": "text", "text": prompt.strip() + _GROUNDING_SUFFIX},
                    ],
                }
            ],
        }

        try:
            r = self._client.post(
                f"{self.base_url}/chat/completions",
                headers=self._headers(),
                json=body,
            )
            r.raise_for_status()
        except Exception as e:  # noqa: BLE001 - mapped to HTTP 500 by server
            raise RuntimeError(f"Cosmos request failed: {e}") from e

        text = r.json()["choices"][0]["message"]["content"]
        result = _parse_cosmos_boxes(text, width, height)
        if result is None:
            raise ValueError(
                f"Cosmos returned no parseable box. Output: {text!r}. "
                "If the grounding schema differs, update _parse_cosmos_boxes."
            )
        return result

    def info(self) -> dict:
        return {
            "name": self.name,
            "model": self.model,
            "base_url": self.base_url,
            "transport": "openai-compatible-http",
            "loaded": self._client is not None,
            "requires_verification": True,
            "auth": "bearer" if self.api_key else "none",
            "notes": (
                "Thin OpenAI client to Cosmos 3 Reasoner. Returns the center of "
                "the first 2D box. Coordinate space and box schema UNVERIFIED — "
                "inspect raw.all_boxes_raw / raw.coord_format / raw.rescaled on "
                "first runs and calibrate."
            ),
        }
