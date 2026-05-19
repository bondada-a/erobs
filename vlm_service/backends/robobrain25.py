"""RoboBrain 2.5 backend (BAAI).

⚠ requires_verification: true ⚠

The exact HF id for RoboBrain 2.5 could not be confirmed from this env.
Best-known confirmed predecessor IDs:
    BAAI/RoboBrain2.0-7B
    BAAI/RoboBrain2.0-32B

We default to `BAAI/RoboBrain2.5-7B` (guessed). Override with env var
`ROBOBRAIN_MODEL_ID` if the actual id differs.

RoboBrain 2.x is built on Qwen2.5-VL. The pointing-mode convention (per
the FlagOpen/RoboBrain repo) is:

    Prompt: "Point to the <object>. Output the pixel coordinates."
    Output: a JSON / Python list like `[[x1, y1], [x2, y2], ...]`
            in **absolute pixel coordinates of the (preprocessed) image**.

We parse the first numeric pair we find. RoboBrain 2.5 also has native 3D
output (per the paper); for parity with the other backends we extract the
2D pixel coord and stash any extra structure in `raw`.
"""
from __future__ import annotations

import json
import logging
import os
import re
from typing import Optional

from PIL import Image

from .base import PointResult, VLMBackend

log = logging.getLogger(__name__)

DEFAULT_MODEL_ID = os.environ.get(
    "ROBOBRAIN_MODEL_ID",
    "BAAI/RoboBrain2.5-7B",  # UNVERIFIED guess; fallback: BAAI/RoboBrain2.0-7B
)

# Match either [[x, y], ...] or single [x, y] with int or float numbers.
_COORD_PAIR_RE = re.compile(r"\[\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\]")


def _parse_robobrain_point(text: str) -> Optional[PointResult]:
    """Extract the first 2D pixel coordinate from RoboBrain output."""
    # Try strict JSON first.
    try:
        # Find JSON-like substring.
        m = re.search(r"\[.*\]", text, re.DOTALL)
        if m:
            data = json.loads(m.group(0))
            if isinstance(data, list) and data:
                first = data[0]
                if isinstance(first, list) and len(first) >= 2:
                    return PointResult(
                        x=float(first[0]),
                        y=float(first[1]),
                        confidence=1.0,
                        raw={"text": text, "all_points": data},
                    )
                if isinstance(first, (int, float)) and len(data) >= 2:
                    return PointResult(
                        x=float(data[0]),
                        y=float(data[1]),
                        confidence=1.0,
                        raw={"text": text, "all_points": data},
                    )
    except (json.JSONDecodeError, ValueError):
        pass

    # Fallback: regex for first [x, y] pair.
    m = _COORD_PAIR_RE.search(text)
    if m:
        return PointResult(
            x=float(m.group(1)),
            y=float(m.group(2)),
            confidence=1.0,
            raw={"text": text},
        )
    return None


class RoboBrain25Backend(VLMBackend):
    name = "robobrain25"
    requires_verification = True

    def __init__(self, model_id: str = DEFAULT_MODEL_ID):
        self.model_id = model_id
        self.processor = None
        self.model = None

    def load(self) -> None:
        import torch
        from transformers import AutoModelForVision2Seq, AutoProcessor

        log.warning(
            "Loading RoboBrain (UNVERIFIED v2.5 id) from %s. If load fails, "
            "set ROBOBRAIN_MODEL_ID=BAAI/RoboBrain2.0-7B as a fallback.",
            self.model_id,
        )
        self.processor = AutoProcessor.from_pretrained(
            self.model_id, trust_remote_code=True
        )
        self.model = AutoModelForVision2Seq.from_pretrained(
            self.model_id,
            trust_remote_code=True,
            torch_dtype=torch.bfloat16,
            device_map="auto",
        )

    def point(self, image: Image.Image, prompt: str) -> PointResult:
        if self.model is None:
            raise RuntimeError("Backend not loaded.")

        # Encourage pointing-mode output.
        full_prompt = (
            f"{prompt}\n"
            "Output only the pixel coordinates as a JSON list [[x, y]]."
        )
        msgs = [{
            "role": "user",
            "content": [
                {"type": "image", "image": image},
                {"type": "text", "text": full_prompt},
            ],
        }]
        text = self.processor.apply_chat_template(
            msgs, add_generation_prompt=True, tokenize=False
        )
        inputs = self.processor(
            text=[text], images=[image], return_tensors="pt"
        ).to(self.model.device)

        gen_ids = self.model.generate(**inputs, max_new_tokens=128, do_sample=False)
        out = self.processor.batch_decode(
            gen_ids[:, inputs.input_ids.size(1):], skip_special_tokens=True
        )[0]

        result = _parse_robobrain_point(out)
        if result is None:
            raise ValueError(f"RoboBrain returned no point. Output: {out!r}")
        return result

    def info(self) -> dict:
        return {
            "name": self.name,
            "model_id": self.model_id,
            "loaded": self.model is not None,
            "requires_verification": True,
            "notes": (
                "RoboBrain 2.5 id unverified. Coordinates returned are "
                "absolute pixels of the processed image; caller may need "
                "to rescale to original image dims."
            ),
        }
