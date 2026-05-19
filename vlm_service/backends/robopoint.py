"""RoboPoint backend.

RoboPoint (Yuan et al., UW + NVIDIA, 2024) — a LLaVA-1.5 model fine-tuned
specifically for *robotic* spatial affordance pointing. Trained on synthetic
robot scenes, it tends to outperform general-purpose VLMs on the
"point to the affordance / target on the workspace" task that EROBS
actually does.

Paper: https://arxiv.org/abs/2406.10721
HF model id: `wentao-yuan/robopoint-v1-vicuna-v1.5-13b` (confirmed default)
A smaller 7B variant exists: `wentao-yuan/robopoint-v1-vicuna-v1.5-7b`.

Output format
-------------
RoboPoint emits a Python-style list of **normalized** (x, y) tuples in [0, 1],
e.g. ``[(0.534, 0.412), (0.601, 0.388)]``. We take the first point and convert
to absolute pixels.

The official prompt template wraps the user instruction:

    "Your task is to identify several spots within the vacant area...
     Your answer should be formatted as a list of tuples, i.e.
     [(x1, y1), (x2, y2), ...], where each tuple contains the
     x and y coordinates of a point... within the range [0, 1]."

If the caller's prompt already contains "[" we pass it through unchanged
(advanced users); otherwise we wrap it with the canonical template so the
model emits the expected format.
"""
from __future__ import annotations

import ast
import logging
import os
import re
from typing import List, Optional, Tuple

from PIL import Image

from .base import PointResult, VLMBackend

log = logging.getLogger(__name__)

DEFAULT_MODEL_ID = os.environ.get(
    "ROBOPOINT_MODEL_ID",
    "wentao-yuan/robopoint-v1-vicuna-v1.5-13b",
)

# Canonical RoboPoint prompt template (from the paper / repo README).
_PROMPT_TEMPLATE = (
    "Your task is to identify points on the image that satisfy the "
    "following instruction: {instruction}\n"
    "Your answer should be formatted as a list of tuples, i.e. "
    "[(x1, y1), (x2, y2), ...], where each tuple contains the x and y "
    "coordinates of a point of interest. The coordinates should be "
    "between 0 and 1, indicating the normalized pixel locations of the "
    "points in the image."
)

# Match the first list-of-tuples in the model output.
_LIST_RE = re.compile(r"\[\s*\(.*?\)\s*(?:,\s*\(.*?\)\s*)*\]", re.DOTALL)


def _parse_robopoint_points(
    text: str, width: int, height: int
) -> Optional[PointResult]:
    """Extract the first (x, y) tuple from RoboPoint output and convert to pixels."""
    m = _LIST_RE.search(text)
    if not m:
        return None
    try:
        points: List[Tuple[float, float]] = ast.literal_eval(m.group(0))
    except (SyntaxError, ValueError):
        return None
    if not points:
        return None
    x_norm, y_norm = points[0]
    # RoboPoint emits normalized [0, 1]. Defensive clamp.
    x_norm = max(0.0, min(1.0, float(x_norm)))
    y_norm = max(0.0, min(1.0, float(y_norm)))
    return PointResult(
        x=x_norm * width,
        y=y_norm * height,
        confidence=1.0,  # RoboPoint doesn't emit a confidence score.
        raw={
            "text": text,
            "all_points_norm": points,
            "selected_index": 0,
        },
    )


class RoboPointBackend(VLMBackend):
    """LLaVA-1.5 fine-tuned for robotic spatial affordance pointing."""

    name = "robopoint"

    def __init__(self, model_id: str = DEFAULT_MODEL_ID):
        self.model_id = model_id
        self.processor = None
        self.model = None
        self.tokenizer = None
        self.device = "cuda"

    def load(self) -> None:
        """Load RoboPoint via the LLaVA loader.

        RoboPoint is published as a LLaVA-1.5 checkpoint and uses LLaVA's
        custom loading path. Two options:

        1. Install the upstream `llava` package (from haotian-liu/LLaVA) and
           use ``load_pretrained_model`` — this is what the RoboPoint repo
           recommends.
        2. Use HuggingFace transformers' ``LlavaForConditionalGeneration``
           directly. This works for the merged-weights checkpoints on the
           Hub but skips a few LLaVA-specific helpers.

        We try option 2 first (no extra deps) and fall back to a clear
        error message pointing at the LLaVA repo if the simple path fails.
        """
        import torch

        log.info("Loading RoboPoint model %s ...", self.model_id)
        try:
            from transformers import (
                AutoProcessor,
                LlavaForConditionalGeneration,
            )

            self.processor = AutoProcessor.from_pretrained(
                self.model_id, trust_remote_code=True
            )
            self.model = LlavaForConditionalGeneration.from_pretrained(
                self.model_id,
                torch_dtype=torch.float16,
                device_map="auto",
                trust_remote_code=True,
            )
            self.tokenizer = self.processor.tokenizer
        except Exception as e:
            raise RuntimeError(
                f"Failed to load RoboPoint via transformers ({e!r}). "
                "If transformers can't load this checkpoint directly, "
                "install the LLaVA package "
                "(`pip install git+https://github.com/haotian-liu/LLaVA.git`) "
                "and switch to its `load_pretrained_model` API. The "
                "RoboPoint repo (github.com/wentaoyuan/RoboPoint) has the "
                "exact loading snippet."
            ) from e
        log.info("RoboPoint loaded.")

    def point(self, image: Image.Image, prompt: str) -> PointResult:
        if self.model is None or self.processor is None:
            raise RuntimeError("Backend not loaded. Call load() first.")

        # Wrap with canonical template unless caller already supplied one.
        if "[" in prompt and "]" in prompt:
            full_prompt = prompt
        else:
            full_prompt = _PROMPT_TEMPLATE.format(instruction=prompt.strip())

        # LLaVA-1.5 chat template expects an <image> token.
        conversation = [
            {
                "role": "user",
                "content": [
                    {"type": "image"},
                    {"type": "text", "text": full_prompt},
                ],
            }
        ]
        text_input = self.processor.apply_chat_template(
            conversation, add_generation_prompt=True
        )
        inputs = self.processor(
            images=image, text=text_input, return_tensors="pt"
        ).to(self.model.device)

        output = self.model.generate(
            **inputs,
            max_new_tokens=256,
            do_sample=False,
        )
        # Strip the prompt prefix.
        generated = output[0, inputs["input_ids"].shape[1]:]
        text = self.tokenizer.decode(generated, skip_special_tokens=True)

        result = _parse_robopoint_points(text, image.size[0], image.size[1])
        if result is None:
            raise ValueError(
                f"RoboPoint returned no parseable points. Output: {text!r}"
            )
        return result

    def info(self) -> dict:
        return {
            "name": self.name,
            "model_id": self.model_id,
            "device": self.device,
            "loaded": self.model is not None,
            "point_format": "normalized [0,1] tuples, converted to pixels",
            "requires_verification": False,
            "notes": (
                "LLaVA-1.5 fine-tuned for robotic spatial affordance pointing "
                "(Yuan et al. 2024). Best fit for puck/sample pointing on "
                "structured workspaces."
            ),
        }
