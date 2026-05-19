"""MolmoAct-7B-D backend.

Model card: https://huggingface.co/allenai/MolmoAct-7B-D-0812
(MolmoAct = action-tuned variant of Molmo. Pointing API matches Molmo.)

Molmo pointing convention: the model emits XML tags like
    <point x="50.4" y="62.1" alt="sample">sample</point>
where x/y are **percentages 0-100** of the image (NOT 0-1, NOT pixels).
We convert to absolute pixels here.

If the model emits multiple points (`<points x1="..." y1="..." x2="..." />`),
we take the first.
"""
from __future__ import annotations

import logging
import os
import re
from typing import Optional

from PIL import Image

from .base import PointResult, VLMBackend

log = logging.getLogger(__name__)

DEFAULT_MODEL_ID = os.environ.get("MOLMOACT_MODEL_ID", "allenai/MolmoAct-7B-D-0812")

# Matches both <point x=".." y=".." /> and <points x1=".." y1=".." ... />
_POINT_RE = re.compile(
    r'<point[s]?\s+([^>]*?)/?>',
    re.IGNORECASE,
)
_XY_RE = re.compile(r'(x\d*)="([\d.]+)"\s+(y\d*)="([\d.]+)"', re.IGNORECASE)


def _parse_molmo_point(text: str, width: int, height: int) -> Optional[PointResult]:
    """Parse the first <point .../> tag from Molmo output.

    Returns absolute pixel coordinates, or None if no point found.
    """
    m = _POINT_RE.search(text)
    if not m:
        return None
    attrs = m.group(1)
    pairs = _XY_RE.findall(attrs)
    if not pairs:
        return None
    # Take the first (x, y) pair.
    _, xs, _, ys = pairs[0]
    # Molmo emits percentages 0-100.
    x_pct = float(xs)
    y_pct = float(ys)
    x_px = (x_pct / 100.0) * width
    y_px = (y_pct / 100.0) * height
    return PointResult(
        x=x_px,
        y=y_px,
        confidence=1.0,  # Molmo doesn't emit a confidence score.
        raw={"text": text, "x_pct": x_pct, "y_pct": y_pct},
    )


class MolmoActBackend(VLMBackend):
    name = "molmoact"

    def __init__(self, model_id: str = DEFAULT_MODEL_ID):
        self.model_id = model_id
        self.processor = None
        self.model = None
        self.device = "cuda"

    def load(self) -> None:
        import torch
        from transformers import AutoModelForImageTextToText, AutoProcessor, BitsAndBytesConfig

        load_in_8bit = os.environ.get("LOAD_IN_8BIT", "0") == "1"
        load_in_4bit = os.environ.get("LOAD_IN_4BIT", "0") == "1"
        log.info("Loading MolmoAct model %s (8bit=%s, 4bit=%s) ...",
                 self.model_id, load_in_8bit, load_in_4bit)

        self.processor = AutoProcessor.from_pretrained(
            self.model_id,
            trust_remote_code=True,
        )

        from transformers import AutoConfig
        config = AutoConfig.from_pretrained(self.model_id, trust_remote_code=True)
        config._attn_implementation = "sdpa"
        for sub_name in ("vit_config", "adapter_config", "llm_config"):
            sub = getattr(config, sub_name, None)
            if sub is not None:
                sub._attn_implementation = "sdpa"

        model_kwargs = {
            "trust_remote_code": True,
            "device_map": "auto",
            "config": config,
        }
        if load_in_4bit:
            torch.cuda.empty_cache()
            model_kwargs["quantization_config"] = BitsAndBytesConfig(
                load_in_4bit=True,
                bnb_4bit_compute_dtype=torch.bfloat16,
                bnb_4bit_quant_type="nf4",
                bnb_4bit_use_double_quant=True,
            )
        elif load_in_8bit:
            model_kwargs["quantization_config"] = BitsAndBytesConfig(
                load_in_8bit=True,
                llm_int8_enable_fp32_cpu_offload=True,
            )
            model_kwargs["max_memory"] = {0: "7GiB", "cpu": "24GiB"}
        else:
            model_kwargs["torch_dtype"] = torch.bfloat16

        self.model = AutoModelForImageTextToText.from_pretrained(
            self.model_id,
            **model_kwargs,
        )
        log.info("MolmoAct loaded.")

    def point(self, image: Image.Image, prompt: str) -> PointResult:
        if self.model is None or self.processor is None:
            raise RuntimeError("Backend not loaded. Call load() first.")

        inputs = self.processor(
            text=prompt,
            images=image,
            return_tensors="pt",
        ).to(self.model.device)

        output = self.model.generate(
            **inputs,
            max_new_tokens=200,
            do_sample=False,
        )
        generated = output[0, inputs["input_ids"].size(1):]
        text = self.processor.tokenizer.decode(generated, skip_special_tokens=True)

        result = _parse_molmo_point(text, image.size[0], image.size[1])
        if result is None:
            raise ValueError(f"MolmoAct returned no point. Output: {text!r}")
        return result

    def info(self) -> dict:
        return {
            "name": self.name,
            "model_id": self.model_id,
            "device": self.device,
            "loaded": self.model is not None,
            "point_format": "percentage (0-100), converted to pixels",
            "requires_verification": False,
        }
