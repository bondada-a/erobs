"""MolmoAct2 backend.

⚠ requires_verification: true ⚠

As of branch creation (May 2026), MolmoAct2 was announced but its exact
HuggingFace model ID and inference API could not be confirmed from this
environment (no network in the verification subagent).

This backend is implemented as a near-clone of the MolmoAct (v1) backend
under the assumption that v2 retains the Molmo pointing output format
(`<point x="..." y="..."/>` with percentage coordinates) and the
`generate_from_batch` API. Override `MOLMOACT2_MODEL_ID` env var to point
at the actual repo when Rocky confirms it.

TODO(rocky):
  - Confirm exact HF id, e.g. `allenai/MolmoAct2-7B-D-2605` (guessed).
  - Confirm pointing output format. If v2 switches to absolute pixels or
    a different tag, update `_parse_molmo_point` import target.
  - Confirm any new prompt template (system message, action vs. point mode).
"""
from __future__ import annotations

import logging
import os

from PIL import Image

from .base import PointResult, VLMBackend
from .molmoact import _parse_molmo_point

log = logging.getLogger(__name__)

# Best-guess id. Override at runtime via env var.
DEFAULT_MODEL_ID = os.environ.get(
    "MOLMOACT2_MODEL_ID",
    "allenai/MolmoAct2-7B-D",  # UNVERIFIED — likely not yet public
)


class MolmoAct2Backend(VLMBackend):
    name = "molmoact2"
    requires_verification = True

    def __init__(self, model_id: str = DEFAULT_MODEL_ID):
        self.model_id = model_id
        self.processor = None
        self.model = None

    def load(self) -> None:
        import torch
        from transformers import AutoModelForCausalLM, AutoProcessor

        log.warning(
            "Loading MolmoAct2 (UNVERIFIED) from %s. If this fails, set "
            "MOLMOACT2_MODEL_ID env var to the actual repo id.",
            self.model_id,
        )
        self.processor = AutoProcessor.from_pretrained(
            self.model_id, trust_remote_code=True, torch_dtype="auto", device_map="auto",
        )
        self.model = AutoModelForCausalLM.from_pretrained(
            self.model_id, trust_remote_code=True, torch_dtype=torch.bfloat16, device_map="auto",
        )

    def point(self, image: Image.Image, prompt: str) -> PointResult:
        if self.model is None:
            raise RuntimeError("Backend not loaded.")
        from transformers import GenerationConfig

        inputs = self.processor.process(images=[image], text=prompt)
        inputs = {k: v.to(self.model.device).unsqueeze(0) for k, v in inputs.items()}
        output = self.model.generate_from_batch(
            inputs,
            GenerationConfig(max_new_tokens=200, stop_strings="<|endoftext|>"),
            tokenizer=self.processor.tokenizer,
        )
        generated = output[0, inputs["input_ids"].size(1):]
        text = self.processor.tokenizer.decode(generated, skip_special_tokens=True)
        result = _parse_molmo_point(text, image.size[0], image.size[1])
        if result is None:
            raise ValueError(
                f"MolmoAct2 returned no point. Output: {text!r}. "
                "If v2 changed the output format, update molmoact2.py."
            )
        return result

    def info(self) -> dict:
        return {
            "name": self.name,
            "model_id": self.model_id,
            "loaded": self.model is not None,
            "requires_verification": True,
            "notes": "MolmoAct2 API not verified. May need updating once weights are public.",
        }
