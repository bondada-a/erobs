"""FastAPI VLM pointing service.

Routes:
    GET  /health         — liveness/readiness probe
    GET  /info           — backend metadata
    POST /point          — single image + prompt -> single pixel coord
    POST /point_batch    — list of {image, prompt} -> list of results

Image is sent as base64-encoded bytes (PNG/JPEG). This avoids multipart
complexity and keeps the client side trivial (httpx + b64encode).

Run with:
    python -m vlm_service.server --backend stub --host 0.0.0.0 --port 8765

or via the convenience launcher:
    ./launch.sh stub
"""
from __future__ import annotations

import argparse
import base64
import io
import logging
import os
from typing import List

from fastapi import FastAPI, HTTPException
from PIL import Image
from pydantic import BaseModel, Field

from .backends import get_backend, list_backends

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
log = logging.getLogger("vlm_service")


# --- Schema ----------------------------------------------------------------

class PointRequest(BaseModel):
    image_b64: str = Field(..., description="Base64-encoded image bytes (PNG/JPEG).")
    prompt: str = Field(..., description="Pointing prompt, e.g. 'Point to the sample on the puck.'")


class PointResponse(BaseModel):
    x: float
    y: float
    confidence: float
    raw: dict
    backend: str


class BatchPointRequest(BaseModel):
    items: List[PointRequest]


class BatchPointResponse(BaseModel):
    results: List[PointResponse]


class HealthResponse(BaseModel):
    status: str
    backend: str
    loaded: bool


# --- App factory -----------------------------------------------------------

def _decode_image(b64: str) -> Image.Image:
    try:
        raw = base64.b64decode(b64, validate=True)
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"image_b64 is not valid base64: {e}")
    try:
        return Image.open(io.BytesIO(raw)).convert("RGB")
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Could not decode image: {e}")


def create_app(backend_name: str = "stub", load_now: bool = True) -> FastAPI:
    app = FastAPI(title="VLM Pointing Service", version="0.1.0")
    backend = get_backend(backend_name)
    state = {"backend": backend, "loaded": False, "name": backend_name}

    if load_now:
        log.info("Loading backend %s ...", backend_name)
        backend.load()
        state["loaded"] = True
        log.info("Backend %s ready.", backend_name)

    @app.get("/health", response_model=HealthResponse)
    def health() -> HealthResponse:
        return HealthResponse(
            status="ok",
            backend=state["name"],
            loaded=state["loaded"],
        )

    @app.get("/info")
    def info() -> dict:
        return {
            "service": "vlm_pointing",
            "available_backends": list_backends(),
            "active": state["backend"].info(),
        }

    @app.post("/point", response_model=PointResponse)
    def point(req: PointRequest) -> PointResponse:
        if not state["loaded"]:
            raise HTTPException(status_code=503, detail="Backend not loaded yet.")
        img = _decode_image(req.image_b64)
        try:
            result = state["backend"].point(img, req.prompt)
        except ValueError as e:
            raise HTTPException(status_code=422, detail=str(e))
        except Exception as e:
            log.exception("Backend error")
            raise HTTPException(status_code=500, detail=f"Backend error: {e}")
        return PointResponse(
            x=result.x, y=result.y, confidence=result.confidence,
            raw=result.raw, backend=state["name"],
        )

    @app.post("/point_batch", response_model=BatchPointResponse)
    def point_batch(req: BatchPointRequest) -> BatchPointResponse:
        if not state["loaded"]:
            raise HTTPException(status_code=503, detail="Backend not loaded yet.")
        out: List[PointResponse] = []
        for item in req.items:
            img = _decode_image(item.image_b64)
            try:
                r = state["backend"].point(img, item.prompt)
            except Exception as e:
                raise HTTPException(status_code=500, detail=f"Backend error in batch: {e}")
            out.append(PointResponse(
                x=r.x, y=r.y, confidence=r.confidence,
                raw=r.raw, backend=state["name"],
            ))
        return BatchPointResponse(results=out)

    return app


# --- CLI -------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--backend", default=os.environ.get("VLM_BACKEND", "stub"),
        choices=list_backends(),
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-load", action="store_true",
                        help="Skip backend.load() at startup (debug only).")
    args = parser.parse_args()

    import uvicorn
    app = create_app(args.backend, load_now=not args.no_load)
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()
