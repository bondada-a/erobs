# VLM pointing service for EROBS

A standalone HTTP service that runs a Vision-Language Model and returns
2D pixel coordinates given an image + a natural-language prompt. The
EROBS ROS workspace calls it through the MCP tool `detect_sample_vlm`.

## 1. What this is

Several pointing-capable VLMs — in-process HuggingFace models (MolmoAct,
MolmoAct2, RoboBrain 2.5, RoboPoint) plus an OpenAI-compatible client backend
(`cosmos`, see §8b) and a deterministic stub — all wrapped behind one FastAPI
service so swapping models is a config change, not a code change. The
service runs in **its own Python venv on its own host** — typically a
remote A100/H100 GPU box. The ROS workspace reaches it over HTTP, which
keeps `torch` / `transformers` out of the ROS Python environment (a
known pain point — see the `erobs-sim-loop` skill on Hermes-venv vs.
ROS Python 3.12).

## 2. Architecture

```
                    ┌─────────────────────┐
   Zivid image ───▶ │ MCP tool            │
   prompt      ───▶ │ detect_sample_vlm() │
                    │ (in ROS workspace)  │
                    └──────────┬──────────┘
                               │  HTTP POST /point
                               │  {image_b64, prompt}
                               ▼
                    ┌─────────────────────┐
                    │  vlm_service        │
                    │  FastAPI (uvicorn)  │
                    │  Port 8765          │
                    │                     │
                    │  ┌───────────────┐  │
                    │  │ Backend       │  │
                    │  │  stub         │  │
                    │  │  molmoact     │  │
                    │  │  molmoact2    │  │
                    │  │  robobrain25  │  │
                    │  └───────┬───────┘  │
                    └──────────┼──────────┘
                               │  pixel (x, y), confidence, raw
                               ▼
                    ┌─────────────────────┐
                    │ MCP tool returns    │
                    │ pixel_x, pixel_y,   │
                    │ pickup_base_xyz,    │
                    │ confidence          │
                    └─────────────────────┘
```

## 3. Hardware requirements per backend

| Backend       | Min VRAM | Recommended | Runs on            | Latency (single image) |
|---------------|----------|-------------|--------------------|------------------------|
| `stub`        | 0 GB     | —           | CPU (any laptop)   | <5 ms                  |
| `molmoact`    | 16 GB    | 24 GB+      | RTX 4090, A100     | 1–3 s (bf16)           |
| `molmoact`    | 8 GB     | 12 GB       | RTX 4060/4070 (INT8) | 3–6 s                |
| `molmoact2`   | ~16 GB*  | 24 GB+      | RTX 4090, A100     | ~1–3 s* (estimated)    |
| `robobrain25` | ~16 GB*  | 24 GB+      | RTX 4090, A100     | ~1–3 s* (estimated)    |
| `robopoint`   | 26 GB    | 32 GB+      | RTX 4090, A100     | 2–4 s (fp16, 13B)      |
| `robopoint`   | 14 GB    | 16 GB       | RTX 4080 (7B variant) | 1–2 s                |

`*` = unverified — see §10.

`stub` is what the test suite uses and what runs on the dev laptop
(GTX 1660 Ti, 6 GB) — it returns deterministic fake coordinates so the
wire format can be tested end-to-end without any model weights.

## 4. Installation

On the GPU host (or laptop, for stub-only):

```bash
git clone <erobs-repo>
cd erobs-jazzy/vlm_service
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

For non-stub backends, pre-download the model weights so first launch
isn't slow / network-dependent:

```bash
# MolmoAct (confirmed)
huggingface-cli download allenai/MolmoAct-7B-D-0812

# MolmoAct2 — UNVERIFIED id (see §10). Best guess:
huggingface-cli download allenai/MolmoAct2-7B-D    # may 404 — confirm with Rocky

# RoboBrain 2.5 — UNVERIFIED id (see §10). Best guesses, in priority order:
huggingface-cli download BAAI/RoboBrain2.5-7B      # may 404
huggingface-cli download BAAI/RoboBrain2.0-7B      # confirmed predecessor

# RoboPoint (confirmed) — LLaVA-1.5 fine-tuned for robot affordance pointing
huggingface-cli download wentao-yuan/robopoint-v1-vicuna-v1.5-13b
# Smaller 7B variant for ≤16 GB cards:
huggingface-cli download wentao-yuan/robopoint-v1-vicuna-v1.5-7b
```

If a guessed id 404s, override at runtime:
```bash
export MOLMOACT2_MODEL_ID=allenai/<actual-id>
export ROBOBRAIN_MODEL_ID=BAAI/RoboBrain2.0-7B   # use predecessor as fallback
```

## 5. Running the server

```bash
# stub (no GPU needed):
./launch.sh stub

# real models:
./launch.sh molmoact
./launch.sh molmoact2
./launch.sh robobrain25
```

Expected startup output (stub):
```
INFO ... vlm_service: Loading backend stub ...
INFO ... vlm_service: Backend stub ready.
INFO ... uvicorn.error: Uvicorn running on http://0.0.0.0:8765
```

Health check (separate shell):
```bash
curl -s http://localhost:8765/health
# {"status":"ok","backend":"stub","loaded":true}

curl -s http://localhost:8765/info | python3 -m json.tool
```

### Troubleshooting

| Symptom | Fix |
|---|---|
| `CUDA out of memory` on a 12 GB card | Set `--load-in-8bit` env (see `bitsandbytes` config in `requirements.txt`) or pass quantized model id |
| Still OOM at INT8 | Use INT4 (NF4) via `bitsandbytes` BitsAndBytesConfig — costs ~2× latency |
| `404 Repository not found` for molmoact2 / robobrain25 | Override with `MOLMOACT2_MODEL_ID` / `ROBOBRAIN_MODEL_ID` env vars |
| Pointing returns coords outside image | Backend may be emitting normalized output; the parser converts MolmoAct (0–100%) and RoboBrain (absolute pixels) — verify `info()` for the active backend's `point_format` field |
| `ModuleNotFoundError: rclpy._rclpy_pybind11` | You're running this in the ROS env. Don't. Use a clean venv on the GPU host. |

## 6. ROS-side configuration

In `src/beambot/config/cms_beamline.yaml` (or the active beamline):

```yaml
vlm:
  service_url: "http://gpu-box.lan:8765"   # or http://localhost:8765
  model_backend: "molmoact"                # informational; backend chosen at server launch
  timeout_seconds: 10
  enabled: true                            # default false; flip to enable detect_sample_vlm
```

When `enabled: false` (default), `detect_sample_vlm` returns a clear
"VLM detector is disabled" error and existing detection paths
(`detect_sample`, `detect_sample_yolo`) work unchanged.

## 7. End-to-end test on real hardware

```bash
# 1. Start the server (GPU host)
cd ~/erobs-jazzy/vlm_service
./launch.sh molmoact

# 2. Verify health (any host with network access)
curl http://gpu-box.lan:8765/health

# 3. Send a real Zivid frame
python3 - <<'PY'
import base64, json, urllib.request
img = open("/tmp/zivid_capture.png", "rb").read()
body = json.dumps({
    "image_b64": base64.b64encode(img).decode(),
    "prompt": "Point to the silicon sample chip on the puck.",
}).encode()
r = urllib.request.urlopen(urllib.request.Request(
    "http://gpu-box.lan:8765/point",
    data=body, headers={"Content-Type": "application/json"}))
print(json.dumps(json.loads(r.read()), indent=2))
PY

# 4. From the ROS side via the Claude agent / MCP (with vlm.enabled=true):
#    - capture_image(camera="zivid", mode="3d")
#    - detect_sample_vlm(prompt="Point to the silicon sample chip on the puck.")
#    Verify: pickup_base_xyz is non-null and roughly matches the puck location.
```

## 8. Switching backends

Restart the server with a different `--backend` flag. **No ROS-side
changes required** — the `model_backend` field in YAML is informational
only; the actual model is whichever the running server loaded.

```bash
# was: ./launch.sh molmoact
pkill -f "vlm_service.server"
./launch.sh robobrain25
```

## 8b. The `cosmos` backend (NVIDIA Cosmos 3 Reasoner)

`cosmos` is **different from every other backend**: it does not load weights
in-process. Cosmos 3 Reasoner can't be loaded via `transformers` yet (NVIDIA
lists that path as "Coming soon"), so it runs only as its own
OpenAI-compatible HTTP server, and this backend is a thin **client** that POSTs
an image + grounding prompt and parses the returned 2D boxes (returning the
center of the first box). `requires_verification: true` — the exact grounding
JSON schema and coordinate space are unconfirmed until a first real run; the
parser is defensive and stashes raw coords in the response `raw` for
calibration.

Three ways to serve Cosmos (pick one, then point the backend at it via env):

| Option | When | Setup |
|---|---|---|
| **Hosted NVIDIA API** | easiest first call, no GPU | API key only; `COSMOS_BASE_URL=https://integrate.api.nvidia.com/v1`, `COSMOS_API_KEY=nvapi-...` |
| **NIM container** | easiest self-host (e.g. Orion) | `docker run ... nvcr.io/nim/nvidia/cosmos3-reasoner` → OpenAI `/v1` on `:8000` |
| **vLLM** | self-host w/o NIM | `vllm serve` + the `vllm-cosmos3` package → `:8000` |

Backend config is via env vars on the **service host** (mirrors the
`XXX_MODEL_ID` pattern of the other backends):

```bash
export COSMOS_BASE_URL="http://localhost:8000/v1"        # or the hosted URL
export COSMOS_API_KEY=""                                  # set for hosted API
export COSMOS_MODEL="nvidia/cosmos3-nano-reasoner"        # match /v1/models
./launch.sh cosmos
```

`load()` does a best-effort `/models` probe and warns (does not crash) if the
endpoint is down, so you can start `cosmos` before the model server is ready.

**ROS-side timeout:** Cosmos reasoning latency exceeds the `VLMDetector`
default of 10 s (`detection/vlm_detector.py`). Set `vlm.timeout_seconds` to
~60 in the beamline yaml (`cms_beamline.yaml` already uses 30 — bump if you see
timeouts). No other ROS-side change is needed; the backend is selected by the
service's `--backend`, and `vlm.model_backend` in the yaml is informational.

## 9a. Local single-machine deployment

The service is just an HTTP server — running it on the same box as ROS works
the same as the remote case, you just point `service_url` at `localhost`.
Two real reasons to do this: dev-loop iteration without a GPU box, or a
single-workstation beamline with a strong GPU (4090 / 5070Ti).

**Critical: keep the VLM venv separate from the ROS Python env.** Mixing
`torch` / `transformers` into the ROS Python 3.12 environment is the same
trap the `erobs-sim-loop` skill warns about. The VLM venv lives at
`vlm_service/.venv` and is never sourced by ROS — they only talk over HTTP.

### Stub backend (no GPU — wire-format smoke test)

Useful on the dev laptop to verify the whole MCP → HTTP → response path
before any model weights exist.

```bash
# Terminal A — start the service in its own venv
cd ~/erobs-jazzy/vlm_service
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt   # for stub-only, fastapi+uvicorn+pillow+pydantic is enough
./launch.sh stub
# → Uvicorn running on http://0.0.0.0:8765

# Terminal B — health check (any shell, including the ROS one)
curl -s http://localhost:8765/health
# {"status":"ok","backend":"stub","loaded":true}

# Terminal B — fire up ROS, flip vlm.enabled=true, run the MCP tool
# (vlm.service_url already defaults to http://localhost:8765 in cms_beamline.yaml)
```

### Real backend on the same machine (GPU required)

```bash
# Terminal A — VLM service in its own venv
cd ~/erobs-jazzy/vlm_service
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
huggingface-cli download allenai/MolmoAct-7B-D-0812
./launch.sh molmoact
# First load takes 30–60s; watch for "Backend molmoact ready."

# Terminal B — ROS + MCP, in the usual env-stripped shell from the
# erobs-sim-loop skill. No env changes needed for the VLM call —
# the MCP tool just does an HTTP POST to localhost:8765.
```

**GPU sharing caveat:** if anything else on the box (Isaac Sim, RViz with
GPU rendering, another model) is competing for VRAM, MolmoAct's
~16 GB BF16 footprint will OOM. Either kill the competitor or launch
with INT8: `LOAD_IN_8BIT=1 ./launch.sh molmoact`.

**Port conflict:** default port 8765 is arbitrary. If something else has
it, pass the port as the second arg: `./launch.sh stub 8766` and update
`vlm.service_url` in the beamline yaml to match.

**Verify which Python is which:**
```bash
# In the VLM venv shell
which python3   # → ~/erobs-jazzy/vlm_service/.venv/bin/python3
# In the ROS shell (after env-strip + ROS source)
which python3   # → /usr/bin/python3
```
If those don't match, you've got cross-contamination — stop and re-do the
venv strip from the `erobs-sim-loop` skill.

## 9b. Remote GPU deployment

Run the service on an A100/H100 box, point ROS at it:

```yaml
# default_beamline.yaml on the ROS host
vlm:
  service_url: "http://gpu.cms.bnl.gov:8765"
  timeout_seconds: 15  # bump for inter-DC latency
  enabled: true
```

**Network requirements:**
- Port 8765 reachable from ROS host (open the firewall both ways).
- Latency budget: image upload (~1 MB PNG) + inference (~2 s) +
  download. Round-trip on a LAN: 2–3 s. Over WAN with 50 ms RTT: still
  ~2–3 s (inference dominates).
- Auth: **none in v0.1.** Only deploy on a trusted internal network.
  Add nginx + a shared header token before exposing publicly.

## 10. Known limitations

- **MolmoAct2 weights may not be public yet.** Backend implemented as a
  near-clone of MolmoAct (v1) under the assumption v2 keeps the
  `<point x="..." y="..."/>` percentage-coords output and the
  `generate_from_batch` API. Marked `requires_verification: true` in
  `/info`. Override `MOLMOACT2_MODEL_ID` env var with the real id when
  available.
- **RoboBrain 2.5 inference API may differ.** Expected to be Qwen2.5-VL
  based with absolute-pixel JSON list output; parser handles both
  `[[x,y]]` and bare `[x,y]`. If 2.5 changes the output format, update
  `_parse_robobrain_point` in `backends/robobrain25.py`. Predecessor id
  `BAAI/RoboBrain2.0-7B` is confirmed and works as a fallback.
- **No multi-image / video.** Single-frame pointing only. The detector
  is stateless — if you need temporal smoothing, do it caller-side.
- **No 3D output for MolmoAct.** RoboBrain 2.5 does emit native 3D per
  the paper; we currently extract just the 2D pixel coord and stash any
  extra structure in `raw` for downstream inspection.
- **Quantization tradeoffs.** INT8 with `bitsandbytes` typically costs
  ~10–15% accuracy on pointing benchmarks; INT4 (NF4) costs ~25%. Acceptable
  for coarse "where is the puck" but may miss small offsets like
  edge-grip vs. center-grip.
- **No auth.** v0.1 is trusted-LAN only.

## 11. Tests

```bash
cd vlm_service
python -m pytest tests/ -v
# 15 passed in ~1 s — wire format + parser logic, all using the stub backend.
```

ROS-side tests (mocks the HTTP layer):
```bash
cd ~/erobs-jazzy
PYTHONPATH=src/beambot /usr/bin/python3 -m pytest src/beambot/test/test_vlm_detector.py -v
# 12 passed in ~1.5 s.
```
