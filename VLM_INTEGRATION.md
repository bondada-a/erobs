# VLM Detector Integration

This branch (`feat/vlm-detector-integration`) adds a Vision-Language Model
based sample detector to EROBS as a swappable third option alongside ArUco
(`detect_sample`) and YOLO (`detect_sample_yolo`).

## Quick links
- Service code, backends, tests, docs: [`vlm_service/`](./vlm_service/)
- Setup + deployment guide: [`vlm_service/README.md`](./vlm_service/README.md)
- ROS-side HTTP client: [`src/beambot/beambot/detection/vlm_detector.py`](./src/beambot/beambot/detection/vlm_detector.py)
- New MCP tool: `detect_sample_vlm` in [`src/beambot/mcp/beambot_mcp_server.py`](./src/beambot/mcp/beambot_mcp_server.py) (next to existing `detect_sample`)

## TL;DR

The VLM runs as a standalone FastAPI service on its own venv (typically a
remote GPU host). The ROS workspace POSTs `{image_b64, prompt}` and gets
back `{x, y, confidence, raw}`. This avoids polluting the ROS Python env
with `torch`/`transformers`.

Three real backends + one stub:

| Backend       | Status         | HF id                                   |
|---------------|----------------|-----------------------------------------|
| `stub`        | ✅ tested      | (no model — deterministic fake coords)  |
| `molmoact`    | ✅ implemented | `allenai/MolmoAct-7B-D-0812` (confirmed)|
| `molmoact2`   | ⚠ unverified  | `allenai/MolmoAct2-7B-D` (guessed)      |
| `robobrain25` | ⚠ unverified  | `BAAI/RoboBrain2.5-7B` (guessed)        |
| `robopoint`   | ✅ implemented | `wentao-yuan/robopoint-v1-vicuna-v1.5-13b` (confirmed) |

`requires_verification: true` is reported in `/info` for the unverified
backends. See [`vlm_service/README.md` §10](./vlm_service/README.md) for
fallbacks.

## Default behavior is unchanged

The new MCP tool checks `vlm.enabled` in the active beamline config (default:
`false`) and returns a clear error if disabled. Existing detection paths are
not touched.

To enable: set `vlm.enabled: true` in `src/beambot/config/<beamline>.yaml`
and start the VLM service somewhere reachable.

## What was added

- `vlm_service/` — new top-level directory (separate process, separate venv)
- `src/beambot/beambot/detection/vlm_detector.py` — new file
- `src/beambot/test/test_vlm_detector.py` — new test file

## What was modified (minimal)

- `src/beambot/mcp/beambot_mcp_server.py` — added `detect_sample_vlm` MCP
  tool next to `detect_sample`. No existing tools changed.
- `src/beambot/config/cms_beamline.yaml`, `lix_beamline.yaml` — appended
  a `vlm:` section, default `enabled: false`.
