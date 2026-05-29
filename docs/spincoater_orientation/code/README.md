# Code — Spincoater Orientation Session

Scratch scripts used to capture and analyze the chuck during the session. These
are **standalone** (not part of the colcon build). They depend only on
`rclpy`, `cv_bridge`, `sensor_msgs`, `opencv-python`, `numpy`.

When productionizing, fold the detection logic into
`src/beambot/beambot/detection/` (the single source of truth for detection) and
expose a 2D-capture path to the orchestrator/MCP (currently only the GUI calls
`/capture_2d`; the MCP `capture_image` uses the projector-on `/capture`).

| File | Purpose |
|------|---------|
| `capture_2d.py` | Trigger the GUI-style 2D capture (`/capture_2d`) and save the frame from `/color/image_color`. Subscribes BEFORE triggering (single-shot). Usage: `python3 capture_2d.py /tmp/out.png` |
| `detect_pocket.py` | **EMPTY-pocket** detector. Auto-locates chuck (largest red blob) → isolates BRIGHT bare-metal square inside red field → `minAreaRect`. Returns center + angle (mod 90°). Validated reliable at a fixed centered pose. Usage: `python3 detect_pocket.py <img> [cx cy half]` |
| `detect_wafer.py` | **WAFER-present** detector (re-pickup, WIP). `v1_nonred` = largest non-red blob (works ideal, fails non-ideal). `v2_hole` = pocket as interior hole of red field (RECOMMENDED, glint-immune, **untested on hardware**). Usage: `python3 detect_wafer.py <img> [cx cy half]` |

## Tunables / conventions (shared across detectors)
- **Red HSV dual-range** (hue wraparound): `(0,60,40)–(14,255,255)` OR
  `(166,60,40)–(179,255,255)`. Raise the min-saturation if ambient red noise
  appears; with opaque paint, saturation should be high (~200+).
- **Bright bare-metal** (empty pocket): `inRange(V≥150, S≤90)`.
- **Filters:** keep candidates with `aspect < 1.25` and `solidity > 0.85`. A
  clean square gives aspect ≈ 1.05, solidity ≈ 0.90–0.97.
- **Angle is mod 90°** (4-fold symmetry). It is in the CAMERA/pixel frame — only
  meaningful at a fixed scan pose (see README §5.2/§6).

## Hardware prerequisites
- Robot + Zivid driver running (the full MoveIt stack is NOT required — the
  Zivid `/capture_2d` service runs independently). `source /opt/ros/jazzy/setup.bash`.
- Chuck framed **centered** in view (off-axis = flash falloff = detection fails;
  README §5.3).
