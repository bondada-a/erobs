# Spincoater Sample Orientation — Development Log & Plan

**Status:** Detection approach validated (empty pocket reliable at fixed pose);
placement-orientation math designed but **not yet implemented**. Hardware setup
is temporary (chuck position will change in deployment).

**Date of session:** 2026-05-29
**Author of session work:** Claude Code (with the EROBS developer driving the robot)

> **Purpose of this folder.** This is a self-contained record of a brainstorming +
> live-experiment session on how the robot should place a silicon wafer into a
> shallow machined indent on the spincoater chuck (and later re-pick it after
> spinning). It contains every captured image, every overlay, the detection code,
> and the full reasoning, so that a fresh session or a different agent can continue
> development **without any additional context.** This folder is temporary/scratch
> — it is not wired into the build.

---

## 1. The problem

The spincoater chuck was modified: it now has a **shallow square-ish indent**
(originally thought ~0.5 mm, **actually ~0.1–0.2 mm deep**) machined into its
polished-metal face, sized so the **silicon wafer sample is the same size or
slightly smaller** than the indent. The wafer must seat into this pocket.

This created two new problems beyond the existing pick-and-place:

1. **Placement orientation (THIS SESSION'S FOCUS):** How do we determine the
   orientation to place the sample so it sits correctly in the square indent?
   The chuck **cannot be homed/indexed** — it stops at a *random* angle every
   cycle — so the indent orientation is unknown and different each time.

2. **Re-pickup after spin (LATER):** After the spincoater spins, we must pick the
   wafer back up. Nuance: (a) we can't control the angle the chuck stops at, and
   (b) the wafer may have wiggled within the pocket slack, so it's not exactly
   aligned with the indent. We pick it up, then move to a destination that
   **cares about orientation**.

This session focused on **Problem 1**, with strong findings that also de-risk
Problem 2.

---

## 2. Established facts about the system (verified this session)

### 2.1 The existing pick→place→spincoat flow
- **Pick** (already built): vision detects the sample in an ROI anchored to an
  ArUco tag, selects a pickup point (`detect_sample` with strategy
  `farthest_edge`/`center`, or fixed-ROI mode), `vacuum_on` (ePick suction
  gripper), retreat. Vision-guided.
- **Transport + place**: `moveto` to **hardcoded** spincoater joint poses
  (`pre_spincoat` → `spincoat`), then `vacuum_off`. NOT vision-guided —
  taught joint angles.
- Reference task file: `src/cms/tasks/spincoat_to_hotplate.json` (shows the
  reverse trip — removing a finished sample). Poses in `src/cms/poses.yaml`
  (`spincoater`, `pre_spincoater`, `spincoater_scan`) and
  `src/cms/beamtime_poses.yaml` (several drifted re-teach variants — `poses.yaml`
  is authoritative).

### 2.2 Suction gripper is yaw-agnostic for *lifting*
A single ePick suction cup grabs a flat sample at any rotation. **Orientation
only matters for *placing* into the keyed indent**, never for picking up. This
simplifies Problem 2 enormously: re-pickup only needs the wafer's *location*,
not a solved orientation (unless the downstream destination needs it).

### 2.3 ArUco pickup uses a FIXED wrist yaw — **VERIFIED**
In `src/beambot/beambot/camera/zivid.py:483-487`
(`detect_sample_roi`), the pickup `Pose` is built with **position only** and
`orientation.w = 1.0` (identity quaternion). The detected sample's *rotation is
never used* to set the wrist. Therefore the sample is always grabbed at the
**same wrist orientation** regardless of how the tag/sample is rotated. This is
the precondition the placement architecture (§6) relies on.

### 2.4 The sample
**Shiny silicon wafer** — a near-mirror, low-saturation surface. In HSV it reads
as high *value*, near-zero *saturation*, meaningless *hue* (it reflects its
surroundings). Assumed **4-fold symmetric and featureless** for now (see open
question in §8 about wafer flat/notch).

---

## 3. The decisive experiment: 3D vs 2D capture

We captured the chuck live (camera mounted eye-in-hand, looking down at the
spincoater). **Working distance ≈ 256 mm.**

### 3.1 Depth (3D) is useless for this feature
- Image `01_3d_capture_full.jpg`, zoom `02_3d_disc_zoom.png`, edges
  `03_3d_disc_edgemap.png`.
- A horizontal depth profile across the disc varied only ~0.8 mm and that was a
  smooth *tilt*, not a step. A 0.1–0.2 mm indent is **at/below the Zivid Z-noise
  floor (~0.05–0.1 mm RMS at 256 mm)**. The edge map is muddy — strongest
  responses are holes and the disc circle, not the square.
- **Conclusion: this is a 2D/RGB problem. Depth is abandoned.**

### 3.2 The 2D capture is dramatically clearer — and why
- The MCP `capture_image` tool calls the **`/capture`** service (full 3D
  capture-and-detect-markers, **fires the Zivid projector** → floods polished
  metal with glare, washes out the shallow edge).
- The GUI "Capture" button calls **`/capture_2d`** (a `std_srvs/Trigger`) — a
  **single dark-exposure 2D color frame, NO projector pattern**. The indent
  reads as a **crisp bright square**. See `04_2d_capture_full.png`,
  `05_2d_indent_zoom.png`.
- **KEY INSIGHT — active illumination:** The 2D capture is *flash-lit* by the
  Zivid's own LED with a short exposure. The sensor integrates almost entirely
  the camera's own light, NOT the room's. **=> Ambient lighting in deployment is
  nearly irrelevant.** The persistent enemy is **specular reflection off
  polished aluminum** (glints from the camera's own flash), whose pattern
  changes with camera-to-chuck geometry.
- Both services publish to the same topic `/color/image_color`. The difference is
  purely the capture settings.
- A geometric `minAreaRect` fit on the raw 2D bright square worked
  (`06_2d_geometric_fit.png`: 132×125 px, aspect 1.05, angle −31°) but was
  **fragile** — sensitive to a fixed brightness threshold and corrupted by
  nearby specular glints. This motivated the marking approach (§4).

---

## 4. The chosen solution: paint the chuck red, detect by color

### 4.1 Why marking, why red, why "negative space"
- **Specular glints are the enemy**, not lighting or distance. The fix is to make
  detection **independent of brightness**. HSV color masking does exactly this:
  a glint is white (low *saturation*, high *value*); a painted region has high
  *saturation* at a specific *hue*. **Thresholding on hue+saturation ignores
  glints entirely.**
- **Constraint:** Setup is **urgent → no machining**. Marker coloring is
  acceptable; solvent exposure assumed not to be an issue on this timescale.
- **Decision (developer's idea):** Color the **entire chuck face red, leaving the
  machined pocket bare**. This is *negative-space detection*: the pocket becomes
  a **non-red hole in a red field**. Bonus: it also helps re-pickup — when the
  wafer sits on the pocket, the red field is a high-contrast backdrop.
- **Red is correct for a silicon wafer:** wafer = low-saturation mirror; red =
  high-saturation hue ~0/179. Maximally separated in *saturation*, which is the
  dimension that matters. The mirror finish (which wrecked brightness-based fits)
  is harmless to a hue/saturation mask.
- **RED HSV QUIRK — handle the hue wraparound:** Red straddles the OpenCV hue
  seam (0–179). A red mask needs **two ranges OR'd**: `inRange(0..~14)` AND
  `inRange(~166..179)`. A naive single-range red threshold silently misses half
  the red pixels.
- Scene note: there is **warm/orange tape** elsewhere in the frame (near the
  ArUco board). It's outside the chuck ROI so it's harmless, but a too-loose
  auto-locate can latch onto it (this actually happened in the offset stress
  test, §5.3).

### 4.2 Marking iterations (what the images show)
1. **Thin red lines** around the pocket (`07_red_lines_full.png`,
   `08_red_lines_hsvmask.png`): red *registers* but lines were **too thin and
   translucent** — metal glinted through, mask was broken/dotted, polluted by
   chromatic specular fringe. **Verdict: lines insufficient → fill solid.**
2. **Solid red fill** (`09_red_filled_full.png`, `10_red_filled_zoom.png`): red
   jumped to ~27% of ROI. Empty-pocket detection improved but coverage was
   **streaky** — thin spots created phantom "holes", and screw holes bit the
   square (`11_emptypocket_pipeline.png` aspect 1.32; `12_emptypocket_brightfit.png`
   aspect 1.16 after targeting bright-metal instead of generic hole).
3. **Re-colored, more opaque, screw holes filled** (`18_recolored_empty_zoom.png`,
   `19_recolored_empty_fit.png`): **clean fit — aspect 1.04, solidity 0.90.**
   The recolor fixed the coverage problem.

**Coverage is the limiting factor, not the algorithm.** Every failure traced to
thin/translucent red. **Recommendation: use an opaque oil-based paint marker**
(e.g. Sharpie paint pen) rather than a felt-tip; apply two coats up to the pocket
edge (that boundary is the detection edge). Saturation should be ~200+, not ~136.

---

## 5. Stress test — reliability (the empty/no-sample case)

Detection script: `code/detect_pocket.py` (auto-locates chuck via largest red
blob, then isolates the bright bare-metal square inside the red field). Capture
helper: `code/capture_2d.py`.

### 5.1 Fixed pose, 5 repeat captures — EXCELLENT
| metric | result |
|---|---|
| center | (1337, 863) **identical all 5 shots — 0 px jitter** |
| angle (mod 90°) | 55.2, 55.2, 55.2, 55.5, 55.2 → **spread 0.3°** |
| aspect / solidity | 1.05–1.06 / 0.89–0.90 — stable |

The flash-lit 2D capture is **deterministic** (no ambient flicker/exposure
drift) and the detector is **numerically stable**. 0.3° ≪ the ~1–2° the 90°
snap + compliant drop tolerates. **Production-grade at a fixed pose.**

### 5.2 Farther distance — STILL GOOD (`20_stresstest_far_fit.png`)
- Pocket area 8830 → 5200 px² (~1.3× farther). center ±1 px; angle 57.4–58.1°
  (spread 0.7°); **solidity improved to 0.97** (fewer pixels = less glint/hole
  contamination).
- **CRITICAL FINDING — angle is pose-dependent:** near pose measured **55.2°**,
  far pose measured **~57.7°**. The chuck did NOT rotate — only the camera moved.
  The ~2.5° shift comes from (a) perspective if the camera isn't perfectly
  perpendicular, and (b) in-plane camera *roll* when the robot moves.
  **=> The measured angle is in the camera/pixel frame and conflates chuck
  orientation with camera pose. A raw pixel-angle is only meaningful if the scan
  is always from the EXACT same robot pose.** This is why the architecture (§6)
  fixes the scan pose and calibrates one constant there.

### 5.3 Large lateral offset — FAILED (`21_stresstest_offset_FAIL.png`,
`22_stresstest_offset_crop.png`)
- Chuck moved to the **edge of the frame** → whole scene went **dark** → red
  field nearly black → red HSV mask collapsed → auto-locate even latched onto the
  orange tape. (The crop also happened to have a wafer in it, but the root cause
  is illumination.)
- **KEY INSIGHT — flash falloff:** The Zivid 2D flash illuminates a *cone*
  centered on the optical axis. Off-axis chuck = under-lit = translucent red goes
  black. **=> The chuck must be roughly CENTERED in the frame (near optical
  axis) for reliable detection.** Detection is robust to *distance* but NOT to
  large *lateral offset*.
- **Mitigation (accepted):** the spincoater scan pose is **hardcoded** anyway, so
  we set it up to frame the chuck centered and well-lit — exactly like the good
  captures (`18`/`19`). This is not a limitation in practice.

### 5.4 Reliability verdict
- **Detection robustness:** excellent at centered poses, any reasonable distance.
- **Angle measurement:** stable at a fixed pose (±0.3–0.7°), shifts ~2.5° if the
  camera moves a few cm → **must scan from one repeatable, centered pose and
  calibrate there.**

---

## 6. THE PLAN — placement-orientation architecture (to implement)

Developer's chosen architecture (validated as sound this session):

> Use the **same wrist orientation for every scan** (one fixed, hardcoded scan
> pose, framing the chuck centered — like images `18`/`19`). Treat that pose's
> measured pocket angle as the reference. Because the sample is **always picked
> from the ArUco sheet at a fixed wrist yaw** (§2.3, verified), the sample is
> locked to the gripper. So we only need to compute **how much to rotate the
> wrist** to align the gripped sample with the detected pocket, then place.

### 6.1 Why it's robust
The fixed scan pose + fixed pickup yaw eliminate every variable except the
**pocket's random rotation** — which is exactly what vision measures. A 6-DOF
placement collapses to a **single scalar** (one wrist/joint-6 angle).

### 6.2 The math
At the fixed scan pose, let `θ_detect` = pocket angle from `minAreaRect`
(mod 90°, since 4-fold symmetric). Command:

```
joint6_place = joint6_pickup + K + θ_detect
```

- `joint6_pickup` — fixed wrist angle at the ArUco pickup (sample now locked to
  gripper at that yaw).
- `θ_detect` — the ONLY live input; the measured pocket angle this cycle.
- `K` — a **single calibration constant** absorbing *everything fixed*: camera
  mount roll, pixel-axis→base-axis rotation, `minAreaRect` sign convention, and
  the sample's yaw-offset-in-gripper. **Do NOT derive K from geometry — measure
  it once empirically.** (This is what makes the scheme robust: all error-prone
  fixed unknowns are captured at their true values in one number.)

### 6.3 The 90° wrap / nearest-rotation
Both pocket and sample are 4-fold symmetric → 4 equivalent wrist angles seat the
sample. Reduce to smallest magnitude:
```
Δθ = (...) mod 90        # in [0, 90)
if Δθ > 45: Δθ -= 90     # now in [-45, 45) — never >45° of wrist motion
```
Also check **joint-6 (UR wrist_3) limits** before commanding; pick the
equivalent rotation that stays in range.

### 6.4 Approach must be TOP-DOWN
The clean "joint-6 = constant + measurement" mapping holds because the suction
approach is **vertical/top-down** at both pick and place (confirmed by developer).
If the approach were oblique, the angle would have to route through TF properly.

### 6.5 Calibration procedure for K (one-time)
1. Move to the fixed (centered) scan pose. Pick a sample from the ArUco sheet →
   records `joint6_pickup`.
2. Capture (`/capture_2d`), run detector → `θ_detect`.
3. **Manually jog the wrist** until the gripped sample visually aligns with the
   pocket; place it; record the working `joint6_place`.
4. `K = joint6_place − joint6_pickup − θ_detect`.
5. **Verify:** rotate the chuck to 3–4 different random angles; for each,
   re-detect and command `joint6_pickup + K + θ_detect`; confirm it seats.
   - Consistent fixed offset → adjust K.
   - Offset that **varies with pocket angle** → that's perspective skew (§7);
     add a small correction term (likely unnecessary if camera is near-perpendicular).

---

## 7. Perspective skew — the one assumption to validate
We measure θ in the *image*. If the camera looks perfectly straight down,
image-angle = true top-down angle and K is a clean constant. If slightly oblique,
a square's apparent angle is skewed by perspective in a way that depends on the
pocket's rotation (not perfectly constant). Captures look near-perpendicular, so
this is likely <1° and absorbed into tolerance — but **validate empirically in
step 6.5.5**, don't assume.

---

## 8. Assumptions & open questions for the next session

**Assumptions this plan rests on (all currently believed true):**
- A1. Scan is always from the **same hardcoded, centered scan pose**. (Required —
  see §5.2/§5.3.)
- A2. ArUco pickup uses a **fixed wrist yaw**. ✅ verified (§2.3).
- A3. Sample **does not move/rotate** under suction during transport. (Developer:
  "usually no, we assume it doesn't.")
- A4. Suction approach is **top-down** at pick and place. ✅ (developer).
- A5. Sample + pocket are **4-fold symmetric** → angle mod 90° suffices.
- A6. Marker red is **opaque enough** (needs the paint-marker upgrade for full
  reliability; felt-tip was marginal).

**Open questions to resolve:**
- Q1. **Does the wafer have a flat/notch or a "correct" coated orientation?** If
  yes, mod-90° is NOT enough — symmetry must be broken with a feature, and the
  destination-orientation requirement (Problem 2) needs more than the pocket
  angle. (Developer said destination "cares about orientation" — clarify whether
  90°-symmetric placement satisfies it.)
- Q2. **Re-pickup detector:** the empty-pocket detector finds a *bright* square;
  the wafer is a *dark* square. The wafer-present case needs its own detector
  (find the non-red region = wafer). Ideal wafer detection was excellent
  (`14_wafer_ideal_fit.png`: aspect 1.03, solidity 0.96) but the **non-ideal
  (rotated/offset) wafer FAILED** (`16_wafer_nonideal_fit_FAIL.png`: aspect 1.52)
  because a specular glint on the wafer edge bridged to a screw-head reflection
  AND thin red coverage on one side merged in. Root cause = same coverage issue +
  glint-on-non-red. **Recommended fix for re-pickup: detect the pocket as the
  *interior hole of the red field* (fill the red mask's holes, difference =
  pocket), keying off the stable red boundary rather than the wafer's
  unpredictable mirror appearance.** Not yet implemented.
- Q3. Confirm joint-6/wrist_3 range is sufficient for all required corrections at
  the chosen scan/place poses.

---

## 9. What to do next (suggested order)
1. Set the hardcoded **spincoater scan pose** to frame the chuck centered &
   well-lit (match images `18`/`19`).
2. Upgrade the marker to **opaque paint** (two coats); re-validate empty-pocket
   detection (should improve aspect toward 1.0, solidity toward 0.95+).
3. Implement the **placement detector** in `beambot.detection` (adapt
   `code/detect_pocket.py`); return `(center_px, angle_mod90)`.
4. Run the **K calibration** (§6.5) and the multi-angle verification.
5. Wire the wrist-angle correction into the place sequence (top-down,
   90°-snap, joint-limit check).
6. THEN tackle re-pickup (Problem 2): implement the hole-in-red-field detector
   (Q2), validate with deliberately off-nominal wafer positions.

---

## 10. File index
See `images/` (22 captures/overlays, numbered in session order) and `code/`
(detection + capture scripts). Full descriptions in `IMAGES.md`.

## 11. Reproducing captures (robot must be running with Zivid up)
```bash
source /opt/ros/jazzy/setup.bash
# Trigger the GUI-style 2D capture and save the frame:
python3 code/capture_2d.py /tmp/out.png
# Run the empty-pocket detector (auto-locates the chuck):
python3 code/detect_pocket.py /tmp/out.png
```
Note: `/capture_2d` (clean, no-projector, flash-lit) is the correct service —
**NOT** the MCP `capture_image`/`/capture` (3D, projector glare). The 2D frame
arrives on `/color/image_color`; subscribe BEFORE triggering (single-shot).
