# Image Index — Spincoater Orientation Session

All images are in `images/`, numbered in the chronological order they were
captured/produced during the session. Working distance ≈ 256 mm, eye-in-hand
Zivid looking down at the chuck. Unless noted, captures are via `/capture_2d`
(flash-lit, no projector).

| # | File | What it shows | Takeaway |
|---|------|---------------|----------|
| 01 | `01_3d_capture_full.jpg` | **3D capture** (MCP `capture_image` / `/capture`, projector ON) of full scene. Chuck centered. | Projector glare washes out the shallow indent. |
| 02 | `02_3d_disc_zoom.png` | Zoom of the polished central disc from the 3D capture. | A faint diamond is visible but partly specular reflection. |
| 03 | `03_3d_disc_edgemap.png` | FIND_EDGES map of #02. | Indent edges are FAINT; strongest responses are holes & disc circle. Depth/3D is not usable. |
| 04 | `04_2d_capture_full.png` | **2D capture** (`/capture_2d`, projector OFF, flash). Same chuck. | Much darker, controlled lighting; a bright square (the indent) is clearly visible center-chuck. |
| 05 | `05_2d_indent_zoom.png` | Zoom of the indent from the 2D capture. | Crisp bright square with sharp edges — the 2D path is far clearer than 3D. |
| 06 | `06_2d_geometric_fit.png` | `minAreaRect` fit (red box, green centroid) on raw 2D bright square, with angle label. | Works (132×125, aspect 1.05, angle −31°) but fragile: corner overshoots into a glint; fixed-threshold dependent. |
| 07 | `07_red_lines_full.png` | First marking attempt: **thin red lines** drawn around the pocket. Full 2D frame. | Lines visible but thin/translucent. |
| 08 | `08_red_lines_hsvmask.png` | 3-panel: ROI \| red HSV mask \| green overlay, for the thin lines. | Red registers but broken/dotted; polluted by chromatic specular fringe. Lines insufficient. |
| 09 | `09_red_filled_full.png` | **Solid red fill** of chuck face (pocket left bare). Full 2D frame. | Strong red disc with bright bare-metal square hole in center. |
| 10 | `10_red_filled_zoom.png` | Zoom of the red-filled chuck. | Red still somewhat streaky/translucent; pocket + screw holes are bright. |
| 11 | `11_emptypocket_pipeline.png` | 4-panel: ROI \| red mask \| holes \| fit. "Find any non-red hole" approach. | FAILS cleanly: square merges with a screw hole + thin-red gaps → aspect 1.32. |
| 12 | `12_emptypocket_brightfit.png` | 3-panel: ROI \| bright-metal mask \| fit. Improved: target high-V/low-S bright square. | Better (aspect 1.16) but box still slightly oversized from glint above pocket. |
| 13 | `13_wafer_ideal_zoom.png` | Silicon wafer placed in pocket in an **ideal** (centered, flat) position. Zoom. | Wafer = dark grey mirror square with a specular streak; clearly distinct from red. |
| 14 | `14_wafer_ideal_fit.png` | 3-panel fit of the ideal wafer (non-red region inside red field). | **Best fit of session: aspect 1.03, solidity 0.96.** Wafer covers screw holes → cleanest target. |
| 15 | `15_wafer_nonideal_zoom.png` | Wafer moved to a **non-ideal** rotated/offset position. Zoom. | Wafer rotated; bright specular reflection on its left edge. |
| 16 | `16_wafer_nonideal_fit_FAIL.png` | 3-panel fit attempt on the non-ideal wafer. | **FAILS: aspect 1.52.** Glint on wafer edge bridges to a screw-head reflection + thin red merges in. |
| 17 | `17_wafer_nonideal_diag.png` | 4-panel diagnostic: ROI \| red \| not-red \| dark(V<140). | Root cause: a black BITE in the red field on the left (thin coverage) joins the wafer blob. Coverage-limited. |
| 18 | `18_recolored_empty_zoom.png` | After **re-coloring more opaquely + filling screw holes**, empty pocket. Zoom. | Much more uniform red; clean bright square pocket. |
| 19 | `19_recolored_empty_fit.png` | 3-panel fit of the recolored empty pocket. | **Clean: aspect 1.04, solidity 0.90.** Recolor fixed the coverage problem. |
| 20 | `20_stresstest_far_fit.png` | Empty-pocket fit at a **farther** camera distance (overlay). | Robust to distance (aspect 1.08, solidity 0.97) BUT angle shifted 55.2°→57.7° because the camera moved (pose-dependence). |
| 21 | `21_stresstest_offset_FAIL.png` | Full frame with chuck moved to the **edge of frame** (large lateral offset). | Scene goes DARK (flash falloff off-axis) → red collapses → detection FAILS. Chuck must be centered. |
| 22 | `22_stresstest_offset_crop.png` | Crop of the dim off-axis chuck from #21 (a wafer was also present). | Confirms under-illumination off the optical axis. |

## Quantitative summary (empty-pocket detector, `minAreaRect`)

| Condition | center jitter | angle (mod 90) | aspect | solidity | verdict |
|-----------|---------------|----------------|--------|----------|---------|
| Recolored, fixed pose, ×5 | 0 px | 55.2–55.5° (0.3° spread) | 1.05 | 0.90 | reliable |
| Farther pose, ×3 | ±1 px | 57.4–58.1° (0.7° spread) | 1.08 | 0.97 | reliable (but different absolute angle — pose-dependent) |
| Large lateral offset | — | — | — | — | **FAIL (dark / red collapses)** |
| Wafer ideal | — | — | 1.03 | 0.96 | reliable |
| Wafer non-ideal | — | — | 1.52 | 0.84 | **FAIL (glint+coverage merge)** |
