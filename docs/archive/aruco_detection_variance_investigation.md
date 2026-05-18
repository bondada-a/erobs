# ArUco Marker Detection Variance Investigation

**Date:** 2026-01-22
**System:** UR5e + Zivid 2+ (eye-in-hand)
**Goal:** Reduce position variance from ~1mm to <0.5mm for reliable sample manipulation

---

## Problem Statement

When detecting ArUco markers with the Zivid camera, we observe:
- **Y-axis variance of ~1mm** (standard deviation)
- **Bimodal pattern** that appears randomly (detections cluster into two groups ~2-3mm apart)
- **High Y-Z correlation (0.99+)** suggesting systematic rather than random error

This variance is problematic for precision sample handling where sub-millimeter accuracy is needed.

---

## System Configuration

### Hardware
- **Robot:** UR5e 6-DOF arm
- **Camera:** Zivid 2+ M60 (3D structured light)
- **Mounting:** Eye-in-hand (camera on robot wrist)
- **Working distance:** ~400-600mm from markers

### Hand-Eye Calibration (2026-01-15)
```
tool0 → zivid_optical_frame
Translation (m): x=0.05675, y=0.10322, z=0.05489
Rotation (rad):  roll=-0.00615, pitch=0.04362, yaw=3.13541
Rotation (deg):  roll=-0.35°, pitch=2.50°, yaw=179.65°
Residuals: rotation < 0.22°, translation < 0.47mm
```

---

## Tests Conducted

### Summary Table

| Test | Detection Method | Configuration | Y σ (mm) | Y-Z Corr | Bimodal? | Tag |
|------|------------------|---------------|----------|----------|----------|-----|
| OpenCV 10s delay (old JSON) | OpenCV + PointCloud | Unknown | **0.658** | 0.744 | No | 9 |
| OpenCV + TF fix | OpenCV + PointCloud | Pre-capture timestamp | 0.875 | 0.995 | No | 9 |
| Zivid Native | Zivid SDK | Single capture | 0.888 | 0.991 | No | 2 |
| OpenCV 10s (new JSON) | OpenCV + PointCloud | 10s delay | 0.898 | 0.997 | No | 9 |
| Multi-capture 3x | Zivid SDK | 3-capture average | 0.906 | 0.993 | Yes | 2 |
| Zivid Native | Zivid SDK | Single capture | 0.916 | 0.998 | No | 9 |
| Zivid + TF fix | Zivid SDK | Pre-capture timestamp | 0.985 | 0.997 | No | 9 |
| tf_opencv_settings | OpenCV + PointCloud | Various settings | 1.071 | 0.997 | No | 9 |
| Pitch zeroed | Zivid SDK | roll=0, pitch=0 | 1.078 | 0.998 | No | 2 |
| OpenCV No delay | OpenCV + PointCloud | No delay | 1.141 | 0.999 | No | 9 |
| OpenCV reverted | OpenCV + PointCloud | Original calibration | 1.250 | 0.998 | Yes | 2 |
| Zivid Native (new run) | Zivid SDK | Single capture | 1.295 | 0.996 | Yes | 2 |

---

## Detailed Test Descriptions

### 1. Detection Method Comparison

#### OpenCV + Point Cloud Lookup
- Triggers Zivid capture via service
- Subscribes to image and point cloud topics
- Runs OpenCV ArUco detection on 2D image
- Looks up 3D positions for 4 corners in point cloud
- Averages corners for center position

#### Zivid Native Detection
- Calls Zivid's `CaptureAndDetectMarkers` service
- Uses Zivid SDK's built-in ArUco detection
- Returns 3D poses directly

**Result:** Both methods show similar ~1mm variance. Detection method is NOT the cause.

---

### 2. TF Timestamp Fix

**Problem:** The Zivid ROS driver assigns timestamps AFTER capture/processing completes (~200-400ms late). TF lookups use this late timestamp, getting the wrong robot pose.

**Fix:** Capture timestamp BEFORE calling the Zivid service:
```python
pre_capture_stamp = node.get_clock().now().to_msg()
# ... call Zivid service ...
return DetectionResult(markers=detected, capture_stamp=pre_capture_stamp)
```

**Result:**
- Zivid + TF fix: 0.985mm (vs 0.916mm baseline)
- OpenCV + TF fix: 0.875mm

The fix is theoretically correct but did not significantly improve variance.

---

### 3. Calibration Roll/Pitch Test

**Hypothesis:** The small calibration tilts (roll=-0.35°, pitch=2.50°) might introduce perspective-dependent error.

**Test:** Set roll=0, pitch=0, yaw=180° (zeroed tilts)

**Result:**
- Pitch zeroed: Y σ = 1.078mm
- Original calibration: Y σ = 0.888mm (same tag, Zivid native)

Zeroing pitch/roll did NOT help. Small calibration tilts are NOT the cause.

---

### 4. Multi-Capture Averaging (Option A: Same Position)

**Hypothesis:** Random sensor noise can be reduced by averaging multiple captures.

**Theory:** σ_avg = σ_single / √N
- 3 captures: 42% reduction (1.0mm → 0.58mm)
- 5 captures: 55% reduction (1.0mm → 0.45mm)

**Implementation:**
```python
def detect_markers(..., num_captures: int = 1):
    positions_by_marker = {mid: [] for mid in marker_ids}

    for capture_num in range(num_captures):
        # Call Zivid service
        # Store positions: positions_by_marker[id].append((x, y, z))

    # Average positions
    avg_x = sum(p[0] for p in positions) / len(positions)
    avg_y = sum(p[1] for p in positions) / len(positions)
    avg_z = sum(p[2] for p in positions) / len(positions)
```

**Result:**
- Multi-capture 3x: Y σ = 0.906mm
- Actual improvement: 9.4%
- Theoretical improvement: 42.3%

**Conclusion:** Multi-capture averaging from the same position does NOT help significantly because the error is SYSTEMATIC (high Y-Z correlation), not random noise.

---

### 5. Random Variation Between Runs

**Key Finding:** Same test configuration produces wildly different results:

| Run | Y σ (mm) | Bimodal? |
|-----|----------|----------|
| Zivid Native [T2] (first run) | 0.888 | No |
| Zivid Native [T2] (second run) | 1.295 | Yes |

Same tag, same method, same calibration → 46% worse variance and bimodal pattern appeared.

**Conclusion:** Significant random variation exists between test runs. Some factor we haven't identified changes between runs.

---

## Key Findings

### 1. Y-Z Correlation
All tests show Y-Z correlation of 0.99+, indicating:
- When Y increases, Z increases proportionally
- This is **systematic error**, not random noise
- Likely related to camera viewing angle or depth estimation

**Exception:** The "old JSON" test had Y-Z correlation of 0.744, much lower than all other tests.

### 2. Bimodal Pattern
The bimodal pattern (two clusters ~2-3mm apart) appears randomly:
- Not tied to detection method
- Not tied to calibration settings
- Not tied to specific tags
- Appears in some runs, not others

### 3. Best Result Unexplained
The "OpenCV 10s delay (old JSON)" test achieved:
- Y σ = 0.658mm (best by far)
- Y-Z correlation = 0.744 (uniquely low)

We cannot reproduce this result. The "old JSON" configuration is unknown.

---

## What We Ruled Out

| Factor | Tested | Result |
|--------|--------|--------|
| Detection method | OpenCV vs Zivid native | Both ~1mm variance |
| TF timestamp | Pre-capture fix | No significant improvement |
| Calibration roll/pitch | Zeroed small tilts | No improvement |
| Multi-capture averaging | 3 captures from same position | Only 9% improvement (expected 42%) |
| Camera settings | Various adjustments | No consistent improvement |
| Specific tags | Tag 2 vs Tag 9 | Similar variance on both |

---

## Current Detection Code

Located in: `src/beambot/beambot/camera/zivid.py`

```python
def detect_markers(
    client,
    node: Node,
    marker_ids: List[int],
    dictionary: str = "aruco4x4_50",
    timeout: float = 45.0,
    settle_time: float = 0.0
) -> DetectionResult:
    """Detect ArUco markers using Zivid's native detection."""

    # TIMESTAMP FIX: Capture timestamp BEFORE calling Zivid service
    pre_capture_stamp = node.get_clock().now().to_msg()

    # Call Zivid service
    request = CaptureAndDetectMarkers.Request()
    request.marker_ids = marker_ids
    request.marker_dictionary = dictionary

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)

    # Extract detected markers directly from Zivid's result
    detected = []
    for marker in result.detection_result.detected_markers:
        if marker.id in marker_ids:
            detected.append((marker.id, marker.pose))

    return DetectionResult(markers=detected, capture_stamp=pre_capture_stamp)
```

---

## Hypotheses for Future Investigation

### 1. Multi-Position Averaging (Option B)
Take captures from different robot positions to cancel perspective-dependent systematic errors.

**Approach:**
```
scan_pose_1 → Capture → (x₁, y₁, z₁)
scan_pose_2 → Capture → (x₂, y₂, z₂)
scan_pose_3 → Capture → (x₃, y₃, z₃)
Average → (x_avg, y_avg, z_avg)
```

Different viewing angles should have different systematic biases that might cancel out.

### 2. Investigate the "Old JSON" Test
The best result (0.658mm, Y-Z corr 0.744) came from a test with unknown configuration. Finding what was different could be the key.

Possible differences:
- Different scan position?
- Different lighting conditions?
- Different camera settings file?
- Robot in different state?

### 3. Depth Engine Settings
Zivid has multiple depth computation engines with different characteristics. Investigating engine-specific settings might reveal the source of variance.

### 4. Point Cloud Quality
Analyze the raw point cloud to see if the variance comes from:
- Depth noise at marker corners
- Point cloud holes/gaps
- Edge effects

### 5. Marker Quality
Physical marker quality could affect detection:
- Print quality
- Surface flatness
- Marker size
- Viewing angle limits

---

## Recommendations

### For Production Use
1. **Accept ~1mm variance** for applications where this is acceptable
2. **Add application-level retry logic** if a detection seems off
3. **Use relative positioning** when possible (detect, move, detect again)

### For Improved Accuracy
1. **Try multi-position averaging** (Option B) - different viewing angles
2. **Investigate Zivid camera settings** - depth engine, filters, etc.
3. **Consider alternative markers** - larger markers, different placement
4. **Profile the "old JSON" environment** if possible

---

## Files Reference

| File | Purpose |
|------|---------|
| `beambot/camera/zivid.py` | Detection implementation |
| `beambot/stages/vision_stages.py` | Vision stage orchestration |
| `cms_robot_description/urdf/zivid_camera_mount.xacro` | Hand-eye calibration |
| `beambot/config/default_beamline.yaml` | Camera configuration |

---

## Rosbag Archive

All test rosbags are stored in `recorded_bags/`:
- `2026_01_22_zividaruco` - Zivid native detection
- `2026_01_22_tffix` - TF timestamp fix test
- `2026_01_22_tf_opencv` - OpenCV + TF fix
- `2026_01_22_pitch_corrected` - Zero pitch/roll test
- `2026_01_22_tag2_reverted` - Original calibration restored
- `2026_01_22_tag2_zivid` - Zivid native on Tag 2
- `2026_01_22_tag2_zivid_new` - Second run same config
- `2026_01_22_tag2_zivid_multicapture` - 3-capture averaging

---

## Conclusion

The ~1mm Y-axis variance in ArUco detection appears to be **systematic** (high Y-Z correlation) rather than random noise. Standard noise reduction techniques (multi-capture averaging from same position) are ineffective.

The variance source remains unidentified. The best approach forward is either:
1. Accept the current accuracy level
2. Try multi-position averaging to cancel systematic perspective errors
3. Deep investigation into what made the "old JSON" test work so well
