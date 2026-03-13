# EROBS Hardware Capabilities Audit

**Date**: 2026-03-10 (reviewed 2026-03-12)
**Branch**: humble-experimental
**Purpose**: Thorough audit of what hardware CAN do that we are NOT using yet.

---

## Executive Summary

EROBS has made progress closing the loop on gripper feedback (ePick vacuum detection is now integrated), but several hardware capabilities remain untapped. The robot runs at a **fixed 20% speed** (intentional safety cap), has a **built-in force/torque sensor that is configured but not useful for current gram-scale samples**, and uses the Zivid camera for basic capture+detection while ignoring advanced features like region-of-interest filtering, diagnostics, and native resampling. The most impactful remaining capabilities are:

1. **~~ePick vacuum object detection~~** — **DONE.** MCP `get_vacuum_status()`, orchestrator watchdog with `VACUUM_LOST`, off→on retry cycle.
2. **Hand-E position feedback for grasp detection** — already in `/joint_states`, not yet integrated
3. **Force mode controller** — for compliant insertion (pipette tips), controller configured
4. **Zivid native resampling** — could replace DIY Open3D downsampling
5. **Zivid depth ROI filtering** — configured but disabled
6. **Iterative visual correction** — coarse-fine positioning for precision picks (software, tracked in STATUS.md)

## Review Notes (Mar 10-12, 2026)

### Priority Re-ranking After Review

The original audit ranked F/T sensor as P0. After review, priorities were adjusted based on current operational reality:

**F/T sensor de-prioritized (P0 → P3):** The samples handled at CMS beamline are very light (thin sample bars, slides, vials). The UR5e's F/T sensor has ~2N resolution (~200g minimum detectable weight) and 3-5N noise floor. Gram-scale samples are 40-100x below these thresholds. Not worth integrating right now. Revisit for force-controlled insertions into tight holders or heavier samples at other beamlines.

**ePick vacuum feedback — DONE:** The ePick is the primary gripper for sample handling at CMS. Fully integrated:
- MCP `get_vacuum_status()` tool + `get_robot_state()` includes vacuum status
- Orchestrator watchdog auto-aborts with `VACUUM_LOST` if object drops between steps
- Batching disabled for ePick to ensure per-step boundary checks
- Off→on retry cycle documented and working

The `ObjectDetectionStatus` publisher reports:
- `OBJECT_DETECTED_AT_MIN_PRESSURE` — holding at minimum vacuum
- `OBJECT_DETECTED_AT_MAX_PRESSURE` — strong seal, holding at max vacuum
- `NO_OBJECT_DETECTED` — no vacuum seal, nothing picked up
- `UNKNOWN` — indeterminate

**Grasp verification approach (ranked):**
1. ~~ePick vacuum feedback~~ — **DONE**
2. Hand-E finger position feedback (from `/joint_states`) — **next, P0**
3. Vision-based verification (iterative visual correction) — tracked in STATUS.md
4. F/T sensor — deprioritized (P3), not useful for current sample weights

---

## 1. UR5e Robot Arm

### Currently Used
- Joint trajectory execution via `scaled_joint_trajectory_controller` (primary)
- Joint state broadcasting at 100 Hz
- GPIO/IO controller for tool voltage switching
- Speed scaling state broadcaster
- TCP pose broadcaster
- Dynamic payload setting per gripper (mass + CoG via `set_payload` service)
- OMPL (RRTConnect), Cartesian path, and Pilz (LIN/PTP) motion planners
- Fixed 20% velocity/acceleration scaling for all operations

### Untapped Capabilities

#### 1.1 Force/Torque Sensor (built-in, 6-axis)
- **What**: The UR5e has a 6-axis F/T sensor in the tool flange. The `force_torque_sensor_broadcaster` is **already configured** in all controller YAMLs, publishing to topic `/ft_data` with frame `tool0`.
- **Current state**: Controller is loaded but data is **never subscribed to or used** by any action server or MCP tool.
- **Integration difficulty**: **LOW** — Add ROS2 subscription in MCP server or action stages. Data is already being published.
- **Scientific/operational value**: **HIGH**
  - Grasp verification: Check force > threshold after gripper close → confirms object held
  - Contact detection: Stop approach on force spike instead of hardcoded Z position
  - Collision detection: Detect unexpected contact during transport
  - Sample weight measurement: Read Z-axis force after grasp to verify correct sample
  - Insertion tasks: Force-controlled sample placement into holders
- **Priority**: **P3** (deprioritized). Sensor resolution (~2N) and noise floor (3-5N) too high for gram-scale samples. Revisit for force-controlled insertion tasks or heavier samples.

#### 1.2 Force Mode Controller
- **What**: `ur_controllers/ForceModeController` — UR's built-in compliant force control mode. Robot becomes "soft" in selected axes while maintaining position in others.
- **Current state**: **Configured in all controller YAMLs** but never activated or used.
- **Integration difficulty**: **MEDIUM** — Need new action server or task type to activate/deactivate force mode. Requires careful safety testing.
- **Scientific/operational value**: **HIGH**
  - Compliant sample insertion: Gently push sample into holder until seated (force limit)
  - Surface following: Trace sample surfaces for inspection
  - Assembly tasks: Insert pipette tips, dock tools with force feedback
  - Delicate manipulation: Handle fragile samples with force limits
- **Priority**: **P1**

#### 1.3 Freedrive / Teach Mode Controller
- **What**: `ur_controllers/FreedriveModeController` — Makes robot manually movable (zero-gravity mode). User physically guides the arm.
- **Current state**: **Configured in all controller YAMLs** but never activated.
- **Integration difficulty**: **LOW-MEDIUM** — Need service or action to toggle on/off. MCP tool to enable, then `save_pose()` to record position.
- **Scientific/operational value**: **MEDIUM-HIGH**
  - Rapid pose teaching: Physically guide robot to position, save via MCP `save_pose()`
  - Beamline setup: New beamline technicians can teach positions without programming
  - Calibration: Hand-guide camera to calibration targets
  - Recovery: Manually move robot out of collision states
- **Priority**: **P3** (deprioritized). Project direction is toward vision-driven positioning, not more hardcoded teach positions.

#### 1.4 Tool Contact Controller
- **What**: `ur_controllers/ToolContactController` — Detects contact events at the tool. Hardware-level contact detection at higher frequency than F/T threshold monitoring.
- **Current state**: **Configured** but never activated.
- **Integration difficulty**: **MEDIUM** — Need integration with approach/place stages.
- **Scientific/operational value**: **MEDIUM**
  - Automatic surface finding: Approach until contact, record height
  - Safer placement: Detect when sample touches surface
  - Collision safety: Hardware-speed contact interrupts
- **Priority**: **P3** (deprioritized). Force Mode (1.2) covers the functional need for contact-based tasks. Tool Contact is a niche safety interrupt — uses joint motor current, not F/T sensor. Works best with rigid-on-rigid contact, less reliable with compliant surfaces (e.g., ePick suction cup).

#### 1.5 Dynamic Speed/Acceleration Profiles
- **What**: Velocity and acceleration scaling are **hardcoded at 20%** in `base_stages.py` (lines 74-76). MoveIt supports per-stage scaling from 0-100%.
- **Current state**: Fixed at 20% for ALL moves — approach, transport, retreat, gripper.
- **Integration difficulty**: **LOW** — Add `velocity_scaling` and `acceleration_scaling` fields to task JSON. Pass through to planner factories.
- **Scientific/operational value**: **MEDIUM-HIGH**
  - Faster transport: 50-80% speed for long moves between stations (currently 5x slower than necessary)
  - Slower approach: 5-10% speed for final approach to delicate samples
  - Throughput: A beamline running 24/7 benefits significantly from faster transport moves
  - Per-task tuning: Different tasks have different speed requirements
- **Priority**: **P3** (deprioritized). 20% speed cap is an intentional safety choice, not a limitation. Higher speeds possible but not wanted for current operations.

#### 1.6 Passthrough Trajectory Controller
- **What**: `ur_controllers/PassthroughTrajectoryController` — Sends trajectories directly to UR controller, bypassing ROS2 interpolation. Lower latency, smoother execution.
- **Current state**: **Configured** but never used.
- **Integration difficulty**: **MEDIUM** — Would need to replace `scaled_joint_trajectory_controller` for specific tasks.
- **Scientific/operational value**: **LOW-MEDIUM**
  - Smoother motions for sensitive operations
  - Lower jitter during liquid handling
- **Priority**: **P4** (deprioritized). No observed jitter issues with current controller.

#### 1.7 Forward Velocity Controller
- **What**: `velocity_controllers/JointGroupVelocityController` — Direct velocity control of joints.
- **Current state**: **Configured** but never used.
- **Integration difficulty**: **HIGH** — Velocity control requires real-time safety monitoring. Not suitable for MTC pipeline.
- **Scientific/operational value**: **LOW** for current use cases.
- **Priority**: **P4** — Only relevant if implementing visual servoing or continuous tracking.

#### 1.8 Safety Limits (URDF)
- **What**: UR5e supports software safety limits (position margins, compliance gains). Currently **disabled** in URDF (`safety_limits: false`).
- **Current state**: `safety_pos_margin: 0.15`, `safety_k_position: 20` configured but inactive.
- **Integration difficulty**: **LOW** — Set `safety_limits: true` in xacro.
- **Scientific/operational value**: **MEDIUM**
  - Workspace boundary enforcement
  - Prevent joint limit collisions
  - Required for unattended 24/7 operation
- **Priority**: **P2**

---

## 2. Zivid 2+ M60 Camera (3D Structured Light)

### Currently Used
- Single-shot 3D capture (dual-exposure HDR: 1ms + 10ms)
- RGB 2D image capture (2448×2048)
- XYZRGBA organized point cloud
- ArUco marker detection via native `CaptureAndDetectMarkers` service
- OpenCV-based detection: Hough circles, contour, HSV color
- Point cloud 3D lookup (pixel → XYZ)
- 10 processing filters (noise, outlier, reflection, hole repair, etc.)
- Multi-position scan averaging (vision_scan task)
- Downsampled point cloud relay for octomap (5M → 10k via Open3D voxel grid)

### Untapped Capabilities

#### 2.1 Region of Interest (Depth + Box)
- **What**: Crop point cloud to a depth range or 3D bounding box BEFORE processing. Currently **configured but disabled** (`Enabled: no`, range set to 300-1100mm).
- **Current state**: Full FOV captured every time, wasting bandwidth and processing time.
- **Integration difficulty**: **LOW** — Change `Enabled: no` to `yes` in `scene_capture.yml`, tune range per beamline.
- **Scientific/operational value**: **MEDIUM**
  - Eliminate background noise (table, walls, floor)
  - Faster point cloud transmission (less data)
  - Cleaner octomap (no phantom obstacles from distant surfaces)
  - Better detection accuracy (no false positives from background)
- **Priority**: **P2**

#### 2.2 Diagnostics Mode
- **What**: Zivid SDK can output capture diagnostics (exposure analysis, SNR maps, quality metrics). Currently **disabled** (`Diagnostics: Enabled: no`).
- **Current state**: No quality feedback on captures.
- **Integration difficulty**: **LOW** — Set `Diagnostics: Enabled: yes` in config.
- **Scientific/operational value**: **MEDIUM**
  - Detect degraded capture quality (dirty lens, bad lighting)
  - Alert when SNR drops below threshold
  - Quality metrics for autonomous operation monitoring
- **Priority**: **P3**

#### 2.3 Resampling Mode
- **What**: Resample point cloud to different density. Currently **disabled** (`Resampling: Mode: disabled`).
- **Current state**: Full-resolution point cloud always generated (~5M points).
- **Integration difficulty**: **LOW** — Change `Mode: disabled` to desired mode in config.
- **Scientific/operational value**: **LOW-MEDIUM**
  - Faster processing with reduced-density clouds
  - Would replace the Open3D voxel downsampling in `pointcloud_relay.py` — prefer native SDK over DIY
- **Priority**: **P2** (bumped up). Config toggle that could simplify the pipeline and remove custom code.

#### 2.4 Contrast Distortion Correction
- **What**: Corrects systematic errors from structured light pattern on shiny surfaces. Currently **partially enabled** (Removal: yes, Correction: no).
- **Current state**: Removal is on but active correction is disabled.
- **Integration difficulty**: **LOW** — Toggle `Correction: Enabled: yes` in config.
- **Scientific/operational value**: **MEDIUM** for shiny/metallic samples
  - Better 3D accuracy on reflective sample holders
  - More accurate depth on glossy surfaces
  - Important for metal sample trays, glass vials
- **Priority**: **P2** (beamline-dependent)

#### 2.5 Noise Suppression Filter
- **What**: Advanced noise suppression. Currently **disabled** (`Suppression: Enabled: no`).
- **Current state**: Only basic noise removal threshold active.
- **Integration difficulty**: **LOW** — Toggle in config.
- **Scientific/operational value**: **LOW-MEDIUM**
  - Cleaner point clouds in noisy environments
  - May improve detection on dark/absorptive surfaces
- **Priority**: **P3**

#### 2.6 Dynamic Settings Adjustment
- **What**: Zivid settings are loaded from YAML files at launch time and never changed. The SDK supports runtime settings changes.
- **Current state**: Fixed 2-acquisition HDR for all captures, regardless of scene.
- **Integration difficulty**: **MEDIUM** — Would need MCP tool or parameter to switch settings profiles (fast single-exposure vs. high-quality HDR).
- **Scientific/operational value**: **MEDIUM**
  - Fast mode (1 acquisition, ~200ms) for position verification
  - High-quality mode (3+ acquisitions) for precision detection
  - Scene-adaptive exposure (bright vs. dark samples)
- **Priority**: **P2**

#### 2.7 2D-Only Capture (Projector Off)
- **What**: Capture 2D RGB image WITHOUT firing structured light projector. Currently every capture fires the projector (3D mode).
- **Current state**: Even `mode="2d"` in MCP triggers the full 3D capture cycle due to how the driver is configured.
- **Integration difficulty**: **MEDIUM** — Need separate 2D-only capture service/topic or settings profile with `Brightness: 0` (projector off).
- **Scientific/operational value**: **MEDIUM**
  - Faster captures for color-only detection (HSV, ArUco from 2D)
  - No projector interference with beamline optics
  - Lower power consumption for rapid scanning
- **Priority**: **P3**

#### 2.8 Projector-Only Illumination
- **What**: Use Zivid's built-in projector as a controllable light source (without 3D capture). Brightness is adjustable 0-2.5.
- **Current state**: Projector only fires during 3D capture.
- **Integration difficulty**: **MEDIUM-HIGH** — Requires Zivid SDK API calls not exposed via ROS2.
- **Scientific/operational value**: **LOW-MEDIUM**
  - Consistent illumination for 2D detection (no ambient light dependency)
  - Could improve circle/contour detection reliability
- **Priority**: **P4**

#### 2.9 Hand-Eye Calibration Verification
- **What**: Zivid SDK includes tools to verify hand-eye calibration accuracy.
- **Current state**: Calibration done manually via Zivid Studio. No automated verification.
- **Integration difficulty**: **MEDIUM** — Would need periodic calibration check routine.
- **Scientific/operational value**: **MEDIUM**
  - Detect calibration drift before it causes pick failures
  - Automated accuracy monitoring for 24/7 operation
- **Priority**: **P3**

---

## 3. ZED 2i Camera (Stereo, External Mount)

### Currently Used
- Continuous RGB streaming to `/zed/image_color`
- Configured as `role: "overview"` in beamline config
- Basic `capture_image(camera="zed")` via MCP (subscribe to existing stream)

### Untapped Capabilities

#### 3.1 Stereo Depth / Point Cloud
- **What**: ZED 2i produces real-time depth maps and point clouds. Not currently used in any action server or MCP tool.
- **Integration difficulty**: **LOW** — Subscribe to depth/cloud topics.
- **Scientific/operational value**: **MEDIUM**
  - Workspace monitoring from external viewpoint
  - Detect obstacles not visible to eye-in-hand Zivid
  - Human detection in workspace for safety
- **Priority**: **P3** (deprioritized). ZED role still maturing — unclear how it fits in the pipeline yet.

#### 3.2 Object Detection / Body Tracking
- **What**: ZED SDK includes built-in object detection and body/skeleton tracking.
- **Integration difficulty**: **MEDIUM-HIGH** — Need ZED SDK node with detection enabled.
- **Scientific/operational value**: **MEDIUM**
  - Human presence detection for safety interlocks
  - Workspace occupancy monitoring
- **Priority**: **P3**

#### 3.3 Spatial Mapping
- **What**: ZED SDK can build 3D spatial maps of the environment.
- **Integration difficulty**: **MEDIUM**
- **Scientific/operational value**: **LOW** — Zivid provides higher-accuracy 3D for close-range.
- **Priority**: **P4**

---

## 4. Robotiq Hand-E Gripper

### Currently Used
- Binary open/close via MoveIt SRDF states (`hande_open`=0.025m, `hande_closed`=0.0m)
- GripperActionController with stall detection configured
- Open-loop control (command sent, assume completion)

### Untapped Capabilities

#### 4.1 Position Feedback for Grasp Detection
- **What**: Hand-E finger position is published to `/joint_states` as `robotiq_hande_left_finger_joint`. After closing, position reveals:
  - Fully closed (0.0m) = missed/empty grasp
  - Partially closed (0.005-0.024m) = holding object, width indicates object size
- **Current state**: Data is being published but **never read or checked**.
- **Integration difficulty**: **LOW** — Subscribe to `/joint_states`, check finger position after close command.
- **Scientific/operational value**: **HIGH**
  - Immediate grasp verification: Did we actually pick up the sample?
  - Object identification: Different objects have different widths
  - Retry logic: If grasp failed, adjust position and retry
  - Prevents transport of "nothing" to place location
- **Priority**: **P0 — Highest**. Data already available, zero hardware work.

#### 4.2 Stall Detection / Grip Confirmation
- **What**: GripperActionController is configured with:
  - `allow_stalling: true`
  - `stall_velocity_threshold: 0.001 m/s`
  - `stall_timeout: 1.0s`
  - Motor stall = object detected in gripper
- **Current state**: Controller handles stalling internally but **stall event is not propagated** to higher-level logic.
- **Integration difficulty**: **LOW-MEDIUM** — Check action result for stall indication.
- **Scientific/operational value**: **HIGH** (same as 4.1)
- **Priority**: **P0** (combined with 4.1)

#### 4.3 Variable Position Control
- **What**: Hand-E supports continuous position control (0-50mm stroke), not just binary open/close.
- **Current state**: Only two SRDF states defined (fully open, fully closed).
- **Integration difficulty**: **LOW** — Add more SRDF states or use direct position commands.
- **Scientific/operational value**: **MEDIUM**
  - Pre-shape gripper to sample width (faster pick)
  - Gentle grip with known force (partially close for delicate samples)
  - Custom grip widths for different sample types
- **Priority**: **P2**

#### 4.4 Speed Control
- **What**: Hand-E supports variable gripper speed (0-150mm/s via Modbus).
- **Current state**: Speed not exposed through ROS2 interface.
- **Integration difficulty**: **MEDIUM** — May need driver modification to expose speed parameter.
- **Scientific/operational value**: **LOW-MEDIUM**
  - Slower close for delicate samples
  - Faster close for robust objects
- **Priority**: **P3**

---

## 5. Robotiq ePick Vacuum Gripper

### Currently Used
- Binary vacuum on/off via MoveIt SRDF states (`vacuum_on`=1, `vacuum_off`=0)
- Open-loop control (vacuum command sent, assume suction achieved)

### Untapped Capabilities

#### 5.1 Vacuum Level Feedback — ✅ DONE
- **What**: ePick hardware reports vacuum pressure level via `ObjectDetectionStatus` publisher on `/object_detection_status`.
- **Status**: **Fully integrated** (completed 2026-03-12):
  - MCP `get_vacuum_status()` tool + `get_robot_state()` includes vacuum status
  - Orchestrator watchdog auto-aborts with `VACUUM_LOST` if object drops between steps
  - Batching disabled for ePick to ensure per-step boundary checks
  - Off→on retry cycle for failed picks (ePick hardware quirk)
- **Priority**: **DONE**

#### 5.2 Vacuum Level Threshold (Adaptive Suction)
- **What**: ePick supports configurable vacuum threshold for grip detection.
- **Current state**: Not configured.
- **Integration difficulty**: **MEDIUM** — Need driver parameter configuration.
- **Scientific/operational value**: **MEDIUM**
  - Different thresholds for porous vs. non-porous surfaces
  - Auto-detect grip quality
- **Priority**: **P3** (deprioritized). Current binary detection (object detected / not detected) works for current samples. Revisit if false negatives appear with different surfaces.

#### 5.3 Suction Cup Pressure Monitoring
- **What**: Real-time pressure monitoring for continuous feedback.
- **Current state**: Not implemented.
- **Integration difficulty**: **MEDIUM**
- **Scientific/operational value**: **MEDIUM**
  - Continuous monitoring during transport
  - Emergency stop on vacuum loss
- **Priority**: **P3** (deprioritized). Per-step boundary check via orchestrator watchdog already covers the practical risk.

---

## 6. Pipettor

### Currently Used
- Four operations: SUCK (aspirate), EXPEL (dispense), EJECT_TIP, SET_LED
- Volume specified as percentage (0.0-1.0)
- LED color control
- Open-loop volume control

### Untapped Capabilities

#### 6.1 Volume Feedback / Measurement
- **What**: Pipettor hardware may track actual aspirated/dispensed volume.
- **Current state**: Volume is commanded as percentage, no feedback on actual volume.
- **Integration difficulty**: **MEDIUM** — Depends on pipettor driver API (external package: sixym3/pipettor).
- **Scientific/operational value**: **HIGH** for liquid handling accuracy
  - Verify correct volume aspirated
  - Detect clogged or leaking tips
  - Precision dispensing with feedback
- **Priority**: **P4** (hardware doesn't support volume feedback)

#### 6.2 Pressure/Flow Monitoring
- **What**: Advanced pipettors monitor aspiration pressure to detect:
  - Air bubbles (pressure irregularity)
  - Clogged tips (high pressure)
  - Tip collision (sudden pressure spike)
  - Empty well (no liquid aspirated)
- **Current state**: Not implemented.
- **Integration difficulty**: **MEDIUM-HIGH** — Depends on hardware capabilities.
- **Scientific/operational value**: **HIGH** for scientific accuracy
  - Critical for liquid handling reliability
  - Prevents wrong experiment results from failed aspiration
- **Priority**: **P4** (hardware doesn't support pressure/flow monitoring)

#### 6.3 Tip Detection / Verification
- **What**: Verify tip is properly seated before aspiration.
- **Current state**: Tip ejection is blind command, no verification.
- **Integration difficulty**: **MEDIUM**
- **Scientific/operational value**: **MEDIUM**
  - Prevent aspiration without tip (contaminates pipettor)
  - Verify tip ejected successfully
- **Priority**: **P4** (hardware doesn't support tip detection)

---

## 7. System-Level / Cross-Component Capabilities

### 7.1 Closed-Loop Grasp Verification Pipeline — PARTIAL
- **What**: Combine gripper feedback + visual re-check for reliable grasping.
- **Current state**: ePick vacuum verification **DONE**. Hand-E position check still open.
- **Implemented flow (ePick)**:
  ```
  approach → close → vacuum_on → CHECK vacuum status → transport → CHECK vacuum (per-step watchdog) → place
  ```
- **Remaining work**:
  - Hand-E finger position check after close (P0, items 4.1/4.2)
  - F/T sensor check deprioritized (P3, gram-scale samples)
- **Integration difficulty**: **LOW** for remaining Hand-E work — pattern established by ePick integration.
- **Scientific/operational value**: **VERY HIGH**
- **Priority**: **PARTIAL DONE** — remaining work is Hand-E feedback (P0)

### 7.2 Sensor Data via MCP — PARTIAL
- **What**: Expose hardware sensor data as MCP tools for LLM-driven verification.
- **MCP tools status**:
  ```
  get_vacuum_status()   → ePick vacuum state          ✅ DONE
  get_robot_state()     → Gripper info + vacuum        ✅ DONE
  get_ft_reading()      → Current force/torque values  ❌ Deprioritized (P3)
  get_gripper_state()   → Hand-E position/stall        ❌ Pending (depends on 4.1/4.2)
  check_grasp()         → Combined verification        ❌ Pending (depends on above)
  ```
- **Integration difficulty**: **LOW** for remaining Hand-E tools — pattern established by vacuum tools.
- **Scientific/operational value**: **VERY HIGH** — Enables LLM to verify and recover from failures.
- **Priority**: **PARTIAL DONE** — remaining work ties to Hand-E integration (P0)

### 7.3 Dynamic Speed for Task Phases
- **What**: Allow different speed profiles for different task phases within a single operation.
- **Proposed extension to task JSON**:
  ```json
  {
    "task_type": "moveto",
    "target": "scan_position",
    "velocity_scaling": 0.5,
    "acceleration_scaling": 0.5
  }
  ```
- **Integration difficulty**: **LOW** — Pass through to planner factories in base_stages.py.
- **Scientific/operational value**: **MEDIUM-HIGH**
  - 2-5x faster transport moves
  - Safer approach/retreat at lower speeds
  - Overall throughput improvement for 24/7 operation
- **Priority**: **P3** (deprioritized). 20% speed cap is intentional safety choice.

### 7.4 Safety Monitoring Service
- **What**: Combine F/T sensor, speed scaling, and workspace boundaries for autonomous safety.
- **Components**:
  - Enable URDF safety limits
  - F/T threshold monitoring → emergency stop on unexpected contact
  - Workspace boundary planes (prevent robot from hitting beamline equipment)
  - Speed reduction near boundaries
- **Integration difficulty**: **MEDIUM-HIGH**
- **Scientific/operational value**: **HIGH** for unattended operation
- **Priority**: **P3** (deprioritized). Multi-week effort with many dependencies (F/T deprioritized, safety limits P2, speed profiles P3).

---

## Priority Summary

### DONE
| Capability | Component | Notes |
|---|---|---|
| ePick vacuum level feedback | ePick | MCP `get_vacuum_status()`, orchestrator watchdog, off→on retry |
| MCP sensor tools (vacuum) | System | `get_vacuum_status()`, `get_robot_state()` includes vacuum |
| Closed-loop pipeline (ePick) | System | Per-step vacuum verification with `VACUUM_LOST` abort |

### P0 — Immediate, High Impact, Low Effort
| Capability | Component | Effort | Impact |
|---|---|---|---|
| Hand-E position feedback | Hand-E | 1-2 hours | Grasp detection (holding vs. empty) |
| Hand-E stall detection | Hand-E | (combine with above) | Grasp confirmation |
| MCP sensor tools (Hand-E) | System | 2-4 hours | `get_gripper_state()`, `check_grasp()` |

### P1 — High Value, Moderate Effort
| Capability | Component | Effort | Impact |
|---|---|---|---|
| Force mode controller | UR5e | 1-2 days | Compliant insertion (pipette tips) |

### P2 — Valuable, Some Effort
| Capability | Component | Effort | Impact |
|---|---|---|---|
| Zivid depth ROI | Zivid | 30 min | Cleaner captures, less noise |
| Zivid resampling (native) | Zivid | 30 min | Replace DIY Open3D downsampling |
| Contrast distortion correction | Zivid | 30 min | Better accuracy on shiny surfaces |
| Dynamic capture settings | Zivid | 4-8 hours | Scene-adaptive capture quality |
| Variable gripper position | Hand-E | 2-4 hours | Custom grip widths |
| Safety limits (URDF) | UR5e | 1 hour | Joint limit protection |

### P3 — Nice to Have
| Capability | Component | Effort | Impact |
|---|---|---|---|
| F/T sensor subscription | UR5e | 2-4 hours | Only for insertion tasks (poor SNR for light samples) |
| Freedrive/teach mode | UR5e | 4-8 hours | Rapid pose teaching |
| Tool contact controller | UR5e | 4-8 hours | Hardware contact detection |
| Dynamic speed profiles | UR5e | 2-4 hours | Speed cap is intentional safety choice |
| Zivid diagnostics mode | Zivid | 30 min | Capture quality monitoring |
| Noise suppression filter | Zivid | 30 min | Cleaner point clouds |
| 2D-only capture (projector off) | Zivid | 4-8 hours | Faster color-only detection |
| Hand-eye calibration verification | Zivid | 1-2 days | Automated accuracy monitoring |
| Hand-E speed control | Hand-E | 4-8 hours | Gripper speed tuning |
| ePick adaptive thresholds | ePick | 2-4 hours | Surface-specific vacuum |
| ePick pressure monitoring | ePick | 2-4 hours | Continuous transport monitoring |
| ZED stereo depth | ZED | 2-4 hours | External depth monitoring |
| ZED object detection | ZED | 1-2 days | Human presence detection |
| Dynamic speed (task phases) | System | 2-4 hours | Per-task speed tuning |
| Safety monitoring service | System | 2-3 days | Multi-dependency, long-term |

### P4 — Future / Low Priority
| Capability | Component | Effort | Impact |
|---|---|---|---|
| Passthrough trajectory controller | UR5e | 1-2 days | Smoother motions (no current issues) |
| Forward velocity controller | UR5e | 2-3 days | Visual servoing (not needed — iterative correction preferred) |
| Projector-only illumination | Zivid | 1-2 days | Consistent lighting |
| ZED spatial mapping | ZED | 1-2 days | Environment mapping |
| Pipettor volume feedback | Pipettor | N/A | Hardware doesn't support |
| Pipettor pressure monitoring | Pipettor | N/A | Hardware doesn't support |
| Pipettor tip detection | Pipettor | N/A | Hardware doesn't support |

---

## Implementation Recommendations

### Phase 1: Hand-E Feedback (P0, ~1 week)
ePick vacuum is done. Complete the grasp verification loop for Hand-E:

1. **MCP server additions** (`beambot_mcp_server.py`):
   - Subscribe to `/joint_states` — extract `robotiq_hande_left_finger_joint` position
   - Check GripperActionController result for stall indication
   - New tools: `get_gripper_state()`, `check_grasp()`

2. **Orchestrator/stage additions**:
   - Post-grasp verification checkpoint in `pick_place_stages.py`
   - Read finger position after close → report in action result
   - Fully closed (0.0m) = empty grasp, partially closed = holding

### Phase 2: Force Mode for Pipette Tips (P1, 1-2 weeks)
1. Controller switching logic in orchestrator (scaled_joint_trajectory ↔ force_mode)
2. New task type or stage for compliant insertion
3. Safety testing with force limits
4. Alternative: MoveIt Servo approach (avoids controller switching)

### Phase 3: Vision Enhancement (P2, 1 week)
1. Enable depth ROI filtering in `scene_capture.yml`
2. Enable Zivid native resampling (replace Open3D in `pointcloud_relay.py`)
3. Enable contrast distortion correction
4. Dynamic settings profiles (fast vs. quality)

### Phase 4: Safety & Deployment (P2, when approaching 24/7)
1. Enable URDF safety limits (tune margins for workspace)
2. Hand-eye calibration verification routine
3. Safety monitoring service (long-term)

---

## File Reference

| Category | Key Files |
|---|---|
| **Controller configs** | `src/custom-ur-descriptions/ur5e_moveit_configs/*/config/ur*_controllers.yaml` |
| **Speed/accel scaling** | `src/beambot/beambot/stages/base_stages.py` (lines 74-76) |
| **Zivid 3D settings** | `src/beambot/config/scene_capture.yml` |
| **Zivid 2D settings** | `src/beambot/config/zivid_settings.yml` |
| **MCP server** | `src/beambot/mcp/beambot_mcp_server.py` |
| **Orchestrator** | `src/beambot/beambot/action_servers/orchestrator.py` |
| **Pick/place stages** | `src/beambot/beambot/stages/pick_place_stages.py` |
| **End effector stages** | `src/beambot/beambot/stages/end_effector_stages.py` |
| **Gripper config** | `src/beambot/config/default_beamline.yaml` |
| **URDF/xacro** | `src/custom-ur-descriptions/ur5e_robot_description/urdf/` |
| **Action definitions** | `src/beambot_interfaces/action/` |
| **ePick driver** | External: `PickNikRobotics/ros2_epick_gripper` (via `end_effectors.repos`) |
| **HandE driver** | External: `AGH-CEAI/robotiq_hande_driver` (via `end_effectors.repos`) |
| **Pipettor driver** | External: `sixym3/pipettor` (via `end_effectors.repos`) |
