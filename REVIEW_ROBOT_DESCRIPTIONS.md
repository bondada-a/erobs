# EROBS Robot Descriptions Review

**Review Date:** 2025-01-28  
**Reviewer:** Automated XACRO/URDF/SRDF Analysis

## Summary

This document provides a comprehensive review of all robot description files in the EROBS workspace, covering UR3e and UR5e configurations with various end effectors.

### Files Reviewed

| Type | Count | Location |
|------|-------|----------|
| XACRO | 22 | `src/custom-ur-descriptions/` |
| URDF | 8 | `src/custom-ur-descriptions/ur5e_robot_description/urdf/` |
| SRDF | 6 | Various MoveIt config directories |
| Mesh (STL) | 6 | Local packages |
| Mesh (DAE) | 6 | Local packages |

---

## 🔴 Critical Issues

### 1. **Incorrect Mass Value in UR3e HandE Physical Parameters**

**File:** `ur3e_hande_robot_description/config/hande/physical_parameters.yaml`

```yaml
hande_base_mass: 86387  # WRONG - appears to be in grams!
```

**Expected:** `0.86387` (kg) - This matches the robotiq_hande_description package value.

**Impact:** Incorrect mass will cause severe physics simulation issues in Gazebo/Isaac and incorrect dynamics calculations.

---

### 2. **Invalid Virtual Joint Configuration in UR3e SRDF**

**Files:** 
- `ur3e_hande_moveit_config/config/ur.srdf`
- `ur3e_hande_moveit_config/srdf/ur.srdf`

```xml
<virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="right_finger"/>
```

**Problem:** The `child_link` should be `base_link`, not `right_finger`. A virtual joint connecting the world frame to a finger link makes no kinematic sense.

**Should be:**
```xml
<virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="base_link"/>
```

---

### 3. **Hardcoded Robot IP Address**

**Files:**
- `ur5e_robot_description/urdf/ur_with_zivid_hande.urdf`
- `ur5e_robot_description/urdf/ur_with_zivid_hande_isaac.urdf`

```xml
<param name="socat_ip_address">192.168.1.101</param>
```

**Impact:** This will fail if robot has different IP. Should be parameterized via xacro args.

---

## 🟡 Consistency Issues

### 4. **Different Root Frame Names: UR3e vs UR5e**

| Robot | Root Frame |
|-------|------------|
| UR3e | `world` |
| UR5e | `map` |

**Impact:** TF tree inconsistency when running both robots or migrating code between them.

**Recommendation:** Standardize on one frame name (suggest `world` for robot-centric or `map` for navigation contexts).

---

### 5. **Duplicate SRDF Files for UR3e**

Two SRDF locations exist:
- `ur3e_hande_moveit_config/config/ur.srdf` (robot name: `ur3e`)
- `ur3e_hande_moveit_config/srdf/ur.srdf` (robot name: `ur`)

Only difference is the robot name attribute. The `srdf/` version appears to be orphaned.

**Recommendation:** Remove the duplicate `srdf/` directory and keep only `config/ur.srdf`.

---

### 6. **Inconsistent Initial Position Format**

**UR3e MoveIt config** uses flat structure:
```yaml
initial_positions:
  elbow_joint: 0
  shoulder_lift_joint: 0
```

**UR3e robot_description** uses different format without wrapper:
```yaml
shoulder_pan_joint: 0.0
shoulder_lift_joint: -1.57
```

This may cause confusion and potential loading issues.

---

### 7. **Different Initial Position Values**

| Config | shoulder_lift | elbow | Notes |
|--------|--------------|-------|-------|
| UR3e MoveIt | 0 | 0 | Stretched out |
| UR3e Description | -1.57 | 0 | More typical |
| UR5e (all) | -1.2132 | -1.4297 | Custom pose |

**Recommendation:** Ensure initial positions don't cause self-collision.

---

### 8. **keep_alive_count Mismatch**

XACRO files have default `keep_alive_count="10"` but generated URDFs have `keep_alive_count="2"`.

**Impact:** The URDFs appear to have been generated with non-default parameters. Regenerating with xacro defaults will change behavior.

---

## 🟢 External Package Dependencies

The following external packages are referenced but not in the workspace:

| Package | Purpose | Notes |
|---------|---------|-------|
| `ur_description` | UR arm meshes & configs | Standard UR ROS2 package |
| `zivid_description` | Zivid camera meshes | From zivid_ros |
| `robotiq_hande_description` | HandE gripper meshes | Robotiq ROS2 driver |
| `epick_description` | EPick vacuum gripper | Robotiq ROS2 driver |
| `pipette_description` | Pipettor tool | Custom package needed |

**All external packages must be installed** for the URDFs to load correctly. The local mesh files are OK.

---

## ✅ Local Mesh Files (All Present)

### ur3e_hande_robot_description
- ✅ `meshes/hande/visual/hande.dae`, `coupler.dae`, `finger_1.dae`, `finger_2.dae`
- ✅ `meshes/hande/collision/hande.stl`, `coupler.stl`, `finger_1.stl`, `finger_2.stl`
- ✅ `meshes/camera_mount/visual/azure_kinect.dae`, `ur3e_flange_camera_mount.dae`
- ✅ `meshes/camera_mount/collision/azure_kinect.stl`, `ur3e_flange_camera_mount.stl`

### ur5e_robot_description
- ✅ `meshes/zivid/zivid_custom_mount.dae`, `zivid_onarm_mount.stl`
- ✅ `meshes/tool_exchanger/Tool_block.stl`, `Tool_block_robotside.stl`

---

## Robot Configuration Details

### UR3e + HandE (with optional Azure Kinect camera)

**XACRO Files:**
| File | Purpose |
|------|---------|
| `ur_with_hande.xacro` | UR3e + HandE only |
| `ur_with_camera_hande.xacro` | UR3e + Camera Mount + HandE |
| `hande.xacro` | HandE gripper macro |
| `camera_mount.xacro` | Azure Kinect mount macro |

**Attachment Chain:**
```
world → base_link → [UR3e arm joints] → flange → mount → robotiq_coupler → hande_base → fingers
                                              ↘ camera_base
```

**Joint Types:**
- Finger joints marked as `type="fixed"` (should be `prismatic` for simulation)

---

### UR5e Configurations

**Four configurations available:**

| Config | End Effector | URDF Files |
|--------|-------------|------------|
| standalone | Tool exchanger robot-side only | `ur_standalone.urdf`, `ur_standalone_isaac.urdf` |
| zivid_hande | Zivid + Tool Block + HandE | `ur_with_zivid_hande.urdf`, `*_isaac.urdf` |
| zivid_epick | Zivid + Tool Block + EPick | `ur_with_zivid_epick.urdf`, `*_isaac.urdf` |
| zivid_pipettor | Zivid + Tool Block + Pipettor | `ur_with_zivid_pipettor.urdf`, `*_isaac.urdf` |

**Attachment Chain (common):**
```
map → base_link → [UR5e arm joints] → flange → tool0 → zivid_optical_frame (calibrated)
                                              ↘ zivid_arm_mount → zivid_base_link (visual)
                                                           ↘ tool_block → [end_effector]
```

**Key Design Choice:** Zivid optical frame uses hand-eye calibrated transform from tool0, separate from the visual mesh chain.

---

## MoveIt Configuration Review

### UR3e HandE MoveIt Config

**Groups:**
- `ur_arm`: 6 DOF arm joints
- `hand`: HandE links (no movable joints defined in ros2_control)

**Issues:**
- Missing gripper joint in ros2_control for simulation
- Virtual joint child_link is wrong (see Critical Issues)

### UR5e MoveIt Configs

All four configs share similar structure:

**Groups:**
- `ur_arm`: 6 DOF chain from base_link to flange
- End effector group varies:
  - `hande_gripper`: robotiq_hande_* links
  - `epick_gripper`: epick_* links (includes tool_block)
  - No gripper group for pipettor (static tool)

**ros2_control joints:**

| Config | Additional Joints |
|--------|------------------|
| standalone | None |
| hande | `robotiq_hande_left_finger_joint` |
| epick | `gripper` |
| pipettor | None (pipette_driver_node handles it) |

---

## Zivid Camera Calibration Notes

The `zivid_camera_mount.xacro` contains multiple calibration date entries:

```xml
<!-- Calibration (2026-01-15): Residuals: rot < 0.22°, trans < 0.47mm -->
<origin xyz="0.05675 0.10322 0.05489" rpy="-0.00615 0.04362 3.13541"/>

<!-- Previous calibrations (commented out) -->
<!-- Calibration (2025-12-17): ...
<!-- Calibration (2026-01-13) - had offset issues: ...
<!-- Old calibration (2025-10-09): ...
```

**Current active calibration:** 2026-01-15 (future date - likely typo, should be 2025)

---

## Recommendations

### High Priority

1. **Fix UR3e hande_base_mass:** Change from `86387` to `0.86387` kg
2. **Fix UR3e SRDF virtual_joint:** Change child_link from `right_finger` to `base_link`
3. **Parameterize socat_ip_address:** Use xacro arg with robot_ip default

### Medium Priority

4. **Standardize root frame name:** Choose `world` or `map` consistently
5. **Remove duplicate SRDF file:** Delete `ur3e_hande_moveit_config/srdf/` directory
6. **Make UR3e finger joints prismatic:** Currently `fixed`, should be `prismatic` for simulation

### Low Priority

7. **Align initial_positions format:** Use consistent YAML structure
8. **Fix calibration date typo:** "2026-01-15" → "2025-01-15"
9. **Document keep_alive_count rationale:** Why 10 in xacro but 2 in generated URDF?

---

## File Structure Overview

```
src/custom-ur-descriptions/
├── ur3e_hande_moveit_config/
│   ├── config/
│   │   ├── ur.srdf                 # MoveIt semantic description
│   │   ├── ur.urdf.xacro           # Imports robot description
│   │   ├── ur.ros2_control.xacro   # Mock hardware config
│   │   └── ...
│   └── srdf/
│       └── ur.srdf                 # DUPLICATE - remove
│
├── ur3e_hande_robot_description/
│   ├── config/
│   │   ├── hande/                  # Physical/kinematic params
│   │   ├── camera_mount/           # Camera mount params
│   │   └── initial_positions.yaml
│   ├── meshes/
│   │   ├── hande/                  # HandE meshes
│   │   └── camera_mount/           # Camera meshes
│   └── urdf/
│       ├── ur_with_hande.xacro     # Main robot
│       ├── ur_with_camera_hande.xacro
│       ├── hande.xacro             # Gripper macro
│       └── camera_mount.xacro      # Camera macro
│
├── ur5e_robot_description/
│   ├── meshes/
│   │   ├── zivid/                  # Zivid mount meshes
│   │   └── tool_exchanger/         # Tool block meshes
│   └── urdf/
│       ├── ur_standalone.xacro     # Base robot
│       ├── ur_with_zivid_*.xacro   # End effector configs
│       ├── zivid_camera_mount.xacro
│       ├── tool_block.xacro
│       ├── te_robotside.xacro
│       └── *.urdf                  # Generated URDFs
│
└── ur5e_moveit_configs/
    ├── ur_standalone_moveit_config/
    ├── ur_zivid_hande_moveit_config/
    ├── ur_zivid_epick_moveit_config/
    └── ur_zivid_pipettor_moveit_config/
```

---

## Appendix: Joint Limits (from URDF)

| Joint | Lower Limit | Upper Limit | Max Velocity | Max Effort |
|-------|-------------|-------------|--------------|------------|
| shoulder_pan_joint | -2π | 2π | π rad/s | 150 N·m |
| shoulder_lift_joint | -2π | 2π | π rad/s | 150 N·m |
| elbow_joint | -π | π | π rad/s | 150 N·m |
| wrist_1_joint | -2π | 2π | π rad/s | 28 N·m |
| wrist_2_joint | -2π | 2π | π rad/s | 28 N·m |
| wrist_3_joint | -2π | 2π | π rad/s | 28 N·m |
| robotiq_hande_*_finger_joint | 0 | 0.025 m | 0.15 m/s | 130 N |

---

*End of Review*
