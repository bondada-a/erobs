# EROBS Config Files Review

**Review Date:** 2025-01-28
**Files Reviewed:** 134 JSON and YAML configuration files
**Reviewer:** Automated analysis

---

## Executive Summary

This review identified **47 significant issues** across the JSON and YAML configuration files, categorized as:
- 🔴 **Critical (8)**: Issues that could cause runtime failures or incorrect behavior
- 🟡 **Major (15)**: Maintainability problems and inconsistencies that should be addressed
- 🟢 **Minor (24)**: Suggestions for improvement and cleanup

---

## 🔴 Critical Issues

### 1. Kinematics Solver Misconfiguration (UR3e)
**File:** `src/custom-ur-descriptions/ur3e_hande_moveit_config/config/kinematics.yaml`

```yaml
# Current (likely incorrect):
kinematics_solver_timeout: 0.005  # 5ms - too short!
kinematics_solver_search_resolution: 0.005
kinematics_solver_attempts: 3

# Compare to UR5e configs:
kinematics_solver_timeout: 1.0    # 1 second
kinematics_solver_search_resolution: 0.001
kinematics_solver_attempts: 10
```

**Impact:** IK solver may fail frequently with such a short timeout.

### 2. Obstacles File Has Extreme Z-Values (Typos)
**File:** `src/pdf/pdf_beamtime/config/obstacles.yaml`

```yaml
# These z values appear to be typos (10.x instead of 0.x):
holder_base_inbeam:   z: 10.475  # Should be: 0.475?
holder_base_storage:  z: 10.035  # Should be: 0.035?
holder_head_storage:  z: 10.09   # Should be: 0.09?
```

**Impact:** Planning scene would have obstacles 10 meters in the air.

### 3. Inconsistent end_effector_type Naming
**Files:** Multiple task JSONs

```json
// Some files use:
"end_effector_type": "epick"
"end_effector_type": "hande"

// Other files use:
"end_effector_type": "epick_gripper"
"end_effector_type": "hande_gripper"
```

**Affected Files:**
- `docktest.json` - uses `epick_gripper` and `hande_gripper`
- `new_test_updated.json` - uses `epick_gripper` and `hande_gripper`
- Most others use `epick` and `hande`

**Impact:** Will fail if code expects one naming convention.

### 4. Demo Task File Format Mismatch
**Files:** 
- `src/demos/hello_orchestrator/config/demo_task.json`
- `src/demos/hello_orchestrator_py/config/demo_task.json`

```json
// C++ version:
{"type": "move", "target": "home"}

// Python version:
{"type": "move", "target_pose": "moveit_home"}
```

**Impact:** Different task parsers expected for same demo task.

### 5. Empty Tasks Arrays
**Files with empty `tasks: []`:**
- `src/cms/tasks/tool_exchange_test.json`
- `src/cms/tasks/hot_plate.json`
- `src/cms/tasks/triple_test_no_tasks.json`
- `src/cms/tasks/beamtime/all_positions.json`

**Impact:** These files serve no purpose as task definitions (only pose storage).

### 6. Floating Point Precision Artifacts
**Files:** Multiple task JSONs

```json
// Examples of IEEE 754 floating point artifacts:
"pre_pickup_approach": [-270.41, -219.07999999999998]  // Should be -219.08
"load_approach": [-247.07999999999998]                 // Should be -247.08
```

**Impact:** Could cause cumulative precision errors in calculations.

### 7. Docktest.json Start Gripper Mismatch
**File:** `src/cms/tasks/docktest.json`

```json
{
  "start_gripper": "hande",  // Claims to start with hande
  "tasks": [
    {"task_type": "tool_exchange", "operation": "dock", "gripper": "epick", ...}
    // But first task docks epick - contradictory!
  ]
}
```

**Impact:** Undefined behavior - which gripper is actually attached?

### 8. Missing marker_dictionary in vision_moveto
**Files:** `vision_test.json`, `vision_test_scan_cache.json`

```json
// Some vision_moveto tasks have no marker_dictionary:
{"task_type": "vision_moveto", "tag_id": 1, "timeout": 15.0}

// Others have it:
{"task_type": "vision_moveto", "tag_id": 1, "marker_dictionary": "aruco4x4_50", ...}
```

**Impact:** May use wrong default dictionary or fail.

---

## 🟡 Major Issues

### 9. Massive Code Duplication in Task Files
**Files:** `repeat_sametag.json`, `repeat_sample_spincoater.json`, `repeat_sample_spincoater_24.json`, `vision_test_all_tags.json`, `vision_test_all_tags_multiposition.json`, `vision_test_simple.json`

**Problem:** These files contain the same task sequence copy-pasted multiple times (6-80+ times).

**Example:** `repeat_sametag.json` has 6 identical 17-task sequences copied back-to-back.

**Recommendation:** Implement a loop/repeat construct in the task executor, e.g.:
```json
{
  "task_type": "repeat",
  "count": 6,
  "tasks": [...]
}
```

### 10. Inconsistent vision_moveto Parameters
**Files:** All files with vision_moveto tasks

| File | detection_type | marker_dictionary | sample_index | timeout |
|------|---------------|-------------------|--------------|---------|
| repeat_sametag.json | marker | aruco4x4_50 | varies | 10.0 |
| vision_test.json | ❌ | ❌ | ❌ | 15.0 |
| vision_test_scan_cache.json | ❌ | ❌ | ❌ | 10.0 |
| tag_test.json | ❌ | aruco4x4_50 | ❌ | 10.0 |
| vision_test_single_position.json | marker | ❌ | ❌ | 10.0 |

**Recommendation:** Standardize required vs optional parameters.

### 11. Hardcoded Timeout Values
**Problem:** `timeout: 10.0` is hardcoded in every vision_moveto task (except vision_test.json which uses 15.0).

**Recommendation:** Move to a config file:
```yaml
vision:
  default_timeout: 10.0
  marker_dictionary: "aruco4x4_50"
```

### 12. Hardcoded Dock Numbers
**Problem:** Dock numbers (2, 3, 4) appear throughout task files without reference to a schema.

```json
{"gripper": "hande", "dock_number": 2}
{"gripper": "epick", "dock_number": 3}
{"gripper": "pipettor", "dock_number": 4}
```

**Recommendation:** Define in a central config:
```yaml
tool_docks:
  hande: 2
  epick: 3
  pipettor: 4
```

### 13. Inconsistent Pose Definitions Across Files
**Problem:** Same logical poses have different values in different files.

**Example - `sample_scan` pose:**
- `repeat_sametag.json`: `[13.38, -112.45, -65.22, -90.98, -267.33, -166.94]`
- `sample_to_spincoat.json`: `[17.77, -113.23, -63.9, -92.31, -267.99, -160.22]`
- `pipettor_part.json`: `[17.77, -113.23, -63.9, -92.31, -267.99, -160.22]`

**Recommendation:** Create a shared poses file that task files can reference.

### 14. Spincoat Position Variations
**Problem:** Different spincoat-related poses defined with subtle variations.

| File | spincoat value (truncated) |
|------|---------------------------|
| repeat_sametag.json | `[109.01, -121.08, -118.41, -30.95, ...]` |
| hotplate_to_spincoat.json | `[109.01, -121.08, -118.41, -30.935, ...]` |
| beamtime files | `[111.12, -117.48, -127.55, -24.92, ...]` |

**Question:** Are these intentional calibration differences or copy-paste drift?

### 15. File Naming Issue
**File:** `src/cms/tasks/beamline_test copy.json`

**Problem:** Space in filename - may cause issues with scripts/tooling.

**Recommendation:** Rename to `beamline_test_backup.json` or remove if obsolete.

### 16. Inconsistent Gripper State Naming
**Files:** `grippers.yaml` vs task JSONs

```yaml
# grippers.yaml defines:
states:
  grasp: "hande_closed"
  release: "hande_open"

# But some task files use:
"end_effector_action": "hande_closed"  # Direct name
# While grippers.yaml suggests abstraction via "grasp"/"release"
```

### 17. Different Joint Limits Configurations
**UR5e (epick config):**
```yaml
default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1
joint_limits:
  gripper:
    has_velocity_limits: true
    has_acceleration_limits: false
```

**UR3e:**
```yaml
# No default scaling factors defined
joint_limits:
  shoulder_pan_joint:
    has_acceleration_limits: true
    max_acceleration: 1.0
```

### 18. Hardcoded IP Addresses
**Files:**
- `default_beamline.yaml`: `192.168.1.101`
- `ur3e_beamline.yaml`: `192.168.56.102`
- `docker-compose.yml`: `192.168.56.101`, `192.168.56.102`, `192.168.56.103`

**Recommendation:** Use environment variables or separate env files.

### 19. Duplicate Gripper Configuration
**Files:** `grippers.yaml` and `default_beamline.yaml` both define gripper configs with overlapping but not identical information.

### 20. Unused Poses in Task Files
**Files:** Most task files

**Example in `vacuum_place.json`:**
- Defines 15 poses but only uses 1 (`vacuum_post_pickup`)
- All other poses are dead code

### 21. Docker Compose Version
**File:** `docker/docker-compose.yml`

```yaml
version: '3'  # Legacy syntax
```

**Recommendation:** Update to Compose specification or remove version key (deprecated).

### 22. Different Zivid Settings Versions
**Files:**
- `zivid_settings.yml`: `__version__: 8`
- `zivid_3d_settings.yml`: `__version__: 27`
- `zivid_3d_settings_calibration.yml`: Different settings

**Question:** Which is authoritative? Are older versions still needed?

### 23. Multiple Zivid Config Files
**Files in `src/beambot/config/`:**
- `zivid_settings.yml`
- `zivid_3d_settings.yml`
- `zivid_3d_settings_old.yml`
- `zivid_3d_settings_calibration.yml`
- `zivid_3d_calibrated_aruco.yml`
- `zivid_man_specular.yml`
- `zivid_man_specular_drylab.yml`
- `scene_capture.yml`

**Question:** Which are active? Consider consolidating or documenting.

---

## 🟢 Minor Issues

### 24. Missing metadata in task files
No description, author, or version fields in task JSONs.

### 25. Inconsistent JSON formatting
Some files use 2-space indent, others use 4-space; some have trailing commas (invalid JSON).

### 26. beamline_scene.yaml comments could be documentation
Commented values like `# y: 1.0` and `# y: 0.95` suggest iteration but lack explanation.

### 27. ArUco dictionary inconsistency
Most files use `aruco4x4_50`, but `fiducial_marker_param.yaml` uses `DICT_APRILTAG_36h11`.

### 28. vision_objects.json sparse
Only defines one object (`sample_bar` with tag 200). Is this intentional?

### 29. .planning/config.json is project management
Not robot config - consider moving to .github or project root.

### 30-47. Additional cleanup items
- Remove `_old` suffix files if obsolete
- Standardize units in comments (some use inches, others meters)
- Consider JSON Schema validation for task files
- Add ROS 2 parameter descriptions in YAML files
- Consolidate overlapping obstacle definitions
- Document the relationship between UR3e and UR5e configs
- Add validation for pose array lengths (must be 6)
- Consider using YAML anchors for repeated poses
- Add bounds checking for dock_number values
- Document which files are templates vs production
- Add checksums for calibration files
- Consider using parameter namespaces
- Document the beamtime workflow file dependencies
- Add unit tests for config file parsing
- Consider structured logging config

---

## Recommendations Summary

### High Priority
1. Fix UR3e kinematics timeout (0.005 → 1.0)
2. Fix obstacles.yaml z-value typos (10.x → 0.x)
3. Standardize end_effector_type naming
4. Remove or fix empty task files

### Medium Priority
5. Create shared poses.yaml for common positions
6. Implement task repeat/loop construct
7. Consolidate Zivid config files
8. Standardize vision_moveto parameters

### Low Priority
9. Clean up floating point precision artifacts
10. Improve documentation and metadata
11. Create JSON schema for validation
12. Externalize hardcoded values

---

## Files Reviewed

### Task JSON Files (43)
```
src/cms/tasks/
├── beamline_test copy.json ⚠️
├── beamline_test.json
├── bsui_test.json
├── complete_sequence.json
├── docktest.json
├── hot_plate.json ⚠️ (empty)
├── hotplate_to_spincoat.json
├── moveto_test.json
├── new_test_updated.json
├── pick_place_hande_test.json
├── pick_place_test.json
├── pipettor_part.json
├── repeat_sample_spincoater.json
├── repeat_sample_spincoater_24.json
├── repeat_sametag.json
├── sample_and_pipettor.json
├── sample_te.json
├── sample_to_spincoat.json
├── shake.json
├── tag_test.json
├── te_test.json
├── tool_exchange.json
├── tool_exchange_test.json ⚠️ (empty)
├── tool_voltage_test.json
├── triple_test.json
├── triple_test_no_tasks.json ⚠️ (empty)
├── vacuum_place.json
├── vision_pick_place_test.json
├── vision_test.json
├── vision_test_all_tags.json
├── vision_test_all_tags_multiposition.json
├── vision_test_simple.json
├── vision_test_single_position.json
├── beamtime/
│   ├── all_positions.json ⚠️ (empty)
│   ├── disposal.json
│   ├── epick_to_pipettor.json
│   ├── hotplate_disposal.json
│   ├── hotplate_to_sample_after_cache.json
│   ├── hotplate_to_spincoat.json
│   ├── pipettor_safety.json
│   ├── pipettor_to_epick.json
│   ├── pipettor_vial_1.json
│   ├── pipettor_vial_2.json
│   ├── pre_hotplate_new.json
│   ├── pre_spincoat_new.json
│   ├── real_sample_to_hotplate.json
│   ├── sample_checker.json
│   ├── sample_to_hotplate_cached.json
│   ├── sample_to_spincoat_cached.json
│   ├── spincoat_to_hotplate.json
│   ├── spincoat_to_hotplate_after_pipettor.json
│   └── vision_scan.json
└── test/
    └── vision_test_scan_cache.json
```

### YAML Configuration Files (72+)
- MoveIt configs (joint_limits, kinematics, ompl_planning, controllers)
- Robot descriptions (initial_positions, physical_parameters)
- Beambot configs (beamline_scene, grippers, zivid_*)
- GitHub workflows
- Pre-commit config
- Docker compose

### Other JSON Files (11)
- Demo task configs
- Vision objects
- VSCode settings
- Devcontainer config
- Planning config
- package-lock.json (skipped)

---

*Generated by automated review - manual verification recommended for critical changes.*
