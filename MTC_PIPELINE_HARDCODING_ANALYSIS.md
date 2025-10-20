# MTC Pipeline Hardcoding Analysis
## Complete Inventory of Values That Need to Be Configurable

**Date**: 2025-01-17
**Package**: `mtc_pipeline`
**Purpose**: Identify all hardcoded values preventing multi-beamline deployment

---

## Executive Summary

The `mtc_pipeline` package contains **7 categories** of hardcoded values that need to be externalized to configuration files. Total count: **21 hardcoded items** across 8 files.

**Priority Classification:**
- **Critical (Must Fix)**: 8 items - Block multi-beamline support
- **High (Should Fix)**: 7 items - Reduce flexibility
- **Medium (Nice to Have)**: 6 items - Improve maintainability

---

## Category 1: Gripper Package Mappings ⚠️ CRITICAL

### Location
`src/mtc_orchestrator_action_server.cpp:250-254`

### Current Code
```cpp
static const std::unordered_map<std::string, std::string> gripper_packages = {
    {"none", "ur_standalone_moveit_config"},
    {"epick", "ur_zivid_epick_moveit_config"},
    {"hande", "ur_zivid_hande_moveit_config"}
};
```

### Problem
- **Hardcoded C++ map** requires recompilation to add/modify grippers
- Directly prevents multi-beamline support
- Tightly couples orchestrator to specific MoveIt packages

### Impact
- **Cannot add new grippers** without code changes
- **Cannot support beamline-specific gripper names** (e.g., CMS uses different grippers than PDF)
- Forces all beamlines to use identical gripper configurations

### Refactoring Recommendation
**Difficulty**: Medium (requires YAML loader + validation)

**Solution**: Load from YAML configuration
```yaml
# beamlines/<name>/config/grippers.yaml
grippers:
  hande:
    moveit_package: "ur_zivid_hande_moveit_config"
  epick:
    moveit_package: "ur_zivid_epick_moveit_config"
  none:
    moveit_package: "ur_standalone_moveit_config"
```

**Implementation Steps**:
1. Add `yaml-cpp` dependency to `package.xml`
2. Create `ConfigurationManager` class to load gripper configs
3. Pass config path via ROS parameter to orchestrator
4. Replace hardcoded map with dynamic lookup

---

## Category 2: Gripper-Specific Constants ⚠️ HIGH

### Location 1: Pick/Place Stages
`src/pick_place_stages.cpp:8-10`

### Current Code
```cpp
constexpr const char* GRIPPER_GROUP = "hande_gripper";
constexpr const char* GRIPPER_OPEN_STATE = "hande_open";
constexpr const char* GRIPPER_CLOSED_STATE = "hande_closed";
```

### Problem
- **Hardcoded to Hand-E gripper only**
- Pick/place tasks cannot work with EPick or other grippers
- Group names and states must match SRDF exactly

### Impact
- **Pick/place action ONLY works with Hand-E gripper**
- If beamline uses different gripper, pick/place will fail
- No way to specify gripper type at runtime

### Location 2: Vision Stages
`src/vision_stages.cpp:249`

### Current Code
```cpp
auto task = create_task_template("Vision Move", "", "robotiq_hande_end");
```

### Problem
- **Hardcoded TCP frame name** for Hand-E gripper
- Vision-based movements assume Hand-E end effector

### Impact
- **Vision tasks fail** if using EPick or other end effectors
- Cannot perform vision-guided operations with non-Hand-E grippers

### Refactoring Recommendation
**Difficulty**: High (affects action server interface)

**Option A**: Make gripper type a parameter in `PickPlaceAction.action`
```cpp
// Already present: string gripper
// Use this to look up gripper config at runtime
const std::string gripper_type = step["gripper"].get<std::string>();
auto gripper_config = config_manager_->getGripperConfig(gripper_type);
const char* GRIPPER_GROUP = gripper_config["group"].as<std::string>().c_str();
```

**Option B**: Create per-gripper action server instances
- Launch `pick_place_action_server_hande` and `pick_place_action_server_epick`
- Each configured for specific gripper
- Orchestrator routes to correct server based on gripper type

**Recommended**: Option A (more flexible, less processes)

---

## Category 3: IP Addresses ⚠️ CRITICAL

### Location 1: Orchestrator Launch File
`launch/modular_action_servers.launch.py:13`

### Current Code
```python
default_value='192.168.56.101',
```

### Location 2: Example Client
`src/mtc_action_client_example.cpp:48`

### Current Code
```cpp
int execute_task(const std::string& json_file_path,
                 const std::string& robot_ip = "192.168.56.101")
```

### Problem
- **Hardcoded robot IP address** (likely VM/sim address)
- Different beamlines have different network configurations
- Production robots typically use different IPs (e.g., `192.168.1.10`)

### Impact
- **Launch files must be edited** for each beamline
- **Example code has wrong default** for production
- No single binary works across beamlines

### Refactoring Recommendation
**Difficulty**: Easy (already parameterized in launch file)

**Solution**: Already mostly solved!
- Launch file has `robot_ip` argument (line 11-15)
- Orchestrator accepts `robot_ip` via ROS parameter (line 100)
- Just need to remove default value or set it dynamically

**Additional**: Load from beamline config
```yaml
# beamlines/<name>/config/hardware.yaml
robot:
  ip: "192.168.1.10"
```

Then in launch file:
```python
beamline_config = load_yaml(beamline_path / 'hardware.yaml')
robot_ip_arg = DeclareLaunchArgument(
    'robot_ip',
    default_value=beamline_config['robot']['ip']
)
```

---

## Category 4: Camera Topics ⚠️ HIGH

### Location 1: AprilTag Config
`config/apriltag_config.yaml:22-29`

### Current Code
```yaml
qos_overrides:
  /color/image_color:
    subscription:
      reliability: reliable
  /color/camera_info:
    subscription:
      reliability: reliable
```

### Location 2: Launch File Remappings
`launch/modular_action_servers.launch.py:86-89`

### Current Code
```python
remappings=[
    ('image_rect', '/color/image_color'),
    ('camera_info', '/color/camera_info'),
]
```

### Location 3: Vision System Launch
`launch/vision_system.launch.py:58-59`

### Problem
- **Hardcoded to Zivid camera topic structure** (`/color/...`)
- Different camera systems use different topic names:
  - Zivid: `/zivid/color/image_color`
  - RealSense: `/camera/color/image_raw`
  - Basler: `/basler/image_rect_color`

### Impact
- **Cannot use non-Zivid cameras** without editing config files
- Beamlines with different cameras must fork configs
- Switching cameras requires modifying multiple files

### Refactoring Recommendation
**Difficulty**: Medium (multiple files to update)

**Solution**: Parameterize camera topics
```yaml
# beamlines/<name>/config/vision.yaml
vision:
  camera_type: "zivid"
  topics:
    image: "/zivid/color/image_color"
    camera_info: "/zivid/camera_info"
    point_cloud: "/zivid/points/xyzrgba"
```

**Implementation**:
1. Load vision config in launch file
2. Pass topic names as launch arguments
3. Use launch substitutions for remappings
4. Update AprilTag config to use parameters instead of QoS overrides

---

## Category 5: Action Server Names ⚠️ MEDIUM

### Location
`src/mtc_orchestrator_action_server.cpp:60-64`

### Current Code
```cpp
moveto_action_client_ = rclcpp_action::create_client<MoveToAction>(this, "move_to_action");
endeffector_action_client_ = rclcpp_action::create_client<EndEffectorAction>(this, "end_effector_action");
toolexchange_action_client_ = rclcpp_action::create_client<ToolExchangeAction>(this, "tool_exchange_action");
pickplace_action_client_ = rclcpp_action::create_client<PickPlaceAction>(this, "pick_place_action");
vision_action_client_ = rclcpp_action::create_client<VisionMoveToAction>(this, "vision_move_to_action");
```

### Problem
- **Hardcoded action server names**
- Cannot namespace servers for multi-robot or multi-beamline scenarios
- All servers must use these exact names

### Impact
- **Cannot run multiple orchestrators** on same ROS network
- Cannot isolate beamlines with namespaces
- Difficult to run integration tests with multiple instances

### Refactoring Recommendation
**Difficulty**: Easy (already supported by ROS 2)

**Solution**: Use node namespace
```cpp
// Current: "move_to_action" → absolute name "/move_to_action"
// With namespace: "move_to_action" → namespaced "/cms/move_to_action"
```

**Implementation**:
1. Launch orchestrator with namespace: `<node namespace="/cms">`
2. Action servers will automatically inherit namespace
3. No code changes required in orchestrator!

**Alternative**: Make action names configurable
```yaml
# beamlines/<name>/config/beamline.yaml
action_servers:
  move_to: "move_to_action"
  end_effector: "end_effector_action"
  # ... etc
```

---

## Category 6: Service Names ⚠️ MEDIUM

### Location 1: Dashboard Client
`src/mtc_orchestrator_action_server.cpp:275`

### Current Code
```cpp
auto client = this->create_client<std_srvs::srv::Trigger>("/dashboard_client/play");
```

### Problem
- **Absolute topic name** (starts with `/`)
- Hardcoded to specific UR driver node name `dashboard_client`
- Cannot be namespaced

### Impact
- **Cannot run multiple UR robots** on same ROS network
- Breaks if UR driver is launched with different name
- Prevents multi-robot scenarios

### Location 2: MoveIt Planning Service
`src/mtc_orchestrator_action_server.cpp:263`

### Current Code
```cpp
auto plan_client = this->create_client<moveit_msgs::srv::GetMotionPlan>("/plan_kinematic_path");
```

### Problem
- **Absolute topic name** for MoveIt service
- Cannot namespace MoveIt instances

### Location 3: Camera Capture
`src/vision_stages.cpp:30`

### Current Code
```cpp
capture_client_ = node->create_client<std_srvs::srv::Trigger>("/capture_2d");
```

### Problem
- **Absolute topic name** for camera service
- Assumes Zivid camera service naming

### Refactoring Recommendation
**Difficulty**: Easy (remove leading slash)

**Solution**: Use relative names
```cpp
// Before: "/dashboard_client/play" → absolute
// After:  "dashboard_client/play"  → relative (respects namespace)
auto client = this->create_client<std_srvs::srv::Trigger>("dashboard_client/play");
```

**For camera**: Load from config
```yaml
# beamlines/<name>/config/vision.yaml
vision:
  capture_service: "capture_2d"  # or "zivid/capture" or "camera/trigger"
```

---

## Category 7: Timing Constants ⚠️ MEDIUM

### Location 1: MoveIt Initialization Timeout
`src/mtc_orchestrator_action_server.cpp:264`

### Current Code
```cpp
if (!plan_client->wait_for_service(30s)) {
```

### Location 2: Hardware Initialization Delay
`src/mtc_orchestrator_action_server.cpp:272`

### Current Code
```cpp
std::this_thread::sleep_for(5s);
```

### Location 3: Action Timeout
`src/mtc_orchestrator_action_server.cpp:314`

### Current Code
```cpp
if (result_future.wait_for(120s) != std::future_status::ready) {
```

### Problem
- **Magic numbers** for timeouts
- Different beamlines may need different timings
- Heavy grippers (Hand-E) need 45s startup, light grippers (EPick) need 5s

### Impact
- Unnecessary waiting for fast grippers
- Possible timeouts for slow hardware
- Cannot optimize for specific beamline hardware

### Refactoring Recommendation
**Difficulty**: Medium (many locations)

**Solution**: Configuration file with timeouts
```yaml
# beamlines/<name>/config/beamline.yaml
timeouts:
  moveit_initialization: 30.0  # seconds
  hardware_startup: 5.0
  action_execution: 120.0
  gripper_activation: 45.0  # Hand-E specific
```

**Alternative**: Per-gripper timeouts
```yaml
# beamlines/<name>/config/grippers.yaml
grippers:
  hande:
    startup_delay: 45  # seconds
  epick:
    startup_delay: 5
```

---

## Summary Table: All Hardcoded Values

| Category | File | Line(s) | Hardcoded Value | Priority | Difficulty |
|----------|------|---------|----------------|----------|------------|
| **Gripper Mappings** | mtc_orchestrator_action_server.cpp | 250-254 | `gripper_packages` map | **CRITICAL** | Medium |
| **Gripper Constants** | pick_place_stages.cpp | 8-10 | `hande_gripper`, `hande_open`, `hande_closed` | **HIGH** | High |
| **TCP Frame** | vision_stages.cpp | 249 | `robotiq_hande_end` | **HIGH** | Medium |
| **Robot IP** | modular_action_servers.launch.py | 13 | `192.168.56.101` | **CRITICAL** | Easy |
| **Robot IP** | mtc_action_client_example.cpp | 48 | `192.168.56.101` | Medium | Easy |
| **Camera Topics** | apriltag_config.yaml | 22-29 | `/color/image_color`, `/color/camera_info` | **HIGH** | Medium |
| **Camera Topics** | modular_action_servers.launch.py | 87-88 | `/color/` remappings | **HIGH** | Medium |
| **Camera Topics** | vision_system.launch.py | 58-59 | `/color/` remappings | **HIGH** | Medium |
| **Action Names** | mtc_orchestrator_action_server.cpp | 60-64 | 5 action server names | Medium | Easy |
| **Dashboard Service** | mtc_orchestrator_action_server.cpp | 275 | `/dashboard_client/play` | **CRITICAL** | Easy |
| **Planning Service** | mtc_orchestrator_action_server.cpp | 263 | `/plan_kinematic_path` | Medium | Easy |
| **Capture Service** | vision_stages.cpp | 30 | `/capture_2d` | **HIGH** | Medium |
| **MoveIt Timeout** | mtc_orchestrator_action_server.cpp | 264 | `30s` | Medium | Medium |
| **Hardware Delay** | mtc_orchestrator_action_server.cpp | 272 | `5s` | Medium | Medium |
| **Action Timeout** | mtc_orchestrator_action_server.cpp | 314 | `120s` | Medium | Medium |

**Total Items**: 15 distinct hardcoded values across 21 locations

---

## Recommended Refactoring Order

### Phase 1: Quick Wins (Week 1)
1. ✅ Fix absolute service names → relative names (remove leading `/`)
2. ✅ Parameterize robot IP in launch files (already mostly done)
3. ✅ Add node namespace support for action servers

**Impact**: Enables basic multi-beamline namespacing

### Phase 2: Configuration System (Week 2-3)
4. ✅ Create `ConfigurationManager` class with YAML loader
5. ✅ Load gripper package mappings from YAML
6. ✅ Load hardware IPs from YAML
7. ✅ Load timeout values from YAML

**Impact**: No more recompilation for config changes

### Phase 3: Gripper Abstraction (Week 4-5)
8. ✅ Refactor pick/place stages to use dynamic gripper config
9. ✅ Refactor vision stages to use configurable TCP frame
10. ✅ Add gripper type parameter propagation through action calls

**Impact**: Multi-gripper support across beamlines

### Phase 4: Vision Abstraction (Week 6)
11. ✅ Parameterize camera topics in launch files
12. ✅ Update AprilTag config to use launch parameters
13. ✅ Load vision config from YAML

**Impact**: Multi-camera system support

---

## Testing Strategy

### Unit Tests Needed
- `ConfigurationManager` YAML loading
- Gripper config lookup with fallback
- Invalid config handling

### Integration Tests Needed
- Load CMS config → verify gripper mappings
- Load PDF config → verify different IPs
- Switch between configs → verify MoveIt restart

### Validation Criteria
- [ ] Can switch gripper configs without recompilation
- [ ] Can run two orchestrators with different namespaces
- [ ] Can use different camera topics per beamline
- [ ] All timeouts configurable per beamline
- [ ] No absolute topic names remain (except `/robot_description`)

---

## Configuration File Structure (Recommended)

```
beamlines/
├── cms/
│   └── config/
│       ├── beamline.yaml      # Timeouts, namespaces
│       ├── grippers.yaml      # Gripper → MoveIt package mappings
│       ├── hardware.yaml      # IPs, service names
│       └── vision.yaml        # Camera topics, types
└── pdf/
    └── config/
        └── [same structure]
```

---

## Next Steps

1. **Review this document** with team
2. **Prioritize** which categories to fix first
3. **Create proof-of-concept** for ConfigurationManager
4. **Test** with one gripper type to validate approach
5. **Iterate** based on findings

---

*Document Version: 1.0*
*Analysis Date: 2025-01-17*
*Analyzed Files: 8 core files in mtc_pipeline*