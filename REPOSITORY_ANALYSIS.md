# EROBS ROS 2 Repository - Detailed Analysis

## Executive Summary

The EROBS (Extensible Robotic Beamline Scientist) repository is a ROS 2 system designed to manage UR5e robotic arms at NSLS-II beamlines. The current architecture uses **MoveIt Task Constructor (MTC)** as the motion planning framework and implements a **modular action server pattern** for orchestrating complex robot tasks.

The repository is **currently hardcoded for a single beamline configuration** with specific hardware setups. Refactoring for multi-beamline support would require abstraction of beamline-specific values, configuration management, and runtime parameters.

---

## 1. Repository Structure Overview

```
erobs/
├── src/
│   ├── Core Planning & Execution (MTC-based)
│   │   ├── mtc_pipeline/           # Main orchestrator and MTC implementation
│   │   ├── mtc_gui/                # GUI for task creation and execution
│   │   └── erobs_planning_scene/   # Shared collision scene (beamline-specific)
│   │
│   ├── Hardware Description & Configuration
│   │   ├── ur5e_robot_description/ # UR5e URDF with gripper variants
│   │   ├── ur5e_moveit_configs/    # 3 MoveIt configurations (standalone, hande, epick)
│   │   └── end_effectors/          # Gripper descriptions & controllers
│   │       ├── robotiq_hande_description/
│   │       ├── robotiq_hande_driver/
│   │       └── epick_config/
│   │
│   ├── Vision & Calibration
│   │   ├── zivid-ros/              # Zivid camera ROS2 wrapper
│   │   ├── ros2_aruco/             # AprilTag detection
│   │   ├── zed-ros2-wrapper/       # ZED camera wrapper (for stereo)
│   │   └── drylab_calibration/     # Hand-eye calibration tools
│   │
│   ├── Bluesky Integration
│   │   └── bluesky_ros/            # Beamline-specific Bluesky/Ophyd bridge
│   │
│   ├── Visualization
│   │   └── rviz/                   # Modified RViz for task visualization
│   │
│   └── External Dependencies (git submodules)
│       ├── moveit_task_constructor/
│       ├── apriltag_ros/
│       └── zivid-python-samples/
│
└── docker/                          # Container definitions for simulation
    ├── ursim/                       # UR simulator
    ├── ur-driver/                   # UR ROS2 driver
    └── erobs-common-img/            # Base image
```

---

## 2. Package Organization & Dependencies

### Core Packages

#### **mtc_pipeline** (Master Orchestrator)
- **Purpose**: Central task orchestration using MoveIt Task Constructor
- **Role**: Manages gripper switching, task sequencing, and MoveIt lifecycle
- **Key Files**:
  - `src/mtc_orchestrator_action_server.cpp` - Main orchestrator
  - Modular action servers (pick_place, tool_exchange, move_to, end_effector, vision)
  
**Dependencies**:
- `moveit_task_constructor_core`
- `rclcpp_action` (ROS2 action framework)
- Custom action message types (MTCExecution, MoveToAction, etc.)

#### **ur5e_robot_description**
- **Purpose**: Robot URDF definitions for different gripper configurations
- **Provides**: 3 XACRO templates
  - `ur_standalone.xacro` - UR5e + Zivid + Tool Exchanger (no gripper)
  - `ur_with_zivid_hande.xacro` - UR5e + Zivid + Hand-E gripper
  - `ur_with_zivid_epick.xacro` - UR5e + Zivid + ePick vacuum gripper
- **Status**: Includes hardcoded camera models and payload assumptions

#### **ur5e_moveit_configs**
- **Purpose**: MoveIt configuration packages (one per gripper variant)
- **Contains**: SRDF, kinematics, joint limits, controller configs, launch files
- **Status**: Currently 3 separate packages with duplicated launch patterns

#### **mtc_gui**
- **Purpose**: Tkinter-based GUI for task creation and execution
- **Features**: Pose editor, poses manager, JSON import/export
- **Dependency**: Tightly coupled to `mtc_pipeline` action server

#### **erobs_planning_scene**
- **Purpose**: Centralized beamline collision scene
- **Configuration**: YAML-based obstacle definitions
- **Status**: Beamline-specific but properly abstracted as shared scene

#### **Bluesky Integration** (`bluesky_ros/`)
- **Purpose**: Integration with NSLS-II Bluesky/Ophyd data collection framework
- **Beamline-Specific**: Hard references to PDF beamline positions
- **Files**:
  - `pdf_beamtime.py` - PDF beamline-specific plans
  - `mtc_ophyd_device.py` - General Ophyd wrapper
  - `ophyd_ros.py` - Base ROS action integration

---

## 3. What Makes Something Beamline-Specific vs Generic

### BEAMLINE-SPECIFIC (Hard to Reuse):
1. **Bluesky integration code** (`bluesky_ros/pdf_beamtime.py`)
   - References specific sample holders: `holder_shaft_storage`, `holder_shaft_inbeam`
   - Specific motor names: `OT_stage_3_X`
   - Specific task names: `PICK_UP`, `PLACE`, `RETURN_PICK_UP`, `RETURN_PLACE`

2. **Planning scene** (`erobs_planning_scene/config/beamline_scene.yaml`)
   - Defines beamline-specific obstacles
   - Frame references to beamline coordinate systems

3. **Orchestrator hardcoding** (lines 249-254 in `mtc_orchestrator_action_server.cpp`)
   ```cpp
   static const std::unordered_map<std::string, std::string> gripper_packages = {
       {"none", "ur_standalone_moveit_config"},
       {"epick", "ur_zivid_epick_moveit_config"},
       {"hande", "ur_zivid_hande_moveit_config"}
   };
   ```
   - Gripper → MoveIt package mapping is hardcoded
   - No support for custom gripper types

4. **Payload configurations** (launch files)
   - Each MoveIt launch file hardcodes payload mass and center of gravity
   - Example: `mass: 2.520, center_of_gravity: {x: 0.018, y: -0.013, z: -0.031}`

5. **IP addresses** (scattered throughout)
   - Default robot IPs: `192.168.1.10`, `192.168.56.101`
   - Gripper IP: `192.168.100.10`
   - Socat IP for HandE: `192.168.1.101`

### GENERIC (Reusable Across Beamlines):
1. **MTC action server pattern** - Well-structured template for motion planning
2. **Vision system architecture** - AprilTag-based pose detection
3. **Modular gripper integration** - End effector abstraction layer
4. **URDF composition** - XACRO-based robot description assembly

---

## 4. The Orchestrator - Current Hardcoded Parts

### Key Hardcoding in `mtc_orchestrator_action_server.cpp`:

**1. Gripper Package Mapping (Lines 249-254)**
```cpp
static const std::unordered_map<std::string, std::string> gripper_packages = {
    {"none", "ur_standalone_moveit_config"},
    {"epick", "ur_zivid_epick_moveit_config"},
    {"hande", "ur_zivid_hande_moveit_config"}
};
```
**Impact**: Only these 3 grippers are supported. Adding a new gripper requires code changes.

**2. MoveIt Lifecycle Management (Lines 235-281)**
```cpp
bool MTCOrchestratorActionServer::initialize_moveit_stack(
    const std::string& start_gripper, 
    const std::string& robot_ip) {
    // Hardcoded gripper package lookup
    // Hardcoded dashboard client path: "/dashboard_client/play"
}
```
**Impact**: Cannot support alternative MoveIt configurations or custom naming schemes.

**3. Action Server Names (Lines 60-64)**
```cpp
moveto_action_client_ = rclcpp_action::create_client<MoveToAction>(this, "move_to_action");
endeffector_action_client_ = rclcpp_action::create_client<EndEffectorAction>(this, "end_effector_action");
toolexchange_action_client_ = rclcpp_action::create_client<ToolExchangeAction>(this, "tool_exchange_action");
pickplace_action_client_ = rclcpp_action::create_client<PickPlaceAction>(this, "pick_place_action");
vision_action_client_ = rclcpp_action::create_client<VisionMoveToAction>(this, "vision_move_to_action");
```
**Impact**: Action server names are hardcoded. Cannot support multiple orchestrators or custom naming.

**4. Payload Configuration (Not in orchestrator, but MoveIt launch files)**
Each configuration has hardcoded payload:
```bash
# ur_standalone: 1.430 kg
# ur_zivid_hande: 2.520 kg
# ur_zivid_epick: Unknown (need to check)
```
**Impact**: Payload changes require launch file modifications.

---

## 5. Current Robot Configuration Architecture

### Design Pattern: 3-Way Configuration Split

```
┌─────────────────────────────────────────────────────┐
│  Gripper Configuration Choice                       │
├─────────────────────────────────────────────────────┤
│                                                     │
│  ① ur_standalone_moveit_config                     │
│     └─ ur_standalone.xacro                         │
│        └─ UR5e + Zivid + TE (no gripper)          │
│                                                     │
│  ② ur_zivid_hande_moveit_config                    │
│     └─ ur_with_zivid_hande.xacro                  │
│        └─ UR5e + Zivid + HandE                    │
│                                                     │
│  ③ ur_zivid_epick_moveit_config                    │
│     └─ ur_with_zivid_epick.xacro                  │
│        └─ UR5e + Zivid + ePick                    │
│                                                     │
└─────────────────────────────────────────────────────┘
```

### How Gripper Switching Works

1. **At orchestrator startup**: Gripper type is sent in JSON script
   ```json
   {
     "start_gripper": "hande",
     "tasks": [...]
   }
   ```

2. **Orchestrator initiates MoveIt**:
   ```
   orchestrator → maps "hande" → "ur_zivid_hande_moveit_config"
                 → launches: ros2 launch ur_zivid_hande_moveit_config robot_bringup.launch.py
   ```

3. **MoveIt bringup**:
   - Loads specific XACRO file
   - Sets up gripper controllers
   - Publishes `/robot_description`

4. **Action servers connect**: Use `/robot_description` from move_group

### Payload Configuration Details

Each MoveIt launch file sets payload via `SetPayload` service call:

**ur_standalone.launch.py (Line 115-128)**:
```python
# Total: 1.430 kg = 0.170 kg (mount) + 1.260 kg (camera + housing) + 0.000 kg (no gripper)
# CoG: [-0.038, -0.022, -0.055]
# Timing: 5 second delay after robot driver startup
```

**ur_zivid_hande.launch.py (Line 121-135)**:
```python
# Total: 2.520 kg = 0.170 kg (mount) + 1.260 kg (camera) + 1.090 kg (HandE)
# CoG: [0.018, -0.013, -0.031]
# Timing: 45 second delay (longer for safety)
```

**Status**: These values are hardcoded execution commands with timing assumptions.

---

## 6. MoveIt Task Constructor Implementation

### Architecture

```
MTCExecution Action (Orchestrator)
│
├─ ParseJSON → identify gripper type
├─ Launch MoveIt (configuration-specific)
├─ For each task in JSON:
│  ├─ Call appropriate modular action server
│  └─ Handle gripper switching if needed
│
└─ Modular Action Servers:
   ├─ MoveToActionServer
   │  └─ move_to_stages.cpp → MTC stages
   ├─ PickPlaceActionServer
   │  └─ pick_place_stages.cpp → MTC stages
   ├─ ToolExchangeActionServer
   │  └─ tool_exchange_stages.cpp → MTC stages
   ├─ EndEffectorActionServer
   │  └─ end_effector_stages.cpp → MTC stages
   └─ VisionMoveToActionServer
      └─ vision_stages.cpp → MTC stages
```

### Stage Template (Modular Pattern)

```cpp
// Base template for all action servers
template<typename ActionType, typename StagesType>
class BaseActionServer : public rclcpp::Node {
    // Generic execution handler
    void execute(const std::shared_ptr<GoalHandle> goal_handle) {
        // 1. Convert goal to JSON step
        // 2. Load poses JSON
        // 3. Execute stages_->run(step, poses)
    }
};
```

**Key Design**: Each modular server loads kinematics from ROS parameters (arm-only), not from full URDF. This allows same server code to work with any gripper configuration.

### Kinematics Configuration (modular_action_servers.launch.py, Lines 20-31)

```python
action_server_parameters = [
    {'use_sim_time': False},
    {
        'robot_description_kinematics': {
            'ur_arm': {
                'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                'kinematics_solver_search_resolution': 0.001,
                'kinematics_solver_timeout': 0.1,
                'kinematics_solver_attempts': 3
            }
        }
    }
]
```

**Design Decision**: Uses arm-only kinematics (not gripper-specific) because all gripper configs share identical UR arm kinematics.

### AprilTag Vision Integration

```python
apriltag_detector = Node(
    package='apriltag_ros',
    executable='apriltag_node',
    remappings=[
        ('image_rect', '/color/image_color'),
        ('camera_info', '/color/camera_info'),
    ]
)
```

**Hardcoding**: Camera topic names are hardcoded for Zivid camera (`/color/image_color`). Would need parameterization for different camera systems.

---

## 7. Key Areas Needing Refactoring for Multi-Beamline Support

### Priority 1: Configuration Management (Critical)

**Current State**: Hardcoded values scattered across launch files and C++ code

**Required Changes**:

1. **Gripper Package Registry** (C++)
   - Move from static unordered_map to dynamic configuration file
   - Load from YAML: `gripper_configs.yaml`
   ```yaml
   grippers:
     hande:
       package: ur_zivid_hande_moveit_config
       payload_mass: 2.520
       payload_cog: [0.018, -0.013, -0.031]
     epick:
       package: ur_zivid_epick_moveit_config
       payload_mass: 1.890
       payload_cog: [0.010, -0.005, -0.020]
     none:
       package: ur_standalone_moveit_config
       payload_mass: 1.430
       payload_cog: [-0.038, -0.022, -0.055]
   ```

2. **Payload Configuration** (Launch files)
   - Extract from hardcoded `ExecuteProcess` calls
   - Move to configuration parameters
   ```python
   payload_config_file = PathJoinSubstitution([
       FindPackageShare('gripper_config'),
       'payloads',
       f'{gripper_type}_payload.yaml'
   ])
   ```

3. **IP Address Configuration**
   - Externalize from launch file defaults
   - Create beamline-specific parameter files
   - Example: `beamline_params.yaml`
   ```yaml
   robot_ip: 192.168.1.10
   dashboard_client: /dashboard_client/play
   gripper_ip: 192.168.100.10
   socat_ip: 192.168.1.101
   ```

### Priority 2: Beamline-Agnostic Planning Scene (High)

**Current State**: `erobs_planning_scene/config/beamline_scene.yaml` is well-designed but name suggests single beamline

**Required Changes**:

1. Support multiple scene files by beamline ID
   ```python
   scene_config = DeclareLaunchArgument(
       'scene_config',
       default_value='beamline_scene.yaml',
       description='Scene config file name'
   )
   ```

2. Scene directory structure:
   ```
   erobs_planning_scene/config/
   ├── scenes/
   │   ├── pdf_beamline.yaml
   │   ├── other_beamline.yaml
   │   └── default_scene.yaml
   └── load_scene.launch.py
   ```

3. Frame parameterization (currently assumes "map")
   ```yaml
   beamline_scenes:
     pdf:
       base_frame: pdf_map
       obstacles:
         - name: beamstation
           frame: pdf_map
           ...
   ```

### Priority 3: Vision System Abstraction (High)

**Current State**: Hardcoded camera topic remappings for Zivid

**Required Changes**:

1. Camera abstraction layer
   ```yaml
   cameras:
     zivid:
       image_topic: /color/image_color
       info_topic: /color/camera_info
       frame_id: zivid_camera_link
     zed:
       image_topic: /zed_node/rgb/image_rect_color
       info_topic: /zed_node/rgb/camera_info
       frame_id: zed_camera_link
   ```

2. Launch-time camera selection
   ```python
   camera_type = DeclareLaunchArgument(
       'camera_type',
       default_value='zivid',
       choices=['zivid', 'zed', 'realsense']
   )
   ```

### Priority 4: Dynamic MoveIt Config Support (Medium)

**Current State**: 3 separate MoveIt packages, one per gripper

**Required Changes**:

1. Create parameterized MoveIt config template
   ```
   ur_moveit_config/
   ├── config/
   │   ├── ur.srdf (shared)
   │   ├── gripper_plugins/
   │   │   ├── hande_gripper.srdf
   │   │   ├── epick_gripper.srdf
   │   │   └── none_gripper.srdf
   │   └── ...
   ```

2. Composite launch approach
   ```python
   gripper_srdf = PathJoinSubstitution([
       FindPackageShare('ur_moveit_config'),
       'config/gripper_plugins',
       f'{gripper_type}_gripper.srdf'
   ])
   ```

### Priority 5: Bluesky Integration Abstraction (Medium)

**Current State**: `bluesky_ros/pdf_beamtime.py` hardcoded to PDF beamline

**Required Changes**:

1. Beamline-agnostic base class
   ```python
   class BeamlineOperationPlan:
       def __init__(self, beamline_config):
           self.config = beamline_config
       
       def generate_sample_plan(self, sample_spec):
           # Generic plan using config
   ```

2. Beamline-specific subclasses
   ```python
   # pdf_beamtime.py
   class PDFBeamlineOperationPlan(BeamlineOperationPlan):
       def __init__(self):
           super().__init__(pdf_config)
   
   # other_beamline.py
   class OtherBeamlineOperationPlan(BeamlineOperationPlan):
       ...
   ```

3. Configuration-driven positions
   ```yaml
   beamline_positions:
     pdf:
       storage: holder_shaft_storage
       inbeam: holder_shaft_inbeam
       motor_name: OT_stage_3_X
       safe_position: 0.0
   ```

### Priority 6: Orchestrator Configuration (Medium)

**Current State**: Gripper packages hardcoded in C++

**Required Changes**:

1. Load configuration at startup
   ```cpp
   // In mtc_orchestrator_action_server.cpp
   bool MTCOrchestratorActionServer::load_gripper_config(
       const std::string& config_file) {
       // Load from YAML instead of hardcoded map
   }
   ```

2. Support custom action server names
   ```yaml
   action_servers:
     move_to: move_to_action      # customize these names
     pick_place: pick_place_action
     tool_exchange: tool_exchange_action
     end_effector: end_effector_action
     vision_moveto: vision_move_to_action
   ```

### Priority 7: Roslaunch Parameterization (Low)

**Current State**: Some hardcoded defaults in launch files

**Required Changes**:

1. Centralize defaults
   ```python
   # defaults.yaml
   defaults:
     robot_type: ur5e
     default_robot_ip: 192.168.1.10
     rviz_enabled: true
     headless_mode: false
   ```

2. Launch parameter override system
   ```bash
   ros2 launch mtc_pipeline modular_action_servers.launch.py \
       beamline:=pdf \
       robot_ip:=192.168.1.50 \
       gripper:=hande
   ```

---

## 8. Package Dependencies Map

```
mtc_pipeline (core orchestrator)
├── depends_on: moveit_task_constructor_core
├── depends_on: ur5e_robot_description (at launch time)
├── depends_on: ur_{gripper}_moveit_config (dynamically selected)
├── depends_on: erobs_planning_scene (shared scene)
├── calls: move_to_action_server (action server)
├── calls: pick_place_action_server
├── calls: tool_exchange_action_server
├── calls: end_effector_action_server
└── calls: vision_move_to_action_server

mtc_gui (GUI client)
├── depends_on: mtc_pipeline
└── communicates_with: mtc_execution action server

ur5e_moveit_configs (3 packages)
├── ur_standalone_moveit_config
│   ├── depends_on: ur_description (upstream)
│   ├── depends_on: ur5e_robot_description
│   ├── depends_on: zivid_description
│   └── includes: erobs_planning_scene
├── ur_zivid_hande_moveit_config
│   └── [same as standalone + robotiq_hande_description]
└── ur_zivid_epick_moveit_config
    └── [same as standalone + epick_config]

bluesky_ros (beamline integration)
├── depends_on: mtc_ophyd_device
└── beamline_specific: pdf_beamtime.py

erobs_planning_scene (shared)
├── standalone deployable
└── included by all MoveIt configs
```

---

## 9. File Paths Summary

### Critical Hardcoding Locations

1. **Gripper Mapping**: `/src/mtc_pipeline/src/mtc_orchestrator_action_server.cpp:249-254`
2. **Payload Configs**: `/src/ur5e_moveit_configs/*/launch/robot_bringup.launch.py:115-135`
3. **IP Defaults**: `/src/mtc_pipeline/launch/modular_action_servers.launch.py:13`
4. **Camera Topics**: `/src/mtc_pipeline/launch/modular_action_servers.launch.py:86-89`
5. **Bluesky Plans**: `/src/bluesky_ros/pdf_beamtime.py:44-62` (beamline-specific)
6. **Scene Config**: `/src/erobs_planning_scene/config/beamline_scene.yaml`
7. **MoveIt Configs**: `/src/ur5e_moveit_configs/*/launch/robot_bringup.launch.py`

### Configuration Files (Already Structured Well)

- `/src/erobs_planning_scene/config/beamline_scene.yaml` - Scene obstacles
- `/src/mtc_pipeline/config/apriltag_config.yaml` - Vision settings
- `/src/ur5e_moveit_configs/*/config/moveit_controllers.yaml` - Controller maps
- `/src/ur5e_moveit_configs/*/config/kinematics.yaml` - IK solver config

---

## 10. Recommended Multi-Beamline Architecture

### Proposed Structure
```
erobs_beamline_configs/
├── pdf_beamline/
│   ├── params/
│   │   ├── robot_params.yaml
│   │   ├── gripper_configs.yaml
│   │   ├── camera_config.yaml
│   │   └── scene_config.yaml
│   └── launch/
│       └── pdf_bringup.launch.py
├── other_beamline/
│   ├── params/
│   └── launch/
└── beamline_templates/
    ├── default_robot_params.yaml
    ├── default_gripper_configs.yaml
    └── ...
```

### Launch Signature
```bash
ros2 launch mtc_pipeline orchestrator.launch.py \
    beamline:=pdf \
    config_dir:=/path/to/erobs_beamline_configs/pdf_beamline
```

### Benefits
- Single codebase supports multiple beamlines
- Configuration externalized from code
- Easy to add new beamlines (copy config directory)
- Non-invasive changes to existing architecture

---

## Summary: Multi-Beamline Refactoring Checklist

- [ ] Extract gripper configuration to external YAML
- [ ] Parameterize payload values in launch files
- [ ] Externalize all IP addresses to config files
- [ ] Abstract camera topic names and frame IDs
- [ ] Create beamline-agnostic Bluesky integration
- [ ] Support multiple planning scene configurations
- [ ] Refactor orchestrator to load config at runtime
- [ ] Create beamline parameter directory structure
- [ ] Update launch files to accept beamline parameter
- [ ] Add validation for configuration completeness
- [ ] Document beamline-specific vs generic packages
