# EROBS Multi-Beamline Integration Plan
## Merging zivid_integration → upstream with Generic Architecture

---

## Executive Summary

This document outlines a comprehensive strategy for integrating the `zivid_integration` branch (CMS beamline) back into upstream while transforming EROBS into a truly generic multi-beamline platform. The plan addresses repository structure, configuration management, and code refactoring needed to support any NSLS-II beamline.

---

## 1. Proposed Multi-Beamline Repository Structure

```
erobs/
├── beamlines/                      # Beamline-specific configurations
│   ├── pdf/                        # PDF beamline configs
│   │   ├── config/
│   │   │   ├── beamline.yaml       # Beamline metadata & settings
│   │   │   ├── hardware.yaml       # IP addresses, device configs
│   │   │   ├── grippers.yaml       # Gripper mappings & payloads
│   │   │   └── vision.yaml         # Camera configurations
│   │   ├── launch/
│   │   │   └── pdf_bringup.launch.py
│   │   ├── planning_scenes/
│   │   │   └── pdf_obstacles.scene
│   │   └── bluesky/
│   │       └── pdf_integration.py
│   │
│   ├── cms/                        # CMS beamline configs
│   │   ├── config/
│   │   │   ├── beamline.yaml
│   │   │   ├── hardware.yaml
│   │   │   ├── grippers.yaml
│   │   │   └── vision.yaml
│   │   ├── launch/
│   │   │   └── cms_bringup.launch.py
│   │   ├── planning_scenes/
│   │   │   └── cms_obstacles.scene
│   │   └── bluesky/
│   │       └── cms_integration.py
│   │
│   └── template/                   # Template for new beamlines
│       └── [same structure]
│
├── core/                           # Generic core packages
│   ├── mtc_pipeline/               # Orchestrator (refactored)
│   ├── mtc_action_servers/         # Modular action servers
│   ├── erobs_interfaces/           # Common messages/services
│   └── erobs_common/               # Shared utilities
│
├── robot/                          # Robot-specific packages
│   ├── ur5e_robot_description/     # URDF definitions
│   └── ur5e_moveit_config/         # Single parameterized MoveIt config
│
├── hardware/                       # Hardware interfaces
│   ├── end_effectors/              # Gripper drivers
│   ├── vision/                     # Camera interfaces
│   │   ├── zivid_ros/
│   │   └── ros2_aruco/
│   └── sensors/                    # Other sensors
│
├── tools/                          # Development tools
│   ├── mtc_gui/                    # Task GUI
│   └── calibration/                # Calibration tools
│
└── docker/                         # Containerization
    ├── Dockerfile.base             # Base ROS 2 image
    └── Dockerfile.beamline         # Beamline-specific layers
```

---

## 2. Configuration Abstraction Strategy

### 2.1 Gripper Configuration (`beamlines/<name>/config/grippers.yaml`)

```yaml
# Replace hardcoded C++ map with YAML configuration
grippers:
  hande:
    package: "ur_zivid_hande_moveit_config"
    payload:
      mass: 2.51  # kg
      center_of_mass: [0.0, 0.0, 0.12]  # meters
      startup_delay: 45  # seconds
    default_force: 20  # N
    default_speed: 100  # mm/s

  epick:
    package: "ur_zivid_epick_moveit_config"
    payload:
      mass: 1.95
      center_of_mass: [0.0, 0.0, 0.10]
      startup_delay: 5
    vacuum_threshold: -30  # kPa

  standalone:
    package: "ur_standalone_moveit_config"
    payload:
      mass: 0.0
      center_of_mass: [0.0, 0.0, 0.0]
      startup_delay: 5

default_gripper: "hande"
```

### 2.2 Hardware Configuration (`beamlines/<name>/config/hardware.yaml`)

```yaml
# Replace scattered IP addresses with centralized config
robot:
  ip: "192.168.1.10"
  dashboard_port: 29999
  ur_cap_port: 50002

gripper:
  hande:
    ip: "192.168.100.10"
    port: 502
  epick:
    ip: "192.168.100.11"
    port: 502

socat:
  host: "192.168.1.101"
  port: 54322

cameras:
  zivid:
    enabled: true
    serial: "12345678"
    topic_prefix: "/zivid"
```

### 2.3 Vision Configuration (`beamlines/<name>/config/vision.yaml`)

```yaml
# Support multiple camera systems
vision_systems:
  zivid:
    type: "zivid_camera"
    enabled: true
    topics:
      color: "/zivid/color/image_color"
      depth: "/zivid/depth/image"
      points: "/zivid/points/xyzrgb"
    calibration:
      eye_to_hand: true
      transform: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  aruco:
    type: "aruco_detector"
    enabled: true
    marker_size: 0.05
    dictionary: "DICT_4X4_50"

default_vision: "zivid"
```

### 2.4 Beamline Metadata (`beamlines/<name>/config/beamline.yaml`)

```yaml
# Beamline identification and settings
beamline:
  name: "cms"
  full_name: "Complex Materials Scattering"
  id: "11-BM"

orchestrator:
  namespace: "/cms"
  action_servers:
    - "pick_and_place"
    - "vision_scan"
    - "sample_exchange"

planning:
  scene_file: "cms_obstacles.scene"
  workspace:
    min: [-0.8, -0.8, 0.0]
    max: [0.8, 0.8, 1.5]

bluesky:
  enabled: true
  module: "cms_integration"
  sample_positions:
    holder_1: [0.3, 0.0, 0.2]
    holder_2: [0.3, 0.1, 0.2]
```

---

## 3. Orchestrator Refactoring Plan

### 3.1 Configuration Loading System

**Current (Hardcoded):**
```cpp
// mtc_orchestrator_action_server.cpp:249
std::map<std::string, std::string> gripper_to_package = {
    {"hande", "ur_zivid_hande_moveit_config"},
    // ...
};
```

**Proposed (Configuration-Driven):**
```cpp
class ConfigurationManager {
private:
    std::string beamline_name_;
    YAML::Node gripper_config_;
    YAML::Node hardware_config_;

public:
    ConfigurationManager(const std::string& beamline) {
        std::string config_path =
            ament_index_cpp::get_package_share_directory("erobs_core") +
            "/beamlines/" + beamline + "/config/";

        gripper_config_ = YAML::LoadFile(config_path + "grippers.yaml");
        hardware_config_ = YAML::LoadFile(config_path + "hardware.yaml");
    }

    std::string getGripperPackage(const std::string& gripper) {
        return gripper_config_["grippers"][gripper]["package"].as<std::string>();
    }

    PayloadParams getPayloadParams(const std::string& gripper) {
        auto payload = gripper_config_["grippers"][gripper]["payload"];
        return {
            .mass = payload["mass"].as<double>(),
            .com = payload["center_of_mass"].as<std::vector<double>>(),
            .delay = payload["startup_delay"].as<int>()
        };
    }
};
```

### 3.2 Launch Parameter Propagation

**New Main Launch File:**
```python
# erobs_bringup.launch.py
def generate_launch_description():
    beamline_arg = DeclareLaunchArgument(
        'beamline',
        default_value='pdf',
        description='Beamline configuration to load'
    )

    # Load beamline-specific config
    config_path = PathJoinSubstitution([
        FindPackageShare('erobs_core'),
        'beamlines',
        LaunchConfiguration('beamline'),
        'config'
    ])

    # Launch orchestrator with config path
    orchestrator = Node(
        package='mtc_pipeline',
        executable='mtc_orchestrator',
        parameters=[{
            'beamline': LaunchConfiguration('beamline'),
            'config_path': config_path,
        }]
    )
```

### 3.3 Dynamic MoveIt Configuration

**Consolidate 3 MoveIt packages into 1:**
```python
# ur5e_moveit_config/launch/moveit.launch.py
def generate_launch_description():
    gripper_type = LaunchConfiguration('gripper_type')

    # Load URDF with gripper parameter
    robot_description = Command([
        'xacro ',
        FindPackageShare('ur5e_robot_description'),
        '/urdf/ur5e.urdf.xacro',
        ' gripper:=', gripper_type,
        ' use_zivid:=', LaunchConfiguration('use_zivid'),
    ])

    # Load SRDF for specific gripper
    srdf_path = PathJoinSubstitution([
        FindPackageShare('ur5e_moveit_config'),
        'config',
        gripper_type,
        'ur5e.srdf'
    ])
```

---

## 4. Migration Strategy

### Phase 1: Preparation (Week 1-2)
1. Create configuration schema definitions
2. Set up beamline directory structure
3. Write configuration loader utilities
4. Create migration scripts for existing configs

### Phase 2: Core Refactoring (Week 3-4)
1. Refactor orchestrator to use ConfigurationManager
2. Consolidate MoveIt packages
3. Update launch files for parameterization
4. Create beamline-agnostic action servers

### Phase 3: Beamline Migration (Week 5)
1. Migrate PDF beamline configs
2. Migrate CMS beamline configs
3. Update Bluesky integration modules
4. Test both beamlines independently

### Phase 4: Integration (Week 6)
1. Merge zivid_integration → upstream
2. Resolve conflicts (prioritize MTC implementation)
3. Run integration tests
4. Update documentation

---

## 5. Git Merge Strategy

### 5.1 Pre-Merge Preparation
```bash
# Create backup branch
git checkout zivid_integration
git checkout -b zivid_integration_backup

# Update from upstream
git fetch upstream
git checkout -b integration_work upstream/main
```

### 5.2 Selective Merge Approach
```bash
# Cherry-pick MTC implementation
git cherry-pick <mtc-commits>

# Merge configuration changes
git checkout zivid_integration -- src/mtc_pipeline/
git checkout zivid_integration -- src/mtc_action_servers/

# Apply refactoring commits
git apply refactoring.patch
```

### 5.3 Conflict Resolution Priority
1. **Keep from zivid_integration:**
   - MTC implementation
   - Action server architecture
   - Vision integration

2. **Keep from upstream:**
   - Generic beamline structure
   - Configuration system
   - Documentation

3. **Merge carefully:**
   - Launch files (parameterize)
   - URDF files (combine variants)
   - CMakeLists.txt (merge dependencies)

---

## 6. Implementation Roadmap

### Immediate Actions (Do First)
1. ✅ Create `INTEGRATION_PLAN.md` (this document)
2. Create beamline config YAMLs for PDF and CMS
3. Write ConfigurationManager class
4. Test configuration loading

### Short Term (1-2 weeks)
1. Refactor orchestrator hardcoding
2. Parameterize launch files
3. Consolidate MoveIt configs
4. Create migration scripts

### Medium Term (3-4 weeks)
1. Complete core refactoring
2. Test with both beamlines
3. Prepare merge branch
4. Document changes

### Long Term (1-2 months)
1. Complete integration
2. Add CI/CD for multi-beamline
3. Create beamline onboarding docs
4. Deploy to production

---

## 7. Testing Strategy

### Unit Tests
- Configuration loading
- Gripper switching logic
- Action server templates

### Integration Tests
```yaml
# .github/workflows/multi_beamline_test.yml
test_matrix:
  beamline: [pdf, cms]
  gripper: [hande, epick, standalone]
  vision: [zivid, aruco, none]
```

### Beamline Validation
```bash
# Test script for each beamline
./test_beamline.sh --beamline cms --gripper hande --full-cycle
```

---

## 8. Documentation Requirements

### For Developers
- Architecture overview
- Configuration schema reference
- Adding new beamline guide
- Action server development guide

### For Beamline Scientists
- Beamline configuration guide
- Gripper setup instructions
- Troubleshooting guide
- Operation manual

### For System Administrators
- Deployment procedures
- Network configuration
- Backup/recovery procedures
- Monitoring setup

---

## 9. Risk Mitigation

### Technical Risks
| Risk | Mitigation |
|------|------------|
| Breaking existing PDF setup | Maintain compatibility mode |
| Config complexity | Provide templates and validators |
| Performance regression | Benchmark before/after |
| Lost features during merge | Comprehensive test coverage |

### Process Risks
| Risk | Mitigation |
|------|------------|
| Long integration time | Incremental merging |
| Upstream conflicts | Regular sync meetings |
| Documentation lag | Doc-as-code approach |

---

## 10. Success Criteria

### Functional Requirements
- [ ] Both PDF and CMS beamlines work
- [ ] Gripper switching is configuration-driven
- [ ] No hardcoded beamline-specific values in core
- [ ] MTC implementation is preserved
- [ ] All tests pass

### Non-Functional Requirements
- [ ] Configuration changes don't require recompilation
- [ ] New beamline setup < 1 hour
- [ ] Performance within 5% of current
- [ ] Documentation complete
- [ ] CI/CD pipeline functional

---

## Appendix A: Configuration File Templates

### Complete Beamline Configuration Example
```yaml
# beamlines/new_beamline/config/beamline.yaml
beamline:
  name: "nbl"
  full_name: "New Beamline"
  id: "XX-YY"

  hardware:
    robot_ip: "192.168.1.20"
    gripper_ip: "192.168.100.20"

  grippers:
    available: ["custom_gripper"]
    default: "custom_gripper"

  vision:
    enabled: true
    type: "realsense"

  planning:
    scene: "nbl_obstacles.scene"
```

---

## Appendix B: Migration Checklist

- [ ] Backup current branches
- [ ] Create config files for PDF
- [ ] Create config files for CMS
- [ ] Refactor orchestrator
- [ ] Update launch files
- [ ] Consolidate MoveIt packages
- [ ] Test PDF beamline
- [ ] Test CMS beamline
- [ ] Merge to integration branch
- [ ] Run full test suite
- [ ] Update documentation
- [ ] Create PR to upstream
- [ ] Deploy and validate

---

## Next Steps

1. **Review this plan** with stakeholders
2. **Prioritize** which hardcoding issues to fix first
3. **Create proof-of-concept** for configuration system
4. **Test** configuration loading with one gripper
5. **Iterate** based on feedback

---

*Document Version: 1.0*
*Created: 2025-01-17*
*Status: DRAFT - Awaiting Review*

<function_calls>
<invoke name="TodoWrite">
<parameter name="todos">[{"content": "Analyze current repository structure and architecture", "status": "completed", "activeForm": "Analyzing current repository structure and architecture"}, {"content": "Design multi-beamline repository structure", "status": "completed", "activeForm": "Designing multi-beamline repository structure"}, {"content": "Create configuration abstraction strategy", "status": "in_progress", "activeForm": "Creating configuration abstraction strategy"}, {"content": "Plan code refactoring for generic orchestrator", "status": "pending", "activeForm": "Planning code refactoring for generic orchestrator"}, {"content": "Define migration and merge strategy", "status": "pending", "activeForm": "Defining migration and merge strategy"}, {"content": "Document integration plan and implementation roadmap", "status": "pending", "activeForm": "Documenting integration plan and implementation roadmap"}]