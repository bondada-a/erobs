# Implementation Guide: Replace Hardcoded Gripper Map with Config File

**Status**: Ready to implement
**Estimated time**: 15 minutes
**Files to modify**: 4 files
**Files to create**: 1 file

---

## Step 1: Create Config File

**Create**: `src/mtc_pipeline/config/gripper_mappings.yaml`

```yaml
mtc_orchestrator_action_server:
  ros__parameters:
    gripper.none: "ur_standalone_moveit_config"
    gripper.epick: "ur_zivid_epick_moveit_config"
    gripper.hande: "ur_zivid_hande_moveit_config"
```

---

## Step 2: Update Header File

**File**: `src/mtc_pipeline/include/mtc_pipeline/mtc_orchestrator_action_server.hpp`

**Find** (around line 59):
```cpp
private:
    ActionServer::SharedPtr action_server_;
    std::atomic<bool> is_executing_;
    std::unique_ptr<SimpleProcessManager> process_manager_;
```

**Add** after `process_manager_`:
```cpp
    std::unordered_map<std::string, std::string> gripper_packages_;
```

---

## Step 3: Update Implementation File

**File**: `src/mtc_pipeline/src/mtc_orchestrator_action_server.cpp`

### Change 3A: Remove Static Map (lines 249-254)

**Delete these lines**:
```cpp
    // Map gripper types to MoveIt config packages                                          //TODO : Add gripper payload for each gripper
    static const std::unordered_map<std::string, std::string> gripper_packages = {
        {"none", "ur_standalone_moveit_config"},  // Temporary fix - use hande config for none gripper
        {"epick", "ur_zivid_epick_moveit_config"},
        {"hande", "ur_zivid_hande_moveit_config"}
    };
```

### Change 3B: Load Gripper Config in Constructor (line ~49)

**Find** (around line 47-49):
```cpp
MTCOrchestratorActionServer::MTCOrchestratorActionServer(const rclcpp::NodeOptions& options)
        : Node("mtc_orchestrator_action_server", options), is_executing_(false) {
        process_manager_ = std::make_unique<SimpleProcessManager>();
```

**Replace with**:
```cpp
MTCOrchestratorActionServer::MTCOrchestratorActionServer(const rclcpp::NodeOptions& options)
        : Node("mtc_orchestrator_action_server", options), is_executing_(false) {

        // Load gripper mappings from ROS parameters
        auto param_names = this->list_parameters({"gripper"}, 1);
        if (param_names.names.empty()) {
            RCLCPP_FATAL(this->get_logger(), "No gripper mappings loaded! Check config/gripper_mappings.yaml");
            throw std::runtime_error("No gripper mappings found in parameters");
        }

        for (const auto& param_name : param_names.names) {
            std::string gripper_name = param_name.substr(8);  // Remove "gripper." prefix
            std::string package = this->get_parameter(param_name).as_string();
            gripper_packages_[gripper_name] = package;
            RCLCPP_INFO(this->get_logger(), "Loaded gripper mapping: '%s' -> '%s'",
                        gripper_name.c_str(), package.c_str());
        }

        process_manager_ = std::make_unique<SimpleProcessManager>();
```

### Change 3C: Use Member Variable Instead of Static (line ~258)

**Find** (around line 258):
```cpp
    // Start MoveIt configuration
    RCLCPP_INFO(this->get_logger(), "Starting MoveIt configuration for gripper: %s", start_gripper.c_str());
    auto it = gripper_packages.find(start_gripper);
    const std::string launch_cmd = "ros2 launch " + it->second + " robot_bringup.launch.py robot_ip:=" + robot_ip;
```

**Replace with**:
```cpp
    // Start MoveIt configuration
    RCLCPP_INFO(this->get_logger(), "Starting MoveIt configuration for gripper: %s", start_gripper.c_str());
    auto it = gripper_packages_.find(start_gripper);
    if (it == gripper_packages_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Unknown gripper type: '%s'", start_gripper.c_str());
        return false;
    }
    const std::string launch_cmd = "ros2 launch " + it->second + " robot_bringup.launch.py robot_ip:=" + robot_ip;
```

---

## Step 4: Update Launch File

**File**: `src/mtc_pipeline/launch/modular_action_servers.launch.py`

**Find** (around lines 92-102):
```python
    # Main Orchestrator - manages MoveIt lifecycle
    orchestrator = Node(
        package='mtc_pipeline',
        executable='mtc_orchestrator_action_server',
        name='mtc_orchestrator_action_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'robot_ip': LaunchConfiguration('robot_ip')},
        ]
    )
```

**Replace with**:
```python
    # Main Orchestrator - manages MoveIt lifecycle
    orchestrator = Node(
        package='mtc_pipeline',
        executable='mtc_orchestrator_action_server',
        name='mtc_orchestrator_action_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'robot_ip': LaunchConfiguration('robot_ip')},
            PathJoinSubstitution([
                FindPackageShare('mtc_pipeline'),
                'config',
                'gripper_mappings.yaml'
            ])
        ]
    )
```

---

## Step 5: Install Config Directory

**File**: `src/mtc_pipeline/CMakeLists.txt`

**Find** the install section (around line 120-130, look for existing `install()` commands)

**Add**:
```cmake
# Install config directory
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
```

---

## Step 6: Build and Test

```bash
cd ~/work/github_ws/erobs
colcon build --packages-select mtc_pipeline
source install/setup.bash

# Test launch (should print loaded gripper mappings)
ros2 launch mtc_pipeline modular_action_servers.launch.py robot_ip:=192.168.56.101
```

**Expected output**:
```
[mtc_orchestrator_action_server]: Loaded gripper mapping: 'none' -> 'ur_standalone_moveit_config'
[mtc_orchestrator_action_server]: Loaded gripper mapping: 'epick' -> 'ur_zivid_epick_moveit_config'
[mtc_orchestrator_action_server]: Loaded gripper mapping: 'hande' -> 'ur_zivid_hande_moveit_config'
[mtc_orchestrator_action_server]: MTC Orchestrator Action Server started
```

---

## Verification

### Test 1: Config is loaded correctly
```bash
# Launch the node
ros2 launch mtc_pipeline modular_action_servers.launch.py

# In another terminal, check parameters
ros2 param list /mtc_orchestrator_action_server
# Should show: gripper.none, gripper.epick, gripper.hande

ros2 param get /mtc_orchestrator_action_server gripper.hande
# Should output: String value is: ur_zivid_hande_moveit_config
```

### Test 2: Add a new gripper (no recompilation)
**Edit** `config/gripper_mappings.yaml`:
```yaml
mtc_orchestrator_action_server:
  ros__parameters:
    gripper.none: "ur_standalone_moveit_config"
    gripper.epick: "ur_zivid_epick_moveit_config"
    gripper.hande: "ur_zivid_hande_moveit_config"
    gripper.custom: "ur_custom_gripper_moveit_config"  # NEW!
```

**Restart** (no rebuild needed):
```bash
ros2 launch mtc_pipeline modular_action_servers.launch.py
# Should see: Loaded gripper mapping: 'custom' -> 'ur_custom_gripper_moveit_config'
```

### Test 3: Missing config fails gracefully
**Temporarily rename** config file:
```bash
mv src/mtc_pipeline/config/gripper_mappings.yaml src/mtc_pipeline/config/gripper_mappings.yaml.bak
```

**Try launching**:
```bash
ros2 launch mtc_pipeline modular_action_servers.launch.py
# Should see: [FATAL] No gripper mappings loaded! Check config/gripper_mappings.yaml
# Node should fail to start
```

**Restore**:
```bash
mv src/mtc_pipeline/config/gripper_mappings.yaml.bak src/mtc_pipeline/config/gripper_mappings.yaml
```

---

## Rollback (If Needed)

If something breaks, revert by restoring the static map:

```cpp
// In src/mtc_orchestrator_action_server.cpp around line 249
static const std::unordered_map<std::string, std::string> gripper_packages = {
    {"none", "ur_standalone_moveit_config"},
    {"epick", "ur_zivid_epick_moveit_config"},
    {"hande", "ur_zivid_hande_moveit_config"}
};
```

And remove parameter loading code from constructor.

---

## Future Enhancements (Later)

Once this works, can easily extend to per-beamline configs:

```yaml
# config/gripper_configs/pdf_beamline.yaml
mtc_orchestrator_action_server:
  ros__parameters:
    gripper.hande: "ur_zivid_hande_moveit_config"
    gripper.epick: "ur_zivid_epick_moveit_config"
```

```yaml
# config/gripper_configs/cms_beamline.yaml
mtc_orchestrator_action_server:
  ros__parameters:
    gripper.robotiq_2f: "ur_robotiq_2f_moveit_config"
    gripper.schunk: "ur_schunk_moveit_config"
```

Then add launch argument to select config file.

---

## Summary

**What this achieves:**
- ✅ No recompilation to add/modify grippers
- ✅ Config file easy to edit for scientists
- ✅ Fail-fast if config missing
- ✅ Introspectable via `ros2 param` commands
- ✅ Foundation for per-beamline configs later

**Files modified**: 4
**Lines of code changed**: ~25
**New dependencies**: 0
**Breaking changes**: None (behavior identical if config matches old map)

---

*Ready to implement when needed - estimated time: 15 minutes*