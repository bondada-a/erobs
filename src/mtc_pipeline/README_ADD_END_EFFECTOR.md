# Adding a New End Effector to MTC Pipeline

This guide explains how to add a new end effector (gripper, vacuum, etc.) to the MTC pipeline system.

## Overview

The system uses a configuration-driven approach where end effectors are defined in:
1. **SRDF files** - Define the MoveIt planning groups and states
2. **C++ configuration** - Map user-friendly names to SRDF states

Adding a new end effector requires creating the URDF/SRDF and adding a few lines of configuration code.

---

## Step-by-Step Instructions

We'll use a hypothetical "suction_cup" gripper as an example throughout this guide.

### Step 1: Create the Gripper URDF/Xacro

**Location:** `/path/to/your/workspace/src/end_effectors/`

Create your gripper package structure:

```bash
cd src/end_effectors/
mkdir -p suction_cup_gripper/{urdf,meshes,config}
```

**Create the URDF/Xacro file:**

`suction_cup_gripper/urdf/suction_cup.urdf.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="suction_cup">

  <xacro:macro name="suction_cup" params="prefix">

    <!-- Gripper base link -->
    <link name="${prefix}suction_cup_base">
      <visual>
        <geometry>
          <cylinder radius="0.02" length="0.05"/>
        </geometry>
        <material name="black"/>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.02" length="0.05"/>
        </geometry>
      </collision>
    </link>

    <!-- Gripper joint (mimics on/off state) -->
    <joint name="${prefix}gripper" type="prismatic">
      <parent link="${prefix}suction_cup_base"/>
      <child link="${prefix}suction_pad"/>
      <origin xyz="0 0 -0.025" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="0" upper="0.001" effort="100" velocity="1.0"/>
    </joint>

    <!-- Suction pad -->
    <link name="${prefix}suction_pad">
      <visual>
        <geometry>
          <cylinder radius="0.015" length="0.01"/>
        </geometry>
        <material name="grey"/>
      </visual>
    </link>

  </xacro:macro>
</robot>
```

---

### Step 2: Create the MoveIt Configuration Package

**Location:** `/path/to/your/workspace/src/ur5e_moveit_configs/`

```bash
cd src/ur5e_moveit_configs/
mkdir -p ur_zivid_suction_moveit_config/{config,urdf,launch}
```

**Create the robot xacro** that combines UR5e + Zivid + your gripper:

`ur_zivid_suction_moveit_config/urdf/ur.urdf.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="ur">

  <!-- Import UR5e -->
  <xacro:include filename="$(find ur_description)/urdf/ur.urdf.xacro"/>
  <xacro:ur_robot prefix="" joint_limits_parameters_file="..."/>

  <!-- Import Zivid camera -->
  <xacro:include filename="$(find zivid_description)/urdf/zivid2.urdf.xacro"/>
  <xacro:zivid2 prefix="zivid_"/>

  <!-- Import your gripper -->
  <xacro:include filename="$(find suction_cup_gripper)/urdf/suction_cup.urdf.xacro"/>
  <xacro:suction_cup prefix=""/>

  <!-- Attach gripper to flange -->
  <joint name="flange_to_gripper" type="fixed">
    <parent link="flange"/>
    <child link="suction_cup_base"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

</robot>
```

---

### Step 3: Create the SRDF with Group States ⚠️ CRITICAL

**This is the most important file - it defines the states that MTC will use!**

`ur_zivid_suction_moveit_config/config/ur.srdf`

```xml
<?xml version="1.0"?>
<robot name="ur">

    <!-- Define the arm group -->
    <group name="ur_arm">
        <chain base_link="base_link" tip_link="flange"/>
    </group>

    <!-- Define the gripper group -->
    <group name="suction_cup_gripper">
        <joint name="gripper"/>  <!-- The joint from your URDF -->
    </group>

    <!-- ⚠️ IMPORTANT: Define gripper states - these are what the code uses -->
    <group_state name="suction_on" group="suction_cup_gripper">
        <joint name="gripper" value="0.001"/>  <!-- Extended = gripping -->
    </group_state>

    <group_state name="suction_off" group="suction_cup_gripper">
        <joint name="gripper" value="0"/>  <!-- Retracted = released -->
    </group_state>

    <!-- Define end effector -->
    <end_effector name="suction_cup_gripper" parent_link="flange" group="suction_cup_gripper"/>

    <!-- Robot poses (optional) -->
    <group_state name="moveit_home" group="ur_arm">
        <joint name="shoulder_pan_joint" value="0"/>
        <joint name="shoulder_lift_joint" value="-1.57"/>
        <joint name="elbow_joint" value="0"/>
        <joint name="wrist_1_joint" value="-1.57"/>
        <joint name="wrist_2_joint" value="0"/>
        <joint name="wrist_3_joint" value="0"/>
    </group_state>

</robot>
```

**Key Elements:**
- **`<group name="suction_cup_gripper">`** - Your gripper's planning group (use this in code)
- **`<group_state name="suction_on">`** - State name for "gripper active" (use this in code)
- **`<group_state name="suction_off">`** - State name for "gripper inactive" (use this in code)

---

### Step 4: Update EndEffectorStages Configuration

**Location:** `src/mtc_pipeline/src/end_effector_stages.cpp`

Find the `initializeGripperConfigs()` function and add your gripper configuration:

```cpp
void EndEffectorStages::initializeGripperConfigs()
{
  // Initialize gripper configurations based on SRDF definitions
  // This matches the actual states defined in the SRDF files

  // Hande gripper configuration
  gripper_configs_["hande"] = {
    "hande_gripper",
    {
      {"open", "hande_open"},
      {"close", "hande_closed"}
    }
  };

  // Epick vacuum gripper configuration
  gripper_configs_["epick"] = {
    "epick_gripper",
    {
      {"on", "vacuum_on"},
      {"off", "vacuum_off"}
    }
  };

  // ⬇️ ADD YOUR GRIPPER HERE ⬇️

  // Suction cup gripper configuration
  gripper_configs_["suction_cup"] = {
    "suction_cup_gripper",    // ⚠️ Must match SRDF <group name="">
    {
      {"grip", "suction_on"},     // action_name -> SRDF <group_state name="">
      {"release", "suction_off"}  // action_name -> SRDF <group_state name="">
    }
  };
}
```

**Configuration Format:**
```cpp
gripper_configs_["json_identifier"] = {
  "srdf_group_name",           // Must match <group name="..."> in SRDF
  {
    {"action_1", "srdf_state_1"},  // User action -> <group_state name="...">
    {"action_2", "srdf_state_2"}   // User action -> <group_state name="...">
  }
};
```

---

### Step 5: Build and Test

```bash
cd /path/to/your/workspace
colman build --packages-select mtc_pipeline
source install/setup.bash
```

---

### Step 6: Create a Test JSON

Create a test file to verify your gripper works:

`test_suction_cup.json`

```json
{
  "poses": {
    "home": [0, -90, 0, -90, 0, 0],
    "pick_approach": [45, -60, 30, -60, -90, 0]
  },
  "tasks": [
    {
      "task_type": "moveto",
      "pose": "home"
    },
    {
      "task_type": "end_effector",
      "end_effector_type": "suction_cup",
      "end_effector_action": "grip"
    },
    {
      "task_type": "moveto",
      "pose": "pick_approach"
    },
    {
      "task_type": "end_effector",
      "end_effector_type": "suction_cup",
      "end_effector_action": "release"
    }
  ]
}
```

---

### Step 7: Run and Test

```bash
# Start the servers
./run_server.sh

# In another terminal, run the test
ros2 run mtc_pipeline mtc_action_client_example test_suction_cup.json 192.168.56.101
```

**Expected output:**
```
[DEBUG] End effector control: type=suction_cup, action=grip
[DEBUG] End effector control successful: suction_cup grip
```

---

## Quick Reference

### SRDF → Code Mapping

| SRDF | Code | JSON Usage |
|------|------|------------|
| `<group name="suction_cup_gripper">` | `"suction_cup_gripper"` (group_name) | - |
| `<group_state name="suction_on">` | `"suction_on"` (state value) | - |
| `<group_state name="suction_off">` | `"suction_off"` (state value) | - |
| - | `"suction_cup"` (config key) | `"end_effector_type": "suction_cup"` |
| - | `"grip"` (action key) | `"end_effector_action": "grip"` |

### Existing Grippers

| Gripper | Type | Actions | SRDF States |
|---------|------|---------|-------------|
| **hande** | Hand-E gripper | `open`, `close` | `hande_open`, `hande_closed` |
| **epick** | EPick vacuum | `on`, `off` | `vacuum_on`, `vacuum_off` |

---

## Checklist

When adding a new end effector, make sure you:

- [ ] Created gripper URDF/xacro package
- [ ] Created MoveIt config package (ur_zivid_yourgriper_moveit_config)
- [ ] Defined `<group>` in SRDF (e.g., `suction_cup_gripper`)
- [ ] Defined `<group_state>` entries in SRDF for each action
- [ ] Added configuration to `end_effector_stages.cpp` → `initializeGripperConfigs()`
- [ ] Verified group name matches between SRDF and code
- [ ] Verified state names match between SRDF and code
- [ ] Rebuilt: `colcon build --packages-select mtc_pipeline`
- [ ] Tested with JSON file
- [ ] Verified successful execution in logs

---

## Troubleshooting

### Error: "Unknown end effector type: 'xxx'"
- Check that the `gripper_configs_` key matches what you use in JSON `"end_effector_type"`
- Make sure you rebuilt after editing the C++ file

### Error: "Unknown action 'xxx' for end effector 'yyy'"
- Check that your action name matches one of the keys in the actions map
- The error message will list valid actions

### Error: "No planning group named 'xxx_gripper'"
- Check that the `group_name` in C++ matches the `<group name="">` in your SRDF
- Make sure your MoveIt config is being loaded correctly

### Error: "No IK solver for group 'xxx_gripper'"
- This is expected for simple grippers - they don't need IK
- As long as the group states are defined, it should work

---

## Design Notes

The system is intentionally kept simple:

- **Hardcoded configuration** - Grippers are defined in C++ for compile-time safety
- **SRDF-driven** - The source of truth is your SRDF file
- **User-friendly actions** - You can use friendly names like "grip" instead of "suction_on"
- **Automatic validation** - The system tells you what's available if you make a mistake

Adding a new gripper requires:
- ~40 lines of URDF
- ~20 lines of SRDF
- **4-6 lines of C++ configuration**
- Rebuild

That's it!