# Adding a New End Effector to MTC Pipeline

This guide explains how to add a new end effector (gripper, vacuum, etc.) to the MTC pipeline system.

## Overview

The system uses an SRDF-driven approach where end effectors are defined in:
1. **URDF files** - Define the physical robot model
2. **SRDF files** - Define the MoveIt planning groups and named states
3. **JSON files** - Use SRDF names directly to control the end effector

Adding a new end effector only requires creating the URDF/SRDF - no C++ code changes needed!

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

### Step 4: Build and Test

```bash
cd /path/to/your/workspace
colman build --packages-select mtc_pipeline
source install/setup.bash
```

---

### Step 5: Create a Test JSON

Create a test file to verify your gripper works. Use the SRDF names directly:

`test_suction_cup.json`

```json
{
  "start_gripper": "suction_cup_gripper",
  "poses": {
    "home": [0, -90, 0, -90, 0, 0],
    "pick_approach": [45, -60, 30, -60, -90, 0]
  },
  "tasks": [
    {
      "task_type": "moveto",
      "target_type": "named_state",
      "target": "moveit_home",
      "planning_type": "joint"
    },
    {
      "task_type": "end_effector",
      "end_effector_type": "suction_cup_gripper",
      "end_effector_action": "suction_on"
    },
    {
      "task_type": "moveto",
      "target_type": "pose",
      "target": "pick_approach",
      "planning_type": "joint"
    },
    {
      "task_type": "end_effector",
      "end_effector_type": "suction_cup_gripper",
      "end_effector_action": "suction_off"
    }
  ]
}
```

**Important:**
- `end_effector_type` must match your SRDF `<group name="...">`
- `end_effector_action` must match your SRDF `<group_state name="...">`

---

### Step 6: Run and Test

```bash
# Start the servers
./run_server.sh

# In another terminal, run the test
ros2 run mtc_pipeline mtc_action_client_example test_suction_cup.json 192.168.56.101
```

**Expected output:**
```
[INFO] Task name: suction_cup_gripper suction_on
[INFO] MoveTo task completed successfully
```

---

## Quick Reference

### SRDF → JSON Mapping

The JSON directly uses SRDF names - no translation needed!

| SRDF Element | SRDF Example | JSON Field | JSON Value |
|--------------|--------------|------------|------------|
| `<group name="...">` | `suction_cup_gripper` | `end_effector_type` | `"suction_cup_gripper"` |
| `<group_state name="...">` | `suction_on` | `end_effector_action` | `"suction_on"` |
| `<group_state name="...">` | `suction_off` | `end_effector_action` | `"suction_off"` |

### Existing Grippers

| Group Name | Type | SRDF States |
|------------|------|-------------|
| `hande_gripper` | Hand-E gripper | `hande_open`, `hande_closed` |
| `epick_gripper` | EPick vacuum | `vacuum_on`, `vacuum_off` |

---

## Checklist

When adding a new end effector, make sure you:

- [ ] Created gripper URDF/xacro package
- [ ] Created MoveIt config package (ur_zivid_yourgriper_moveit_config)
- [ ] Defined `<group>` in SRDF (e.g., `suction_cup_gripper`)
- [ ] Defined `<group_state>` entries in SRDF for each action
- [ ] Created test JSON file with SRDF names
- [ ] Verified JSON uses correct SRDF group name for `end_effector_type`
- [ ] Verified JSON uses correct SRDF state names for `end_effector_action`
- [ ] Tested with action client
- [ ] Verified successful execution in logs

---

## Troubleshooting

### Error: JSON parsing error with end_effector fields
- Verify `end_effector_type` exactly matches your SRDF `<group name="">`
- Verify `end_effector_action` exactly matches your SRDF `<group_state name="">`
- SRDF names are case-sensitive!

### Error: "No planning group named 'xxx_gripper'"
- Check that your JSON `end_effector_type` matches the `<group name="">` in your SRDF
- Make sure your MoveIt config is being loaded correctly
- Verify you're launching with the correct robot configuration

### Error: "Goal state 'xxx' not found"
- Check that your JSON `end_effector_action` matches a `<group_state name="">` in your SRDF
- Make sure the state is defined for the correct group

### Error: "No IK solver for group 'xxx_gripper'"
- This is expected for simple grippers - they don't need IK
- As long as the group states are defined, it should work

---

## Design Notes

The system is intentionally kept simple and SRDF-driven:

- **No C++ code needed** - Everything is defined in URDF/SRDF
- **SRDF is the source of truth** - JSON uses SRDF names directly
- **Zero abstraction** - What you define in SRDF is what you use in JSON
- **Simple and explicit** - Easy to understand and debug

Adding a new gripper requires:
- ~40 lines of URDF (robot model)
- ~20 lines of SRDF (planning groups and states)
- **0 lines of C++** - No code changes!
- JSON uses the SRDF names directly

That's it!