# End Effectors

This directory mixes project-owned packages with separately imported drivers and
models for grippers, vacuum systems and the pipettor.

## Packages already tracked here

| Package | Role |
|---|---|
| `epick_config` | Local ePick overlay, suction-cup profiles and launch configuration. |
| `onrobot_2fg7_description` | Local 2FG7 geometry and model parameters. |
| `onrobot_2fg7_driver` | Local ROS/Modbus driver, launch and hardware exercise scripts. |

These packages arrive with the EROBS checkout; they are not downloaded by the
import command below.

## Getting the Drivers

The following additional dependencies live in separate repositories. From the
EROBS repository root, the manifest imports them into named subdirectories:

```bash
vcs import src/end_effectors < src/end_effectors/end_effectors.repos
```

This pulls in:

- `serial` - ROS2 serial communication
- `robotiq_hande_driver` - Robotiq HandE gripper driver
- `robotiq_hande_description` - Robotiq HandE URDF models
- `ros2_epick_gripper` - EPick vacuum gripper driver (forked from [PickNikRobotics/ros2_epick_gripper](https://github.com/PickNikRobotics/ros2_epick_gripper) — our fork adds always-present extension link for stable URDF chain across suction cup configurations)
- `pipettor` - Pipettor model, driver and action interface

Imported directories are gitignored and versioned separately. Re-importing can
change existing checkouts.

### Install dependencies and build

From the EROBS repository root, with ROS 2 Jazzy sourced:

```bash
# Install dependencies
rosdep install --from-paths src --ignore-src -y

# Build workspace
colcon build
```

## EPick Configuration

The `epick_config` package provides an overlay configuration (over ros2_epick_driver) for our specific setup:

- Serial port: `/tmp/ttyUR`
- Custom positioning offsets
- Hardware vs simulation mode
- **Suction cup profiles** (`config/suction_cups.yaml`)

### Suction Cup Profiles

The ePick supports swappable suction cups with different dimensions. Profiles are defined in `epick_config/config/suction_cups.yaml` — this is the **single source of truth** for all cup dimensions. The URDF xacro loads this file directly via `xacro.load_yaml()`, so dimensions can never drift between config and URDF.

Available profiles: `pen_vacuum`, `7mm_dia`, `3mm_dia`, `default`

#### Changing the physical suction cup

2 edits, both just the profile name string:

1. **Xacro default** — `src/custom-ur-descriptions/cms_robot_description/urdf/ur_with_zivid_epick.xacro`:
   ```xml
   <xacro:arg name="cup_profile" default="3mm_dia"/>  <!-- change this -->
   ```

2. **Beamline config** — the YAML pointed at by `$BEAMBOT_BEAMLINE_CONFIG` (CMS: `src/beambot/config/cms_beamline.yaml`) under `grippers.epick`:
   ```yaml
   cup_profile: "3mm_dia"  # change this
   ```

Both must be the same string. Then rebuild:
```bash
colcon build --packages-select cms_robot_description
```

**Why two files?** The UR driver's robot_state_publisher processes the xacro with defaults only (it can't receive args from the orchestrator). The beamline config is what the orchestrator reads at runtime. Both must agree.

#### Adding a new cup profile

Edit **only** `epick_config/config/suction_cups.yaml`:
```yaml
cups:
  my_new_cup:
    description: "My custom cup"
    extension_length: 0.025   # meters
    extension_radius: 0.005
    suction_cup_height: 0.004
    suction_cup_radius: 0.002
```

Then set `"my_new_cup"` in the 2 files above and rebuild. No other files need changes.

#### Runtime override (temporary, current session only)

```
set_cup_profile(name="7mm_dia")
```

Takes effect on the next MoveIt restart. Does **not** change the xacro default — the UR driver RSP still uses the default until you edit the xacro and rebuild.
