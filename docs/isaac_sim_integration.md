# Isaac Sim Integration

## Programmatic GUI Integration (Isaac Sim 6)

The reproducible integration does not require manually editing a stage. Source
your ROS 2 installation, then set the repository and Isaac Sim locations for
your machine:

```bash
cd /path/to/erobs
export EROBS_ROOT="$(git rev-parse --show-toplevel)"
export ISAACSIM_ROOT=/path/to/isaacsim
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
"$ISAACSIM_ROOT/isaac-sim.streaming.sh" \
  --exec "$EROBS_ROOT/scripts/isaac_sim/start_erobs_sim.py"
```

Connect with the Isaac Sim WebRTC Streaming Client using this machine's host
address. The default signaling port is `49100`; only one livestream client is
supported at a time.

The launcher loads the EROBS robot asset, discovers its articulation root,
starts physics, and constructs a ROS 2 graph that publishes `/joint_states`
and `/clock` and consumes `/isaac_joint_commands`. It starts at
`safe_sample_transport`; override that pose with six joint angles in degrees:

```bash
"$ISAACSIM_ROOT/isaac-sim.streaming.sh" --exec \
  "$EROBS_ROOT/scripts/isaac_sim/start_erobs_sim.py --initial-arm-degrees J1 J2 J3 J4 J5 J6"
```

Build the changed ROS packages, then start EROBS in a second terminal:

```bash
cd /path/to/erobs
export EROBS_ROOT="$(git rev-parse --show-toplevel)"
colcon build --packages-select cms_moveit_config beambot --symlink-install
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
ros2 launch beambot beambot_bringup.launch.py \
  use_isaac_sim:=true enable_vision:=false enable_pipettor:=false
```

`use_isaac_sim` prevents the Universal Robots driver from starting, launches a
`FollowJointTrajectory` adapter at MoveIt's existing controller action name,
and treats physical peripherals as mock hardware. Isaac subscribes to the
existing latched `beambot/current_gripper` topic. When EROBS completes a tool
exchange, Isaac swaps to the matching `none`, `hande`, `epick`, `2fg7`, or
`pipettor` model, recreates the action graph, and preserves the arm pose.
The trajectory adapter also exposes the Hand-E, ePick, and 2FG7 gripper action
types/names already configured in MoveIt.

Check the connection with:

```bash
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
ros2 topic echo /clock --once
ros2 action list | grep scaled_joint_trajectory_controller
```

`/joint_states` must contain the six UR joint names and non-empty positions.

## Automatic URDF Import

Manual URDF loading and conversion are not part of the runtime workflow.
Isaac Sim 6 supports explicit `ros_package_paths`; the launcher supplies the
EROBS package mappings when it needs to import a model that has no cached USD.

Install the UR mesh package before first-time imports:

```bash
sudo apt install "ros-${ROS_DISTRO}-ur-description"
```

Generated assets are cached under
`cms_robot_description/urdf/generated_isaac/`. The old
`convert_urdf_for_isaac.sh`, `*_isaac.urdf`, and prebuilt `*_isaac/` USD
directories remain only for legacy Isaac Sim 4.5 imports; the launcher does
not load them.

## URDF Import Settings

| Setting | Recommended Value | Notes |
|---------|-------------------|-------|
| **Fix Base Link** | ✅ ON | Anchors robot to world |
| **Joint Drive Type** | `Stiffness` | Not Natural Frequency |
| **Allow Self Collision** | ❌ OFF | OFF = self-collision enabled |
| **Create Collisions from Visuals** | ✅ ON | Generates collision meshes for links without them |

## Common Import Warnings (Safe to Ignore)

- `The path base_link-base_link_inertia is not a valid usd path` - USD doesn't allow hyphens, auto-renamed
- `link X has no body properties and is being merged into Y` - Frame-only links merged into parents (expected)
- `No mass specified for link map` - Fixed by adding inertial to map link in `*_isaac.urdf`

## Physics Inspector Empty Fix

If Physics Inspector shows no joints after import:
1. The `map` link needs inertial properties (already fixed in `*_isaac.urdf`)
2. Delete old USD output folder and re-import
3. Ensure ArticulationRoot exists on robot root prim

## Joint Drive Parameters

Official NVIDIA UR5e + Robotiq Hand-E values stored in:
- `cms_robot_description/urdf/isaac_sim_joint_params.yaml`

**UR5e Arm Joints** (Revolute → Angular Drive):

| Joint | Stiffness | Damping |
|-------|-----------|---------|
| shoulder_pan | 9400.5 | 0.378 |
| shoulder_lift | 10020.9 | 0.412 |
| elbow | 10230.2 | 4.093 |
| wrist_1 | 3940.6 | 1.579 |
| wrist_2 | 3940.6 | 0.061 |
| wrist_3 | 1000.1 | 0.004 |

**Robotiq Hand-E** (Prismatic → Linear Drive):

| Joint | Stiffness | Damping | Max Force |
|-------|-----------|---------|-----------|
| left_finger | 1000.0 | 1000.0 | 70.0 |
| right_finger | (mimic - no drive needed) | - | - |

## Mimic Joint Configuration

The right finger is a **mimic joint** that follows the left finger.

**Correct Setup**:
- Left finger: Has Linear Drive with stiffness/damping values
- Right finger: **No drive** (uneditable) - mimic constraint controls it

**If right finger has Natural Frequency mode**: This causes oscillation. Re-import with `Joint Drive Type: Stiffness` and leave mimic joint uneditable.

## Drive Types

| Joint Type | Drive Type | Use Case |
|------------|------------|----------|
| Revolute | Angular Drive | Arm joints (rotation) |
| Prismatic | Linear Drive | Gripper fingers (translation) |

## Pre-built NVIDIA UR5e

Isaac Sim includes official UR robots with tuned physics:
```
omniverse://localhost/NVIDIA/Assets/Isaac/<version>/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd
```
Use this to extract/verify joint parameters.

## Useful References

- [Joint Tuning Guide](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/joint_tuning.html)
- [URDF Import Tutorial](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/import_urdf.html)
- [Gripper Tuning Example](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/guides/gripper_tuning_example.html)
