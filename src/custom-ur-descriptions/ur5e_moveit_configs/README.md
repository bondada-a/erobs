# MoveIt Configurations

Unified MoveIt configuration for UR5e with parameterized gripper support via `ur5e_moveit_config`.

## Usage

```bash
# Launch with specific gripper
ros2 launch ur5e_moveit_config robot_bringup.launch.py gripper:=epick
ros2 launch ur5e_moveit_config robot_bringup.launch.py gripper:=hande
ros2 launch ur5e_moveit_config robot_bringup.launch.py gripper:=pipettor
ros2 launch ur5e_moveit_config robot_bringup.launch.py gripper:=2fg7
ros2 launch ur5e_moveit_config robot_bringup.launch.py gripper:=none         # standalone (no gripper)

# With parameters
ros2 launch ur5e_moveit_config robot_bringup.launch.py gripper:=epick robot_ip:=192.168.1.101
ros2 launch ur5e_moveit_config robot_bringup.launch.py gripper:=none use_fake_hardware:=true
```

### Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gripper` | `none` | Gripper type: `none`, `epick`, `hande`, `2fg7`, `pipettor` |
| `robot_ip` | `192.168.1.10` | UR robot IP address |
| `ur_type` | `ur5e` | UR robot model |
| `use_fake_hardware` | `false` | Enable simulation mode |
| `extension_length` | `0.013` | ePick cup extension length (m) |
| `extension_radius` | `0.004` | ePick cup extension radius (m) |
| `suction_cup_height` | `0.003` | ePick cup height (m) |
| `suction_cup_radius` | `0.0015` | ePick cup radius (m) |

## Available Grippers

| Gripper | Kinematic Chain | MoveIt Group | States |
|---------|----------------|--------------|--------|
| `none` | UR5e → Zivid → TE_RobotSide | (none) | — |
| `epick` | UR5e → Zivid → Tool Block → ePick | `epick_gripper` | `vacuum_on`, `vacuum_off` |
| `hande` | UR5e → Zivid → Tool Block → Hand-E | `hande_gripper` | `hande_open`, `hande_closed` |
| `2fg7` | UR5e → Zivid → Tool Block → 2FG7 | `2fg7_gripper` | `2fg7_open`, `2fg7_closed` |
| `pipettor` | UR5e → Zivid → Tool Block → Pipettor | (none) | via `pipette_driver_node` |

## Package Structure

```
ur5e_moveit_config/
├── config/
│   ├── kinematics.yaml                    # Shared: IK solver config
│   ├── ompl_planning.yaml                 # Shared: OMPL planner config
│   ├── pilz_cartesian_limits.yaml         # Shared: Pilz Cartesian limits
│   ├── pilz_industrial_motion_planner_planning.yaml
│   ├── none/                              # Per-gripper configs
│   │   ├── joint_limits.yaml
│   │   ├── initial_positions.yaml
│   │   ├── moveit_controllers.yaml
│   │   ├── ur_controllers.yaml
│   │   └── ur.ros2_control.xacro
│   ├── epick/                             # (same structure)
│   ├── hande/
│   ├── 2fg7/
│   └── pipettor/
├── srdf/
│   ├── ur.srdf.xacro                      # Top-level: includes common + gripper
│   ├── common.srdf.xacro                  # Shared: ur_arm, moveit_home, arm collisions
│   ├── none.srdf.xacro                    # Gripper-specific SRDF fragments
│   ├── epick.srdf.xacro
│   ├── hande.srdf.xacro
│   ├── 2fg7.srdf.xacro
│   └── pipettor.srdf.xacro
├── launch/
│   └── robot_bringup.launch.py            # OpaqueFunction + GRIPPER_CONFIGS dict
└── rviz/
    └── view_robot_mtc.rviz
```

## Payload Configuration

| Gripper | Total Mass | Components |
|---------|------------|------------|
| `none` | 1.430 kg | Mount (0.17) + Camera (1.26) |
| `epick` | 2.150 kg | Mount + Camera + ePick (0.72) |
| `hande` | 2.520 kg | Mount + Camera + Hand-E (1.09) |
| `2fg7` | 2.210 kg | Mount + Camera + 2FG7 (0.78) |
| `pipettor` | 1.630 kg | Mount + Camera + Pipettor (0.20) |

Payload is set automatically at startup via `/io_and_status_controller/set_payload` (5s delay after driver init).

## Adding a New Gripper

1. Create `config/<gripper_name>/` with: `joint_limits.yaml`, `initial_positions.yaml`, `moveit_controllers.yaml`, `ur_<name>_controllers.yaml`, `ur.ros2_control.xacro`
2. Create `srdf/<gripper_name>.srdf.xacro` with group, states, end_effector, collision pairs
3. Add `xacro:if` block in `srdf/ur.srdf.xacro`
4. Add entry to `GRIPPER_CONFIGS` dict in `launch/robot_bringup.launch.py`
5. Add entry in `default_beamline.yaml` with `moveit_package: "ur5e_moveit_config"` and `gripper_arg: "<name>"`

## Architecture Notes

- Uses **OpaqueFunction** pattern (same as UR driver upstream) to resolve `gripper` arg before constructing `MoveItConfigsBuilder` calls
- Does **NOT** start `robot_state_publisher` — the UR driver's `ur_control.launch.py` handles this
- SRDF uses **composable xacro includes** rather than monolithic conditionals
- MoveIt Setup Assistant is not compatible with parameterized configs — manage collision matrices manually

## History

This package replaces the former per-gripper packages (removed in issue #48). All MoveIt launches now go through `ur5e_moveit_config` with the `gripper:=` argument.
