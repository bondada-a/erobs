# EROBS Shared Planning Scene

Centralized planning scene package for all MoveIt configurations in EROBS.

## Overview

This package provides a **single source of truth** for collision obstacles that should be present across all gripper configurations (hande, epick, standalone). Instead of duplicating obstacle definitions in each MoveIt config, we maintain one YAML file that all configs load automatically.

## Features

- ✅ **Single YAML config** - Edit one file, all configs updated
- ✅ **Automatic loading** - Integrated into all MoveIt launch files
- ✅ **Version controlled** - Track obstacle changes in git
- ✅ **Easy to modify** - No code changes needed, just YAML
- ✅ **Gripper-agnostic** - Works with any end-effector configuration

## Quick Start

### 1. Build the package

```bash
cd /home/aditya/work/github_ws/erobs
colcon build --packages-select erobs_planning_scene
source install/setup.bash
```

### 2. Edit obstacles

Modify the obstacles in the YAML file:

```bash
vim src/erobs_planning_scene/config/beamline_scene.yaml
```

### 3. Test the scene independently

```bash
ros2 launch erobs_planning_scene load_scene.launch.py
```

### 4. Launch with MoveIt (automatic)

```bash
# Scene loads automatically with any MoveIt config
ros2 launch ur_zivid_hande_moveit_config robot_bringup.launch.py
```

## File Structure

```
erobs_planning_scene/
├── config/
│   └── beamline_scene.yaml        # Define obstacles here
├── launch/
│   └── load_scene.launch.py       # Launch file (included by MoveIt configs)
├── scripts/
│   └── scene_publisher.py         # Reads YAML, publishes to /planning_scene
└── README.md
```

## Adding/Modifying Obstacles

Edit `config/beamline_scene.yaml`:

```yaml
obstacles:
  - name: "my_new_obstacle"
    type: "box"                    # box, cylinder, or sphere
    frame: "map"
    pose:
      x: 0.5
      y: 0.3
      z: 0.2
      roll: 0.0
      pitch: 0.0
      yaw: 0.0
    size: [0.2, 0.3, 0.4]          # [length, width, height] for box
```

### Supported obstacle types:

**Box:**
```yaml
type: "box"
size: [length, width, height]
```

**Cylinder:**
```yaml
type: "cylinder"
height: 0.5
radius: 0.1
```

**Sphere:**
```yaml
type: "sphere"
radius: 0.1
```

## Visualization

View obstacles in RViz:
1. Launch any MoveIt config
2. In RViz, enable `PlanningScene` display
3. Set topic to `/planning_scene`

## Integration

This package is automatically integrated into:
- `ur_zivid_hande_moveit_config`
- `ur_zivid_epick_moveit_config`
- `ur_standalone_moveit_config`

Each launch file includes `load_scene.launch.py` to load the shared obstacles.

## Advanced Usage

### Custom scene config file

```bash
ros2 launch erobs_planning_scene load_scene.launch.py scene_config:=my_custom_scene.yaml
```

### Standalone testing

Test scene publisher without MoveIt:

```bash
ros2 run erobs_planning_scene scene_publisher.py
```

View published scene:

```bash
ros2 topic echo /planning_scene
```

## Troubleshooting

**Scene not appearing:**
- Check if node is running: `ros2 node list | grep shared_planning_scene`
- Verify topic: `ros2 topic echo /planning_scene --once`
- Check RViz display settings

**Obstacles in wrong location:**
- Verify `frame` parameter (usually "map" or "base_link")
- Check pose values in YAML
- Use RViz TF display to visualize frames

## Notes

- Scene publishes every 5 seconds to ensure all nodes receive it
- Uses `is_diff: true` to avoid overwriting robot state
- All obstacles are added with `ADD` operation
- Scene persists across gripper tool exchanges
