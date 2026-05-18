# Isaac Sim Integration

## URDF Import for Isaac Sim

URDFs using `package://` URIs don't work directly in Isaac Sim because it doesn't have access to `ROS_PACKAGE_PATH`.

**Solution**: Convert to absolute paths using the conversion script.

**Files**:
- `cms_robot_description/urdf/convert_urdf_for_isaac.sh` - Conversion script
- `cms_robot_description/urdf/*_isaac.urdf` - Converted URDFs with absolute paths

**Usage**:
```bash
cd src/custom-ur-descriptions/cms_robot_description/urdf/
./convert_urdf_for_isaac.sh ur_with_zivid_hande.urdf ur_with_zivid_hande_isaac.urdf
```

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
