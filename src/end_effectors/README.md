# End effectors

This workspace groups end-effector drivers and descriptions under `src/end_effectors`.

## Tracking upstream via .repos

Use `src/end_effectors/end_effectors.repos` to fetch or update drivers from their official repositories:

```bash
vcs import src < src/end_effectors/end_effectors.repos
vcs pull src
```

This avoids copying code directly into the repo and keeps packages up to date.

## Site-specific configuration for EPick

We track `ros2_epick_gripper` upstream, but keep site-specific settings in a small overlay package `epick_config`:

- Pose (TCP to parent):
  - `origin_xyz`: `0 -0.019 0`
  - `origin_rpy`: `1.5708 0 0`
  - `suction_cup_height`: `0.020`
- Serial interface:
  - `usb_port`: `/tmp/ttyUR` (recommend creating a udev symlink for stability)
  - `use_fake_hardware`: `false`

### Using the overlay

- Include the overlay xacro from `epick_config` instead of editing upstream files:

```xml
<xacro:include filename="$(find-pkg-share epick_config)/urdf/epick_overlay.xacro"/>
<xacro:epick_overlay parent="tool0"/>
```

- Or launch the controller with the overlay parameters:

```bash
ros2 launch epick_config epick_bringup.launch.py usb_port:=/tmp/ttyUR use_fake_hardware:=false
```

### Recommended udev rule

Create a stable device symlink `/dev/ttyUR`:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="vvvv", ATTRS{idProduct}=="pppp", SYMLINK+="ttyUR", MODE="0666"
```

Replace `vvvv`/`pppp` with your actual device IDs.

## Packages moved here

- `serial`
- `robotiq_hande_driver`
- `robotiq_hande_description`

`ros2_epick_gripper` remains upstream-tracked. Local pose/port settings live in `epick_config`.
