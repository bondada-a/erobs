 UR MoveIt2 OctoMap Package

A fork of the official `ur_moveit_config` with **optional** OctoMap support using live point clouds.

**Launch**
```bash
   ros2 launch ur_moveit_config ur_moveit.launch.py \
     ur_type:=ur5e \
     launch_rviz:=true \
     launch_servo:=false \
     launch_octomap:=true \
     description_package:=ur5e_hande_robot_description \
     description_file:=ur_with_hande.xacro \
     moveit_config_package:=ur5e_hande_moveit_config \
     moveit_config_file:=ur.srdf
   ```

## Configuration

OctoMap parameters live in:  
```
ur5e_hande_moveit_config/config/planning_scene_monitor.yaml
```