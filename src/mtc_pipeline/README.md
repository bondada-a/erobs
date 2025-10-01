# MTC Pipeline

## Design Considerations

### Kinematics Loading

Action servers use gripper-agnostic kinematics parameters (arm-only) hardcoded in the launch file, while subscribing to `/robot_description` topic for the full URDF published by MoveIt. This design allows action servers to remain stateless and work with any gripper configuration, since all gripper configs use identical kinematics for the `ur_arm` planning group (grippers don't affect arm inverse kinematics).

### MoveIt Initialization

The orchestrator waits for the `/plan_kinematic_path` service to become available before proceeding, ensuring the OMPL planning pipeline is fully initialized (~7-10 seconds). This prevents action servers from attempting to plan before MoveIt is ready, which would cause indefinite hangs.

### ROS Logging Warnings

`[WARN] [rcl.logging_rosout]: Publisher already registered` warnings are cosmetic and can be ignored. They occur because MoveIt's internal components use static global loggers that re-register on each `loadRobotModel()` call. This is a known ROS2 limitation (see [ros2/rcl#984](https://github.com/ros2/rcl/issues/984), [ros2/rcl#1088](https://github.com/ros2/rcl/pull/1088)).
