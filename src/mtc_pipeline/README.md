# MTC Pipeline

## Design Considerations

### Kinematics Loading

Action servers use gripper-agnostic kinematics parameters (arm-only) hardcoded in the launch file, while subscribing to `/robot_description` topic for the full URDF published by MoveIt. This design allows action servers to remain stateless and work with any gripper configuration, since all gripper configs use identical kinematics for the `ur_arm` planning group (grippers don't affect arm inverse kinematics).

### MoveIt Initialization

The orchestrator waits for the `/plan_kinematic_path` service to become available before proceeding, ensuring the OMPL planning pipeline is fully initialized (~7-10 seconds). This prevents action servers from attempting to plan before MoveIt is ready, which would cause indefinite hangs.
