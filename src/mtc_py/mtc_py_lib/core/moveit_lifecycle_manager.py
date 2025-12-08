"""MoveIt Lifecycle Manager - manages MoveIt process lifecycle.

Python equivalent of moveit_lifecycle_manager.cpp.
Handles launching and killing MoveIt based on gripper configuration.

Complete launch sequence (matching C++):
1. Set tool voltage via URToolInterface
2. Launch MoveIt subprocess
3. Wait for MoveIt planning service
4. Load collision obstacles
5. Restart UR external_control via dashboard
"""

import atexit
import os
import signal
import subprocess
import time
from typing import Optional

from rclpy.node import Node

from mtc_py_lib.core.beamline_config import BeamlineConfig
from mtc_py_lib.core.ur_tool_interface import URToolInterface


class MoveItLifecycleManager:
    """Manages MoveIt move_group lifecycle for gripper-specific configurations.

    Handles:
    - Setting tool voltage before MoveIt launch
    - Launching MoveIt with the correct config package for each gripper
    - Loading collision obstacles for safety
    - Restarting UR external_control program
    - Graceful shutdown when switching grippers
    """

    def __init__(self, node: Node, beamline_config: BeamlineConfig):
        """Initialize the lifecycle manager.

        Args:
            node: ROS node for logging and service checks
            beamline_config: Beamline configuration with gripper definitions
        """
        self._node = node
        self._logger = node.get_logger()
        self._beamline_config = beamline_config
        self._tool_interface = URToolInterface(node)

        self._moveit_process: Optional[subprocess.Popen] = None
        self._current_gripper: str = ""
        self._robot_ip: str = ""  # Track for potential relaunch

        # Ensure MoveIt is killed when orchestrator exits
        atexit.register(self.kill_current_process)

    @property
    def current_gripper(self) -> str:
        """Get the current gripper type.

        Returns:
            Current gripper name, or "none" if no gripper
        """
        return self._current_gripper if self._current_gripper else "none"

    @property
    def robot_ip(self) -> str:
        """Get the current robot IP address.

        Returns:
            Robot IP address, or empty string if not set
        """
        return self._robot_ip

    def launch_for_gripper(self, gripper: str, robot_ip: str) -> bool:
        """Launch MoveIt with configuration for the specified gripper.

        Complete sequence (matching C++ moveit_lifecycle_manager.cpp):
        1. Set tool voltage (must happen BEFORE MoveIt launches)
        2. Launch MoveIt subprocess
        3. Wait for MoveIt planning service to be ready
        4. Load collision obstacles for safety
        5. Restart UR external_control program (voltage command stops it)

        If MoveIt is already running with the same gripper config, reuses it.
        If running with different gripper, kills and relaunches.

        Args:
            gripper: Gripper name (e.g., "epick", "hande", "none")
            robot_ip: Robot IP address for MoveIt connection

        Returns:
            True if MoveIt is ready, False on failure
        """
        # Store robot IP for tool interface (with validation)
        if not self._tool_interface.set_robot_ip(robot_ip):
            self._logger.error(f"Invalid robot IP address: {robot_ip}")
            return False
        self._robot_ip = robot_ip

        # Reuse existing MoveIt if same gripper
        if self._moveit_process and self._current_gripper == gripper:
            if self._moveit_process.poll() is None:  # Still running
                self._logger.info(f"MoveIt already running for {gripper}, reusing")
                return True

        # Kill existing MoveIt if different gripper or process died
        if self._moveit_process:
            self._logger.info(f"Switching gripper: {self._current_gripper} → {gripper}")
            self.kill_current_process()

        # Get gripper configuration
        config = self._beamline_config.get_gripper(gripper)
        if not config:
            available = ", ".join(self._beamline_config.get_available_grippers())
            self._logger.error(
                f"Unknown gripper type: {gripper} (available: {available})"
            )
            return False

        # === Step 1: Set tool voltage (BEFORE MoveIt launches) ===
        self._logger.info(f"Step 1/5: Setting tool voltage to {config.tool_voltage}V")
        if not self._tool_interface.set_tool_voltage(config.tool_voltage):
            self._logger.error("Failed to set tool voltage")
            return False

        # === Step 2: Launch MoveIt ===
        self._logger.info(f"Step 2/5: Launching MoveIt for {gripper} gripper")
        self._logger.info(f"  Package: {config.moveit_package}")
        self._logger.info(f"  Robot IP: {robot_ip}")

        if not self._launch_moveit_process(config.moveit_package, robot_ip):
            self._logger.error("Failed to launch MoveIt process")
            return False

        # === Step 3: Wait for MoveIt to be ready ===
        self._logger.info("Step 3/5: Waiting for MoveIt planning service...")
        if not self._wait_for_moveit_ready(timeout_sec=45.0):
            self._logger.error("MoveIt not ready within timeout")
            self.kill_current_process()
            return False

        # === Step 4: Load collision obstacles ===
        self._logger.info("Step 4/5: Loading collision obstacles...")
        if not self._load_collision_obstacles():
            self._logger.warn("Failed to load obstacles - continuing anyway")
            # Note: C++ version aborts here, but we'll continue for now

        # === Step 5: Restart external_control program ===
        self._logger.info("Step 5/5: Restarting UR external_control program...")
        if not self._tool_interface.restart_external_control():
            self._logger.error("Failed to restart external_control")
            self.kill_current_process()
            return False

        self._current_gripper = gripper
        self._logger.info(f"Robot ready with {gripper} configuration")
        return True

    def kill_current_process(self):
        """Kill the current MoveIt process gracefully.

        After killing, waits for the MoveIt service to disappear from the
        ROS graph. This prevents race conditions where the old service is
        detected before the new one starts.
        """
        if not self._moveit_process:
            return

        self._logger.info("Stopping MoveIt process...")

        try:
            # Send SIGTERM to process group
            pgid = os.getpgid(self._moveit_process.pid)
            os.killpg(pgid, signal.SIGTERM)

            # Wait for graceful shutdown (up to 3 seconds)
            try:
                self._moveit_process.wait(timeout=3.0)
                self._logger.info("MoveIt stopped gracefully")
            except subprocess.TimeoutExpired:
                # Force kill if still running
                self._logger.warn("MoveIt did not stop gracefully, force killing...")
                os.killpg(pgid, signal.SIGKILL)
                self._moveit_process.wait(timeout=2.0)

        except (ProcessLookupError, OSError) as e:
            # Process already dead
            self._logger.debug(f"Process already terminated: {e}")

        self._moveit_process = None
        self._current_gripper = ""

        # Wait for old service to disappear from ROS graph
        # This prevents race conditions where wait_for_moveit_ready() finds
        # the old (dead) service before it's cleaned up from DDS
        self._wait_for_service_gone(timeout_sec=10.0)

    def _launch_moveit_process(self, package: str, robot_ip: str) -> bool:
        """Launch MoveIt as a subprocess.

        Args:
            package: MoveIt config package name
            robot_ip: Robot IP address

        Returns:
            True if process started, False on failure
        """
        try:
            cmd = [
                "ros2", "launch",
                package,
                "robot_bringup.launch.py",
                f"robot_ip:={robot_ip}",
            ]

            self._logger.info(f"Executing: {' '.join(cmd)}")

            # Start in new process group for clean shutdown
            # Note: No stdout/stderr redirect - logs flow to terminal like C++ version
            self._moveit_process = subprocess.Popen(
                cmd,
                start_new_session=True,  # Creates new process group
            )

            return True

        except Exception as e:
            self._logger.error(f"Failed to start MoveIt: {e}")
            return False

    def _wait_for_service_gone(self, timeout_sec: float = 10.0) -> bool:
        """Wait for MoveIt planning service to disappear from ROS graph.

        This is critical after killing MoveIt to prevent race conditions.
        The DDS layer may still advertise the old service for a few seconds
        after the process is killed.

        Args:
            timeout_sec: Maximum time to wait for service to disappear

        Returns:
            True if service disappeared, False on timeout (continues anyway)
        """
        service_name = "/plan_kinematic_path"
        self._logger.info(f"Waiting for old MoveIt service to disappear...")

        start_time = time.time()
        check_interval = 0.5  # Check every 500ms

        while (time.time() - start_time) < timeout_sec:
            # Get list of available services
            service_names_and_types = self._node.get_service_names_and_types()
            service_names = [name for name, _ in service_names_and_types]

            if service_name not in service_names:
                elapsed = time.time() - start_time
                self._logger.info(
                    f"Old MoveIt service gone after {elapsed:.1f}s"
                )
                return True

            time.sleep(check_interval)

        self._logger.warn(
            f"Old service still visible after {timeout_sec}s - "
            "continuing anyway (may cause brief delay)"
        )
        return False

    def _wait_for_moveit_ready(self, timeout_sec: float = 30.0) -> bool:
        """Wait for MoveIt to be ready by checking for planning service.

        Args:
            timeout_sec: Maximum time to wait

        Returns:
            True if MoveIt is ready, False on timeout
        """
        from moveit_msgs.srv import GetMotionPlan

        self._logger.info("Waiting for MoveIt to be ready...")

        client = self._node.create_client(GetMotionPlan, "/plan_kinematic_path")

        start_time = time.time()
        check_interval = 1.0  # Check every second

        while (time.time() - start_time) < timeout_sec:
            # Check if process died
            if self._moveit_process and self._moveit_process.poll() is not None:
                self._logger.error("MoveIt process died during startup")
                return False

            # Check if service is available
            if client.wait_for_service(timeout_sec=check_interval):
                self._node.destroy_client(client)
                return True

            elapsed = time.time() - start_time
            self._logger.info(f"Still waiting for MoveIt... ({elapsed:.0f}s)")

        self._node.destroy_client(client)
        return False

    def _load_collision_obstacles(self) -> bool:
        """Load collision obstacles into the planning scene.

        Loads obstacles from mtc_pipeline/config/beamline_scene.yaml.
        Uses MoveIt's PlanningSceneInterface.

        Returns:
            True if successful, False on failure
        """
        try:
            import yaml
            from ament_index_python.packages import get_package_share_directory
            from moveit_msgs.msg import CollisionObject, PlanningScene
            from shape_msgs.msg import SolidPrimitive
            from geometry_msgs.msg import Pose
            from tf_transformations import quaternion_from_euler

            # Get obstacle config path
            try:
                mtc_pipeline_share = get_package_share_directory("mtc_pipeline")
                config_path = os.path.join(
                    mtc_pipeline_share, "config", "beamline_scene.yaml"
                )
            except Exception:
                self._logger.warn("mtc_pipeline package not found, skipping obstacles")
                return True

            if not os.path.exists(config_path):
                self._logger.warn(f"Obstacle config not found: {config_path}")
                return True

            # Load YAML
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            if not config or "obstacles" not in config:
                self._logger.info("No obstacles defined in config")
                return True

            # Create planning scene publisher
            scene_pub = self._node.create_publisher(
                PlanningScene, "/planning_scene", 10
            )

            # Give publisher time to connect
            time.sleep(0.5)

            # Build collision objects
            planning_scene = PlanningScene()
            planning_scene.is_diff = True

            for obs in config["obstacles"]:
                collision_object = CollisionObject()
                collision_object.id = obs["name"]
                collision_object.header.frame_id = obs["frame"]
                collision_object.operation = CollisionObject.ADD

                # Parse pose
                pose = Pose()
                pose.position.x = float(obs["pose"]["x"])
                pose.position.y = float(obs["pose"]["y"])
                pose.position.z = float(obs["pose"]["z"])

                roll = float(obs["pose"].get("roll", 0))
                pitch = float(obs["pose"].get("pitch", 0))
                yaw = float(obs["pose"].get("yaw", 0))
                q = quaternion_from_euler(roll, pitch, yaw)
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]

                # Parse primitive
                primitive = SolidPrimitive()
                obs_type = obs["type"]

                if obs_type == "box":
                    primitive.type = SolidPrimitive.BOX
                    size = obs["size"]
                    primitive.dimensions = [
                        float(size[0]), float(size[1]), float(size[2])
                    ]
                elif obs_type == "cylinder":
                    primitive.type = SolidPrimitive.CYLINDER
                    primitive.dimensions = [
                        float(obs["height"]), float(obs["radius"])
                    ]
                elif obs_type == "sphere":
                    primitive.type = SolidPrimitive.SPHERE
                    primitive.dimensions = [float(obs["radius"])]
                else:
                    self._logger.warn(f"Unknown obstacle type: {obs_type}")
                    continue

                collision_object.primitives.append(primitive)
                collision_object.primitive_poses.append(pose)
                planning_scene.world.collision_objects.append(collision_object)

                self._logger.info(
                    f"  - Added {obs_type} '{collision_object.id}' "
                    f"in frame '{collision_object.header.frame_id}'"
                )

            # Publish planning scene
            if planning_scene.world.collision_objects:
                scene_pub.publish(planning_scene)
                self._logger.info(
                    f"Loaded {len(planning_scene.world.collision_objects)} obstacles"
                )

            self._node.destroy_publisher(scene_pub)
            return True

        except Exception as e:
            self._logger.error(f"Failed to load obstacles: {e}")
            import traceback
            self._logger.error(traceback.format_exc())
            return False

    def __del__(self):
        """Cleanup on destruction."""
        self.kill_current_process()
