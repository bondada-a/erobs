"""MoveIt Lifecycle Manager - manages MoveIt process lifecycle.

Python equivalent of moveit_lifecycle_manager.cpp.
Handles launching and killing MoveIt based on gripper configuration.

Complete launch sequence (matching C++):
1. Set tool voltage via raw socket (port 30002)
2. Launch MoveIt subprocess
3. Wait for MoveIt planning service
4. Load collision obstacles
5. Restart UR external_control via dashboard
"""

import atexit
import os
import signal
import socket
import subprocess
import time
import traceback
from typing import Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf_transformations import quaternion_from_euler


class MoveItLifecycleManager:
    """Manages MoveIt move_group lifecycle for gripper-specific configurations."""

    # UR robot constants
    UR_SECONDARY_PORT = 30002  # URScript command port
    SOCKET_TIMEOUT = 2.0

    def __init__(self, node: Node, grippers: dict, robot_ip: str, callback_group=None,
                 use_fake_hardware: bool = False):
        """Initialize the lifecycle manager.

        Args:
            node: ROS node for logging and service checks
            grippers: Dict of gripper_name -> {moveit_package, tool_voltage, gripper_group}
            robot_ip: Robot IP address (constant for beamline)
            callback_group: Optional callback group for service clients
            use_fake_hardware: If True, launch MoveIt in simulation mode (no real robot)
        """
        self._node = node
        self._logger = node.get_logger()
        self._grippers = grippers
        self._robot_ip = robot_ip
        self._callback_group = callback_group
        self._use_fake_hardware = use_fake_hardware

        self._moveit_process: Optional[subprocess.Popen] = None
        self._current_gripper: str = ""
        self._cup_z_offset: float = 0.0

        # Ensure MoveIt is killed when orchestrator exits
        atexit.register(self.kill_current_process)

    def is_moveit_alive(self) -> bool:
        """Check if the MoveIt subprocess is still running."""
        if self._moveit_process is None:
            return False
        return self._moveit_process.poll() is None

    def get_moveit_exit_info(self) -> str:
        """Get exit info if MoveIt has died. Empty string if still running."""
        if self._moveit_process is None:
            return "MoveIt process not started"
        rc = self._moveit_process.poll()
        if rc is None:
            return ""
        return f"MoveIt process exited with code {rc}"

    @property
    def cup_z_offset(self) -> float:
        """Z offset for the active suction cup profile (meters)."""
        return self._cup_z_offset

    def _resolve_cup_profile(self, gripper_config: dict) -> Optional[dict]:
        """Resolve suction cup dimensions from the cup profile in gripper config.

        Returns dict of cup dimensions, or None if no cup_profile specified.
        """
        profile_name = gripper_config.get("cup_profile")
        if not profile_name:
            return None

        try:
            cups_file = os.path.join(
                get_package_share_directory("epick_config"), "config", "suction_cups.yaml"
            )
            with open(cups_file, "r") as f:
                cups_data = yaml.safe_load(f)

            cups = cups_data.get("cups", {})
            if profile_name not in cups:
                self._logger.error(
                    f"Cup profile '{profile_name}' not found in {cups_file}. "
                    f"Available: {list(cups.keys())}"
                )
                return None

            profile = cups[profile_name]
            self._logger.info(
                f"Cup profile '{profile_name}': {profile.get('description', '')}"
            )
            return profile

        except Exception as e:
            self._logger.error(f"Failed to load cup profile '{profile_name}': {e}")
            return None

    def launch_moveit_with_gripper(self, gripper: str) -> bool:
        """Launch MoveIt with configuration for the specified gripper.

        Sequence: set voltage → launch MoveIt → wait ready → load obstacles → restart external_control

        Returns:
            True if MoveIt is ready, False on failure
        """
        # Check existing MoveIt process
        if self._moveit_process:
            if self._current_gripper == gripper and self._moveit_process.poll() is None:
                self._logger.info(f"MoveIt already running for {gripper}, reusing")
                return True
            # Different gripper OR process died - kill and restart
            self._logger.info(f"Switching gripper: {self._current_gripper} → {gripper}")
            self.kill_current_process()

        config = self._grippers[gripper]  # Validated by orchestrator before calling
        self._logger.info(f"Launching MoveIt for {gripper} ({config['moveit_package']})")

        # Set tool voltage (must happen BEFORE MoveIt launches)
        # Skip for fake hardware - no real robot to configure
        if not self._use_fake_hardware:
            if not self._set_tool_voltage(config["tool_voltage"]):
                self._logger.error("Failed to set tool voltage")
                return False

            # Wait for gripper to power up after voltage change.
            # Without this delay, Hand-E activation fails with Modbus errors
            # because the gripper hardware isn't ready when ros2_control tries to initialize it.
            time.sleep(2.0)

        # Launch MoveIt subprocess
        try:
            gripper_arg = config.get("gripper_arg", gripper)  # Fallback to gripper name
            cmd = [
                "ros2", "launch", config["moveit_package"], "robot_bringup.launch.py",
                f"robot_ip:={self._robot_ip}",
                f"use_fake_hardware:={'true' if self._use_fake_hardware else 'false'}",
                f"gripper:={gripper_arg}",
            ]

            # Add suction cup dimensions if cup_profile is specified (ePick)
            cup_dims = self._resolve_cup_profile(config)
            if cup_dims:
                for key, value in cup_dims.items():
                    if key != "description":
                        cmd.append(f"{key}:={value}")

            # z_offset lives in the gripper config (shared chain error, not per-cup)
            self._cup_z_offset = float(config.get("z_offset", 0.0))
            self._logger.info(f"Executing: {' '.join(cmd)}")
            self._moveit_process = subprocess.Popen(cmd, start_new_session=True)
        except Exception as e:
            self._logger.error(f"Failed to launch MoveIt process: {e}")
            return False

        # Wait for MoveIt to be ready
        if not self._wait_for_moveit_ready(timeout_sec=45.0):
            self._logger.error("MoveIt not ready within timeout")
            self.kill_current_process()
            return False

        # Load collision obstacles
        if not self._load_collision_obstacles():
            self._logger.error("Failed to load collision obstacles")
            self.kill_current_process()
            return False

        # Restart external_control program (voltage command stops it)
        # Skip for fake hardware - no real robot to control
        if not self._use_fake_hardware:
            if not self._restart_external_control():
                self._logger.error("Failed to restart external_control")
                self.kill_current_process()
                return False

        self._current_gripper = gripper
        self._logger.info(f"Robot ready with {gripper} configuration")
        return True

    def kill_current_process(self):
        """Kill the current MoveIt process gracefully."""
        if not self._moveit_process:
            return

        self._logger.info("Stopping MoveIt process...")

        try:
            pgid = os.getpgid(self._moveit_process.pid)
            os.killpg(pgid, signal.SIGTERM)
            self._moveit_process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(pgid, signal.SIGKILL)
            self._moveit_process.wait()
        except (ProcessLookupError, OSError):
            pass  # Process already dead

        self._moveit_process = None
        self._current_gripper = ""

    def _wait_for_moveit_ready(self, timeout_sec: float = 45.0) -> bool:
        """Wait for MoveIt to be ready using polling approach.

        Polls /get_planning_scene service until it actually responds,
        not just until it exists. This ensures MoveGroup is fully initialized.
        """
        from moveit_msgs.srv import GetPlanningScene

        self._logger.info("Waiting for MoveIt to be ready...")

        # Brief delay to let old services clean up after MoveIt restart
        time.sleep(2.0)

        poll_interval = 1.0
        max_attempts = int(timeout_sec / poll_interval)

        for attempt in range(max_attempts):
            client = None
            try:
                # Create fresh client each attempt (avoids stale connections)
                # Use callback group for proper threading with MultiThreadedExecutor
                client = self._node.create_client(
                    GetPlanningScene, "/get_planning_scene",
                    callback_group=self._callback_group
                )

                # Wait briefly for service to exist
                if not client.wait_for_service(timeout_sec=2.0):
                    self._node.destroy_client(client)
                    self._logger.debug(f"Service not available yet (attempt {attempt + 1}/{max_attempts})")
                    time.sleep(poll_interval)
                    continue

                # Actually call the service to verify MoveGroup is responding
                request = GetPlanningScene.Request()
                request.components.components = 0  # Minimal request

                future = client.call_async(request)

                # Wait for future - MultiThreadedExecutor processes callbacks
                # Don't use spin_once as it can cause executor state issues
                start = time.time()
                while not future.done() and (time.time() - start) < 5.0:
                    time.sleep(0.05)

                self._node.destroy_client(client)

                if future.done() and future.result() is not None:
                    self._logger.info(f"MoveIt ready (verified after {attempt + 1} attempt(s))")
                    return True

                self._logger.debug(f"Service call failed (attempt {attempt + 1}/{max_attempts})")

            except Exception as e:
                self._logger.debug(f"Poll attempt {attempt + 1} failed: {e}")
                try:
                    self._node.destroy_client(client)
                except Exception:
                    pass

            time.sleep(poll_interval)

        self._logger.error("MoveIt not ready within timeout")
        return False

    def _load_collision_obstacles(self) -> bool:
        """Load collision obstacles into the planning scene.

        Loads obstacles from beambot/config/beamline_scene.yaml and publishes
        them to the /planning_scene topic.

        Returns:
            True if successful, False on failure
        """
        try:
            # Get obstacle config path
            try:
                beambot_share = get_package_share_directory("beambot")
                config_path = os.path.join(
                    beambot_share, "config", "beamline_scene.yaml"
                )
            except Exception:
                self._logger.error("beambot package not found")
                return False

            if not os.path.exists(config_path):
                self._logger.error(f"Obstacle config not found: {config_path}")
                return False

            # Load YAML
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            if not config or "obstacles" not in config:
                self._logger.info("No obstacles defined in config")
                return True

            # Use default QoS with reliable delivery
            # MoveGroup subscribes with VOLATILE durability, so we must match
            scene_qos = QoSProfile(
                depth=10,
                durability=DurabilityPolicy.VOLATILE,
                reliability=ReliabilityPolicy.RELIABLE
            )

            scene_pub = self._node.create_publisher(
                PlanningScene, "/planning_scene", scene_qos
            )

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

            # Publish obstacles to planning scene
            if planning_scene.world.collision_objects:
                # Wait for MoveGroup to subscribe (it should already be listening)
                timeout = 5.0
                start = time.time()
                while scene_pub.get_subscription_count() == 0 and (time.time() - start) < timeout:
                    time.sleep(0.1)

                if scene_pub.get_subscription_count() == 0:
                    self._logger.warn("No subscribers on /planning_scene, publishing anyway")

                # Publish the planning scene
                scene_pub.publish(planning_scene)
                self._logger.info(
                    f"Loaded {len(planning_scene.world.collision_objects)} obstacles"
                )

                # Brief delay to ensure message is processed
                time.sleep(0.2)

            self._node.destroy_publisher(scene_pub)
            return True

        except Exception as e:
            self._logger.error(f"Failed to load obstacles: {e}")
            self._logger.error(traceback.format_exc())
            return False

    def _set_tool_voltage(self, voltage: int) -> bool:
        """Set tool voltage via raw socket.

        Uses raw socket because this runs BEFORE MoveIt/ROS services are available.
        Connects to UR secondary interface (port 30002) and sends URScript command.

        Args:
            voltage: Tool voltage (0 or 24)

        Returns:
            True if successful, False on failure
        """
        _ALLOWED_VOLTAGES = {0, 12, 24}
        if int(voltage) not in _ALLOWED_VOLTAGES:
            self._logger.error(
                f"Invalid tool voltage {voltage}. Allowed: {_ALLOWED_VOLTAGES}"
            )
            return False

        if not self._robot_ip:
            self._logger.error("Robot IP not set")
            return False

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.SOCKET_TIMEOUT)

            self._logger.info(f"Connecting to {self._robot_ip}:{self.UR_SECONDARY_PORT}")
            sock.connect((self._robot_ip, self.UR_SECONDARY_PORT))

            cmd = f"set_tool_voltage({int(voltage)})\n"
            sock.sendall(cmd.encode())

            sock.close()
            self._logger.info(f"Tool voltage set to {voltage}V")
            return True

        except socket.timeout:
            self._logger.error(f"Timeout connecting to {self._robot_ip}:{self.UR_SECONDARY_PORT}")
            return False
        except socket.error as e:
            self._logger.error(f"Socket error: {e}")
            return False
        except Exception as e:
            self._logger.error(f"Failed to set tool voltage: {e}")
            return False

    def _restart_external_control(self) -> bool:
        """Restart UR external_control program via dashboard service.

        The tool voltage command stops the external_control program,
        so we need to restart it before robot can execute trajectories.

        Returns:
            True if successful, False on failure
        """
        try:
            client = self._node.create_client(Trigger, "/dashboard_client/play")

            if not client.wait_for_service(timeout_sec=5.0):
                self._logger.error("Dashboard play service not available")
                self._node.destroy_client(client)
                return False

            self._logger.info("Calling /dashboard_client/play...")

            # Fire-and-forget: send command, don't wait for response
            # The robot connects asynchronously and we verify via trajectory execution
            client.call_async(Trigger.Request())

            # Brief delay to let robot process the command
            time.sleep(1.0)

            self._node.destroy_client(client)
            self._logger.info("External control program restart requested")
            return True

        except Exception as e:
            self._logger.error(f"Failed to restart external_control: {e}")
            return False
