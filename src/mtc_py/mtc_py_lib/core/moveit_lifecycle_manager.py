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
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf_transformations import quaternion_from_euler


class MoveItLifecycleManager:
    """Manages MoveIt move_group lifecycle for gripper-specific configurations."""

    # UR robot constants
    UR_SECONDARY_PORT = 30002  # URScript command port
    SOCKET_TIMEOUT = 2.0

    def __init__(self, node: Node, grippers: dict, robot_ip: str):
        """Initialize the lifecycle manager.

        Args:
            node: ROS node for logging and service checks
            grippers: Dict of gripper_name -> {moveit_package, tool_voltage, gripper_group}
            robot_ip: Robot IP address (constant for beamline)
        """
        self._node = node
        self._logger = node.get_logger()
        self._grippers = grippers
        self._robot_ip = robot_ip

        self._moveit_process: Optional[subprocess.Popen] = None
        self._current_gripper: str = ""

        # Ensure MoveIt is killed when orchestrator exits
        atexit.register(self.kill_current_process)

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
        if not self._set_tool_voltage(config["tool_voltage"]):
            self._logger.error("Failed to set tool voltage")
            return False

        # Launch MoveIt subprocess
        try:
            cmd = [
                "ros2", "launch", config["moveit_package"], "robot_bringup.launch.py",
                f"robot_ip:={self._robot_ip}",
            ]
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
            try:
                # Create fresh client each attempt (avoids stale connections)
                client = self._node.create_client(GetPlanningScene, "/get_planning_scene")

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

                # Spin to process the call (with timeout)
                start = time.time()
                while not future.done() and (time.time() - start) < 5.0:
                    rclpy.spin_once(self._node, timeout_sec=0.1)

                self._node.destroy_client(client)

                if future.done() and future.result() is not None:
                    self._logger.info(f"MoveIt ready (verified after {attempt + 1} attempt(s))")
                    return True

                self._logger.debug(f"Service call failed (attempt {attempt + 1}/{max_attempts})")

            except Exception as e:
                self._logger.debug(f"Poll attempt {attempt + 1} failed: {e}")
                try:
                    self._node.destroy_client(client)
                except:
                    pass

            time.sleep(poll_interval)

        self._logger.error("MoveIt not ready within timeout")
        return False

    def _load_collision_obstacles(self) -> bool:
        """Load collision obstacles into the planning scene.

        Loads obstacles from mtc_py/config/beamline_scene.yaml and publishes
        them to the /planning_scene topic.

        Returns:
            True if successful, False on failure
        """
        try:
            # Get obstacle config path
            try:
                mtc_py_share = get_package_share_directory("mtc_py")
                config_path = os.path.join(
                    mtc_py_share, "config", "beamline_scene.yaml"
                )
            except Exception:
                self._logger.error("mtc_py package not found")
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
        if not self._robot_ip:
            self._logger.error("Robot IP not set")
            return False

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.SOCKET_TIMEOUT)

            self._logger.info(f"Connecting to {self._robot_ip}:{self.UR_SECONDARY_PORT}")
            sock.connect((self._robot_ip, self.UR_SECONDARY_PORT))

            cmd = f"set_tool_voltage({voltage})\n"
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
            request = Trigger.Request()
            future = client.call_async(request)

            rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            self._node.destroy_client(client)

            if not future.done():
                self._logger.error("Dashboard play command timeout")
                return False

            result = future.result()
            if not result.success:
                self._logger.error(f"Failed to restart external_control: {result.message}")
                return False

            self._logger.info("External control program restarted")
            return True

        except Exception as e:
            self._logger.error(f"Failed to restart external_control: {e}")
            return False
