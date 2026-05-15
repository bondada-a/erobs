"""MoveIt lifecycle manager for dynamic gripper reconfiguration.

Swaps MoveIt configs at runtime by killing the running instance, setting
tool voltage on the UR controller, relaunching MoveIt with the new
gripper config, reloading collision obstacles, restarting external_control,
and verifying hardware is live before accepting goals.

Invariant: the running MoveIt always matches the currently-attached gripper.
"""

import atexit
import os
import signal
import socket
import subprocess
import time
import traceback

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf_transformations import quaternion_from_euler


class MoveItLifecycleManager:
    """Manages MoveIt move_group lifecycle for gripper-specific configurations."""

    # UR robot constants
    UR_SECONDARY_PORT = 30002  # URScript command port
    SOCKET_TIMEOUT = 2.0
    ALLOWED_TOOL_VOLTAGES = frozenset({0, 12, 24})

    @property
    def ARM_JOINTS(self) -> tuple:
        """Arm joint names from the active beamline YAML."""
        from beambot.config_loader import arm_joint_names
        return tuple(arm_joint_names())

    def __init__(self, node: Node, grippers: dict, robot_ip: str, callback_group=None,
                 use_mock_hardware: bool = False):
        """Initialize the lifecycle manager.

        Args:
            node: ROS node for logging and service checks
            grippers: Dict of gripper_name -> {moveit_package, tool_voltage, gripper_group}
            robot_ip: Robot IP address (constant for beamline)
            callback_group: Optional callback group for service clients
            use_mock_hardware: If True, launch MoveIt in simulation mode (no real robot)
        """
        self._node = node
        self._logger = node.get_logger()
        self._grippers = grippers
        self._robot_ip = robot_ip
        self._callback_group = callback_group
        self._use_mock_hardware = use_mock_hardware

        self._moveit_process: subprocess.Popen | None = None
        self._current_gripper: str = ""
        self._current_voltage: int | None = None

        # Persistent joint state subscription for hardware verification.
        # Created once to avoid create/destroy races with MultiThreadedExecutor.
        self._joint_positions: dict = {}
        # Tracks the UR external_control program running state. The
        # subscription is created per-launch inside _restart_external_control
        # (not persistent) because a persistent subscription misses the True
        # edge after a ros2_control_node restart — DDS discovery for the new
        # publisher lags and the 5s gate can time out on a good run.
        self._robot_program_running: bool = False
        if not use_mock_hardware:
            self._node.create_subscription(
                JointState, "/joint_states", self._joint_state_cb, 10,
                callback_group=self._callback_group,
            )

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

    def notify_voltage_change(self, voltage: int):
        """Update cached voltage state when orchestrator sets voltage via set_io."""
        self._current_voltage = voltage

    def _joint_state_cb(self, msg):
        """Cache arm joint positions from /joint_states."""
        for name, pos in zip(msg.name, msg.position):
            if name in self.ARM_JOINTS:
                self._joint_positions[name] = pos

    def launch_moveit_with_gripper(self, gripper: str) -> bool:
        """Launch MoveIt for the specified gripper, verifying hardware is live.

        Reuses the running process if the gripper matches; otherwise kills it
        and relaunches. On real hardware, retries once — ur_ros2_control_node
        occasionally crashes silently on startup (stale TCP socket to the
        robot) and the launch parent stays alive with dead joint states.

        Returns True if MoveIt is up and the hardware interface is connected.
        """
        if self._moveit_process:
            if self._current_gripper == gripper and self._moveit_process.poll() is None:
                self._logger.info(f"MoveIt already running for {gripper}, reusing")
                return True
            self._logger.info(f"Switching gripper: {self._current_gripper} → {gripper}")
            self.kill_current_process()

        if self._attempt_launch(gripper):
            return True

        if self._use_mock_hardware:
            return False

        self._logger.error("Launch failed, retrying once")
        self.kill_current_process()
        return self._attempt_launch(gripper)

    def _attempt_launch(self, gripper: str) -> bool:
        """Single launch attempt: voltage → MoveIt → verify hardware.

        Returns True if MoveIt is up and hardware interface is connected.
        """
        config = self._grippers[gripper]
        self._logger.info(f"Launching MoveIt for {gripper} ({config['moveit_package']})")

        # Set tool voltage BEFORE MoveIt launches so ur_ros2_control_node
        # activates gripper Modbus at the correct voltage.
        if not self._use_mock_hardware:
            desired_voltage = int(config["tool_voltage"])
            if self._current_voltage == desired_voltage:
                self._logger.info(f"Tool voltage already at {desired_voltage}V, skipping")
            elif not self._set_tool_voltage(desired_voltage):
                return False
            else:
                self._current_voltage = desired_voltage
                # Wait for gripper hardware to power up — without this delay,
                # Hand-E/ePick activation fails with Modbus errors.
                time.sleep(2.0)

        # Launch MoveIt as a subprocess. LaunchService can't run alongside an
        # rclpy executor in the same process (signal-handler conflicts, main-
        # thread-only), and its shutdown() leaks children. `ros2 launch` is
        # the maintainer-sanctioned pattern for this use case — see
        # github.com/ros2/launch issues #126, #545, #724.
        try:
            gripper_arg = config.get("gripper_arg", gripper)
            cmd = [
                "ros2", "launch", config["moveit_package"], "robot_bringup.launch.py",
                f"robot_ip:={self._robot_ip}",
                f"use_mock_hardware:={'true' if self._use_mock_hardware else 'false'}",
                f"gripper:={gripper_arg}",
            ]

            cup_profile = config.get("cup_profile")
            if cup_profile:
                cmd.append(f"cup_profile:={cup_profile}")

            self._logger.info(f"Executing: {' '.join(cmd)}")
            # start_new_session=True puts MoveIt in its own process group so
            # kill_current_process() can signal the whole tree via killpg().
            self._moveit_process = subprocess.Popen(cmd, start_new_session=True)
        except Exception as e:
            self._logger.error(f"Failed to launch MoveIt process: {e}")
            return False

        if not self._wait_for_moveit_ready(timeout_sec=45.0):
            return False

        if not self._load_collision_obstacles():
            return False

        # Always restart external_control after a fresh MoveIt launch.
        # Killing the old MoveIt terminates ur_ros2_control_node, which drops
        # the external_control connection — even if voltage didn't change.
        if not self._use_mock_hardware:
            if not self._restart_external_control():
                return False
            if not self._verify_hardware_connected():
                return False

        self._current_gripper = gripper
        self._logger.info(f"Robot ready with {gripper} configuration")
        return True

    def kill_current_process(self):
        """Kill the current MoveIt process and wait for its servers to drop
        from DDS discovery.

        Killing the subprocess doesn't immediately evict its /execute_trajectory
        action server from our node's discovery cache. Without the drain, the
        next _wait_for_moveit_ready would return True on the stale cached
        entry — and subsequent calls (e.g. /apply_planning_scene) would then
        hit a not-yet-ready move_group.
        """
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

        self._drain_stale_execute_trajectory()

    def _drain_stale_execute_trajectory(self, max_wait_sec: float = 10.0):
        """Wait until /execute_trajectory is not advertised to our node."""
        deadline = time.monotonic() + max_wait_sec
        while time.monotonic() < deadline:
            probe = ActionClient(
                self._node, ExecuteTrajectory, "/execute_trajectory",
                callback_group=self._callback_group,
            )
            try:
                if not probe.wait_for_server(timeout_sec=0.5):
                    return
            finally:
                probe.destroy()
        self._logger.warning(
            "Stale /execute_trajectory still advertised after "
            f"{max_wait_sec:.0f}s drain; readiness check may fire early"
        )

    def _wait_for_moveit_ready(self, timeout_sec: float = 45.0) -> bool:
        """Wait for MoveIt to be ready to accept trajectories.

        Waits for the /execute_trajectory action server to be advertised.
        This is the signal MoveIt's own MoveGroupInterface client waits on
        at construction — the action server only appears once all move_group
        capabilities are loaded and trajectory execution is wired up.

        Assumes kill_current_process drained any stale server from DDS
        discovery, so a True return reflects the new MoveIt.
        """
        self._logger.info("Waiting for MoveIt to be ready...")

        client = ActionClient(
            self._node, ExecuteTrajectory, "/execute_trajectory",
            callback_group=self._callback_group,
        )
        try:
            if client.wait_for_server(timeout_sec=timeout_sec):
                self._logger.info("MoveIt ready (/execute_trajectory advertised)")
                return True
            self._logger.error(f"MoveIt not ready within {timeout_sec:.0f}s")
            return False
        finally:
            client.destroy()

    def _verify_hardware_connected(self, timeout_sec: float = 10.0) -> bool:
        """Verify ur_ros2_control_node is alive by checking /joint_states.

        When the hardware interface crashes, MoveGroup stays alive but
        joint_state_broadcaster is dead — no messages on /joint_states.
        Uses the persistent _joint_positions dict (updated by _joint_state_cb)
        to avoid create/destroy subscription races with MultiThreadedExecutor.
        """
        self._logger.info("Verifying hardware interface connection...")
        self._joint_positions.clear()

        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if (len(self._joint_positions) == 6 and
                    sum(abs(v) for v in self._joint_positions.values()) > 0.01):
                self._logger.info("Hardware connected (joint states verified)")
                return True
            time.sleep(0.1)

        if not self._joint_positions:
            self._logger.error("No joint states received — hardware interface is dead")
        else:
            self._logger.error(
                f"Joint states suspect (got {len(self._joint_positions)}/6 joints, "
                f"sum(abs)={sum(abs(v) for v in self._joint_positions.values()):.4f})"
            )
        return False

    def _load_collision_obstacles(self) -> bool:
        """Load collision obstacles into the planning scene.

        Reads obstacles from the beamline's scene_file (declared in the
        YAML pointed at by $BEAMBOT_BEAMLINE_CONFIG) and applies them via
        /apply_planning_scene (transactional — no subscriber-discovery race
        like publishing to /planning_scene).
        """
        try:
            from beambot.config_loader import load_beamline_config, resolve_beamline_path
            config, config_path = load_beamline_config()
            scene_rel = config.get("scene_file", "")
            if not scene_rel:
                self._logger.info("No scene_file declared in beamline config; skipping obstacles")
                return True
            scene_path = resolve_beamline_path(scene_rel, config_path)

            if not os.path.exists(scene_path):
                self._logger.error(f"Obstacle config not found: {scene_path}")
                return False
            config_path = scene_path  # rest of method reads from config_path

            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            if not config or "obstacles" not in config:
                self._logger.info("No obstacles defined in config")
                return True

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
                    self._logger.warning(f"Unknown obstacle type: {obs_type}")
                    continue

                collision_object.primitives.append(primitive)
                collision_object.primitive_poses.append(pose)
                planning_scene.world.collision_objects.append(collision_object)

                self._logger.info(
                    f"  - Added {obs_type} '{collision_object.id}' "
                    f"in frame '{collision_object.header.frame_id}'"
                )

            if not planning_scene.world.collision_objects:
                return True

            # Apply the planning scene diff via service call. Unlike publishing
            # to /planning_scene, this is transactional — MoveGroup confirms
            # the update and there's no subscriber-discovery race.
            client = self._node.create_client(
                ApplyPlanningScene, "/apply_planning_scene",
                callback_group=self._callback_group,
            )
            try:
                if not client.wait_for_service(timeout_sec=5.0):
                    self._logger.error("/apply_planning_scene service not available")
                    return False

                request = ApplyPlanningScene.Request()
                request.scene = planning_scene
                future = client.call_async(request)

                # Poll future.done() — see _wait_for_moveit_ready for the
                # reason we can't spin the executor from this callback thread.
                deadline = time.monotonic() + 5.0
                while not future.done() and time.monotonic() < deadline:
                    time.sleep(0.05)

                if not future.done():
                    self._logger.error("/apply_planning_scene call timed out")
                    return False

                response = future.result()
                if not response.success:
                    self._logger.error("MoveGroup rejected the planning scene diff")
                    return False

                self._logger.info(
                    f"Loaded {len(planning_scene.world.collision_objects)} obstacles"
                )
                return True
            finally:
                self._node.destroy_client(client)

        except Exception as e:
            self._logger.error(f"Failed to load obstacles: {e}")
            self._logger.error(traceback.format_exc())
            return False

    def _set_tool_voltage(self, voltage: int) -> bool:
        """Set UR tool voltage via raw socket on the secondary interface.

        Uses a raw socket because this runs BEFORE MoveIt / ROS services are
        available. Allowed values: ALLOWED_TOOL_VOLTAGES (0, 12, 24).
        """
        if int(voltage) not in self.ALLOWED_TOOL_VOLTAGES:
            self._logger.error(
                f"Invalid tool voltage {voltage}. "
                f"Allowed: {sorted(self.ALLOWED_TOOL_VOLTAGES)}"
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
        """Restart the UR external_control program via dashboard service.

        The tool voltage command stops external_control, so it must be
        restarted before the robot can execute trajectories.
        """
        client = self._node.create_client(
            Trigger, "/dashboard_client/play",
            callback_group=self._callback_group,
        )
        # Fresh subscription per call — the publisher inside
        # io_and_status_controller is a new process after each MoveIt
        # relaunch, and a persistent subscription misses its first True edge
        # while DDS discovery catches up to the new publisher.
        self._robot_program_running = False
        program_sub = self._node.create_subscription(
            Bool, "/io_and_status_controller/robot_program_running",
            self._robot_program_cb, 10, callback_group=self._callback_group,
        )
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                self._logger.error("Dashboard play service not available")
                return False

            self._logger.info("Calling /dashboard_client/play...")
            future = client.call_async(Trigger.Request())

            # Poll future.done() — see _wait_for_moveit_ready for the
            # reason we can't spin the executor from this callback thread.
            deadline = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)

            if not future.done():
                self._logger.error("Dashboard /play call timed out")
                return False

            response = future.result()
            if not response.success:
                self._logger.error(f"Dashboard /play rejected: {response.message}")
                return False

            # Gate on the UR driver's own signal that the external_control
            # program is running. Dashboard accepting /play is not the same
            # as the program actually executing — that gap is what used to
            # cause "Can't accept new action goals. Controller is not
            # running." on the first trajectory.
            deadline = time.monotonic() + 5.0
            while not self._robot_program_running and time.monotonic() < deadline:
                time.sleep(0.05)
            if not self._robot_program_running:
                self._logger.error(
                    "robot_program_running=True not observed within 5s after /play"
                )
                return False

            self._logger.info("External control program restarted")
            return True

        except Exception as e:
            self._logger.error(f"Failed to restart external_control: {e}")
            return False
        finally:
            self._node.destroy_subscription(program_sub)
            self._node.destroy_client(client)

    def _robot_program_cb(self, msg):
        """Cache the UR external_control program running state."""
        self._robot_program_running = bool(msg.data)
