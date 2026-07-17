"""MoveIt lifecycle manager for dynamic gripper reconfiguration.

Swaps MoveIt configs at runtime by killing the running instance, setting
tool voltage on the UR controller, relaunching MoveIt with the new
gripper config, reloading collision obstacles, restarting external_control,
and verifying hardware is live before accepting goals.

Invariant: the running MoveIt always matches the currently-attached gripper.
"""

import atexit
import math
import os
import signal
import socket
import subprocess
import time
import traceback
import uuid
import xml.etree.ElementTree as ET
from numbers import Real

import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf_transformations import quaternion_from_euler

from beambot.core import MODEL_REVISION_PREFIX, VERIFIED_MODEL_DESCRIPTION


class MoveItLifecycleManager:
    """Manages MoveIt move_group lifecycle for gripper-specific configurations."""

    # UR robot constants
    UR_SECONDARY_PORT = 30002  # URScript command port
    SOCKET_TIMEOUT = 2.0
    ALLOWED_TOOL_VOLTAGES = frozenset({0, 12, 24})

    @property
    def ARM_JOINTS(self) -> frozenset:
        """Arm joint names from the active beamline YAML, memoized.

        Cached on first access: the joint set is fixed for the process
        lifetime (it comes from the beamline YAML, which does not change
        while running). Previously this re-read and re-parsed the YAML from
        disk on EVERY access — and it is accessed from `_joint_state_cb`,
        which fires per /joint_states message (~500 Hz). Across the
        executor's worker pool that produced a continuous storm of
        yaml.safe_load() calls that thrashed the GIL and starved goal
        planning (proven via py-spy). Memoizing collapses it to one parse,
        and a frozenset makes the per-message membership test O(1).
        """
        cached = getattr(self, "_arm_joints_cache", None)
        if cached is None:
            from beambot.config_loader import arm_joint_names
            cached = frozenset(arm_joint_names())
            self._arm_joints_cache = cached
        return cached

    def __init__(self, node: Node, grippers: dict, robot_ip: str, callback_group=None,
                 use_mock_hardware: bool = False, enable_joystick: bool = False):
        """Initialize the lifecycle manager.

        Args:
            node: ROS node for logging and service checks
            grippers: Dict of gripper_name -> {moveit_package, tool_voltage, gripper_group}
            robot_ip: Robot IP address (constant for beamline)
            callback_group: Optional callback group for service clients
            use_mock_hardware: If True, launch MoveIt in simulation mode (no real robot)
            enable_joystick: If True, launch MoveIt Servo gamepad control
        """
        self._node = node
        self._logger = node.get_logger()
        self._grippers = grippers
        self._robot_ip = robot_ip
        self._callback_group = callback_group
        self._use_mock_hardware = use_mock_hardware
        self._enable_joystick = enable_joystick

        self._moveit_process: subprocess.Popen | None = None
        self._current_gripper: str = ""
        self._current_cup_profile: str = ""
        self._model_revision: str = ""
        self._model_description_publishers: tuple = ()
        self._current_voltage: int | None = None

        # Read the exact descriptions used by move_group. MTC must not load its
        # model from the shared /robot_description topics at a relaunch boundary:
        # URDF and SRDF arrive independently there and can come from different
        # launches. The verified pair is republished below on a managed topic.
        self._move_group_parameters = AsyncParameterClient(
            node, "/move_group", callback_group=callback_group
        )

        # Runtime ePick cup-profile override (set by the orchestrator from the
        # cup_profile ROS param). Kept here as instance state rather than
        # written back into the shared, cached beamline config dict. Empty
        # string ("") means "no override — use the gripper's YAML value".
        self.cup_override: str = ""

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

    @property
    def model_revision(self) -> str:
        """Unique revision for the currently running MoveIt RobotModel."""
        return self._model_revision if self.is_moveit_alive() else ""

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

    def current_arm_joints(self):
        """Latest arm joint positions (radians) in canonical group order, or None.

        Used as the start-state component of the trajectory-cache key. Returns
        None when not all 6 joints are known yet (startup) or under mock
        hardware (no /joint_states subscription is created) — the cache then
        keys on goal+gripper only, which fails toward re-planning rather than a
        wrong replay. Re-maps name->position into the canonical
        arm_joint_names() order (the live /joint_states order differs and
        interleaves a gripper joint).
        """
        from beambot.config_loader import arm_joint_names
        positions = [self._joint_positions.get(n) for n in arm_joint_names()]
        if len(positions) != 6 or any(p is None for p in positions):
            return None
        return positions

    def launch_moveit_with_gripper(self, gripper: str) -> bool:
        """Launch MoveIt for the specified gripper, verifying hardware is live.

        Reuses the running process if its gripper and cup profile match;
        otherwise kills it and relaunches. On real hardware, retries once —
        ur_ros2_control_node occasionally crashes silently on startup (stale
        TCP socket to the robot) and the launch parent stays alive with dead
        joint states.

        Returns True if MoveIt is up and the hardware interface is connected.
        """
        config = self._grippers[gripper]
        cup_profile = self.cup_override or config.get("cup_profile") or ""

        if self._moveit_process:
            if (
                self._current_gripper == gripper
                and self._current_cup_profile == cup_profile
                and self._moveit_process.poll() is None
                and self._model_revision
            ):
                self._logger.info(f"MoveIt already running for {gripper}, reusing")
                return True
            self._logger.info(
                f"Restarting MoveIt model: {self._current_gripper} → {gripper}"
            )
            self.kill_current_process()

        revision = ""
        if self._attempt_launch(gripper, cup_profile):
            revision = self._publish_verified_model(gripper)
        if not revision and not self._use_mock_hardware:
            self._logger.error("Launch failed, retrying once")
            self.kill_current_process()
            if self._attempt_launch(gripper, cup_profile):
                revision = self._publish_verified_model(gripper)

        if revision:
            self._current_gripper = gripper
            self._current_cup_profile = cup_profile
            self._model_revision = revision
            self._logger.info(f"Robot ready with {gripper} configuration")
        return bool(revision)

    def _publish_verified_model(self, gripper: str) -> str:
        """Publish move_group's verified URDF/SRDF pair on the managed topics."""
        descriptions = self._wait_for_verified_model(gripper)
        if descriptions is None:
            return ""

        robot_description, semantic_description = descriptions
        revision = f"{MODEL_REVISION_PREFIX}{gripper}__{uuid.uuid4().hex}"
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        publishers = (
            self._node.create_publisher(String, VERIFIED_MODEL_DESCRIPTION, qos),
            self._node.create_publisher(
                String, f"{VERIFIED_MODEL_DESCRIPTION}_semantic", qos
            ),
        )
        publishers[0].publish(String(data=robot_description))
        publishers[1].publish(String(data=semantic_description))
        self._model_description_publishers = publishers
        self._logger.info(f"Published verified {gripper} model for revision {revision}")
        return revision

    def _wait_for_verified_model(
        self, gripper: str, timeout_sec: float = 10.0
    ) -> tuple[str, str] | None:
        """Wait until move_group exposes descriptions for the requested tool."""
        deadline = time.monotonic() + timeout_sec
        if not self._move_group_parameters.wait_for_services(timeout_sec=timeout_sec):
            self._logger.error("move_group parameter services are not available")
            return None

        last_error = "move_group returned no robot descriptions"
        while time.monotonic() < deadline:
            future = self._move_group_parameters.get_parameters(
                ["robot_description", "robot_description_semantic"]
            )
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            if not future.done():
                break

            try:
                values = future.result().values
                descriptions = tuple(parameter_value_to_python(v) for v in values)
                if len(descriptions) != 2 or not all(
                    isinstance(value, str) and value for value in descriptions
                ):
                    last_error = "move_group returned empty robot descriptions"
                else:
                    last_error = self._model_content_error(
                        gripper, descriptions[0], descriptions[1]
                    )
                    if not last_error:
                        return descriptions
            except Exception as error:
                last_error = f"could not read move_group descriptions: {error}"
            time.sleep(0.05)

        self._logger.error(
            f"Robot model for {gripper} not ready within {timeout_sec:.0f}s: "
            f"{last_error}"
        )
        return None

    def _model_content_error(self, gripper: str, urdf: str, srdf: str) -> str:
        """Return why descriptions do not match the configured tool, or empty."""
        try:
            urdf_root = ET.fromstring(urdf)
            srdf_root = ET.fromstring(srdf)
        except ET.ParseError as error:
            return f"invalid robot description XML: {error}"

        config = self._grippers[gripper]
        links = {link.get("name"): link for link in urdf_root.findall("link")}
        expected_tip = config.get("tip_frame", "")
        if expected_tip and expected_tip not in links:
            return f"expected link '{expected_tip}' is missing"

        known_tips = {
            item.get("tip_frame")
            for item in self._grippers.values()
            if item.get("tip_frame") and item.get("tip_frame") != "flange"
        }
        expected_tips = {expected_tip} if expected_tip != "flange" else set()
        if known_tips & links.keys() != expected_tips:
            return (
                f"tool links do not match {gripper}: expected "
                f"{sorted(expected_tips)}, got {sorted(known_tips & links.keys())}"
            )

        groups = {group.get("name"): group for group in srdf_root.findall("group")}
        known_groups = {
            item.get("gripper_group")
            for item in self._grippers.values()
            if item.get("gripper_group")
        }
        expected_group = config.get("gripper_group", "")
        expected_groups = {expected_group} if expected_group else set()
        if known_groups & groups.keys() != expected_groups:
            return (
                f"tool groups do not match {gripper}: expected "
                f"{sorted(expected_groups)}, got {sorted(known_groups & groups.keys())}"
            )

        expected_states = set((config.get("states") or {}).values())
        actual_states = {
            state.get("name")
            for state in srdf_root.findall("group_state")
            if state.get("group") == expected_group
        }
        if not expected_states <= actual_states:
            return f"expected named states are missing: {sorted(expected_states - actual_states)}"

        if expected_group:
            tool_links = {
                link.get("name") for link in groups[expected_group].findall("link")
            }
            missing_links = tool_links - links.keys()
            if missing_links:
                return f"SRDF tool links are missing from URDF: {sorted(missing_links)}"
        elif expected_tip and expected_tip != "flange":
            parents = {
                joint.find("child").get("link"): joint.find("parent").get("link")
                for joint in urdf_root.findall("joint")
                if joint.find("child") is not None and joint.find("parent") is not None
            }
            tool_links = set()
            link = expected_tip
            while link and link != "flange" and link not in tool_links:
                tool_links.add(link)
                link = parents.get(link)
        else:
            tool_links = set()

        if tool_links and not any(
            links[name].find("collision/geometry") is not None
            for name in tool_links
        ):
            return f"tool collision geometry is missing for {gripper}"
        return ""

    def _attempt_launch(self, gripper: str, cup_profile: str) -> bool:
        """Single launch attempt: voltage → MoveIt → verify hardware.

        Returns True if MoveIt is up and hardware interface is connected.
        """
        config = self._grippers[gripper]
        if not self._validate_payload(config):
            return False
        self._logger.info(f"Launching MoveIt for {gripper} ({config['moveit_package']})")

        # Set tool voltage BEFORE MoveIt launches so ur_ros2_control_node
        # activates gripper Modbus at the correct voltage. tool_voltage is
        # optional in the YAML — passive/mechanical grippers without UR-IO
        # tool control can omit it; we default to 0V (TOOL_OUTPUT_OFF).
        if not self._use_mock_hardware:
            desired_voltage = int(config.get("tool_voltage", 0))
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
                f"enable_joystick:={'true' if self._enable_joystick else 'false'}",
                f"gripper:={gripper_arg}",
            ]

            # Runtime override (cup_override) wins over the gripper's YAML value.
            # Only ePick's launch consumes cup_profile:=; it's an inert,
            # default-valued arg for every other gripper's launch.
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
            # Set the arm payload now that io_and_status_controller is up —
            # gated on the service, not a fixed launch-time timer.
            if not self._set_payload(config):
                return False

        return True

    def _validate_payload(self, config: dict) -> bool:
        """Require a finite positive mass and finite three-axis CoG."""
        try:
            values = (
                config["payload_mass"],
                config["payload_cog"]["x"],
                config["payload_cog"]["y"],
                config["payload_cog"]["z"],
            )
            valid = (
                all(
                    not isinstance(value, bool)
                    and isinstance(value, Real)
                    and math.isfinite(value)
                    for value in values
                )
                and values[0] > 0
            )
        except (KeyError, TypeError, ValueError, OverflowError):
            valid = False

        if not valid:
            self._logger.error(
                "Payload requires a finite positive payload_mass and finite numeric "
                "payload_cog x, y, and z values"
            )
            return False
        return True

    def _set_payload(self, config: dict) -> bool:
        """Set the validated arm payload before declaring the robot ready."""
        mass = config["payload_mass"]
        cog = config["payload_cog"]

        from ur_msgs.srv import SetPayload
        from geometry_msgs.msg import Vector3

        client = None
        try:
            client = self._node.create_client(
                SetPayload, "/io_and_status_controller/set_payload",
                callback_group=self._callback_group,
            )
            if not client.wait_for_service(timeout_sec=5.0):
                self._logger.error("set_payload service not available")
                return False
            request = SetPayload.Request()
            request.mass = float(mass)
            request.center_of_gravity = Vector3(
                x=float(cog["x"]), y=float(cog["y"]), z=float(cog["z"])
            )
            future = client.call_async(request)
            deadline = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            if not future.done():
                self._logger.error("set_payload call timed out")
                return False

            response = future.result()
            if response is None or not response.success:
                self._logger.error("set_payload call failed")
                return False

            self._logger.info(f"Payload set to {mass}kg")
            return True
        except Exception as error:
            self._logger.error(f"set_payload failed: {error}")
            return False
        finally:
            if client is not None:
                self._node.destroy_client(client)

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
            self._current_gripper = ""
            self._current_cup_profile = ""
            self._model_revision = ""
            self._clear_model_description_publishers()
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
        self._current_cup_profile = ""
        self._model_revision = ""
        self._clear_model_description_publishers()

        self._drain_stale_execute_trajectory()

    def _clear_model_description_publishers(self):
        """Stop serving the description pair for the retired revision."""
        for publisher in getattr(self, "_model_description_publishers", ()):
            self._node.destroy_publisher(publisher)
        self._model_description_publishers = ()

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
