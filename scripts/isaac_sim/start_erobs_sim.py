#!/usr/bin/env python3
"""Launch the EROBS UR5e in Isaac Sim with a ROS 2 joint-command graph.

Run with Isaac Sim's WebRTC streaming application::

    /path/to/isaacsim/isaac-sim.streaming.sh --exec scripts/isaac_sim/start_erobs_sim.py

The GUI is enabled by default.  The ROS graph publishes JointState feedback and
simulation time and accepts JointState position/velocity/effort commands.
"""

import argparse
import math
import os
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
URDF_DIR = REPO_ROOT / "src/custom-ur-descriptions/cms_robot_description/urdf"
MODEL_STEMS = {
    "none": "ur_standalone",
    "hande": "ur_with_zivid_hande",
    "epick": "ur_with_zivid_epick",
    "2fg7": "ur_with_zivid_2fg7",
    "pipettor": "ur_with_zivid_pipettor",
}
SOURCE_URDFS = {
    name: URDF_DIR / f"{stem}.urdf" for name, stem in MODEL_STEMS.items()
}
ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
SAFE_SAMPLE_TRANSPORT_DEGREES = (
    79.72,
    -69.5,
    -81.91,
    -117.92,
    -268.05,
    -157.15,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-usd", type=Path)
    parser.add_argument("--robot-prim", default="/World/UR5e")
    parser.add_argument(
        "--articulation-prim",
        default=None,
        help="Articulation root path (default: discover it below --robot-prim)",
    )
    parser.add_argument("--joint-state-topic", default="joint_states")
    parser.add_argument("--joint-command-topic", default="isaac_joint_commands")
    parser.add_argument("--clock-topic", default="clock")
    parser.add_argument("--gripper-topic", default="beambot/current_gripper")
    parser.add_argument(
        "--asset-cache", type=Path, default=URDF_DIR / "generated_isaac",
        help="Output directory for models that have not been imported yet",
    )
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--physics-hz", type=float, default=60.0)
    parser.add_argument(
        "--initial-arm-degrees",
        type=float,
        nargs=6,
        default=SAFE_SAMPLE_TRANSPORT_DEGREES,
        metavar="DEG",
        help="Initial arm joint pose (default: safe_sample_transport)",
    )
    parser.add_argument("--headless", action="store_true", help="Disable the Isaac Sim GUI")
    parser.add_argument("--renderer", default="RaytracedLighting")
    parser.add_argument(
        "--test-frames",
        type=int,
        default=0,
        help="Exit after this many frames (0 runs until the window is closed)",
    )
    args, _ = parser.parse_known_args()
    if args.physics_hz <= 0:
        parser.error("--physics-hz must be positive")
    if args.domain_id < 0 or args.domain_id > 232:
        parser.error("--domain-id must be between 0 and 232")
    if args.robot_usd is not None:
        args.robot_usd = args.robot_usd.expanduser().resolve()
        if not args.robot_usd.is_file():
            parser.error(f"robot USD does not exist: {args.robot_usd}")
    args.asset_cache = args.asset_cache.expanduser().resolve()
    if not args.robot_prim.startswith("/"):
        parser.error("--robot-prim must be an absolute USD path")
    return args


args = parse_args()
initial_arm_positions = dict(zip(ARM_JOINTS, map(math.radians, args.initial_arm_degrees)))

# Under python.sh we create the application. With streaming.sh --exec, Kit has
# already created its WebRTC application and this file installs update hooks.
try:
    import omni.kit.app

    kit_app = omni.kit.app.get_app()
    running_in_kit = kit_app.is_running()
except (ImportError, RuntimeError):
    kit_app = None
    running_in_kit = False

if running_in_kit:
    simulation_app = None
else:
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless, "renderer": args.renderer})

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import usdrt.Sdf
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationManager


GRAPH_PATH = "/World/EROBS_ROS2_ActionGraph"
ROS_GRAPH_NODES = (
    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
    ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
    ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
)
articulation_prim = args.articulation_prim
current_gripper = None
requested_gripper = "none"
last_arm_positions = dict(initial_arm_positions)
restore_arm_positions = dict(initial_arm_positions)
hold_frames = 0
articulation_handle = None
pose_pending = False
pending_model = None
ros_context = (None, None, None)
update_subscription = None


def app_update() -> None:
    if simulation_app is not None:
        simulation_app.update()


def ros_package_share(package: str) -> Path:
    """Resolve a ROS package without assuming its install prefix or distro."""
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError as exc:
        raise RuntimeError(
            "ament_index_python is unavailable; source your ROS 2 environment "
            "before starting Isaac Sim"
        ) from exc
    try:
        return Path(get_package_share_directory(package))
    except LookupError as exc:
        distro = os.environ.get("ROS_DISTRO", "<distro>")
        ros_package = package.replace("_", "-")
        raise RuntimeError(
            f"ROS package {package!r} was not found; install "
            f"ros-{distro}-{ros_package} and restart this launcher"
        ) from exc


def package_paths() -> list[dict[str, str]]:
    """Mappings used by Isaac 6 to resolve package:// mesh URLs."""
    paths = {
        "ur_description": ros_package_share("ur_description"),
        "cms_robot_description": REPO_ROOT / "src/custom-ur-descriptions/cms_robot_description",
        "zivid_description": REPO_ROOT / "src/vision/zivid-ros/zivid_description",
        "robotiq_hande_description": REPO_ROOT / "src/end_effectors/robotiq_hande_description",
        "epick_description": REPO_ROOT / "src/end_effectors/ros2_epick_gripper/epick_description",
        "pipette_description": REPO_ROOT / "src/end_effectors/pipettor/pipette_description",
        "onrobot_2fg7_description": REPO_ROOT / "src/end_effectors/onrobot_2fg7_description",
    }
    return [{"name": name, "path": str(path)} for name, path in paths.items() if path.is_dir()]


def model_usd(gripper: str) -> Path:
    """Return a cached USD, importing its normal package:// URDF if needed."""
    if gripper == "none" and args.robot_usd is not None:
        return args.robot_usd
    source_stem = SOURCE_URDFS[gripper].stem
    cached = args.asset_cache / source_stem / f"{source_stem}.usda"
    if cached.is_file():
        return cached

    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    args.asset_cache.mkdir(parents=True, exist_ok=True)
    stiffness = {
        ".*shoulder_pan_joint": 9400.5,
        ".*shoulder_lift_joint": 10020.9,
        ".*elbow_joint": 10230.2,
        ".*wrist_1_joint": 3940.6,
        ".*wrist_2_joint": 3940.6,
        ".*wrist_3_joint": 1000.1,
    }
    damping = {
        ".*shoulder_pan_joint": 0.378,
        ".*shoulder_lift_joint": 0.412,
        ".*elbow_joint": 4.093,
        ".*wrist_1_joint": 1.579,
        ".*wrist_2_joint": 0.061,
        ".*wrist_3_joint": 0.004,
    }
    gripper_pattern = {
        "hande": ".*left_finger_joint",
        "epick": ".*gripper",
        "2fg7": ".*left_finger_joint",
    }.get(gripper)
    if gripper_pattern:
        stiffness[gripper_pattern] = 1000.0
        damping[gripper_pattern] = 100.0 if gripper == "epick" else 1000.0
    config = URDFImporterConfig(
        urdf_path=str(SOURCE_URDFS[gripper]),
        usd_path=str(args.asset_cache),
        ros_package_paths=package_paths(),
        fix_base=True,
        merge_fixed_joints=True,
        collision_from_visuals=True,
        allow_self_collision=True,
        joint_drive_type="force",
        joint_target_type="position",
        override_joint_stiffness=stiffness,
        override_joint_damping=damping,
    )
    result = Path(URDFImporter(config).import_urdf())
    carb.log_info(f"Imported {gripper} model from {SOURCE_URDFS[gripper]} to {result}")
    return result


def discover_articulation() -> str:
    import omni.usd
    from pxr import Usd, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    container = stage.GetPrimAtPath(args.robot_prim)
    roots = [
        str(prim.GetPath())
        for prim in Usd.PrimRange(container)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(roots) != 1:
        raise RuntimeError(
            f"expected one articulation below {args.robot_prim}, found {roots}; "
            "pass --articulation-prim explicitly if the asset has multiple roots"
        )
    return roots[0]


def apply_arm_pose() -> None:
    """Teleport the articulation to its preserved arm pose once physics is ready."""
    global articulation_handle, pose_pending
    from isaacsim.core.experimental.prims import Articulation

    if articulation_handle is None:
        articulation_handle = Articulation(articulation_prim)
    if not articulation_handle.is_physics_tensor_entity_valid():
        pose_pending = True
        return
    dof_indices = [articulation_handle.dof_names.index(name) for name in ARM_JOINTS]
    articulation_handle.set_dof_positions(
        [restore_arm_positions[name] for name in ARM_JOINTS], dof_indices=dof_indices
    )
    articulation_handle.set_dof_velocities([0.0] * len(ARM_JOINTS), dof_indices=dof_indices)
    pose_pending = False


def build_ros_graph() -> None:
    """Create the ROS 2 bridge graph used by the trajectory adapter."""
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: list(ROS_GRAPH_NODES),
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
                ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
                ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
                ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
                ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
                ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
                ("ReadJointState.outputs:stageMetersPerUnit", "PublishJointState.inputs:stageMetersPerUnit"),
                ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Context.inputs:domain_id", args.domain_id),
                ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(articulation_prim)]),
                ("ArticulationController.inputs:robotPath", articulation_prim),
                ("PublishJointState.inputs:topicName", args.joint_state_topic),
                ("SubscribeJointState.inputs:topicName", args.joint_command_topic),
                ("PublishClock.inputs:topicName", args.clock_topic),
            ],
        },
    )


def load_model(gripper: str, *, first: bool = False) -> None:
    """Replace the robot reference and keep the action graph on its articulation."""
    global articulation_handle, articulation_prim, current_gripper, hold_frames, pending_model
    global restore_arm_positions
    import omni.usd

    if gripper not in MODEL_STEMS or gripper == current_gripper:
        return
    if not hold_frames:
        restore_arm_positions = {
            name: last_arm_positions.get(name, initial_arm_positions[name])
            for name in ARM_JOINTS
        }
    usd = model_usd(gripper)
    articulation_handle = None
    app_utils.stop()
    if not first:
        # Kit must process one update after stop before any prim used by a
        # physics tensor view is removed. tick() finishes the swap next update.
        pending_model = (gripper, usd)
        return
    app_update()
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(args.robot_prim):
        stage.RemovePrim(args.robot_prim)
        app_update()
    stage_utils.add_reference_to_stage(str(usd), args.robot_prim)
    app_update()
    articulation_prim = args.articulation_prim or discover_articulation()
    current_gripper = gripper
    hold_frames = 60
    carb.log_info(f"EROBS model switched to {gripper}: {usd} ({articulation_prim})")


def finish_model_swap() -> None:
    """Replace the stopped articulation after Kit released its tensor views."""
    global articulation_prim, current_gripper, hold_frames, pending_model, pose_pending
    import omni.usd

    gripper, usd = pending_model
    stage = omni.usd.get_context().get_stage()
    stage.RemovePrim(GRAPH_PATH)
    stage.RemovePrim(args.robot_prim)
    stage_utils.add_reference_to_stage(str(usd), args.robot_prim)
    articulation_prim = args.articulation_prim or discover_articulation()
    build_ros_graph()
    current_gripper = gripper
    hold_frames = 60
    pending_model = None
    pose_pending = True
    app_utils.play()
    carb.log_info(f"EROBS model switched to {gripper}: {usd} ({articulation_prim})")


def create_ros_node():
    """Observe EROBS' existing gripper state and retain arm pose across swaps."""
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("erobs_isaac_model_manager")
    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )

    def gripper_callback(msg):
        global requested_gripper
        if msg.data in MODEL_STEMS:
            requested_gripper = msg.data

    def joint_callback(msg):
        last_arm_positions.update(
            (name, position)
            for name, position in zip(msg.name, msg.position)
            if name in ARM_JOINTS
        )

    node.create_subscription(String, args.gripper_topic, gripper_callback, qos)
    node.create_subscription(JointState, args.joint_state_topic, joint_callback, 10)
    publisher = node.create_publisher(JointState, args.joint_command_topic, 10)
    return rclpy, node, publisher


def setup() -> None:
    global ros_context
    app_utils.enable_extension("isaacsim.ros2.bridge")
    app_utils.enable_extension("isaacsim.asset.importer.urdf")
    app_update()
    stage_utils.set_stage_units(meters_per_unit=1.0)
    load_model("none", first=True)

    if not args.headless:
        ViewportManager.set_camera_view(
            "/OmniverseKit_Persp", eye=[1.5, 1.5, 1.2], target=[0.0, 0.0, 0.45]
        )

    build_ros_graph()
    app_update()
    SimulationManager.setup_simulation(dt=1.0 / args.physics_hz, device="cpu")
    app_utils.play()
    app_update()
    apply_arm_pose()
    ros_context = create_ros_node()

    carb.log_info(
        f"EROBS UR5e running at {articulation_prim}: feedback=/{args.joint_state_topic.lstrip('/')} "
        f"commands=/{args.joint_command_topic.lstrip('/')} domain={args.domain_id}"
    )


def tick() -> None:
    global hold_frames
    rclpy, ros_node, hold_publisher = ros_context
    rclpy.spin_once(ros_node, timeout_sec=0.0)
    if pending_model is not None:
        finish_model_swap()
        return
    if requested_gripper != current_gripper:
        load_model(requested_gripper)
        return
    if pose_pending:
        apply_arm_pose()
    if hold_frames:
        from sensor_msgs.msg import JointState

        command = JointState()
        command.name = list(ARM_JOINTS)
        command.position = [restore_arm_positions[name] for name in ARM_JOINTS]
        hold_publisher.publish(command)
        hold_frames -= 1


def main() -> int:
    rclpy = ros_node = None
    try:
        setup()
        rclpy, ros_node, _ = ros_context
        frames = 0
        while simulation_app.is_running():
            tick()
            simulation_app.update()
            frames += 1
            if args.test_frames and frames >= args.test_frames:
                break
        return 0
    except Exception as exc:
        carb.log_error(f"Failed to launch EROBS simulation: {exc}")
        return 1
    finally:
        if ros_node is not None:
            ros_node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        app_utils.stop()
        if simulation_app is not None:
            simulation_app.close()


if running_in_kit:
    import builtins

    setup()
    update_subscription = kit_app.get_update_event_stream().create_subscription_to_pop(
        lambda _event: tick(), name="EROBS simulation update"
    )
    # Kit discards the --exec namespace after this file returns.
    builtins._erobs_stream_update_subscription = update_subscription
else:
    sys.exit(main())
