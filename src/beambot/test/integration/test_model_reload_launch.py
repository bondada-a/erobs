"""Reload real MTC models over ROS topics; no motion or hardware is launched."""

import os
from pathlib import Path
import sys
import tempfile
import unittest
import uuid

import launch
from launch.actions import ExecuteProcess
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util


def generate_test_description():
    probe = ExecuteProcess(
        cmd=[sys.executable, "-u", str(Path(__file__).resolve()), "--probe"],
        output="screen",
        sigterm_timeout="2",
        sigkill_timeout="2",
    )
    return launch.LaunchDescription([
        probe, launch_testing.util.KeepAliveProc(), launch_testing.actions.ReadyToTest()
    ]), {"probe": probe}


class TestModelReload(unittest.TestCase):
    def test_managed_models_and_collision_geometry_reload(self, proc_output, proc_info, probe):
        proc_output.assertWaitFor("MODEL_RELOAD_OK", process=probe, timeout=30, stream="stdout")
        proc_info.assertWaitForShutdown(process=probe, timeout=5)


@launch_testing.post_shutdown_test()
class TestModelProbeExit(unittest.TestCase):
    def test_clean_exit(self, proc_info, probe):
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)


def _probe():
    # The integration runner reserves an isolated, local-only ROS domain.
    if not os.environ.get("ROS_DOMAIN_ID"):
        raise RuntimeError("Run through the isolated integration runner")
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

    import rclcpp
    import rclpy
    from geometry_msgs.msg import Pose
    from moveit.core.planning_scene import PlanningScene
    from moveit_msgs.msg import CollisionObject
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from shape_msgs.msg import SolidPrimitive
    from std_msgs.msg import String
    import yaml

    with tempfile.TemporaryDirectory(prefix="beambot-model-test-") as directory:
        config = Path(directory) / "beamline.yaml"
        config.write_text(yaml.safe_dump({
            "robot": {"moveit_config_package": "cms_moveit_config", "arm_joints": ["hinge"]},
            "grippers": {
                tool: {"gripper_group": f"{tool}_gripper", "tip_frame": f"{tool}_tip"}
                for tool in ("alpha", "beta")
            },
        }))
        os.environ["BEAMBOT_BEAMLINE_CONFIG"] = str(config)
        from beambot.utils import MODEL_REVISION_PREFIX, VERIFIED_MODEL_DESCRIPTION
        from beambot.motion import task_builder
        from beambot.motion.task_builder import MtcTaskBuilder

        namespace = f"/beambot_model_test_{uuid.uuid4().hex}"
        task_builder._options.arguments = [
            *task_builder._options.arguments, "-r", f"__ns:={namespace}",
        ]
        rclpy.init()
        node = rclpy.create_node("model_reload_fixture", namespace=namespace)
        try:
            qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            urdf_pub = node.create_publisher(String, VERIFIED_MODEL_DESCRIPTION, qos)
            srdf_pub = node.create_publisher(String, f"{VERIFIED_MODEL_DESCRIPTION}_semantic", qos)
            builder = MtcTaskBuilder(node, arm_group="test_arm")

            for tool, width, collides in (("alpha", 0.02, False), ("beta", 0.2, True)):
                urdf = f"""<robot name="synthetic_{tool}">
                  <link name="base_link"/><link name="arm_link"/>
                  <joint name="hinge" type="revolute">
                    <parent link="base_link"/><child link="arm_link"/>
                    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
                  </joint>
                  <link name="{tool}_tip"><collision><geometry>
                    <box size="{width} {width} {width}"/>
                  </geometry></collision></link>
                  <joint name="tool_mount" type="fixed">
                    <parent link="arm_link"/><child link="{tool}_tip"/>
                  </joint>
                </robot>"""
                srdf = f"""<robot name="synthetic_{tool}">
                  <group name="test_arm"><joint name="hinge"/></group>
                  <group name="{tool}_gripper"><link name="{tool}_tip"/></group>
                </robot>"""
                urdf_pub.publish(String(data=urdf))
                srdf_pub.publish(String(data=srdf))
                node._robot_model_revision = f"{MODEL_REVISION_PREFIX}{tool}__integration"

                task = builder.create_task_template(f"model_{tool}")
                model = task.getRobotModel()
                assert model.name == f"synthetic_{tool}"
                assert set(model.joint_model_group_names) == {"test_arm", f"{tool}_gripper"}
                assert set(model.get_joint_model_group(f"{tool}_gripper").link_model_names) == {
                    f"{tool}_tip"
                }

                scene = PlanningScene(model)
                scene.current_state.set_to_default_values()
                scene.current_state.update()
                obstacle = CollisionObject(id="geometry_probe", operation=CollisionObject.ADD)
                obstacle.header.frame_id = "base_link"
                obstacle.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.02] * 3)]
                pose = Pose()
                pose.position.x = 0.075
                pose.orientation.w = 1.0
                obstacle.primitive_poses = [pose]
                scene.apply_collision_object(obstacle)
                assert scene.is_state_colliding(scene.current_state, "", False) is collides
                print(f"MODEL_{tool.upper()}_OK", flush=True)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            rclcpp.shutdown()
    print("MODEL_RELOAD_OK", flush=True)


if __name__ == "__main__":
    if sys.argv[1:] != ["--probe"]:
        raise SystemExit("Use launch_test through the isolated integration runner")
    _probe()
