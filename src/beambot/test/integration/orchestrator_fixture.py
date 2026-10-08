"""Launch real action servers with inert task/lifecycle boundaries only."""

import json
import os
from pathlib import Path
import threading
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch


def main():
    # Discard any real beamline configuration and command-line ROS parameters.
    with TemporaryDirectory(prefix="beambot-action-fixture-") as directory:
        config_path = Path(directory) / "beamline.json"
        config_path.write_text(json.dumps({
            "beamline": "action_transport_test",
            "robot": {"ip": "127.0.0.1", "arm_group": "ur_arm"},
            "grippers": {"test": {}},
            "vision_targets": {},
        }))
        os.environ["BEAMBOT_BEAMLINE_CONFIG"] = str(config_path)
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        os.environ.pop("ROS_LOCALHOST_ONLY", None)

        import rclcpp
        import rclpy
        from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
        from rclpy.node import Node
        from std_srvs.srv import Trigger

        from beambot.action_servers import move_to_server, orchestrator

        held = threading.Event()
        trace = []

        class InertTask:
            def run(self, goal):
                trace.append(goal.target)
                print(f"STAGE {goal.target}", flush=True)
                if goal.robot_model_revision != "integration-test-model":
                    return "Model revision did not cross the action boundary"
                if goal.target == "held:A" and not held.wait(timeout=12.0):
                    return "Test did not release the held stage"
                if goal.target == "failure:B":
                    return "Injected stage B failure"
                return None

        def lifecycle(*_args, **kwargs):
            assert kwargs["use_mock_hardware"] is True
            return SimpleNamespace(
                cup_override="", model_revision="integration-test-model",
                current_arm_joints=lambda: [0.0] * 6,
                launch_moveit_with_gripper=lambda gripper: gripper == "test",
                is_moveit_alive=lambda: True,
                kill_current_process=lambda **_: None,
            )

        def release(_request, response):
            held.set()
            response.success = True
            return response

        def snapshot(_request, response):
            response.success = True
            response.message = json.dumps(trace)
            return response

        forbidden = AssertionError("Hardware and MTC startup are forbidden in this fixture")
        with (
            patch.object(orchestrator, "MoveItLifecycleManager", lifecycle),
            patch.object(move_to_server, "MoveToTask", return_value=InertTask()),
            patch.object(orchestrator, "MoveToTask", side_effect=forbidden),
            patch.object(orchestrator, "EndEffectorTask", side_effect=forbidden),
            patch.object(rclcpp, "init", side_effect=forbidden),
            patch.object(rclcpp, "Node", side_effect=forbidden),
            patch("subprocess.Popen", side_effect=forbidden),
        ):
            rclpy.init(args=[
                "--ros-args", "-r", f"__ns:={os.environ['BEAMBOT_TEST_NAMESPACE']}",
                "-p", "use_mock_hardware:=true", "-p", "enable_batching:=false",
                "-p", "timeout.moveto:=15.0",
            ])
            control = Node("fixture_control")
            control.create_service(Trigger, "fixture/release", release)
            control.create_service(Trigger, "fixture/trace", snapshot)
            nodes = [control, move_to_server.MoveToActionServer(),
                     orchestrator.BeambotOrchestratorServer()]
            executor = MultiThreadedExecutor(num_threads=6)
            for node in nodes:
                executor.add_node(node)
            print("ACTION FIXTURE READY", flush=True)
            try:
                executor.spin()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            finally:
                held.set()
                executor.shutdown(timeout_sec=5.0)
                for node in reversed(nodes):
                    node.destroy_node()
                rclpy.try_shutdown()


if __name__ == "__main__":
    main()
