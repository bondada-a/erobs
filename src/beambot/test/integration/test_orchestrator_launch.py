"""Real ROS action transport; all robot/MTC execution boundaries are inert."""

import json
from pathlib import Path
import sys
import unittest
import uuid

import launch
from launch.actions import ExecuteProcess
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger

from beambot_interfaces.action import BeambotExecution


@pytest.mark.launch_test
def generate_test_description():
    namespace = f"/beambot_action_test_{uuid.uuid4().hex}"
    fixture = ExecuteProcess(
        cmd=[sys.executable, str(Path(__file__).with_name("orchestrator_fixture.py"))],
        additional_env={"BEAMBOT_TEST_NAMESPACE": namespace, "PYTHONUNBUFFERED": "1"},
        output="screen",
    )
    return launch.LaunchDescription([
        fixture, launch_testing.actions.ReadyToTest(),
    ]), {"fixture": fixture, "namespace": namespace}


class TestOrchestratorActions(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(f"orchestrator_action_test_{uuid.uuid4().hex}")
        self.action = None

    def tearDown(self):
        if self.action is not None:
            self.action.destroy()
        self.node.destroy_node()

    def connect(self, namespace):
        self.action = ActionClient(self.node, BeambotExecution, f"{namespace}/beambot_execution")
        self.trace = self.node.create_client(Trigger, f"{namespace}/fixture/trace")
        self.release = self.node.create_client(Trigger, f"{namespace}/fixture/release")
        self.assertTrue(self.action.wait_for_server(timeout_sec=10.0))
        self.assertTrue(self.trace.wait_for_service(timeout_sec=5.0))
        self.assertTrue(self.release.wait_for_service(timeout_sec=5.0))

    def wait(self, future):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done(), "ROS action/service did not finish within 10 seconds")
        return future.result()

    def send(self, targets):
        goal = BeambotExecution.Goal(full_json=json.dumps({
            "start_gripper": "test",
            "tasks": [{"task_type": "moveto", "target": target} for target in targets],
        }))
        return self.wait(self.action.send_goal_async(goal))

    def finish(self, goal, *, status=GoalStatus.STATUS_SUCCEEDED, completed=1, total=1):
        self.assertTrue(goal.accepted)
        response = self.wait(goal.get_result_async())
        self.assertEqual(response.status, status)
        self.assertEqual(response.result.success, status == GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(response.result.completed_steps, completed)
        self.assertEqual(response.result.total_steps, total)
        return response.result

    def targets(self, prefix):
        response = self.wait(self.trace.call_async(Trigger.Request()))
        self.assertTrue(response.success)
        return [target for target in json.loads(response.message) if target.startswith(prefix)]

    def test_a_b_c_succeed_in_order(self, namespace):
        self.connect(namespace)
        targets = ["success:A", "success:B", "success:C"]
        self.finish(self.send(targets), completed=3, total=3)
        self.assertEqual(self.targets("success:"), targets)

    def test_b_failure_skips_c_and_next_goal_is_admitted(self, namespace):
        self.connect(namespace)
        result = self.finish(
            self.send(["failure:A", "failure:B", "failure:C"]),
            status=GoalStatus.STATUS_ABORTED, completed=1, total=3,
        )
        self.assertIn("Injected stage B failure", result.error_message)
        self.assertEqual(self.targets("failure:"), ["failure:A", "failure:B"])
        self.finish(self.send(["recovered_after_failure"]))

    def test_busy_rejection_then_cancel_between_tasks_and_readmit(
        self, namespace, proc_output, fixture,
    ):
        self.connect(namespace)
        active = self.send(["held:A", "held:B"])
        self.assertTrue(active.accepted)
        try:
            proc_output.assertWaitFor(
                "STAGE held:A", process=fixture, timeout=10.0, stream="stdout",
            )
            self.assertFalse(self.send(["busy:must_not_run"]).accepted)
            cancellation = self.wait(active.cancel_goal_async())
            self.assertEqual(cancellation.return_code, 0)
            self.assertEqual(len(cancellation.goals_canceling), 1)
            self.assertEqual(self.targets("held:"), ["held:A"])
        finally:
            self.assertTrue(self.wait(self.release.call_async(Trigger.Request())).success)
            # Drain an accepted goal even if an assertion fails before cancellation.
            self.wait(active.get_result_async())
        self.finish(active, status=GoalStatus.STATUS_CANCELED, completed=1, total=2)
        self.assertEqual(self.targets("held:"), ["held:A"])
        self.assertEqual(self.targets("busy:"), [])
        self.finish(self.send(["recovered_after_cancel"]))


@launch_testing.post_shutdown_test()
class TestFixtureShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info, fixture):
        launch_testing.asserts.assertExitCodes(proc_info, process=fixture)
