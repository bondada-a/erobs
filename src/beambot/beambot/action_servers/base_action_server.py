"""Shared action-server lifecycle and ROS node runner."""

import traceback
import uuid

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class BaseActionServer(Node):
    """Manage goal admission, execution results, and errors for operation servers."""

    def __init__(self, node_name: str, action_name: str, action_type):
        super().__init__(node_name)

        self._executing = False
        self._action_type = action_type
        self._robot_model_revision = ""

        self._stages = self.create_stages()

        # Cancellation defaults to rejection; the orchestrator cancels between tasks.
        self._action_server = ActionServer(
            self,
            action_type,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
        )

        self.get_logger().info(f"{node_name} started on '{action_name}'")

    def create_stages(self):
        """Return operation stages; subclasses must implement this method."""
        raise NotImplementedError("Subclass must implement create_stages()")

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Reserve an idle server and record the goal's model revision."""
        if self._executing:
            self.get_logger().warning("Rejecting goal: server busy")
            return GoalResponse.REJECT
        self._executing = True
        if hasattr(goal_request, "robot_model_revision"):
            # Unique fallback keys prevent model reuse across unversioned goals.
            self._robot_model_revision = (
                goal_request.robot_model_revision or uuid.uuid4().hex
            )
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        """Finalize the goal and release the server, including after exceptions."""
        try:
            result = self._execute(goal_handle)

            if result.success:
                goal_handle.succeed()
                self.get_logger().info("Goal succeeded")
            else:
                goal_handle.abort()
                self.get_logger().error(f"Goal failed: {result.error_message}")

            return result

        except Exception as e:
            # rclpy loggers lack exc_info support; include the traceback explicitly.
            self.get_logger().error(
                f"Exception during execution: {e}\n{traceback.format_exc()}"
            )
            goal_handle.abort()
            return self._action_type.Result(success=False, error_message=str(e))

        finally:
            self._executing = False

    def _execute(self, goal_handle: ServerGoalHandle):
        """Run stages: None means success; a string becomes the result error."""
        error = self._stages.run(goal_handle.request)
        if error is not None:
            return self._action_type.Result(success=False, error_message=error)
        return self._action_type.Result(success=True)


def run_server(server_class, args=None):
    """Run a ROS node with a multithreaded executor and shutdown cleanup."""
    rclpy.init(args=args)
    node = server_class()

    # Multiple threads allow downstream ROS responses while execution waits.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
