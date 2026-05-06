"""Base class for MTC action servers.

Provides goal lifecycle management, concurrent execution prevention,
and standard error handling for all MTC action servers.
"""

import threading
import traceback

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class BaseActionServer(Node):
    """Base class for MTC action servers.

    Subclasses must implement create_stages() to return a stages object
    whose run(request) yields None on success or an error string on
    failure. Optionally override _execute() for custom goal handling.
    """

    def __init__(self, node_name: str, action_name: str, action_type):
        super().__init__(node_name)

        self._executing = False
        self._lock = threading.Lock()
        self._action_type = action_type

        self._stages = self.create_stages()

        # Note: cancel_callback is omitted - defaults to REJECT. Individual action
        # servers cannot safely cancel mid-execution (MTC/MoveIt is controlling the
        # robot). Cancellation is handled at the orchestrator level (between tasks).
        self._action_server = ActionServer(
            self,
            action_type,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
        )

        self.get_logger().info(f"{node_name} started on '{action_name}'")

    def create_stages(self):
        """Return a stages instance that exposes `.run(request) -> Optional[str]`.

        The returned object's run() should yield None on success or an error
        string on failure. Subclasses must override.
        """
        raise NotImplementedError("Subclass must implement create_stages()")

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Accept goal if not already executing, otherwise reject."""
        with self._lock:
            if self._executing:
                self.get_logger().warning("Rejecting goal: server busy")
                return GoalResponse.REJECT
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute goal with error handling and state management."""
        with self._lock:
            self._executing = True

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
            # rclpy loggers don't support exc_info=, so format the traceback
            # manually. Without this, every action-server error loses its stack
            # trace and only the str(e) line hits the logs — making bugs below
            # the action callback (stage code, MTC, MoveIt) effectively invisible.
            self.get_logger().error(
                f"Exception during execution: {e}\n{traceback.format_exc()}"
            )
            goal_handle.abort()
            return self._action_type.Result(success=False, error_message=str(e))

        finally:
            with self._lock:
                self._executing = False

    def _execute(self, goal_handle: ServerGoalHandle):
        """Execute goal. Override for custom behavior (e.g., logging).

        Stages.run() returns Optional[str]: None on success, error string on failure.
        """
        error = self._stages.run(goal_handle.request)
        if error is not None:
            return self._action_type.Result(success=False, error_message=error)
        return self._action_type.Result(success=True)


def run_server(server_class, args=None):
    """Run an action server with standard ROS 2 lifecycle.

    Uses MultiThreadedExecutor because stages make concurrent ROS calls
    during a single goal (action clients to other servers, service calls,
    TF lookups). A single-threaded spin would serialize all of them and
    stall the execute callback on its own downstream traffic.
    """
    rclpy.init(args=args)
    node = server_class()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
