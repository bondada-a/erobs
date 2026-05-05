"""Base class for MTC action servers.

Provides goal lifecycle management, concurrent execution prevention,
and standard error handling for all MTC action servers.
"""

import threading

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class BaseActionServer(Node):
    """Base class for MTC action servers.

    Subclasses must implement initialize_stages() to set self._stages.
    Optionally override _execute() for custom goal handling.
    """

    def __init__(self, node_name: str, action_name: str, action_type):
        super().__init__(node_name)

        self._executing = False
        self._lock = threading.Lock()
        self._action_type = action_type

        self._stages = None
        self.initialize_stages()

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

    def initialize_stages(self):
        """Create and assign the stages instance. Must set self._stages."""
        raise NotImplementedError("Subclass must implement initialize_stages()")

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
                error_msg = getattr(result, 'error_message', 'Unknown error')
                self.get_logger().error(f"Goal failed: {error_msg}")

            return result

        except Exception as e:
            self.get_logger().error(f"Exception during execution: {e}")
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

    Uses MultiThreadedExecutor so that callbacks (e.g. goal/result responses from
    downstream action clients used inside an action callback) can run concurrently
    with the blocking action handler. A single-threaded spin deadlocks any stage
    that polls a future while holding the executor.
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
