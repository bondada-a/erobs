"""Base action server template - Python equivalent of base_action_server.hpp.

Provides a reusable base class for MTC action servers with:
- Concurrent execution prevention
- Goal lifecycle management
- Standard error handling pattern

Uses rclpy for the action server (standard ROS 2 Python pattern).
MTC operations use rclcpp.Node internally via MTCNode singleton.
"""

import threading
from typing import TypeVar, Generic, Type

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

ActionT = TypeVar('ActionT')


class BaseActionServer(Node, Generic[ActionT]):
    """Template base class for MTC action servers.

    Handles:
    - Goal lifecycle management (accept, execute, complete/abort)
    - Concurrent execution prevention (one goal at a time)
    - Standard error handling and logging

    Subclasses must implement:
    - initialize_stages(): Create the stages instance
    - _execute(): Handle the actual goal execution

    Example:
        class MoveToActionServer(BaseActionServer):
            def __init__(self):
                super().__init__(
                    node_name="mtc_moveto_server_py",
                    action_name="mtc_moveto_py",
                    action_type=MoveToAction,
                )
                self.initialize_stages()

            def initialize_stages(self):
                self._stages = MoveToStages(self)

            def _get_failure_message(self) -> str:
                return "Motion planning or execution failed"

        # In main:
        if __name__ == '__main__':
            run_server(MoveToActionServer)
    """

    def __init__(
        self,
        node_name: str,
        action_name: str,
        action_type: Type[ActionT],
    ):
        """Initialize action server.

        Args:
            node_name: ROS node name
            action_name: Action server name (e.g., "mtc_moveto_py")
            action_type: Action type class
        """
        super().__init__(node_name)

        self._executing = False
        self._lock = threading.Lock()
        self._action_type = action_type

        # Stages instance (created by subclass via initialize_stages)
        self._stages = None

        # Create action server with reentrant callback group
        self._callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            action_type,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(f"{node_name} started on '{action_name}'")

    def initialize_stages(self):
        """Initialize stages - must be called after construction.

        Override in subclass to create specific stages instance.

        Raises:
            NotImplementedError: Always - subclass must override
        """
        raise NotImplementedError("Subclass must implement initialize_stages()")

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Handle incoming goal requests.

        Args:
            goal_request: The incoming goal request

        Returns:
            GoalResponse.ACCEPT to accept the goal
        """
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Handle cancel requests.

        MTC operations cannot be safely cancelled mid-motion, so we reject
        all cancel requests.

        Args:
            goal_handle: The goal handle to potentially cancel

        Returns:
            CancelResponse.REJECT always
        """
        self.get_logger().warn(
            "Cancel request rejected - cannot safely abort mid-motion"
        )
        return CancelResponse.REJECT

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute the goal with concurrency protection.

        Only one goal can execute at a time. If a goal is already executing,
        the new goal is aborted with "Server busy" error.

        Args:
            goal_handle: The goal handle to execute

        Returns:
            The action result
        """
        # Check for concurrent execution
        with self._lock:
            if self._executing:
                self.get_logger().warn("Rejecting goal: server busy")
                result = self._create_error_result("Server busy")
                goal_handle.abort()
                return result
            self._executing = True

        # Check stages initialized
        if self._stages is None:
            self.get_logger().error("Stages not initialized")
            result = self._create_error_result("Stages not initialized")
            goal_handle.abort()
            with self._lock:
                self._executing = False
            return result

        try:
            # Execute the goal
            result = self._execute(goal_handle)

            # Handle result
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
            result = self._create_error_result(str(e))
            goal_handle.abort()
            return result

        finally:
            with self._lock:
                self._executing = False

    def _execute(self, goal_handle: ServerGoalHandle):
        """Execute goal with default implementation.

        Default behavior: calls self._stages.run(goal) and sets error message
        on failure using _get_failure_message().

        Override this method if you need custom behavior like logging goal
        details before execution.

        Args:
            goal_handle: The goal handle with request data

        Returns:
            Action result with success and error_message fields
        """
        result = self._action_type.Result()
        result.success = self._stages.run(goal_handle.request)
        if not result.success:
            result.error_message = self._get_failure_message()
        return result

    def _get_failure_message(self) -> str:
        """Get the error message for when stages.run() returns False.

        Override in subclass to customize the error message.

        Returns:
            Error message string
        """
        return "Operation failed"

    def _create_error_result(self, error_message: str):
        """Create an error result with the given message.

        Args:
            error_message: The error message to include

        Returns:
            Action result with success=False and error_message set
        """
        result = self._action_type.Result()
        result.success = False
        result.error_message = error_message
        return result

    @property
    def is_executing(self) -> bool:
        """Check if the server is currently executing a goal.

        Returns:
            True if executing, False otherwise
        """
        with self._lock:
            return self._executing


def run_server(server_class: Type[BaseActionServer], args=None):
    """Run an action server with standard lifecycle management.

    Handles rclpy init/shutdown and KeyboardInterrupt gracefully.

    Args:
        server_class: The action server class to instantiate and run
        args: Command line arguments (default: None)

    Example:
        if __name__ == '__main__':
            run_server(MoveToActionServer)
    """
    rclpy.init(args=args)
    node = server_class()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
