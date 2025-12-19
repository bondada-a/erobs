"""Async MTC Python Ophyd Device for Bluesky Integration

This is an async-capable version of MTCExecutionDevice that properly supports
Bluesky's wait=True/False parameter and task cancellation.

Key differences from mtc_ophyd_device.py:
- Returns immediately from set() instead of blocking
- Uses background thread for ROS spinning
- Proper cancellation support via cancel_goal()
- Compatible with Bluesky's pause/resume mechanisms

Backend: beambot (Python MTC implementation)
Action server: beambot_execution

Usage:
    from bluesky_ros.mtc_ophyd_device_async import MTCExecutionDeviceAsync
    robot = MTCExecutionDeviceAsync(name="ur5e_robot")

    # Non-blocking execution
    RE(bps.abs_set(robot, "task.json", wait=False))

    # Blocking execution
    RE(bps.abs_set(robot, "task.json", wait=True))

    # Cancellation
    robot.cancel_goal()
"""

import json
from threading import Thread, Lock, current_thread
from ophyd.status import DeviceStatus
from rclpy.node import Node
from rclpy.action import ActionClient
from bluesky.protocols import Movable
import rclpy


class ActionStatus(DeviceStatus):
    """Track the status of a ROS Action for Bluesky"""

    def __init__(self, device, **kwargs):
        super().__init__(device, **kwargs)

    def _handle_failure(self):
        """Called when status is marked as failed - attempt cancellation"""
        self.log.debug("Trying to stop %s", repr(self.device))
        self.device.cancel_goal()


class MTCExecutionDeviceAsync(Node, Movable):
    """Async Ophyd device for MTC task execution via Bluesky

    This device properly supports async execution, allowing Bluesky's
    wait=True/False parameter to work as expected.
    """

    def __init__(self, name="mtc_executor", robot_ip="192.168.56.101", **kwargs):
        super().__init__(name, **kwargs)

        # Set name attribute for Ophyd
        self.name = name

        # Dynamically load the action type from beambot
        from rosidl_runtime_py.utilities import get_action
        self.action_type = get_action('beambot/MTCExecution')

        self.robot_ip = robot_ip  # Kept for reference, not sent to action server
        self._action_client = ActionClient(self, self.action_type, 'beambot_execution')
        self._goal_handle = None
        self._bluesky_status = None
        self._send_goal_future = None
        self._get_result_future = None

        # Background spinning thread
        self._spin_thread = None
        self._spinning = False
        self._spin_lock = Lock()

    def construct_goal_message(self, json_path_or_string):
        """Construct goal from JSON file path or JSON string

        Args:
            json_path_or_string: Either a path to a JSON file or a JSON string

        Returns:
            MTCExecution.Goal: The constructed goal message
        """
        goal = self.action_type.Goal()

        # If it's a file path, read the file
        if isinstance(json_path_or_string, str) and json_path_or_string.endswith('.json'):
            with open(json_path_or_string, 'r') as f:
                goal.full_json = f.read()
        else:
            # Assume it's already a JSON string
            goal.full_json = json_path_or_string

        # Note: beambot gets robot_ip from beamline config, not from action goal
        return goal

    def _spin_in_background(self):
        """Spin ROS in background thread to handle action callbacks

        This allows the set() method to return immediately while still
        processing ROS callbacks for the action.
        """
        while self._spinning:
            if self._bluesky_status and self._bluesky_status.done:
                break
            try:
                rclpy.spin_once(self, timeout_sec=0.1)
            except Exception as e:
                self.get_logger().error(f"Error in spin thread: {e}")
                break

        with self._spin_lock:
            self._spinning = False
            self._spin_thread = None

    def set(self, json_path_or_string):
        """Execute MTC task (async-capable)

        This method returns immediately after sending the goal, allowing
        Bluesky to manage the waiting behavior based on the wait parameter.

        Args:
            json_path_or_string: Path to JSON task file or JSON string

        Returns:
            ActionStatus: Status object that Bluesky can monitor
        """
        # Stop any previous spin thread
        self._stop_spinning()

        # Create new status
        self._bluesky_status = ActionStatus(self)

        # Construct and send goal
        goal_msg = self.construct_goal_message(json_path_or_string)

        self.get_logger().info(f"Waiting for action server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available!")
            self._bluesky_status.set_exception(
                Exception("Action server not available after 10 seconds")
            )
            return self._bluesky_status

        self.get_logger().info("Sending goal...")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)

        # Start background spinning to handle callbacks
        with self._spin_lock:
            self._spinning = True
            self._spin_thread = Thread(target=self._spin_in_background, daemon=True)
            self._spin_thread.start()

        # Return immediately - Bluesky will manage the wait!
        return self._bluesky_status

    def _stop_spinning(self):
        """Stop the background spin thread"""
        with self._spin_lock:
            self._spinning = False
            # Only join if we're NOT being called from within the spin thread itself
            # (ROS callbacks execute in the spin thread, so _result_callback can trigger this)
            if self._spin_thread and self._spin_thread.is_alive():
                if current_thread() != self._spin_thread:
                    self._spin_thread.join(timeout=1.0)
            self._spin_thread = None

    def _feedback_callback(self, feedback_msg):
        """Handle execution feedback from the action server

        This provides real-time progress updates during task execution.
        """
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Step {fb.current_step}: {fb.current_action} "
            f"({fb.progress_percentage:.1f}%) - {fb.status_message}"
        )

    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection from action server"""
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Goal send failed: {e}")
            if self._bluesky_status:
                self._bluesky_status.set_exception(e)
            self._stop_spinning()
            return

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            if self._bluesky_status:
                self._bluesky_status.set_exception(Exception("Goal rejected"))
            self._stop_spinning()
            return

        self.get_logger().info("Goal accepted by action server")
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        """Handle final result from action server

        This is called when the action completes (success, failure, or cancel).
        """
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Error getting result: {e}")
            if self._bluesky_status:
                self._bluesky_status.set_exception(e)
            self._stop_spinning()
            return

        # Check result status
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(
                f"✓ Task completed successfully: "
                f"{result.result.completed_steps}/{result.result.total_steps} steps"
            )
            if self._bluesky_status:
                self._bluesky_status.set_finished()

        elif result.status == 5:  # ABORTED
            error_msg = result.result.error_message if hasattr(result.result, 'error_message') else "Unknown error"
            self.get_logger().error(f"✗ Task aborted: {error_msg}")
            if self._bluesky_status:
                self._bluesky_status.set_exception(Exception(f"Task aborted: {error_msg}"))

        elif result.status == 6:  # CANCELED
            self.get_logger().warning("⊗ Task canceled")
            if self._bluesky_status:
                self._bluesky_status.set_exception(Exception("Task canceled"))

        else:
            self.get_logger().error(f"✗ Unknown result status: {result.status}")
            if self._bluesky_status:
                self._bluesky_status.set_exception(Exception(f"Unknown status: {result.status}"))

        self._stop_spinning()

    def cancel_goal(self):
        """Cancel the currently executing goal

        This sends a cancel request to the action server. The cancellation
        is asynchronous - the status will be updated when the server responds.

        Returns:
            bool: True if cancel was requested, False if no active goal
        """
        if self._goal_handle is None:
            self.get_logger().warning("No active goal to cancel")
            return False

        self.get_logger().info("Requesting goal cancellation...")
        cancel_future = self._goal_handle.cancel_goal_async()

        def cancel_done(future):
            try:
                cancel_response = future.result()
                if len(cancel_response.goals_canceling) > 0:
                    self.get_logger().info("✓ Goal cancellation accepted")
                else:
                    self.get_logger().warning("⚠ Goal cancellation rejected or already complete")
            except Exception as e:
                self.get_logger().error(f"Error during cancellation: {e}")

        cancel_future.add_done_callback(cancel_done)
        return True

    def stop(self, *, success=False):
        """Stop the device (called by Bluesky on pause/abort)

        This is part of the Bluesky protocol for handling interruptions.

        Args:
            success: Whether this is a successful stop or an error condition
        """
        self.get_logger().info(f"Stop requested (success={success})")

        # Only cancel if this is NOT a successful completion
        # When success=True, the plan finished normally and we should
        # let the background task continue
        if not success:
            self.get_logger().info("Canceling goal due to error/pause...")
            self.cancel_goal()
        else:
            self.get_logger().info("Plan completed, task continues in background")

        # Don't stop spinning - let the background thread continue
        # It will stop automatically when the task completes

    def __del__(self):
        """Cleanup on deletion"""
        self._stop_spinning()
