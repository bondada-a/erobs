#!/usr/bin/env python3
"""Serve pick and place sample actions with shared camera configuration."""

from rclpy.action import ActionServer

from beambot.action_servers.base_action_server import BaseActionServer, run_server
from beambot.config_loader import load_beamline_config
from beambot.motion.pick_sample_task import PickSampleTask
from beambot.motion.place_sample_task import PlaceSampleTask
from beambot_interfaces.action import PickSampleAction, PlaceSampleAction


class SampleActionServer(BaseActionServer):
    """Host PickSampleAction and PlaceSampleAction."""

    def __init__(self):
        super().__init__(
            node_name="beambot_sample_server",
            action_name="beambot_pick_sample",
            action_type=PickSampleAction,
        )

        # Base class owns pick; register place separately.
        self._place_action_server = ActionServer(
            self,
            PlaceSampleAction,
            "beambot_place_sample",
            execute_callback=self._execute_place,
            goal_callback=self._goal_callback,
        )
        self.get_logger().info(
            "PlaceSample action server started: beambot_place_sample"
        )

    def create_task(self):
        """Create pick and place tasks with shared camera settings."""
        config, _ = load_beamline_config()
        camera_config = config.get("camera", {})
        self.get_logger().info(
            f"Camera config: type={camera_config.get('type')}, "
            f"frame={camera_config.get('frame')}"
        )

        cam_kwargs = dict(
            camera_type=camera_config.get("type"),
            camera_frame=camera_config.get("frame"),
            marker_dictionary=camera_config.get("marker_dictionary"),
        )

        self._place_task = PlaceSampleTask(self, **cam_kwargs)
        return PickSampleTask(self, **cam_kwargs)

    def _execute(self, goal_handle):
        """Run the pick task."""
        goal = goal_handle.request
        error = self._task.run(goal)

        result = PickSampleAction.Result()
        if error is not None:
            result.success = False
            result.error_message = error
        else:
            result.success = True
            result.vacuum_ok = self._task.vacuum_ok

            if self._task.last_detected_pose is not None:
                pose = self._task.last_detected_pose.pose
                result.detected_position = [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ]
                result.detected_orientation = [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]

        return result

    def _execute_place(self, goal_handle):
        """Run the place task and finalize its directly registered goal."""
        try:
            goal = goal_handle.request
            error = self._place_task.run(goal)

            result = PlaceSampleAction.Result()
            if error is not None:
                result.success = False
                result.error_message = error
                goal_handle.abort()
            else:
                result.success = True

                if self._place_task.last_detected_pose is not None:
                    pose = self._place_task.last_detected_pose.pose
                    result.detected_position = [
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                    ]
                    result.detected_orientation = [
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ]

                goal_handle.succeed()

            return result
        finally:
            self._executing = False


def main(args=None):
    run_server(SampleActionServer, args)


if __name__ == "__main__":
    main()
