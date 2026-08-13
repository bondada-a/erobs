#!/usr/bin/env python3
"""Serve pick and place sample actions with shared camera configuration."""

from rclpy.action import ActionServer

from beambot.action_servers.base_action_server import BaseActionServer, run_server
from beambot.stages.pick_sample_stages import PickSampleStages
from beambot.stages.place_sample_stages import PlaceSampleStages
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

    def create_stages(self):
        """Create pick and place stages with shared camera settings."""
        from beambot.config_loader import load_beamline_config

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

        self._place_stages = PlaceSampleStages(self, **cam_kwargs)
        return PickSampleStages(self, **cam_kwargs)

    def _execute(self, goal_handle):
        """Run pick stages."""
        goal = goal_handle.request
        error = self._stages.run(goal)

        result = PickSampleAction.Result()
        if error is not None:
            result.success = False
            result.error_message = error
        else:
            result.success = True
            result.vacuum_ok = self._stages.vacuum_ok

            if self._stages.last_detected_pose is not None:
                pose = self._stages.last_detected_pose.pose
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
        """Run place stages and finalize its directly registered goal."""
        try:
            goal = goal_handle.request
            error = self._place_stages.run(goal)

            result = PlaceSampleAction.Result()
            if error is not None:
                result.success = False
                result.error_message = error
                goal_handle.abort()
            else:
                result.success = True

                if self._place_stages.last_detected_pose is not None:
                    pose = self._place_stages.last_detected_pose.pose
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
            with self._lock:
                self._executing = False


def main(args=None):
    run_server(SampleActionServer, args)


if __name__ == "__main__":
    main()
