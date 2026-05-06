#!/usr/bin/env python3
"""PickSampleAction and PlaceSampleAction server.

Dual action server hosting both pick and place operations. Both share
the same camera config but use separate stages instances.

Replaces pick_place_server.py and vision_pick_place_server.py.
"""

import yaml
from rclpy.action import ActionServer

from ament_index_python.packages import get_package_share_directory

from beambot.action_servers.base_action_server import BaseActionServer, run_server
from beambot.stages.pick_sample_stages import PickSampleStages
from beambot.stages.place_sample_stages import PlaceSampleStages
from beambot_interfaces.action import PickSampleAction, PlaceSampleAction


class SampleActionServer(BaseActionServer):
    """Action server for pick and place sample operations.

    Hosts two actions:
    - PickSampleAction: Unified pick with optional vision guidance + vacuum check
    - PlaceSampleAction: Unified place with optional vision guidance
    """

    def __init__(self):
        """Initialize the Sample action server."""
        super().__init__(
            node_name="beambot_sample_server",
            action_name="beambot_pick_sample",
            action_type=PickSampleAction,
        )

        # Second action server for place_sample
        self._place_action_server = ActionServer(
            self,
            PlaceSampleAction,
            "beambot_place_sample",
            execute_callback=self._execute_place,
        )
        self.get_logger().info("PlaceSample action server started: beambot_place_sample")

    def create_stages(self):
        """Build PickSampleStages (returned) and PlaceSampleStages (side effect).

        Both stages share the same camera config. The base class owns
        self._stages; the place-side lives on self._place_stages, set here
        because place shares the same config load.
        """
        self.declare_parameter(
            "beamline_config",
            get_package_share_directory("beambot") + "/config/default_beamline.yaml"
        )
        config_file = self.get_parameter("beamline_config").value

        camera_config = {}
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            camera_config = config.get("camera", {})
            self.get_logger().info(
                f"Camera config: type={camera_config.get('type', 'zivid')}, "
                f"frame={camera_config.get('frame', 'zivid_optical_frame')}"
            )
        except Exception as e:
            self.get_logger().warning(f"Failed to load camera config: {e}, using defaults")

        cam_kwargs = dict(
            camera_type=camera_config.get("type"),
            camera_frame=camera_config.get("frame"),
            marker_dictionary=camera_config.get("marker_dictionary"),
        )

        self._place_stages = PlaceSampleStages(self, **cam_kwargs)
        return PickSampleStages(self, **cam_kwargs)

    def _execute(self, goal_handle):
        """Execute PickSampleAction."""
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
                    pose.position.x, pose.position.y, pose.position.z,
                ]
                result.detected_orientation = [
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w,
                ]

        return result

    def _execute_place(self, goal_handle):
        """Execute PlaceSampleAction.

        This is the direct execute_callback for the second ActionServer,
        so it must call goal_handle.succeed()/abort() explicitly
        (unlike _execute which goes through base class _execute_callback).
        """
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
                    pose.position.x, pose.position.y, pose.position.z,
                ]
                result.detected_orientation = [
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w,
                ]

            goal_handle.succeed()

        return result


def main(args=None):
    run_server(SampleActionServer, args)


if __name__ == '__main__':
    main()
