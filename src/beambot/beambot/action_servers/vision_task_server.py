#!/usr/bin/env python3
"""VisionTaskAction server — unified vision-guided pipeline (issue #88).

Hosts the one pipeline (detect -> compute goal -> execute -> terminal) for all
vision-guided tasks: vision_moveto, vision-mode pick/place, and pick/place
spincoater. The orchestrator routes them all here via VisionTask goals.
"""

from std_srvs.srv import Trigger

from beambot.action_servers.base_action_server import BaseActionServer, run_server
from beambot.stages.vision_task_stages import VisionTaskStages
from beambot_interfaces.action import VisionTaskAction


class VisionTaskActionServer(BaseActionServer):
    """Action server for the unified vision pipeline."""

    def __init__(self):
        super().__init__(
            node_name="beambot_vision_task_server",
            action_name="beambot_vision_task",
            action_type=VisionTaskAction,
        )

        # Service to reset TF buffer after tool exchange (URDF change), parity
        # with VisionActionServer's beambot_vision_reset_tf.
        self._reset_tf_service = self.create_service(
            Trigger, "beambot_vision_task_reset_tf", self._reset_tf_callback
        )
        self.get_logger().info("TF reset service: beambot_vision_task_reset_tf")

    def create_stages(self):
        """Build VisionTaskStages with camera config from the beamline YAML."""
        from beambot.config_loader import load_beamline_config

        config, _ = load_beamline_config()
        camera_config = config.get("camera", {})
        self.get_logger().info(
            f"Camera config: type={camera_config.get('type')}, "
            f"frame={camera_config.get('frame')}"
        )
        return VisionTaskStages(
            self,
            camera_type=camera_config.get("type"),
            camera_frame=camera_config.get("frame"),
            marker_dictionary=camera_config.get("marker_dictionary"),
        )

    def _execute(self, goal_handle):
        """Run the pipeline and populate the result (incl. detect_only pose)."""
        goal = goal_handle.request
        error = self._stages.run(goal)

        result = VisionTaskAction.Result()
        if error is not None:
            result.success = False
            result.error_message = error
            return result

        result.success = True
        result.vacuum_ok = self._stages.vacuum_ok
        result.motion_kind = (
            "NONE" if self._stages.last_detected_pose else "CARTESIAN_POSE"
        )
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

    def _reset_tf_callback(self, request, response):
        self._stages.reset_tf()
        response.success = True
        response.message = "TF buffer cleared and listener re-created"
        return response


def main(args=None):
    run_server(VisionTaskActionServer, args)


if __name__ == "__main__":
    main()
