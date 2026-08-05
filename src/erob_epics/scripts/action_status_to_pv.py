#!/usr/bin/env python3
"""Mirror the beambot orchestrator's action GoalStatus to the erob:Status
EPICS mbbo via the epics_ros2_bridge write topic.

Read-only observer of /beambot_execution/_action/status. Publishes PVValue
(type=ENUM, int_value=<mbbo index>). Never sends goals; no control-path effect.
PAUSED is not represented (pause keeps the goal EXECUTING -> reads as Running).

mbbo index order read live from the CMS IOC 2026-08-05:
    caget -d DBR_GR_ENUM 'XF:11BM-ES:CMX{EPICS}erob:Status'
    [0] Idle  [1] Running  [2] Succeeded  [3] Failed  [4] Canceled
If the record is reordered, update STATUS_TO_INDEX (or the mbbo .db to match).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatusArray, GoalStatus

IDLE = 0
# GoalStatus -> erob:Status mbbo index
STATUS_TO_INDEX = {
    GoalStatus.STATUS_ACCEPTED: 1,   # Running
    GoalStatus.STATUS_EXECUTING: 1,  # Running
    GoalStatus.STATUS_CANCELING: 1,  # Running (winding down)
    GoalStatus.STATUS_SUCCEEDED: 2,  # Succeeded
    GoalStatus.STATUS_ABORTED: 3,    # Failed
    GoalStatus.STATUS_CANCELED: 4,   # Canceled
}


def index_for(status_list):
    """Pure mapping: newest goal's status -> mbbo index. Idle if no goals."""
    if not status_list:
        return IDLE
    latest = max(  # serial orchestrator: newest goal wins
        status_list, key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec)
    )
    return STATUS_TO_INDEX.get(latest.status, IDLE)


class ActionStatusToPV(Node):
    def __init__(self):
        super().__init__("action_status_to_pv")
        # lazy import: keeps index_for unit-testable without the bridge built
        from epics_ros2_bridge.msg import PVValue

        self._PVValue = PVValue
        self.declare_parameter("status_topic", "/beambot_execution/_action/status")
        self.declare_parameter("pv_topic", "epics/erob_status")
        status_topic = self.get_parameter("status_topic").value
        pv_topic = self.get_parameter("pv_topic").value

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(PVValue, pv_topic, 10)
        self.create_subscription(GoalStatusArray, status_topic, self._on_status, qos)
        self.get_logger().info(f"{status_topic} -> {pv_topic}")

    def _on_status(self, msg):
        idx = index_for(msg.status_list)
        self.pub.publish(
            self._PVValue(pv_name="", type=self._PVValue.ENUM, int_value=idx)
        )


def main():
    rclpy.init()
    try:
        rclpy.spin(ActionStatusToPV())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
