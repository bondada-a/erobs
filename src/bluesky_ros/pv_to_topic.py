#!/usr/bin/env python3
"""Bridge one EPICS PV onto a ROS topic. Run: python3 pv_to_topic.py <PV_NAME>"""
import sys

import epics
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class PVBridge(Node):
    def __init__(self, pvname):
        super().__init__("pv_bridge")
        self.pub = self.create_publisher(Float64, "pv_value", 10)
        epics.camonitor(pvname, callback=self.on_change)
        self.get_logger().info(f"Bridging {pvname} -> /pv_value")

    def on_change(self, value=None, **kw):
        self.pub.publish(Float64(data=float(value)))


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: pv_to_topic.py <PV_NAME>")
    rclpy.init()
    rclpy.spin(PVBridge(sys.argv[1]))


if __name__ == "__main__":
    main()
