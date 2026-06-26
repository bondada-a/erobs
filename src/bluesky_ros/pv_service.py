#!/usr/bin/env python3
"""ROS service returning a PV's caget value. Run: python3 pv_service.py <PV_NAME>"""
import sys

import epics
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class PVService(Node):
    def __init__(self, pvname):
        super().__init__("pv_service")
        self.pvname = pvname
        self.create_service(Trigger, "get_pv", self.on_request)
        self.get_logger().info(f"Serving caget({pvname}) on /get_pv")

    def on_request(self, request, response):
        value = epics.caget(self.pvname)
        response.success = value is not None
        response.message = str(value)
        return response


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: pv_service.py <PV_NAME>")
    rclpy.init()
    rclpy.spin(PVService(sys.argv[1]))


if __name__ == "__main__":
    main()
