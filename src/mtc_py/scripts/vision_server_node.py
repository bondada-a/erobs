#!/usr/bin/env python3
"""VisionMoveToAction server node entry point."""

import rclpy
from rclpy.node import Node


def main(args=None):
    """Run the VisionMoveTo action server."""
    rclpy.init(args=args)
    node = Node("vision_server_py_placeholder")
    node.get_logger().error("Vision server not yet implemented")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
