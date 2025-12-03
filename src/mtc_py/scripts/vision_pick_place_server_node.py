#!/usr/bin/env python3
"""VisionPickPlaceAction server node entry point."""

import rclpy
from rclpy.node import Node


def main(args=None):
    """Run the VisionPickPlace action server."""
    rclpy.init(args=args)
    node = Node("vision_pick_place_server_py_placeholder")
    node.get_logger().error("VisionPickPlace server not yet implemented")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
