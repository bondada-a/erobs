#!/usr/bin/env python3
"""PickPlaceAction server node entry point."""

import rclpy
from mtc_py_lib.actions.pick_place_server import PickPlaceActionServer


def main(args=None):
    """Run the PickPlace action server."""
    rclpy.init(args=args)
    node = PickPlaceActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PickPlace server...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
