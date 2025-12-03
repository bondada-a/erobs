#!/usr/bin/env python3
"""MoveToAction server node entry point."""

import rclpy
from mtc_py_lib.actions.move_to_server import MoveToActionServer


def main(args=None):
    """Run the MoveTo action server."""
    rclpy.init(args=args)
    node = MoveToActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MoveTo server...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
