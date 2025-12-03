#!/usr/bin/env python3
"""EndEffectorAction server node entry point."""

import rclpy
from mtc_py_lib.actions.end_effector_server import EndEffectorActionServer


def main(args=None):
    """Run the EndEffector action server."""
    rclpy.init(args=args)
    node = EndEffectorActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down EndEffector server...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
