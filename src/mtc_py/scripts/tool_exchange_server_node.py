#!/usr/bin/env python3
"""ToolExchangeAction server node entry point."""

import rclpy
from mtc_py_lib.actions.tool_exchange_server import ToolExchangeActionServer


def main(args=None):
    """Run the ToolExchange action server."""
    rclpy.init(args=args)
    node = ToolExchangeActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ToolExchange server...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
