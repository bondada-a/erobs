#!/usr/bin/env python3
"""MTCOrchestrator server node entry point."""

import rclpy
from rclpy.executors import MultiThreadedExecutor
from mtc_py_lib.actions.orchestrator import MTCOrchestratorServer


def main(args=None):
    """Run the MTC Orchestrator server."""
    rclpy.init(args=args)
    node = MTCOrchestratorServer()

    # Use multithreaded executor for concurrent action client calls
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MTC Orchestrator...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
