#!/usr/bin/env python3
"""
Simple tool to test and identify button/axis mappings on the 8BitDo controller.
Press buttons and move joysticks to see their indices.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class ControllerTester(Node):
    def __init__(self):
        super().__init__('controller_tester')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.last_joy = None
        self.get_logger().info('Controller Tester Started!')
        self.get_logger().info('Press buttons and move joysticks to see their indices')
        self.get_logger().info('-' * 60)

    def joy_callback(self, msg):
        if self.last_joy is None:
            self.last_joy = msg
            return

        # Check for button presses
        for i, (current, previous) in enumerate(zip(msg.buttons, self.last_joy.buttons)):
            if current == 1 and previous == 0:
                self.get_logger().info(f'🔵 BUTTON {i} pressed')
            elif current == 0 and previous == 1:
                self.get_logger().info(f'⚪ BUTTON {i} released')

        # Check for significant axis changes
        for i, (current, previous) in enumerate(zip(msg.axes, self.last_joy.axes)):
            if abs(current - previous) > 0.3:  # Significant change
                self.get_logger().info(f'🎮 AXIS {i}: {current:+.3f}')

        self.last_joy = msg

def main(args=None):
    rclpy.init(args=args)
    tester = ControllerTester()

    print("\n" + "="*60)
    print("8BitDo Controller Tester")
    print("="*60)
    print("Press buttons and move joysticks to identify indices")
    print("Press Ctrl+C to exit")
    print("="*60 + "\n")

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
