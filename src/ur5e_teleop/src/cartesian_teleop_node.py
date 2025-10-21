#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class CartesianTeleopNode(Node):
    """
    Teleoperation node for UR5e using 8BitDo Ultimate 2C controller.
    Converts joystick inputs to Cartesian velocity commands.
    """

    def __init__(self):
        super().__init__('cartesian_teleop_node')

        # Declare parameters
        self.declare_parameter('config_file', '8bitdo_ultimate_2c.yaml')
        self.declare_parameter('twist_topic', '/servo_node/delta_twist_cmds')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('gripper_action', '/gripper_action_controller/gripper_cmd')
        self.declare_parameter('base_frame', 'base_link')

        # Load configuration
        config_file = self.get_parameter('config_file').value
        config_path = os.path.join(
            get_package_share_directory('ur5e_teleop'),
            'config',
            config_file
        )

        try:
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            self.get_logger().info(f'Loaded configuration from {config_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')
            self.config = self._get_default_config()

        # Extract configuration
        self.axes = self.config['axes']
        self.buttons = self.config['buttons']
        self.velocity_scaling = self.config['velocity_scaling']
        self.deadzone = self.config['deadzone']
        self.publish_rate = self.config['publish_rate']

        # State variables
        self.enabled = False
        self.speed_multiplier = 1.0
        self.last_joy = None
        self.prev_button_state = {}

        # Publishers and subscribers
        twist_topic = self.get_parameter('twist_topic').value
        joy_topic = self.get_parameter('joy_topic').value

        self.twist_pub = self.create_publisher(
            TwistStamped,
            twist_topic,
            10
        )

        self.joy_sub = self.create_subscription(
            Joy,
            joy_topic,
            self.joy_callback,
            10
        )

        # Gripper action client
        gripper_action = self.get_parameter('gripper_action').value
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            gripper_action
        )

        # Timer for publishing velocity commands
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_twist)

        self.get_logger().info('Cartesian teleop node initialized')
        self.get_logger().info(f'Publishing twist commands to: {twist_topic}')
        self.get_logger().info(f'Listening to joystick on: {joy_topic}')
        self.get_logger().info('Press A button to enable motion')

    def _get_default_config(self):
        """Return default configuration if file not found"""
        return {
            'axes': {
                'left_stick_horizontal': 0,
                'left_stick_vertical': 1,
                'right_stick_horizontal': 2,
                'right_stick_vertical': 3,
                'left_trigger': 4,
                'right_trigger': 5,
                'dpad_horizontal': 6,
                'dpad_vertical': 7
            },
            'buttons': {
                'a': 0, 'b': 1, 'x': 2, 'y': 3,
                'left_bumper': 4, 'right_bumper': 5,
                'back': 6, 'start': 7, 'home': 8,
                'left_stick_press': 9, 'right_stick_press': 10
            },
            'velocity_scaling': {
                'linear_base': 0.05,
                'angular_base': 0.2,
                'min_multiplier': 0.1,
                'max_multiplier': 2.0,
                'multiplier_step': 0.1
            },
            'deadzone': 0.1,
            'publish_rate': 50
        }

    def apply_deadzone(self, value):
        """Apply deadzone to joystick value"""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale to full range after deadzone
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def get_axis_value(self, joy_msg, axis_name):
        """Safely get axis value with bounds checking"""
        axis_idx = self.axes.get(axis_name, -1)
        if axis_idx >= 0 and axis_idx < len(joy_msg.axes):
            return self.apply_deadzone(joy_msg.axes[axis_idx])
        return 0.0

    def get_button_value(self, joy_msg, button_name):
        """Safely get button value with bounds checking"""
        button_idx = self.buttons.get(button_name, -1)
        if button_idx >= 0 and button_idx < len(joy_msg.buttons):
            return joy_msg.buttons[button_idx]
        return 0

    def is_button_pressed(self, joy_msg, button_name):
        """Detect button press (rising edge)"""
        current = self.get_button_value(joy_msg, button_name)
        previous = self.prev_button_state.get(button_name, 0)
        self.prev_button_state[button_name] = current
        return current == 1 and previous == 0

    def joy_callback(self, msg):
        """Handle incoming joystick messages"""
        self.last_joy = msg

        # Check for button presses (rising edge detection)
        if self.is_button_pressed(msg, 'a'):
            self.enabled = not self.enabled
            status = "ENABLED" if self.enabled else "DISABLED"
            self.get_logger().info(f'Motion control {status}')

        if self.is_button_pressed(msg, 'x'):
            self.speed_multiplier = max(
                self.velocity_scaling['min_multiplier'],
                self.speed_multiplier - self.velocity_scaling['multiplier_step']
            )
            self.get_logger().info(f'Speed multiplier: {self.speed_multiplier:.2f}x')

        if self.is_button_pressed(msg, 'y'):
            self.speed_multiplier = min(
                self.velocity_scaling['max_multiplier'],
                self.speed_multiplier + self.velocity_scaling['multiplier_step']
            )
            self.get_logger().info(f'Speed multiplier: {self.speed_multiplier:.2f}x')

        if self.is_button_pressed(msg, 'left_bumper'):
            self.send_gripper_command(0.0)  # Close

        if self.is_button_pressed(msg, 'right_bumper'):
            self.send_gripper_command(1.0)  # Open

        if self.is_button_pressed(msg, 'home'):
            self.enabled = False
            self.get_logger().warn('EMERGENCY STOP - Motion disabled')

        if self.is_button_pressed(msg, 'b'):
            self.get_logger().info('Return to home requested (not implemented)')
            # TODO: Implement home position return

    def publish_twist(self):
        """Publish Cartesian velocity commands based on joystick state"""
        if not self.enabled or self.last_joy is None:
            # Publish zero twist when disabled
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = self.get_parameter('base_frame').value
            self.twist_pub.publish(twist_msg)
            return

        # Get joystick values
        left_h = self.get_axis_value(self.last_joy, 'left_stick_horizontal')
        left_v = self.get_axis_value(self.last_joy, 'left_stick_vertical')
        right_h = self.get_axis_value(self.last_joy, 'right_stick_horizontal')
        right_v = self.get_axis_value(self.last_joy, 'right_stick_vertical')

        # Triggers (convert from -1..1 to 0..1)
        left_trigger = (self.get_axis_value(self.last_joy, 'left_trigger') + 1.0) / 2.0
        right_trigger = (self.get_axis_value(self.last_joy, 'right_trigger') + 1.0) / 2.0

        dpad_h = self.get_axis_value(self.last_joy, 'dpad_horizontal')

        # Compute velocities
        linear_scale = self.velocity_scaling['linear_base'] * self.speed_multiplier
        angular_scale = self.velocity_scaling['angular_base'] * self.speed_multiplier

        # Create twist message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.get_parameter('base_frame').value

        # Linear velocities (in base_link frame)
        twist_msg.twist.linear.x = -left_v * linear_scale      # Forward/backward
        twist_msg.twist.linear.y = -left_h * linear_scale      # Left/right
        twist_msg.twist.linear.z = (right_trigger - left_trigger) * linear_scale  # Up/down

        # Angular velocities (in base_link frame)
        twist_msg.twist.angular.x = dpad_h * angular_scale     # Roll
        twist_msg.twist.angular.y = -right_v * angular_scale   # Pitch
        twist_msg.twist.angular.z = -right_h * angular_scale   # Yaw

        self.twist_pub.publish(twist_msg)

    def send_gripper_command(self, position):
        """Send gripper command (0.0 = closed, 1.0 = open)"""
        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Gripper action server not available')
            return

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 100.0

        self.get_logger().info(f'Sending gripper command: {position}')
        self.gripper_client.send_goal_async(goal)

def main(args=None):
    rclpy.init(args=args)
    node = CartesianTeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
