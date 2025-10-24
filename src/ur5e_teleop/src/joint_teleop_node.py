#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class JointTeleopNode(Node):
    """
    Direct joint velocity control for UR5e using 8BitDo Ultimate 2C controller.
    Maps joystick inputs directly to individual joint velocities.
    """

    def __init__(self):
        super().__init__('joint_teleop_node')

        # Declare parameters
        self.declare_parameter('config_file', '8bitdo_ultimate_2c.yaml')
        self.declare_parameter('velocity_topic', '/forward_velocity_controller/commands')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('gripper_action', '/gripper_action_controller/gripper_cmd')
        self.declare_parameter('max_joint_velocity', 0.5)  # rad/s

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
        self.velocity_scaling = self.config.get('velocity_scaling', {})
        self.deadzone = self.config.get('deadzone', 0.1)
        self.publish_rate = self.config.get('publish_rate', 50)

        # State variables
        self.enabled = False
        self.speed_multiplier = 1.0
        self.last_joy = None
        self.prev_button_state = {}
        self.max_joint_vel = self.get_parameter('max_joint_velocity').value

        # Publishers and subscribers
        velocity_topic = self.get_parameter('velocity_topic').value
        joy_topic = self.get_parameter('joy_topic').value

        self.velocity_pub = self.create_publisher(
            Float64MultiArray,
            velocity_topic,
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
        self.timer = self.create_timer(timer_period, self.publish_velocities)

        # Joint names for UR5e (6 joints)
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        self.get_logger().info('Joint velocity teleop node initialized')
        self.get_logger().info(f'Publishing to: {velocity_topic}')
        self.get_logger().info(f'Listening to joystick on: {joy_topic}')
        self.get_logger().info('')
        self.get_logger().info('=== CONTROLLER MAPPING ===')
        self.get_logger().info('A button: Enable/Disable motion')
        self.get_logger().info('Left stick L/R: Shoulder pan (joint 0)')
        self.get_logger().info('Left stick U/D: Shoulder lift (joint 1)')
        self.get_logger().info('Right stick U/D: Elbow (joint 2)')
        self.get_logger().info('Right stick L/R: Wrist 1 (joint 3)')
        self.get_logger().info('LT/RT triggers: Wrist 2 (joint 4)')
        self.get_logger().info('D-pad L/R: Wrist 3 (joint 5)')
        self.get_logger().info('LB: Close gripper | RB: Open gripper')
        self.get_logger().info('X: Decrease speed | Y: Increase speed')
        self.get_logger().info('Home: Emergency stop')
        self.get_logger().info('========================')

    def _get_default_config(self):
        """Return default configuration"""
        return {
            'axes': {
                'left_stick_horizontal': 0,
                'left_stick_vertical': 1,
                'right_stick_horizontal': 2,
                'right_stick_vertical': 3,
                'left_trigger': 4,
                'right_trigger': 5,
                'dpad_horizontal': 6,
            },
            'buttons': {
                'a': 0, 'b': 1, 'x': 2, 'y': 3,
                'left_bumper': 4, 'right_bumper': 5,
                'home': 8,
            },
            'deadzone': 0.1,
            'publish_rate': 50
        }

    def apply_deadzone(self, value):
        """Apply deadzone to joystick value"""
        if abs(value) < self.deadzone:
            return 0.0
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

        # Button handling
        if self.is_button_pressed(msg, 'a'):
            self.enabled = not self.enabled
            status = "ENABLED" if self.enabled else "DISABLED"
            self.get_logger().info(f'🎮 Motion control {status}')

        if self.is_button_pressed(msg, 'x'):
            self.speed_multiplier = max(0.1, self.speed_multiplier - 0.1)
            self.get_logger().info(f'⚡ Speed: {self.speed_multiplier:.1f}x')

        if self.is_button_pressed(msg, 'y'):
            self.speed_multiplier = min(2.0, self.speed_multiplier + 0.1)
            self.get_logger().info(f'⚡ Speed: {self.speed_multiplier:.1f}x')

        if self.is_button_pressed(msg, 'left_bumper'):
            self.send_gripper_command(0.0)  # Close

        if self.is_button_pressed(msg, 'right_bumper'):
            self.send_gripper_command(1.0)  # Open

        if self.is_button_pressed(msg, 'home'):
            self.enabled = False
            self.get_logger().warn('🛑 EMERGENCY STOP - Motion disabled')

    def publish_velocities(self):
        """Publish joint velocity commands based on joystick state"""
        if not self.enabled or self.last_joy is None:
            # Publish zero velocities when disabled
            msg = Float64MultiArray()
            msg.data = [0.0] * 6
            self.velocity_pub.publish(msg)
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

        # Map joysticks to joint velocities (rad/s)
        # UR5e joint order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
        joint_velocities = [
            -left_h,                        # Joint 0: Shoulder pan (left stick L/R)
            -left_v,                        # Joint 1: Shoulder lift (left stick U/D)
            -right_v,                       # Joint 2: Elbow (right stick U/D)
            -right_h,                       # Joint 3: Wrist 1 (right stick L/R)
            (right_trigger - left_trigger), # Joint 4: Wrist 2 (triggers)
            dpad_h,                         # Joint 5: Wrist 3 (D-pad L/R)
        ]

        # Scale by max velocity and speed multiplier
        scaled_velocities = [
            vel * self.max_joint_vel * self.speed_multiplier
            for vel in joint_velocities
        ]

        # Publish
        msg = Float64MultiArray()
        msg.data = scaled_velocities
        self.velocity_pub.publish(msg)

    def send_gripper_command(self, position):
        """Send gripper command (0.0 = closed, 1.0 = open)"""
        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Gripper action server not available')
            return

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 100.0

        action = "CLOSING" if position < 0.5 else "OPENING"
        self.get_logger().info(f'🤏 {action} gripper')
        self.gripper_client.send_goal_async(goal)

def main(args=None):
    rclpy.init(args=args)
    node = JointTeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
