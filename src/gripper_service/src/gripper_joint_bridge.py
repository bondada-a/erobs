#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from cms_beamtime_interfaces.srv import GripperControlMsg

class GripperJointBridge(Node):
    def __init__(self):
        super().__init__('gripper_joint_bridge')
        self.srv_client = self.create_client(GripperControlMsg, 'gripper_service')
        self.sub = self.create_subscription(JointTrajectory, '/gripper_controller/command', self.cb, 10)
        self.get_logger().info('GripperJointBridge node started.')

    def cb(self, msg):
        # Expecting only joint_finger
        if len(msg.joint_names) != 1 or msg.joint_names[0] != 'joint_finger':
            self.get_logger().warn('Received JointTrajectory without joint_finger. Ignoring.')
            return
        if not msg.points:
            self.get_logger().warn('Received JointTrajectory with no points. Ignoring.')
            return
        pos = msg.points[-1].positions[0]  # Use last point as target

        # Scale 0.0(open)-0.025(closed) to 0-100%
        grip_pct = int(max(0.0, min(pos, 0.025)) / 0.025 * 100)
        if grip_pct <= 2:
            cmd = "OPEN"
        elif grip_pct >= 98:
            cmd = "CLOSE"
        else:
            cmd = "PARTIAL"

        req = GripperControlMsg.Request()
        req.command = cmd
        req.grip = grip_pct

        if not self.srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("gripper_service not available")
            return

        fut = self.srv_client.call_async(req)
        fut.add_done_callback(lambda fut: self.handle_response(fut, cmd, grip_pct))

    def handle_response(self, fut, cmd, grip_pct):
        try:
            resp = fut.result()
            if resp.results:
                self.get_logger().info(f'Successfully sent gripper command: {cmd} ({grip_pct}%)')
            else:
                self.get_logger().error(f'Failed to send gripper command: {cmd} ({grip_pct}%)')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GripperJointBridge()
    rclpy.spin(node)
    rclpy.shutdown()