#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.action import GripperCommand
from rclpy.action import ActionServer
from cms_beamtime_interfaces.srv import GripperControlMsg

class GripperActionBridge(Node):
    def __init__(self):
        super().__init__('gripper_action_bridge')
        self._action_server = ActionServer(
            self,
            GripperCommand,
            'gripper_cmd',
            execute_callback=self.execute_callback)
        self._cli = self.create_client(GripperControlMsg, 'gripper_service')
        while not self._cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for gripper_service...')

    async def execute_callback(self, goal_handle):
        command = goal_handle.request.command.position
        request = GripperControlMsg.Request()
        if command < 0.0125:
            request.command = "OPEN"
            request.grip = 100
        else:
            request.command = "CLOSE"
            request.grip = 100

        future = self._cli.call_async(request)
        await future
        result = GripperCommand.Result()
        if future.result().results == 1:
            result.reached_goal = True
            goal_handle.succeed()
        else:
            result.reached_goal = False
            goal_handle.abort()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = GripperActionBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()