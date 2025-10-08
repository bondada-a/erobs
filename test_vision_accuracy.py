#!/usr/bin/env python3
"""
Test vision system accuracy by moving robot TCP to detected tag position.
This helps verify if the vision calibration is correct.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import time
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
import numpy as np

class VisionAccuracyTest(Node):
    def __init__(self):
        super().__init__('vision_accuracy_test')

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MoveIt
        self.moveit = MoveItPy(node_name="vision_accuracy_test")
        self.arm = self.moveit.get_planning_component("ur_arm")
        self.gripper = self.moveit.get_planning_component("robotiq_hande")

        self.get_logger().info("Vision accuracy test ready")

    def get_tag_pose(self, tag_id=1, reference_frame='base_link'):
        """Get tag position in reference frame"""
        tag_frame = f'tag36h11:{tag_id}'

        try:
            # Wait for transform
            transform = self.tf_buffer.lookup_transform(
                reference_frame,
                tag_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )

            pose = PoseStamped()
            pose.header.frame_id = reference_frame
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation

            return pose
        except Exception as e:
            self.get_logger().error(f"Failed to get tag transform: {e}")
            return None

    def test_move_to_tag(self, tag_id=1):
        """Test moving TCP to detected tag position"""

        # Step 1: Open gripper
        self.get_logger().info("Opening gripper...")
        gripper_state = [0.0]  # 0 = open
        self.gripper.set_goal_state(configuration_name="open")
        plan = self.gripper.plan()
        if plan:
            self.gripper.execute(plan.trajectory)
            time.sleep(2)

        # Step 2: Get tag pose
        self.get_logger().info(f"Getting tag {tag_id} position...")
        tag_pose = self.get_tag_pose(tag_id, 'base_link')

        if tag_pose is None:
            self.get_logger().error("Could not detect tag!")
            return False

        self.get_logger().info(f"Tag detected at: [{tag_pose.pose.position.x:.3f}, "
                               f"{tag_pose.pose.position.y:.3f}, "
                               f"{tag_pose.pose.position.z:.3f}]")

        # Step 3: Calculate approach pose (above tag)
        approach_pose = PoseStamped()
        approach_pose.header.frame_id = 'base_link'
        approach_pose.pose.position.x = tag_pose.pose.position.x
        approach_pose.pose.position.y = tag_pose.pose.position.y
        approach_pose.pose.position.z = tag_pose.pose.position.z + 0.10  # 10cm above tag

        # Keep gripper pointing down (assuming tag is on table)
        approach_pose.pose.orientation.x = 0.707  # Pointing down
        approach_pose.pose.orientation.y = 0.0
        approach_pose.pose.orientation.z = 0.0
        approach_pose.pose.orientation.w = 0.707

        # Step 4: Move to approach pose
        self.get_logger().info("Moving to approach position (10cm above tag)...")
        self.arm.set_goal_state(pose_stamped_msg=approach_pose,
                                pose_link="robotiq_hande_end")

        plan = self.arm.plan()
        if plan:
            self.get_logger().info("Executing approach move...")
            success = self.arm.execute(plan.trajectory)
            time.sleep(2)

            if success:
                self.get_logger().info("SUCCESS: Robot at approach position")
                self.get_logger().info("Check if tag is directly below the gripper")

                # Step 5: Move down to grasp height (optional - be careful!)
                response = input("Move down to tag? (y/n): ")
                if response.lower() == 'y':
                    grasp_pose = PoseStamped()
                    grasp_pose.header.frame_id = 'base_link'
                    grasp_pose.pose.position.x = tag_pose.pose.position.x
                    grasp_pose.pose.position.y = tag_pose.pose.position.y
                    grasp_pose.pose.position.z = tag_pose.pose.position.z + 0.02  # 2cm above tag
                    grasp_pose.pose.orientation = approach_pose.pose.orientation

                    self.arm.set_goal_state(pose_stamped_msg=grasp_pose,
                                           pose_link="robotiq_hande_end")
                    plan = self.arm.plan()
                    if plan:
                        self.arm.execute(plan.trajectory)
                        self.get_logger().info("At grasp position - check if tag is between fingers")
            else:
                self.get_logger().error("Failed to execute approach move")
                return False
        else:
            self.get_logger().error("Failed to plan approach move")
            return False

        return True

def main():
    rclpy.init()

    test = VisionAccuracyTest()

    # Wait for services
    time.sleep(2)

    print("\n" + "="*60)
    print("  Vision System Accuracy Test")
    print("="*60)
    print("\nThis will move the robot to the detected tag position")
    print("to verify vision system accuracy.")
    print("\n1. Place tag in camera view")
    print("2. Trigger capture: ros2 service call /capture_2d std_srvs/srv/Trigger")
    print("3. Press ENTER to move robot to tag")
    print("="*60)

    input("\nPress ENTER when tag is detected...")

    # Test move to tag
    test.test_move_to_tag(tag_id=1)

    input("\nPress ENTER to finish...")

    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()