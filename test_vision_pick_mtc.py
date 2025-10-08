#!/usr/bin/env python3
"""
Proper MTC-based vision pick using detected AprilTag.
This uses MoveIt Task Constructor to create a proper pick pipeline.
"""

import rclpy
from rclpy.node import Node
from moveit.task_constructor import core, stages
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import time

class VisionPickMTC(Node):
    def __init__(self):
        super().__init__('vision_pick_mtc')

        # TF2 for getting tag poses
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Service client for camera capture
        self.capture_client = self.create_client(Trigger, '/capture_2d')

        self.get_logger().info("Vision Pick MTC initialized")

    def create_pick_task(self, target_pose):
        """
        Create a proper MTC pick task with stages.
        This is the RIGHT way to do vision-based picking.
        """

        # Create task
        task = core.Task()
        task.name = "vision_pick_task"
        task.loadRobotModel(self)

        # Get robot model
        arm = task.getRobotModel().getJointModelGroup("ur_arm")
        gripper = task.getRobotModel().getJointModelGroup("robotiq_hande")

        # Set task properties
        task.setProperty("group", arm.getName())
        task.setProperty("eef", "robotiq_hande")
        task.setProperty("ik_frame", "robotiq_hande_grasp_point")  # Use grasp point as TCP!

        # Stage 1: Current State
        stage_current = stages.CurrentState("current")
        task.add(stage_current)

        # Stage 2: Open Gripper
        stage_open = stages.MoveTo("open gripper", gripper)
        stage_open.setGoal("open")
        task.add(stage_open)

        # Stage 3: Generate Grasp Poses
        grasp_generator = stages.GenerateGraspPose("generate grasp poses")
        grasp_generator.setAngleDelta(0.2)  # Allow some rotation variance
        grasp_generator.setPreGraspPose("open")
        grasp_generator.setGraspPose("closed")
        grasp_generator.setMonitoredStage(stage_current)

        # Define approach direction (from above for table-top grasping)
        approach = TwistStamped()
        approach.header.frame_id = "world"
        approach.twist.linear.z = -1.0  # Approach from above
        grasp_generator.setApproachMotion(approach, 0.05, 0.15)  # Min 5cm, Max 15cm

        # Add the target object pose
        grasp_generator.setObject(target_pose)

        # Wrap in compute IK
        ik_wrapper = stages.ComputeIK("compute grasp IK", grasp_generator)
        ik_wrapper.setMaxIKSolutions(8)
        ik_wrapper.setIKFrame("robotiq_hande_grasp_point")
        ik_wrapper.setGroup(arm.getName())
        task.add(ik_wrapper)

        # Stage 4: Allow Collision (object <-> gripper)
        allow_collision = stages.ModifyPlanningScene("allow collision")
        allow_collision.allowCollisions("tag",
                                        task.getRobotModel().getJointModelGroup("robotiq_hande").getLinkModelNames(),
                                        True)
        task.add(allow_collision)

        # Stage 5: Close Gripper (actual grasp)
        stage_close = stages.MoveTo("close gripper", gripper)
        stage_close.setGoal("closed")
        task.add(stage_close)

        # Stage 6: Attach object
        attach = stages.ModifyPlanningScene("attach object")
        attach.attachObject("tag", "robotiq_hande_grasp_point")
        task.add(attach)

        # Stage 7: Lift
        lift = stages.MoveRelative("lift", arm)
        lift.setDirection(Vector3Stamped(header=approach.header, vector=Vector3(z=0.1)))
        lift.setMinDistance(0.05)
        lift.setMaxDistance(0.15)
        task.add(lift)

        return task

    def detect_and_pick(self, tag_id=1):
        """
        Complete vision pick pipeline:
        1. Capture image
        2. Detect tag
        3. Create MTC pick task
        4. Execute
        """

        # Step 1: Trigger camera capture
        self.get_logger().info("Triggering camera capture...")
        if not self.capture_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Capture service not available")
            return False

        future = self.capture_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        if not future.result().success:
            self.get_logger().error("Camera capture failed")
            return False

        # Step 2: Get tag pose from TF
        self.get_logger().info(f"Looking for tag {tag_id}...")
        tag_frame = f'tag36h11:{tag_id}'

        try:
            # Get transform from base to tag
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                tag_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )

            # Convert to PoseStamped
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'base_link'
            target_pose.pose.position = transform.transform.translation
            target_pose.pose.orientation = transform.transform.rotation

            self.get_logger().info(f"Tag detected at: [{target_pose.pose.position.x:.3f}, "
                                  f"{target_pose.pose.position.y:.3f}, "
                                  f"{target_pose.pose.position.z:.3f}]")

        except Exception as e:
            self.get_logger().error(f"Failed to get tag pose: {e}")
            return False

        # Step 3: Create pick task
        self.get_logger().info("Creating pick task...")
        task = self.create_pick_task(target_pose)

        # Step 4: Plan
        self.get_logger().info("Planning pick sequence...")
        if not task.plan():
            self.get_logger().error("Failed to plan pick task")
            return False

        # Step 5: Execute
        self.get_logger().info("Executing pick...")
        task.execute()

        return True

def main():
    rclpy.init()

    node = VisionPickMTC()

    print("\n" + "="*60)
    print("  Vision Pick with MTC (Proper Design)")
    print("="*60)
    print("\nThis demonstrates the RIGHT way to do vision picking:")
    print("1. Detect object with vision")
    print("2. Use MTC to generate proper pick sequence")
    print("3. Execute with collision checking, approach, grasp, lift")
    print("="*60)

    input("\nPlace tag in view and press ENTER to pick...")

    success = node.detect_and_pick(tag_id=1)

    if success:
        print("✓ Pick successful!")
    else:
        print("✗ Pick failed!")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()