from ophyd import Device, Signal
from ophyd import Component as Cpt

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
import time

class ROS_Node(Node):

    def __init__(self, joint_state_callback):
        super().__init__("arm_node")

        self._joint_state_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            joint_state_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Subscribed to /joint_states")


    def list_topics(self):
        return self.get_topic_names_and_types()
    
class Joint(Device):

    # read from /joint_states topic
    readback = Cpt(Signal, value=None)
    velocity = Cpt(Signal, value=None)
    effort = Cpt(Signal, value=None)

class Robotic_Arm(Device):

    # Joint components
    base = Cpt(Joint, "base")
    shoulder = Cpt(Joint, "shoulder")
    elbow = Cpt(Joint, "elbow")
    w1 = Cpt(Joint, "w1")
    w2 = Cpt(Joint, "w2")
    w3 = Cpt(Joint, "w3")

    def __init__(self, prefix="", *, name, **kwargs):
        super().__init__(prefix=prefix, name=name, **kwargs)

        if not rclpy.ok():
            rclpy.init()

        # Change the keys if /joint_states uses different joint names.
        self._joint_name_map = {
            "base": self.base,
            "shoulder": self.shoulder,
            "elbow": self.elbow,
            "w1": self.w1,
            "w2": self.w2,
            "w3": self.w3,
        }

        # ROS node calls self._joint_state_callback when data arrives.
        self._ros_node = ROS_Node(self._joint_state_callback)

    def _joint_state_callback(self, message):
        """
        Transfer values from a ROS JointState message into Ophyd Signals.

        The arrays in JointState are parallel:
            message.name[i]
            message.position[i]
            message.velocity[i]
            message.effort[i]
        all describe the same joint.
        """

        updated_joints = []

        for index, ros_joint_name in enumerate(message.name):
            joint = self._joint_name_map.get(ros_joint_name)

            if joint is None:
                self._ros_node.get_logger().warning(
                    f"Unrecognized joint: {ros_joint_name}"
                )
                continue

            # edge case: position, velocity, and effort arrays may be empty

            if index < len(message.position):
                joint.readback.put(message.position[index])

            if index < len(message.velocity):
                joint.velocity.put(message.velocity[index])

            if index < len(message.effort):
                joint.effort.put(message.effort[index])

            updated_joints.append(ros_joint_name)

        if updated_joints:
            self.print_joint_states(updated_joints)


    def process_ros_events(self, timeout_sec: float = 0.1) -> None:
        if rclpy.ok():
            rclpy.spin_once(
                self._ros_node,
                timeout_sec = timeout_sec
            )

    def list_topics(self):
        topics = self._ros_node.list_topics()

        for topic_name, topic_types in topics:
            print(f"{topic_name}: {topic_types}")

        return topics
    
    def print_joint_states(self, joint_names=None) -> None:
        if joint_names is None:
            joint_names = self._joint_name_map.keys()

        print("-" * 78)

        for joint_name in joint_names:
            joint = self._joint_name_map[joint_name]

            print(
                f"{joint_name:<10} "
                f"position={joint.readback.get():>12.6f}  "
                f"velocity={joint.velocity.get():>12.6f}  "
                f"effort={joint.effort.get():>12.6f}"
            )

    def close(self):
        self._ros_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        

if __name__ == "__main__":
    arm = Robotic_Arm(name="arm")

    try:
        print("Listening to /joint_states. Press Ctrl+C to stop.")
        print("Expected joint names:", ", ".join(arm._joint_name_map))

        while rclpy.ok():
            arm.process_ros_events(timeout_sec=0.1)
 
    except KeyboardInterrupt:
        print("\nStopping.")

    finally:
        arm.close()