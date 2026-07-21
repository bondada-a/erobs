from ophyd import Device, Component as Cpt, EpicsSignal, EpicsSignalRO
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JoinState
import time

class ROS_Node(Node):
    def __init__(self):
        super().__init__("arm_node")

        self._joint_state_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            joint_state_callback,
            qos_profile_sensor_data,
        )


    def list_topics(self):
        return self.get_topic_names_and_types()
    
class Joint(Device):
    def __init__(self, device, **kwargs):
        super().__init__(device, **kwargs)

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
        for index, ros_joint_name in enumerate(message.name):
            joint = self._joint_name_map.get(ros_joint_name)

            if joint is None:
                self._ros_node.get_logger().warning(
                    f"Unrecognized joint: {ros_joint_name}"
                )
                continue

            if index < len(message.position):
                joint.readback.put(message.position[index])

            if index < len(message.velocity):
                joint.velocity.put(message.velocity[index])

            if index < len(message.effort):
                joint.effort.put(message.effort[index])

    def list_topics(self):
        topics = self._ros_node.list_topics()

        for topic_name, topic_types in topics:
            print(f"{topic_name}: {topic_types}")

        return topics
    
    def print_joint_states(self):
        for joint_name, joint in self._joint_name_map.items():
            print(
                f"{joint_name:10} "
                f"position={joint.readback.get()}  "
                f"velocity={joint.velocity.get()}  "
                f"effort={joint.effort.get()}"
            )

    def close(self):
        self._ros_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        

if __name__ == "__main__":
    arm = Robotic_Arm(name="arm")

    try:
        print("Listening to /joint_states. Press Ctrl+C to stop.")

        while rclpy.ok():
            arm.process_ros_events(timeout_sec=0.1)
            arm.print_joint_states()
            print("-" * 70)

    except KeyboardInterrupt:
        print("\nStopping.")

    finally:
        arm.close()