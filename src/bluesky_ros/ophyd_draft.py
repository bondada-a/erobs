from ophyd import Device, Component as Cpt, EpicsSignal, EpicsSignalRO
import rclpy
from rclpy.node import Node
import time

"""
My first draft of an ophyd device yay!
"""

class ROS_Node(Node):
    def __init__(self):
        super().__init__("arm_node")

    def list_topics(self):
        return self.get_topic_names_and_types()
    
class Joint(Device):
    def __init__(self, device, **kwargs):
        super().__init__(device, **kwargs)

    # ros topic - can you see it, use rclpy to list it

    # where joint is
    readback = Cpt(EpicsSignalRO, "RB")
    # where joint should go
    setpoint = Cpt(EpicsSignal, "SP")


class Robotic_Arm(Device):

    # class init
    def __init__(self, prefix="", *, name, **kwargs):
        super().__init__(prefix=prefix, name=name, **kwargs)

        if not rclpy.ok():
            rclpy.init()

        self.ros = ROS_Node()

    # Joint components
    base = Cpt(Joint, "base")
    shoulder = Cpt(Joint, "shoulder")
    elbow = Cpt(Joint, "elbow")
    w1 = Cpt(Joint, "w1")
    w2 = Cpt(Joint, "w2")
    w3 = Cpt(Joint, "w3")

    def list_topics(self):
        time.sleep(2)
        topics = self.ros.list_topics()
        for topic, types in topics:
             print(f"{topic}: {types}")

if __name__ == "__main__":
    arm = Robotic_Arm(name="arm")
    arm.list_topics()