#!/usr/bin/env python3
"""Trigger /capture_2d and save the resulting frame to a given path."""
import rclpy, time, subprocess, threading, sys
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/cap.png"

class Grab(Node):
    def __init__(self):
        super().__init__('grab_capN')
        self.bridge = CvBridge(); self.got = False
        self.sub = self.create_subscription(Image, '/color/image_color', self.cb, 10)
    def cb(self, msg):
        if self.got:
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite(out, img)
        self.got = True

rclpy.init(); n = Grab()
def trig():
    time.sleep(1.0)
    subprocess.run(['ros2', 'service', 'call', '/capture_2d', 'std_srvs/srv/Trigger'],
                   capture_output=True)
threading.Thread(target=trig, daemon=True).start()
t = time.time()
while rclpy.ok() and not n.got and time.time() - t < 15:
    rclpy.spin_once(n, timeout_sec=0.1)
print('SAVED' if n.got else 'NOFRAME', out)
n.destroy_node(); rclpy.shutdown()
