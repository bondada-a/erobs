import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseArray

class ArucoPoseSubscriber(Node):
    def __init__(self):
        super().__init__('aruco_pose_subscriber')
        self.subscription = self.create_subscription(
            PoseArray,          # Message type
            '/aruco_poses',     # Topic name
            self.pose_callback, # Callback function
            10                  # QoS profile depth
        )
        
        # Transformation matrix from frame C to frame M (hardcoded)
        self.transform_MC = np.array([
            [-0.424,  0.034, -0.905,  0.807],
            [ 0.906, -0.004, -0.424,  0.417],
            [-0.018, -0.999, -0.029,  0.257],
            [ 0.000,  0.000,  0.000,  1.000]
        ])
        
        self.subscription  # prevent unused variable warning

    def transform_position(self, position_C):
        """Transform a position from frame C to frame M"""
        # Create homogeneous position vector
        position_C_homogeneous = np.array([
            position_C.x,
            position_C.y,
            position_C.z,
            1.0
        ])
        
        # Apply transformation
        position_M_homogeneous = self.transform_MC @ position_C_homogeneous
        
        # Return the x, y, z components
        return position_M_homogeneous[0:3]

    def pose_callback(self, msg: PoseArray):
        self.get_logger().info(f"Received PoseArray with header frame id: {msg.header.frame_id}")
        for i, pose in enumerate(msg.poses):
            # Original position in frame C
            self.get_logger().info(
                f"Marker {i} in frame C: Position(x={pose.position.x:.6f}, y={pose.position.y:.6f}, z={pose.position.z:.6f})"
            )
            
            # Transform to frame M
            pos_M = self.transform_position(pose.position)
            
            # Print transformed position
            self.get_logger().info(
                f"Marker {i} in frame M: Position(x={pos_M[0]:.6f}, y={pos_M[1]:.6f}, z={pos_M[2]:.6f})"
            )
            
            # Original orientation information
            self.get_logger().info(
                f"Marker {i} Orientation: (x={pose.orientation.x:.3f}, y={pose.orientation.y:.3f}, z={pose.orientation.z:.3f}, w={pose.orientation.w:.3f})"
            )

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Aruco Pose Subscriber Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
