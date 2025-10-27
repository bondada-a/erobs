#!/usr/bin/env python3

"""
Test script to compare AprilTag (apriltag_ros) vs ArUco (Zivid built-in) detection.

IMPORTANT: You need DIFFERENT physical markers:
- AprilTag: For apriltag_ros (currently working with tag36h11)
- ArUco: For Zivid built-in (e.g., aruco4x4_50)

They are NOT compatible!
"""

import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from zivid_interfaces.srv import CaptureAndDetectMarkers
import sys


class MarkerDetectionComparison(Node):
    def __init__(self):
        super().__init__('marker_detection_comparison')

        # Service client for Zivid detection
        self.zivid_client = self.create_client(
            CaptureAndDetectMarkers,
            '/capture_and_detect_markers'
        )

        self.get_logger().info('Marker Detection Comparison Tool')
        self.get_logger().info('='*60)

    def test_apriltag_detection(self, tag_id):
        """Test current AprilTag detection (via apriltag_ros)."""
        self.get_logger().info('\n--- Testing AprilTag Detection ---')
        self.get_logger().info('Method: apriltag_ros (2D image-based)')
        self.get_logger().info(f'Looking for AprilTag ID: {tag_id}')

        # Subscribe to detections
        detection_received = False

        def detection_callback(msg):
            nonlocal detection_received
            for detection in msg.detections:
                if detection.id == tag_id:
                    detection_received = True
                    self.get_logger().info(f'\n✓ AprilTag {tag_id} detected!')
                    self.get_logger().info(f'  Family: {detection.family}')
                    self.get_logger().info(f'  Centre: [{detection.centre.x:.2f}, {detection.centre.y:.2f}] px')
                    self.get_logger().info(f'  Decision margin: {detection.decision_margin:.2f}')
                    self.get_logger().info(f'  Hamming: {detection.hamming}')

        sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            detection_callback,
            10
        )

        # Wait for detection
        import time
        timeout = 5.0
        start = time.time()
        while not detection_received and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not detection_received:
            self.get_logger().warn(f'✗ No AprilTag {tag_id} detected in {timeout}s')
            self.get_logger().info('  Make sure AprilTag is visible to camera')

        self.destroy_subscription(sub)
        return detection_received

    def test_zivid_detection(self, marker_id, dictionary='aruco4x4_50'):
        """Test Zivid built-in ArUco detection (3D point cloud-based)."""
        self.get_logger().info('\n--- Testing Zivid Built-in Detection ---')
        self.get_logger().info('Method: Zivid SDK (3D point cloud-based)')
        self.get_logger().info(f'Looking for ArUco ID: {marker_id}')
        self.get_logger().info(f'Dictionary: {dictionary}')

        # Wait for service
        if not self.zivid_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('✗ Zivid capture_and_detect_markers service not available!')
            self.get_logger().info('  Make sure zivid_camera node is running with 3D settings')
            return False

        # Call service
        request = CaptureAndDetectMarkers.Request()
        request.marker_ids = [marker_id]
        request.marker_dictionary = dictionary

        self.get_logger().info('Capturing and detecting... (this may take a few seconds)')

        try:
            future = self.zivid_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

            if future.done():
                result = future.result()

                if result.success:
                    if result.detection_result.detected_markers:
                        for marker in result.detection_result.detected_markers:
                            self.get_logger().info(f'\n✓ ArUco marker {marker.id} detected!')
                            self.get_logger().info(f'  Pose (camera frame):')
                            self.get_logger().info(f'    Position: [{marker.pose.position.x:.3f}, '
                                                   f'{marker.pose.position.y:.3f}, '
                                                   f'{marker.pose.position.z:.3f}] m')
                            self.get_logger().info(f'    Orientation: [{marker.pose.orientation.x:.3f}, '
                                                   f'{marker.pose.orientation.y:.3f}, '
                                                   f'{marker.pose.orientation.z:.3f}, '
                                                   f'{marker.pose.orientation.w:.3f}]')
                            self.get_logger().info(f'  4 Corners in 3D:')
                            for i, corner in enumerate(marker.corners_in_camera_coordinates):
                                self.get_logger().info(f'    Corner {i}: [{corner.x:.3f}, {corner.y:.3f}, {corner.z:.3f}] m')
                        return True
                    else:
                        self.get_logger().warn(f'✗ No ArUco marker {marker_id} detected')
                        self.get_logger().info('  Make sure ArUco marker (not AprilTag!) is visible')
                        return False
                else:
                    self.get_logger().error(f'✗ Detection failed: {result.message}')
                    return False
            else:
                self.get_logger().error('✗ Service call timeout')
                return False

        except Exception as e:
            self.get_logger().error(f'✗ Exception during detection: {str(e)}')
            return False


def main():
    rclpy.init()

    node = MarkerDetectionComparison()

    print("\n" + "="*60)
    print("AprilTag vs Zivid ArUco Detection Comparison")
    print("="*60)

    # Get marker ID from command line or use default
    if len(sys.argv) > 1:
        marker_id = int(sys.argv[1])
    else:
        marker_id = 3

    print(f"\nTesting with marker ID: {marker_id}")
    print("\nIMPORTANT NOTES:")
    print("- AprilTag test: Uses your current tag36h11 markers")
    print("- ArUco test: REQUIRES ArUco markers (different format!)")
    print("- You cannot detect AprilTag with ArUco detector or vice versa")
    print("\nTo generate ArUco marker: https://chev.me/arucogen/")
    print("Select: DICT_4X4_50, ID: 3, Size: 100mm")
    print("="*60 + "\n")

    # Test 1: AprilTag (current working method)
    apriltag_works = node.test_apriltag_detection(marker_id)

    # Test 2: Zivid ArUco (if you have ArUco marker)
    input("\nPress Enter to test Zivid ArUco detection (or Ctrl+C to skip)...")
    zivid_works = node.test_zivid_detection(marker_id, 'aruco4x4_50')

    # Summary
    print("\n" + "="*60)
    print("COMPARISON SUMMARY")
    print("="*60)
    print(f"AprilTag (apriltag_ros):  {'✓ WORKING' if apriltag_works else '✗ NOT DETECTED'}")
    print(f"ArUco (Zivid built-in):   {'✓ WORKING' if zivid_works else '✗ NOT DETECTED'}")
    print("="*60)

    if apriltag_works and not zivid_works:
        print("\nRECOMMENDATION:")
        print("AprilTag detection is working well. Unless you need sub-mm accuracy,")
        print("stick with apriltag_ros. It's simpler and already integrated.")
    elif zivid_works:
        print("\nZivid detection is working! Compare accuracy:")
        print("- AprilTag: ~1-2mm accuracy (sufficient for most grasping)")
        print("- Zivid: ~0.5-1mm accuracy (better but requires ArUco markers)")
    else:
        print("\nNeed to set up markers and test!")

    print("\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nTest interrupted by user')
