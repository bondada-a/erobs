#!/usr/bin/env python3
"""
Standalone script to test contour detection with Zivid camera.

This script:
1. Calls the Zivid capture service
2. Receives image + point cloud
3. Detects objects of ANY shape using contour detection
4. Displays the detection with visualization (contours + centroids)
5. Prints 3D poses for all detected objects
6. Allows parameter tuning via keyboard

Unlike circle detection, this works for squares, triangles, irregular shapes,
or any closed boundary that meets the area criteria.

Usage:
    # Make sure beambot_bringup is running first:
    ros2 launch beambot beambot_bringup.launch.py

    # Then run this script:
    python3 test_contour_detection.py

    # Or if installed:
    ros2 run beambot test_contour_detection.py

Controls:
    SPACE  - Capture image and detect contours
    +/-    - Adjust min_area threshold
    [/]    - Adjust max_area threshold
    ,/.    - Adjust Canny low threshold
    </> (SHIFT+,/.)  - Adjust Canny high threshold
    Q      - Quit
"""

import cv2
import numpy as np
import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Pose, PoseStamped
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

# TF2 for coordinate transforms
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped

# Zivid ArUco detection service (used to trigger capture)
from zivid_interfaces.srv import CaptureAndDetectMarkers


@dataclass
class ContourDetectionParams:
    """Parameters for contour-based object detection."""
    min_area: int = 500          # Min contour area in pixels²
    max_area: int = 50000        # Max contour area in pixels²
    blur_kernel: int = 5         # Gaussian blur kernel size (must be odd)
    canny_low: int = 50          # Canny edge detection lower threshold
    canny_high: int = 150        # Canny edge detection upper threshold
    search_radius: int = 10      # Pixels to search for valid depth
    approx_epsilon: float = 0.02 # Contour approximation epsilon
    row_tolerance: int = 50      # Y-pixel tolerance for grouping into rows


def sort_contours_reading_order(
    detections: List[Tuple],
    row_tolerance: int = 50
) -> List[Tuple]:
    """Sort contours in reading order: left-to-right, top-to-bottom.

    Groups objects into rows based on Y-coordinate proximity,
    then sorts each row by X-coordinate.

    Args:
        detections: List of (contour, cx, cy, area, vertices) tuples
        row_tolerance: Max Y-pixel difference for objects to be in same row

    Returns:
        Sorted list of detections
    """
    if not detections:
        return detections

    # Sort by Y first to process top-to-bottom
    sorted_by_y = sorted(detections, key=lambda d: d[2])  # d[2] = cy

    # Group into rows
    rows = []
    current_row = [sorted_by_y[0]]
    current_row_y = sorted_by_y[0][2]

    for detection in sorted_by_y[1:]:
        cy = detection[2]
        # If Y is close enough to current row, add to same row
        if abs(cy - current_row_y) <= row_tolerance:
            current_row.append(detection)
        else:
            # Start new row
            rows.append(current_row)
            current_row = [detection]
            current_row_y = cy

    # Don't forget the last row
    rows.append(current_row)

    # Sort each row by X (left to right) and flatten
    result = []
    for row in rows:
        row_sorted = sorted(row, key=lambda d: d[1])  # d[1] = cx
        result.extend(row_sorted)

    return result


class ContourDetectionTest(Node):
    def __init__(self):
        super().__init__('contour_detection_test')

        self.bridge = CvBridge()

        # Storage for received data
        self.latest_image: Optional[Image] = None
        self.latest_cloud: Optional[PointCloud2] = None

        # Detection parameters (tune these for your setup!)
        self.params = ContourDetectionParams()

        # Parameter adjustment step sizes
        self.area_step = 500       # pixels²
        self.canny_step = 10

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'color/image_color', self.on_image, 10
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2, 'points/xyzrgba', self.on_cloud, 10
        )

        # Capture service client
        self.capture_client = self.create_client(Trigger, 'capture')

        # Marker detection service (used to trigger capture)
        self.marker_client = self.create_client(
            CaptureAndDetectMarkers, '/capture_and_detect_markers'
        )

        # Camera frame (must match your TF tree)
        self.camera_frame = 'zivid_optical_frame'

        # TF2 for transforms
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Store detected objects for display
        self.detected_objects: List[dict] = []

        self.get_logger().info('=' * 60)
        self.get_logger().info('Contour Detection Test')
        self.get_logger().info('=' * 60)
        self._print_params()
        self.get_logger().info('=' * 60)

    def _print_params(self):
        """Print current detection parameters."""
        self.get_logger().info(f'Detection parameters:')
        self.get_logger().info(f'  min_area: {self.params.min_area} px²')
        self.get_logger().info(f'  max_area: {self.params.max_area} px²')
        self.get_logger().info(f'  canny_low: {self.params.canny_low}')
        self.get_logger().info(f'  canny_high: {self.params.canny_high}')
        self.get_logger().info(f'  blur_kernel: {self.params.blur_kernel}')

    def on_image(self, msg: Image):
        """Store latest image."""
        self.latest_image = msg
        self.get_logger().debug(f'Received image: {msg.width}x{msg.height}')

    def on_cloud(self, msg: PointCloud2):
        """Store latest point cloud."""
        self.latest_cloud = msg
        self.get_logger().debug(f'Received point cloud: {msg.width}x{msg.height} points')

    def wait_for_service(self) -> bool:
        """Wait for capture service to be available."""
        self.get_logger().info('Waiting for capture service...')
        if not self.marker_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Zivid capture service not available!')
            self.get_logger().error('Make sure zivid_camera node is running:')
            self.get_logger().error('  ros2 launch zivid_camera zivid_camera.launch.py')
            return False
        self.get_logger().info('Capture service available!')
        return True

    def capture(self) -> bool:
        """Trigger a capture and wait for data."""
        self.get_logger().info('Triggering capture...')

        # Clear previous data
        self.latest_image = None
        self.latest_cloud = None

        # Call marker detection service (triggers capture + publishes image/cloud)
        request = CaptureAndDetectMarkers.Request()
        request.marker_ids = [999]  # Dummy ID - we don't care about markers
        request.marker_dictionary = "aruco4x4_50"

        future = self.marker_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done():
            self.get_logger().error('Capture service call timed out!')
            return False

        self.get_logger().info('Capture triggered, waiting for data...')

        # Wait for image and cloud to arrive
        # Point cloud takes 3-4s longer to transmit than image
        timeout = 8.0
        elapsed = 0.0

        while elapsed < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            if elapsed > 0 and int(elapsed) % 1 == 0 and int(elapsed * 10) % 10 == 0:
                img_status = "✓" if self.latest_image else "waiting"
                cloud_status = "✓" if self.latest_cloud else "waiting"
                self.get_logger().info(f'  {elapsed:.1f}s - image: {img_status}, cloud: {cloud_status}')

            if self.latest_image is not None and self.latest_cloud is not None:
                self.get_logger().info('Data received!')
                return True
            elapsed += 0.1

        self.get_logger().error('Timeout waiting for image/cloud data!')
        return False

    def detect_contours(
        self, rgb_image: np.ndarray
    ) -> List[Tuple[np.ndarray, int, int, int, int]]:
        """
        Detect contours in RGB image and filter by area.

        Returns:
            List of (contour_points, center_x, center_y, area, num_vertices)
        """
        # Convert to grayscale
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

        # Blur to reduce noise
        blurred = cv2.GaussianBlur(
            gray, (self.params.blur_kernel, self.params.blur_kernel), 0
        )

        # Edge detection
        edges = cv2.Canny(blurred, self.params.canny_low, self.params.canny_high)

        # Find contours
        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        self.get_logger().info(f'Found {len(contours)} raw contours')

        # Filter by area and extract info
        result = []
        for contour in contours:
            area = cv2.contourArea(contour)

            # Filter by area
            if area < self.params.min_area or area > self.params.max_area:
                continue

            # Get centroid using moments
            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue  # Skip degenerate contours

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # Approximate contour to count vertices
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, self.params.approx_epsilon * perimeter, True)
            num_vertices = len(approx)

            result.append((contour, cx, cy, int(area), num_vertices))

        self.get_logger().info(
            f'Filtered to {len(result)} contours in area range '
            f'[{self.params.min_area}, {self.params.max_area}]'
        )

        # Sort in reading order (left-to-right, top-to-bottom)
        result = sort_contours_reading_order(result, self.params.row_tolerance)
        self.get_logger().info(f'Sorted {len(result)} objects in reading order')

        return result

    def get_3d_position(
        self,
        cloud: PointCloud2,
        cx: int,
        cy: int,
    ) -> Optional[Tuple[float, float, float]]:
        """Get 3D position from organized point cloud at pixel (cx, cy)."""
        width = cloud.width
        height = cloud.height
        point_step = cloud.point_step

        def get_xyz_at(u: int, v: int) -> Optional[Tuple[float, float, float]]:
            """Extract XYZ at pixel (u, v)."""
            if u < 0 or u >= width or v < 0 or v >= height:
                return None

            offset = v * cloud.row_step + u * point_step

            try:
                x, y, z = struct.unpack_from('<fff', cloud.data, offset)
            except struct.error:
                return None

            if np.isnan(x) or np.isnan(y) or np.isnan(z):
                return None

            if x == 0.0 and y == 0.0 and z == 0.0:
                return None

            return (x, y, z)

        # Try center first
        xyz = get_xyz_at(cx, cy)
        if xyz is not None:
            return xyz

        # Search in expanding squares around center
        for r in range(1, self.params.search_radius + 1):
            for du in range(-r, r + 1):
                for dv in range(-r, r + 1):
                    if abs(du) == r or abs(dv) == r:
                        xyz = get_xyz_at(cx + du, cy + dv)
                        if xyz is not None:
                            return xyz

        return None

    def transform_to_base_link(self, pose_camera: PoseStamped) -> Optional[PoseStamped]:
        """Transform pose from camera frame to base_link."""
        try:
            if not self._tf_buffer.can_transform(
                "base_link",
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            ):
                self.get_logger().warning(
                    f"TF {self.camera_frame} -> base_link not available"
                )
                return None

            transform = self._tf_buffer.lookup_transform(
                "base_link",
                self.camera_frame,
                rclpy.time.Time()
            )
            pose_base = do_transform_pose_stamped(pose_camera, transform)
            pose_base.header.frame_id = "base_link"

            return pose_base

        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return None

    def classify_shape(self, num_vertices: int) -> str:
        """Classify shape based on number of vertices."""
        if num_vertices == 3:
            return "Triangle"
        elif num_vertices == 4:
            return "Rectangle/Square"
        elif num_vertices == 5:
            return "Pentagon"
        elif num_vertices == 6:
            return "Hexagon"
        elif num_vertices > 6 and num_vertices < 10:
            return f"Polygon ({num_vertices})"
        else:
            return "Circle/Ellipse"

    def visualize_detection(
        self,
        rgb_image: np.ndarray,
        detections: List[Tuple[np.ndarray, int, int, int, int]],
    ) -> np.ndarray:
        """
        Draw all detected contours on image with annotations.

        Returns:
            Annotated image (BGR format for OpenCV display)
        """
        # Convert RGB to BGR for OpenCV
        display = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

        if not detections:
            # No detection - show message
            cv2.putText(
                display, 'No objects detected!', (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2
            )
            cv2.putText(
                display, 'Try adjusting area thresholds (+/-/[/])', (50, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
            )
        else:
            # Generate colors for different objects
            colors = [
                (0, 255, 0),    # Green
                (255, 0, 0),    # Blue
                (0, 255, 255),  # Yellow
                (255, 0, 255),  # Magenta
                (255, 255, 0),  # Cyan
                (128, 0, 255),  # Purple
                (255, 128, 0),  # Orange
            ]

            # Draw each detected contour
            for i, (contour, cx, cy, area, num_vertices) in enumerate(detections):
                color = colors[i % len(colors)]

                # Draw contour outline
                cv2.drawContours(display, [contour], 0, color, 2)

                # Draw centroid
                cv2.circle(display, (cx, cy), 8, (0, 0, 255), -1)  # Red dot

                # Draw crosshairs at centroid
                cv2.line(display, (cx - 15, cy), (cx + 15, cy), (0, 0, 255), 2)
                cv2.line(display, (cx, cy - 15), (cx, cy + 15), (0, 0, 255), 2)

                # Draw label with area and shape
                shape_name = self.classify_shape(num_vertices)
                label = f"#{i+1}: {shape_name} ({area} px²)"
                cv2.putText(
                    display, label, (cx + 15, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
                )

        # Draw parameter info at top
        text_y = 30
        cv2.putText(
            display, f'Params: area=[{self.params.min_area}, {self.params.max_area}] '
                     f'canny=[{self.params.canny_low}, {self.params.canny_high}] '
                     f'row_tol={self.params.row_tolerance}',
            (10, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
        )
        text_y += 25
        cv2.putText(
            display, f'Detected: {len(detections)} object(s) (sorted L-to-R, T-to-B)',
            (10, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
        )

        # Instructions at bottom
        h = display.shape[0]
        cv2.putText(
            display, 'SPACE: capture | +/- area | [/] max | ,/. canny | R/r row_tol | Q: quit',
            (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1
        )

        return display

    def run_detection(self):
        """Run detection on latest captured data."""
        if self.latest_image is None or self.latest_cloud is None:
            self.get_logger().error('No data available!')
            return

        # Convert ROS Image to OpenCV
        rgb_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='rgb8')

        self.get_logger().info('Running contour detection...')

        # Detect contours
        detections = self.detect_contours(rgb_image)

        # Clear previous detected objects
        self.detected_objects = []

        if detections:
            self.get_logger().info('=' * 60)
            self.get_logger().info('DETECTION RESULTS:')
            self.get_logger().info('=' * 60)

            for i, (contour, cx, cy, area, num_vertices) in enumerate(detections):
                shape_name = self.classify_shape(num_vertices)
                self.get_logger().info(f'Object #{i+1}: {shape_name}')
                self.get_logger().info(f'  Pixel: ({cx}, {cy}), Area: {area} px², Vertices: {num_vertices}')

                # Get 3D position
                xyz = self.get_3d_position(self.latest_cloud, cx, cy)

                if xyz is not None:
                    x, y, z = xyz
                    self.get_logger().info(f'  3D (camera): ({x:.4f}, {y:.4f}, {z:.4f}) m')
                    self.get_logger().info(f'               ({x*1000:.1f}, {y*1000:.1f}, {z*1000:.1f}) mm')

                    # Transform to base_link
                    pose_camera = PoseStamped()
                    pose_camera.header.frame_id = self.camera_frame
                    pose_camera.header.stamp = self.latest_image.header.stamp
                    pose_camera.pose.position.x = x
                    pose_camera.pose.position.y = y
                    pose_camera.pose.position.z = z
                    pose_camera.pose.orientation.w = 1.0

                    pose_base = self.transform_to_base_link(pose_camera)

                    if pose_base is not None:
                        pb = pose_base.pose.position
                        self.get_logger().info(f'  3D (base):   ({pb.x:.4f}, {pb.y:.4f}, {pb.z:.4f}) m')
                        self.get_logger().info(f'               ({pb.x*1000:.1f}, {pb.y*1000:.1f}, {pb.z*1000:.1f}) mm')

                    # Store for later
                    self.detected_objects.append({
                        'index': i,
                        'shape': shape_name,
                        'pixel': (cx, cy),
                        'area': area,
                        'vertices': num_vertices,
                        'xyz_camera': xyz,
                        'pose_base': pose_base,
                    })
                else:
                    self.get_logger().warning(f'  No valid depth at centroid!')

                self.get_logger().info('')

            self.get_logger().info('=' * 60)
        else:
            self.get_logger().warning('No contours detected matching area criteria!')
            self.get_logger().info(f'Try adjusting parameters:')
            self.get_logger().info(f'  - Lower min_area (current: {self.params.min_area})')
            self.get_logger().info(f'  - Raise max_area (current: {self.params.max_area})')
            self.get_logger().info(f'  - Adjust canny thresholds ({self.params.canny_low}, {self.params.canny_high})')

        # Visualize
        display = self.visualize_detection(rgb_image, detections)

        # Show image
        cv2.imshow('Contour Detection', display)

        return detections

    def adjust_param(self, param_name: str, delta: int):
        """Adjust a detection parameter and re-run detection."""
        if param_name == 'min_area':
            self.params.min_area = max(100, self.params.min_area + delta)
        elif param_name == 'max_area':
            self.params.max_area = max(1000, self.params.max_area + delta)
        elif param_name == 'canny_low':
            self.params.canny_low = max(10, min(200, self.params.canny_low + delta))
        elif param_name == 'canny_high':
            self.params.canny_high = max(50, min(300, self.params.canny_high + delta))
        elif param_name == 'row_tolerance':
            self.params.row_tolerance = max(10, min(200, self.params.row_tolerance + delta))

        self.get_logger().info(f'{param_name} = {getattr(self.params, param_name)}')

        # Re-run detection with new params
        if self.latest_image is not None:
            self.run_detection()

    def run(self):
        """Main loop - capture and detect on keypress."""
        if not self.wait_for_service():
            return

        self.get_logger().info('')
        self.get_logger().info('Controls:')
        self.get_logger().info('  SPACE     - Capture image and detect contours')
        self.get_logger().info('  + / -     - Increase/decrease min_area')
        self.get_logger().info('  ] / [     - Increase/decrease max_area')
        self.get_logger().info('  . / ,     - Increase/decrease canny_low')
        self.get_logger().info('  > / <     - Increase/decrease canny_high')
        self.get_logger().info('  R / r     - Increase/decrease row_tolerance (for sorting)')
        self.get_logger().info('  Q         - Quit')
        self.get_logger().info('')
        self.get_logger().info('Sorting: Objects are labeled 1,2,3... left-to-right, top-to-bottom')
        self.get_logger().info(f'         row_tolerance={self.params.row_tolerance}px (objects within this Y-distance are same row)')
        self.get_logger().info('')

        # Create a resizable window
        cv2.namedWindow('Contour Detection', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Contour Detection', 1280, 800)

        # Initial capture
        if self.capture():
            self.run_detection()
        else:
            # Show blank window with instructions
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                blank, 'Capture failed - press SPACE to retry', (50, 240),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2
            )
            cv2.imshow('Contour Detection', blank)

        # Main loop
        while True:
            key = cv2.waitKey(100) & 0xFF

            # Spin ROS callbacks
            rclpy.spin_once(self, timeout_sec=0.01)

            if key == ord('q') or key == ord('Q'):
                self.get_logger().info('Quitting...')
                break
            elif key == ord(' '):  # Space bar
                self.get_logger().info('Capturing...')
                if self.capture():
                    self.run_detection()
            # Parameter adjustments
            elif key == ord('+') or key == ord('='):
                self.adjust_param('min_area', self.area_step)
            elif key == ord('-') or key == ord('_'):
                self.adjust_param('min_area', -self.area_step)
            elif key == ord(']') or key == ord('}'):
                self.adjust_param('max_area', self.area_step * 10)
            elif key == ord('[') or key == ord('{'):
                self.adjust_param('max_area', -self.area_step * 10)
            elif key == ord('.'):
                self.adjust_param('canny_low', self.canny_step)
            elif key == ord(','):
                self.adjust_param('canny_low', -self.canny_step)
            elif key == ord('>'):
                self.adjust_param('canny_high', self.canny_step)
            elif key == ord('<'):
                self.adjust_param('canny_high', -self.canny_step)
            elif key == ord('R'):
                self.adjust_param('row_tolerance', 10)
            elif key == ord('r'):
                self.adjust_param('row_tolerance', -10)

        cv2.destroyAllWindows()


def main():
    rclpy.init()

    node = ContourDetectionTest()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
