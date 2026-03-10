#!/usr/bin/env python3
"""ZED eye-to-hand calibration script.

Collects samples of (robot FK, ChArUco detection) pairs and solves
for the base_link -> zed_optical_frame transform.

Usage:
    1. Launch robot driver + ZED camera
    2. Run this script
    3. Move robot to diverse poses using teach pendant
    4. Press ENTER to take each sample, 's' to solve, 'q' to quit

The script reads:
    - Robot FK from TF (base_link -> tool0)
    - ChArUco board pose from ZED image using OpenCV
"""

import sys
import time
import json
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped


# === CONFIGURATION ===
# Must match your printed ChArUco board
CHARUCO_SQUARES_X = 5
CHARUCO_SQUARES_Y = 7
CHARUCO_SQUARE_LENGTH = 0.02429  # meters — MEASURE YOUR PRINT
CHARUCO_MARKER_LENGTH = 0.01500  # meters — MEASURE YOUR PRINT
ARUCO_DICT = cv2.aruco.DICT_5X5_250

# ZED topics
ZED_IMAGE_TOPIC = "/zed/zed_node/rgb/color/rect/image"
ZED_CAMERA_INFO_TOPIC = "/zed/zed_node/rgb/color/rect/camera_info"

# TF frames
ROBOT_BASE_FRAME = "base_link"
ROBOT_EE_FRAME = "tool0"

# Output
OUTPUT_FILE = "zed_calibration_result.json"


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("zed_calibration")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._latest_image = None
        self._camera_matrix = None
        self._dist_coeffs = None

        self.create_subscription(Image, ZED_IMAGE_TOPIC, self._on_image, qos)
        self.create_subscription(CameraInfo, ZED_CAMERA_INFO_TOPIC, self._on_camera_info, qos)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ChArUco board
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        self._charuco_board = cv2.aruco.CharucoBoard(
            (CHARUCO_SQUARES_X, CHARUCO_SQUARES_Y),
            CHARUCO_SQUARE_LENGTH,
            CHARUCO_MARKER_LENGTH,
            self._aruco_dict,
        )
        self._charuco_detector = cv2.aruco.CharucoDetector(self._charuco_board)

    def _on_image(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "bgra8":
            self._latest_image = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        elif msg.encoding == "rgba8":
            self._latest_image = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        elif msg.encoding == "bgr8":
            self._latest_image = arr.copy()
        else:
            self._latest_image = arr[:, :, :3].copy()

    def _on_camera_info(self, msg):
        self._camera_matrix = np.array(msg.k).reshape(3, 3)
        self._dist_coeffs = np.array(msg.d[:5])  # Use first 5 (plumb_bob)

    def spin_a_bit(self, duration=0.5):
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def get_robot_pose(self):
        """Get base_link -> tool0 as 4x4 matrix."""
        try:
            t = self._tf_buffer.lookup_transform(
                ROBOT_BASE_FRAME, ROBOT_EE_FRAME, rclpy.time.Time()
            )
            return tf_to_matrix(t)
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None

    def detect_charuco(self):
        """Detect ChArUco board and return camera->board pose as 4x4 matrix."""
        if self._latest_image is None:
            self.get_logger().error("No image available")
            return None
        if self._camera_matrix is None:
            self.get_logger().error("No camera intrinsics available")
            return None

        gray = cv2.cvtColor(self._latest_image, cv2.COLOR_BGR2GRAY)

        # Detect ChArUco board
        charuco_corners, charuco_ids, marker_corners, marker_ids = \
            self._charuco_detector.detectBoard(gray)

        if charuco_ids is None or len(charuco_ids) < 6:
            n = 0 if charuco_ids is None else len(charuco_ids)
            self.get_logger().warn(f"Only {n} ChArUco corners found (need >= 6)")
            return None

        n_markers = 0 if marker_ids is None else len(marker_ids)

        # Estimate board pose using solvePnP
        obj_points, img_points = self._charuco_board.matchImagePoints(
            charuco_corners, charuco_ids
        )

        success, rvec, tvec = cv2.solvePnP(
            obj_points, img_points,
            self._camera_matrix, self._dist_coeffs,
        )

        if not success:
            self.get_logger().warn("Pose estimation failed")
            return None

        # Convert to 4x4 matrix
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()

        self.get_logger().info(
            f"Detected {len(charuco_ids)} corners, {n_markers} markers | "
            f"Board at ({tvec[0,0]:.3f}, {tvec[1,0]:.3f}, {tvec[2,0]:.3f})m"
        )
        return T

    def get_image_preview(self):
        """Return image with detected markers and board pose drawn."""
        if self._latest_image is None:
            return None
        img = self._latest_image.copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        charuco_corners, charuco_ids, marker_corners, marker_ids = \
            self._charuco_detector.detectBoard(gray)

        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(img, marker_corners, marker_ids)
        if charuco_corners is not None and len(charuco_corners) > 0:
            cv2.aruco.drawDetectedCornersCharuco(img, charuco_corners, charuco_ids)

            # Draw pose axes if we have intrinsics and enough corners
            if self._camera_matrix is not None and len(charuco_corners) >= 4:
                obj_points, img_points = self._charuco_board.matchImagePoints(
                    charuco_corners, charuco_ids
                )
                if obj_points is not None and len(obj_points) >= 4:
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, img_points,
                        self._camera_matrix, self._dist_coeffs,
                    )
                    if success:
                        cv2.drawFrameAxes(img, self._camera_matrix, self._dist_coeffs,
                                          rvec, tvec, 0.05)

        # Add status text
        n_corners = 0 if charuco_corners is None else len(charuco_corners)
        n_markers = 0 if marker_ids is None else len(marker_ids)
        status = f"Markers: {n_markers} | Corners: {n_corners}"
        cv2.putText(img, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        return img


def tf_to_matrix(t: TransformStamped) -> np.ndarray:
    """Convert a TF TransformStamped to a 4x4 matrix."""
    tr = t.transform.translation
    rot = t.transform.rotation
    qx, qy, qz, qw = rot.x, rot.y, rot.z, rot.w

    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tr.x, tr.y, tr.z]
    return T


def matrix_to_quat(R):
    """Rotation matrix to quaternion (xyzw)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def solve_eye_to_hand(R_base2ee_list, t_base2ee_list, R_cam2target_list, t_cam2target_list):
    """Solve eye-to-hand calibration using all OpenCV methods.

    For eye-to-hand, we invert the robot poses before passing to calibrateHandEye.
    Input: base->ee (FK), cam->target (detection)
    Output: base->cam (what we want)
    """
    # Invert robot poses: base->ee becomes ee->base
    R_ee2base = [R.T for R in R_base2ee_list]
    t_ee2base = [-R.T @ t for R, t in zip(R_base2ee_list, t_base2ee_list)]

    methods = {
        "Tsai": cv2.CALIB_HAND_EYE_TSAI,
        "Park": cv2.CALIB_HAND_EYE_PARK,
        "Horaud": cv2.CALIB_HAND_EYE_HORAUD,
        "Andreff": cv2.CALIB_HAND_EYE_ANDREFF,
        "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    results = {}
    print("\n" + "=" * 60)
    print("Eye-to-hand calibration results: base_link -> zed_optical")
    print("=" * 60)

    for name, method in methods.items():
        try:
            R, t = cv2.calibrateHandEye(
                R_ee2base, t_ee2base,
                R_cam2target_list, [tv for tv in t_cam2target_list],
                method=method,
            )
            roll = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
            pitch = np.degrees(np.arcsin(np.clip(-R[2, 0], -1, 1)))
            yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
            qx, qy, qz, qw = matrix_to_quat(R)

            print(f"\n{name}:")
            print(f"  xyz  = ({t[0,0]:.4f}, {t[1,0]:.4f}, {t[2,0]:.4f})")
            print(f"  rpy  = ({roll:.1f}°, {pitch:.1f}°, {yaw:.1f}°)")
            print(f"  quat = ({qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f})")

            results[name] = {"R": R, "t": t, "quat": (qx, qy, qz, qw)}
        except Exception as e:
            print(f"\n{name}: FAILED - {e}")

    return results


def main():
    rclpy.init()
    node = CalibrationNode()

    # Wait for data
    print("Waiting for ZED image and camera info...")
    for _ in range(100):
        node.spin_a_bit(0.1)
        if node._latest_image is not None and node._camera_matrix is not None:
            break

    if node._latest_image is None:
        print("ERROR: No ZED image received. Is the camera running?")
        return
    if node._camera_matrix is None:
        print("ERROR: No camera info received.")
        return

    print(f"Camera ready: {node._latest_image.shape[1]}x{node._latest_image.shape[0]}")
    print(f"Intrinsics: fx={node._camera_matrix[0,0]:.1f}, fy={node._camera_matrix[1,1]:.1f}")

    # Collect samples
    R_base2ee_list = []
    t_base2ee_list = []
    R_cam2target_list = []
    t_cam2target_list = []

    # Open live preview window
    WINDOW_NAME = "ZED ChArUco Calibration"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 960, 540)

    print("\n" + "=" * 60)
    print("Move the robot to diverse poses with the ChArUco visible.")
    print("Live preview is shown in the OpenCV window.")
    print("Commands:")
    print("  ENTER  - take sample")
    print("  s      - solve with current samples")
    print("  q      - quit")
    print("=" * 60)

    sample_num = 0
    import select as _select

    def _update_preview():
        """Update the live preview window. Non-blocking."""
        node.spin_a_bit(0.05)
        img = node.get_image_preview()
        if img is not None:
            info = f"Samples: {sample_num}"
            cv2.putText(img, info, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.imshow(WINDOW_NAME, img)
        cv2.waitKey(1)

    print(f"[{sample_num} samples] Press ENTER=sample, s=solve, q=quit")
    while True:
        # Update preview continuously while waiting for input
        while not _select.select([sys.stdin], [], [], 0.0)[0]:
            _update_preview()

        cmd = sys.stdin.readline().strip().lower()

        if cmd == "q":
            break
        elif cmd == "s":
            if sample_num < 5:
                print(f"Need at least 5 samples (have {sample_num})")
                continue
            results = solve_eye_to_hand(
                R_base2ee_list, t_base2ee_list,
                R_cam2target_list, t_cam2target_list,
            )
            # Save best result (Park is generally most reliable)
            if "Park" in results:
                save_result(results["Park"], sample_num)
            elif results:
                name = list(results.keys())[0]
                save_result(results[name], sample_num)
        elif cmd == "":
            # Take sample
            node.spin_a_bit(0.3)  # Get fresh data

            T_base_ee = node.get_robot_pose()
            if T_base_ee is None:
                print("  FAILED: Could not read robot pose from TF")
                continue

            T_cam_target = node.detect_charuco()
            if T_cam_target is None:
                print("  FAILED: Could not detect ChArUco board")
                continue

            R_base2ee_list.append(T_base_ee[:3, :3])
            t_base2ee_list.append(T_base_ee[:3, 3].reshape(3, 1))
            R_cam2target_list.append(T_cam_target[:3, :3])
            t_cam2target_list.append(T_cam_target[:3, 3].reshape(3, 1))

            sample_num += 1
            ee_pos = T_base_ee[:3, 3]
            print(f"  Sample {sample_num}: ee=({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})")

        print(f"[{sample_num} samples] Press ENTER=sample, s=solve, q=quit")

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


def save_result(result, n_samples):
    R = result["R"]
    t = result["t"]
    qx, qy, qz, qw = result["quat"]

    data = {
        "calibration": "base_link -> zed_left_camera_optical_frame",
        "type": "eye-to-hand",
        "n_samples": n_samples,
        "translation": {"x": float(t[0, 0]), "y": float(t[1, 0]), "z": float(t[2, 0])},
        "quaternion": {"x": float(qx), "y": float(qy), "z": float(qz), "w": float(qw)},
        "static_tf_command": (
            f"ros2 run tf2_ros static_transform_publisher "
            f"--x {t[0,0]:.6f} --y {t[1,0]:.6f} --z {t[2,0]:.6f} "
            f"--qx {qx:.6f} --qy {qy:.6f} --qz {qz:.6f} --qw {qw:.6f} "
            f"--frame-id base_link --child-frame-id zed_left_camera_optical_frame"
        ),
    }

    with open(OUTPUT_FILE, "w") as f:
        json.dump(data, f, indent=2)
    print(f"\nSaved to {OUTPUT_FILE}")
    print(f"To publish: {data['static_tf_command']}")


if __name__ == "__main__":
    main()
