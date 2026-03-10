# ZED Eye-to-Hand Calibration

How to calibrate the ZED 2i camera position relative to the UR5e robot base.

## Prerequisites

- UR5e robot driver running (TF publishing `base_link` -> `tool0`)
- ZED 2i camera connected and ZED SDK installed
- ChArUco board printed and measured (see Board Specification below)
- ChArUco board mounted on the robot end-effector (e.g. taped to the Zivid camera on the flange)

## Board Specification

| Parameter | Value |
|---|---|
| Dictionary | DICT_5X5_250 |
| Grid | 5x7 squares |
| Square size | 24.29 mm (measure your print!) |
| Marker size | 15.00 mm (measure your print!) |

The board should be printed on Letter paper. **Measure the actual printed dimensions** — printers may scale. Update `CHARUCO_SQUARE_LENGTH` and `CHARUCO_MARKER_LENGTH` in the script if your measurements differ.

## Calibration Procedure

### 1. Launch ZED camera

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i enable_ipc:=false publish_tf:=false publish_urdf:=false
```

Key flags:
- `enable_ipc:=false` — required so external ROS nodes can subscribe to ZED topics
- `publish_tf:=false` — prevents ZED's own TF from conflicting with calibration
- `publish_urdf:=false` — prevents two-parent TF conflict on `zed_left_camera_optical_frame`

### 2. Launch robot driver

The UR driver must be running and publishing TF. Verify with:
```bash
ros2 run tf2_ros tf2_echo base_link tool0
```

### 3. Run the calibration script

```bash
cd ~/work/github_ws/experimental
source install/setup.bash
python3 src/vision/zed_calibration/zed_hand_eye_calibration.py
```

A live preview window shows the camera feed with detected markers highlighted.

### 4. Collect samples

Move the robot to diverse poses using the teach pendant. For each pose:
1. Ensure the ChArUco board is visible in the preview window (green markers, red corners)
2. Press **ENTER** in the terminal to capture the sample
3. Verify both TF and ChArUco detection succeeded

**Tips for good calibration:**
- Collect at least 10-15 samples
- Include both translational AND rotational diversity
- Rotate the wrist (wrist_1, wrist_2, wrist_3 joints), don't just translate
- Cover different areas of the camera's field of view
- Tilt the board at various angles (not always parallel to the camera)
- Avoid poses where few corners are detected (<6)

### 5. Solve

Press **s** in the terminal. The script runs all 5 OpenCV solvers:
- Tsai, Park, Horaud, Andreff, Daniilidis

**What to look for:**
- At least 4 out of 5 methods should agree within ~5mm on translation
- Andreff may diverge slightly — this is normal with fewer samples
- The Park result is saved by default (generally most reliable)

The result is saved to `zed_calibration_result.json` with the `static_transform_publisher` command.

### 6. Verify with point cloud

```bash
# Terminal 1: ZED (keep running from step 1, or relaunch)
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i enable_ipc:=false publish_tf:=false publish_urdf:=false

# Terminal 2: Publish calibration TF (copy command from script output)
ros2 run tf2_ros static_transform_publisher --x ... --frame-id base_link --child-frame-id zed_left_camera_optical_frame

# Terminal 3: Bridge optical -> camera frame
ros2 run tf2_ros static_transform_publisher --qx 0.5 --qy -0.5 --qz 0.5 --qw 0.5 --frame-id zed_left_camera_optical_frame --child-frame-id zed_left_camera_frame
```

In RViz:
- Set **Fixed Frame** = `base_link`
- Add the ZED point cloud topic
- The point cloud of the workspace should align with the robot model

### 7. Update the launch file

Once verified, update `src/vision/zed_calibration/zed_camera_pose.launch.py` with the new calibration values.

## Running the Calibrated ZED

After calibration, to use the ZED with the robot:

```bash
# Terminal 1: ZED camera (no internal TF)
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i enable_ipc:=false publish_tf:=false publish_urdf:=false

# Terminal 2: Calibration TF (launches both transforms)
ros2 launch zed_camera_pose.launch.py
```

Or include `zed_camera_pose.launch.py` in your main launch file.

## Frame Reference

```
base_link
  └─ zed_left_camera_optical_frame   (from calibration: base_link -> optical)
       └─ zed_left_camera_frame      (from bridge TF: standard optical rotation inverse)
```

- `zed_left_camera_optical_frame`: OpenCV convention (X-right, Y-down, Z-forward). Used by the calibration solver and vision processing.
- `zed_left_camera_frame`: ROS camera convention. Used by ZED point cloud messages (`frame_id` in header).
- The bridge TF between them is a pure rotation: quaternion (0.5, -0.5, 0.5, 0.5).

## Troubleshooting

| Problem | Solution |
|---|---|
| No ZED image received | Check `enable_ipc:=false`, verify topics with `ros2 topic list` |
| 0 points in RViz point cloud | Missing bridge TF — need both `base_link→optical` and `optical→camera_frame` |
| "frame ... discarding message" | TF tree is broken — check for two-parent conflicts with `ros2 run tf2_ros tf2_monitor` |
| Few corners detected | Move board closer, improve lighting, ensure board is flat |
| Solvers disagree wildly | Not enough rotational diversity — rotate the wrist more between samples |
| Point cloud offset in one direction | Re-calibrate with more samples and better pose diversity |

## Technical Notes

- The script uses OpenCV 4.11+ API (`CharucoDetector.detectBoard()`, `solvePnP` with `matchImagePoints`)
- Eye-to-hand formulation: robot poses are inverted (base→ee becomes ee→base) before passing to `cv2.calibrateHandEye()`
- Camera intrinsics are read from the ZED camera_info topic (not hardcoded)
- The calibration solves for `base_link → zed_left_camera_optical_frame` because the ChArUco detection operates in the optical frame
