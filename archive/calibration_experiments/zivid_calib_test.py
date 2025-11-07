detection_result = zivid.calibration.detect_calibration_board(frame)

if detection_result.valid():
    print("Calibration board detected")
    hand_eye_input.append(zivid.calibration.HandEyeInput(robot_pose, detection_result))
    current_pose_id += 1
else:
    print(f"Failed to detect calibration board. {detection_result.status_description()}")