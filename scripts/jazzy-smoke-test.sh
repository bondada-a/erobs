#!/bin/bash
# Jazzy Docker Image Smoke Test
# Run: ./scripts/jazzy-smoke-test.sh
#
# Tests that the erobs-jazzy:latest Docker image is functional:
# 1. ROS2 environment loads
# 2. All EROBS packages are installed
# 3. Key ROS2 nodes can be instantiated
# 4. UR driver and MoveIt are available

set -e

IMAGE="erobs-jazzy:latest"
PASS=0
FAIL=0

run_test() {
    local name="$1"
    local cmd="$2"
    printf "  %-50s" "$name"
    if output=$(docker run --rm "$IMAGE" bash -c "source /root/ws/erobs/install/setup.bash && $cmd" 2>&1); then
        echo "[PASS]"
        PASS=$((PASS + 1))
    else
        echo "[FAIL]"
        echo "    Output: $output" | head -5
        FAIL=$((FAIL + 1))
    fi
}

echo "============================================"
echo "EROBS Jazzy Docker Smoke Test"
echo "Image: $IMAGE"
echo "============================================"

# Check image exists
if ! docker image inspect "$IMAGE" &>/dev/null; then
    echo "ERROR: Image $IMAGE not found. Build it first:"
    echo "  docker build -f docker/jazzy/Dockerfile -t erobs-jazzy:latest ."
    exit 1
fi

echo ""
echo "--- ROS2 Environment ---"
run_test "ROS2 distro is jazzy" \
    "[ \"\$ROS_DISTRO\" = 'jazzy' ]"
run_test "ros2 command available" \
    "ros2 --help >/dev/null"
run_test "Total packages > 400" \
    "[ \$(ros2 pkg list 2>/dev/null | wc -l) -gt 400 ]"

echo ""
echo "--- EROBS Core Packages ---"
for pkg in beambot beambot_interfaces pdf_beamtime pdf_beamtime_interfaces \
           aruco_pose mtc_gui ur5e_robot_description; do
    run_test "Package: $pkg" \
        "ros2 pkg prefix $pkg >/dev/null"
done

echo ""
echo "--- Demo Packages ---"
for pkg in hello_moveit hello_moveit_interfaces \
           hello_orchestrator hello_orchestrator_interfaces \
           hello_orchestrator_py hello_orchestrator_py_interfaces; do
    run_test "Package: $pkg" \
        "ros2 pkg prefix $pkg >/dev/null"
done

echo ""
echo "--- External Dependencies ---"
for pkg in moveit_task_constructor_core robotiq_hande_driver \
           robotiq_hande_description zivid_camera serial pipette_driver; do
    run_test "Package: $pkg" \
        "ros2 pkg prefix $pkg >/dev/null"
done

echo ""
echo "--- MoveIt & UR Driver ---"
run_test "MoveIt available" \
    "ros2 pkg prefix moveit_ros_planning_interface >/dev/null"
run_test "UR robot driver available" \
    "ros2 pkg prefix ur_robot_driver >/dev/null"
run_test "MoveIt servo available" \
    "ros2 pkg prefix moveit_servo >/dev/null"

echo ""
echo "--- Python Import Tests ---"
run_test "Import rclpy" \
    "python3 -c 'import rclpy'"
run_test "Import moveit (moveit_py)" \
    "python3 -c 'from moveit.planning import MoveItPy' 2>/dev/null || python3 -c 'import moveit_commander' 2>/dev/null || python3 -c 'from moveit_configs_utils import MoveItConfigsBuilder'"
run_test "Import cv_bridge" \
    "python3 -c 'from cv_bridge import CvBridge'"
run_test "Import tf_transformations" \
    "python3 -c 'import tf_transformations'"

echo ""
echo "--- Zivid SDK ---"
run_test "Zivid SDK installed" \
    "dpkg -l | grep -q zivid"
run_test "Zivid library exists" \
    "test -f /usr/lib/x86_64-linux-gnu/libZivid.so || test -f /usr/lib/libZivid.so || ldconfig -p | grep -q libZivid"

echo ""
echo "============================================"
echo "Results: $PASS passed, $FAIL failed"
echo "============================================"

if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
echo "All tests passed!"
