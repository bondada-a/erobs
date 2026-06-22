cmake_minimum_required(VERSION 3.22)
project(beambot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(beambot)

# Install robot_operation.md next to the beambot.agent module.
# RobotAgent loads it via _PROMPT_PATH (os.path.dirname(__file__) + "robot_operation.md"),
# and the file also backs the .claude/skills/robot-operation skill (which cats it from
# src/). Both paths need it, so ship the install copy alongside the Python files.
install(FILES beambot/agent/robot_operation.md
  DESTINATION ${PYTHON_INSTALL_DIR}/beambot/agent
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python scripts as executables
install(PROGRAMS
  # Action servers (consolidated - main() is in each server file)
  beambot/action_servers/move_to_server.py
  beambot/action_servers/end_effector_server.py
  beambot/action_servers/sample_server.py
  beambot/action_servers/tool_exchange_server.py
  beambot/action_servers/vision_server.py
  beambot/action_servers/vision_task_server.py
  beambot/action_servers/pipettor_server.py
  beambot/action_servers/orchestrator.py
  # Utility nodes
  beambot/pointcloud_relay.py
  beambot/octomap_to_planning_scene.py
  # Client utilities
  scripts/beambot_client.py
  # Perf / tracing utilities (ros2_tracing wrapper + report)
  scripts/perf_trace_run.sh
  scripts/perf_trace_summarize.py
  # Test/debug scripts
  scripts/test_wafer_detection.py
  scripts/test_contour_detection.py
  scripts/test_sample_detection.py
  # Rosbag helper: periodic Zivid capture trigger
  scripts/trigger_zivid_capture.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
