cmake_minimum_required(VERSION 3.8)
project(beambot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(beambot)

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
  beambot/action_servers/pipettor_server.py
  beambot/action_servers/orchestrator.py
  # Utility nodes
  beambot/pointcloud_relay.py
  beambot/octomap_to_planning_scene.py
  # Client utilities
  scripts/beambot_client.py
  # Test/debug scripts
  scripts/test_wafer_detection.py
  scripts/test_contour_detection.py
  scripts/test_sample_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
