cmake_minimum_required(VERSION 3.22)
project(beambot)

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

install(FILES models/spincoater_sample_seg.pt
  DESTINATION share/${PROJECT_NAME}/models
)

# Install Python scripts as executables
install(PROGRAMS
  # Action servers (consolidated - main() is in each server file)
  beambot/action_servers/move_to_server.py
  beambot/action_servers/end_effector_server.py
  beambot/action_servers/sample_pick_place_server.py
  beambot/action_servers/tool_exchange_server.py
  beambot/action_servers/vision_scan_server.py
  beambot/action_servers/vision_task_server.py
  beambot/action_servers/pipettor_server.py
  beambot/action_servers/orchestrator.py
  # Utility nodes
  beambot/mapping/pointcloud_relay.py
  beambot/mapping/octomap_to_planning_scene.py
  # Client utilities
  scripts/beambot_client.py
  # Perf / tracing utilities (ros2_tracing wrapper + report)
  scripts/perf_trace_run.sh
  scripts/perf_trace_summarize.py
  # Octomap capture run: move through scan poses + capture to build a map
  scripts/capture_octomap.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_pytest_test(beambot_unit test
    TIMEOUT 180
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../.."
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1" "PYTHONDONTWRITEBYTECODE=1"
  )

  foreach(suite orchestrator model_reload process_cleanup)
    add_launch_test(test/integration/test_${suite}_launch.py
      TARGET beambot_${suite}_integration
      TIMEOUT 120
      RUNNER "${CMAKE_CURRENT_SOURCE_DIR}/test/run_isolated.py"
      ENV "PYTHONDONTWRITEBYTECODE=1" "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_ros_logs"
      APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
    )
  endforeach()
endif()

ament_package()
