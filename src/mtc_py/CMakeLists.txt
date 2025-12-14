cmake_minimum_required(VERSION 3.8)
project(mtc_py)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
# Note: Named mtc_py_lib for now; will be renamed in future cleanup (#9)
ament_python_install_package(mtc_py_lib)

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
  mtc_py_lib/action_servers/move_to_server.py
  mtc_py_lib/action_servers/end_effector_server.py
  mtc_py_lib/action_servers/pick_place_server.py
  mtc_py_lib/action_servers/tool_exchange_server.py
  mtc_py_lib/action_servers/vision_server.py
  mtc_py_lib/action_servers/vision_pick_place_server.py
  mtc_py_lib/action_servers/pipettor_server.py
  mtc_py_lib/action_servers/orchestrator.py
  # Client utilities
  scripts/mtc_py_client.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
