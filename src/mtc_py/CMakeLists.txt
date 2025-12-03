cmake_minimum_required(VERSION 3.8)
project(mtc_py)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generate action interfaces (copied from mtc_pipeline, without PipettorAction)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MTCExecution.action"
  "action/MoveToAction.action"
  "action/EndEffectorAction.action"
  "action/PickPlaceAction.action"
  "action/ToolExchangeAction.action"
  "action/VisionMoveToAction.action"
  "action/VisionPickPlaceAction.action"
  DEPENDENCIES geometry_msgs std_msgs
)

# Install Python package (named mtc_py_lib to avoid conflict with rosidl target)
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
  scripts/move_to_server_node.py
  scripts/end_effector_server_node.py
  scripts/pick_place_server_node.py
  scripts/tool_exchange_server_node.py
  scripts/vision_server_node.py
  scripts/vision_pick_place_server_node.py
  scripts/orchestrator_node.py
  scripts/send_task_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
