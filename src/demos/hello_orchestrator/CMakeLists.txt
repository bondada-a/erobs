cmake_minimum_required(VERSION 3.8)
project(hello_orchestrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(nlohmann_json REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PrintMessage.action"
  "action/MoveToNamedState.action"
  "action/OrchestratorTask.action"
  ADD_LINTER_TESTS
)

# Get typesupport target for linking against generated actions
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Print server
add_executable(print_server src/print_server.cpp)
ament_target_dependencies(print_server
  rclcpp
  rclcpp_action
)
target_link_libraries(print_server "${cpp_typesupport_target}")

# Move server
add_executable(move_server src/move_server.cpp)
ament_target_dependencies(move_server
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
)
target_link_libraries(move_server "${cpp_typesupport_target}")

# Orchestrator server
add_executable(orchestrator_server src/orchestrator_server.cpp)
ament_target_dependencies(orchestrator_server
  rclcpp
  rclcpp_action
)
target_link_libraries(orchestrator_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json)

install(TARGETS
  print_server
  move_server
  orchestrator_server
  DESTINATION lib/${PROJECT_NAME}
)

# Python client
install(PROGRAMS
  src/task_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# Config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
