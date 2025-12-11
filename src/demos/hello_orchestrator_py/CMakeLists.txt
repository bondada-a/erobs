cmake_minimum_required(VERSION 3.8)
project(hello_orchestrator_py)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PrintMessage.action"
  "action/MoveToNamedState.action"
  "action/OrchestratorTask.action"
  ADD_LINTER_TESTS
)

# Install Python package
ament_python_install_package(hello_orchestrator_py)

# Install Python executables
install(PROGRAMS
  scripts/print_server.py
  scripts/move_server.py
  scripts/orchestrator_server.py
  scripts/task_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
