cmake_minimum_required(VERSION 3.8)
project(cms_beamtime)

# Use Ament CMake with Python support
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cms_beamtime_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs)


# Install the Python package directory
ament_python_install_package(${PROJECT_NAME})

set(dependencies
  moveit_ros_planning_interface
  cms_beamtime_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  yaml-cpp
  geometry_msgs
)

include_directories(include)  # This makes sure the libraries are visible
add_executable(cms_beamtime_server src/beamtime_entrypoint.cpp src/cms_beamtime_server.cpp src/inner_state_machine.cpp)
ament_target_dependencies(cms_beamtime_server ${dependencies})

install(TARGETS
  cms_beamtime_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)


# If you prefer not to use setup.py, install your node scripts directly:
# Place executable Python scripts (with shebang) in scripts/
install(PROGRAMS
  cms_beamtime/cms_beamtime_server.py
  src/cms_beamtime_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})


# All ROS2 and Python dependencies (e.g. rclpy, tf2_ros, geometry_msgs, moveit_msgs, control_msgs,
# moveit_msgs, cms_beamtime_interfaces) should be declared in package.xml as <exec_depend> tags.
# No need to find_package here since ament_python_install_package and ament_cmake_python
# handle the Python package layout and installation.

ament_package()
