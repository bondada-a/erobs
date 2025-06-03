cmake_minimum_required(VERSION 3.8)
project(cms_beamtime)

# Use Ament CMake with Python support
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the Python package directory
ament_python_install_package(${PROJECT_NAME})


# If you prefer not to use setup.py, install your node scripts directly:
# Place executable Python scripts (with shebang) in scripts/
install(PROGRAMS
  cms_beamtime/cms_beamtime_server.py
  src/cms_beamtime_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})


# All ROS2 and Python dependencies (e.g. rclpy, tf2_ros, geometry_msgs, moveit_msgs, control_msgs,
# moveit_msgs, pdf_beamtime_interfaces) should be declared in package.xml as <exec_depend> tags.
# No need to find_package here since ament_python_install_package and ament_cmake_python
# handle the Python package layout and installation.

ament_package()
