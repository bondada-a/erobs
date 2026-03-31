cmake_minimum_required(VERSION 3.8)
project(onrobot_2fg7_driver)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
install(
  DIRECTORY onrobot_2fg7_driver/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}
  PATTERN "*.py"
)

# Install Python executables
install(PROGRAMS
  onrobot_2fg7_driver/driver_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME onrobot_2fg7_driver_node
)

# Install config and launch
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
