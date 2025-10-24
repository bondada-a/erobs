cmake_minimum_required(VERSION 3.8)
project(mtc_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(mtc_pipeline REQUIRED)

# Install Python scripts
install(PROGRAMS
  src/mtc_gui_client.py
  src/pose_editor.py
  src/poses_manager.py
  src/test_gui_components.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install resource files
install(DIRECTORY resource DESTINATION share/${PROJECT_NAME})

ament_package()

