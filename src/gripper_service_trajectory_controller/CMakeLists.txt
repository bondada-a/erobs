cmake_minimum_required(VERSION 3.8)
project(gripper_service_trajectory_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(cms_beamtime_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/gripper_service_trajectory_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  realtime_tools
  rclcpp_action
  rclcpp_components
  controller_interface
  hardware_interface
  pluginlib
  trajectory_msgs
  cms_beamtime_interfaces
  sensor_msgs
)

pluginlib_export_plugin_description_file(controller_interface gripper_service_trajectory_controller.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
