cmake_minimum_required(VERSION 3.22)
project(cms_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(cms_servo_node src/servo_node_jazzy.cpp)
target_link_libraries(cms_servo_node
  moveit_servo::moveit_servo_lib_cpp
  rclcpp::rclcpp)

install(TARGETS cms_servo_node DESTINATION lib/${PROJECT_NAME})

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY srdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
