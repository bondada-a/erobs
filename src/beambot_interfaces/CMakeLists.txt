cmake_minimum_required(VERSION 3.8)
project(beambot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MTCExecution.action"
  "action/MoveToAction.action"
  "action/EndEffectorAction.action"
  "action/PickPlaceAction.action"
  "action/ToolExchangeAction.action"
  "action/VisionMoveToAction.action"
  "action/VisionPickPlaceAction.action"
  "action/VisionScanAction.action"
  "action/PipettorAction.action"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
