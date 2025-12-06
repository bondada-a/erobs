cmake_minimum_required(VERSION 3.8)
project(mtc_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(zivid_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pipette_driver REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)


set(MODULAR_ACTION_SERVER_DEPS
  moveit_task_constructor_core
  rclcpp
  rclcpp_action
)

set(ORCHESTRATOR_DEPS
  moveit_task_constructor_core
  rclcpp
  rclcpp_action
  std_srvs
  pipette_driver
  ur_msgs
  ament_index_cpp
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MTCExecution.action"
  "action/MoveToAction.action"
  "action/EndEffectorAction.action"
  "action/ToolExchangeAction.action"
  "action/PickPlaceAction.action"
  "action/VisionMoveToAction.action"
  "action/PipettorAction.action"
  "action/VisionPickPlaceAction.action"
  DEPENDENCIES std_srvs geometry_msgs
  ADD_LINTER_TESTS
)


add_library(mtc_pipeline_core
  src/stages/base_stages.cpp
  src/stages/pick_place_stages.cpp
  src/stages/tool_exchange_stages.cpp
  src/stages/move_to_stages.cpp
  src/stages/end_effector_stages.cpp
  src/stages/vision_stages.cpp
  src/stages/pipettor_stages.cpp
  src/stages/pipettor_operation_stage.cpp
  src/stages/vision_pick_place_stages.cpp
  src/utils/gripper_config_registry.cpp
  src/utils/beamline_config.cpp
)
ament_target_dependencies(mtc_pipeline_core
  moveit_task_constructor_core
  rclcpp
  rclcpp_action
  nlohmann_json
  tf2_eigen
  tf2_ros
  tf2_geometry_msgs
  zivid_interfaces
  geometry_msgs
  pipette_driver
  moveit_ros_planning_interface
  ament_index_cpp
)
target_include_directories(mtc_pipeline_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mtc_pipeline_core yaml-cpp)
target_compile_features(mtc_pipeline_core PUBLIC c_std_99 cxx_std_17)

# Make sure the library depends on the generated interfaces
rosidl_target_interfaces(mtc_pipeline_core ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Obstacle loader library
add_library(obstacle_loader src/utils/obstacle_loader.cpp)
target_include_directories(obstacle_loader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(obstacle_loader
  rclcpp
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
  geometry_msgs
  shape_msgs
  moveit_msgs
)
target_link_libraries(obstacle_loader yaml-cpp)
target_compile_features(obstacle_loader PUBLIC c_std_99 cxx_std_17)

# Core orchestrator components library
add_library(orchestrator_core
  src/core/ur_tool_interface.cpp
  src/core/moveit_lifecycle_manager.cpp
)
target_include_directories(orchestrator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(orchestrator_core
  rclcpp
  std_srvs
  moveit_msgs
  ament_index_cpp
)
target_link_libraries(orchestrator_core mtc_pipeline_core obstacle_loader)
target_compile_features(orchestrator_core PUBLIC c_std_99 cxx_std_17)
rosidl_target_interfaces(orchestrator_core ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(include)


add_executable(mtc_orchestrator_action_server src/action_servers/mtc_orchestrator_action_server.cpp)
add_executable(mtc_client src/utils/mtc_client.cpp)
add_executable(pick_place_action_server src/action_servers/pick_place_action_server.cpp)
add_executable(tool_exchange_action_server src/action_servers/tool_exchange_action_server.cpp)
add_executable(move_to_action_server src/action_servers/move_to_action_server.cpp)
add_executable(end_effector_action_server src/action_servers/end_effector_action_server.cpp)
add_executable(vision_action_server src/action_servers/vision_action_server.cpp)
add_executable(pipettor_action_server src/action_servers/pipettor_action_server.cpp)
add_executable(vision_pick_place_action_server src/action_servers/vision_pick_place_action_server.cpp)


rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")


ament_target_dependencies(mtc_orchestrator_action_server ${ORCHESTRATOR_DEPS})
ament_target_dependencies(mtc_client rclcpp rclcpp_action)
ament_target_dependencies(pick_place_action_server ${MODULAR_ACTION_SERVER_DEPS})
ament_target_dependencies(tool_exchange_action_server ${MODULAR_ACTION_SERVER_DEPS})
ament_target_dependencies(move_to_action_server ${MODULAR_ACTION_SERVER_DEPS})
ament_target_dependencies(end_effector_action_server ${MODULAR_ACTION_SERVER_DEPS})
ament_target_dependencies(vision_action_server ${MODULAR_ACTION_SERVER_DEPS})
ament_target_dependencies(pipettor_action_server ${MODULAR_ACTION_SERVER_DEPS} pipette_driver)
ament_target_dependencies(vision_pick_place_action_server ${MODULAR_ACTION_SERVER_DEPS})

# Link Libraries
target_link_libraries(mtc_orchestrator_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core obstacle_loader orchestrator_core)
target_link_libraries(mtc_client "${cpp_typesupport_target}" nlohmann_json::nlohmann_json)
target_link_libraries(pick_place_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)
target_link_libraries(tool_exchange_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)
target_link_libraries(move_to_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)
target_link_libraries(end_effector_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)
target_link_libraries(vision_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)
target_link_libraries(pipettor_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)
target_link_libraries(vision_pick_place_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)

target_compile_features(mtc_orchestrator_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(mtc_client PUBLIC c_std_99 cxx_std_17)
target_compile_features(pick_place_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(tool_exchange_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(move_to_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(end_effector_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(vision_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(pipettor_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(vision_pick_place_action_server PUBLIC c_std_99 cxx_std_17)

install(TARGETS
  mtc_orchestrator_action_server
  mtc_client
  pick_place_action_server
  tool_exchange_action_server
  move_to_action_server
  end_effector_action_server
  vision_action_server
  pipettor_action_server
  vision_pick_place_action_server
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  mtc_pipeline_core
  obstacle_loader
  orchestrator_core
  DESTINATION lib
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Disable all linters
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
