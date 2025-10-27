cmake_minimum_required(VERSION 3.8)
project(mtc_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pipette_driver REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


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
  src/base_stages.cpp
  src/pick_place_stages.cpp
  src/tool_exchange_stages.cpp
  src/move_to_stages.cpp
  src/end_effector_stages.cpp
  src/vision_stages.cpp
  src/pipettor_stages.cpp
  src/pipettor_operation_stage.cpp
  src/vision_pick_place_stages.cpp
)
ament_target_dependencies(mtc_pipeline_core
  moveit_task_constructor_core
  rclcpp
  rclcpp_action
  nlohmann_json
  tf2_eigen
  tf2_ros
  tf2_geometry_msgs
  apriltag_msgs
  geometry_msgs
  pipette_driver
  moveit_ros_planning_interface
)
target_include_directories(mtc_pipeline_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(mtc_pipeline_core PUBLIC c_std_99 cxx_std_17)


include_directories(include)


add_executable(mtc_orchestrator_action_server src/mtc_orchestrator_action_server.cpp)
add_executable(mtc_action_client_example src/mtc_action_client_example.cpp)
add_executable(pick_place_action_server src/pick_place_action_server.cpp)
add_executable(tool_exchange_action_server src/tool_exchange_action_server.cpp)
add_executable(move_to_action_server src/move_to_action_server.cpp)
add_executable(end_effector_action_server src/end_effector_action_server.cpp)
add_executable(vision_action_server src/vision_action_server.cpp)
add_executable(pipettor_action_server src/pipettor_action_server.cpp)
add_executable(vision_pick_place_action_server src/vision_pick_place_action_server.cpp)


rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")


ament_target_dependencies(mtc_orchestrator_action_server ${ORCHESTRATOR_DEPS})
ament_target_dependencies(mtc_action_client_example rclcpp rclcpp_action)
ament_target_dependencies(pick_place_action_server ${MODULAR_ACTION_SERVER_DEPS})
ament_target_dependencies(tool_exchange_action_server ${MODULAR_ACTION_SERVER_DEPS})
ament_target_dependencies(move_to_action_server ${MODULAR_ACTION_SERVER_DEPS})
ament_target_dependencies(end_effector_action_server ${MODULAR_ACTION_SERVER_DEPS})
ament_target_dependencies(vision_action_server ${MODULAR_ACTION_SERVER_DEPS})
ament_target_dependencies(pipettor_action_server ${MODULAR_ACTION_SERVER_DEPS} pipette_driver)
ament_target_dependencies(vision_pick_place_action_server ${MODULAR_ACTION_SERVER_DEPS})

# Link Libraries
target_link_libraries(mtc_orchestrator_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)
target_link_libraries(mtc_action_client_example "${cpp_typesupport_target}" nlohmann_json::nlohmann_json)
target_link_libraries(pick_place_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)
target_link_libraries(tool_exchange_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)
target_link_libraries(move_to_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)
target_link_libraries(end_effector_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)
target_link_libraries(vision_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)
target_link_libraries(pipettor_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)
target_link_libraries(vision_pick_place_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)

target_compile_features(mtc_orchestrator_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(mtc_action_client_example PUBLIC c_std_99 cxx_std_17)
target_compile_features(pick_place_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(tool_exchange_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(move_to_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(end_effector_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(vision_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(pipettor_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(vision_pick_place_action_server PUBLIC c_std_99 cxx_std_17)

install(TARGETS
  mtc_orchestrator_action_server
  mtc_action_client_example
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
  DESTINATION lib
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/vision_pick_predefined_place.py
  scripts/test_vision.py
  DESTINATION lib/${PROJECT_NAME}
)


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
