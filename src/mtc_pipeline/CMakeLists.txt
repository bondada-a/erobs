cmake_minimum_required(VERSION 3.8)
project(mtc_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)


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
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MTCExecution.action"
  "action/MoveToAction.action"
  "action/EndEffectorAction.action"
  "action/ToolExchangeAction.action"
  "action/PickPlaceAction.action"
  DEPENDENCIES std_srvs
  ADD_LINTER_TESTS
)


add_library(mtc_pipeline_core
  src/base_stages.cpp
  src/pick_place_stages.cpp
  src/tool_exchange_stages.cpp
  src/moveto_stages.cpp
  src/end_effector_stages.cpp
)
ament_target_dependencies(mtc_pipeline_core
  moveit_task_constructor_core
  rclcpp
  nlohmann_json
  tf2_eigen
)
target_include_directories(mtc_pipeline_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(mtc_pipeline_core PUBLIC c_std_99 cxx_std_17)


include_directories(include)


add_executable(mtc_orchestrator_action_server src/mtc_orchestrator_action_server.cpp)
add_executable(mtc_action_client_example src/mtc_action_client_example.cpp)
add_executable(pickplace_action_server src/pickplace_action_server.cpp)
add_executable(toolexchange_action_server src/toolexchange_action_server.cpp)
add_executable(moveto_action_server src/moveto_action_server.cpp)
add_executable(end_effector_action_server src/end_effector_action_server.cpp)


rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")


ament_target_dependencies(mtc_orchestrator_action_server ${ORCHESTRATOR_DEPS})
ament_target_dependencies(mtc_action_client_example rclcpp rclcpp_action)
ament_target_dependencies(pickplace_action_server ${MODULAR_ACTION_SERVER_DEPS})
ament_target_dependencies(toolexchange_action_server ${MODULAR_ACTION_SERVER_DEPS})
ament_target_dependencies(moveto_action_server ${MODULAR_ACTION_SERVER_DEPS})
ament_target_dependencies(end_effector_action_server ${MODULAR_ACTION_SERVER_DEPS})

# Link Libraries 
target_link_libraries(mtc_orchestrator_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)
target_link_libraries(mtc_action_client_example "${cpp_typesupport_target}" nlohmann_json::nlohmann_json)
target_link_libraries(pickplace_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)
target_link_libraries(toolexchange_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)
target_link_libraries(moveto_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)
target_link_libraries(end_effector_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json mtc_pipeline_core)

target_compile_features(mtc_orchestrator_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(mtc_action_client_example PUBLIC c_std_99 cxx_std_17)
target_compile_features(pickplace_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(toolexchange_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(moveto_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(end_effector_action_server PUBLIC c_std_99 cxx_std_17)

install(TARGETS
  mtc_orchestrator_action_server
  mtc_action_client_example
  pickplace_action_server
  toolexchange_action_server
  moveto_action_server
  end_effector_action_server
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  mtc_pipeline_core
  DESTINATION lib
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
