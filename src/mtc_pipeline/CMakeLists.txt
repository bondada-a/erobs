cmake_minimum_required(VERSION 3.8)
project(mtc_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Generate actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MTCExecution.action"
  DEPENDENCIES std_srvs
)

# === Create a library for shared logic ===
add_library(mtc_pipeline_core
  src/pick_place_stages.cpp
  src/tool_exchange_stages.cpp
  src/moveto_stages.cpp
  src/end_effector_stages.cpp
)
ament_target_dependencies(mtc_pipeline_core
  moveit_task_constructor_core
  rclcpp
  ur_msgs
  nlohmann_json
)
target_include_directories(mtc_pipeline_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(mtc_pipeline_core PUBLIC c_std_99 cxx_std_17)

# === Executables ===
add_executable(mtc_orchestrator  src/mtc_orchestrator.cpp)
add_executable(mtc_orchestrator_action_server  src/mtc_orchestrator_action_server.cpp)
add_executable(mtc_action_client_example  src/mtc_action_client_example.cpp)

# === Link executables to dependencies ===
ament_target_dependencies(mtc_orchestrator moveit_task_constructor_core rclcpp ur_msgs)
ament_target_dependencies(mtc_orchestrator_action_server moveit_task_constructor_core rclcpp rclcpp_action ur_msgs std_srvs)
ament_target_dependencies(mtc_action_client_example rclcpp rclcpp_action)

# Make executables depend on generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(mtc_orchestrator_action_server "${cpp_typesupport_target}")
target_link_libraries(mtc_action_client_example "${cpp_typesupport_target}")

target_link_libraries(mtc_orchestrator nlohmann_json::nlohmann_json)
target_link_libraries(mtc_orchestrator_action_server nlohmann_json::nlohmann_json)
target_link_libraries(mtc_action_client_example nlohmann_json::nlohmann_json)

# === Link mtc_pipeline_core library to relevant executables ===
target_link_libraries(mtc_orchestrator mtc_pipeline_core)
target_link_libraries(mtc_orchestrator_action_server mtc_pipeline_core)

# === Include directories ===
target_include_directories(mtc_orchestrator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(mtc_orchestrator_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(mtc_action_client_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# === C++ Standards ===
target_compile_features(mtc_orchestrator PUBLIC c_std_99 cxx_std_17)
target_compile_features(mtc_orchestrator_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(mtc_action_client_example PUBLIC c_std_99 cxx_std_17)

# === Install targets ===
# Install executables to lib/${PROJECT_NAME} (ROS2 standard)
install(TARGETS
  mtc_orchestrator
  mtc_orchestrator_action_server
  mtc_action_client_example
  DESTINATION lib/${PROJECT_NAME}
)

# Install library to lib/ (for runtime linking)
install(TARGETS
  mtc_pipeline_core
  DESTINATION lib
)

# === Install launch files ===
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# === Testing (optional) ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
