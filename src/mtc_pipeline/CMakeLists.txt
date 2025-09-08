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

# Generate actions and Python modules
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MTCExecution.action"
  "action/MoveToAction.action"
  DEPENDENCIES std_srvs
  ADD_LINTER_TESTS
)

# Install Python modules if they exist
if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME})
  install(DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}
    DESTINATION lib/python3*/site-packages/
  )
endif()

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
# moveto_action_server removed - now embedded in orchestrator

# === Link executables to dependencies ===
ament_target_dependencies(mtc_orchestrator moveit_task_constructor_core rclcpp ur_msgs)
ament_target_dependencies(mtc_orchestrator_action_server moveit_task_constructor_core rclcpp rclcpp_action ur_msgs std_srvs)
ament_target_dependencies(mtc_action_client_example rclcpp rclcpp_action)
# moveto_action_server dependencies removed

# Make executables depend on generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(mtc_orchestrator_action_server "${cpp_typesupport_target}")
target_link_libraries(mtc_action_client_example "${cpp_typesupport_target}")
# moveto_action_server typesupport removed

target_link_libraries(mtc_orchestrator nlohmann_json::nlohmann_json)
target_link_libraries(mtc_orchestrator_action_server nlohmann_json::nlohmann_json)
target_link_libraries(mtc_action_client_example nlohmann_json::nlohmann_json)
# moveto_action_server nlohmann_json removed

# === Link mtc_pipeline_core library to relevant executables ===
target_link_libraries(mtc_orchestrator mtc_pipeline_core)
target_link_libraries(mtc_orchestrator_action_server mtc_pipeline_core)
# moveto_action_server core library removed

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
# moveto_action_server include directories removed

# === C++ Standards ===
target_compile_features(mtc_orchestrator PUBLIC c_std_99 cxx_std_17)
target_compile_features(mtc_orchestrator_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(mtc_action_client_example PUBLIC c_std_99 cxx_std_17)
# moveto_action_server compile features removed

# === Install targets ===
# Install executables to lib/${PROJECT_NAME} (ROS2 standard)
install(TARGETS
  mtc_orchestrator
  mtc_orchestrator_action_server
  mtc_action_client_example
  # moveto_action_server removed - now embedded
  DESTINATION lib/${PROJECT_NAME}
)

# Install library to lib/ (for runtime linking)
install(TARGETS
  mtc_pipeline_core
  DESTINATION lib
)

# === Install launch files ===
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# === Install Python scripts ===
# Note: GUI components have been moved to the mtc_gui package

# === Install test script ===
# Note: GUI test script has been moved to the mtc_gui package

# === Testing (optional) ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
